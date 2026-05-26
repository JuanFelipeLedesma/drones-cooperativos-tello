"""
═══════════════════════════════════════════════════════════════
PRUEBA 5.1 — Misión cooperativa completa (MASTER, Mac)
═══════════════════════════════════════════════════════════════
Drones: 2  |  Complejidad: Alta  |  Tiempo de vuelo: ~2 min

ROL: MASTER (líder). Ejecuta una misión secuencial de 7 fases que
integra todo lo desarrollado en OE2: despegue secuencial, formación
estática, trayectoria coordinada, hover cooperativo, aterrizaje
secuencial.

FASES (mission_state publicado al SLAVE):
    1=takeoff      → despegue + climb + init hover (25 s)
    3=formation    → formación estática con SLAVE (10 s)
    4=trajectory   → master mueve a waypoint (+50 cm X), slave sigue
    2=hover        → hover cooperativo en destino (15 s)
    8=slave_land   → master ordena al slave aterrizar
    6=landing      → master aterriza tras slave

USO:
    python test_5_1_master.py
"""
import sys
import os
import time
import math
import socket
import statistics
import threading

import cv2

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from djitellopy import Tello  # noqa: E402

from utils import ArUcoTracker, FlightLogger  # noqa: E402
from utils.pid import PIDController  # noqa: E402
import config  # noqa: E402

from test_3_2_protocol import CoopMessage, encode_binary

# ----------------------------------------------------------------
# Parámetros de la misión
# ----------------------------------------------------------------
INITIAL_CLIMB_CM   = 60
IMU_WAIT_S         = 4.0
INIT_HOVER_S       = 5.0
WAIT_SLAVE_S       = 25.0    # tiempo para que slave: detect+takeoff+IMU+climb+hover
                             # (slave necesita ~15-18 s, dejamos margen)
FORMATION_HOLD_S   = 10.0    # formación estática inicial (después del wait)
HOVER_AT_DEST_S    = 15.0    # hover cooperativo en destino
WAIT_FOR_SLAVE_LANDING_S = 6.0  # esperar a que slave aterrice
WAYPOINT_OFFSET_X  = 0.5     # destino: +50 cm en X desde target_pos

WP_TOL_M           = 0.10    # tolerancia llegada a waypoint
WP_SETTLE_S        = 0.5
WP_TIMEOUT_S       = 12.0
LOOP_DT            = 0.05
PUBLISH_HZ         = 50

DRONE_ID = 1
SLAVE_ADDR = (config.SLAVE_IP, config.COMMS_PORT)

# Mission states
MS_TAKEOFF      = 1
MS_HOVER        = 2
MS_FORMATION    = 3
MS_TRAJECTORY   = 4
MS_LANDING      = 6
MS_SLAVE_LAND   = 8   # nuevo: master ordena al slave aterrizar


def safe_frame(tello):
    fr = tello.get_frame_read()
    if fr is None: return None
    f = fr.frame
    if f is None or f.size == 0: return None
    return f


def telemetry_snapshot(tello):
    try:
        return {
            "tello_height_cm": tello.get_height(), "tello_baro_cm": tello.get_barometer(),
            "tello_battery": tello.get_battery(), "tello_temp_c": tello.get_temperature(),
            "tello_pitch": tello.get_pitch(), "tello_roll": tello.get_roll(),
            "tello_yaw": tello.get_yaw(),
            "tello_vgx": tello.get_speed_x(), "tello_vgy": tello.get_speed_y(),
            "tello_vgz": tello.get_speed_z(),
        }
    except Exception:
        return {k: None for k in (
            "tello_height_cm","tello_baro_cm","tello_battery","tello_temp_c",
            "tello_pitch","tello_roll","tello_yaw","tello_vgx","tello_vgy","tello_vgz")}


# ============================================================
# Hilo publicador (igual que 2.2/2.3)
# ============================================================
class PosePublisher:
    def __init__(self, target_addr, hz):
        self._addr = target_addr
        self._interval = 1.0 / hz
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._lock = threading.Lock()
        self._latest = None
        self._stop = threading.Event()
        self._thread = None
        self._seq = 0
        self.sent_count = 0

    def update(self, pos, mission_state, battery, vel=None):
        with self._lock:
            self._latest = (pos, mission_state, battery, vel or (0.0, 0.0, 0.0))

    def _run(self):
        next_t = time.time()
        while not self._stop.is_set():
            now = time.time()
            if now >= next_t:
                with self._lock:
                    snap = self._latest
                if snap is not None:
                    pos, ms, bat, vel = snap
                    msg = CoopMessage(
                        drone_id=DRONE_ID, seq=self._seq, timestamp=time.time(),
                        pos_x=pos["x"], pos_y=pos["y"], pos_z=pos["z"],
                        vel_x=vel[0], vel_y=vel[1], vel_z=vel[2],
                        battery=int(bat) if bat else 0, mission_state=ms,
                    )
                    try:
                        self._sock.sendto(encode_binary(msg), self._addr)
                        self.sent_count += 1
                        self._seq = (self._seq + 1) & 0xFFFFFFFF
                    except Exception: pass
                next_t += self._interval
            else:
                time.sleep(min(0.005, next_t - now))

    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def send_state_burst(self, pos, mission_state, battery, vel=None, n=10):
        """Envía N mensajes seguidos con un mission_state específico (redundancia)."""
        with self._lock:
            self._latest = (pos, mission_state, battery, vel or (0.0, 0.0, 0.0))
        # Inundar el canal por un breve momento para garantizar entrega
        for _ in range(n):
            with self._lock:
                snap = self._latest
            if snap is None: continue
            p, ms, bat, v = snap
            msg = CoopMessage(
                drone_id=DRONE_ID, seq=self._seq, timestamp=time.time(),
                pos_x=p["x"], pos_y=p["y"], pos_z=p["z"],
                vel_x=v[0], vel_y=v[1], vel_z=v[2],
                battery=int(bat) if bat else 0, mission_state=ms,
            )
            try:
                self._sock.sendto(encode_binary(msg), self._addr)
                self.sent_count += 1
                self._seq = (self._seq + 1) & 0xFFFFFFFF
            except Exception: pass
            time.sleep(0.02)

    def stop(self):
        self._stop.set()
        if self._thread: self._thread.join(timeout=1.0)
        self._sock.close()


# ============================================================
# Helper: hover en un target (X, Y, Z) con PID hasta una condición
# ============================================================
def hover_at_target(tello, tracker, target, pids, publisher,
                    duration_s, mission_state, logger, t_start, phase_name,
                    last_pose_t_ref):
    """Mantiene PID en target durante duration_s. Loguea cada iteración.
    Devuelve (errors_3d_list, last_pos)."""
    pid_lr, pid_ud, pid_fb = pids
    errors = []
    last_pos = None
    t_phase_start = time.time()
    while time.time() - t_phase_start < duration_s:
        t_now = time.time()
        f = safe_frame(tello)
        pos, ann = (None, None)
        if f is not None: pos, ann = tracker.detect_and_estimate(f)

        if pos and (t_now - last_pose_t_ref[0]) < 1.5:
            last_pose_t_ref[0] = t_now
            ex = target["x"] - pos["x"]
            ey = target["y"] - pos["y"]
            ez = target["z"] - pos["z"]
            e3d = math.sqrt(ex*ex + ey*ey + ez*ez)
            cmd_lr = pid_lr.compute(ex)
            cmd_ud = pid_ud.compute(ey)
            cmd_fb = -pid_fb.compute(ez)
            tello.send_rc_control(cmd_lr, cmd_fb, cmd_ud, 0)
            publisher.update(pos, mission_state, tello.get_battery(), vel=(0,0,0))
            errors.append(e3d)
            last_pos = pos
            row = {
                "timestamp": t_now, "elapsed": t_now - t_start, "phase": phase_name,
                "mission_state": mission_state,
                "target_x": target["x"], "target_y": target["y"], "target_z": target["z"],
                "pos_x": pos["x"], "pos_y": pos["y"], "pos_z": pos["z"],
                "marker_id": pos["marker_id"],
                "err_x_cm": ex*100, "err_y_cm": ey*100, "err_z_cm": ez*100,
                "err_3d_cm": e3d*100,
                "cmd_lr": cmd_lr, "cmd_fb": cmd_fb, "cmd_ud": cmd_ud,
                "publish_count": publisher.sent_count,
            }
            row.update(telemetry_snapshot(tello))
            logger.log(row)
            if ann is not None:
                cv2.putText(ann, f"5.1 [{phase_name}] err={e3d*100:.0f}cm",
                            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                            (255, 255, 0), 2)
                cv2.putText(ann, f"PUB → SLAVE: {publisher.sent_count}",
                            (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0, 255, 0), 2)
        else:
            tello.send_rc_control(0, 0, 0, 0)
            row = {
                "timestamp": t_now, "elapsed": t_now - t_start, "phase": phase_name,
                "mission_state": mission_state,
                "target_x": target["x"], "target_y": target["y"], "target_z": target["z"],
                "pos_x": None, "pos_y": None, "pos_z": None, "marker_id": None,
                "err_x_cm": None, "err_y_cm": None, "err_z_cm": None, "err_3d_cm": None,
                "cmd_lr": 0, "cmd_fb": 0, "cmd_ud": 0,
                "publish_count": publisher.sent_count,
            }
            row.update(telemetry_snapshot(tello))
            logger.log(row)
            if ann is not None:
                cv2.putText(ann, f"5.1 [{phase_name}] FEEDBACK LOST",
                            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (0, 0, 255), 2)
        if ann is not None:
            cv2.imshow("MASTER (Tello A) — 5.1", ann)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                raise KeyboardInterrupt
        time.sleep(LOOP_DT)
    return errors, last_pos


# ============================================================
# Main
# ============================================================
def main():
    tello = Tello()
    tello.connect()
    bat_initial = tello.get_battery()
    print(f"[INFO] MASTER · Batería Tello A inicial: {bat_initial}%")
    if bat_initial < config.MIN_BATTERY_PCT:
        print("[ERROR] Batería insuficiente."); return

    tracker = ArUcoTracker()
    logger = FlightLogger("test_5_1_master")
    pid_lr = PIDController(**config.PID_LR, output_limit=config.RC_MAX)
    pid_ud = PIDController(**config.PID_UD, output_limit=config.RC_MAX)
    pid_fb = PIDController(**config.PID_FB, output_limit=config.RC_MAX)
    pids = (pid_lr, pid_ud, pid_fb)

    publisher = PosePublisher(SLAVE_ADDR, PUBLISH_HZ)

    tello.streamon()
    t0 = time.time()
    while safe_frame(tello) is None and time.time() - t0 < 5.0: time.sleep(0.1)

    # ---- FASE 1: TAKEOFF ----
    print(f"\n[5.1] FASE 1: TAKEOFF + climb + init hover")
    tello.takeoff()
    print(f"[5.1] IMU wait {IMU_WAIT_S:.0f} s...")
    t0 = time.time()
    while time.time() - t0 < IMU_WAIT_S:
        tello.send_rc_control(0, 0, 0, 0); time.sleep(0.05)

    if INITIAL_CLIMB_CM > 0:
        print(f"[5.1] Subiendo {INITIAL_CLIMB_CM} cm...")
        try: tello.move_up(INITIAL_CLIMB_CM)
        except Exception as e:
            err = str(e)
            print(f"[ERROR] move_up: {err}")
            if "imu" in err.lower():
                try: tello.land(); tello.streamoff()
                except: pass
                logger.close(); return

    # Capturar target_pos del MASTER
    print(f"[5.1] Hover {INIT_HOVER_S:.0f} s para target_pos...")
    init_samples = []
    t0 = time.time()
    while time.time() - t0 < INIT_HOVER_S:
        tello.send_rc_control(0, 0, 0, 0)
        f = safe_frame(tello)
        if f is not None:
            pos, ann = tracker.detect_and_estimate(f)
            if pos: init_samples.append(pos)
            if ann is not None:
                cv2.putText(ann, "5.1 [TAKEOFF] capturando target_pos", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 200, 0), 2)
                cv2.imshow("MASTER (Tello A) — 5.1", ann); cv2.waitKey(1)
        time.sleep(0.05)

    if len(init_samples) < 10:
        print("[ERROR] No suficientes detecciones ArUco.")
        try: tello.land(); tello.streamoff()
        except: pass
        logger.close(); cv2.destroyAllWindows(); return

    tail = init_samples[-int(2.0/0.05):]
    target_pos = {
        "x": statistics.mean(s["x"] for s in tail),
        "y": statistics.mean(s["y"] for s in tail),
        "z": statistics.mean(s["z"] for s in tail),
    }
    print(f"[5.1] target_pos MASTER: X={target_pos['x']:.2f} "
          f"Y={target_pos['y']:.2f} Z={target_pos['z']:.2f}")

    # Iniciar publicador (mission_state=takeoff inicial)
    publisher.update(target_pos, MS_TAKEOFF, tello.get_battery())
    publisher.start()
    print(f"[5.1] Publicando al SLAVE. Slave debería despegar en breve.")

    t_mission_start = time.time()
    last_pose_t_ref = [t_mission_start]   # mutable container para pasar por ref
    all_errors = []

    try:
        # ---- FASE 2: WAIT_SLAVE (master hover, slave despega y se une) ----
        print(f"\n[5.1] FASE 2: WAIT_SLAVE  (esperando {WAIT_SLAVE_S:.0f} s)")
        errs, _ = hover_at_target(tello, tracker, target_pos, pids, publisher,
                                  WAIT_SLAVE_S, MS_HOVER, logger,
                                  t_mission_start, "wait_slave", last_pose_t_ref)
        all_errors.extend(errs)

        # ---- FASE 3: FORMATION (formación estática, ambos en hover) ----
        print(f"\n[5.1] FASE 3: FORMATION  ({FORMATION_HOLD_S:.0f} s)")
        errs, _ = hover_at_target(tello, tracker, target_pos, pids, publisher,
                                  FORMATION_HOLD_S, MS_FORMATION, logger,
                                  t_mission_start, "formation", last_pose_t_ref)
        all_errors.extend(errs)

        # ---- FASE 4: TRAJECTORY (master se mueve a destination) ----
        destination = {
            "x": target_pos["x"] + WAYPOINT_OFFSET_X,
            "y": target_pos["y"],
            "z": target_pos["z"],
        }
        print(f"\n[5.1] FASE 4: TRAJECTORY  → destination "
              f"X={destination['x']:.2f} (offset +{WAYPOINT_OFFSET_X*100:.0f} cm)")
        # Sub-fase navegación al waypoint
        t_traj_start = time.time()
        in_tol_since = None
        while time.time() - t_traj_start < WP_TIMEOUT_S:
            t_now = time.time()
            f = safe_frame(tello)
            pos, ann = (None, None)
            if f is not None: pos, ann = tracker.detect_and_estimate(f)

            if pos and (t_now - last_pose_t_ref[0]) < 1.5:
                last_pose_t_ref[0] = t_now
                ex = destination["x"] - pos["x"]
                ey = destination["y"] - pos["y"]
                ez = destination["z"] - pos["z"]
                e3d = math.sqrt(ex*ex + ey*ey + ez*ez)
                cmd_lr = pid_lr.compute(ex)
                cmd_ud = pid_ud.compute(ey)
                cmd_fb = -pid_fb.compute(ez)
                tello.send_rc_control(cmd_lr, cmd_fb, cmd_ud, 0)
                publisher.update(pos, MS_TRAJECTORY, tello.get_battery(),
                                 vel=(0,0,0))
                all_errors.append(e3d)
                row = {
                    "timestamp": t_now, "elapsed": t_now - t_mission_start,
                    "phase": "trajectory", "mission_state": MS_TRAJECTORY,
                    "target_x": destination["x"], "target_y": destination["y"],
                    "target_z": destination["z"],
                    "pos_x": pos["x"], "pos_y": pos["y"], "pos_z": pos["z"],
                    "marker_id": pos["marker_id"],
                    "err_x_cm": ex*100, "err_y_cm": ey*100, "err_z_cm": ez*100,
                    "err_3d_cm": e3d*100,
                    "cmd_lr": cmd_lr, "cmd_fb": cmd_fb, "cmd_ud": cmd_ud,
                    "publish_count": publisher.sent_count,
                }
                row.update(telemetry_snapshot(tello))
                logger.log(row)
                if ann is not None:
                    cv2.putText(ann, f"5.1 [TRAJECTORY] err={e3d*100:.0f}cm",
                                (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                                (255, 255, 0), 2)
                    cv2.imshow("MASTER (Tello A) — 5.1", ann)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        raise KeyboardInterrupt
                # Llegada
                in_tol = (abs(ex) < WP_TOL_M and abs(ey) < WP_TOL_M
                          and abs(ez) < WP_TOL_M)
                if in_tol:
                    if in_tol_since is None: in_tol_since = t_now
                    elif t_now - in_tol_since >= WP_SETTLE_S:
                        print(f"[5.1] Llegado a destination en "
                              f"{t_now - t_traj_start:.1f}s")
                        break
                else:
                    in_tol_since = None
            else:
                tello.send_rc_control(0, 0, 0, 0)
            time.sleep(LOOP_DT)
        else:
            print(f"[5.1] [WARN] timeout en trajectory.")

        # ---- FASE 5: HOVER_AT_DEST (cooperativo en destination) ----
        print(f"\n[5.1] FASE 5: HOVER_AT_DEST  ({HOVER_AT_DEST_S:.0f} s)")
        errs, _ = hover_at_target(tello, tracker, destination, pids, publisher,
                                  HOVER_AT_DEST_S, MS_HOVER, logger,
                                  t_mission_start, "hover_at_dest",
                                  last_pose_t_ref)
        all_errors.extend(errs)

        # ---- FASE 6: SLAVE_LAND (master ordena al slave aterrizar) ----
        print(f"\n[5.1] FASE 6: SLAVE_LAND  → notificando al slave")
        publisher.send_state_burst(destination, MS_SLAVE_LAND,
                                   tello.get_battery(), n=15)
        # Master mantiene hover mientras el slave aterriza
        errs, _ = hover_at_target(tello, tracker, destination, pids, publisher,
                                  WAIT_FOR_SLAVE_LANDING_S, MS_SLAVE_LAND,
                                  logger, t_mission_start, "wait_slave_land",
                                  last_pose_t_ref)
        all_errors.extend(errs)

        # ---- FASE 7: MASTER_LAND ----
        print(f"\n[5.1] FASE 7: MASTER_LAND")
        publisher.send_state_burst(destination, MS_LANDING,
                                   tello.get_battery(), n=10)

    except KeyboardInterrupt:
        print("\n[5.1] Interrumpido por usuario.")
    finally:
        # Asegurar que el slave reciba el landing
        try:
            with publisher._lock:
                snap = publisher._latest
            if snap is not None:
                publisher.update(snap[0], MS_LANDING,
                                 tello.get_battery(), vel=(0,0,0))
            publisher.send_state_burst(snap[0] if snap else target_pos,
                                       MS_LANDING, tello.get_battery(), n=5)
        except Exception:
            pass
        publisher.stop()

        tello.send_rc_control(0, 0, 0, 0); time.sleep(0.5)
        try: tello.land()
        except Exception: pass
        try: tello.streamoff()
        except Exception: pass
        logger.close()
        cv2.destroyAllWindows()

        bat_final = tello.get_battery() or bat_initial
        if all_errors:
            elapsed_total = time.time() - t_mission_start
            print("\n" + "="*64)
            print("RESULTADOS — MASTER 5.1 (misión cooperativa completa)")
            print("="*64)
            print(f"  Duración total misión:       {elapsed_total:.1f} s")
            print(f"  Mensajes publicados:         {publisher.sent_count}")
            print(f"  Error 3D promedio (todo):    {statistics.mean(all_errors)*100:.2f} cm")
            print(f"  Error 3D máximo:              {max(all_errors)*100:.2f} cm")
            print(f"  Batería: {bat_initial}% → {bat_final}% (consumo {bat_initial-bat_final}%)")
            print("="*64)


if __name__ == "__main__":
    main()
