"""
═══════════════════════════════════════════════════════════════
PRUEBA 2.3 — Formación dinámica líder-seguidor (MASTER, Mac)
═══════════════════════════════════════════════════════════════
Drones: 1 (Tello A)  |  Complejidad: Alta  |  Tiempo: ~5 min vuelo

ROL: MASTER (líder). Después del takeoff y captura de target_pos,
recorre una trayectoria CUADRADA en el plano X-Z (altura constante)
mientras publica su posición al SLAVE a 50 Hz. El control de
seguimiento de waypoint usa el mismo PID validado en 2.1/2.2 con
RC_MAX = 30 cm/s.

USO:
    python test_2_3_master.py

ABORT: 'q' o Ctrl+C. El finally aterriza el dron y notifica al SLAVE.
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
LOOP_DT            = 0.05
PUBLISH_HZ         = 50

# Trayectoria CUADRADO 0.6 × 0.6 m en plano X-Z (Y constante)
SQUARE_HALF_SIDE_M  = 0.30      # ±30 cm desde target_pos en X y Z
WAYPOINT_TOL_M      = 0.12      # tolerancia para considerar "llegado" (12 cm)
WAYPOINT_SETTLE_S   = 0.5       # tiempo dentro de tolerancia para confirmar
HOVER_AT_WP_S       = 3.0       # hover en cada esquina
LEG_TIMEOUT_S       = 15.0      # timeout máximo por waypoint

DRONE_ID = 1
SLAVE_ADDR = (config.SLAVE_IP, config.COMMS_PORT)

MISSION_STATE_TRAJECTORY = 4
MISSION_STATE_LANDING    = 6


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
# Hilo publicador (igual que 2.2)
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

    def send_termination(self, mission_state=MISSION_STATE_LANDING):
        with self._lock: snap = self._latest
        if snap is None: return
        pos, _, bat, vel = snap
        msg = CoopMessage(
            drone_id=DRONE_ID, seq=self._seq, timestamp=time.time(),
            pos_x=pos["x"], pos_y=pos["y"], pos_z=pos["z"],
            vel_x=vel[0], vel_y=vel[1], vel_z=vel[2],
            battery=int(bat) if bat else 0, mission_state=mission_state,
        )
        try:
            for _ in range(5):
                self._sock.sendto(encode_binary(msg), self._addr)
                time.sleep(0.02)
        except Exception: pass

    def stop(self):
        self._stop.set()
        if self._thread: self._thread.join(timeout=1.0)
        self._sock.close()


# ============================================================
# Generar la trayectoria cuadrada relativa a target_pos
# ============================================================
def square_trajectory(target_pos, half_side_m):
    """
    Devuelve lista de waypoints (dicts con x, y, z) que forman un cuadrado
    en el plano X-Z (Y constante), centrado en target_pos.

    Orden:  WP1 (front-left)  →  WP2 (front-right)
                                    ↓
            WP4 (back-left)  ←  WP3 (back-right)
                                    ↓
            return WP5 = target_pos (centro)

    Donde "front" = más cerca de la pared (Z más pequeño),
          "back"  = más lejos (Z más grande).
    """
    h = half_side_m
    return [
        {"x": target_pos["x"] - h, "y": target_pos["y"], "z": target_pos["z"] - h, "name": "front-left"},
        {"x": target_pos["x"] + h, "y": target_pos["y"], "z": target_pos["z"] - h, "name": "front-right"},
        {"x": target_pos["x"] + h, "y": target_pos["y"], "z": target_pos["z"] + h, "name": "back-right"},
        {"x": target_pos["x"] - h, "y": target_pos["y"], "z": target_pos["z"] + h, "name": "back-left"},
        {"x": target_pos["x"],     "y": target_pos["y"], "z": target_pos["z"],     "name": "center (return)"},
    ]


# ============================================================
# Main
# ============================================================
def main():
    tello = Tello()
    tello.connect()
    bat = tello.get_battery()
    print(f"[INFO] MASTER · Batería Tello A: {bat}%")
    if bat < config.MIN_BATTERY_PCT:
        print("[ERROR] Batería insuficiente."); return

    tracker = ArUcoTracker()
    logger = FlightLogger("test_2_3_master")
    pid_lr = PIDController(**config.PID_LR, output_limit=config.RC_MAX)
    pid_ud = PIDController(**config.PID_UD, output_limit=config.RC_MAX)
    pid_fb = PIDController(**config.PID_FB, output_limit=config.RC_MAX)

    publisher = PosePublisher(SLAVE_ADDR, PUBLISH_HZ)

    tello.streamon()
    t0 = time.time()
    while safe_frame(tello) is None and time.time() - t0 < 5.0: time.sleep(0.1)

    tello.takeoff()
    print(f"[INFO] Esperando {IMU_WAIT_S:.0f} s estabilización IMU...")
    t0 = time.time()
    while time.time() - t0 < IMU_WAIT_S:
        tello.send_rc_control(0, 0, 0, 0); time.sleep(0.05)

    if INITIAL_CLIMB_CM > 0:
        print(f"[INFO] Subiendo {INITIAL_CLIMB_CM} cm...")
        try: tello.move_up(INITIAL_CLIMB_CM)
        except Exception as e:
            err = str(e)
            print(f"[ERROR] move_up falló: {err}")
            if "imu" in err.lower():
                try: tello.land(); tello.streamoff()
                except: pass
                logger.close(); return

    # ----- Captura target_pos -----
    print(f"[INFO] Hover inicial {INIT_HOVER_S:.0f} s para target_pos...")
    init_samples = []
    t0 = time.time()
    while time.time() - t0 < INIT_HOVER_S:
        tello.send_rc_control(0, 0, 0, 0)
        f = safe_frame(tello)
        if f is not None:
            pos, ann = tracker.detect_and_estimate(f)
            if pos: init_samples.append(pos)
            if ann is not None:
                cv2.putText(ann, "MASTER 2.3 · captando target_pos",
                            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 200, 0), 2)
                cv2.imshow("MASTER (Tello A)", ann); cv2.waitKey(1)
        time.sleep(0.05)

    if len(init_samples) < 10:
        print("[ERROR] No suficientes detecciones ArUco.");
        try: tello.land(); tello.streamoff()
        except: pass
        logger.close(); cv2.destroyAllWindows(); return

    tail = init_samples[-int(2.0/0.05):]
    target_pos = {
        "x": statistics.mean(s["x"] for s in tail),
        "y": statistics.mean(s["y"] for s in tail),
        "z": statistics.mean(s["z"] for s in tail),
    }
    print(f"[INFO] target_pos MASTER = X={target_pos['x']:.2f} Y={target_pos['y']:.2f} Z={target_pos['z']:.2f}")

    # ----- Generar trayectoria + arrancar publicador -----
    waypoints = square_trajectory(target_pos, SQUARE_HALF_SIDE_M)
    print(f"\n[INFO] Trayectoria: cuadrado {2*SQUARE_HALF_SIDE_M*100:.0f} × {2*SQUARE_HALF_SIDE_M*100:.0f} cm en X-Z")
    for i, wp in enumerate(waypoints, 1):
        print(f"  WP{i} ({wp['name']:<18}): X={wp['x']:.2f} Y={wp['y']:.2f} Z={wp['z']:.2f}")
    publisher.start()
    print(f"\n[INFO] Publicando al SLAVE a {PUBLISH_HZ} Hz. Trayectoria iniciando.\n")

    # ----- Bucle principal: ir waypoint por waypoint -----
    t_start = time.time()
    last_pose_t = t_start
    last_pos_for_vel = None
    last_t_for_vel = None
    errors_3d_global = []
    loop_count = 0

    def log_row(phase, wp_idx, wp, pos, ex, ey, ez, e3d, cmd_lr, cmd_fb, cmd_ud,
                feedback_state):
        t_now = time.time()
        row = {
            "timestamp": t_now, "elapsed": t_now - t_start, "phase": phase,
            "wp_idx": wp_idx, "wp_name": wp["name"] if wp else None,
            "wp_x": wp["x"] if wp else None,
            "wp_y": wp["y"] if wp else None,
            "wp_z": wp["z"] if wp else None,
            "pos_x": pos["x"] if pos else None,
            "pos_y": pos["y"] if pos else None,
            "pos_z": pos["z"] if pos else None,
            "marker_id": pos["marker_id"] if pos else None,
            "err_x_cm": ex*100 if ex is not None else None,
            "err_y_cm": ey*100 if ey is not None else None,
            "err_z_cm": ez*100 if ez is not None else None,
            "err_3d_cm": e3d*100 if e3d is not None else None,
            "cmd_lr": cmd_lr, "cmd_fb": cmd_fb, "cmd_ud": cmd_ud,
            "feedback": feedback_state,
            "publish_count": publisher.sent_count,
        }
        row.update(telemetry_snapshot(tello))
        logger.log(row)

    try:
        for wp_idx, wp in enumerate(waypoints, 1):
            print(f"\n[WP{wp_idx}/{len(waypoints)}] → {wp['name']:<18}  "
                  f"X={wp['x']:.2f} Y={wp['y']:.2f} Z={wp['z']:.2f}")

            t_leg_start = time.time()
            in_tolerance_since = None

            # ---- Fase 1: navegar al waypoint con PID ----
            while time.time() - t_leg_start < LEG_TIMEOUT_S:
                t_now = time.time()
                f = safe_frame(tello)
                pos, ann = (None, None)
                if f is not None: pos, ann = tracker.detect_and_estimate(f)

                if pos and (t_now - last_pose_t) < 1.5:
                    last_pose_t = t_now
                    ex = wp["x"] - pos["x"]
                    ey = wp["y"] - pos["y"]
                    ez = wp["z"] - pos["z"]
                    e3d = math.sqrt(ex*ex + ey*ey + ez*ez)

                    cmd_lr = pid_lr.compute(ex)
                    cmd_ud = pid_ud.compute(ey)
                    cmd_fb = -pid_fb.compute(ez)
                    tello.send_rc_control(cmd_lr, cmd_fb, cmd_ud, 0)

                    # Estimar velocidad para publicar al SLAVE
                    vel = (0.0, 0.0, 0.0)
                    if last_pos_for_vel and last_t_for_vel:
                        dt = t_now - last_t_for_vel
                        if dt > 0.01:
                            vel = ((pos["x"] - last_pos_for_vel["x"]) / dt,
                                   (pos["y"] - last_pos_for_vel["y"]) / dt,
                                   (pos["z"] - last_pos_for_vel["z"]) / dt)
                    last_pos_for_vel = pos; last_t_for_vel = t_now

                    publisher.update(pos, mission_state=MISSION_STATE_TRAJECTORY,
                                     battery=tello.get_battery(), vel=vel)
                    errors_3d_global.append(e3d)

                    log_row("nav", wp_idx, wp, pos, ex, ey, ez, e3d,
                            cmd_lr, cmd_fb, cmd_ud, "ok")

                    # Verificar si llegamos
                    in_tol = (abs(ex) < WAYPOINT_TOL_M and
                              abs(ey) < WAYPOINT_TOL_M and
                              abs(ez) < WAYPOINT_TOL_M)
                    if in_tol:
                        if in_tolerance_since is None:
                            in_tolerance_since = t_now
                        elif t_now - in_tolerance_since >= WAYPOINT_SETTLE_S:
                            print(f"  ✓ Llegado a WP{wp_idx} en "
                                  f"{t_now - t_leg_start:.1f} s, "
                                  f"err_3d = {e3d*100:.1f} cm")
                            break
                    else:
                        in_tolerance_since = None

                    if ann is not None:
                        cv2.putText(ann, f"WP{wp_idx} ({wp['name']}) · "
                                          f"err {e3d*100:.0f} cm",
                                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                                    (255, 255, 0), 2)
                        cv2.putText(ann, f"PUB → SLAVE: {publisher.sent_count}",
                                    (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                    (0, 255, 0), 2)
                else:
                    tello.send_rc_control(0, 0, 0, 0)
                    log_row("nav", wp_idx, wp, None,
                            None, None, None, None, 0, 0, 0, "lost")
                    if ann is not None:
                        cv2.putText(ann, "FEEDBACK LOST — HOVER",
                                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                                    (0, 0, 255), 2)

                if ann is not None:
                    cv2.imshow("MASTER (Tello A)", ann)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        raise KeyboardInterrupt

                loop_count += 1
                time.sleep(LOOP_DT)
            else:
                print(f"  [WARN] WP{wp_idx} timeout. Pasando al siguiente.")

            # ---- Fase 2: hover en el waypoint ----
            print(f"  Hover {HOVER_AT_WP_S:.0f} s en WP{wp_idx}...")
            t_hover_start = time.time()
            while time.time() - t_hover_start < HOVER_AT_WP_S:
                t_now = time.time()
                f = safe_frame(tello)
                pos, ann = (None, None)
                if f is not None: pos, ann = tracker.detect_and_estimate(f)

                if pos:
                    ex = wp["x"] - pos["x"]
                    ey = wp["y"] - pos["y"]
                    ez = wp["z"] - pos["z"]
                    e3d = math.sqrt(ex*ex + ey*ey + ez*ez)
                    cmd_lr = pid_lr.compute(ex)
                    cmd_ud = pid_ud.compute(ey)
                    cmd_fb = -pid_fb.compute(ez)
                    tello.send_rc_control(cmd_lr, cmd_fb, cmd_ud, 0)
                    publisher.update(pos, mission_state=MISSION_STATE_TRAJECTORY,
                                     battery=tello.get_battery(),
                                     vel=(0.0, 0.0, 0.0))
                    log_row("hover_at_wp", wp_idx, wp, pos, ex, ey, ez, e3d,
                            cmd_lr, cmd_fb, cmd_ud, "ok")
                    if ann is not None:
                        cv2.putText(ann, f"WP{wp_idx} HOVER · err {e3d*100:.0f} cm",
                                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                                    (255, 200, 0), 2)
                else:
                    tello.send_rc_control(0, 0, 0, 0)
                    log_row("hover_at_wp", wp_idx, wp, None,
                            None, None, None, None, 0, 0, 0, "lost")
                if ann is not None:
                    cv2.imshow("MASTER (Tello A)", ann)
                    if cv2.waitKey(1) & 0xFF == ord('q'): raise KeyboardInterrupt
                loop_count += 1
                time.sleep(LOOP_DT)

        print("\n[INFO] Trayectoria completada.")

    except KeyboardInterrupt:
        print("\n[INFO] Interrumpido por usuario.")
    finally:
        publisher.update(target_pos, mission_state=MISSION_STATE_LANDING,
                         battery=tello.get_battery(), vel=(0,0,0))
        publisher.send_termination(MISSION_STATE_LANDING)
        publisher.stop()

        tello.send_rc_control(0, 0, 0, 0); time.sleep(0.5)
        try: tello.land()
        except Exception: pass
        try: tello.streamoff()
        except Exception: pass
        logger.close()
        cv2.destroyAllWindows()

        if errors_3d_global:
            elapsed = time.time() - t_start
            print("\n" + "="*64)
            print("RESULTADOS — MASTER 2.3 (Tello A)")
            print("="*64)
            print(f"  Lazo: {loop_count/elapsed:.1f} Hz")
            print(f"  Mensajes publicados: {publisher.sent_count}")
            print(f"  Error 3D al target del WP medio:    "
                  f"{statistics.mean(errors_3d_global)*100:.2f} cm")
            print(f"  Error 3D máximo:                     "
                  f"{max(errors_3d_global)*100:.2f} cm")
            print("="*64)


if __name__ == "__main__":
    main()
