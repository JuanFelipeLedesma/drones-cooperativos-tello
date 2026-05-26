"""
═══════════════════════════════════════════════════════════════
PRUEBA 3.4 — Tolerancia a fallas de comunicación (MASTER, Mac)
═══════════════════════════════════════════════════════════════
Drones: 1 (Tello A)  |  Complejidad: Media  |  Tiempo: ~3 min vuelo

ROL: MASTER (líder). Mantiene hover estable y PUBLICA su posición
mundo al SLAVE por Ethernet a 50 Hz.

Setup requerido:
    - Mac conectado por WiFi al Tello-E92E66 (Tello A).
    - Ethernet conectado al Ubuntu (192.168.1.2).
    - Receiver / SLAVE corriendo en Ubuntu en paralelo.
    - Markers ArUco visibles para el MASTER.

USO:
    python test_3_4_master.py

ABORTAR: 'q' en la ventana, o Ctrl+C en la terminal. El finally
aterriza al dron limpio y notifica al SLAVE de que la misión terminó.
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

# Reusar codec binario validado en 3.2
from test_3_2_protocol import (
    CoopMessage, encode_binary,
)

# ----------------------------------------------------------------
# Parámetros de la misión
# ----------------------------------------------------------------
INITIAL_CLIMB_CM   = 60          # subida tras takeoff
IMU_WAIT_S         = 4.0         # estabilización IMU
INIT_HOVER_S       = 5.0         # captura de target_pos
MISSION_DURATION_S = 120.0   # 2 min para hacer las 3 desconexiones        # 60 s de hover cooperativo en formación
LOOP_DT            = 0.05        # ~20 Hz lazo PID
PUBLISH_HZ         = 50          # frecuencia de envío al SLAVE

DRONE_ID = 1                     # MASTER es id=1
SLAVE_ADDR = (config.SLAVE_IP, config.COMMS_PORT)


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
# Hilo publicador: manda la última pose al SLAVE a PUBLISH_HZ
# ============================================================
class PosePublisher:
    """Thread-safe publicador de pose al SLAVE."""
    def __init__(self, target_addr, hz):
        self._addr = target_addr
        self._interval = 1.0 / hz
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._lock = threading.Lock()
        self._latest = None        # (pos_dict, mission_state, battery, vel)
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
                        battery=int(bat) if bat else 0,
                        mission_state=ms,
                    )
                    try:
                        self._sock.sendto(encode_binary(msg), self._addr)
                        self.sent_count += 1
                        self._seq = (self._seq + 1) & 0xFFFFFFFF
                    except Exception:
                        pass
                next_t += self._interval
            else:
                time.sleep(min(0.005, next_t - now))

    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def send_termination(self, mission_state=6):
        """Envía un mensaje final con mission_state=landing/emergency. Redundante."""
        with self._lock:
            snap = self._latest
        if snap is None: return
        pos, _, bat, vel = snap
        msg = CoopMessage(
            drone_id=DRONE_ID, seq=self._seq, timestamp=time.time(),
            pos_x=pos["x"], pos_y=pos["y"], pos_z=pos["z"],
            vel_x=vel[0], vel_y=vel[1], vel_z=vel[2],
            battery=int(bat) if bat else 0,
            mission_state=mission_state,
        )
        try:
            for _ in range(5):
                self._sock.sendto(encode_binary(msg), self._addr)
                time.sleep(0.02)
        except Exception:
            pass

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        self._sock.close()


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
    logger = FlightLogger("test_3_4_master")
    pid_lr = PIDController(**config.PID_LR, output_limit=config.RC_MAX)
    pid_ud = PIDController(**config.PID_UD, output_limit=config.RC_MAX)
    pid_fb = PIDController(**config.PID_FB, output_limit=config.RC_MAX)

    publisher = PosePublisher(SLAVE_ADDR, PUBLISH_HZ)

    tello.streamon()
    t0 = time.time()
    while safe_frame(tello) is None and time.time() - t0 < 5.0:
        time.sleep(0.1)

    tello.takeoff()
    print(f"[INFO] Esperando {IMU_WAIT_S:.0f} s estabilización IMU...")
    t0 = time.time()
    while time.time() - t0 < IMU_WAIT_S:
        tello.send_rc_control(0, 0, 0, 0); time.sleep(0.05)

    if INITIAL_CLIMB_CM > 0:
        print(f"[INFO] Subiendo {INITIAL_CLIMB_CM} cm...")
        try:
            tello.move_up(INITIAL_CLIMB_CM)
        except Exception as e:
            err = str(e)
            print(f"[ERROR] move_up falló: {err}")
            if "imu" in err.lower():
                print("[ABORT] IMU no válido. Reinicia el Tello.")
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
                cv2.putText(ann, "MASTER · captando target_pos",
                            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 200, 0), 2)
                cv2.imshow("MASTER (Tello A)", ann); cv2.waitKey(1)
        time.sleep(0.05)

    if len(init_samples) < 10:
        print("[ERROR] No suficientes detecciones ArUco para fijar target_pos.")
        try: tello.land(); tello.streamoff()
        except: pass
        logger.close(); cv2.destroyAllWindows(); return

    tail = init_samples[-int(2.0/0.05):]
    target = {
        "x": statistics.mean(s["x"] for s in tail),
        "y": statistics.mean(s["y"] for s in tail),
        "z": statistics.mean(s["z"] for s in tail),
    }
    print(f"[INFO] target_pos MASTER = X={target['x']:.2f} Y={target['y']:.2f} Z={target['z']:.2f}")

    # ----- Arranca el publicador -----
    publisher.start()
    print(f"[INFO] Publicando posición al SLAVE ({SLAVE_ADDR}) a {PUBLISH_HZ} Hz")
    print(f"[INFO] El SLAVE va a despegar al detectar mensajes. Espera unos segundos.\n")

    # ----- Hover cooperativo durante MISSION_DURATION_S -----
    t_start = time.time()
    last_pose_t = t_start
    last_pos_for_vel = None
    last_t_for_vel = None
    errors_3d = []
    loop_count = 0

    print(f"[INFO] Hover cooperativo por {MISSION_DURATION_S:.0f} s. 'q' para abortar.\n")

    try:
        while time.time() - t_start < MISSION_DURATION_S:
            t_now = time.time()
            f = safe_frame(tello)
            pos, ann = (None, None)
            if f is not None: pos, ann = tracker.detect_and_estimate(f)

            row = {
                "timestamp": t_now, "elapsed": t_now - t_start,
                "target_x": target["x"], "target_y": target["y"], "target_z": target["z"],
                "pos_x": pos["x"] if pos else None,
                "pos_y": pos["y"] if pos else None,
                "pos_z": pos["z"] if pos else None,
                "marker_id": pos["marker_id"] if pos else None,
                "publish_count": publisher.sent_count,
            }

            if pos and (t_now - last_pose_t) < 1.5:
                last_pose_t = t_now
                ex = target["x"] - pos["x"]
                ey = target["y"] - pos["y"]
                ez = target["z"] - pos["z"]
                e3d = math.sqrt(ex*ex + ey*ey + ez*ez)

                cmd_lr = pid_lr.compute(ex)
                cmd_ud = pid_ud.compute(ey)
                cmd_fb = -pid_fb.compute(ez)
                tello.send_rc_control(cmd_lr, cmd_fb, cmd_ud, 0)

                # Estimar velocidad del MASTER (diferencias finitas)
                vel = (0.0, 0.0, 0.0)
                if last_pos_for_vel and last_t_for_vel:
                    dt = t_now - last_t_for_vel
                    if dt > 0.01:
                        vel = ((pos["x"] - last_pos_for_vel["x"]) / dt,
                               (pos["y"] - last_pos_for_vel["y"]) / dt,
                               (pos["z"] - last_pos_for_vel["z"]) / dt)
                last_pos_for_vel = pos; last_t_for_vel = t_now

                # mission_state=3 = formation
                publisher.update(pos, mission_state=3,
                                 battery=tello.get_battery(), vel=vel)

                row.update({
                    "err_x_cm": ex*100, "err_y_cm": ey*100,
                    "err_z_cm": ez*100, "err_3d_cm": e3d*100,
                    "cmd_lr": cmd_lr, "cmd_fb": cmd_fb, "cmd_ud": cmd_ud,
                    "vel_x_pub": vel[0], "vel_y_pub": vel[1], "vel_z_pub": vel[2],
                })
                errors_3d.append(e3d)

                if ann is not None:
                    cv2.putText(ann, f"MASTER · Err 3D: {e3d*100:.1f} cm",
                                (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                    cv2.putText(ann, f"PUB → SLAVE: {publisher.sent_count} msg",
                                (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            else:
                tello.send_rc_control(0, 0, 0, 0)
                row.update({"err_x_cm": None, "err_y_cm": None, "err_z_cm": None,
                            "err_3d_cm": None, "cmd_lr": 0, "cmd_fb": 0, "cmd_ud": 0})
                if ann is not None:
                    cv2.putText(ann, "FEEDBACK LOST — HOVER",
                                (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            row.update(telemetry_snapshot(tello))
            logger.log(row)

            if ann is not None:
                cv2.imshow("MASTER (Tello A)", ann)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("[INFO] Abort manual con 'q'."); break

            loop_count += 1
            time.sleep(LOOP_DT)

    except KeyboardInterrupt:
        print("\n[INFO] Interrumpido por usuario.")
    finally:
        publisher.update(target, mission_state=6,
                         battery=tello.get_battery(), vel=(0,0,0))
        publisher.send_termination(mission_state=6)
        publisher.stop()

        tello.send_rc_control(0, 0, 0, 0); time.sleep(0.5)
        try: tello.land()
        except Exception: pass
        try: tello.streamoff()
        except Exception: pass
        logger.close()
        cv2.destroyAllWindows()

        if errors_3d:
            elapsed = time.time() - t_start
            cut = max(1, len(errors_3d)//4)
            steady = errors_3d[cut:]
            print("\n" + "="*64)
            print("RESULTADOS — MASTER (Tello A)")
            print("="*64)
            print(f"  Lazo de hover: {loop_count/elapsed:.1f} Hz")
            print(f"  Mensajes publicados al SLAVE: {publisher.sent_count}")
            print(f"  Error hover 3D estacionario: {statistics.mean(steady)*100:.2f} cm")
            print(f"  Error hover 3D máximo:        {max(errors_3d)*100:.2f} cm")
            print("="*64)


if __name__ == "__main__":
    main()
