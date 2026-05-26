"""
═══════════════════════════════════════════════════════════════
PRUEBA 3.3 — Degradación de red durante formación (SLAVE, Ubuntu)
═══════════════════════════════════════════════════════════════
Drones: 1 (Tello B)  |  Complejidad: Media  |  Tiempo: ~3 min vuelo

ROL: SLAVE (seguidor). Recibe la posición del MASTER por Ethernet,
calcula su target = pos_master + offset_formacion, y mantiene
la formación con su propio lazo cerrado ArUco.

Setup requerido:
    - Ubuntu conectado por WiFi al Tello-E92948 (Tello B).
    - Ethernet conectado al Mac (192.168.1.1).
    - MASTER ya corriendo (o por arrancar) en el Mac.
    - Markers ArUco visibles para el SLAVE.

USO (en el Ubuntu):
    python test_3_3_slave.py

ABORTAR: 'q' o Ctrl+C. El finally aterriza el dron.

LÓGICA DE SEGURIDAD:
    - Antes de despegar: espera al primer mensaje del MASTER (timeout 30 s).
    - Durante vuelo: si pierde mensajes del MASTER por > COMMS_TIMEOUT_S,
      hace hover seguro (rc 0 0 0 0) en su última posición.
    - Si recibe mission_state=6 (landing) o =7 (emergency), aterriza.
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

from collections import deque

from utils import ArUcoTracker, FlightLogger  # noqa: E402
from utils.pid import PIDController  # noqa: E402
import config  # noqa: E402

from test_3_2_protocol import (
    CoopMessage, decode_binary, BINARY_SIZE,
)

# ----------------------------------------------------------------
# Parámetros de la misión
# ----------------------------------------------------------------
INITIAL_CLIMB_CM   = 60          # subida tras takeoff
IMU_WAIT_S         = 4.0
INIT_HOVER_S       = 5.0
WAIT_FIRST_MSG_S   = 30.0        # timeout al esperar primer mensaje del MASTER
LOOP_DT            = 0.05
LOST_TIMEOUT_S     = config.COMMS_TIMEOUT_S   # 5 s de silencio → hover seguro

DRONE_ID = 2                     # SLAVE es id=2
LISTEN_PORT = config.COMMS_PORT  # recibe del MASTER en este puerto

# 3.3: archivo flag escrito por test_3_3_inject.py con la condición de red activa.
# Cada iteración del slave lee este archivo para etiquetar la fila del CSV.
NET_CONDITION_FILE = "/tmp/net_condition.txt"


def read_network_condition():
    try:
        with open(NET_CONDITION_FILE, "r") as f:
            return f.read().strip()
    except (FileNotFoundError, OSError):
        return "unknown"

# Offset de formación (del config)
OFFSET = (config.FORMATION_OFFSET_X,
          config.FORMATION_OFFSET_Y,
          config.FORMATION_OFFSET_Z)

# Filtro promedio móvil sobre la pose del MASTER. Evita que el SLAVE
# "persiga el ruido" del hover del MASTER (que tiene σ ≈ 5 cm por eje).
# Con N=8 a ~14 Hz tenemos ventana de ~0.6 s — suficientemente corta
# para responder a movimientos reales del MASTER, pero larga para filtrar
# el ruido de pose ArUco.
MASTER_POSE_FILTER_N = 8


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
# Hilo receiver: actualiza el último mensaje del MASTER
# ============================================================
class MasterListener:
    """Escucha mensajes del MASTER y mantiene la última pose recibida."""
    def __init__(self, port):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(("0.0.0.0", port))
        self._sock.settimeout(0.2)
        self._lock = threading.Lock()
        self._latest = None       # CoopMessage
        self._last_recv_t = 0.0
        self._stop = threading.Event()
        self._thread = None
        self.received_count = 0
        self.crc_errors = 0

    def _run(self):
        while not self._stop.is_set():
            try:
                data, _ = self._sock.recvfrom(4096)
                if len(data) != BINARY_SIZE:
                    continue
                try:
                    msg, crc_ok = decode_binary(data)
                except Exception:
                    continue
                if not crc_ok:
                    self.crc_errors += 1
                    continue
                with self._lock:
                    self._latest = msg
                    self._last_recv_t = time.time()
                    self.received_count += 1
            except socket.timeout:
                continue
            except Exception:
                continue

    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def get(self):
        with self._lock:
            return self._latest, self._last_recv_t

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        self._sock.close()


def wait_for_first_message(listener, timeout_s):
    """Espera hasta que llegue el primer mensaje del MASTER."""
    print(f"[SLAVE] Esperando primer mensaje del MASTER (timeout {timeout_s:.0f} s)...")
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        msg, _ = listener.get()
        if msg is not None:
            print(f"[SLAVE] ✓ Mensaje recibido: drone_id={msg.drone_id}, "
                  f"pos=({msg.pos_x:.2f}, {msg.pos_y:.2f}, {msg.pos_z:.2f})")
            return msg
        time.sleep(0.1)
    return None


# ============================================================
# Main
# ============================================================
def main():
    tello = Tello()
    tello.connect()
    bat = tello.get_battery()
    print(f"[INFO] SLAVE · Batería Tello B: {bat}%")
    if bat < config.MIN_BATTERY_PCT:
        print("[ERROR] Batería insuficiente."); return

    tracker = ArUcoTracker()
    logger = FlightLogger("test_3_3_slave")
    pid_lr = PIDController(**config.PID_LR, output_limit=config.RC_MAX)
    pid_ud = PIDController(**config.PID_UD, output_limit=config.RC_MAX)
    pid_fb = PIDController(**config.PID_FB, output_limit=config.RC_MAX)

    listener = MasterListener(LISTEN_PORT)
    listener.start()
    print(f"[INFO] SLAVE escuchando en puerto {LISTEN_PORT}")
    print(f"[INFO] Offset de formación: dx={OFFSET[0]:+.1f} dy={OFFSET[1]:+.1f} dz={OFFSET[2]:+.1f} m")

    # Esperar el primer mensaje del MASTER ANTES de despegar
    first_msg = wait_for_first_message(listener, WAIT_FIRST_MSG_S)
    if first_msg is None:
        print("[ERROR] Timeout sin recibir mensajes del MASTER. Abort.")
        listener.stop(); logger.close(); return

    # Despegar
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
                print("[ABORT] IMU no válido.");
                try: tello.land(); tello.streamoff()
                except: pass
                listener.stop(); logger.close(); return

    # ----- Hover inicial breve (no captura target_pos: el target es dinámico) -----
    print(f"[INFO] Hover inicial {INIT_HOVER_S:.0f} s...")
    t0 = time.time()
    pre_samples = []
    while time.time() - t0 < INIT_HOVER_S:
        tello.send_rc_control(0, 0, 0, 0)
        f = safe_frame(tello)
        if f is not None:
            pos, ann = tracker.detect_and_estimate(f)
            if pos: pre_samples.append(pos)
            if ann is not None:
                cv2.putText(ann, "SLAVE · pre-hover", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 200, 0), 2)
                cv2.imshow("SLAVE (Tello B)", ann); cv2.waitKey(1)
        time.sleep(0.05)

    if len(pre_samples) < 5:
        print("[ERROR] No suficientes detecciones ArUco en pre-hover. Abort.")
        try: tello.land(); tello.streamoff()
        except: pass
        listener.stop(); logger.close(); cv2.destroyAllWindows(); return

    # ----- Lazo cooperativo: target = filtered(pos_master) + OFFSET -----
    print(f"\n[INFO] Lazo de formación activo. 'q' para abortar.")
    print(f"[INFO] Filtro promedio móvil del MASTER: ventana N={MASTER_POSE_FILTER_N}\n")
    t_start = time.time()
    last_pose_t = t_start
    errors_3d = []
    formation_errors = []   # error vs el target dinámico
    loop_count = 0
    last_master_seq = -1
    landing_requested = False
    # Buffer del filtro promedio móvil de la pose del MASTER
    master_pose_buf = deque(maxlen=MASTER_POSE_FILTER_N)

    try:
        while True:
            t_now = time.time()
            f = safe_frame(tello)
            pos, ann = (None, None)
            if f is not None: pos, ann = tracker.detect_and_estimate(f)

            master_msg, master_t = listener.get()
            master_age = t_now - master_t if master_t else 999

            # Verificar terminación enviada por el MASTER
            if master_msg and master_msg.mission_state in (6, 7):
                print(f"[INFO] MASTER notificó fin de misión "
                      f"(mission_state={master_msg.mission_state}). Aterrizando.")
                landing_requested = True
                break

            row = {
                "timestamp": t_now, "elapsed": t_now - t_start,
                "master_age_s": master_age,
                "master_seq": master_msg.seq if master_msg else None,
                "master_pos_x": master_msg.pos_x if master_msg else None,
                "master_pos_y": master_msg.pos_y if master_msg else None,
                "master_pos_z": master_msg.pos_z if master_msg else None,
                "master_state": master_msg.mission_state if master_msg else None,
                "pos_x": pos["x"] if pos else None,
                "pos_y": pos["y"] if pos else None,
                "pos_z": pos["z"] if pos else None,
                "marker_id": pos["marker_id"] if pos else None,
                "received_count": listener.received_count,
                "crc_errors": listener.crc_errors,
                # 3.3: condición de red en este instante (escrita por inject script)
                "network_condition": read_network_condition(),
            }

            # Decidir qué hacer
            if master_msg is None or master_age > LOST_TIMEOUT_S:
                # Sin feedback del MASTER → hover seguro
                tello.send_rc_control(0, 0, 0, 0)
                row.update({"target_x": None, "target_y": None, "target_z": None,
                            "err_x_cm": None, "err_y_cm": None, "err_z_cm": None,
                            "err_3d_cm": None, "cmd_lr": 0, "cmd_fb": 0, "cmd_ud": 0,
                            "feedback": "lost_master"})
                if ann is not None:
                    cv2.putText(ann, f"SIN MASTER ({master_age:.1f}s) — HOVER",
                                (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            elif pos is None or (t_now - last_pose_t) > 1.5:
                # Tengo MASTER pero sin pose propia → hover seguro
                tello.send_rc_control(0, 0, 0, 0)
                row.update({"target_x": None, "target_y": None, "target_z": None,
                            "err_x_cm": None, "err_y_cm": None, "err_z_cm": None,
                            "err_3d_cm": None, "cmd_lr": 0, "cmd_fb": 0, "cmd_ud": 0,
                            "feedback": "lost_self"})
                if ann is not None:
                    cv2.putText(ann, "SLAVE · sin pose ArUco propia",
                                (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            else:
                # Caso normal: lazo cerrado contra target dinámico FILTRADO.
                # Acumulamos la pose del MASTER en un buffer y promediamos
                # antes de añadir el offset. Esto suaviza el target y evita
                # que el SLAVE persiga el ruido del hover del MASTER.
                last_pose_t = t_now
                # Solo acumulamos si llegó un mensaje NUEVO (seq distinto)
                if master_msg.seq != last_master_seq:
                    master_pose_buf.append(
                        (master_msg.pos_x, master_msg.pos_y, master_msg.pos_z))
                    last_master_seq = master_msg.seq

                if master_pose_buf:
                    n = len(master_pose_buf)
                    avg_mx = sum(p[0] for p in master_pose_buf) / n
                    avg_my = sum(p[1] for p in master_pose_buf) / n
                    avg_mz = sum(p[2] for p in master_pose_buf) / n
                else:
                    avg_mx, avg_my, avg_mz = master_msg.pos_x, master_msg.pos_y, master_msg.pos_z

                tx = avg_mx + OFFSET[0]
                ty = avg_my + OFFSET[1]
                tz = avg_mz + OFFSET[2]
                ex = tx - pos["x"]
                ey = ty - pos["y"]
                ez = tz - pos["z"]
                e3d = math.sqrt(ex*ex + ey*ey + ez*ez)

                cmd_lr = pid_lr.compute(ex)
                cmd_ud = pid_ud.compute(ey)
                cmd_fb = -pid_fb.compute(ez)
                tello.send_rc_control(cmd_lr, cmd_fb, cmd_ud, 0)

                row.update({
                    "target_x": tx, "target_y": ty, "target_z": tz,
                    "err_x_cm": ex*100, "err_y_cm": ey*100,
                    "err_z_cm": ez*100, "err_3d_cm": e3d*100,
                    "cmd_lr": cmd_lr, "cmd_fb": cmd_fb, "cmd_ud": cmd_ud,
                    "feedback": "ok",
                })
                errors_3d.append(e3d)
                formation_errors.append((ex, ey, ez))

                if ann is not None:
                    cv2.putText(ann, f"SLAVE · Err formación: {e3d*100:.1f} cm",
                                (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                    cv2.putText(ann, f"RX MASTER: {listener.received_count} msg "
                                     f"(age {master_age*1000:.0f}ms)",
                                (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    cv2.putText(ann, f"FORMATION ACTIVE",
                                (10, 115), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            row.update(telemetry_snapshot(tello))
            logger.log(row)

            if ann is not None:
                cv2.imshow("SLAVE (Tello B)", ann)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("[INFO] Abort manual con 'q'."); break

            loop_count += 1
            time.sleep(LOOP_DT)

    except KeyboardInterrupt:
        print("\n[INFO] Interrumpido por usuario.")
    finally:
        listener.stop()
        tello.send_rc_control(0, 0, 0, 0); time.sleep(0.5)
        try: tello.land()
        except Exception: pass
        try: tello.streamoff()
        except Exception: pass
        logger.close()
        cv2.destroyAllWindows()

        if errors_3d and formation_errors:
            elapsed = time.time() - t_start
            cut = max(1, len(errors_3d)//4)
            steady_3d = errors_3d[cut:]
            steady_xyz = formation_errors[cut:]
            ex_m = statistics.mean(e[0] for e in steady_xyz)*100
            ey_m = statistics.mean(e[1] for e in steady_xyz)*100
            ez_m = statistics.mean(e[2] for e in steady_xyz)*100
            ex_s = statistics.stdev(e[0] for e in steady_xyz)*100 if len(steady_xyz) > 1 else 0
            ey_s = statistics.stdev(e[1] for e in steady_xyz)*100 if len(steady_xyz) > 1 else 0
            ez_s = statistics.stdev(e[2] for e in steady_xyz)*100 if len(steady_xyz) > 1 else 0

            print("\n" + "="*64)
            print("RESULTADOS — SLAVE (Tello B), Formación 2.2")
            print("="*64)
            print(f"  Lazo: {loop_count/elapsed:.1f} Hz")
            print(f"  Mensajes recibidos del MASTER: {listener.received_count}  "
                  f"(CRC errors: {listener.crc_errors})")
            print(f"  Error de formación 3D estacionario: "
                  f"{statistics.mean(steady_3d)*100:.2f} cm")
            print(f"  Error de formación 3D máximo:        "
                  f"{max(errors_3d)*100:.2f} cm")
            print(f"  Por eje (mean ± std):")
            print(f"    err_x = {ex_m:+6.2f} ± {ex_s:5.2f} cm")
            print(f"    err_y = {ey_m:+6.2f} ± {ey_s:5.2f} cm")
            print(f"    err_z = {ez_m:+6.2f} ± {ez_s:5.2f} cm")
            print("="*64)
        print("[DONE] Prueba 2.2 SLAVE completada.")


if __name__ == "__main__":
    main()
