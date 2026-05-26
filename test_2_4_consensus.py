"""
═══════════════════════════════════════════════════════════════
PRUEBA 2.4 — Consenso de posición distribuido (sin líder)
═══════════════════════════════════════════════════════════════
Drones: 2  |  Complejidad: Alta  |  Tiempo: ~3 min vuelo

Ley de control simétrica en cada dron i:
    target_i(t) = (pos_i(t) + pos_j_filtrado(t)) / 2

Donde pos_j es la posición del OTRO dron (recibida por Ethernet).
Como ambos drones aplican la misma ley, convergen al promedio
(centroide geométrico de las posiciones iniciales).

CONTROL POR EJE:
    - X y Z (horizontal): consenso activo
    - Y (vertical):       cada dron mantiene su altura individual
                          (evita que se persigan en altura)

USO:
    Mac (drone A, IP 192.168.1.1):
        python test_2_4_consensus.py --id 1
    Ubuntu (drone B, IP 192.168.1.2):
        python test_2_4_consensus.py --id 2

ABORT: 'q' o Ctrl+C en cualquiera de los dos.
"""
import sys
import os
import time
import math
import socket
import statistics
import threading
import argparse
from collections import deque

import cv2

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from djitellopy import Tello  # noqa: E402

from utils import ArUcoTracker, FlightLogger  # noqa: E402
from utils.pid import PIDController  # noqa: E402
import config  # noqa: E402

from test_3_2_protocol import CoopMessage, encode_binary, decode_binary, BINARY_SIZE

# ----------------------------------------------------------------
# Parámetros
# ----------------------------------------------------------------
INITIAL_CLIMB_CM   = 60
IMU_WAIT_S         = 4.0
INIT_HOVER_S       = 5.0
WAIT_PEER_S        = 30.0           # esperar al peer antes de despegar
MISSION_DURATION_S = 45.0           # consenso activo
LOOP_DT            = 0.05
PUBLISH_HZ         = 50

# Filtro promedio móvil sobre la pose del peer (igual que 2.2/2.3)
PEER_POSE_FILTER_N = 8

# Banda muerta: si error 3D < esto, no enviamos comandos.
# Evita oscilación cerca del consenso y reduce riesgo de colisión.
DEAD_ZONE_M = 0.08

# ----- Separación mínima de seguridad (CRÍTICO) -----
# El consenso clásico converge ambos drones al MISMO punto X-Z, lo que
# físicamente los apila verticalmente y causa colisión por downwash.
# Modificamos la ley para que cada dron converja a un punto OFFSET del
# centro: el id=1 va a la izquierda, el id=2 a la derecha. Sigue siendo
# consenso (ambos convergen a un punto función de los dos), pero con
# separación lateral garantizada.
MIN_SEPARATION_X_M = 0.60   # 60 cm de separación horizontal en estado estacionario

# Timeout sin mensajes del peer → hover seguro
LOST_TIMEOUT_S = config.COMMS_TIMEOUT_S   # 5 s

PORT = config.COMMS_PORT
MISSION_STATE_CONSENSUS = 5
MISSION_STATE_LANDING   = 6


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
# Comunicador bidireccional: publica al peer y escucha al peer
# ============================================================
class PeerComms:
    """
    Maneja comunicación bidireccional con el otro dron.
    - Publica `latest_local` al peer a PUBLISH_HZ.
    - Escucha mensajes del peer en el mismo socket.
    """
    def __init__(self, my_id, peer_addr, listen_port, hz):
        self._my_id = my_id
        self._peer_addr = peer_addr
        self._interval = 1.0 / hz

        # Un mismo socket UDP envía y escucha
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(("0.0.0.0", listen_port))
        self._sock.settimeout(0.05)

        self._lock = threading.Lock()
        self._latest_local = None     # tupla (pos, mission_state, battery, vel)
        self._latest_peer = None       # CoopMessage del peer
        self._last_peer_t = 0.0
        self._stop = threading.Event()
        self._tx_thread = None
        self._rx_thread = None
        self._seq = 0
        self.tx_count = 0
        self.rx_count = 0
        self.crc_errors = 0

    def update_local(self, pos, mission_state, battery, vel=None):
        with self._lock:
            self._latest_local = (pos, mission_state, battery, vel or (0.0, 0.0, 0.0))

    def get_peer(self):
        with self._lock:
            return self._latest_peer, self._last_peer_t

    def _tx_loop(self):
        next_t = time.time()
        while not self._stop.is_set():
            now = time.time()
            if now >= next_t:
                with self._lock:
                    snap = self._latest_local
                if snap is not None:
                    pos, ms, bat, vel = snap
                    msg = CoopMessage(
                        drone_id=self._my_id, seq=self._seq, timestamp=time.time(),
                        pos_x=pos["x"], pos_y=pos["y"], pos_z=pos["z"],
                        vel_x=vel[0], vel_y=vel[1], vel_z=vel[2],
                        battery=int(bat) if bat else 0, mission_state=ms,
                    )
                    try:
                        self._sock.sendto(encode_binary(msg), self._peer_addr)
                        self.tx_count += 1
                        self._seq = (self._seq + 1) & 0xFFFFFFFF
                    except Exception:
                        pass
                next_t += self._interval
            else:
                time.sleep(min(0.005, next_t - now))

    def _rx_loop(self):
        while not self._stop.is_set():
            try:
                data, _ = self._sock.recvfrom(4096)
                if len(data) != BINARY_SIZE:
                    continue
                msg, crc_ok = decode_binary(data)
                if not crc_ok:
                    self.crc_errors += 1
                    continue
                if msg.drone_id == self._my_id:
                    continue   # ignorar mis propios mensajes (loopback)
                with self._lock:
                    self._latest_peer = msg
                    self._last_peer_t = time.time()
                    self.rx_count += 1
            except socket.timeout:
                continue
            except Exception:
                continue

    def start(self):
        self._tx_thread = threading.Thread(target=self._tx_loop, daemon=True)
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._tx_thread.start()
        self._rx_thread.start()

    def send_termination(self):
        with self._lock:
            snap = self._latest_local
        if snap is None: return
        pos, _, bat, vel = snap
        msg = CoopMessage(
            drone_id=self._my_id, seq=self._seq, timestamp=time.time(),
            pos_x=pos["x"], pos_y=pos["y"], pos_z=pos["z"],
            vel_x=vel[0], vel_y=vel[1], vel_z=vel[2],
            battery=int(bat) if bat else 0,
            mission_state=MISSION_STATE_LANDING,
        )
        try:
            for _ in range(5):
                self._sock.sendto(encode_binary(msg), self._peer_addr)
                time.sleep(0.02)
        except Exception:
            pass

    def stop(self):
        self._stop.set()
        if self._tx_thread: self._tx_thread.join(timeout=1.0)
        if self._rx_thread: self._rx_thread.join(timeout=1.0)
        self._sock.close()


# ============================================================
# Main
# ============================================================
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--id", type=int, choices=[1, 2], required=True,
                        help="ID del dron (1=Mac/A, 2=Ubuntu/B)")
    args = parser.parse_args()

    my_id = args.id
    if my_id == 1:
        peer_ip = config.SLAVE_IP    # Mac → Ubuntu
        my_label = "DRONE A (Mac)"
        peer_label = "DRONE B (Ubuntu)"
    else:
        peer_ip = config.MASTER_IP   # Ubuntu → Mac
        my_label = "DRONE B (Ubuntu)"
        peer_label = "DRONE A (Mac)"

    peer_addr = (peer_ip, PORT)
    print(f"[INFO] Yo soy {my_label} (id={my_id})")
    print(f"[INFO] Peer: {peer_label} en {peer_addr}")

    tello = Tello()
    tello.connect()
    bat = tello.get_battery()
    print(f"[INFO] Batería: {bat}%")
    if bat < config.MIN_BATTERY_PCT:
        print("[ERROR] Batería insuficiente."); return

    tracker = ArUcoTracker()
    logger = FlightLogger(f"test_2_4_consensus_id{my_id}")
    pid_lr = PIDController(**config.PID_LR, output_limit=config.RC_MAX)
    pid_ud = PIDController(**config.PID_UD, output_limit=config.RC_MAX)
    pid_fb = PIDController(**config.PID_FB, output_limit=config.RC_MAX)

    comms = PeerComms(my_id=my_id, peer_addr=peer_addr,
                      listen_port=PORT, hz=PUBLISH_HZ)
    comms.start()
    print(f"[INFO] Comms bidireccionales activas en puerto {PORT}.")

    tello.streamon()
    t0 = time.time()
    while safe_frame(tello) is None and time.time() - t0 < 5.0: time.sleep(0.1)

    tello.takeoff()
    print(f"[INFO] IMU wait {IMU_WAIT_S:.0f} s...")
    t0 = time.time()
    while time.time() - t0 < IMU_WAIT_S:
        tello.send_rc_control(0, 0, 0, 0); time.sleep(0.05)

    if INITIAL_CLIMB_CM > 0:
        print(f"[INFO] Subiendo {INITIAL_CLIMB_CM} cm...")
        try: tello.move_up(INITIAL_CLIMB_CM)
        except Exception as e:
            err = str(e)
            print(f"[ERROR] move_up: {err}")
            if "imu" in err.lower():
                try: tello.land(); tello.streamoff()
                except: pass
                comms.stop(); logger.close(); return

    # ----- Init hover: capturar pose inicial propia -----
    print(f"[INFO] Hover inicial {INIT_HOVER_S:.0f} s...")
    init_samples = []
    t0 = time.time()
    while time.time() - t0 < INIT_HOVER_S:
        tello.send_rc_control(0, 0, 0, 0)
        f = safe_frame(tello)
        if f is not None:
            pos, ann = tracker.detect_and_estimate(f)
            if pos:
                init_samples.append(pos)
                # publicar ya nuestra pose para que el peer la vea
                comms.update_local(pos, MISSION_STATE_CONSENSUS,
                                   tello.get_battery(), vel=(0,0,0))
            if ann is not None:
                cv2.putText(ann, f"{my_label} · init hover", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 0), 2)
                cv2.imshow(my_label, ann); cv2.waitKey(1)
        time.sleep(0.05)

    if len(init_samples) < 10:
        print("[ERROR] Pocas detecciones ArUco en init hover.")
        try: tello.land(); tello.streamoff()
        except: pass
        comms.stop(); logger.close(); cv2.destroyAllWindows(); return

    tail = init_samples[-int(2.0/0.05):]
    my_init_pos = {
        "x": statistics.mean(s["x"] for s in tail),
        "y": statistics.mean(s["y"] for s in tail),
        "z": statistics.mean(s["z"] for s in tail),
    }
    # Y SE QUEDA fija en el hover individual (no consenso vertical)
    target_y = my_init_pos["y"]
    print(f"[INFO] Pose inicial: X={my_init_pos['x']:.2f} "
          f"Y={my_init_pos['y']:.2f} Z={my_init_pos['z']:.2f}")
    print(f"[INFO] Y mantenida en {target_y:.2f} m (control vertical individual)")

    # ----- Esperar al peer (debe estar despegando o ya volando) -----
    print(f"\n[INFO] Esperando primer mensaje del peer (timeout {WAIT_PEER_S:.0f} s)...")
    t0 = time.time()
    peer_first = None
    while time.time() - t0 < WAIT_PEER_S:
        peer_msg, _ = comms.get_peer()
        if peer_msg is not None:
            peer_first = peer_msg
            print(f"[INFO] ✓ Peer detectado en X={peer_msg.pos_x:.2f} "
                  f"Y={peer_msg.pos_y:.2f} Z={peer_msg.pos_z:.2f}")
            break
        # Mantener hover y publicar nuestra pose
        f = safe_frame(tello)
        if f is not None:
            pos, ann = tracker.detect_and_estimate(f)
            if pos:
                comms.update_local(pos, MISSION_STATE_CONSENSUS,
                                   tello.get_battery(), vel=(0,0,0))
            if ann is not None:
                cv2.putText(ann, f"{my_label} · esperando peer ({time.time()-t0:.0f}s)",
                            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 2)
                cv2.imshow(my_label, ann); cv2.waitKey(1)
        tello.send_rc_control(0, 0, 0, 0)
        time.sleep(0.05)

    if peer_first is None:
        print("[ERROR] Peer no respondió. Aborto.")
        try: tello.land(); tello.streamoff()
        except: pass
        comms.stop(); logger.close(); cv2.destroyAllWindows(); return

    # ----- Lazo de consenso -----
    print(f"\n[INFO] Lazo de consenso activo por {MISSION_DURATION_S:.0f} s.")
    print(f"[INFO] Control: X+Z consenso (filtro N={PEER_POSE_FILTER_N}), "
          f"Y individual.")
    print(f"[INFO] Banda muerta: {DEAD_ZONE_M*100:.0f} cm.\n")

    peer_buf = deque(maxlen=PEER_POSE_FILTER_N)
    last_peer_seq = -1
    t_start = time.time()
    last_pose_t = t_start
    consensus_errors = []   # distancia entre los dos drones (X-Z)
    consensus_to_mid = []   # distancia mía al punto medio
    loop_count = 0

    try:
        while time.time() - t_start < MISSION_DURATION_S:
            t_now = time.time()
            f = safe_frame(tello)
            pos, ann = (None, None)
            if f is not None: pos, ann = tracker.detect_and_estimate(f)

            peer_msg, peer_t = comms.get_peer()
            peer_age = t_now - peer_t if peer_t else 999

            row = {
                "timestamp": t_now, "elapsed": t_now - t_start,
                "my_id": my_id,
                "peer_age_s": peer_age,
                "peer_seq": peer_msg.seq if peer_msg else None,
                "peer_pos_x": peer_msg.pos_x if peer_msg else None,
                "peer_pos_y": peer_msg.pos_y if peer_msg else None,
                "peer_pos_z": peer_msg.pos_z if peer_msg else None,
                "peer_state": peer_msg.mission_state if peer_msg else None,
                "pos_x": pos["x"] if pos else None,
                "pos_y": pos["y"] if pos else None,
                "pos_z": pos["z"] if pos else None,
                "marker_id": pos["marker_id"] if pos else None,
                "tx_count": comms.tx_count, "rx_count": comms.rx_count,
                "crc_errors": comms.crc_errors,
            }

            # Si el peer dice "landing", aterrizamos
            if peer_msg and peer_msg.mission_state == MISSION_STATE_LANDING:
                print("[INFO] Peer notificó landing. Aterrizando.")
                break

            if peer_msg is None or peer_age > LOST_TIMEOUT_S:
                tello.send_rc_control(0, 0, 0, 0)
                row.update({"target_x": None, "target_z": None,
                            "err_x_cm": None, "err_y_cm": None, "err_z_cm": None,
                            "err_3d_cm": None, "cmd_lr": 0, "cmd_fb": 0, "cmd_ud": 0,
                            "feedback": "lost_peer"})
                if ann is not None:
                    cv2.putText(ann, f"PEER LOST ({peer_age:.1f}s) — HOVER",
                                (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                                (0, 0, 255), 2)
            elif pos is None or (t_now - last_pose_t) > 1.5:
                tello.send_rc_control(0, 0, 0, 0)
                row.update({"target_x": None, "target_z": None,
                            "err_x_cm": None, "err_y_cm": None, "err_z_cm": None,
                            "err_3d_cm": None, "cmd_lr": 0, "cmd_fb": 0, "cmd_ud": 0,
                            "feedback": "lost_self"})
                if ann is not None:
                    cv2.putText(ann, "Sin pose ArUco propia — HOVER",
                                (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                                (0, 0, 255), 2)
            else:
                # Caso normal: consenso activo
                last_pose_t = t_now

                # Acumular pose del peer si es seq nueva
                if peer_msg.seq != last_peer_seq:
                    peer_buf.append((peer_msg.pos_x, peer_msg.pos_y, peer_msg.pos_z))
                    last_peer_seq = peer_msg.seq

                if peer_buf:
                    n = len(peer_buf)
                    avg_px = sum(p[0] for p in peer_buf) / n
                    avg_py = sum(p[1] for p in peer_buf) / n  # no usado
                    avg_pz = sum(p[2] for p in peer_buf) / n
                else:
                    avg_px = peer_msg.pos_x
                    avg_pz = peer_msg.pos_z

                # Ley de consenso CON SEPARACIÓN MÍNIMA: ambos convergen al
                # punto medio X-Z, pero con un offset lateral fijo según
                # su id. Esto evita que se apilen verticalmente y choquen.
                center_x = (pos["x"] + avg_px) / 2.0
                center_z = (pos["z"] + avg_pz) / 2.0
                # id=1 (Drone A, Mac) → izquierda del centro
                # id=2 (Drone B, Ubuntu) → derecha del centro
                lateral_offset = -MIN_SEPARATION_X_M / 2.0 if my_id == 1 else +MIN_SEPARATION_X_M / 2.0
                target_x = center_x + lateral_offset
                target_z = center_z

                ex = target_x - pos["x"]
                ey = target_y  - pos["y"]   # Y individual (estabilidad)
                ez = target_z - pos["z"]
                e3d = math.sqrt(ex*ex + ey*ey + ez*ez)

                # Distancia entre drones (X-Z) — métrica clave del plan
                dist_to_peer = math.sqrt(
                    (pos["x"] - avg_px)**2 + (pos["z"] - avg_pz)**2)
                # Distancia mía al punto medio teórico (= target)
                dist_to_mid = math.sqrt(ex*ex + ez*ez)

                # Banda muerta: si error 3D < DEAD_ZONE_M, no movemos en X-Z
                # (sí mantenemos altitud)
                if e3d < DEAD_ZONE_M:
                    cmd_lr, cmd_fb = 0, 0
                else:
                    cmd_lr = pid_lr.compute(ex)
                    cmd_fb = -pid_fb.compute(ez)
                cmd_ud = pid_ud.compute(ey)
                tello.send_rc_control(cmd_lr, cmd_fb, cmd_ud, 0)

                # Publicar mi pose al peer
                comms.update_local(pos, MISSION_STATE_CONSENSUS,
                                   tello.get_battery(), vel=(0,0,0))

                row.update({
                    "target_x": target_x, "target_z": target_z,
                    "err_x_cm": ex*100, "err_y_cm": ey*100,
                    "err_z_cm": ez*100, "err_3d_cm": e3d*100,
                    "dist_to_peer_cm": dist_to_peer*100,
                    "dist_to_mid_cm": dist_to_mid*100,
                    "cmd_lr": cmd_lr, "cmd_fb": cmd_fb, "cmd_ud": cmd_ud,
                    "feedback": "ok",
                })
                consensus_errors.append(dist_to_peer)
                consensus_to_mid.append(dist_to_mid)

                if ann is not None:
                    cv2.putText(ann, f"{my_label} · consenso",
                                (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                                (255, 255, 0), 2)
                    cv2.putText(ann, f"dist peer: {dist_to_peer*100:.0f} cm  "
                                     f"to_mid: {dist_to_mid*100:.0f} cm",
                                (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (0, 255, 255), 2)
                    cv2.putText(ann, f"TX:{comms.tx_count} RX:{comms.rx_count}",
                                (10, 115), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (0, 255, 0), 2)

            row.update(telemetry_snapshot(tello))
            logger.log(row)

            if ann is not None:
                cv2.imshow(my_label, ann)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("[INFO] Abort manual con 'q'."); break

            loop_count += 1
            time.sleep(LOOP_DT)

    except KeyboardInterrupt:
        print("\n[INFO] Interrumpido por usuario.")
    finally:
        # Avisar al peer y aterrizar limpio
        with comms._lock:
            snap = comms._latest_local
        if snap is not None:
            comms.update_local(snap[0], MISSION_STATE_LANDING,
                               tello.get_battery(), vel=(0,0,0))
        comms.send_termination()
        comms.stop()

        tello.send_rc_control(0, 0, 0, 0); time.sleep(0.5)
        try: tello.land()
        except Exception: pass
        try: tello.streamoff()
        except Exception: pass
        logger.close(); cv2.destroyAllWindows()

        if consensus_errors:
            elapsed = time.time() - t_start
            cut = max(1, len(consensus_errors)//4)
            steady_dist_peer = consensus_errors[cut:]
            steady_dist_mid = consensus_to_mid[cut:]

            # Tiempo de convergencia (cuándo dist_to_peer baja a <15 cm sostenida)
            conv_t = None
            window_n = 15
            for i in range(len(consensus_errors) - window_n):
                if all(consensus_errors[i+k] < 0.15 for k in range(window_n)):
                    conv_t = i * LOOP_DT
                    break

            print("\n" + "="*64)
            print(f"RESULTADOS — {my_label} · Consenso 2.4")
            print("="*64)
            print(f"  Lazo: {loop_count/elapsed:.1f} Hz")
            print(f"  TX: {comms.tx_count}  RX: {comms.rx_count}  "
                  f"CRC errors: {comms.crc_errors}")
            print(f"  Distancia inter-dron (X-Z) inicial: "
                  f"{consensus_errors[0]*100:.1f} cm")
            print(f"  Distancia inter-dron estado estac.: "
                  f"{statistics.mean(steady_dist_peer)*100:.2f} cm "
                  f"± {statistics.stdev(steady_dist_peer)*100 if len(steady_dist_peer)>1 else 0:.2f}")
            print(f"  Distancia mía al punto medio est.:  "
                  f"{statistics.mean(steady_dist_mid)*100:.2f} cm")
            print(f"  Tiempo de convergencia (<15 cm):    "
                  f"{conv_t:.2f} s" if conv_t else "  No convergió a <15 cm")
            print("="*64)


if __name__ == "__main__":
    main()
