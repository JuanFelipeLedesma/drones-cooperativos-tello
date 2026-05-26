"""
═══════════════════════════════════════════════════════════════
PRUEBA 1.2 — Latencia comando → acción
═══════════════════════════════════════════════════════════════
Drones: 1  |  Complejidad: Baja  |  Tiempo: ~20 min

Objetivo (plan de pruebas, OE1):
    Medir la distribución estadística del retardo entre el envío
    de un comando UDP al Tello y el inicio del movimiento real,
    para parametrizar el delay del modelo dinámico.

Procedimiento:
    1. Despegar y subir a la altitud de vuelo (encuadre del grid).
    2. Para cada uno de NUM_COMMANDS comandos:
       a) Capturar 1 s de hover para definir posición base.
       b) Enviar el comando con send_command_without_return (no
          bloqueante) y registrar t_send.
       c) Monitorear continuamente la pose ArUco. El primer instante
          donde el desplazamiento desde la base supera 3 cm se toma
          como t_move.
       d) Latencia = t_move − t_send.
    3. Comandos alternados right 20 / left 20 para que el dron oscile
       y no se salga del campo de visión de los markers.
    4. Resumen estadístico: mean, std, min, max, percentiles.

USO:
    python test_1_2_latency.py
"""

import sys
import os
import time
import math
import statistics

import cv2

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from djitellopy import Tello  # noqa: E402

from utils import ArUcoTracker, FlightLogger  # noqa: E402
from utils.pid import PIDController  # noqa: E402
import config  # noqa: E402

# ----------------------------------------------------------------
# Parámetros del experimento
# ----------------------------------------------------------------
NUM_COMMANDS         = 12      # nº de comandos para estadística (suficiente)
INTERVAL_S           = 5.0     # tiempo entre comandos
# El ruido de pose en hover es ±4-5 cm (1σ) según la prueba 1.3.
# Picos de ruido pueden alcanzar 10 cm. Para estar SEGUROS de que es
# movimiento físico real y no ruido, exigimos:
#   (a) desplazamiento > 10 cm
#   (b) sostenido durante ≥ 5 frames consecutivos (~100 ms a 50 Hz)
MOVEMENT_THRESHOLD_M = 0.10
CONSECUTIVE_FRAMES   = 5
MAX_MONITOR_S        = 3.0
PRE_HOVER_S          = 1.5     # más muestras → base más estable
SAMPLE_DT            = 0.020   # ~50 Hz

# Re-centrado por ArUco entre comandos. Sin esto, tras 12 comandos el dron
# acumula 20-40 cm de deriva en X y se sale del campo de visión de los
# markers. Con esto, antes de cada comando el dron vuelve por PID a la
# misma posición de referencia capturada al inicio.
RECENTER_EVERY_N_CMDS = 1      # 1 = recentra antes de cada comando
RECENTER_MAX_S        = 5.0
RECENTER_TOL_M        = 0.10

INITIAL_CLIMB_CM = 60
IMU_WAIT_S       = 4.0

# Comandos alternados right ↔ left para que el dron no se salga del grid
SDK_COMMANDS = ["right", "left"]
STEP_CM      = 20


def safe_frame(tello):
    fr = tello.get_frame_read()
    if fr is None:
        return None
    frame = fr.frame
    if frame is None or frame.size == 0:
        return None
    return frame


def recenter_to_target(tello, tracker, target, *, max_time_s, tol_m):
    """Cierra lazo PID hasta llegar a `target` (dict {x,y,z}) o timeout."""
    pid_x = PIDController(**config.PID_LR, output_limit=config.RC_MAX)
    pid_y = PIDController(**config.PID_UD, output_limit=config.RC_MAX)
    pid_z = PIDController(**config.PID_FB, output_limit=config.RC_MAX)
    t0 = time.time()
    converged_for = 0.0
    last_t = t0
    while time.time() - t0 < max_time_s:
        frame = safe_frame(tello)
        if frame is None:
            tello.send_rc_control(0, 0, 0, 0)
            time.sleep(0.05); converged_for = 0.0; last_t = time.time()
            continue
        pos, annotated = tracker.detect_and_estimate(frame)
        if annotated is not None:
            cv2.putText(annotated, "RECENTER", (10, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 200, 0), 2)
            cv2.imshow("Tello — Prueba 1.2", annotated)
            cv2.waitKey(1)
        if pos is None:
            tello.send_rc_control(0, 0, 0, 0)
            time.sleep(0.05); converged_for = 0.0; last_t = time.time()
            continue
        ex = target["x"] - pos["x"]
        ey = target["y"] - pos["y"]
        ez = target["z"] - pos["z"]
        if max(abs(ex), abs(ey), abs(ez)) < tol_m:
            tello.send_rc_control(0, 0, 0, 0)
            converged_for += time.time() - last_t
            last_t = time.time()
            if converged_for >= 0.4:
                return True
            time.sleep(0.05)
            continue
        else:
            converged_for = 0.0
            last_t = time.time()
        rc_lr = pid_x.compute(ex)
        rc_ud = pid_y.compute(ey)
        rc_fb = -pid_z.compute(ez)
        tello.send_rc_control(rc_lr, rc_fb, rc_ud, 0)
        time.sleep(0.05)
    tello.send_rc_control(0, 0, 0, 0)
    return False


def telemetry_snapshot(tello):
    try:
        return {
            "tello_height_cm": tello.get_height(),
            "tello_battery":   tello.get_battery(),
            "tello_pitch":     tello.get_pitch(),
            "tello_roll":      tello.get_roll(),
            "tello_yaw":       tello.get_yaw(),
            "tello_vgx":       tello.get_speed_x(),
            "tello_vgy":       tello.get_speed_y(),
            "tello_vgz":       tello.get_speed_z(),
        }
    except Exception:
        return {k: None for k in (
            "tello_height_cm","tello_battery","tello_pitch","tello_roll",
            "tello_yaw","tello_vgx","tello_vgy","tello_vgz",
        )}


def main():
    tello = Tello()
    tello.connect()
    bat = tello.get_battery()
    print(f"[INFO] Batería: {bat}%")
    if bat < config.MIN_BATTERY_PCT:
        print("[ERROR] Batería insuficiente.")
        return

    tracker = ArUcoTracker()
    logger = FlightLogger("test_1_2_latency")

    tello.streamon()
    t0 = time.time()
    while safe_frame(tello) is None and time.time() - t0 < 5.0:
        time.sleep(0.1)

    tello.takeoff()

    print(f"[INFO] Esperando {IMU_WAIT_S:.0f} s a que el IMU se estabilice...")
    t0 = time.time()
    while time.time() - t0 < IMU_WAIT_S:
        tello.send_rc_control(0, 0, 0, 0)
        time.sleep(0.05)

    if INITIAL_CLIMB_CM > 0:
        print(f"[INFO] Subiendo {INITIAL_CLIMB_CM} cm...")
        try:
            tello.move_up(INITIAL_CLIMB_CM)
        except Exception as e:
            err = str(e)
            print(f"[ERROR] move_up({INITIAL_CLIMB_CM}) falló: {err}")
            if "imu" in err.lower():
                print("[ERROR] IMU inválido — apaga y reinicia el Tello. ABORTANDO.")
                try: tello.land()
                except Exception: pass
                try: tello.streamoff()
                except Exception: pass
                logger.close(); cv2.destroyAllWindows()
                return

    # Capturar target_pos para el recenter (mismas condiciones iniciales en cada cmd)
    print(f"[INFO] Hover 5 s para capturar target_pos del recenter...")
    init_samples = []
    t_init = time.time()
    while time.time() - t_init < 5.0:
        tello.send_rc_control(0, 0, 0, 0)
        frame = safe_frame(tello)
        if frame is not None:
            pos, ann = tracker.detect_and_estimate(frame)
            if pos: init_samples.append(pos)
            if ann is not None:
                cv2.imshow("Tello — Prueba 1.2", ann); cv2.waitKey(1)
        time.sleep(0.05)
    if len(init_samples) < 10:
        print("[ERROR] No hay suficientes detecciones ArUco para fijar target.")
        try: tello.land()
        except Exception: pass
        try: tello.streamoff()
        except Exception: pass
        logger.close(); cv2.destroyAllWindows(); return
    tail = init_samples[-int(2.0/0.05):]
    target_pos = {
        "x": statistics.mean(s["x"] for s in tail),
        "y": statistics.mean(s["y"] for s in tail),
        "z": statistics.mean(s["z"] for s in tail),
    }
    print(f"[INFO] target_pos = X={target_pos['x']:.2f}  "
          f"Y={target_pos['y']:.2f}  Z={target_pos['z']:.2f}")

    print(f"\n[INFO] Vamos a enviar {NUM_COMMANDS} comandos alternados "
          f"({SDK_COMMANDS[0]}/{SDK_COMMANDS[1]} {STEP_CM} cm) cada {INTERVAL_S} s.")
    print(f"[INFO] Re-centrado por PID antes de cada comando (tol {RECENTER_TOL_M*100:.0f} cm).")
    print("[INFO] Presiona 'q' en la ventana para abortar.\n")

    latencies = []   # latencia por cada comando exitoso (ms)

    try:
        for i in range(1, NUM_COMMANDS + 1):
            sdk_cmd = SDK_COMMANDS[(i - 1) % len(SDK_COMMANDS)]
            print(f"[CMD {i}/{NUM_COMMANDS}] Próximo: {sdk_cmd} {STEP_CM}")

            # ----- Re-centrado por PID antes del comando -----
            if (i - 1) % RECENTER_EVERY_N_CMDS == 0:
                ok = recenter_to_target(tello, tracker, target_pos,
                                        max_time_s=RECENTER_MAX_S,
                                        tol_m=RECENTER_TOL_M)
                print(f"  recenter: {'OK' if ok else 'TIMEOUT'}")

            # ----- Capturar posición base durante PRE_HOVER_S -----
            positions_pre = []
            t_pre = time.time()
            while time.time() - t_pre < PRE_HOVER_S:
                tello.send_rc_control(0, 0, 0, 0)
                frame = safe_frame(tello)
                if frame is not None:
                    pos, annotated = tracker.detect_and_estimate(frame)
                    if pos:
                        positions_pre.append(pos)
                    cv2.putText(annotated,
                                f"PRE-HOVER {i}/{NUM_COMMANDS}  prox: {sdk_cmd} {STEP_CM}",
                                (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                (255, 200, 0), 2)
                    cv2.imshow("Tello — Prueba 1.2", annotated)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        raise KeyboardInterrupt
                time.sleep(SAMPLE_DT)

            if len(positions_pre) < 3:
                print(f"  [WARN] No hay suficientes detecciones ArUco en pre-hover, salto.")
                # No logueo nada (mantenemos esquema CSV consistente)
                time.sleep(INTERVAL_S - PRE_HOVER_S)
                continue

            base_x = statistics.mean(p["x"] for p in positions_pre)
            base_y = statistics.mean(p["y"] for p in positions_pre)
            base_z = statistics.mean(p["z"] for p in positions_pre)

            # ----- Enviar comando NO BLOQUEANTE y registrar t_send -----
            t_send = time.time()
            try:
                tello.send_command_without_return(f"{sdk_cmd} {STEP_CM}")
            except Exception as e:
                print(f"  [WARN] envío falló: {e}")
                continue
            print(f"  → enviado en t={t_send:.4f}")

            # ----- Monitorear hasta detectar movimiento -----
            # t_move se confirma cuando hay CONSECUTIVE_FRAMES seguidos por
            # encima del umbral (evita disparos por ruido de pose). El
            # tiempo registrado es el del PRIMER frame de la racha.
            t_move = None
            t_monitor_start = time.time()
            consec_count = 0
            first_above_t = None
            while time.time() - t_monitor_start < MAX_MONITOR_S:
                t_now = time.time()
                frame = safe_frame(tello)
                if frame is None:
                    time.sleep(SAMPLE_DT)
                    continue
                pos, annotated = tracker.detect_and_estimate(frame)
                row = {
                    "timestamp":    t_now,
                    "command_idx":  i,
                    "sdk_cmd":      sdk_cmd,
                    "t_send":       t_send,
                    "t_since_send": t_now - t_send,
                    "base_x":       base_x, "base_y": base_y, "base_z": base_z,
                    "pos_x":        pos["x"] if pos else None,
                    "pos_y":        pos["y"] if pos else None,
                    "pos_z":        pos["z"] if pos else None,
                    "displacement_m": None,
                    "movement_detected": False,
                    "t_move":       None,
                    "latency_ms":   None,
                }
                if pos:
                    dx = pos["x"] - base_x
                    dy = pos["y"] - base_y
                    dz = pos["z"] - base_z
                    disp = math.sqrt(dx*dx + dy*dy + dz*dz)
                    row["displacement_m"] = disp
                    above = disp > MOVEMENT_THRESHOLD_M
                    row["movement_detected"] = above

                    if above:
                        if consec_count == 0:
                            first_above_t = t_now
                        consec_count += 1
                        if consec_count >= CONSECUTIVE_FRAMES and t_move is None:
                            t_move = first_above_t
                            row["t_move"] = t_move
                            row["latency_ms"] = (t_move - t_send) * 1000.0
                    else:
                        consec_count = 0
                        first_above_t = None
                row.update(telemetry_snapshot(tello))
                logger.log(row)

                if annotated is not None:
                    label = (f"MONITORING t+{t_now-t_send:.2f}s  "
                             f"disp={(row['displacement_m'] or 0)*100:5.1f} cm  "
                             f"(consec={consec_count})")
                    cv2.putText(annotated, label, (10, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                    if t_move is not None:
                        cv2.putText(annotated,
                                    f"MOVE CONFIRMED @ t+{(t_move-t_send)*1000:.0f}ms",
                                    (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                    (0, 255, 0), 2)
                    cv2.imshow("Tello — Prueba 1.2", annotated)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        raise KeyboardInterrupt
                time.sleep(SAMPLE_DT)

                if t_move is not None and (t_now - t_move) > 0.5:
                    break

            if t_move is not None:
                lat_ms = (t_move - t_send) * 1000.0
                latencies.append(lat_ms)
                print(f"  [OK] latencia = {lat_ms:.1f} ms")
            else:
                print(f"  [WARN] no se detectó movimiento dentro de {MAX_MONITOR_S}s")

            # ----- Esperar al siguiente ciclo -----
            wait = INTERVAL_S - (time.time() - t_send)
            if wait > 0:
                t_w = time.time()
                while time.time() - t_w < wait:
                    tello.send_rc_control(0, 0, 0, 0)
                    time.sleep(0.05)

            bat = tello.get_battery()
            if bat is not None and bat < config.MIN_BATTERY_PCT:
                print(f"  [WARN] Batería {bat}% baja, abortando.")
                break

    except KeyboardInterrupt:
        print("\n[INFO] Interrumpido por usuario.")
    finally:
        tello.send_rc_control(0, 0, 0, 0)
        time.sleep(0.5)
        try: tello.land()
        except Exception: pass
        try: tello.streamoff()
        except Exception: pass
        logger.close()
        cv2.destroyAllWindows()

        # ----- Resumen estadístico -----
        if latencies:
            srt = sorted(latencies)
            n = len(srt)
            def pct(p): return srt[max(0, int(p * n) - 1)]
            print("\n" + "=" * 64)
            print("RESULTADOS — Prueba 1.2 (latencia comando → acción)")
            print("=" * 64)
            print(f"  Comandos enviados:        {NUM_COMMANDS}")
            print(f"  Mediciones válidas:       {n}")
            print(f"  Latencia media:           {statistics.mean(latencies):7.1f} ms")
            if n > 1:
                print(f"  Latencia desv. estándar:  {statistics.stdev(latencies):7.1f} ms")
            print(f"  Mínima:                   {min(latencies):7.1f} ms")
            print(f"  Máxima:                   {max(latencies):7.1f} ms")
            print(f"  Mediana (p50):            {pct(0.50):7.1f} ms")
            print(f"  Percentil 90:             {pct(0.90):7.1f} ms")
            print(f"  Percentil 95:             {pct(0.95):7.1f} ms")
            print()
            # Frecuencia máxima viable de control (regla del plan)
            mean_ms = statistics.mean(latencies)
            f_max = 1000.0 / mean_ms
            print(f"  → Frecuencia máxima estimada de lazo cerrado: {f_max:.1f} Hz")
            print(f"    (1 / latencia_media)")
            print("=" * 64)
        print("[DONE] Prueba 1.2 completada. Revisa el CSV en logs/.")


if __name__ == "__main__":
    main()
