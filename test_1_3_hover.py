"""
═══════════════════════════════════════════════════════════════
PRUEBA 1.3 — Estimación de parámetros de hover
═══════════════════════════════════════════════════════════════
Drones: 1  |  Complejidad: Baja  |  Tiempo: ~15 min

Objetivo (plan de pruebas, OE1):
    Medir el drift natural y la varianza de posición del Tello en
    vuelo estacionario para calibrar el modelo linealizado alrededor
    del punto de operación de hover.

Procedimiento:
    1. Despegar y subir a la altitud de vuelo.
    2. Dejar el dron en hover SIN comandos durante HOVER_DURATION_S
       (rc_control 0 0 0 0 cada SAMPLE_DT s).
    3. Registrar continuamente: pose ArUco, altura del barómetro
       SDK, datos del acelerómetro, velocidades.
    4. Análisis post-flight: drift, varianza por eje, FFT para
       frecuencia dominante, comparación baro vs ArUco.

USO:
    python test_1_3_hover.py
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
import config  # noqa: E402

# ----------------------------------------------------------------
# Parámetros del experimento
# ----------------------------------------------------------------
HOVER_DURATION_S = 60.0
SAMPLE_DT        = 0.030     # ~33 Hz
INITIAL_CLIMB_CM = 60
IMU_WAIT_S       = 4.0


def safe_frame(tello):
    fr = tello.get_frame_read()
    if fr is None:
        return None
    frame = fr.frame
    if frame is None or frame.size == 0:
        return None
    return frame


def telemetry_snapshot(tello):
    try:
        return {
            "tello_height_cm":   tello.get_height(),
            "tello_baro_cm":     tello.get_barometer(),
            "tello_battery":     tello.get_battery(),
            "tello_temp_c":      tello.get_temperature(),
            "tello_pitch":       tello.get_pitch(),
            "tello_roll":        tello.get_roll(),
            "tello_yaw":         tello.get_yaw(),
            "tello_vgx":         tello.get_speed_x(),
            "tello_vgy":         tello.get_speed_y(),
            "tello_vgz":         tello.get_speed_z(),
            "tello_agx":         tello.get_acceleration_x(),
            "tello_agy":         tello.get_acceleration_y(),
            "tello_agz":         tello.get_acceleration_z(),
        }
    except Exception:
        return {k: None for k in (
            "tello_height_cm","tello_baro_cm","tello_battery","tello_temp_c",
            "tello_pitch","tello_roll","tello_yaw",
            "tello_vgx","tello_vgy","tello_vgz",
            "tello_agx","tello_agy","tello_agz",
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
    logger = FlightLogger("test_1_3_hover")

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
                print("[ERROR] IMU inválido — reinicia el Tello. ABORTANDO.")
                try: tello.land()
                except Exception: pass
                try: tello.streamoff()
                except Exception: pass
                logger.close(); cv2.destroyAllWindows()
                return

    # Permitir que el dron se asiente tras el climb
    print("[INFO] Esperando 2 s a que se asiente tras el climb...")
    t0 = time.time()
    while time.time() - t0 < 2.0:
        tello.send_rc_control(0, 0, 0, 0)
        time.sleep(0.05)

    print(f"\n[INFO] Grabando hover por {HOVER_DURATION_S:.0f} s SIN comandos.")
    print("[INFO] Presiona 'q' para abortar.\n")

    positions_xyz = []
    baros = []
    aruco_heights = []
    t_start = time.time()
    last_progress = -1

    try:
        while time.time() - t_start < HOVER_DURATION_S:
            t_now = time.time()
            elapsed = t_now - t_start
            tello.send_rc_control(0, 0, 0, 0)

            frame = safe_frame(tello)
            pos, annotated = (None, None)
            if frame is not None:
                pos, annotated = tracker.detect_and_estimate(frame)

            row = {
                "timestamp":  t_now,
                "elapsed":    elapsed,
                "pos_x":      pos["x"] if pos else None,
                "pos_y":      pos["y"] if pos else None,
                "pos_z":      pos["z"] if pos else None,
                "marker_id":  pos["marker_id"] if pos else None,
            }
            row.update(telemetry_snapshot(tello))
            logger.log(row)

            if pos:
                positions_xyz.append((pos["x"], pos["y"], pos["z"]))
                aruco_heights.append(pos["y"])
            if row.get("tello_baro_cm") is not None:
                baros.append(row["tello_baro_cm"] / 100.0)  # cm → m

            # Progreso cada 10 s
            sec = int(elapsed)
            if sec != last_progress and sec % 10 == 0 and sec > 0:
                print(f"  [{sec}/{int(HOVER_DURATION_S)}s] "
                      f"bat={tello.get_battery()}%  detecciones={len(positions_xyz)}")
                last_progress = sec

            if annotated is not None:
                cv2.putText(annotated, f"HOVER {elapsed:.1f}/{HOVER_DURATION_S:.0f} s",
                            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                            (255, 255, 0), 2)
                cv2.imshow("Tello — Prueba 1.3", annotated)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            time.sleep(SAMPLE_DT)

    except KeyboardInterrupt:
        print("\n[INFO] Interrumpido.")
    finally:
        tello.send_rc_control(0, 0, 0, 0)
        time.sleep(0.5)
        try: tello.land()
        except Exception: pass
        try: tello.streamoff()
        except Exception: pass
        logger.close()
        cv2.destroyAllWindows()

        # ----- Análisis -----
        if len(positions_xyz) > 10:
            xs = [p[0] for p in positions_xyz]
            ys = [p[1] for p in positions_xyz]
            zs = [p[2] for p in positions_xyz]

            # Drift respecto a la primera muestra
            drift_xy = [math.sqrt((x - xs[0])**2 + (y - ys[0])**2)
                        for x, y in zip(xs, ys)]
            drift_3d = [math.sqrt((x - xs[0])**2 + (y - ys[0])**2 + (z - zs[0])**2)
                        for x, y, z in zip(xs, ys, zs)]

            # Diferencia barómetro vs ArUco
            baro_diff = None
            if len(baros) > 5 and len(aruco_heights) > 5:
                m_baro = statistics.mean(baros)
                m_aruco = statistics.mean(aruco_heights)
                baro_diff = (m_baro - m_aruco) * 100  # cm

            # Frecuencia dominante (FFT) en eje Y, si numpy disponible
            dom_freq_hz = None
            try:
                import numpy as np
                ts = [i * SAMPLE_DT for i in range(len(ys))]
                ys_arr = np.array(ys) - np.mean(ys)
                if len(ys_arr) > 32:
                    freqs = np.fft.rfftfreq(len(ys_arr), d=SAMPLE_DT)
                    fft_mag = np.abs(np.fft.rfft(ys_arr))
                    # ignorar DC (idx 0) y frecuencias > 5 Hz
                    mask = (freqs > 0.05) & (freqs < 5.0)
                    if mask.any():
                        peak_idx = np.argmax(fft_mag[mask])
                        dom_freq_hz = float(freqs[mask][peak_idx])
            except Exception:
                pass

            print("\n" + "=" * 64)
            print("RESULTADOS — Prueba 1.3 (hover natural)")
            print("=" * 64)
            print(f"  Muestras con ArUco:       {len(positions_xyz)} en {HOVER_DURATION_S:.0f} s")
            print(f"  Posición media (x,y,z):   ({statistics.mean(xs):.2f}, "
                  f"{statistics.mean(ys):.2f}, {statistics.mean(zs):.2f}) m")
            print()
            print(f"  Desviación estándar:")
            print(f"    σ_x = {statistics.stdev(xs)*100:6.2f} cm")
            print(f"    σ_y = {statistics.stdev(ys)*100:6.2f} cm")
            print(f"    σ_z = {statistics.stdev(zs)*100:6.2f} cm")
            print()
            print(f"  Drift máximo XY:          {max(drift_xy)*100:6.1f} cm")
            print(f"  Drift máximo 3D:          {max(drift_3d)*100:6.1f} cm")
            print()
            print(f"  Rango por eje:")
            print(f"    X: {(max(xs)-min(xs))*100:5.1f} cm  "
                  f"[{min(xs):.2f}, {max(xs):.2f}] m")
            print(f"    Y: {(max(ys)-min(ys))*100:5.1f} cm  "
                  f"[{min(ys):.2f}, {max(ys):.2f}] m")
            print(f"    Z: {(max(zs)-min(zs))*100:5.1f} cm  "
                  f"[{min(zs):.2f}, {max(zs):.2f}] m")
            if baro_diff is not None:
                print()
                print(f"  Comparación altura:")
                print(f"    Baro media:  {statistics.mean(baros):.2f} m")
                print(f"    ArUco media: {statistics.mean(aruco_heights):.2f} m")
                print(f"    Diferencia (baro − aruco): {baro_diff:+.1f} cm")
            if dom_freq_hz is not None:
                print()
                print(f"  Frecuencia dominante de oscilación (FFT eje Y): "
                      f"{dom_freq_hz:.2f} Hz")
            print("=" * 64)
        print("[DONE] Prueba 1.3 completada. Revisa el CSV en logs/.")


if __name__ == "__main__":
    main()
