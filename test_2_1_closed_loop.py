"""
═══════════════════════════════════════════════════════════════
PRUEBA 2.1 — Lazo cerrado ArUco (un solo dron)
═══════════════════════════════════════════════════════════════
Drones: 1  |  Complejidad: Media  |  Tiempo: ~45 min

Objetivo (plan de pruebas, OE2):
    Verificar que un dron puede mantener una posición fija relativa
    a uno o varios marcadores ArUco usando un controlador PID en
    lazo cerrado basado en la cámara.

Procedimiento (del plan):
    1. Despegar y subir a la altitud de vuelo.
    2. Capturar la posición de hover natural como TARGET_POS.
    3. Activar el lazo cerrado durante CONTROL_DURATION_S segundos.
    4. (Manual) Empujar suavemente al dron con un cartón y observar
       cómo el controlador lo regresa al target.
    5. Registrar error de posición continuo.
    6. Métricas finales: error estac., error máx. transitorio, tiempo
       de recuperación tras perturbación, frecuencia efectiva del lazo.

USO:
    python test_2_1_closed_loop.py
"""

import sys
import os
import time
import math
import statistics

import cv2

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from djitellopy import Tello  # noqa: E402

from utils import ArUcoTracker, FlightLogger, PIDController  # noqa: E402
import config  # noqa: E402

# ----------------------------------------------------------------
# Parámetros del experimento
# ----------------------------------------------------------------
INITIAL_CLIMB_CM   = 60     # Subir tras takeoff para encuadrar el grid
IMU_WAIT_S         = 4.0    # Pausa post-takeoff para que el IMU se estabilice
INIT_HOVER_S       = 5.0    # Hover de captura del target_pos
CONTROL_DURATION_S = 60.0   # Duración del lazo cerrado activo
LOOP_DT            = 0.05   # ~20 Hz de control
LOST_FEEDBACK_HOLD_S = 1.5  # Si no hay ArUco por más tiempo → hover seguro


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
    logger = FlightLogger("test_2_1_closed_loop")

    # PIDs por eje (usan ganancias de config.py)
    pid_lr = PIDController(**config.PID_LR, output_limit=config.RC_MAX)  # X mundo
    pid_ud = PIDController(**config.PID_UD, output_limit=config.RC_MAX)  # Y mundo
    pid_fb = PIDController(**config.PID_FB, output_limit=config.RC_MAX)  # Z mundo (invertido)

    tello.streamon()
    # Esperar primer frame
    t0 = time.time()
    while safe_frame(tello) is None and time.time() - t0 < 5.0:
        time.sleep(0.1)

    tello.takeoff()

    # Pausa para estabilizar IMU
    print(f"[INFO] Esperando {IMU_WAIT_S:.0f} s a que el IMU se estabilice tras takeoff...")
    t0 = time.time()
    while time.time() - t0 < IMU_WAIT_S:
        tello.send_rc_control(0, 0, 0, 0)
        time.sleep(0.05)

    # Subir para encuadrar el grid completo
    if INITIAL_CLIMB_CM > 0:
        print(f"[INFO] Subiendo {INITIAL_CLIMB_CM} cm para encuadrar el grid...")
        try:
            tello.move_up(INITIAL_CLIMB_CM)
        except Exception as e:
            err = str(e)
            print(f"[ERROR] move_up({INITIAL_CLIMB_CM}) falló: {err}")
            if "imu" in err.lower():
                print("[ERROR] IMU inválido — apaga el Tello, espera 30s en superficie plana,")
                print("        enciéndelo y reintenta. ABORTANDO.")
                try: tello.land()
                except Exception: pass
                try: tello.streamoff()
                except Exception: pass
                logger.close(); cv2.destroyAllWindows()
                return

    # ----- Hover inicial: capturar TARGET_POS automáticamente -----
    print(f"[INFO] Hover inicial {INIT_HOVER_S:.0f} s para capturar target_pos...")
    init_samples = []
    t0 = time.time()
    while time.time() - t0 < INIT_HOVER_S:
        tello.send_rc_control(0, 0, 0, 0)
        frame = safe_frame(tello)
        if frame is not None:
            pos, annotated = tracker.detect_and_estimate(frame)
            if pos:
                init_samples.append(pos)
            cv2.putText(annotated, "INIT HOVER (captando target)",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (255, 200, 0), 2)
            cv2.imshow("Tello — Prueba 2.1", annotated)
            cv2.waitKey(1)
        time.sleep(0.05)

    if len(init_samples) < 10:
        print("[ERROR] No hay suficientes detecciones ArUco para fijar target. Aborta.")
        try: tello.land()
        except Exception: pass
        try: tello.streamoff()
        except Exception: pass
        logger.close(); cv2.destroyAllWindows()
        return

    # Promedio del último ~2s de muestras
    tail = init_samples[-int(2.0/0.05):]
    target = {
        "x": statistics.mean(s["x"] for s in tail),
        "y": statistics.mean(s["y"] for s in tail),
        "z": statistics.mean(s["z"] for s in tail),
    }
    print(f"[INFO] target_pos = X={target['x']:.2f}  Y={target['y']:.2f}  "
          f"Z={target['z']:.2f} m")

    # ----- Lazo cerrado de control -----
    print(f"\n[INFO] Activando lazo cerrado por {CONTROL_DURATION_S:.0f} s.")
    print("[INFO] Cuando estable, EMPUJA suavemente al dron con un cartón")
    print("       para verificar que recupera la posición.")
    print("[INFO] Presiona 'q' en la ventana para abortar de forma segura.\n")

    errors_total = []      # error 3D para resumen
    errors_xyz = []        # tuplas (ex, ey, ez) para análisis por eje
    last_pose_t = time.time()
    t_start = time.time()
    loop_count = 0
    perturbation_count = 0
    perturbation_active = 0  # decae a 0 tras unos frames de marcado

    try:
        while time.time() - t_start < CONTROL_DURATION_S:
            t_now = time.time()
            frame = safe_frame(tello)
            pos, annotated = (None, None)
            if frame is not None:
                pos, annotated = tracker.detect_and_estimate(frame)

            row = {
                "timestamp":   t_now,
                "elapsed":     t_now - t_start,
                "target_x":    target["x"],
                "target_y":    target["y"],
                "target_z":    target["z"],
                "pos_x":       pos["x"] if pos else None,
                "pos_y":       pos["y"] if pos else None,
                "pos_z":       pos["z"] if pos else None,
                "marker_id":   pos["marker_id"] if pos else None,
            }

            if pos and (t_now - last_pose_t) < LOST_FEEDBACK_HOLD_S:
                last_pose_t = t_now
                ex = target["x"] - pos["x"]
                ey = target["y"] - pos["y"]
                ez = target["z"] - pos["z"]
                e3d = math.sqrt(ex*ex + ey*ey + ez*ez)

                # PID → comandos rc
                cmd_lr = pid_lr.compute(ex)
                cmd_ud = pid_ud.compute(ey)
                cmd_fb = -pid_fb.compute(ez)   # Z mundo invertido respecto a fb del Tello

                tello.send_rc_control(cmd_lr, cmd_fb, cmd_ud, 0)

                row.update({
                    "err_x_cm":  ex * 100,
                    "err_y_cm":  ey * 100,
                    "err_z_cm":  ez * 100,
                    "err_3d_cm": e3d * 100,
                    "cmd_lr": cmd_lr, "cmd_fb": cmd_fb, "cmd_ud": cmd_ud,
                    "feedback": "ok",
                })
                errors_total.append(e3d)
                errors_xyz.append((ex, ey, ez))

                if annotated is not None:
                    cv2.putText(annotated,
                                f"Err 3D: {e3d*100:5.1f} cm  "
                                f"(x={ex*100:+5.1f}, y={ey*100:+5.1f}, z={ez*100:+5.1f})",
                                (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                (255, 255, 0), 2)
                    cv2.putText(annotated, "CONTROL ACTIVE",
                                (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                (0, 255, 0), 2)
            else:
                # Sin pose reciente → hover seguro, no comandos PID
                tello.send_rc_control(0, 0, 0, 0)
                row.update({
                    "err_x_cm":  None, "err_y_cm": None, "err_z_cm": None,
                    "err_3d_cm": None,
                    "cmd_lr": 0, "cmd_fb": 0, "cmd_ud": 0,
                    "feedback": "lost",
                })
                if annotated is not None:
                    cv2.putText(annotated, "FEEDBACK LOST — HOVER",
                                (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                                (0, 0, 255), 2)

            row.update(telemetry_snapshot(tello))
            row["perturbation"] = perturbation_active
            logger.log(row)
            if perturbation_active > 0:
                perturbation_active = max(0, perturbation_active - 1)

            if annotated is not None:
                if perturbation_active > 0:
                    cv2.putText(annotated, f"PERTURBATION #{perturbation_count}",
                                (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                                (0, 0, 255), 3)
                cv2.imshow("Tello — Prueba 2.1", annotated)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    print("[INFO] Abort manual con 'q'.")
                    break
                elif key == ord('p'):
                    # Marca el frame actual y los siguientes ~10 (~0.5 s)
                    perturbation_count += 1
                    perturbation_active = 10
                    print(f"[PERTURB] #{perturbation_count} marcada en t={t_now-t_start:.2f}s")

            loop_count += 1
            time.sleep(LOOP_DT)

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

        # ----- Resumen -----
        if errors_total:
            elapsed = time.time() - t_start
            loop_hz = loop_count / elapsed if elapsed > 0 else 0
            # Estado estacionario: ignorar primer 25 % (transitorios de captura)
            cut = max(1, len(errors_total) // 4)
            steady_total = errors_total[cut:]
            steady_xyz = errors_xyz[cut:]

            def axis_stats(vals):
                return (statistics.mean(vals)*100,
                        (statistics.stdev(vals)*100 if len(vals)>1 else 0.0))

            ex_mean, ex_std = axis_stats([e[0] for e in steady_xyz])
            ey_mean, ey_std = axis_stats([e[1] for e in steady_xyz])
            ez_mean, ez_std = axis_stats([e[2] for e in steady_xyz])

            print("\n" + "=" * 64)
            print("RESULTADOS — Prueba 2.1 (lazo cerrado, un dron)")
            print("=" * 64)
            print(f"  Muestras válidas:           {len(errors_total)}/{loop_count}")
            print(f"  Frecuencia efectiva lazo:   {loop_hz:.1f} Hz")
            print(f"  Error 3D promedio (todo):   {statistics.mean(errors_total)*100:.1f} cm")
            print(f"  Error 3D promedio (estac.): {statistics.mean(steady_total)*100:.1f} cm")
            print(f"  Error 3D máximo:            {max(errors_total)*100:.1f} cm")
            print()
            print(f"  Estado estacionario por eje (mean ± std):")
            print(f"    err_x: {ex_mean:+6.2f} ± {ex_std:5.2f} cm")
            print(f"    err_y: {ey_mean:+6.2f} ± {ey_std:5.2f} cm")
            print(f"    err_z: {ez_mean:+6.2f} ± {ez_std:5.2f} cm")
            print("=" * 64)
        print("[DONE] Prueba 2.1 completada. Revisa el CSV en logs/.")


if __name__ == "__main__":
    main()
