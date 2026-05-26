"""
═══════════════════════════════════════════════════════════════
PRUEBA 1.1 — Respuesta escalón individual
═══════════════════════════════════════════════════════════════
Drones: 1  |  Complejidad: Baja  |  Tiempo: ~30 min

Objetivo (plan de pruebas, OE1):
    Caracterizar la respuesta dinámica del Tello ante comandos
    discretos y compararla con la respuesta simulada del modelo
    matemático. Métricas: RMSE de posición, tiempo de respuesta,
    overshoot y settling time al 2%.

Setup físico asumido:
    - Pared con 4 markers ArUco (IDs 0..3): fila baja a Y=0.6 m,
      fila alta a Y=1.0 m, columnas separadas 1.0 m.
    - El dron se posiciona a ~1.5 m frente a la pared, mirándola.
    - Coordenadas mundo: X horizontal sobre la pared, Y vertical,
      Z perpendicular a la pared (positivo alejándose).

Estrategia experimental:
    Para cada eje (X derecha, Y arriba, X izquierda, Y abajo) se
    aplica un escalón de 30 cm con `move_*` (entrada tipo step en
    posición), se graba video + telemetría durante varios segundos
    y se aplica el comando inverso para regresar al punto de
    referencia antes del siguiente escalón. Cada escalón se repite
    REPEATS veces para promediar.

USO:
    python test_1_1_step_response.py
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
# Cada tupla: (nombre, método del SDK, distancia [cm], método inverso para regresar)
# Mantenemos pasos modestos (30 cm) para no perder los markers de vista
# y respetar el área frente a la pared (markers a 1 m entre sí).
STEP_CM = 30
# Cada tupla: (nombre, sdk_cmd_step, distancia, sdk_cmd_inverso, eje_mundo, signo_step)
# sdk_cmd_step se envía con send_command_without_return() (NO bloqueante)
# para grabar la respuesta desde t_send. El inverso se envía BLOQUEANTE
# (move_*) para asegurar que termina antes del recovery.
COMMANDS = [
    ("right", "right", STEP_CM, "move_left",  "x",  1),
    ("up",    "up",    STEP_CM, "move_down",  "y",  1),
    ("left",  "left",  STEP_CM, "move_right", "x", -1),
    ("down",  "down",  STEP_CM, "move_up",    "y", -1),
]

REPEATS = 3                # Repeticiones por comando (plan: ≥3)
PRE_HOVER_S = 3.0          # Hover de referencia antes del escalón
POST_CMD_RECORD_S = 10.0   # Grabación tras el comando (cubre rise+settling con margen)
RETURN_RECOVERY_S = 1.0    # Hover corto tras el inverso (re-center hace el trabajo real)
SAMPLE_DT = 0.03           # ~33 Hz de muestreo en los lazos de grabación

# ---- Re-centrado por lazo cerrado ArUco (entre comandos) -------
# Sin esto, los pares move_X(30) + move_inv(30) acumulan 5-10 cm de error
# por par y tras una ronda completa el dron deriva 20-40 cm fuera del
# campo de visión de los markers. Con esto, antes de cada step el dron
# vuelve por PID a la misma posición de referencia (capturada en init_hover).
RECENTER_MAX_S = 6.0       # Tiempo máximo del lazo de re-centrado [s]
RECENTER_TOL_M = 0.07      # Tolerancia para considerar convergido [m]

# Tello despega a ~0.8 m. Su cámara está angulada un pelín hacia abajo, así
# que a esa altura sólo ve la fila baja de markers. Subimos a ~1.3 m para
# encuadrar ambas filas (bajos a Y=0.4 m, altos a Y=1.4 m).
INITIAL_CLIMB_CM = 60


def safe_frame(tello):
    """Lee un frame; devuelve None si aún no hay imagen."""
    fr = tello.get_frame_read()
    if fr is None:
        return None
    frame = fr.frame
    if frame is None or frame.size == 0:
        return None
    return frame


def telemetry_snapshot(tello):
    """Telemetría barata del SDK para cada fila del CSV."""
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
        # Si el SDK responde tarde, no bloqueamos el bucle de grabación.
        return {
            "tello_height_cm":   None, "tello_baro_cm":   None,
            "tello_battery":     None, "tello_temp_c":    None,
            "tello_pitch":       None, "tello_roll":      None,
            "tello_yaw":         None,
            "tello_vgx":         None, "tello_vgy":       None, "tello_vgz": None,
            "tello_agx":         None, "tello_agy":       None, "tello_agz": None,
        }


def log_row(logger, tello, tracker, *, phase, cmd_name, axis, repeat,
            t_send, t_now=None):
    """Graba una fila uniforme: pose ArUco + telemetría + metadatos."""
    if t_now is None:
        t_now = time.time()
    frame = safe_frame(tello)
    pos, annotated = (None, None)
    if frame is not None:
        pos, annotated = tracker.detect_and_estimate(frame)

    row = {
        "timestamp":     t_now,
        "phase":         phase,
        "command":       cmd_name,
        "axis":          axis,
        "repeat":        repeat,
        "t_send":        t_send,
        "t_since_cmd":   None if t_send is None else (t_now - t_send),
        "marker_id":     pos["marker_id"] if pos else None,
        "n_markers":     pos.get("n_markers") if pos else None,
        "pos_x":         pos["x"] if pos else None,
        "pos_y":         pos["y"] if pos else None,
        "pos_z":         pos["z"] if pos else None,
        "marker_dist":   pos["distance"] if pos else None,
    }
    row.update(telemetry_snapshot(tello))
    logger.log(row)

    if annotated is not None:
        cv2.imshow("Tello — Prueba 1.1", annotated)
        cv2.waitKey(1)
    return pos


def hover_and_record(logger, tello, tracker, *, duration, phase, cmd_name,
                     axis, repeat, t_send=None):
    """Mantiene RC=0 (hover), graba y devuelve la lista de samples."""
    samples = []
    t_start = time.time()
    while time.time() - t_start < duration:
        tello.send_rc_control(0, 0, 0, 0)
        pos = log_row(logger, tello, tracker,
                      phase=phase, cmd_name=cmd_name, axis=axis,
                      repeat=repeat, t_send=t_send)
        if pos:
            samples.append({"t": time.time(),
                            "x": pos["x"], "y": pos["y"], "z": pos["z"],
                            "marker_id": pos["marker_id"]})
        time.sleep(SAMPLE_DT)
    return samples


def record_response(logger, tello, tracker, *, duration, cmd_name,
                    axis, repeat, t_send):
    """Graba respuesta tras el escalón (sin RC) y devuelve la lista de samples."""
    samples = []
    t_start = time.time()
    while time.time() - t_start < duration:
        pos = log_row(logger, tello, tracker,
                      phase="response", cmd_name=cmd_name, axis=axis,
                      repeat=repeat, t_send=t_send)
        if pos:
            samples.append({"t": time.time(),
                            "x": pos["x"], "y": pos["y"], "z": pos["z"],
                            "marker_id": pos["marker_id"]})
        time.sleep(SAMPLE_DT)
    return samples


def recenter_to_target(tello, tracker, target, *, max_time_s, tol_m,
                       logger=None, cmd_name="recenter", repeat=0):
    """
    Cierra el lazo PID en X, Y, Z (mundo) usando ArUco hasta llegar al
    target o hasta que se agote `max_time_s`. Devuelve True si convergió.

    target: dict {x, y, z} en metros (coords mundo).
    Mapeo a comandos rc del Tello (ver config.py):
        Mundo X (a lo largo de la pared) → rc left_right    (+rc = derecha)
        Mundo Y (vertical)               → rc up_down       (+rc = arriba)
        Mundo Z (perpendicular a pared)  → rc forward_back  (INVERTIDO)
    """
    pid_x = PIDController(**config.PID_LR, output_limit=config.RC_MAX)
    pid_y = PIDController(**config.PID_UD, output_limit=config.RC_MAX)
    pid_z = PIDController(**config.PID_FB, output_limit=config.RC_MAX)

    t0 = time.time()
    converged_for = 0.0  # tiempo dentro de tolerancia (necesita estabilidad)
    last_t = t0
    while time.time() - t0 < max_time_s:
        frame = safe_frame(tello)
        pos = None
        if frame is not None:
            pos, annotated = tracker.detect_and_estimate(frame)
            if annotated is not None:
                cv2.putText(annotated, "RECENTER", (10, 90),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 200, 0), 2)
                cv2.imshow("Tello — Prueba 1.1", annotated)
                cv2.waitKey(1)

        if pos is None:
            # Sin feedback ArUco: mantenemos hover y esperamos otro frame.
            tello.send_rc_control(0, 0, 0, 0)
            time.sleep(SAMPLE_DT)
            converged_for = 0.0
            last_t = time.time()
            continue

        ex = target["x"] - pos["x"]
        ey = target["y"] - pos["y"]
        ez = target["z"] - pos["z"]
        err = max(abs(ex), abs(ey), abs(ez))

        # Logging opcional para inspección post-vuelo.
        if logger is not None:
            logger.log({
                "timestamp": time.time(), "phase": "recenter",
                "command": cmd_name, "axis": "-", "repeat": repeat,
                "t_send": None, "t_since_cmd": None,
                "marker_id": pos["marker_id"],
                "pos_x": pos["x"], "pos_y": pos["y"], "pos_z": pos["z"],
                "marker_dist": pos["distance"],
                **telemetry_snapshot(tello),
            })

        if err < tol_m:
            converged_for += time.time() - last_t
            last_t = time.time()
            tello.send_rc_control(0, 0, 0, 0)
            if converged_for >= 0.5:  # estable 0.5 s
                return True
            time.sleep(SAMPLE_DT)
            continue
        else:
            converged_for = 0.0
            last_t = time.time()

        rc_lr = pid_x.compute(ex)
        rc_ud = pid_y.compute(ey)
        rc_fb = -pid_z.compute(ez)   # inversión por convención de ejes
        tello.send_rc_control(rc_lr, rc_fb, rc_ud, 0)
        time.sleep(SAMPLE_DT)

    tello.send_rc_control(0, 0, 0, 0)
    return False


def dominant_marker(samples):
    """Devuelve el marker_id con más muestras en la lista."""
    if not samples:
        return None
    counts = {}
    for s in samples:
        m = s.get("marker_id")
        if m is None:
            continue
        counts[m] = counts.get(m, 0) + 1
    if not counts:
        return None
    return max(counts.items(), key=lambda kv: kv[1])[0]


def step_metrics(response_samples, axis, target_delta_m, t_send,
                 p0, ref_marker):
    """
    Calcula métricas de step response a partir de las muestras de la
    fase 'response' (DURANTE la respuesta, NO después).

    Para evitar saltos espurios cuando el tracker cambia de marker
    entre frames, se filtran las muestras al `ref_marker` dominante
    de la fase pre_hover. p0 también se calcula con ese mismo marker.

    Devuelve: response_time_ms, overshoot_cm, settling_time_s,
              final_error_cm, n_samples_used.
    """
    pts = [s for s in response_samples
           if s.get(axis) is not None
           and (ref_marker is None or s.get("marker_id") == ref_marker)]
    if len(pts) < 5 or p0 is None:
        return {"response_time_ms": None, "overshoot_cm": None,
                "settling_time_s":  None, "final_error_cm": None,
                "n_samples_used":   len(pts)}

    target = p0 + target_delta_m
    sign = 1.0 if target_delta_m >= 0 else -1.0

    # Tiempo de respuesta: primer instante con desplazamiento >2 cm en el sentido del comando.
    response_time_ms = None
    for s in pts:
        if sign * (s[axis] - p0) > 0.02:
            response_time_ms = (s["t"] - t_send) * 1000.0
            break

    # Overshoot respecto al objetivo (en el sentido del comando).
    extreme = max(pts, key=lambda s: sign * (s[axis] - p0))[axis]
    overshoot_cm = max(0.0, sign * (extreme - target)) * 100.0

    # Error final: promedio del último tercio de muestras.
    tail = pts[max(1, int(len(pts) * 2 / 3)):]
    final_pos = statistics.mean(s[axis] for s in tail)
    final_error_cm = (final_pos - target) * 100.0

    # Settling time al 2% del paso (banda mínima 2 cm) — último instante fuera de banda.
    band = max(0.02, 0.02 * abs(target_delta_m))
    settling_time_s = None
    for s in reversed(pts):
        if abs(s[axis] - target) > band:
            settling_time_s = s["t"] - t_send
            break

    return {
        "response_time_ms": response_time_ms,
        "overshoot_cm":     overshoot_cm,
        "settling_time_s":  settling_time_s,
        "final_error_cm":   final_error_cm,
        "n_samples_used":   len(pts),
    }


def main():
    tello = Tello()
    tello.connect()
    battery = tello.get_battery()
    print(f"[INFO] Batería: {battery}%")
    if battery < config.MIN_BATTERY_PCT:
        print("[ERROR] Batería insuficiente. Cambia batería.")
        return

    tracker = ArUcoTracker()
    logger = FlightLogger("test_1_1_step_response")

    tello.streamon()
    # Espera a que el stream entregue frames antes de despegar.
    t0 = time.time()
    while safe_frame(tello) is None and time.time() - t0 < 5.0:
        time.sleep(0.1)

    tello.takeoff()
    # Esperar a que el IMU se estabilice tras el takeoff antes de enviar el
    # siguiente comando. Sin esta pausa el Tello frecuentemente devuelve
    # 'error No valid imu' al primer move_*, y queda en un estado donde NO
    # puede mantener hover (deriva incontrolable en una dirección aleatoria
    # según el bias físico de cada unidad).
    print("[INFO] Esperando 4 s a que el IMU se estabilice tras takeoff...")
    t0 = time.time()
    while time.time() - t0 < 4.0:
        tello.send_rc_control(0, 0, 0, 0)
        time.sleep(0.05)

    # Subir hasta una altitud donde la cámara encuadre ambas filas del grid.
    if INITIAL_CLIMB_CM > 0:
        print(f"[INFO] Subiendo {INITIAL_CLIMB_CM} cm para encuadrar el grid completo...")
        try:
            tello.move_up(INITIAL_CLIMB_CM)
        except Exception as e:
            err = str(e)
            print(f"[ERROR] move_up({INITIAL_CLIMB_CM}) falló: {err}")
            if "imu" in err.lower():
                print("\n" + "=" * 64)
                print("ABORTANDO: el IMU del Tello no está válido.")
                print("=" * 64)
                print("Acciones recomendadas (en orden):")
                print("  1. Aterriza el dron (Ctrl+C ahora si está volando).")
                print("  2. APAGA el dron (botón al lado de la batería).")
                print("  3. Espera 10 s y enciéndelo de nuevo.")
                print("  4. Coloca el dron sobre superficie 100% plana y nivelada.")
                print("  5. NO lo muevas durante 30 s tras encender.")
                print("  6. Verifica LED verde sólido antes de correr el script.")
                print("  7. Verifica batería >= 50% (la 2ª prueba tenía 37%).")
                print("  8. Si falla otra vez, calibra IMU desde la app de Tello.")
                print("=" * 64 + "\n")
                # Aterrizamos el dron limpio.
                try: tello.land()
                except Exception: pass
                try: tello.streamoff()
                except Exception: pass
                logger.close()
                cv2.destroyAllWindows()
                return
    print("[INFO] Hover inicial 5 s para estabilizar (debe verse al menos 1 ArUco)...")
    init_samples = hover_and_record(
        logger, tello, tracker,
        duration=5.0, phase="init_hover", cmd_name="none",
        axis="-", repeat=0)

    # Posición de referencia: media del último ~2 s del init_hover, usando
    # el marker más visto. Esta será la posición a la que volveremos por PID
    # antes de cada step para garantizar mismas condiciones iniciales.
    init_mk = dominant_marker(init_samples)
    init_filt = [s for s in init_samples
                 if init_mk is None or s.get("marker_id") == init_mk]
    if len(init_filt) < 10:
        print("[ERROR] No hay suficientes detecciones ArUco en init_hover. "
              "Posiciona el dron de modo que vea al menos 1 marker estable.")
        try: tello.land()
        except Exception: pass
        try: tello.streamoff()
        except Exception: pass
        logger.close(); cv2.destroyAllWindows()
        return

    tail = init_filt[-int(2.0 / SAMPLE_DT):]
    target_pos = {
        "x": statistics.mean(s["x"] for s in tail),
        "y": statistics.mean(s["y"] for s in tail),
        "z": statistics.mean(s["z"] for s in tail),
    }
    print(f"[INFO] target_pos = X={target_pos['x']:.2f} "
          f"Y={target_pos['y']:.2f} Z={target_pos['z']:.2f} m "
          f"(ref marker {init_mk})")

    # Resumen por (comando, repetición) para imprimir al final.
    summary_rows = []

    try:
        # Bucles intercalados: rep externo, comando interno.
        # Así la secuencia es right→up→left→down → right→up→left→down → ...
        # En lugar de 3 reps seguidas en el mismo eje, lo cual evita que
        # el error residual de los pares move/inverse se acumule en X (o en Y)
        # antes de cambiar de eje y saque al dron del campo de visión ArUco.
        for rep in range(1, REPEATS + 1):
            print(f"\n========== RONDA {rep}/{REPEATS} ==========")
            for cmd_name, sdk_cmd, cmd_arg_cm, inverse_method, axis, sign in COMMANDS:
                target_delta_m = (cmd_arg_cm / 100.0) * sign
                print(f"\n[CMD] {cmd_name} {cmd_arg_cm} cm  (rep {rep}/{REPEATS})")

                # 0) Re-centrado por PID a la posición de referencia.
                #    Elimina la deriva acumulada de los pares move/inverso
                #    anteriores y garantiza condición inicial idéntica.
                print(f"      recenter (target X={target_pos['x']:.2f}, "
                      f"Y={target_pos['y']:.2f}, Z={target_pos['z']:.2f})...")
                ok = recenter_to_target(
                    tello, tracker, target_pos,
                    max_time_s=RECENTER_MAX_S, tol_m=RECENTER_TOL_M,
                    logger=logger, cmd_name=cmd_name, repeat=rep)
                print(f"      → recenter {'OK' if ok else 'TIMEOUT'}")

                # 1) Pre-hover corto: confirma que el recenter dejó al dron
                #    estable. NO usamos sus muestras para p0 (usamos target_pos
                #    directamente, que es la referencia conocida del recenter).
                print(f"      pre-hover {PRE_HOVER_S:.0f} s...")
                hover_and_record(
                    logger, tello, tracker,
                    duration=PRE_HOVER_S, phase="pre_hover",
                    cmd_name=cmd_name, axis=axis, repeat=rep)

                # p0 = posición de referencia del recenter (eje afectado).
                p0 = target_pos[axis]
                print(f"      p0_{axis} = {p0:.3f} m (de target_pos)")

                # 2) STEP NO-BLOQUEANTE: send_command_without_return envía
                #    el comando UDP y retorna de inmediato, sin esperar el "ok"
                #    que el Tello manda al COMPLETAR el movimiento. Así
                #    arrancamos record_response justo en t_send y capturamos
                #    el rise time real (no sólo el settling).
                t_send = time.time()
                print(f"      step '{sdk_cmd} {cmd_arg_cm}' (no-bloq) @ t={t_send:.3f}")
                try:
                    tello.send_command_without_return(f"{sdk_cmd} {cmd_arg_cm}")
                except Exception as e:
                    print(f"      [WARN] SDK no aceptó comando ({e}); se omite rep.")
                    continue

                # 3) Grabar respuesta (rise + overshoot + settling)
                response_samples = record_response(
                    logger, tello, tracker,
                    duration=POST_CMD_RECORD_S,
                    cmd_name=cmd_name, axis=axis,
                    repeat=rep, t_send=t_send)

                # 4) Métricas: usamos TODAS las muestras válidas (sin filtrar
                #    por marker), porque con IPPE_SQUARE + outlier filter las
                #    poses son consistentes incluso cambiando de marker.
                metrics = step_metrics(response_samples, axis, target_delta_m,
                                       t_send, p0=p0, ref_marker=None)
                metrics.update({"command": cmd_name, "axis": axis, "repeat": rep,
                                "response_window_s": POST_CMD_RECORD_S,
                                "ref_marker": None, "p0": p0,
                                "t_send": t_send})
                summary_rows.append(metrics)
                print(f"      → t_resp={metrics['response_time_ms']}, "
                      f"overshoot={metrics['overshoot_cm']}, "
                      f"settling={metrics['settling_time_s']}, "
                      f"err_final={metrics['final_error_cm']}, "
                      f"n={metrics['n_samples_used']}")

                # 5) Regreso bloqueante: inverso del comando para acercar al
                #    target. NO depende de exactitud; el recenter PID al inicio
                #    del próximo comando se encarga de cerrar la diferencia.
                try:
                    print(f"      return {inverse_method}({cmd_arg_cm}) [bloqueante]")
                    getattr(tello, inverse_method)(cmd_arg_cm)
                except Exception as e:
                    print(f"      [WARN] return command falló ({e}); continuamos.")

                # 6) Hover de recuperación
                hover_and_record(logger, tello, tracker,
                                 duration=RETURN_RECOVERY_S,
                                 phase="recovery", cmd_name=cmd_name,
                                 axis=axis, repeat=rep)

                # Verificación de batería
                bat = tello.get_battery()
                if bat is not None and bat < config.MIN_BATTERY_PCT:
                    print(f"[WARN] Batería {bat}% < {config.MIN_BATTERY_PCT}% — abortando.")
                    raise KeyboardInterrupt

    except KeyboardInterrupt:
        print("\n[INFO] Prueba interrumpida.")
    finally:
        print("[INFO] Aterrizando...")
        try:
            tello.land()
        except Exception:
            pass
        try:
            tello.streamoff()
        except Exception:
            pass
        logger.close()
        cv2.destroyAllWindows()

        # Resumen por consola (las gráficas/RMSE finales se hacen en post-proceso).
        if summary_rows:
            print("\n" + "=" * 78)
            print("RESUMEN — Prueba 1.1 (estimación en línea, refinar con CSV)")
            print("=" * 78)
            print(f"{'cmd':<6} {'rep':>3} {'mk':>3} {'n':>4} "
                  f"{'t_resp(ms)':>11} {'oversh(cm)':>11} "
                  f"{'settling(s)':>12} {'err_fin(cm)':>12}")
            for m in summary_rows:
                def f(v, fmt):
                    return ("-" if v is None else format(v, fmt))
                print(f"{m['command']:<6} {m['repeat']:>3} "
                      f"{str(m.get('ref_marker', '-')):>3} "
                      f"{m.get('n_samples_used', 0):>4} "
                      f"{f(m['response_time_ms'], '11.1f')} "
                      f"{f(m['overshoot_cm'],     '11.2f')} "
                      f"{f(m['settling_time_s'],  '12.2f')} "
                      f"{f(m['final_error_cm'],   '12.2f')}")
            print("=" * 78)
        print("[DONE] Prueba 1.1 completada. Revisa el CSV en logs/.")


if __name__ == "__main__":
    main()
