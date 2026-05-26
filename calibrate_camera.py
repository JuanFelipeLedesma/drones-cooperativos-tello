"""
Calibra la cámara del Tello usando un patrón de chessboard impreso.

USO:
    1. Imprime el chessboard generado por generate_chessboard.py.
    2. Pégalo a una superficie rígida (cartón, cartulina, mesa).
    3. Mide CON REGLA un cuadrado del impreso y mete el valor en
       SQUARE_SIZE_M abajo (en METROS, p.ej. 0.030 para 30 mm).
    4. Conéctate al WiFi del Tello.
    5. NO HACE FALTA QUE EL DRON DESPEGUE — sólo enciéndelo y deja la
       cámara apuntando al chessboard.
    6. Corre:
            python calibrate_camera.py
    7. En la ventana, mueve el chessboard a distintas posiciones y
       ángulos. Pulsa 'c' para capturar cuando se vean las esquinas
       en VERDE (chessboard detectado). Necesitas ≥ 15 capturas.
       Pulsa 'q' cuando termines.
    8. El script corre cv2.calibrateCamera y te imprime la nueva
       CAMERA_MATRIX y DIST_COEFFS para que las pegues en config.py.

CONSEJOS para una buena calibración:
    - Captura el chessboard a distintas distancias (cerca, medio, lejos).
    - Captura desde distintos ángulos (frontal, oblicuo, con tilt).
    - Cubre todo el campo de visión: capturas con el chessboard en
      cada esquina del frame, no solo en el centro.
    - 20-30 capturas suelen dar mejor resultado que 15.
"""

import os
import sys
import time
import cv2
import numpy as np

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from djitellopy import Tello

# ---------- Parámetros del patrón ----------
# El chessboard impreso tiene 8 × 7 cuadrados → 7 × 6 esquinas interiores.
COLS_INNER = 7          # esquinas interiores horizontales (= cuadrados − 1)
ROWS_INNER = 6          # esquinas interiores verticales (= cuadrados − 1)
SQUARE_SIZE_M = 0.030   # tamaño físico del cuadrado en METROS — MIDE CON REGLA

# ---------- Carpeta de capturas ----------
CAPTURE_DIR = "calibration_captures"
os.makedirs(CAPTURE_DIR, exist_ok=True)


def main():
    pattern_size = (COLS_INNER, ROWS_INNER)

    # Puntos 3D del chessboard en su frame local (z = 0).
    # Construye una grilla (0,0,0), (1,0,0), ... × SQUARE_SIZE_M.
    objp = np.zeros((COLS_INNER * ROWS_INNER, 3), np.float32)
    objp[:, :2] = np.mgrid[0:COLS_INNER, 0:ROWS_INNER].T.reshape(-1, 2)
    objp *= SQUARE_SIZE_M

    obj_points_all = []   # 3D points por captura
    img_points_all = []   # 2D points por captura
    captured_frames = []  # para reportar tamaño

    print("[INFO] Conectando al Tello...")
    tello = Tello()
    tello.connect()
    print(f"[INFO] Batería: {tello.get_battery()}%")
    print("[INFO] Iniciando video stream (NO despega)...")
    tello.streamon()

    # Esperar primer frame
    t0 = time.time()
    frame = None
    while time.time() - t0 < 5.0:
        try:
            fr = tello.get_frame_read()
            if fr is not None and fr.frame is not None and fr.frame.size > 0:
                frame = fr.frame
                break
        except Exception:
            pass
        time.sleep(0.1)

    if frame is None:
        print("[ERROR] No llegó video del Tello. Aborto.")
        tello.streamoff()
        return

    print()
    print("=" * 64)
    print("CAPTURA INTERACTIVA")
    print("=" * 64)
    print("  c = capturar el frame actual (sólo si se ven las esquinas verdes)")
    print("  q = terminar y calibrar")
    print()
    print("Mueve el chessboard a distintas posiciones, ángulos y distancias.")
    print("Necesitas AL MENOS 15 capturas; 20-30 es mejor.")
    print("=" * 64)

    n_captured = 0
    last_capture_t = 0
    while True:
        try:
            fr = tello.get_frame_read()
            if fr is None or fr.frame is None or fr.frame.size == 0:
                time.sleep(0.03)
                continue
            frame = fr.frame
        except Exception:
            time.sleep(0.03)
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detección robusta: primero intenta findChessboardCornersSB (método
        # estructurado moderno, tolera mejor sombras y poco contraste). Si
        # falla, cae al método clásico findChessboardCorners.
        corners_refined = None
        found = False
        try:
            found, corners_sb = cv2.findChessboardCornersSB(
                gray, pattern_size,
                flags=cv2.CALIB_CB_NORMALIZE_IMAGE
                      + cv2.CALIB_CB_EXHAUSTIVE,
            )
            if found:
                corners_refined = corners_sb  # SB ya devuelve sub-pixel
        except Exception:
            pass

        if not found:
            found, corners = cv2.findChessboardCorners(
                gray, pattern_size,
                flags=cv2.CALIB_CB_ADAPTIVE_THRESH
                      + cv2.CALIB_CB_NORMALIZE_IMAGE,
            )
            if found:
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                            30, 0.001)
                corners_refined = cv2.cornerSubPix(
                    gray, corners, (11, 11), (-1, -1), criteria,
                )

        annotated = frame.copy()
        if found and corners_refined is not None:
            cv2.drawChessboardCorners(annotated, pattern_size,
                                      corners_refined, True)
            cv2.putText(annotated, "DETECTED - press 'c' to capture",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 255, 0), 2)
        else:
            corners_refined = None
            cv2.putText(annotated, f"no chessboard (esperando {pattern_size[0]}x{pattern_size[1]})",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 0, 255), 2)

        cv2.putText(annotated, f"Capturados: {n_captured}",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (255, 255, 0), 2)
        cv2.imshow("Tello Calibration (c=capture, q=quit)", annotated)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break
        elif key == ord('c') and found and corners_refined is not None:
            # Evitar capturas duplicadas por holding del 'c'
            if time.time() - last_capture_t < 0.5:
                continue
            last_capture_t = time.time()

            obj_points_all.append(objp.copy())
            img_points_all.append(corners_refined)
            captured_frames.append(gray.shape[::-1])  # (w, h)
            n_captured += 1
            fname = os.path.join(CAPTURE_DIR,
                                 f"calib_{n_captured:03d}.png")
            cv2.imwrite(fname, frame)
            print(f"  [CAPTURE {n_captured}] guardado en {fname}")

    cv2.destroyAllWindows()
    tello.streamoff()

    print()
    print(f"[INFO] {n_captured} capturas totales.")
    if n_captured < 10:
        print("[ERROR] Muy pocas capturas (<10). No se puede calibrar.")
        return

    print("[INFO] Calculando calibración (esto toma ~10-30 s)...")
    image_size = captured_frames[0]   # (w, h)
    ret, K, D, rvecs, tvecs = cv2.calibrateCamera(
        obj_points_all, img_points_all, image_size, None, None,
    )

    # Error de reproyección (cuanto más bajo, mejor; <1 px es excelente)
    total_err = 0.0
    total_pts = 0
    for i in range(len(obj_points_all)):
        proj, _ = cv2.projectPoints(obj_points_all[i], rvecs[i], tvecs[i], K, D)
        err = cv2.norm(img_points_all[i], proj, cv2.NORM_L2) ** 2
        total_err += err
        total_pts += len(proj)
    rms_per_pt = np.sqrt(total_err / total_pts)

    print()
    print("=" * 64)
    print("RESULTADO DE LA CALIBRACIÓN")
    print("=" * 64)
    print(f"Imágenes usadas:           {n_captured}")
    print(f"Tamaño de imagen:          {image_size[0]} x {image_size[1]}")
    print(f"RMS reproyección global:   {ret:.4f} px")
    print(f"Error medio por punto:     {rms_per_pt:.4f} px")
    print()
    print("CAMERA_MATRIX (pega esto en config.py):")
    print()
    print("CAMERA_MATRIX = [")
    for row in K:
        print(f"    [{row[0]:10.4f}, {row[1]:10.4f}, {row[2]:10.4f}],")
    print("]")
    print()
    print(f"DIST_COEFFS = {D.flatten().tolist()}")
    print()
    print("=" * 64)

    # Guardar también a archivo
    out_file = os.path.join(CAPTURE_DIR, "calibration_result.txt")
    with open(out_file, "w") as f:
        f.write(f"Imágenes usadas: {n_captured}\n")
        f.write(f"Tamaño de imagen: {image_size[0]} x {image_size[1]}\n")
        f.write(f"RMS reproyección global: {ret:.6f} px\n")
        f.write(f"Error medio por punto:   {rms_per_pt:.6f} px\n\n")
        f.write("CAMERA_MATRIX = [\n")
        for row in K:
            f.write(f"    [{row[0]:10.6f}, {row[1]:10.6f}, {row[2]:10.6f}],\n")
        f.write("]\n\n")
        f.write(f"DIST_COEFFS = {D.flatten().tolist()}\n")
    print(f"[OK] Resultado también guardado en: {out_file}")

    # Interpretación
    print()
    if ret < 0.5:
        print("[VERDICT] Excelente calibración (RMS < 0.5 px).")
    elif ret < 1.0:
        print("[VERDICT] Buena calibración (RMS < 1 px).")
    elif ret < 2.0:
        print("[VERDICT] Calibración aceptable (RMS < 2 px).")
        print("          Considera repetir con más capturas variadas.")
    else:
        print("[VERDICT] Calibración pobre (RMS >= 2 px).")
        print("          Repite asegurando capturas a distintas distancias y ángulos,")
        print("          y verificando el SQUARE_SIZE_M real del impreso.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[INFO] Interrumpido por usuario.")
        cv2.destroyAllWindows()
