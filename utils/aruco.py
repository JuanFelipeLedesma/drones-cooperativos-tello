"""
Módulo de detección ArUco y estimación de pose.
Usa la cámara del Tello para detectar markers y calcular posición del dron.

Estrategia: SINGLE MARKER (el más cercano al centro del frame).
La versión multi-marker fusion fue revertida porque hacía solvePnP por cada
marker visible (~6×) y saturaba el loop, dejando al Tello sin comandos rc a
tiempo y causando deriva descontrolada en hover.
"""

import cv2
import numpy as np
import time
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import config


class ArUcoTracker:
    def __init__(self):
        # Diccionario ArUco
        dict_name = getattr(cv2.aruco, config.ARUCO_DICT_ID)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_name)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # Parámetros de cámara
        self.camera_matrix = np.array(config.CAMERA_MATRIX, dtype=np.float64)
        self.dist_coeffs = np.array(config.DIST_COEFFS, dtype=np.float64)
        self.marker_size = config.MARKER_SIZE_M

        # Posiciones conocidas de markers
        self.marker_positions = config.MARKER_WORLD_POSITIONS

        # Object points del marker (esquinas en su frame local)
        s = self.marker_size / 2.0
        self._obj_points = np.array([
            [-s,  s, 0],
            [ s,  s, 0],
            [ s, -s, 0],
            [-s, -s, 0],
        ], dtype=np.float32)

        # Último resultado
        self.last_position = None
        self.last_timestamp = 0
        self.last_markers_seen = []

    def detect_and_estimate(self, frame):
        """
        Detecta ArUco markers en el frame y estima la posición del dron
        usando el marker conocido más cercano al centro de la imagen.

        Retorna: (position_dict, annotated_frame)
            position_dict = {x, y, z, timestamp, marker_id, n_markers, distance}
            o None si ningún marker conocido fue detectado o si la pose
            cae el filtro de outliers.
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        timestamp = time.time()

        annotated = frame.copy()
        position = None

        if ids is None or len(ids) == 0:
            cv2.putText(annotated, "NO ARUCO DETECTED", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            return None, annotated

        cv2.aruco.drawDetectedMarkers(annotated, corners, ids)
        self.last_markers_seen = ids.flatten().tolist()

        # Elegir el marker conocido cuyo centro esté MÁS CERCA del centro
        # del frame (más estable que "el primero detectado").
        h, w = gray.shape[:2]
        frame_cx, frame_cy = w / 2.0, h / 2.0

        best = None  # (dist², idx, marker_id)
        for i, marker_id_arr in enumerate(ids.flatten()):
            marker_id = int(marker_id_arr)
            if marker_id not in self.marker_positions:
                continue
            pts = corners[i][0]
            cx = float(pts[:, 0].mean())
            cy = float(pts[:, 1].mean())
            d = (cx - frame_cx) ** 2 + (cy - frame_cy) ** 2
            if best is None or d < best[0]:
                best = (d, i, marker_id)

        if best is None:
            cv2.putText(annotated, "NO KNOWN ARUCO", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            return None, annotated

        _, i, marker_id = best
        # IPPE_SQUARE específicamente para markers planares cuadrados:
        # resuelve la ambigüedad de pose planar.
        success, rvec, tvec = cv2.solvePnP(
            self._obj_points, corners[i][0],
            self.camera_matrix, self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )

        if success:
            R, _ = cv2.Rodrigues(rvec)
            cam_pos_marker = -R.T @ tvec.flatten()
            marker_world = np.array(self.marker_positions[marker_id])
            drone_world = marker_world + cam_pos_marker

            candidate = {
                "x": float(drone_world[0]),
                "y": float(drone_world[1]),
                "z": float(drone_world[2]),
                "timestamp": timestamp,
                "marker_id": marker_id,
                "n_markers": 1,  # mantener API; siempre 1 en este modo
                "distance": float(np.linalg.norm(tvec)),
            }

            # Filtro de outlier temporal
            MAX_JUMP_SAME_MARKER_M = 0.5
            MAX_JUMP_DIFF_MARKER_M = 0.3
            OUTLIER_DT_S = 0.5
            if (self.last_position is not None
                    and (timestamp - self.last_timestamp) < OUTLIER_DT_S):
                dx = candidate["x"] - self.last_position["x"]
                dy = candidate["y"] - self.last_position["y"]
                dz = candidate["z"] - self.last_position["z"]
                same = self.last_position.get("marker_id") == marker_id
                max_jump = (MAX_JUMP_SAME_MARKER_M if same
                            else MAX_JUMP_DIFF_MARKER_M)
                if max(abs(dx), abs(dy), abs(dz)) > max_jump:
                    tag = "OUTLIER-SAME" if same else "OUTLIER-SWAP"
                    cv2.putText(annotated,
                                f"{tag} ({dx:+.2f},{dy:+.2f},{dz:+.2f})",
                                (10, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                (0, 165, 255), 2)
                    position = None
                else:
                    position = candidate
            else:
                position = candidate

            if position is not None:
                self.last_position = position
                self.last_timestamp = timestamp
                cv2.drawFrameAxes(annotated, self.camera_matrix,
                                  self.dist_coeffs, rvec, tvec, 0.1)

        if position:
            text = (f"X:{position['x']:.2f} Y:{position['y']:.2f} "
                    f"Z:{position['z']:.2f}")
            cv2.putText(annotated, text, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        return position, annotated

    def get_position(self):
        """Retorna la última posición estimada (puede ser None)."""
        return self.last_position

    def position_age(self):
        """Retorna cuántos segundos han pasado desde la última detección."""
        if self.last_timestamp == 0:
            return float('inf')
        return time.time() - self.last_timestamp
