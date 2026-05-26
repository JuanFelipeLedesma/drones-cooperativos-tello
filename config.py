"""
Configuración central del proyecto de pruebas experimentales.
Ajusta estos valores según tu setup en CREA antes de correr las pruebas.
"""

# ============================================================
# RED — Direcciones IP del enlace Ethernet entre computadores
# ============================================================
MASTER_IP = "192.168.1.1"       # Mac (líder)
SLAVE_IP  = "192.168.1.2"       # Ubuntu (seguidor)
COMMS_PORT = 5005               # Puerto UDP para mensajes de cooperación
COMMS_FREQ_HZ = 20              # Frecuencia de publicación de estado [Hz]

# ============================================================
# TELLO — Conexión WiFi (cada dron crea su propia red)
# ============================================================
TELLO_IP = "192.168.10.1"       # IP por defecto del Tello (no cambiar)

# ============================================================
# ARUCO — Marcadores y cámara
# ============================================================
ARUCO_DICT_ID = "DICT_4X4_50"   # Diccionario ArUco a usar
MARKER_SIZE_M = 0.48            # Tamaño físico del lado del cuadrado negro [m]

# Posiciones conocidas de los ArUco markers en el espacio [x, y, z] en metros
#
# ⚠️  IMPORTANTE: La cámara del Tello apunta al FRENTE, NO hacia abajo.
#     Los markers van pegados en la PARED, a la altura de vuelo (~0.6-1.0m).
#
# Sistema de coordenadas (visto de frente a la pared con markers):
#     X = horizontal a lo largo de la pared (izq→der visto de frente)
#     Y = vertical (piso→techo)
#     Z = profundidad PERPENDICULAR a la pared, hacia el interior de la sala
#         (Z=0 es la pared, Z crece alejándose de la pared)
#
# Los markers van TODOS en la misma pared, todos mirando hacia la sala.
# El dron vuela frente a esa pared y siempre la tiene en su campo de visión.
#
# Setup actualizado: 6 markers de 48 cm sobre la pared, grid 3 cols × 2 rows.
# Visto desde el dron mirando la pared (X aumenta hacia la DERECHA del dron):
#
#   Fila alta (Y=1.4m, centro):   ID 1     ID 3     ID 5
#                                  |        |        |
#   Fila baja (Y=0.4m, centro):   ID 0     ID 2     ID 4
#                                  X=0     X=1.0    X=2.0
#
# Cada marker = 48 cm × 48 cm.
# Separación horizontal entre centros: 1.0 m → 52 cm de gap entre bordes.
# Separación vertical entre centros:  1.0 m → 52 cm de gap entre bordes.
# Origen del mundo: marker 0 (centro del cuadrado negro abajo-izquierda).
#
# Espacio de pared mínimo necesario:
#   Ancho:  2.0 m (centro a centro) + 0.48 m (marker) = ~2.5 m
#   Alto:   1.0 m (centro a centro) + 0.48 m         = ~1.5 m
#   Y_inferior del marker más bajo: 0.4 - 0.24 = 0.16 m sobre el piso
#   Y_superior del marker más alto: 1.4 + 0.24 = 1.64 m sobre el piso
MARKER_WORLD_POSITIONS = {
    0: [0.0, 0.4, 0.0],   # columna izq, fila baja  (ORIGEN del mundo)
    1: [0.0, 1.4, 0.0],   # columna izq, fila alta
    2: [1.0, 0.4, 0.0],   # columna centro, fila baja
    3: [1.0, 1.4, 0.0],   # columna centro, fila alta
    4: [2.0, 0.4, 0.0],   # columna der, fila baja
    5: [2.0, 1.4, 0.0],   # columna der, fila alta
}

# Parámetros de cámara aproximados del Tello (720p, 82.6° FOV).
# Restaurados tras una calibración fallida (cy quedó sesgado a 543 vs 360
# esperado; capturas no cubrieron uniformemente el frame).
CAMERA_MATRIX = [
    [921.17,   0.0,   459.90],
    [  0.0, 919.02,   351.24],
    [  0.0,   0.0,     1.0  ],
]
DIST_COEFFS = [0.0, 0.0, 0.0, 0.0, 0.0]  # Sin distorsión como aprox. inicial

# ============================================================
# CONTROL PID — Ganancias para lazo cerrado de posición
# ============================================================
# Mapeo de ejes del mundo (pared) a comandos rc del Tello:
#   Mundo X (a lo largo de la pared) → rc left_right
#   Mundo Y (altura)                 → rc up_down
#   Mundo Z (distancia a la pared)   → rc forward_backward (INVERTIDO)
#       (forward del dron = acercarse a la pared = -Z)
# Las ganancias se aplican a error en METROS y producen comando rc en
# escala -RC_MAX..+RC_MAX (entero). Para que un error típico de 0.20 m
# produzca un comando útil (~10-20), kp debe estar en el rango 50-100.
# kp por defecto: 60 (m → rc), bastante para empezar sin oscilar.
# ki pequeño elimina el error estacionario (drift, ground effect en Z).
# kd suaviza la respuesta y evita overshoot.
PID_LR = {"kp": 60.0, "ki": 8.0,  "kd": 25.0}   # Left/right (eje X mundo)
PID_UD = {"kp": 70.0, "ki": 10.0, "kd": 25.0}   # Up/down (eje Y mundo)
PID_FB = {"kp": 60.0, "ki": 8.0,  "kd": 25.0}   # Forward/backward (eje Z mundo)

# Límite de velocidad para comandos rc (-100 a 100, usar valores bajos)
RC_MAX = 30   # Velocidad máxima rc para seguridad

# ============================================================
# FORMACIÓN — Offsets líder-seguidor
# ============================================================
# En coordenadas mundo (pared):
FORMATION_OFFSET_X =  1.0   # Seguidor 1.0 m a la derecha del líder (a lo largo de la pared)
FORMATION_OFFSET_Y =  0.0   # Misma altura
FORMATION_OFFSET_Z =  0.0   # Misma distancia a la pared

# ============================================================
# TRAYECTORIAS predefinidas para el líder (waypoints [X, Y, Z] en metros)
# ============================================================
# X = posición a lo largo de la pared
# Y = altura (mantener constante ~0.8m)
# Z = distancia desde la pared (1.0-2.5m para buena visibilidad ArUco)

TRAJECTORY_SQUARE = [
    [0.5, 0.9, 1.5],    # Inicio: frente al marker central, 1.5m de la pared
    [1.5, 0.9, 1.5],    # Derecha a lo largo de la pared
    [1.5, 0.9, 2.2],    # Alejarse de la pared
    [0.5, 0.9, 2.2],    # Izquierda, lejos de la pared
    [0.5, 0.9, 1.5],    # Volver al inicio
]

TRAJECTORY_LINE = [
    [0.5, 0.9, 1.5],    # Inicio
    [1.5, 0.9, 1.5],    # Ida: derecha a lo largo de la pared
    [0.5, 0.9, 1.5],    # Vuelta
]

# ============================================================
# SEGURIDAD
# ============================================================
COMMS_TIMEOUT_S = 5.0        # Si no recibe mensajes por este tiempo → hover
MAX_FLIGHT_TIME_S = 120      # Tiempo máximo de vuelo por prueba [s]
MIN_BATTERY_PCT = 20         # Batería mínima para volar [%]
