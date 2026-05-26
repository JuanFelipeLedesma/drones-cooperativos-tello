# Arquitectura del sistema

Este documento describe cómo está construido el sistema cooperativo: sus capas, el flujo de datos en
cada ciclo de control, el sistema de coordenadas, y el protocolo de comunicación. Para *por qué* se
tomó cada decisión, ver [`DESIGN_DECISIONS.md`](DESIGN_DECISIONS.md).

## 1. Visión general en capas

El sistema tiene cuatro capas que corren en paralelo en cada computador:

```
┌──────────────────────────────────────────────────────────────┐
│ CAPA DE COOPERACIÓN     (líder-seguidor / consenso / misión)  │
│   - calcula el objetivo (setpoint) de este dron               │
├──────────────────────────────────────────────────────────────┤
│ CAPA DE CONTROL         (PIDController, utils/pid.py)         │
│   - error = setpoint - pose_actual  →  comando rc (±RC_MAX)   │
├──────────────────────────────────────────────────────────────┤
│ CAPA DE LOCALIZACIÓN    (ArUcoTracker, utils/aruco.py)       │
│   - frame de cámara → pose mundo (x, y, z)                     │
├──────────────────────────────────────────────────────────────┤
│ CAPA DE COMUNICACIÓN    (UDP, utils/comms.py + protocolo bin) │
│   - publica/recibe el estado del otro dron a 20–50 Hz         │
└──────────────────────────────────────────────────────────────┘
        │                                          │
        ▼ djitellopy (SDK Tello, UDP/WiFi)         ▼ socket UDP (Ethernet)
   ┌─────────┐                              ┌──────────────┐
   │  TELLO  │                              │  OTRO DRONE  │
   └─────────┘                              └──────────────┘
```

### Por qué la cooperación vive en los computadores y no entre drones

Cada Ryze/DJI Tello **crea su propia red WiFi** y solo admite un cliente conectado a la vez. No existe
una forma soportada de poner dos Tello en la misma red. Por eso:

- El **MASTER** (Mac) habla con **Tello A** por su WiFi.
- El **SLAVE** (Ubuntu) habla con **Tello B** por su WiFi.
- La coordinación entre A y B ocurre entre **Mac ↔ Ubuntu** por un **cable Ethernet directo**, que
  funciona como backbone de la red Ad-Hoc.

Esta restricción de hardware es la razón de toda la arquitectura de dos computadores. Con Tello **EDU**
existe una alternativa (modo estación / enjambre); ver [`TELLO_EDU.md`](TELLO_EDU.md).

## 2. Ciclo de control (un dron seguidor)

Cada iteración del lazo (≈13.6 Hz medidos en hardware real) hace:

1. **Capturar** frame de la cámara del Tello (`tello.get_frame_read()`).
2. **Localizar:** `ArUcoTracker.detect_and_estimate(frame)` → pose mundo `(x, y, z)` o `None`.
3. **Recibir** la última pose del líder vía UDP (`CommsReceiver.get_latest()`), con filtro de
   promedio móvil (N=8) para no perseguir el ruido del líder.
4. **Calcular el objetivo:** `setpoint = pose_líder_filtrada + FORMATION_OFFSET`.
5. **Controlar:** un PID por eje convierte el error (en metros) en un comando `rc` entero en ±RC_MAX.
6. **Actuar:** `tello.send_rc_control(lr, fb, ud, yaw)`.
7. **Registrar** la fila en CSV (`FlightLogger`).
8. **Seguridad:** si no llega un mensaje del líder en `COMMS_TIMEOUT_S` (5 s) → `rc 0 0 0 0` (hover).

El líder (MASTER) corre el mismo lazo pero su objetivo es un waypoint fijo (formación estática) o un
waypoint de una trayectoria predefinida (formación dinámica), y **publica** su pose a `COMMS_FREQ_HZ`.

## 3. Sistema de coordenadas

La cámara del Tello mira **al frente**, no hacia abajo. Los marcadores van pegados a una **pared** y el
dron vuela de frente a ella. El sistema de coordenadas mundo, visto de frente a la pared:

| Eje | Dirección | Mapea al comando rc |
|-----|-----------|---------------------|
| **X** | horizontal a lo largo de la pared (izq→der) | `left_right` |
| **Y** | vertical (piso→techo) | `up_down` |
| **Z** | perpendicular a la pared, hacia el interior de la sala (Z=0 = pared) | `forward_backward` **invertido** (avanzar = acercarse a la pared = −Z) |

El **origen** del mundo es el centro del marcador **ID 0** (esquina inferior izquierda del grid).

```
   Fila alta (Y=1.4 m):   [1]      [3]      [5]
   Fila baja (Y=0.4 m):   [0]      [2]      [4]
                          X=0     X=1.0    X=2.0   (metros)
```

Posiciones exactas en `config.py → MARKER_WORLD_POSITIONS`.

## 4. Localización por ArUco (`utils/aruco.py`)

- **Diccionario:** `DICT_4X4_50`, marcadores de **48 cm** de lado.
- **Selección de marcador:** se usa **un solo marcador**, el más cercano al centro del frame (más
  estable que "el primero detectado"). *La fusión multi-marcador fue probada y descartada por saturar
  el lazo — ver [`DESIGN_DECISIONS.md`](DESIGN_DECISIONS.md).*
- **Estimación de pose:** `cv2.solvePnP(..., flags=cv2.SOLVEPNP_IPPE_SQUARE)`. IPPE_SQUARE es
  específico para marcadores planos cuadrados y **resuelve la ambigüedad de pose planar** que hace
  saltar `SOLVEPNP_ITERATIVE` entre dos soluciones.
- **De marcador a dron:** con la rotación `R` y traslación `tvec` de la cámara respecto al marcador,
  la posición de la cámara en el frame del marcador es `-Rᵀ·tvec`; sumando la posición mundo conocida
  del marcador se obtiene la pose mundo del dron.
- **Filtro de outliers:** si entre dos detecciones consecutivas (< 0.5 s) la pose salta más de 0.5 m
  (mismo marcador) o 0.3 m (cambio de marcador), se descarta la lectura.

## 5. Control PID (`utils/pid.py`)

`PIDController(kp, ki, kd, output_limit)`:

- El **error está en metros** y la salida es un comando `rc` entero en ±`output_limit` (±30).
  Por eso las ganancias son "grandes" (kp≈60–70): un error típico de 0.20 m debe producir un comando
  útil (~12). *Las ganancias originales (kp≈0.4) producían `int(0.08)=0` → el dron no se movía; ver
  decisiones.*
- **Anti-windup:** el término integral se satura a `±output_limit/ki`.
- Ganancias en `config.py`: `PID_LR`, `PID_UD` (más fuerte, contra gravedad), `PID_FB`.

## 6. Comunicación

### 6.1 Versión de referencia (JSON) — `utils/comms.py`

`DroneMessage` serializado a JSON, `CommsSender`/`CommsReceiver` sobre UDP, con un hilo receptor que
mantiene el último mensaje, cuenta recibidos/perdidos por número de secuencia, y expone
`is_connected()` (timeout configurable). Útil por su legibilidad y como referencia.

### 6.2 Protocolo binario de producción — `test_3_2_protocol.py`

El protocolo que se usa en las pruebas cooperativas es **binario, de 41 bytes**, en orden de red
(big-endian), con CRC-16/CCITT al final para integridad de extremo a extremo:

| Campo | Tipo | Bytes | Descripción |
|-------|------|:-----:|-------------|
| `drone_id` | uint8 | 1 | Identificador del dron |
| `seq` | uint32 | 4 | Número de secuencia |
| `timestamp` | float64 | 8 | Marca de tiempo Unix [s] |
| `pos_x/y/z` | float32 ×3 | 12 | Posición mundo [m] |
| `vel_x/y/z` | float32 ×3 | 12 | Velocidad mundo [m/s] |
| `battery` | uint8 | 1 | Batería [%] |
| `mission_state` | uint8 | 1 | Estado de misión (enum) |
| `crc16` | uint16 | 2 | CRC-16/CCITT |
| **TOTAL** | | **41** | |

- **4× más compacto** que el JSON equivalente (162 bytes).
- **3–5× más rápido** de (de)serializar usando `binascii.crc_hqx` (implementación C del CRC). *Una
  primera versión del CRC en Python puro era ~280× más lenta — ver decisiones.*
- A 50 Hz consume ~16 kbps: **0.02 %** del enlace de 93 Mbps.
- **0 errores de CRC en >50 000 mensajes** acumulados en todas las pruebas, incluso bajo 20 % de pérdida.

### 6.3 `mission_state` (enum)

Usado para coordinar la máquina de estados de la misión (OE5). Valores relevantes: estados de fase
(idle, takeoff, formación, trayectoria, hover) y dos señales de control: aterrizaje del seguidor
(`mission_state = 8`, "slave_land") y abort/aterrizaje del líder, que el otro dron detecta para aterrizar
de forma segura.

## 7. La misión cooperativa (OE5) — máquina de estados

`test_5_1_master.py` orquesta **7 fases secuenciales**:

```
TAKEOFF → WAIT_SLAVE → FORMATION → TRAJECTORY → HOVER_AT_DEST → SLAVE_LAND → MASTER_LAND
```

- **WAIT_SLAVE** debe durar ≥ 25 s: el seguidor tarda ~15–18 s entre detectar el primer mensaje y
  entrar al lazo (takeoff + espera de IMU + ascenso + hover inicial). Si el líder arranca la trayectoria
  antes, el seguidor nunca alcanza la formación y los PID saturan.
- **SLAVE_LAND:** el líder publica `mission_state=8`; el seguidor aterriza; luego el líder aterriza
  (**aterrizaje secuencial** para evitar interferencia aerodinámica entre ambos).

## 8. Registro de datos (`utils/logger.py`)

`FlightLogger` escribe un CSV con timestamp en el nombre. **El esquema de columnas se fija en la
primera fila escrita**: para columnas que solo aparecen en ciertos eventos, hay que poblarlas con
`None` desde la primera fila o el escritor falla. Todos los CSV crudos están versionados en `logs/`.
