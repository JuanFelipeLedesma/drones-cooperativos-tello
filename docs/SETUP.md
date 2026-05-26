# Guía de montaje (hardware, red y marcadores)

Lo que necesitas y cómo dejarlo listo antes de volar. Para correr cada prueba, ver
[`REPLICATION.md`](REPLICATION.md).

## 1. Lista de materiales

| Componente | Detalle | Notas |
|---|---|---|
| 2 × Ryze/DJI **Tello** | Boost Combo recomendado (baterías extra) | Cada uno crea su propia red WiFi |
| **Computador MASTER** | macOS (en este proyecto, MacBook Pro) | Líder, IP 192.168.1.1 |
| **Computador SLAVE** | Ubuntu 22.04 (probado) | Seguidor, IP 192.168.1.2 |
| **Cable Ethernet** | directo Mac ↔ Ubuntu | Backbone de la red Ad-Hoc |
| Adaptadores USB-Ethernet | si los equipos no traen puerto RJ45 | |
| 6 × marcadores **ArUco** | `DICT_4X4_50`, IDs 0–5, **48 cm** de lado | Impresos en papel/cartón rígido |
| Pared lisa | ~2.5 m de ancho × 1.5 m de alto libres | Para el grid 3×2 |
| Cinta métrica + nivel | para ubicar los marcadores con precisión | El error de montaje se traslada al mundo |
| (Opcional) trípode + celular | para el video cenital de la prueba 5.3 | |
| (Opcional) cartón rígido | para perturbar el dron a salvo en la prueba 2.1 | **Nunca la mano** |

## 2. Software

En **ambos** computadores:

```bash
python3 -m venv venv
source venv/bin/activate          # Windows: venv\Scripts\activate
pip install -r requirements.txt
```

> Probado con Python 3.14 (Mac) y 3.10 (Ubuntu). `djitellopy` maneja el SDK del Tello;
> `opencv-python` la visión ArUco. Si `opencv` no detecta `cv2.aruco`, instala
> `opencv-contrib-python` en su lugar.

Verifica la conexión a un Tello (con el equipo unido a la WiFi del dron):

```bash
python -c "from djitellopy import Tello; t=Tello(); t.connect(); print('batería:', t.get_battery(), '%')"
```

## 3. Red Ethernet entre los dos computadores

El enlace directo Mac↔Ubuntu transporta los mensajes de cooperación. Subred `192.168.1.0/24`.

### 3.1 Ubuntu (SLAVE) — IP estática **persistente**

> ⚠️ **No uses `sudo ip addr add ...`**: esa IP se borra al desconectar el cable y la interfaz queda
> sin dirección al reconectar, rompiendo la reconexión (lo descubrimos a la mala — ver
> [`DESIGN_DECISIONS.md`](DESIGN_DECISIONS.md)). Configura persistencia con NetworkManager:

```bash
# Sustituye "Wired connection 1" por el nombre real (nmcli connection show)
sudo nmcli connection modify "Wired connection 1" \
    ipv4.method manual ipv4.addresses 192.168.1.2/24 \
    ipv4.gateway "" ipv4.ignore-auto-dns yes connection.autoconnect yes
sudo nmcli connection up "Wired connection 1"

ip addr show enp1s0        # debe mostrar  inet 192.168.1.2/24
```

### 3.2 macOS (MASTER) — IP estática

Ajustes del Sistema → Red → (adaptador Ethernet) → Detalles → TCP/IP → Configurar IPv4:
**Manualmente**, dirección `192.168.1.1`, máscara `255.255.255.0`, sin router.

### 3.3 Verificar el enlace

```bash
# desde Ubuntu
ping 192.168.1.1
# desde Mac
ping 192.168.1.2
```

RTT esperado ~1.5 ms, 0 % de pérdida (ver benchmark, prueba 3.1).

Las IPs y el puerto UDP están en `config.py` (`MASTER_IP`, `SLAVE_IP`, `COMMS_PORT = 5005`).

## 4. Marcadores ArUco

### 4.1 Generar e imprimir

```bash
python generate_aruco_markers.py     # escribe aruco_markers/aruco_marker_{0..5}.png
```

Imprime los 6 a **48 cm de lado** (el cuadrado negro debe medir exactamente 48 cm; esa medida está en
`config.py → MARKER_SIZE_M`). Si imprimes a otro tamaño, **actualiza `MARKER_SIZE_M`** o la escala de
la pose saldrá mal. *Los marcadores iniciales de 11.5 cm daban demasiado ruido de pose — ver decisiones.*

### 4.2 Montar en la pared

Grid de **3 columnas × 2 filas**, separación de **1.0 m entre centros** (vertical y horizontal):

```
   [ID 1]      [ID 3]      [ID 5]      ← centros a Y = 1.4 m del piso
   [ID 0]      [ID 2]      [ID 4]      ← centros a Y = 0.4 m del piso
   X=0.0       X=1.0       X=2.0
```

- **ID 0** (inferior izquierda) es el **origen** del mundo.
- Usa nivel y cinta: el error de montaje se traduce directamente en error de localización.
- Espacio de pared necesario: ~2.5 m de ancho × ~1.5 m de alto. El marcador más bajo queda con su
  borde inferior a ~0.16 m del piso; el más alto, con su borde superior a ~1.64 m.

Si tu disposición física es distinta, edita `config.py → MARKER_WORLD_POSITIONS` con las coordenadas
reales `[X, Y, Z]` (en metros) del centro de cada marcador.

### 4.3 Posición de los drones al inicio

- Mac/**Tello A** → X ≈ 0.5 m (a la izquierda del centro).
- Ubuntu/**Tello B** → X ≈ 1.5 m (a la derecha, ~1 m del A).
- Ambos a ~2.5 m de la pared, **mirando los marcadores** (que entren en el campo de visión al despegar).

## 5. Calibración de la cámara (opcional, actualmente **no** usada)

El proyecto usa **parámetros de cámara aproximados** (`config.py → CAMERA_MATRIX`, `DIST_COEFFS`).
Hay utilidades para una calibración formal con chessboard:

```bash
python generate_chessboard.py    # patrón 9×6, 30 mm → aruco_markers/chessboard_9x6_30mm.pdf
python calibrate_camera.py        # captura y calcula la matriz de cámara
```

⚠️ La calibración formal se **descartó** en este proyecto porque, con capturas mal distribuidas en el
frame, el punto principal salió sesgado y el ruido de pose **empeoró**. Si la intentas, cubre el campo
de visión de forma uniforme (esquinas incluidas). Detalle en [`DESIGN_DECISIONS.md`](DESIGN_DECISIONS.md).

## 6. Antes de volar: checklist de seguridad

- [ ] Baterías de ambos Tello ≥ 60 %.
- [ ] **IMU calibrado** desde la app de Ryze (si el log dice `No valid imu`, abortar y recalibrar).
- [ ] Cable Ethernet conectado; `ping` en ambos sentidos OK.
- [ ] Cada computador conectado a la WiFi de **su** Tello.
- [ ] Espacio despejado; hélices sin grietas (ten repuestos).
- [ ] Mano cerca de `Ctrl+C`. Para abortar de emergencia: `Ctrl+C` (el otro dron detecta el abort y
      aterriza). Si se atasca: `Ctrl+Z` + `kill %1`; en último caso, apagar el Tello físicamente.

Las reglas de seguridad completas están en [`REPLICATION.md`](REPLICATION.md#reglas-de-seguridad).
