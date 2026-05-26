# Guía de replicación (cómo correr cada prueba)

Procedimiento paso a paso de las 16 pruebas. Antes, completa el montaje de
[`SETUP.md`](SETUP.md). Los resultados esperados están en [`RESULTS.md`](RESULTS.md).

> **Convención de dos computadores:** en las pruebas cooperativas, **arranca SIEMPRE el SLAVE
> (Ubuntu) primero** y el MASTER (Mac) después. El SLAVE espera mensajes; el MASTER los inicia.

## Procedimiento estándar de una prueba con 2 drones

1. Mac y Ubuntu encendidos, cable Ethernet conectado, `ping` OK en ambos sentidos.
2. Confirmar IP del Ubuntu: `ip addr show enp1s0` → `inet 192.168.1.2/24`.
3. Conectar cada equipo a la WiFi de su Tello (Mac→A, Ubuntu→B).
4. Verificar baterías ≥ 60 % e **IMU calibrado**.
5. Posicionar los drones (A en X≈0.5 m, B en X≈1.5 m, ambos a ~2.5 m de la pared mirando los marcadores).
6. Lanzar el script **SLAVE**, luego el **MASTER**.
7. Mano cerca de `Ctrl+C`.

Cada script escribe su CSV en `logs/` con timestamp.

---

## OE1 — Validación del modelo dinámico (1 dron)

### 1.1 — Respuesta escalón
```bash
python test_1_1_step_response.py
```
Aplica escalones de 30 cm en 4 ejes (derecha, arriba, izquierda, abajo), 3 repeticiones por eje (12
respuestas), re-centrando por lazo cerrado entre repeticiones para no acumular deriva. De aquí salen
los parámetros del modelo de 2.º orden (ωₙ, ζ) que alimentan OE4.

### 1.2 — Latencia comando-acción
```bash
python test_1_2_latency.py
```
Mide el tiempo entre enviar un comando discreto `move_*` y detectar movimiento (>10 cm) por ArUco.
Conclusión: `move_*` tarda ~1 s → **inviable para lazo cerrado**; por eso todo lo cooperativo usa
`rc_control`.

### 1.3 — Caracterización del hover
```bash
python test_1_3_hover.py
```
Registra 60 s de hover libre (sin control) para medir deriva y ruido de posición. Justifica la
necesidad del lazo cerrado.

## OE2 — Control cooperativo

### 2.1 — Lazo cerrado ArUco (1 dron)
```bash
python test_2_1_closed_loop.py
```
El dron mantiene una posición fija con PID. Pulsa la tecla **`p`** para marcar en el log el instante en
que aplicas una perturbación física (empuja con **cartón rígido**, nunca la mano). Sirve para medir el
tiempo de recuperación.

### 2.2 — Formación estática líder-seguidor (2 drones)
```bash
# SLAVE (Ubuntu):
python test_2_2_slave.py
# MASTER (Mac):
python test_2_2_master.py
```
El MASTER mantiene hover y publica su pose a 50 Hz; el SLAVE calcula `pose_master + offset` y cierra su
lazo. El SLAVE aplica un **filtro de promedio móvil (N=8)** sobre la pose recibida (sin él, persigue el
ruido del líder y el error se duplica).

### 2.3 — Formación dinámica (2 drones)
```bash
python test_2_3_slave.py      # Ubuntu
python test_2_3_master.py     # Mac
```
El MASTER recorre una trayectoria cuadrada (`config.py → TRAJECTORY_SQUARE`) mientras el SLAVE lo sigue.

### 2.4 — Consenso distribuido (2 drones)
```bash
# Un dron con --id 1, el otro con --id 2 (mismo script):
python test_2_4_consensus.py --id 2   # Ubuntu
python test_2_4_consensus.py --id 1   # Mac
```
Ambos drones publican y reciben la pose del otro y convergen **simétricamente alrededor del centroide**,
respetando una **separación mínima de 60 cm** (`MIN_SEPARATION_X_M`). ⚠️ Sin esa separación los drones
se apilan y chocan por *downwash* — fue el primer intento y falló.

## OE3 — Red Ad-Hoc

### 3.1 — Benchmark del enlace (sin drones)
```bash
# Lado servidor y lado cliente según el script (ping + iperf3 + UDP del protocolo a 10/25/50 Hz).
python test_3_1_ethernet.py
```
> Si `iperf3` da `Address already in use`, espera unos segundos o cambia el puerto (`-p`): el socket
> queda en `TIME_WAIT`. Instala `iperf3` si falta (`apt install iperf3` / `brew install iperf3`).

### 3.2 — Protocolo de mensajes (sin drones)
```bash
python test_3_2_protocol.py
```
Compara serialización **binaria (41 B) vs JSON (162 B)**: tamaño, tiempo de encode/decode e integridad
CRC. No requiere red ni drones.

### 3.3 — Degradación controlada de la red (2 drones + inyección)
La prueba estrella de OE3. Se inyecta retardo/pérdida en el flujo MASTER→SLAVE con `tc/netem`.

> ⚠️ **`tc netem` aplicado a la interfaz solo degrada el tráfico de SALIDA (egress).** El flujo crítico
> líder→seguidor es **ENTRADA (ingress)** al Ubuntu, así que hay que redirigirlo a un dispositivo
> **IFB** y degradar su egress. Setup (una vez por sesión, en Ubuntu):

```bash
sudo modprobe ifb
sudo ip link add ifb0 type ifb
sudo ip link set ifb0 up
sudo tc qdisc add dev enp1s0 ingress
sudo tc filter add dev enp1s0 parent ffff: protocol ip u32 \
    match u32 0 0 flowid 1:1 action mirred egress redirect dev ifb0
```

Luego corre la prueba:
```bash
python test_3_3_slave.py            # Ubuntu (anota la condición de red en el CSV)
python test_3_3_master.py           # Mac
python test_3_3_inject.py           # Ubuntu, EN PARALELO: aplica netem a ifb0 por condición
```
`test_3_3_inject.py` barre 8 condiciones (baseline, delay 50/100/200 ms, loss 5/10/20 %, combo
100 ms+10 %), ~25 s cada una, aplicando p. ej. `tc qdisc ... dev ifb0 root netem delay 200ms`.

**Validar que la inyección funciona:** desde Ubuntu, `ping 192.168.1.1` debe responder con ~200 ms
durante la condición de 200 ms. Si `ip link show ifb0` no existe (el módulo se descarga entre
sesiones), repite el setup IFB.

Limpieza al terminar:
```bash
sudo tc qdisc del dev ifb0 root
sudo tc qdisc del dev enp1s0 ingress
sudo ip link del ifb0
```

### 3.4 — Tolerancia a fallas de comunicación (2 drones)
```bash
python test_3_4_slave.py            # Ubuntu
python test_3_4_master.py           # Mac
```
Durante el vuelo en formación, **desconecta físicamente el cable Ethernet** unos segundos (se hicieron
3 cortes de 8/10/18 s) y reconéctalo. El SLAVE debe entrar en hover seguro al timeout (5 s) sin diverger.

## OE4 — Simulación (sin drones, sin red)

```bash
python test_4_1_simulation.py       # replica la formación dinámica 2.3 con el modelo identificado
python test_4_2_simulation.py       # replica la degradación 3.3 con un modelo de canal (delay+loss)
```
`test_4_2` imprime una tabla comparando el error simulado contra el real de la 3.3 y discute el
hallazgo central (el retardo no desestabiliza al seguidor). Reproducibles en cualquier máquina.

## OE5 — Integración

### 5.1 — Misión cooperativa completa (2 drones)
```bash
python test_5_1_slave.py            # Ubuntu (PRIMERO)
python test_5_1_master.py           # Mac
```
7 fases: TAKEOFF → WAIT_SLAVE → FORMATION → TRAJECTORY → HOVER_AT_DEST → SLAVE_LAND → MASTER_LAND.
La espera `WAIT_SLAVE` debe ser ≥ 25 s (el seguidor tarda ~15–18 s en entrar al lazo).

### 5.2 — Repetibilidad
Repite la 5.1 varias veces (se hicieron **8**). El análisis estadístico (7/8 válidas) lo produce
`generate_all_figures.py` (`fig_5_2`).

### 5.3 — Registro visual cenital
Graba video con una cámara externa (celular en trípode) durante las repeticiones de 5.2, como *ground
truth* visual independiente del ArUco. Los videos no se versionan por tamaño.

---

## Análisis y figuras (sin hardware)

```bash
python analyze_for_presentation.py  # figuras de OE1 + 2.1 y un results.json
python generate_all_figures.py      # las 10 figuras restantes (2.2 … 5.2)
```
Ambos scripts leen los CSV/JSON de `logs/` (incluidos en el repo) y escriben PNG en
`presentation_assets/`. Son **autocontenidos**: no dependen de rutas absolutas.

---

## Reglas de seguridad

Aprendidas en el laboratorio, muchas a la mala:

- **Nunca volar sin IMU calibrado.** Si el log muestra `No valid imu`, abortar y recalibrar desde la
  app de Ryze (es un paso físico, no del código).
- **Espera 4 s de IMU tras el takeoff** antes de cualquier `move_*` (ya implementado en los scripts).
- **Consenso ⇒ separación mínima ≥ 60 cm**, o los drones se apilan y chocan por *downwash*.
- **No metas la mano entre drones en vuelo.** Para perturbar (prueba 2.1) usa **cartón rígido**.
- **Hélices delicadas:** tras un golpe, revisa y cambia si hay grietas. Ten repuestos.
- **Abortar:** `Ctrl+C` aterriza de forma ordenada y el otro dron lo detecta. Si se atasca,
  `Ctrl+Z` + `kill %1`; en último caso, apaga el Tello físicamente.
- **IP estática:** usa NetworkManager (persistente), no `ip addr add` (se borra al desconectar el cable).
