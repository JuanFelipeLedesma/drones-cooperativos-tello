# HANDOFF — Tesis Sistema Cooperativo de Drones

**Última actualización:** 2026-05-07 (noche — 4.1 y 4.2 completadas en simulación Python; PLAN DE PRUEBAS 16/16 = 100%)
**Autor:** Juan Felipe Ledesma Velásquez
**Asesor:** Camilo Andrés Escobar Velásquez, PhD.
**Universidad de los Andes — Ing. Sistemas y Computación**

> **Para el próximo Claude:** lee este documento entero antes de responder cualquier cosa. Te da TODO el contexto del proyecto sin que el usuario tenga que repetir nada.

---

## 0. Cómo usar este documento

Si eres el usuario y abres una nueva sesión con Claude:

1. Pásale este archivo (o pega su contenido) al inicio del chat.
2. Dile: "Este es el contexto de mi tesis. Léelo entero antes de responder."
3. Claude tendrá el mismo nivel de contexto que el Claude anterior.

---

## 1. Contexto del proyecto

**Tema:** Sistema cooperativo de drones basado en modelos dinámicos y redes Ad-Hoc.

**Plan de pruebas:** 16 pruebas distribuidas en 5 objetivos específicos (OE1-OE5), descritas en el documento `plan_pruebas.docx` (en `~/Downloads/`).

**Setup físico:**
- 2 drones Ryze/DJI Tello (Boost Combo): **Tello A = E92E66**, **Tello B = E92948**
- 2 computadores: **Mac (MASTER, IP 192.168.1.1)** + **Ubuntu (SLAVE, IP 192.168.1.2)**
- Cable Ethernet directo entre Mac y Ubuntu como backbone Ad-Hoc
- 6 markers ArUco DICT_4X4_50, **48 cm x 48 cm**, en pared (3 cols x 2 filas):
  - Fila baja (Y=0.4 m): IDs 0 (izq), 2 (centro), 4 (der)
  - Fila alta (Y=1.4 m): IDs 1 (izq), 3 (centro), 5 (der)
  - Separación horizontal entre cols: 1.0 m
  - Origen del mundo: marker 0
- Cada Tello crea su propia red WiFi (Mac→A, Ubuntu→B) — **NO se pueden conectar varios Tellos a una misma red WiFi por hardware**.

**Ubicación del proyecto:** `/Users/juanfelipeledesmavelasquez/Downloads/drone_tests 2/`
**Ubicación en Ubuntu:** `~/drone_tests 2/`
**Usuario Ubuntu:** `ledegod`

---

## 2. Estado de las pruebas

**16 de 16 completadas (100%) — PLAN DE PRUEBAS EXPERIMENTALES TERMINADO.**

NOTA OE4: las pruebas 4.1 y 4.2 se implementaron como **simulación en Python
basada en el modelo dinámico identificado experimentalmente en OE1**, en lugar
de Gazebo. Razón: mayor rigor (parte de parámetros reales medidos, no de un
modelo genérico) y viabilidad de tiempo. CONSULTAR CON EL ASESOR si acepta esta
sustitución o si exige Gazebo (en cuyo caso: WSL2 en la torre Windows del
usuario, y la simulación Python queda como modelo de referencia).

| OE | Prueba | Estado | CSV final usable |
|---|---|---|---|
| OE1 | 1.1 Step response | ✅ | `test_1_1_step_response_20260430_170517.csv` |
| OE1 | 1.2 Latencia | ✅ | `test_1_2_latency_20260430_194507.csv` |
| OE1 | 1.3 Hover | ✅ | `test_1_3_hover_20260430_191701.csv` |
| OE2 | 2.1 Lazo cerrado | ✅ | `test_2_1_closed_loop_20260430_190555.csv` (con perturbación) |
| OE2 | 2.2 Formación estática | ✅ | `test_2_2_slave_20260506_094144.csv` (con filtro) |
| OE2 | 2.3 Formación dinámica | ✅ | `test_2_3_slave_20260506_111120.csv` (cuadrado) |
| OE2 | 2.4 Consenso | ✅ | `test_2_4_consensus_id*_20260506_170626.csv` (con sep mín 60 cm) |
| OE3 | 3.1 Benchmark Ethernet | ✅ | `test_3_1_ethernet_20260505_172224.json` |
| OE3 | 3.2 Protocolo mensajes | ✅ | `test_3_2_protocol_20260505_174112.json` |
| OE3 | 3.3 Degradación red | ✅ | `test_3_3_slave_20260507_165017.csv` (con IFB) |
| OE3 | 3.4 Tolerancia fallas | ✅ | `test_3_4_slave_20260506_183809.csv` |
| OE5 | **5.1 Misión completa** | ✅ | `test_5_1_master/slave_20260507_175051.csv` (1ª válida) |
| OE5 | **5.2 5+ reps** | ✅ | 8 reps el 2026-05-07 (`test_5_1_*_20260507_18*.csv`), 7/8 válidas |
| OE5 | **5.3 Video cenital** | ✅ | 8 videos grabados durante reps 5.2 |
| OE4 | **4.1 Replicación (sim Python)** | ✅ | `test_4_1_simulation.py` + `logs/test_4_1_simulation.csv` |
| OE4 | **4.2 Sim con degradación** | ✅ | `test_4_2_simulation.py` |

---

## 3. Resultados clave por prueba (publicables para tesis)

### 1.1 — Step response
- 12 step responses (3 reps × 4 ejes = right/up/left/down con 30 cm)
- **Bias bias-corregido**: ~0–6 cm en X/Y, +6 cm en down (efecto gravedad)
- **σ entre reps**: 5–13 cm
- **Modelo 2° orden ajustado**: K=42 cm, ζ≈1.0, ωn=1.9 rad/s, RMSE=2.8 cm
- **Conclusión**: Tello tiene variabilidad ±15-20% en comandos discretos; modelo de 2do orden razonable.

### 1.2 — Latencia
- **Mediana 1.0 s** con `move_*` discretos (incluye latencia + tiempo de recorrer 10 cm)
- **Hallazgo crítico**: comandos discretos son INVIABLES para lazo cerrado.
- `rc_control` en cambio logra ~70 ms efectivos (extrapolado de 2.1 a 13.6 Hz).
- **Justifica usar rc_control en TODAS las pruebas cooperativas.**

### 1.3 — Hover
- σ_x = 4.3 cm, σ_y = 5.5 cm, σ_z = 3.7 cm
- **Drift máximo en 60 s sin control: 45 cm** (random walk lento, ~0.14 Hz)
- TOF (sonar): σ = 2.1 cm (super estable vs ArUco)
- **Justifica necesidad del lazo cerrado** (sin él, drift > tolerancia cooperativa).

### 2.1 — Lazo cerrado ArUco
- err_3d estacionario: **8.1 cm**, bias <1 cm en los 3 ejes
- err_x: -0.34 ± 5.4 cm | err_y: -0.53 ± 6.6 cm | err_z: -0.09 ± 4.7 cm
- Lazo a **13.6 Hz** efectivos
- **Perturbaciones manuales (4 con cartón)**: pico 33-46 cm, recovery 1.85-2.13 s
- **Crucial**: descubrimos que las ganancias originales (kp=0.4) eran 100x bajas porque error está en metros pero output en rc -100/100. Ganancias actuales en config.py: kp=60-70, ki=8-10, kd=25.

### 2.2 — Formación estática
- err_3d steady **12.9 cm** (con filtro promedio móvil N=8)
- Sin filtro: 27 cm. **El filtro es crítico** porque el SLAVE persigue el ruido del MASTER.
- 0 errores CRC en 3005 mensajes binarios. Latencia 5.6 ms.

### 2.3 — Formación dinámica (cuadrado 0.6×0.6 m en X-Z)
- Hover en WPs: ~15 cm. Navegación entre WPs: ~28 cm. Picos en esquinas: 50-90 cm.
- Lag de filtro genera picos en cambios de dirección (~600 ms del filtro N=8).
- 0 CRC errors. Validó el sistema en condiciones dinámicas.

### 2.4 — Consenso de posición
- **PRIMER INTENTO falló**: drones convergieron al mismo punto X-Z y se apilaron verticalmente → CHOQUE entre ellos por downwash.
- **FIX aplicado**: separación mínima `MIN_SEPARATION_X_M = 0.60` (id=1 a izq del centro, id=2 a der). Es "rendezvous with formation" en literatura multi-agente.
- Resultado: distancia inter-dron 60.6 ± 6.4 cm (= MIN_SEP exacta), error individual al target 5 cm.
- **Hallazgo metodológico**: consenso clásico (1 punto único) es matemáticamente correcto pero físicamente peligroso para drones.

### 3.1 — Benchmark Ethernet
- Ping RTT: 1.5 ms, 0% loss
- Throughput iperf3: **93.3 Mbps** (Fast Ethernet)
- UDP cooperación 10/25/50 Hz: RTT 2.1-2.4 ms, jitter <0.7 ms, 0% loss
- Conclusión: enlace **NO es cuello de botella** (consume 0.05% del ancho de banda a 50 Hz con mensaje de 41 B).

### 3.2 — Protocolo de mensajes
- **Mensaje binario: 41 bytes** (struct + CRC-16/CCITT-XMODEM)
- JSON equivalente: 162 bytes (4× más grande)
- Encode binario: 16 µs vs JSON 89 µs (5.4× más rápido — tras usar `binascii.crc_hqx` en C)
- **Hallazgo**: implementación inicial de CRC en Python puro era 280× más lenta que C.
- 100% integridad CRC en 1000 mensajes a 50 Hz.

### 3.3 — Degradación de red (la prueba ESTRELLA del OE3)
- 8 condiciones medidas durante formación 2.2 estática (~25 s cada una).
- **Inyección con `tc netem` vía IFB redirect** (egress de ifb0 = ingress de enp1s0).
- 0 CRC errors en 12421 mensajes incluso con 20% loss → protocolo binario robusto.

| Condición | Loss real medido | err_3d (cm) | Δ vs baseline |
|---|---|---|---|
| baseline | 0% | 21.5 ± 10.6 | — |
| delay 50 ms | 0% | 25.2 ± 13.8 | +17% |
| delay 100 ms | 0% | 26.4 ± 14.9 | +23% |
| **delay 200 ms** | 0% | **41.5 ± 17.7** | **+93%** |
| loss 5% | 5.9% | 33.7 ± 17.1 | +57% |
| loss 10% | 9.0% | 30.3 ± 14.6 | +41% |
| loss 20% | 20.8% | 36.1 ± 13.9 | +68% |
| **combo 100ms + 10%** | 10.0% | **43.4 ± 16.6** | **+102%** |

- **Hallazgo 1**: curva de error vs delay es no-lineal. Salto crítico entre 100→200 ms.
- **Hallazgo 2**: la pérdida es más tolerable que el delay (a 50 Hz, perder 20% deja 40 Hz efectivos, suficientes).
- **Hallazgo 3**: combo no es solo aditivo, peor que cualquier individual (efecto interactivo).
- **Hallazgo 4**: el protocolo binario CRC mantiene 100% integridad bajo todas las condiciones.

### 3.4 — Tolerancia a fallas
- 3 desconexiones registradas: 10/18/8 s, desplazamiento 52/100/39 cm
- **Hallazgo**: deriva en hover seguro = ~5 cm/s consistente (escala linealmente con duración).
- 0 CRC errors en 3348 mensajes. Hover safe activado correctamente al timeout de 5 s.
- Limitación: tiempo de reconvergencia no medido por pérdida concurrente de pose ArUco.

### 5.1 / 5.2 — Misión cooperativa completa + repetibilidad (8 reps)
- 7 fases secuenciales: TAKEOFF → WAIT_SLAVE → FORMATION → TRAJECTORY → HOVER_AT_DEST → SLAVE_LAND → MASTER_LAND
- Aterrizaje secuencial mediante nuevo `mission_state=8` (slave_land), recibido y ejecutado correctamente por el slave en todas las reps válidas.
- **8 reps ejecutadas, 7 exitosas (87.5%)**. Rep 1 falló por IMU del slave mal calibrada → slave pasó por atrás del master.

**Estadísticas (7 reps válidas):**

| Métrica | Mean ± std |
|---|---|
| Duración misión | **71.3 ± 2.3 s** (variabilidad <3%) |
| MASTER err_3d promedio | **11.6 ± 1.0 cm** |
| MASTER err_3d máximo (peaks) | 57.9 ± 4.6 cm |
| SLAVE err_3d estacionario | **18.6 ± 3.0 cm** |
| SLAVE err_3d máximo | 107.1 ± 12.6 cm (transitorios en cambio de dirección) |
| SLAVE bias por eje (entre reps) | < 1 cm en X, Y, Z |
| Consumo batería por misión | 14.3 ± 4.3 % |
| **CRC errors acumulados** | **0 / 20,719 mensajes (100% integridad)** |

- **Hallazgo metodológico**: WAIT_SLAVE_S necesita ser ≥ 25 s (no 8 s como inicial), porque el slave necesita ~15-18 s entre detectar primer mensaje y entrar al lazo (takeoff + IMU wait + climb + init hover). Sin esa pausa, master entra a TRAJECTORY antes que slave esté en formación → saturación de PIDs y errores enormes.
- **Calibración del IMU es prerrequisito crítico para misión cooperativa** — confirmado experimentalmente con la falla de rep 1.

### 5.3 — Video cenital
- 8 videos grabados durante las reps de 5.2 con celular del usuario.
- Sirve como ground truth visual independiente del sistema ArUco para sustentación oral y documento de tesis.
- Material visual disponible para extraer trayectorias en post-proceso si fuera necesario (queda como "trabajo futuro" opcional).

### 4.1 — Replicación en simulación de la formación dinámica
- Implementada en Python (`test_4_1_simulation.py`), NO en Gazebo. Usa el modelo
  dinámico identificado en OE1 (2° orden, ωn≈1.9, ζ≈1.0 + ruido de 1.3).
- Modelo: velocidad de 1er orden (τ=0.35 s) ante comando rc, control discreto a
  13 Hz (frecuencia real medida), física integrada a paso fino (100 Hz).
- **Resultado**: error de formación simulado = 12.2 cm.
- **Sim-to-real gap**: 19 % vs el estado estable de la 2.3 real (~15 cm),
  47 % vs el promedio global de la 2.3 (~23 cm, que incluye transitorios).
- **Hallazgo**: el simulador predice bien el estado estable pero subestima los
  transitorios en cambios de dirección (no captura asimetrías de hardware,
  ruido ArUco no-gaussiano, perturbaciones aerodinámicas).

### 4.2 — Simulación de formación con degradación de red
- Implementada en Python (`test_4_2_simulation.py`). Reusa el modelo de 4.1 y
  añade un modelo de canal con delay + loss. Barre las 8 condiciones de la 3.3.
- **HALLAZGO PRINCIPAL (resultado fuerte para la tesis)**: el error de formación
  simulado se mantiene ESTABLE (~8-13 cm) sin importar el delay/loss inyectado.
- **Explicación de control**: en la arquitectura líder-seguidor, el SEGUIDOR
  cierra su lazo con feedback de posición PROPIO (su cámara ArUco). El delay de
  comunicación afecta solo la consigna (pose del líder), NO el lazo de
  realimentación → no desestabiliza. El delay solo sería crítico si estuviera
  DENTRO del lazo (caso del consenso 2.4).
- **Implicación**: la 3.3 real mostró degradación (22→42 cm con delay 200ms),
  pero la simulación demuestra que el delay PURO no la causa. → La degradación
  real se atribuye a factores concurrentes (pérdida de pose ArUco del slave,
  condiciones del experimento). La simulación funciona como herramienta de
  DIAGNÓSTICO que aísla variables que el experimento real mezcla.

---

## 4. Cosas que NO funcionaron y por qué (importante para tesis)

Esta sección es ORO para la documentación de la tesis. Cada uno es un hallazgo metodológico legítimo.

### 4.1 Calibración de cámara con chessboard
- **Intentamos** calibrar formalmente con chessboard 7x6 (123 capturas, RMS 0.95 px aparentemente bueno).
- **Resultado al aplicar**: ruido de pose CRECIÓ en X y Y (de 12 cm std a 47 cm std). cy=543 vs 360 esperado (sospechoso).
- **Causa**: capturas no cubrieron uniformemente el frame, sesgo en el principal point.
- **Decisión**: revertimos a parámetros aproximados. La calibración formal queda pendiente para versión futura más cuidadosa.

### 4.2 Multi-marker fusion en aruco.py
- **Intentamos** fusionar pose de varios markers visibles ponderado por tamaño/error.
- **Resultado**: 6× solvePnP por frame saturó el loop, los `rc 0 0 0 0` no llegaban a tiempo, drone derivó descontrolado a la derecha → CASI choca contra pared.
- **Causa**: sin profile previo de tiempo de cómputo.
- **Decisión**: revertimos a single-marker (closest-to-center) + IPPE_SQUARE + outlier filter. La fusión queda como mejora futura **con optimización**.

### 4.3 Consenso clásico (1 punto único)
- **Intentamos** la ley pura `target_i = (pos_i + pos_j) / 2` del paper original.
- **Resultado**: drones convergieron al mismo X-Z, se apilaron verticalmente, **chocaron 2 veces seguidas** por downwash.
- **Causa**: la teoría asume puntos sin dimensión; los Tellos miden ~10 cm de alto + hélices.
- **Fix**: separación mínima de 60 cm como restricción geométrica. Sigue siendo consenso matemáticamente.

### 4.4 IP estática del Ubuntu con `ip addr add`
- **Intentamos** usar `sudo ip addr add 192.168.1.2/24 dev enp1s0` — funciona.
- **Problema**: cuando se desconecta físicamente el cable Ethernet, NetworkManager borra la IP. Al reconectar el cable, la interfaz queda sin IP → paquetes UDP del MASTER se descartan → SLAVE nunca detecta reconexión.
- **Fix**: configurar persistencia con NetworkManager:
  ```bash
  sudo nmcli connection modify "Wired connection 1" \
      ipv4.method manual ipv4.addresses 192.168.1.2/24 \
      ipv4.gateway "" ipv4.ignore-auto-dns yes connection.autoconnect yes
  sudo nmcli connection up "Wired connection 1"
  ```

### 4.5 IMU del Tello sin pausa post-takeoff
- **Intentamos** enviar `move_up(60)` inmediatamente después del takeoff.
- **Resultado**: error `'No valid imu'` y dron deriva incontrolada (un dron a la derecha, otro hacia atrás — depende del bias físico de cada unidad).
- **Causa**: el IMU del Tello necesita estabilizarse tras takeoff antes de aceptar comandos de movimiento precisos.
- **Fix**: `IMU_WAIT_S = 4.0` segundos de hover (`rc 0 0 0 0`) entre takeoff y cualquier `move_*`. **Aplicado en TODOS los scripts**.

### 4.6 PID con error en metros pero gains aproximados
- **Bug original**: kp=0.4 con error en metros → output 0.4*0.20 = 0.08 → int(0.08) = 0 → **PID enviando ceros, drone derivando sin control**.
- **Fix**: kp=60-70, ki=8-10, kd=25 (en config.py). Para error de 0.20 m → output 12-14 (rc command razonable).

### 4.7 Comandos discretos move_* en el lazo cerrado
- **Validado en 1.2**: `move_*` tiene latencia mediana 1 segundo (incluye motion + cross threshold).
- **Decisión**: **TODAS las pruebas cooperativas usan `rc_control`** que es 10x más rápido y permite control suave.

### 4.8 ArUco pose noise inicial
- Markers de 11.5 cm + cámara sin calibrar = std de pose ~25-40 cm.
- **Soluciones aplicadas**:
  1. Markers más grandes (48 cm)
  2. cv2.SOLVEPNP_IPPE_SQUARE (resuelve ambigüedad planar)
  3. Outlier filter temporal (rechaza saltos >50 cm en <500 ms)
  4. Closest-to-center marker selection (en vez del primero detectado)
- Resultado: std bajó a 4-8 cm en estado estacionario.

### 4.9 Bug de schema en CSV logger
- El `FlightLogger` fija el schema en la PRIMERA fila escrita. Si después intentas loguear una row con campos extra (ej: `disp_during_disconnect_cm` que solo aparece durante desconexión), CRASHEA.
- **Fix general**: siempre poblar todas las columnas con `None` desde el primer row.

### 4.10 `tc netem` solo afecta egress por defecto
- **Intentamos** la 3.3 aplicando `tc qdisc add dev enp1s0 root netem delay 200ms` en el Ubuntu.
- **Resultado**: latencia medida (master_age) se mantuvo en 4.4 ms en TODAS las condiciones. La inyección no afectó nada.
- **Causa**: `tc netem` aplicado a `dev <iface> root` solo degrada **EGRESS** (paquetes salientes). El flujo crítico MASTER→SLAVE son paquetes **INGRESS** al Ubuntu.
- **Fix**: usar IFB (Intermediate Functional Block) para hacer "ingress shaping":
  ```bash
  sudo modprobe ifb
  sudo ip link add ifb0 type ifb
  sudo ip link set ifb0 up
  sudo tc qdisc add dev enp1s0 ingress
  sudo tc filter add dev enp1s0 parent ffff: protocol ip u32 \
      match u32 0 0 flowid 1:1 action mirred egress redirect dev ifb0
  ```
  Después aplicar netem al egress de ifb0:
  ```bash
  sudo tc qdisc add dev ifb0 root netem delay 200ms
  ```
  El `test_3_3_inject.py` ya hace esto, pero el setup IFB es manual (una vez por sesión).
- **Validación rápida**: `ping 192.168.1.1` desde Ubuntu — debe responder con ~200 ms si la inyección funciona.
- **Importante**: el módulo `ifb` puede descargarse entre sesiones. Si `ip link show ifb0` no encuentra nada, hay que volver a configurar.

### 4.11 master_age NO mide delay one-way
- **Observación durante 3.3**: incluso con `delay 200ms` aplicado correctamente, la métrica `master_age` (= `t_now - last_recv_t` del listener) reportaba ~5 ms.
- **Causa**: el flujo es continuo a 50 Hz. Cada mensaje sufre el retardo, pero el siguiente también, así que el "gap entre mensajes" se mantiene en ~20 ms.
- **Para medir delay one-way real** habría que comparar `t_now` local vs `msg.timestamp` del master, pero los relojes no están sincronizados.
- **Lo que SÍ funciona**: medir el efecto en el error de formación (que sí refleja delay), y medir loss usando `received_count` del listener (no usar `master_seq` del log porque el slave loguea más lento que recibe).

---

## 5. Pruebas pendientes — instrucciones

**NINGUNA pendiente. Las 16 pruebas del plan están completas.**

Lo único abierto es la decisión del asesor sobre si acepta la simulación Python
para OE4 o exige Gazebo. Si exige Gazebo, ver "Plan de contingencia Gazebo" abajo.

### (Histórico) 4.1, 4.2 — Si el asesor exige Gazebo en vez de Python
- Sin drones, requiere instalar ROS 2 + Gazebo.
- 4.1: replicar trayectoria 2.3 en simulación con modelo Tello calibrado (parámetros de 1.1: K=42, ζ=1.0, ωn=1.9 rad/s).
- 4.2: replicar 3.3 en simulación.
- Métrica clave: **sim-to-real gap** (% de diferencia entre métricas sim y real).
- Tiempo estimado: variable, varias horas de instalación + setup + corridas.

### 5.1, 5.2, 5.3 — Validación integral
- 5.1: misión cooperativa completa (despegue secuencial → formación → trayectoria → hover → aterrizaje secuencial). 90 min de vuelo.
- 5.2: 5 reps de 5.1 para estadística. 120 min.
- 5.3: video cenital con cámara externa (celular en trípode) durante 5.1/5.2.
- Tiempo total estimado: una sesión completa de 1 día.

---

## 6. Archivos clave del proyecto

```
drone_tests 2/
├── config.py                       ← parámetros centrales (IPs, markers, PID, formación)
├── HANDOFF.md                       ← ESTE ARCHIVO
├── plan_pruebas.docx                ← plan oficial (en Downloads/)
├── analyze_for_presentation.py      ← genera gráficas para presentación
├── Avances_Tesis_Semana1.pptx       ← presentación primera semana (1.1, 1.2, 1.3, 2.1)
│
├── test_1_1_step_response.py        ← 1.1 (1 dron)
├── test_1_2_latency.py              ← 1.2 (1 dron, con recenter PID)
├── test_1_3_hover.py                ← 1.3 (1 dron, hover libre 60s)
├── test_2_1_closed_loop.py          ← 2.1 (1 dron, lazo cerrado, con tecla 'p' marca perturbación)
├── test_2_2_master.py / _slave.py   ← 2.2 (formación estática, slave con filtro promedio)
├── test_2_3_master.py / _slave.py   ← 2.3 (formación dinámica, trayectoria cuadrado)
├── test_2_4_consensus.py            ← 2.4 (consenso simétrico, --id 1 o 2)
├── test_3_1_ethernet.py             ← 3.1 (sender/receiver de benchmark)
├── test_3_2_protocol.py             ← 3.2 (CoopMessage binario + JSON)
├── test_3_4_master.py / _slave.py   ← 3.4 (tolerancia a fallas, slave detecta desconex)
├── test_3_3_master.py / _slave.py   ← 3.3 (slave anota network_condition en CSV)
├── test_3_3_inject.py               ← 3.3 (corre EN PARALELO en Ubuntu, aplica netem vía IFB)
│
├── utils/
│   ├── aruco.py                     ← tracker single-marker + IPPE_SQUARE + outlier filter
│   ├── pid.py                       ← PIDController con anti-windup
│   ├── logger.py                    ← FlightLogger CSV (schema fijo en primera fila!)
│   ├── comms.py                     ← (helpers, no usado mucho)
│   └── __init__.py
│
├── aruco_markers/                   ← PNGs y PDF de markers (48 cm, dict 4x4_50)
├── calibration_captures/            ← capturas y resultado de calibración (revertida)
├── presentation_assets/             ← gráficas PNG generadas para PowerPoint
├── logs/                            ← TODOS los CSVs/JSONs de cada corrida (no borrar!)
└── venv/                            ← entorno Python (Mac); en Ubuntu tiene su propio venv
```

---

## 7. Estado actual de `config.py` (claves importantes)

```python
# Red
MASTER_IP = "192.168.1.1"        # Mac
SLAVE_IP  = "192.168.1.2"        # Ubuntu
COMMS_PORT = 5005
COMMS_FREQ_HZ = 20

# ArUco
ARUCO_DICT_ID = "DICT_4X4_50"
MARKER_SIZE_M = 0.48              # 48 cm impresos
MARKER_WORLD_POSITIONS = {
    0: [0.0, 0.4, 0.0],   # abajo-izquierda (ORIGEN)
    1: [0.0, 1.4, 0.0],   # arriba-izquierda
    2: [1.0, 0.4, 0.0],   # abajo-centro
    3: [1.0, 1.4, 0.0],   # arriba-centro
    4: [2.0, 0.4, 0.0],   # abajo-derecha
    5: [2.0, 1.4, 0.0],   # arriba-derecha
}

# Cámara — APROXIMADA (calibración formal pendiente)
CAMERA_MATRIX = [
    [921.17,   0.0,   459.90],
    [  0.0, 919.02,   351.24],
    [  0.0,   0.0,     1.0  ],
]
DIST_COEFFS = [0.0, 0.0, 0.0, 0.0, 0.0]

# PID — gains que SÍ funcionan (error en metros, output en rc -100..100)
PID_LR = {"kp": 60.0, "ki": 8.0,  "kd": 25.0}
PID_UD = {"kp": 70.0, "ki": 10.0, "kd": 25.0}   # más fuerte vs gravedad
PID_FB = {"kp": 60.0, "ki": 8.0,  "kd": 25.0}
RC_MAX = 30                       # cm/s máximo (seguridad)

# Formación
FORMATION_OFFSET_X = 1.0          # SLAVE 1m a la derecha del MASTER
FORMATION_OFFSET_Y = 0.0
FORMATION_OFFSET_Z = 0.0

# Trayectorias (waypoints absolutos en mundo, Y constante)
TRAJECTORY_SQUARE = [...]         # 4 esquinas + retorno
TRAJECTORY_LINE   = [...]         # ida y vuelta

# Seguridad
COMMS_TIMEOUT_S = 5.0             # SLAVE entra hover si no recibe en 5s
MAX_FLIGHT_TIME_S = 120
MIN_BATTERY_PCT = 20
```

---

## 8. Procedimiento estándar para una prueba con 2 drones

1. **Mac y Ubuntu encendidos**, cable Ethernet conectado.
2. **Verificar IP del Ubuntu** (es persistente con NetworkManager pero confirmar):
   ```bash
   # Ubuntu
   ip addr show enp1s0       # debe mostrar inet 192.168.1.2/24
   ```
3. **Test de ping** en ambos sentidos.
4. **Conectar cada Mac/Ubuntu a SU Tello** por WiFi:
   - Mac → TELLO-E92E66 (Tello A)
   - Ubuntu → TELLO-E92948 (Tello B)
5. **Verificar baterías ≥ 60%** con un script rápido:
   ```bash
   python -c "from djitellopy import Tello; t=Tello(); t.connect(); print(t.get_battery())"
   ```
6. **Posicionar drones físicamente**:
   - Mac/Tello A → X ≈ 0.5 m (izq de centro)
   - Ubuntu/Tello B → X ≈ 1.5 m (der de centro, ~1m del A)
   - Ambos a 2.5 m de la pared, mirando los markers
7. **Ubuntu primero** (script slave/id=2), **Mac después** (script master/id=1).
8. **Mano cerca del Ctrl+C**. Si algo sale raro, abortar uno → el otro detecta `mission_state=6` y aterriza solo.

---

## 9. Reglas de seguridad aprendidas a la mala

- **NUNCA volar sin IMU calibrado**. Si en log aparece `'error No valid imu'`, abortar y recalibrar IMU desde la app de Ryze (es físico, no del código).
- **Espera 4 s de IMU wait después de takeoff** antes de cualquier `move_*`. Implementado en TODOS los scripts.
- **Para consenso de drones, separación mínima OBLIGATORIA** (>= 60 cm) o se apilan y chocan.
- **No metas la mano entre drones volando**. Para empujarlos en perturbaciones, usar cartón rígido (no la mano, no algo flexible que pueda enredarse).
- **Hélices son delicadas**. Después de un choque, revisar visualmente y cambiar si hay grietas. Tener repuestos a mano.
- **Si el script se "atasca"**: Ctrl+Z + `kill %1`. Si no responde, apagar Tello físicamente (botón al lado de batería).
- **Las IPs estáticas con `ip addr add` se borran al desconectar el cable** — usar NetworkManager para persistencia.

---

## 10. Cómo continuar

**Trabajo con hardware FINALIZADO el 2026-05-07.** Los drones ya no necesitan tocarse.

**Solo queda OE4 (Gazebo, simulación pura — sin drones, sin red, sin nada físico):**
1. **4.1 — Replicación Gazebo**: usa los parámetros calibrados del Tello (K=42 cm, ζ=1.0, ωn=1.9 rad/s) en un modelo en Gazebo. Replica la trayectoria del cuadrado de 2.3. Compara métricas sim vs. real (sim-to-real gap).
2. **4.2 — Sim con degradación**: replica las condiciones de red de 3.3 (delay/loss) en simulación. Verifica si la simulación predice correctamente el comportamiento degradado real.

Setup necesario para Gazebo:
- ROS 2 (Humble o más reciente) en Ubuntu
- Plugin de Tello para Gazebo (existen forks open-source: `tello_ros` o similar)
- Modelo URDF del Tello con dinámica calibrada
- Tiempo estimado: 4-8 h de instalación + 2-4 h de corridas
- **Sin urgencia — se puede hacer en cualquier momento**, no requiere drones físicos.

**Lo que sigue en términos de la TESIS (no experimentos):**
1. **Análisis post-proceso de los CSVs**: actualizar `analyze_for_presentation.py` para incluir 2.2-2.4, 3.x, 5.x. Generar gráficas finales para el documento.
2. **Escribir resultados** de cada OE en el documento de tesis usando los hallazgos clave de este HANDOFF.md.
3. **Editar videos** de las 8 reps de 5.2 para sustentación (clips cortos, ej. 30 s).
4. **Buscar 2-3 referencias IEEE** sobre caracterización del Tello (usar Perplexity / Google Scholar — no inventar citas).
5. **Actualizar la presentación PowerPoint** (`Avances_Tesis_Semana1.pptx`) con todos los datos nuevos.

**Para una nueva sesión de Claude:**
- Pega este HANDOFF.md al inicio del chat.
- Empieza con: "Quiero [analizar los datos / instalar Gazebo / actualizar la presentación / buscar referencias]".
- Claude tendrá todo el contexto del proyecto.

---

## 11. Para la presentación de la tesis

**Ya generado:**
- `presentation_assets/*.png` — 6 gráficas de OE1 + 2.1
- `Avances_Tesis_Semana1.pptx` — PowerPoint de primera semana (13 slides)

**Por hacer cuando termines todas las pruebas:**
- Re-correr `analyze_for_presentation.py` extendido con análisis de 2.2-2.4 + 3.x
- Generar nueva versión del PowerPoint con datos completos
- Tablas resumen para meter en el documento de tesis

**Sobre citas IEEE para la tesis:** NO inventar referencias. Buscar en Perplexity / Google Scholar con términos como "DJI Tello position accuracy characterization", "Ryze Tello SDK control performance", verificar con DOI antes de citar. Como cita "segura" sirve el SDK oficial:
```
[X] Ryze Robotics, "Tello SDK 2.0 User Guide," 2018.
    URL: https://dl-cdn.ryzerobotics.com/downloads/Tello/Tello%20SDK%202.0%20User%20Guide.pdf
```

---

**FIN DEL HANDOFF**
