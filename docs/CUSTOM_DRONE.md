# Dron personalizado: XIAO ESP32-S3 como puente de sensores a iNav

> Documentación técnica del dron construido a partir de componentes
> individuales, complementario al sistema cooperativo principal basado en los
> drones Tello. Cubre el inventario, la arquitectura final (el XIAO ESP32-S3
> como **puente de sensores** que entrega rangefinder y magnetómetro a una
> controladora de vuelo iNav vía MSP v2) y las iteraciones del desarrollo.
> El código fuente referenciado vive en [`firmware/`](../firmware/) y un índice
> rápido por carpeta está en [`firmware/README.md`](../firmware/README.md).

## 1. Introducción y objetivo

Como complemento al sistema cooperativo principal basado en los drones
Ryze/DJI Tello, se construyó un **dron personalizado** a partir de componentes
individuales. El objetivo de este desarrollo fue doble: por un lado, contar con
una plataforma abierta sobre la que se pudiera modificar libremente la cadena
de control (a diferencia del Tello, que es una caja cerrada con SDK limitado);
por otro, prototipar un **sistema auxiliar de sensado** que pudiera integrarse
en un futuro como un nodo más del esquema cooperativo.

El producto final del desarrollo es un dron donde la **controladora de vuelo
(FC) corre iNav 9.0.1** sobre un microcontrolador STM32F405 (target
OMNIBUSF4PRO) y un **microcontrolador auxiliar XIAO ESP32-S3 Sense** se conecta
a la FC vía UART para entregarle dos sensores adicionales que la FC no tiene de
fábrica: un **medidor de distancia VL53L0X** (para altitud) y un
**magnetómetro QMC5883L** (para heading). Adicionalmente, el XIAO expone la
**cámara OV2640 integrada como stream MJPEG por WiFi**, en paralelo con el
bridge de sensores. La comunicación XIAO ↔ iNav utiliza el protocolo MSP v2
estándar de iNav/Betaflight.

## 2. Inventario final de componentes

La tabla siguiente presenta los componentes utilizados en la versión final del
dron. Varios componentes del inventario inicial fueron sustituidos durante el
desarrollo por razones técnicas que se justifican al pie de la tabla.

| N.º | Componente | Especificación | Función |
|:---:|---|---|---|
| 1 | **IMU MPU-6050 GY-521** | Acelerómetro + giroscopio de 3 ejes (6 DOF), I²C | Medición de actitud y velocidades angulares para la FC |
| 2 | **Hélices Gemfan 51499** (4 colores) | Tri-pala, **5 pulgadas** (5.1″ × 4.9″), CW/CCW | Propulsión; geometría dimensionada para motores clase 22xx |
| 3 | **Sensor de distancia VL53L0X** | ToF infrarrojo (λ = 940 nm), I²C, alcance ~2 m | Medición de altura sobre el suelo, entregada a la FC vía MSP |
| 4 | **Módulo GPS HGLRC M100** | Chipset u-blox M10, comunicación por UART, magnetómetro QMC5883L integrado por I²C | Posición geográfica (UART → FC) y heading (I²C → XIAO → FC vía MSP) |
| 5 | **Motores brushless DYS SF2207-2750KV** (×4) | Estator 22 × 07 mm, **2750 KV**, compatible 2S–6S | Propulsión |
| 6 | **Frame de fibra de carbono** | Estructura tipo FPV con correa de batería | Estructura mecánica |
| 7 | **Radio FlySky FS-i6X** | 6 canales, protocolo IBUS / AFHDS 2A | Control manual del piloto |
| 8 | **Microcontrolador XIAO ESP32-S3 Sense** | SoC ESP32-S3 dual-core, Wi-Fi + BLE, cámara OV2640 integrada | Bridge MSP (VL53L0X + magnetómetro), stream WiFi de cámara |
| 9 | **Baterías LiPo Tattu Funfly 4S 1300 mAh 100C** (×2) | 14.8 V nominal, conector XT60 | Alimentación |
| 10 | **Flight Controller (FC) basada en STM32F405** | Target **OMNIBUSF4PRO** en iNav 9.0.1; entradas para receptor SBUS/PPM/IBUS, salidas M1–M4 PWM/DSHOT, UART para GPS, UART para MSP del XIAO, barómetro BMP280 integrado | Estabilización, mezcla, control de vuelo, integración de sensores externos |
| 11 | **ESC AIO Aero Selfie** (4 en 1) | Cuatro controladores de velocidad en una sola PCB | Conmutación de potencia hacia los motores |

### Sustituciones respecto al inventario inicial

- **Hélices (N.º 2):** Las Gemfan Hurricane 3016-3 (3″) se reemplazaron por las
  **Gemfan 51499 de 5″** porque los motores adoptados son de clase 2207 —
  dimensionados para hélices de 5″, no de 3″.
- **Sensor de posicionamiento (N.º 3):** El sensor de flujo óptico inicial
  mide desplazamiento horizontal relativo al suelo, lo cual no era aprovechable
  para el control de altitud. Se sustituyó por un **VL53L0X**, que mide
  directamente la altura sobre la superficie inferior y se entrega al iNav
  como sensor virtual de rangefinder.
- **Motores (N.º 5):** Los HGLRC Specter 1303.5 5500 KV son adecuados para
  drones *toothpick* con hélices de 3″. Al adoptar hélices de 5″, se cambiaron
  por **DYS SF2207-2750 KV**, una clase de motor más grande y de menor KV
  adecuada para esa hélice y para una batería de mayor tensión.
- **Baterías (N.º 9):** Las LiPo 2S 3000 mAh 25C iniciales fueron sustituidas
  por **LiPo Tattu Funfly 4S 1300 mAh 100C** (14.8 V, 100C) para cubrir la
  demanda de potencia de los motores 2207/2750KV.
- **Asignación FC / ESC:** En el inventario inicial el componente N.º 10 se
  describió como "FC + ESC integrado". En la práctica, la **Aero Selfie se
  utiliza únicamente como ESC** 4-en-1; la **FC es una placa independiente**
  basada en STM32F405 (target OMNIBUSF4PRO en iNav).
- **Sensores adicionales no listados en el inventario inicial:** El **VL53L0X**
  y, en una iteración posterior, el **magnetómetro QMC5883L** (integrado en el
  módulo GPS HGLRC M100 pero no detectado por la FC en su bus I²C nativo)
  fueron incorporados por el autor por fuera del inventario entregado.

## 3. Arquitectura final del sistema

La cadena de control del dron personalizado opera en su mayor parte como un
quad FPV estándar (radio → FC → ESC → motores). La contribución de este
desarrollo es **insertar al XIAO ESP32-S3 como un proveedor de sensores
adicionales** que la FC consume vía MSP v2, sin que el XIAO toque la cadena
de control de motores.

```
   Radio FlySky FS-i6X
            │ IBUS (CH 5 = ARM, CH 6 = ANGLE)
            ▼
   ┌────────────────────────────────┐         ┌─────────────────────────┐
   │ FC iNav 9.0.1 (OMNIBUSF4PRO)   │ ◄──UART─│  XIAO ESP32-S3 Sense    │
   │   - MPU6500 (gyro/acc, I²C)    │   MSP v2│   - VL53L0X (I²C0)      │
   │   - BMP280  (baro, I²C)        │ @115200 │   - QMC5883L (I²C0)     │
   │   - GPS HGLRC M100 (UART)      │   30 Hz │   - OV2640 (SCCB I²C1)  │
   │   - Mezcla X, DSHOT300         │         │   - Wi-Fi AP "DRONE_CAM" │
   └──────┬────────────────────────┘         └────────────┬────────────┘
          │ DSHOT300 a M1..M4                              │ Stream MJPEG
          ▼                                                ▼
   ┌──────────────┐   ┌────────────────┐         ┌──────────────────┐
   │ ESC Aero     │──►│ 4 motores      │         │ PC: navegador →  │
   │ Selfie 4-in-1│   │ DYS SF2207     │         │ http://192.168   │
   └──────────────┘   │ + Gemfan 51499 │         │       .4.1/      │
                     └────────────────┘         └──────────────────┘
```

Esta arquitectura tiene tres propiedades importantes:

1. **El XIAO no participa del control de motores.** Si el XIAO falla o se
   reinicia, la FC sigue volando con los sensores que sí tiene de fábrica
   (gyro, acc, baro, GPS). El bridge MSP sólo aporta *sensores adicionales*;
   la FC trata su ausencia exactamente igual que una desconexión de cualquier
   periférico de telemetría.
2. **iNav nativo hace todo el control de vuelo** (estabilización, mezcla,
   modos Angle/Altitude Hold/Position Hold). El XIAO no necesita implementar
   ni PIDs ni mezclas — sólo proveer datos crudos en el formato que iNav
   espera.
3. **La cámara OV2640 del XIAO Sense corre en paralelo** con el bridge MSP,
   sirviendo un stream MJPEG por WiFi accesible desde un navegador. Esto la
   habilita como herramienta de inspección/monitoreo (no como FPV de baja
   latencia, ver Limitaciones).

## 4. Configuración de la controladora de vuelo (iNav 9.0.1)

La FC se configuró con el target **OMNIBUSF4PRO** del firmware **iNav 9.0.1**
(commit `d44f2cf6`, compilado el 2026-02-13). Los parámetros relevantes
quedaron registrados en tres archivos de configuración (ver Anexo A para el
índice completo):

- **`dump_v7.txt`** — `dump` completo de la configuración: 2216 líneas con todos
  los `set ...`, mezcla de motores, perfiles de control, batería y servos.
  Es el archivo que se reaplica vía CLI para restaurar la FC a un estado
  conocido.
- **`diff_all_v7.txt`** — `diff all` de los cambios respecto a los valores por
  defecto: contiene exclusivamente los parámetros modificados (más legible).
- **`cambios_v7_estable_4.txt`** — bitácora de los cambios manuales que
  llevaron a la configuración estable (versión "Estable 4").

Aspectos relevantes de la configuración:

| Parámetro | Valor | Observación |
|---|---|---|
| Target | OMNIBUSF4PRO | MCU STM32F405 |
| Firmware | iNav 9.0.1 | Reemplazó a Betaflight 2025.12.2 |
| Hardware acelerómetro | `acc_hardware = MPU6500` | Calibrado: `acczero_x=-12, y=64, z=-78` |
| Hardware giroscopio | MPU6500 | Cero: `gyro_zero_x=13, y=-20, z=15` |
| Barómetro | `baro_hardware = BMP280` | Sensor por defecto de altitud |
| Magnetómetro nativo | `mag_hardware = NONE` | **No detectado** en el bus I²C de la FC; entregado por el XIAO vía MSP |
| Sensor de altitud por defecto | `inav_default_alt_sensor = BARO` | Posible alternar a RANGEFINDER (MSP) |
| Receptor | `receiver_type = SERIAL`, `serialrx_provider = IBUS` | Conexión IBUS desde el FS-i6X |
| Protocolo a ESC | `motor_pwm_protocol = DSHOT300` | Salida digital a la Aero Selfie |
| Mezcla de motores (X) | `mmix 0..3` con signos `+/-` por cuadrante | Configuración de quadcopter en X |
| Receptor — canales | CH5 = ARM (1000), CH6 = ANGLE (1500/2000) | Definido en bitácora `cambios_v7_estable_4.txt` |
| Failsafe | `failsafe_procedure = DROP` | Comportamiento ante pérdida de RC |

Para que el iNav consuma los sensores entregados por el XIAO, se configuran
adicionalmente vía CLI:

```text
set rangefinder_hardware = MSP    # acepta MSP2_SENSOR_RANGEFINDER (0x1F01)
set mag_hardware          = MSP   # acepta MSP2_SENSOR_COMPASS    (0x1F04)
set align_mag             = CW0
save
```

y la UART correspondiente del XIAO se habilita con la función **MSP** desde la
pestaña *Ports* del configurador.

## 5. Iteraciones del firmware del XIAO ESP32-S3

El desarrollo del firmware del XIAO atravesó **dos rutas arquitectónicamente
distintas** antes de converger a la versión final. Esta sección documenta
ambas porque el proceso es ilustrativo de las decisiones de diseño que
configuran un sistema embebido fiable.

### 5.1 Ruta inicial — descartada: el XIAO como puente PWM físico FC → ESC

La primera ruta exploró usar al XIAO como **puente físico** entre la FC y los
ESC: el XIAO leía las 4 señales PWM que la FC genera para los motores, las
procesaba y las reenviaba a los ESC, con la intención de inyectar correcciones
de altura en línea.

Se desarrollaron tres iteraciones sucesivas en esta ruta:

1. **`01_vuelo_pwm.ino`** — versión mínima del puente: captura las 4 PWM por
   **interrupciones** en flanco de subida/bajada (la primera versión usó
   `pulseIn()`, descartada por ser bloqueante) y las reenvía con
   `analogWriteFrequency()` + `analogWrite()` a 400 Hz. Escala experimental
   `1000–2000 µs ↔ analogWrite 100–200`. Failsafe simple por timeout
   (10 ms sin pulso → comando a 1000 µs).
2. **`02_vuelo_pwm_lectura_sensor.ino`** — extiende lo anterior añadiendo
   lectura del VL53L0X en una **tarea de FreeRTOS fijada al núcleo 0**
   (la lectura I²C en el mismo lazo que los motores los hacía trabarse). El
   sensor sólo se lee y se reporta por serial; no afecta a los motores.
3. **`03_vuelo_pwm_pid_altura.ino`** — agrega un **PID de altura** que se
   activa al cruzar 50 cm desde abajo y permanece activo durante 30 s,
   inyectando una corrección colectiva en microsegundos a los 4 canales antes
   de escribir a los ESC. Incluye zona muerta, suavizado exponencial asimétrico
   (subir lento, bajar rápido), límite por muestra y anti-windup en el
   integrador.

**Por qué se descartó esta ruta.** El comportamiento del PID era errático y un
ajuste fino requería un trabajo experimental que excedía el alcance del
proyecto. Más importante aún, esta arquitectura es **frágil por diseño**:
cualquier fallo del XIAO (un reset, una corrupción del bus I²C, un *crash* del
firmware) interrumpe la cadena de PWM hacia los motores y el dron pierde
control. La ruta MSP que se describe a continuación es estructuralmente más
segura.

### 5.2 Ruta adoptada: el XIAO como bridge de sensores MSP v2

En lugar de interponerse en el camino crítico FC → ESC, el XIAO se conecta a
una **UART libre** de la FC y le entrega sensores adicionales usando el
protocolo **MSP v2** estándar de iNav/Betaflight. La FC los acepta como
sensores virtuales (`rangefinder_hardware = MSP`, `mag_hardware = MSP`) y los
usa exactamente igual que si vinieran de un periférico físico conectado a sus
buses propios.

El protocolo **MSP v2** encapsula la trama en un encabezado `$ X <`, función
(16 bits little-endian), tamaño (16 bits LE), payload y un CRC-8/DVB-S2 al
final. Para sensores se usaron dos funciones estándar de iNav:

- **`MSP2_SENSOR_RANGEFINDER` (`0x1F01`)** — payload de 5 bytes:
  `quality (u8) + distance_mm (i32 LE)`. `distance < 0` indica fuera de rango.
- **`MSP2_SENSOR_COMPASS` (`0x1F04`)** — payload de 11 bytes:
  `instance (u8) + time_ms (u32 LE) + magX, magY, magZ (i16 LE)`.

Se desarrollaron cinco iteraciones en esta ruta:

1. **`v1_msp_rangefinder_bridge.ino`** — versión mínima: sólo VL53L0X enviado
   por MSP a 30 Hz, sin filtrado. Establece el cableado base
   `XIAO D6 (GPIO43) → FC UART RX`, comparten GND, y la configuración mínima
   en iNav.
2. **`v2_msp_rangefinder_bridge.ino`** — agrega un **promedio móvil opcional**
   sobre la distancia. Incluye documentación explícita del *trade-off* (un
   promedio de N muestras reduce ruido en ~1/√N pero introduce un retraso de
   (N−1)/2 muestras, lo cual a 30 Hz son ~67 ms con N = 5 — relevante porque
   ese retraso queda dentro del lazo de altitud de iNav).
3. **`v3_msp_rangefinder_bridge.ino`** — añade el **magnetómetro** (HMC5883L
   o QMC5883L según el módulo) por `MSP2_SENSOR_COMPASS`. El script imprime
   un escaneo I²C al arranque para identificar qué variante está conectada.
4. **`v3_1_msp_rangefinder_bridge.ino`** — fix del bug del v3: si el
   magnetómetro no responde, el v3 colgaba el bus I²C y tumbaba también la
   lectura del rangefinder. El v3.1 hace el magnetómetro **fail-safe**: se
   prueba una vez al arranque y sólo se lee si efectivamente respondió;
   adicionalmente, se cambia el patrón I²C a **STOP** en lugar de
   *repeated-start* y se configura un *timeout* en `Wire.setTimeOut(25)`.
5. **`v4_msp_rangefinder_bridge.ino`** — versión limpia para el airframe final:
   se eliminan el escáner y la variante HMC5883L, dado que el hardware quedó
   confirmado en VL53L0X = `0x29` y QMC5883L = `0x0D`. Es la versión
   "production" del bridge MSP independiente.

### 5.3 Cámara OV2640 — sketch standalone

En paralelo al desarrollo del bridge, se implementó la captura y el streaming
de la cámara OV2640 integrada del XIAO Sense:

- **`xiao_camera_wifi_stream.ino`** — sketch independiente que pone al XIAO
  en modo Access Point (SSID `DRONE_CAM`, password `drone12345`), inicializa
  la cámara con PSRAM OPI y resolución VGA (640 × 480, JPEG quality 12), y
  expone un servidor HTTP en `http://192.168.4.1/` con un stream MJPEG
  (multipart/x-mixed-replace) en `/stream`. Está documentado que **el alcance
  WiFi de la antena del XIAO es de decenas de metros con visión directa y cae
  rápido**, lo que limita la cámara a inspección de banco/corto alcance
  (~100–300 ms de latencia), no a FPV de largo alcance o baja latencia.

### 5.4 Versión final integrada — `drone_xiao_v5_sensors_camera.ino`

La versión **v5** integra todo lo anterior corriendo simultáneamente en el
ESP32-S3 dual-core:

- **Núcleo principal** — la cámara OV2640 + el servidor HTTP, con la cámara
  forzada a usar `sccb_i2c_port = 1` (I²C1) para no chocar con el bus de
  sensores.
- **Tarea FreeRTOS dedicada (`vTaskSensors`)** — el bridge MSP a 30 Hz:
  lectura del VL53L0X, lectura del QMC5883L, envío por UART a la FC. Usa
  `Wire` sobre I²C0 (D4/D5) y `HardwareSerial FC(1)` sobre el pin TX D6 hacia
  la UART de la FC.

Los **alineamientos de ejes del magnetómetro** quedaron grabados en el código
una vez verificados experimentalmente en el banco y en la pestaña *Setup* de
iNav:

```cpp
int16_t bx =  my;   // body X = forward
int16_t by =  mx;   // body Y = right
int16_t bz = -mz;   // body Z = down
```

así como las dos correcciones de orientación de la cámara para el montaje
específico del dron:

```cpp
s->set_vflip(s,   1);   // voltea vertical
s->set_hmirror(s, 1);   // voltea horizontal
```

El reparto entre las dos instancias de I²C del ESP32-S3 es la clave que
permite que ambos subsistemas corran sin contención:

| Subsistema | Bus | Pines | Periféricos |
|---|---|---|---|
| Sensores | I²C0 (`Wire`) | SDA = D4 (GPIO5), SCL = D5 (GPIO6) | VL53L0X (0x29), QMC5883L (0x0D) |
| Cámara | I²C1 (SCCB) | pines internos del módulo | OV2640 |
| MSP a FC | UART1 | TX = D6 (GPIO43) | conectada a UART3 RX del iNav |

## 6. Problemas encontrados y mitigaciones

| Síntoma | Causa | Mitigación |
|---|---|---|
| `pulseIn()` introducía latencia inaceptable en el puente PWM | Función bloqueante | Lectura por interrupciones en flanco de subida/bajada |
| El VL53L0X reportaba `65535` o se congelaba al arrancar los motores | Ruido EMI de motores/ESC y caídas de tensión en el bus de alimentación del sensor | Capacitores de 100 µF y 100 nF entre VCC y GND del sensor; cableado I²C corto; alimentación separada |
| Los motores se trababan al leer el sensor en el mismo núcleo | I²C bloqueante en el mismo lazo que la PWM | Tarea FreeRTOS dedicada en el segundo núcleo |
| El PCA9685 ocupaba el bus I²C necesario para el VL53L0X | Conflicto de buses | PCA9685 eliminado; PWM directa desde GPIO del XIAO (sólo válido en la ruta PWM) |
| iNav `Failed to open MSP connection` en macOS | Permisos/estado del configurador | Se conectó por otra vía; configuración aplicada por CLI |
| `feature NAV` no disponible en Betaflight | El firmware cargado no incluía la característica NAV | Migración a iNav, que sí trae Position Hold/Altitude Hold |
| El magnetómetro no era detectado en el bus I²C de la FC | Limitación de hardware/recursos de la FC | Cableado del magnetómetro al **XIAO** y entrega por MSP a la FC |
| El v3 del bridge MSP colgaba el bus si el magnetómetro no respondía | Repeated-start sin timeout dejaba el bus tomado | v3.1: probe del mag al arranque + STOP + `Wire.setTimeOut(25)` |
| `master_age` no medía retardo one-way en el flujo continuo | Los mensajes llegan a su cadencia normal aunque cada uno sufra retardo | Métrica de degradación medida por error de formación, no por *age* |
| Comportamiento errático en Betaflight tras cambios de configuración | Parámetros como `pid_process_denom` cambiados implícitamente | Restauración de configuración estable por `diff`/`dump` desde CLI |
| Cámara WiFi no inicializaba | PSRAM no configurada como OPI PSRAM | Tools → PSRAM = "OPI PSRAM" en Arduino IDE |
| Conflicto potencial entre sensores e I²C del OV2640 | Compartirían `Wire` | `sccb_i2c_port = 1` en la cámara para forzarla al I²C1 separado |

## 7. Estado final y limitaciones

El sistema final cumple con:

1. **FC iNav 9.0.1** configurada y estable (versión "Estable 4"), con
   receptor IBUS, mezcla en X, salida DSHOT300 a los ESC.
2. **Bridge MSP del XIAO** funcional a 30 Hz, entregando VL53L0X
   (`MSP2_SENSOR_RANGEFINDER`) y QMC5883L (`MSP2_SENSOR_COMPASS`).
3. **Cámara WiFi** del XIAO disponible en `http://192.168.4.1/` corriendo en
   paralelo con el bridge.
4. **Configuración versionada** (`dump`, `diff all` y notas de cambios) que
   permite reproducir el estado de la FC.

### Limitaciones

- La **integración del dron personalizado al sistema cooperativo principal
  está pendiente**: el XIAO ya tiene WiFi, pero no implementa todavía el
  protocolo binario UDP de 41 bytes con CRC-16/CCITT descrito en el capítulo
  de comunicaciones.
- El **alineamiento de ejes del magnetómetro** quedó grabado en el código
  para este *airframe* específico (`bx = my, by = mx, bz = -mz`). Cualquier
  cambio físico de orientación del módulo requiere repetir la verificación.
- La **cámara WiFi tiene alcance limitado** (decenas de metros con visión
  directa) y latencia de ~100–300 ms; sirve para inspección/monitoreo, no
  como link FPV.
- El **GPS HGLRC M100** funciona para reportar posición, pero las pruebas con
  Position Hold quedaron fuera del alcance debido a la combinación entre la
  configuración del receiver y la inhibición de armado por las banderas
  `POS_HOLD_SW` observadas durante el desarrollo.

## 8. Trabajo futuro

- **Integrar el XIAO al protocolo binario UDP** del sistema cooperativo: el
  ESP32-S3 ya tiene WiFi/BLE; añadir un socket UDP que publique la altura
  (`MSP2_SENSOR_RANGEFINDER`) y consuma consignas convertiría el dron
  personalizado en un nodo más de la red Ad-Hoc descrita en el capítulo de
  comunicaciones.
- **Validar en vuelo modos asistidos de iNav** (Altitude Hold con
  rangefinder MSP, Position Hold con magnetómetro MSP + GPS) y cuantificar la
  precisión equivalente al control cooperativo del Tello.
- **Implementar detección de ArUco a bordo** usando la cámara OV2640 del
  XIAO Sense; esto unificaría el esquema de localización del dron personalizado
  con el del Tello y permitiría escalar la flota a vehículos heterogéneos.
- **Mejorar la robustez de I²C bajo motores en marcha** con capacitores
  adicionales en la línea de alimentación del sensor y, opcionalmente, bajar
  el clock I²C a 100 kHz si se observan errores.

## Anexo A. Índice de archivos del firmware

Los archivos siguientes están versionados en [`firmware/`](../firmware/).

### Configuración de la Flight Controller (iNav)

| Archivo | Función |
|---|---|
| [`fc_inav/dump_v7.txt`](../firmware/fc_inav/dump_v7.txt) | Dump completo de la configuración iNav 9.0.1 (2216 líneas). Se reaplica por CLI con `batch start` … `batch end`. |
| [`fc_inav/diff_all_v7.txt`](../firmware/fc_inav/diff_all_v7.txt) | Diff de los cambios respecto a los valores por defecto del target OMNIBUSF4PRO. Más legible que el dump. |
| [`fc_inav/cambios_v7_estable_4.txt`](../firmware/fc_inav/cambios_v7_estable_4.txt) | Bitácora textual de los cambios manuales que llevaron a la versión "Estable 4" (orden de motores, modos, CH5/CH6, etc.). |

### Firmware del XIAO ESP32-S3 — ruta PWM (descartada)

| Archivo | Función |
|---|---|
| [`xiao_pwm_bridge/01_vuelo_pwm.ino`](../firmware/xiao_pwm_bridge/01_vuelo_pwm.ino) | Puente PWM mínimo: captura 4 PWM por interrupciones y reenvía a ESC con `analogWrite`. |
| [`xiao_pwm_bridge/02_vuelo_pwm_lectura_sensor.ino`](../firmware/xiao_pwm_bridge/02_vuelo_pwm_lectura_sensor.ino) | Lo anterior + lectura del VL53L0X en tarea de FreeRTOS fija al núcleo 0 (sin afectar los motores). |
| [`xiao_pwm_bridge/03_vuelo_pwm_pid_altura.ino`](../firmware/xiao_pwm_bridge/03_vuelo_pwm_pid_altura.ino) | Lo anterior + PID experimental de altura que aplica corrección colectiva. Descartado tras observar respuesta errática. |

### Firmware del XIAO ESP32-S3 — ruta MSP (adoptada)

| Archivo | Función |
|---|---|
| [`xiao_msp_bridge/v1_msp_rangefinder_bridge.ino`](../firmware/xiao_msp_bridge/v1_msp_rangefinder_bridge.ino) | Bridge mínimo: VL53L0X → `MSP2_SENSOR_RANGEFINDER` por UART a iNav, a 30 Hz. |
| [`xiao_msp_bridge/v2_msp_rangefinder_bridge.ino`](../firmware/xiao_msp_bridge/v2_msp_rangefinder_bridge.ino) | Añade un promedio móvil opcional sobre la distancia, con discusión del *trade-off* latencia/ruido. |
| [`xiao_msp_bridge/v3_msp_rangefinder_bridge.ino`](../firmware/xiao_msp_bridge/v3_msp_rangefinder_bridge.ino) | Añade magnetómetro HMC5883L/QMC5883L vía `MSP2_SENSOR_COMPASS` + escáner I²C al arranque. |
| [`xiao_msp_bridge/v3_1_msp_rangefinder_bridge.ino`](../firmware/xiao_msp_bridge/v3_1_msp_rangefinder_bridge.ino) | Fix del v3: magnetómetro fail-safe (no cuelga el bus si no responde) + STOP + `Wire.setTimeOut`. |
| [`xiao_msp_bridge/v4_msp_rangefinder_bridge.ino`](../firmware/xiao_msp_bridge/v4_msp_rangefinder_bridge.ino) | Versión limpia para el airframe final, direcciones de I²C confirmadas, sin escáner. |

### Firmware del XIAO ESP32-S3 — cámara y versión integrada

| Archivo | Función |
|---|---|
| [`xiao_camera/xiao_camera_wifi_stream.ino`](../firmware/xiao_camera/xiao_camera_wifi_stream.ino) | Sketch standalone: la cámara OV2640 expuesta como stream MJPEG por WiFi en modo AP. |
| [`xiao_integrated/drone_xiao_v5_sensors_camera.ino`](../firmware/xiao_integrated/drone_xiao_v5_sensors_camera.ino) | **Versión final integrada.** Corre en paralelo el bridge MSP (núcleo 0, tarea FreeRTOS) y la cámara WiFi (núcleo principal). Sensores en I²C0, cámara en I²C1, sin contención. Alineamiento de ejes del magnetómetro y orientación de cámara confirmados experimentalmente. |
