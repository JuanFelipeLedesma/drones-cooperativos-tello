# Trabajo futuro y mejoras

Ideas concretas para quien retome el proyecto, ordenadas de menor a mayor esfuerzo. Cada una se conecta
con una limitación o un hallazgo documentado en [`DESIGN_DECISIONS.md`](DESIGN_DECISIONS.md) y
[`RESULTS.md`](RESULTS.md).

## 1. Mejoras de bajo esfuerzo

- **Filtro adaptativo en el seguidor.** El promedio móvil (N=8) reduce el error en hover pero introduce
  retraso en movimiento (picos en las esquinas de la trayectoria, prueba 2.3). Un filtro que **relaje N
  cuando detecta movimiento del líder** atenuaría el compromiso entre error estacionario y lag.
- **Calibración de cámara bien hecha.** La calibración con chessboard se descartó por capturas mal
  distribuidas (sesgo en el *principal point*). Repetirla **cubriendo todo el campo de visión**
  (esquinas incluidas, varias distancias e inclinaciones) debería bajar el ruido de pose residual.
  Utilidades ya en el repo: `generate_chessboard.py`, `calibrate_camera.py`.
- **Sincronizar relojes para medir retardo one-way.** `master_age` no mide el delay real (ver B.12). Con
  NTP/PTP entre los dos PC, o un *handshake* de offset de reloj, se podría reportar el retardo one-way
  efectivo además del efecto en el error.

## 2. Mejoras de esfuerzo medio

- **Fusión multi-marcador con perfilado.** La fusión se descartó por saturar el lazo (>50 ms/iteración).
  Con perfilado y optimización (limitar a los 2–3 marcadores más grandes, vectorizar, o mover la visión
  a otro hilo/proceso) se podría aprovechar más información sin perder frecuencia de control.
- **Migrar a Tello EDU.** Modo estación + `TelloSwarm` permitiría controlar ambos drones desde un solo
  computador y escalar el número de drones. Trade-offs y código en [`TELLO_EDU.md`](TELLO_EDU.md).
- **Replicar OE4 en Gazebo.** La simulación se hizo en Python sobre el modelo identificado. Un simulador
  físico completo (ROS 2 / Gazebo) capturaría transitorios que el modelo simplificado subestima.
  **Pendiente de confirmar con el asesor** si esto es requerido o si la simulación Python es suficiente.

## 3. Líneas de investigación mayores

- **Localización sin marcadores fijos.** Odometría visual-inercial (VIO) o SLAM para operar fuera del
  área instrumentada con ArUco de pared.
- **Escalar a flotas heterogéneas.** El sistema se validó con 2 drones; estudiar consenso y formación
  con N>2 y con vehículos de distinta dinámica.
- **Retardo dentro del lazo (consenso).** El hallazgo central es que el retardo no afecta al
  líder-seguidor porque queda fuera del lazo de realimentación. En el **consenso distribuido** sí está
  dentro: caracterizar experimentalmente cuánto retardo tolera antes de desestabilizarse sería una
  contribución natural.

## 4. Prototipo de dron propio (XIAO ESP32S3 + PCA9685)

Durante el proyecto se exploró construir un **dron personalizado** para integrarlo como un nodo más del
sistema cooperativo (un autopiloto abierto que hablara el mismo protocolo binario UDP). El asesor
sugirió usarlo primero solo **como cámara**. El trabajo de firmware llegó a:

- **Microcontrolador:** Seeed Studio **XIAO ESP32S3**.
- **Driver PWM:** **PCA9685** por I²C (16 canales PWM para ESC/servos).
- Firmware exploratorio desarrollado (en iteraciones): *passthrough* PWM, lectura de PWM de entrada por
  interrupciones, rampa de motores 0→máx secuencial, y un "puente" que replica la señal PWM de entrada
  del motor 4 hacia el ESC.
- **Acondicionamiento de señal:** divisor de tensión (1 kΩ / 560 Ω → ~3.21 V) para leer con seguridad la
  PWM en una entrada de 3.3 V de la ESP32.

> El código del firmware **no está versionado en este repositorio** (se desarrolló aparte y no llegó a
> integrarse). Queda como punto de partida documentado: la **integración de un dron propio como nodo del
> protocolo de cooperación** es la extensión natural hacia una flota heterogénea. Para retomarlo,
> reimplementar el firmware en el repo (carpeta sugerida `firmware/`) y hacer que publique/consuma el
> mensaje binario de 41 bytes descrito en [`ARCHITECTURE.md`](ARCHITECTURE.md#62-protocolo-binario-de-producción).

## 5. Tareas de cierre del documento de tesis (responsabilidad del autor)

No son código, pero quedan pendientes para la entrega formal:

- Desarrollar el **Marco Teórico** con literatura verificada.
- Insertar las **referencias IEEE reales** donde el informe marca `[REFERENCIA PENDIENTE]` (no inventar
  citas; verificar DOI/URL). Cita "segura" de partida: *Ryze Robotics, "Tello SDK 2.0 User Guide", 2018*.
- Actualizar la **tabla de contenido** del `.docx` (abrir en Word y refrescar campos).
- **Confirmar con el asesor** la sustitución Gazebo → simulación en Python para OE4.
