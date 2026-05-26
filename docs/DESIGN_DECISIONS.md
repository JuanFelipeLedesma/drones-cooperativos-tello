# Decisiones de diseño y métodos descartados

Por qué el sistema está construido como está, y un catálogo honesto de lo que **probamos y no
funcionó**. Cada fallo es un hallazgo metodológico legítimo: documentarlos evita que quien retome el
proyecto repita el mismo camino.

---

## Parte A — Decisiones de diseño (el "por qué")

### A.1 Dos computadores unidos por Ethernet
Cada Tello crea su propia red WiFi y admite **un solo cliente**. No hay forma soportada de poner dos
Tello en la misma red. Por tanto la cooperación no puede ocurrir entre los drones; vive entre los
**computadores**, unidos por un cable Ethernet directo que actúa como backbone Ad-Hoc. Toda la
arquitectura de dos PC deriva de esta restricción de hardware. (Con Tello **EDU** hay alternativas;
ver [`TELLO_EDU.md`](TELLO_EDU.md).)

### A.2 `rc_control` en vez de `move_*`
Los comandos discretos `move_*` tienen latencia ~1 s (medido en la prueba 1.2, incluye el tiempo de
recorrer el umbral de detección) y son bloqueantes. Inviables para un lazo cerrado. **Todo el control
cooperativo usa `rc_control`** (comando continuo de velocidad), que permite ~13.6 Hz de lazo efectivo.

### A.3 ArUco de marcador único + IPPE_SQUARE
- **Marcador único** (el más cercano al centro del frame): es estable y barato de computar. La fusión
  multi-marcador se descartó por costo (ver B.2).
- **`SOLVEPNP_IPPE_SQUARE`**: específico para marcadores planos cuadrados; **resuelve la ambigüedad de
  pose planar** que hace a `SOLVEPNP_ITERATIVE` saltar entre dos soluciones (ver B.1).
- **Filtro temporal de outliers:** descarta saltos físicamente imposibles (>0.5 m en <0.5 s).

### A.4 Error de control en metros, ganancias "grandes"
El error se expresa en **metros** y la salida del PID es un comando `rc` entero en ±30. Para que un
error típico de 0.20 m produzca un comando útil (~12), `kp` debe estar en el rango 60–70. (El bug de no
hacerlo está en B.6.)

### A.5 Filtro de promedio móvil (N=8) en el seguidor
El seguidor calcula su objetivo a partir de la pose **ruidosa** del líder. Sin filtrar, persigue ese
ruido y el error de formación se duplica (27.4 → 12.9 cm con filtro). Es un **compromiso explícito**:
reduce el error en hover, pero introduce retraso de seguimiento en movimiento (picos en las esquinas de
la trayectoria). Un filtro adaptativo es trabajo futuro.

### A.6 Protocolo binario con CRC en C
41 bytes vs 162 de JSON (4× más compacto) y 3–5× más rápido usando `binascii.crc_hqx` (CRC en C). El
CRC da integridad de extremo a extremo: **0 errores en >50 000 mensajes**. (La trampa de implementarlo
en Python puro está en B.10.)

### A.7 Consenso con separación mínima de seguridad
La ley de consenso clásica (converger a un punto único) es correcta en el papel pero **físicamente
peligrosa** para multirrotores: se apilan y chocan. Añadimos una separación lateral mínima de 60 cm;
los drones convergen simétricamente alrededor del centroide. Sigue siendo consenso (es *rendezvous with
formation*). (El choque original está en B.3.)

### A.8 Aterrizaje secuencial en la misión
En la misión integral, el seguidor aterriza **antes** que el líder (señalizado con `mission_state=8`)
para evitar interferencia aerodinámica entre ambos al descender juntos.

### A.9 OE4 en Python en vez de Gazebo
La simulación se construyó sobre el **modelo dinámico identificado experimentalmente** en OE1 (2.º
orden, ωₙ≈1.9, ζ≈1.0), no sobre un modelo genérico de simulador. Argumento: mayor rigor (parte de
parámetros reales medidos) y viabilidad de tiempo. **Pendiente de confirmar con el asesor**; si exige
Gazebo, la simulación Python queda como modelo de referencia (ver [`FUTURE_WORK.md`](FUTURE_WORK.md)).

---

## Parte B — Lo que NO funcionó (y cómo se resolvió)

### B.1 `SOLVEPNP_ITERATIVE` — ambigüedad de pose planar
**Síntoma:** la posición saltaba 1–2 m con el dron inmóvil.
**Causa:** para un marcador cuadrado en perspectiva, el método iterativo alterna entre dos soluciones
de pose válidas.
**Fix:** `SOLVEPNP_IPPE_SQUARE`, diseñado para marcadores planos cuadrados.

### B.2 Fusión multi-marcador — saturó el lazo
**Síntoma:** el dron derivaba descontrolado (casi choca contra la pared).
**Causa:** hacer `solvePnP` por cada marcador visible (~6×) tardaba >50 ms/iteración; los `rc 0 0 0 0`
no llegaban a tiempo.
**Fix:** marcador único (más cercano al centro) + IPPE + filtro de outliers. La fusión queda como
mejora futura **con perfilado de tiempo**.

### B.3 Consenso clásico (punto único) — colisión por *downwash*
**Síntoma:** los drones se apilaron verticalmente y chocaron dos veces.
**Causa:** la teoría asume puntos sin dimensión; los Tello miden ~10 cm + hélices y generan *downwash*.
**Fix:** separación mínima de 60 cm como restricción geométrica (ver A.7).

### B.4 IP estática con `ip addr add` — se borra al desconectar el cable
**Síntoma:** tras desconectar/reconectar el Ethernet, el SLAVE no se reconectaba.
**Causa:** NetworkManager elimina la IP asignada manualmente al perder el enlace; la interfaz queda sin
dirección y descarta los UDP entrantes.
**Fix:** IP persistente vía `nmcli` (ver [`SETUP.md`](SETUP.md#31-ubuntu-slave--ip-estática-persistente)).

### B.5 IMU sin pausa post-takeoff — `No valid imu`
**Síntoma:** error `No valid imu` y deriva incontrolada al primer `move_*` tras despegar.
**Causa:** el IMU del Tello necesita estabilizarse después del takeoff.
**Fix:** `IMU_WAIT_S = 4.0` s de hover (`rc 0 0 0 0`) entre takeoff y cualquier movimiento. Aplicado en
**todos** los scripts. (Además, el IMU debe estar calibrado de fábrica/app; ver reglas de seguridad.)

### B.6 PID con ganancias 100× demasiado bajas
**Síntoma:** el dron no respondía y derivaba.
**Causa:** `kp≈0.4` con error en metros → `0.4 × 0.20 = 0.08` → `int(0.08) = 0`. El PID enviaba ceros.
**Fix:** ganancias en el rango correcto (kp 60–70, ki 8–10, kd 25) para error en metros → comando rc.

### B.7 Calibración de cámara con chessboard — empeoró el ruido
**Síntoma:** tras "calibrar" (123 capturas, RMS 0.95 px aparentemente bueno), el ruido de pose **subió**
de ~12 cm a ~47 cm de desviación; el punto principal salió en cy=543 (esperado ~360).
**Causa:** las capturas no cubrieron el frame de forma uniforme → sesgo en el *principal point*.
**Decisión:** revertir a **parámetros aproximados** (`config.py`). La calibración formal queda pendiente
con un protocolo de captura más cuidadoso (cubrir esquinas y todo el campo de visión).

### B.8 Ruido de pose ArUco inicial (markers de 11.5 cm)
**Síntoma:** desviación de pose de 25–40 cm.
**Causa:** marcadores pequeños (tamaño aparente bajo) + cámara sin calibrar.
**Fix combinado:** (1) markers de **48 cm**; (2) `IPPE_SQUARE`; (3) filtro temporal de outliers;
(4) selección del marcador más cercano al centro. Resultado: desviación de 4–8 cm en estado estacionario.

### B.9 Bug de esquema en el `FlightLogger`
**Síntoma:** el logger crasheaba al escribir una fila con columnas nuevas (p. ej. una métrica que solo
aparece durante una desconexión).
**Causa:** `csv.DictWriter` fija el esquema en la primera fila.
**Fix:** poblar **todas** las columnas (con `None`) desde la primera fila.

### B.10 CRC en Python puro — más lento que JSON
**Síntoma:** la "ventaja" del binario desaparecía; encode más lento que JSON.
**Causa:** implementar el CRC-16 en Python puro (bit a bit) es ~280× más lento que la versión C.
**Fix:** `binascii.crc_hqx` (CRC-16/CCITT en C). El binario quedó 3–5× más rápido que JSON.

### B.11 `tc netem` solo afecta egress — la inyección de red "no hacía nada"
**Síntoma:** con `tc qdisc add dev enp1s0 root netem delay 200ms`, la latencia medida seguía en ~4 ms.
**Causa:** `netem` en `dev <iface> root` degrada solo el **egress**; el flujo crítico MASTER→SLAVE es
**ingress** al Ubuntu.
**Fix:** redirigir el ingress a un dispositivo **IFB** y aplicar `netem` a su egress (procedimiento
completo en [`REPLICATION.md`](REPLICATION.md#33--degradación-controlada-de-la-red-2-drones--inyección)).

### B.12 `master_age` no mide el retardo one-way
**Observación:** aun con 200 ms inyectados correctamente, la métrica `master_age` (`t_now − last_recv`)
reportaba ~5 ms.
**Causa:** el flujo es continuo a 50 Hz; cada mensaje sufre el retardo, pero el *gap entre mensajes*
sigue siendo ~20 ms. Para medir el retardo one-way real haría falta sincronizar relojes.
**Lo que sí funciona:** medir el efecto en el **error de formación** (que sí refleja el retardo) y la
**pérdida** vía `received_count` del listener.

---

## Apéndice — un hallazgo de control que vale como conclusión

En la arquitectura **líder-seguidor**, el seguidor cierra su lazo con **feedback de posición propio**
(su cámara ArUco). El retardo de red afecta solo la **consigna** (la pose del líder), no el lazo de
realimentación, así que **no desestabiliza** el control. La simulación (4.2) muestra el error plano
ante delay/loss, mientras que el experimento real (3.3) sí se degrada — lo que implica que la
degradación real proviene de **factores concurrentes** (pérdida de pose ArUco del propio seguidor,
condiciones del experimento), no del retardo de red en sí. El retardo solo sería crítico si estuviera
**dentro** del lazo de realimentación, como en el consenso distribuido. La simulación funcionó así como
herramienta de **diagnóstico** que aísla variables que el experimento físico mezcla.
