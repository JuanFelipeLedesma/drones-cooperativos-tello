"""
═══════════════════════════════════════════════════════════════════════
 GUÍA DE EJECUCIÓN — DÍA DE PRUEBAS EN CREA
 Sistema Cooperativo de Drones — Tesis Juan Felipe Ledesma
═══════════════════════════════════════════════════════════════════════

 ANTES DE IR AL LAB — Checklist de preparación:

 □ Cargar TODAS las baterías Tello (mínimo 4 baterías cargadas al 100%)
 □ Copiar esta carpeta drone_tests/ completa a AMBOS computadores (Mac + Ubuntu)
 □ Instalar dependencias en ambos computadores:
       pip install djitellopy opencv-python numpy
 □ Verificar cable Ethernet y configurar IPs estáticas:
       Mac:    192.168.1.1  (o ajustar en config.py → MASTER_IP)
       Ubuntu: 192.168.1.2  (o ajustar en config.py → SLAVE_IP)
 □ Probar ping entre computadores: ping 192.168.1.1 / ping 192.168.1.2
 □ Imprimir marcadores ArUco (DICT_4X4_50, IDs 0-5, tamaño 15cm)
   Puedes generarlos con:
       python -c "
       import cv2
       d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
       for i in range(6):
           img = cv2.aruco.generateImageMarker(d, i, 600)
           cv2.imwrite(f'aruco_{i}.png', img)
       "
 □ Llevar cinta adhesiva para pegar markers en la PARED (NO en el piso)
 □ Llevar trípode o soporte para cámara cenital (celular) — Prueba 5.3
 □ Medir y anotar las posiciones exactas donde pegarás los markers
   (actualizar MARKER_WORLD_POSITIONS en config.py)

═══════════════════════════════════════════════════════════════════════
"""

EXECUTION_GUIDE = """
═══════════════════════════════════════════════════════════════════════
 ORDEN DE EJECUCIÓN EN EL LABORATORIO
═══════════════════════════════════════════════════════════════════════

 Tiempo estimado total: ~6-7 horas (con descansos y cambios de batería)

───────────────────────────────────────────────────────────────────────
 SETUP INICIAL (~20 min)
───────────────────────────────────────────────────────────────────────
 1. Pegar 6 marcadores ArUco en la PARED (la cámara del Tello mira al frente, no abajo)
    Grid 3x2 en la pared, a la altura de vuelo del dron:
      Fila baja (Y≈0.6m):  Marker 0 → X=0.5m  |  Marker 1 → X=1.5m  |  Marker 2 → X=2.5m
      Fila alta (Y≈1.0m):  Marker 3 → X=0.5m  |  Marker 4 → X=1.5m  |  Marker 5 → X=2.5m
    Medir X desde la esquina izquierda de la pared, Y desde el piso.

 2. Medir las posiciones exactas y actualizar config.py:
      MARKER_WORLD_POSITIONS

 3. Conectar Mac ↔ Ubuntu con cable Ethernet
    Verificar: ping 192.168.1.1 y ping 192.168.1.2

 4. Conectar cada computador a su Tello (WiFi del Tello)
    Mac → WiFi del Tello 1
    Ubuntu → WiFi del Tello 2

───────────────────────────────────────────────────────────────────────
 FASE 1: Caracterización individual (~65 min) — 1 DRON
───────────────────────────────────────────────────────────────────────

 ▸ Prueba 1.1 — Respuesta escalón (~30 min)
   Computador: Mac (o Ubuntu, da igual, solo 1 dron)
   Comando:    python test_1_1_step_response.py
   Qué hace:   Envía comandos discretos y graba trayectoria ArUco
   Output:     logs/test_1_1_step_response_YYYYMMDD_HHMMSS.csv
   ⚡ Cambiar batería si baja del 20%

 ▸ Prueba 1.2 — Latencia comando→acción (~20 min)
   Computador: Mismo
   Comando:    python test_1_2_latency.py
   Qué hace:   30 comandos con medición de latencia por ArUco
   Output:     logs/test_1_2_latency_*.csv + resumen en terminal
   ⚡ Cambiar batería

 ▸ Prueba 1.3 — Parámetros de hover (~15 min)
   Computador: Mismo
   Comando:    python test_1_3_hover.py
   Qué hace:   Hover 60s, registra drift y varianza
   Output:     logs/test_1_3_hover_*.csv + resumen en terminal

   ☕ Descanso 5 min, cambiar baterías

───────────────────────────────────────────────────────────────────────
 FASE 2: Lazo cerrado individual (~45 min) — 1 DRON
───────────────────────────────────────────────────────────────────────

 ▸ Prueba 2.1 — Control de posición ArUco (~45 min)
   Computador: Mac (o Ubuntu)
   Comando:    python test_2_1_closed_loop.py
   Qué hace:   Mantiene posición fija con PID sobre ArUco por 30s
   Output:     logs/test_2_1_closed_loop_*.csv

   ⚠️  IMPORTANTE: Si el error estacionario es >15cm o hay oscilaciones,
   ajusta las ganancias PID en config.py (PID_X, PID_Y, PID_Z) y repite.
   NO avanzar a pruebas cooperativas hasta que este lazo funcione bien.

   ⚡ Cambiar baterías en ambos Tellos

───────────────────────────────────────────────────────────────────────
 FASE 4: Primer vuelo cooperativo (~45 min) — 2 DRONES
───────────────────────────────────────────────────────────────────────

 ▸ Prueba 2.2 — Formación estática líder-seguidor
   Terminal Mac:     python test_2_2_master.py
   Terminal Ubuntu:  python test_2_2_slave.py
   (Ejecutar MASTER primero, SLAVE se conecta automáticamente)

   Qué hace: Líder en hover, seguidor mantiene offset de 1.5m
   Output:   logs/test_2_2_master_*.csv + logs/test_2_2_slave_*.csv

   ⚡ Cambiar baterías

───────────────────────────────────────────────────────────────────────
 FASE 5: Cooperación dinámica (~120 min) — 2 DRONES
───────────────────────────────────────────────────────────────────────

 ▸ Prueba 2.3 — Formación dinámica (trayectoria cuadrado)
   Terminal Mac:     python test_2_3_master.py --trajectory square
   Terminal Ubuntu:  python test_2_3_slave.py

   ⚡ Cambiar baterías

 ▸ Prueba 2.3 — Formación dinámica (trayectoria línea)
   Terminal Mac:     python test_2_3_master.py --trajectory line
   Terminal Ubuntu:  python test_2_3_slave.py

   ⚡ Cambiar baterías

 ▸ Prueba 2.4 — Consenso distribuido
   Terminal Mac:     python test_2_4_consensus.py --role A --dest-ip 192.168.1.2
   Terminal Ubuntu:  python test_2_4_consensus.py --role B --dest-ip 192.168.1.1
   (Posicionar drones en extremos opuestos, ~2.5m de separación)

   ☕ Descanso 10 min, cambiar baterías

───────────────────────────────────────────────────────────────────────
 FASE 6: Efectos de red sobre control (~90 min) — 2 DRONES
───────────────────────────────────────────────────────────────────────

 ▸ Prueba 3.3 — Degradación controlada de red
   Se necesitan 3 terminales en Ubuntu:
     Terminal 1 (Ubuntu): python test_2_3_slave.py
     Terminal 2 (Ubuntu): sudo bash test_3_3_degradation.sh <nombre_interfaz>
     Terminal Mac:        python test_2_3_master.py --trajectory line

   Para encontrar el nombre de la interfaz Ethernet en Ubuntu:
     ip link show | grep -v lo
   (Suele ser eth0, enp0s3, o eno1)

   Qué hace: Mientras vuelan en formación, inyecta retardo y pérdida
   en la red progresivamente (50ms → 100ms → 200ms, luego 5% → 10% → 20%)
   Output: El CSV del SLAVE tendrá los datos de formación bajo cada condición

   ⚡ Cambiar baterías

 ▸ Prueba 3.4 — Reconexión y tolerancia a fallas
   Terminal Mac:    python test_2_2_master.py
   Terminal Ubuntu: python test_2_2_slave.py

   MANUAL: Una vez estabilizada la formación (~15s):
     → Desconectar cable Ethernet (5s) → Reconectar → Observar
     → Repetir con 10s y 15s de desconexión
   Observar en terminal del SLAVE los mensajes "SIN CONEXION"

───────────────────────────────────────────────────────────────────────
 FASE 8: Validación integral (~120 min) — 2 DRONES
───────────────────────────────────────────────────────────────────────

 ▸ Prueba 5.3 — Montar cámara cenital
   Colocar celular/cámara en trípode alto apuntando hacia abajo.
   Pegar marcadores de escala en el piso (ej. cinta cada 50cm).
   INICIAR GRABACIÓN antes de cada run.

 ▸ Prueba 5.1 + 5.2 — Misión completa × 5 runs
   Para cada run (1 a 5):
     1. Verificar baterías >90% en ambos Tellos
     2. Terminal Mac:    python test_5_1_master.py --run N
     3. Terminal Ubuntu: python test_5_1_slave.py --run N
     4. Esperar a que termine (~60-90s por run)
     5. Anotar: ¿Éxito Sí/No? ¿Algún incidente?
     6. Cambiar baterías
     7. Esperar 5 min
     8. Repetir

   Output por run:
     logs/test_5_1_master_runN_*.csv
     logs/test_5_1_slave_runN_*.csv
     Video cenital del celular

═══════════════════════════════════════════════════════════════════════
 AL FINALIZAR
═══════════════════════════════════════════════════════════════════════

 1. Copiar TODAS las carpetas logs/ de ambos computadores a un USB
 2. Copiar los videos cenitales del celular
 3. Revisar que todos los CSV se generaron correctamente
 4. ¡Listo! Los datos se analizan después en casa.

 Archivos esperados (si todo sale bien):
   logs/test_1_1_step_response_*.csv
   logs/test_1_2_latency_*.csv
   logs/test_1_3_hover_*.csv
   logs/test_2_1_closed_loop_*.csv
   logs/test_2_2_master_*.csv + test_2_2_slave_*.csv
   logs/test_2_3_master_square_*.csv + test_2_3_slave_*.csv
   logs/test_2_3_master_line_*.csv + test_2_3_slave_*.csv
   logs/test_2_4_consensus_A_*.csv + test_2_4_consensus_B_*.csv
   logs/test_2_3_slave_*.csv (con datos de degradación 3.3)
   logs/test_2_2_slave_*.csv (con datos de reconexión 3.4)
   logs/test_5_1_master_run1..5_*.csv + test_5_1_slave_run1..5_*.csv
"""

if __name__ == "__main__":
    print(EXECUTION_GUIDE)
