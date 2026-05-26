"""
═══════════════════════════════════════════════════════════════
PRUEBA 3.4 — Reconexión y tolerancia a fallas
═══════════════════════════════════════════════════════════════
Drones: 2  |  Complejidad: Media  |  Tiempo: ~30 min

Esta prueba usa los mismos scripts de formación (2.2 o 2.3)
pero durante el vuelo se desconecta MANUALMENTE el cable Ethernet.

NO necesita script propio. Las instrucciones son:

PROCEDIMIENTO:
    1. Iniciar formación estática (test_2_2_master.py + test_2_2_slave.py)
    2. Esperar 15s a que la formación se estabilice.
    3. DESCONECTAR el cable Ethernet durante 5 segundos.
    4. RECONECTAR el cable.
    5. Observar en la terminal del SLAVE:
       - ¿Aparece "SIN CONEXION - HOVER SEGURO"?
       - ¿Cuánto tarda en reconverger después de reconectar?
    6. Repetir con desconexiones de 10s y 15s.

MÉTRICAS (del log del SLAVE):
    - Comportamiento durante desconexión (hover/drift/divergencia)
    - Desplazamiento máximo del SLAVE durante la desconexión
    - Tiempo de reconvergencia después de reconectar
    - Duración máxima tolerable antes de pérdida de formación

NOTA:
    El SLAVE ya tiene implementado el mecanismo de seguridad:
    si no recibe mensajes por más de COMMS_TIMEOUT_S (5s),
    entra en hover automáticamente (send_rc_control(0,0,0,0)).
    Revisa los logs CSV para las métricas detalladas.
"""

print(__doc__)
print("Esta prueba no requiere script adicional.")
print("Usa los scripts de test_2_2 o test_2_3 y desconecta el cable Ethernet manualmente.")
