# Firmware del dron personalizado

Archivos versionados del desarrollo del dron construido desde componentes
(XIAO ESP32-S3 + FC iNav F405). Explicación completa de la evolución y de qué
hace cada archivo en [`docs/CUSTOM_DRONE.md`](../docs/CUSTOM_DRONE.md).

## Estructura

```
firmware/
├── fc_inav/                          # Configuración del Flight Controller
│   ├── dump_v7.txt                   #   dump completo iNav 9.0.1 (OMNIBUSF4PRO/F405)
│   ├── diff_all_v7.txt               #   diff vs defaults: PIDs, filtros, mezcla
│   └── cambios_v7_estable_4.txt      #   notas y CLI de la configuración estable
│
├── xiao_pwm_bridge/                  # Ruta DESCARTADA: XIAO como puente PWM FC→ESC
│   ├── 01_vuelo_pwm.ino              #   puente PWM puro (captura por interrupciones)
│   ├── 02_vuelo_pwm_lectura_sensor.ino  # + VL53L0X en tarea separada (sin control)
│   └── 03_vuelo_pwm_pid_altura.ino   #   + PID experimental de altura (descartado)
│
├── xiao_msp_bridge/                  # Ruta ADOPTADA: XIAO → iNav por MSP v2
│   ├── v1_msp_rangefinder_bridge.ino #   v1: VL53L0X → MSP2_SENSOR_RANGEFINDER
│   ├── v2_msp_rangefinder_bridge.ino #   v2: + promedio móvil opcional
│   ├── v3_msp_rangefinder_bridge.ino #   v3: + magnetómetro (MSP2_SENSOR_COMPASS)
│   ├── v3_1_msp_rangefinder_bridge.ino  # v3.1: mag fail-safe (fix de bug del v3)
│   └── v4_msp_rangefinder_bridge.ino #   v4: versión limpia, direcciones confirmadas
│
├── xiao_camera/                      # Cámara WiFi (sketch standalone)
│   └── xiao_camera_wifi_stream.ino   #   OV2640 MJPEG por WiFi (modo AP)
│
└── xiao_integrated/                  # Versión FINAL: sensores + cámara
    └── drone_xiao_v5_sensors_camera.ino  # MSP bridge + cámara WiFi en paralelo (FreeRTOS)
```

## Resumen de cómo se llegó al firmware final

1. **`fc_inav/`** — la FC corre **iNav 9.0.1**. El archivo `dump_v7.txt` es la
   configuración completa que se sube a la FC; `diff_all_v7.txt` muestra solo
   los cambios respecto a defaults (PIDs, filtros, mezcla en X, receptor IBUS,
   protocolo DSHOT300). `cambios_v7_estable_4.txt` documenta las modificaciones
   manuales (orden de motores, modos desactivados) que llevaron a la
   configuración estable.

2. **`xiao_pwm_bridge/`** — primera ruta exploratoria: el XIAO se interponía
   **físicamente** entre la FC y los ESC, leía la PWM de la FC y la reenviaba.
   Se llegó a probar un PID experimental de altura inyectando corrección
   colectiva, pero la ruta se **descartó** por su fragilidad (cualquier fallo
   del XIAO mata el control de motores).

3. **`xiao_msp_bridge/`** — ruta **adoptada**: el XIAO ya no toca los motores.
   Lee VL53L0X (distancia) y QMC5883L (magnetómetro) por I²C y se los entrega a
   la FC por **UART usando MSP v2** (`MSP2_SENSOR_RANGEFINDER` y
   `MSP2_SENSOR_COMPASS`). El iNav los acepta como sensores virtuales y hace
   todo el control de vuelo.

4. **`xiao_camera/`** + **`xiao_integrated/`** — el XIAO ESP32-S3 **Sense** trae
   cámara OV2640; el sketch de la carpeta `xiao_camera/` muestra cómo
   exponerla como stream MJPEG por WiFi. La versión `v5` integra el bridge MSP
   y la cámara WiFi corriendo **al mismo tiempo** en los dos núcleos del
   ESP32-S3 (sensores en I²C0, cámara en I²C1 dedicado, sin contención).
