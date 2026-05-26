# Cómo se haría con Tello EDU

Este proyecto usó el **Tello estándar**, cuya principal restricción es que cada dron crea su propia red
WiFi y solo acepta un cliente. Eso obligó a la arquitectura de **dos computadores unidos por Ethernet**.
El **Tello EDU** levanta esa restricción y habilitaría un diseño más simple. Aquí se documenta qué
cambiaría, para quien retome el proyecto con ese hardware.

> Resumen: con Tello EDU podrías controlar **ambos drones desde un solo computador** (modo estación +
> enjambre) y, opcionalmente, reemplazar los marcadores ArUco de pared por los **mission pads** del
> propio EDU. A cambio, pierdes el ejercicio de red Ad-Hoc real entre dos máquinas (que aquí era parte
> del objetivo OE3).

## 1. Diferencias de hardware/firmware relevantes

| Capacidad | Tello (estándar, este proyecto) | Tello EDU |
|---|---|---|
| Modo estación (unirse a un router) | ❌ solo crea su propio AP | ✅ comando `ap`/`connect_to_wifi` |
| Varios drones en una red | ❌ (1 cliente por dron) | ✅ todos al mismo router |
| Enjambre desde un PC | ❌ (requiere 1 PC por dron) | ✅ `TelloSwarm` (djitellopy) |
| Mission pads (fiduciales propios) | ❌ | ✅ 8 pads, cámara inferior |
| Cámara frontal + ToF inferior | ✅ | ✅ |

## 2. Arquitectura alternativa: un solo computador + enjambre

Con EDU, ambos drones se unen al **mismo router** (modo estación) y un único computador los controla:

```
   ┌─────────────────────────────────────────────┐
   │            Computador único                  │
   │  TelloSwarm  ──────┬──────────────┐          │
   └────────────────────┼──────────────┼──────────┘
                        │ WiFi          │ WiFi
                   ┌────────┐      ┌────────┐
                   │TelloEDU│      │TelloEDU│   ←── ambos al mismo router
                   │   A    │      │   B    │
                   └────────┘      └────────┘
```

Poner cada EDU en modo estación (una sola vez, conectado a su AP por defecto):

```python
from djitellopy import Tello
t = Tello()
t.connect()
# Conecta el EDU al router 'MiRouter' (el dron se reinicia en modo estación):
t.connect_to_wifi("MiRouter", "password")
```

Controlar ambos a la vez por sus IPs en el router:

```python
from djitellopy import TelloSwarm

swarm = TelloSwarm.fromIps(["192.168.0.101", "192.168.0.102"])
swarm.connect()
swarm.takeoff()

# Comando idéntico a todos:
swarm.move_up(50)

# Comando distinto por dron (i = índice, tello = instancia):
swarm.parallel(lambda i, tello: tello.move_left(30) if i == 0 else tello.move_right(30))

swarm.land()
swarm.end()
```

**Qué cambiaría en este repo:**
- Desaparece toda la capa de **Ethernet entre dos PC** y el `utils/comms.py` UDP: la "comunicación"
  entre drones pasa a ser estado compartido en la memoria de un solo proceso.
- Las pruebas de pares MASTER/SLAVE (`test_2_2_master.py`/`_slave.py`, etc.) se colapsan en **un solo
  script** que mantiene el estado de ambos drones y aplica la ley de cooperación (formación/consenso) en
  el mismo lazo.
- El control sigue igual: PID por eje, error en metros, `send_rc_control`.

## 3. Localización: ArUco de pared vs mission pads

El EDU trae **mission pads** (8 tarjetas con IDs 1–8) que detecta con su **cámara inferior**, dando
posición relativa al pad sin montar nada en la pared:

```python
t.enable_mission_pads()
t.set_mission_pad_detection_direction(0)   # 0=abajo, 1=adelante, 2=ambos
pad_id = t.get_mission_pad_id()            # -1 si no ve ninguno
x = t.get_mission_pad_distance_x()         # cm relativos al pad
y = t.get_mission_pad_distance_y()
z = t.get_mission_pad_distance_z()
```

**Trade-offs frente al ArUco de pared de este proyecto:**

| | ArUco de pared (este proyecto) | Mission pads (EDU) |
|---|---|---|
| Montaje | imprimir + pegar 6 markers en pared | poner pads en el piso |
| Cámara usada | frontal | inferior |
| Rango/altura | amplio (vuela de frente a la pared) | limitado (el pad debe estar bajo el dron, a poca altura) |
| Cobertura | área grande frente a la pared | zona acotada sobre los pads |
| Control | tú defines todo (transparente para la tesis) | caja negra del firmware |

Para una **formación que se desplaza por una sala**, el ArUco de pared cubre más espacio; los mission
pads obligan a volar bajo y sobre las tarjetas. Por eso, incluso con EDU, podría convenir **mantener la
localización ArUco frontal de este repo** y aprovechar del EDU solo el modo estación/enjambre.

## 4. Qué se perdería (y por qué aquí no se usó EDU)

- **OE3 (red Ad-Hoc) pierde sentido literal:** con un solo computador no hay enlace entre máquinas que
  caracterizar ni degradar. El estudio de latencia/pérdida/protocolo binario CRC y la inyección con
  `tc/netem`+IFB —una parte central de este trabajo— solo aplica cuando la cooperación cruza una red
  física real entre dos nodos. El Tello estándar **forzó** ese escenario, que resultó didácticamente
  valioso.
- **Disponibilidad:** el proyecto se hizo con el hardware disponible (Tello estándar).

## 5. Recomendación para quien continúe

- Si el objetivo es **escalar el número de drones** o simplificar el control: migrar a **Tello EDU +
  `TelloSwarm`** (un PC, modo estación).
- Si el objetivo es **estudiar la red cooperativa** (como aquí): conservar el esquema de **un computador
  por dron + backbone Ethernet**, sea con Tello estándar o EDU en APs separados.
- Híbrido razonable: **Tello EDU en modo estación** para la conectividad, pero **mantener la
  localización ArUco frontal** de este repo por su mayor rango, y conservar el **protocolo binario UDP**
  si se quiere seguir evaluando comunicación entre nodos.

> Las APIs citadas (`connect_to_wifi`, `TelloSwarm`, `enable_mission_pads`, `get_mission_pad_*`)
> pertenecen a `djitellopy` y al SDK del Tello EDU. Verifica la versión de tu firmware/SDK antes de
> depender de ellas.
