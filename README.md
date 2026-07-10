# 🏎️ F1TENTH Reactive Static and Dynamic Racing — Budapest (Parte 2)

**Autor:** George Gabriel Giler Vega
**Institución:** Escuela Superior Politécnica del Litoral (ESPOL)
**Stack tecnológico:** ROS 2 (Humble), Python, Simulador F1TENTH

Segunda parte del proyecto de carreras reactivas en el circuito de Budapest:
evasión de obstáculos **estáticos** y **dinámicos**, construida sobre el
controlador Follow The Gap de
[`f1_reactive`](https://github.com/ggilerv/F1TENTH-Reactive-Racing-Budapest)
(Parte 1). Este paquete es autocontenido: incluye su propio bridge
multi-agente, sus propios controladores especializados y su propio
cronómetro con detección de colisión, sin modificar `f1tenth_gym_ros` (el
paquete base del curso) ni `f1_reactive`.

## Tabla de contenidos

1. [Arquitectura](#arquitectura)
2. [Demostración en pista](#demostración-en-pista)
3. [Las dos pruebas](#las-dos-pruebas)
   1. [Prueba 1 — Obstáculos estáticos](#prueba-1--obstáculos-estáticos)
   2. [Prueba 2 — Obstáculos dinámicos](#prueba-2--obstáculos-dinámicos)
4. [Detección de colisión y criterio de aprobación](#detección-de-colisión-y-criterio-de-aprobación)
5. [Instalación desde cero](#instalación-desde-cero)
6. [Estructura del repositorio](#estructura-del-repositorio)
7. [Parámetros principales](#parámetros-principales)

## Arquitectura

- **`gym_bridge_multi.py`** — fork del bridge ROS 2 ↔ `f110_gym` de
  `f1tenth_gym_ros`, generalizado de "ego + 1 oponente" a **N agentes**
  arbitrarios (hasta 8), cada uno con su propio namespace, tópicos de
  scan/odom/drive y pose inicial. Publica además `/collisions`
  (`std_msgs/Float32MultiArray`, un valor 0/1 por agente), tomado
  directamente de la detección de colisiones del motor físico, para dar
  una verificación exacta de choque en la Prueba 2. El motor físico
  (`f110_gym`) ya soporta N agentes de forma nativa; el bridge original
  solo estaba limitado a 2 por la capa ROS 2, que es lo que este fork
  generaliza.
- **`ftg_node_obstaculos.py`** — copia del controlador `ftg_node.py` de la
  Parte 1, con los parámetros reajustados para esquivar obstáculos
  pequeños y aislados a un costado de la pista (no solo seguir las
  paredes de un pasillo continuo): burbuja de seguridad más amplia,
  frenado por proximidad frontal (no solo un cono angosto), histéresis en
  la elección de hueco para no dudar entre esquivar por un lado u otro, y
  un tope de velocidad mayor en recta. Se conservan además dos copias de
  respaldo (`ftg_node_obstaculos_respaldo.py`,
  `ftg_node_obstaculos_respaldo2.py`) en puntos de ajuste anteriores ya
  validados sin choques, por si un ajuste posterior resultara menos
  confiable — ver [Parámetros principales](#parámetros-principales).
- **`lap_timer.py`** — cronómetro de vueltas (igual al de la Parte 1),
  extendido con detección de colisión: usa `/collisions` como fuente
  exacta cuando está disponible (Prueba 2), o una heurística de velocidad
  sostenidamente nula cuando no (Prueba 1, que corre sobre el bridge
  original sin ese tópico).

## Demostración en pista

Evidencia de la Prueba 1 (obstáculos estáticos) completada de forma autónoma:

https://youtu.be/vNRCDcRi82w

## Las dos pruebas

### Prueba 1 — Obstáculos estáticos

5 obstáculos fijos, dibujados directamente sobre una copia del mapa de
Budapest (`maps/Budapest_map_obstaculos.png`), a un costado del carril
(no en el centro) para exigir una maniobra real de esquive. Como el LiDAR
los ve exactamente igual que a los muros de la pista, **corre con el
bridge original de `f1tenth_gym_ros`, sin ningún cambio** — solo cambia
el mapa.

```bash
source ~/F1Tenth-Repository/install/setup.bash

# Terminal 1: bridge original + RViz + mapa, apuntando al mapa con obstáculos
ros2 launch f1tenth_multiagent_race test1_static_obstacles_launch.py

# Terminal 2: controlador especializado para obstáculos estáticos
ros2 run f1tenth_multiagent_race ftg_control_obstaculos

# Terminal 3: cronómetro con detección de colisión
ros2 run f1tenth_multiagent_race lap_timer --ros-args -p track_name:="BUDAPEST (OBSTÁCULOS ESTÁTICOS)"
```

### Prueba 2 — Obstáculos dinámicos

3 agentes: el ego (`velocidad_max=17`) y 2 autos adicionales corriendo el
mismo `ftg_control` de la Parte 1 pero con `velocidad_max=4` (velocidad
moderada), para que el ego sea claramente el auto más rápido/objetivo.
Esta prueba sí necesita el bridge multi-agente de este paquete
(`gym_bridge_multi`), ya que el bridge original solo soporta 2 agentes en
total.

```bash
source ~/F1Tenth-Repository/install/setup.bash

# Terminal 1: bridge multi-agente + RViz + robot_state_publisher (los autos no se mueven todavía)
ros2 launch f1tenth_multiagent_race multiagent_bridge_launch.py \
    agents_config:=~/F1Tenth-Repository/src/f1tenth_multiagent_race/config/agents_test2_budapest.yaml \
    map_path:=~/F1Tenth-Repository/src/f1tenth_gym_ros/maps/Budapest_map

# Terminal 2: controlador del ego
ros2 run f1_reactive ftg_control --ros-args \
    -r /scan:=ego_racecar/scan -r /drive:=ego_racecar/drive \
    -p velocidad_max:=17.0

# Terminal 3: controlador del obstáculo 1 (velocidad moderada)
ros2 run f1_reactive ftg_control --ros-args \
    -r /scan:=obstaculo_1/scan -r /drive:=obstaculo_1/drive \
    -p velocidad_max:=4.0

# Terminal 4: controlador del obstáculo 2 (velocidad moderada)
ros2 run f1_reactive ftg_control --ros-args \
    -r /scan:=obstaculo_2/scan -r /drive:=obstaculo_2/drive \
    -p velocidad_max:=4.0

# Terminal 5: cronómetro con detección de colisión (ground truth, vía /collisions)
ros2 run f1tenth_multiagent_race lap_timer --ros-args -p track_name:="BUDAPEST (OBSTÁCULOS DINÁMICOS)"
```

El launch de la prueba 2 solo levanta el bridge, RViz, el mapa y los
modelos de los 3 autos — ningún auto se mueve hasta que lances su
`ftg_control` a mano, igual que en la prueba 1.

## Detección de colisión y criterio de aprobación

`lap_timer` muestra en vivo la vuelta actual, el mejor tiempo, el
historial, y el resultado de la prueba:
- **Prueba 2** (bridge multi-agente): usa el tópico `/collisions`
  (ground truth exacto, publicado por `gym_bridge_multi` a partir de la
  detección de colisiones del propio motor físico) — cualquier choque del
  ego, contra pared o contra otro agente, marca la prueba como fallida.
- **Prueba 1** (bridge original, sin `/collisions`): usa una heurística de
  respaldo — si la velocidad del ego queda sostenidamente cerca de cero,
  se asume un choque.

Criterio de aprobación en ambas pruebas: **10 vueltas consecutivas sin que
se dispare ninguna de las dos detecciones de colisión.**

## Instalación desde cero

Estos pasos asumen que ya tienes ROS 2 Humble, el simulador F1TENTH base y
el controlador de la Parte 1 instalados. Si partes de cero, sigue primero
la guía de instalación completa de
[`F1TENTH-Reactive-Racing-Budapest`](https://github.com/ggilerv/F1TENTH-Reactive-Racing-Budapest)
(pasos 1 a 6: ROS 2, simulador, paquete `f1_reactive`, mapa de Budapest y
`sim.yaml`) — este repositorio se agrega como un paquete más sobre ese
mismo workspace.

### 1. Agregar este paquete al workspace

Clona este repositorio directamente como el paquete `f1tenth_multiagent_race`
dentro de `src/`, junto a `f1tenth_gym_ros` y `f1_reactive`:
```bash
cd ~/F1Tenth-Repository/src
git clone https://github.com/ggilerv/F1TENTH-Reactive-Static-and-Dinamic-Racing-Budapest.git f1tenth_multiagent_race
```

### 2. Configurar la ruta del mapa de la Prueba 1

Edita `~/F1Tenth-Repository/src/f1tenth_multiagent_race/config/sim_test1_budapest.yaml`
y reemplaza `<tu_usuario>` en `map_path` por el nombre de usuario de tu
máquina:
```yaml
    map_path: '/home/<tu_usuario>/F1Tenth-Repository/src/f1tenth_multiagent_race/maps/Budapest_map_obstaculos'
```

### 3. Compilar el workspace

```bash
cd ~/F1Tenth-Repository
colcon build --symlink-install
source install/setup.bash
```

### 4. Ejecutar las pruebas

Ver [Las dos pruebas](#las-dos-pruebas) más arriba para los comandos
completos de cada una.

## Estructura del repositorio

```
f1tenth_multiagent_race/
├── f1tenth_multiagent_race/
│   ├── gym_bridge_multi.py               # bridge N-agentes (solo prueba 2)
│   ├── ftg_node_obstaculos.py            # controlador especializado, prueba 1 (versión actual)
│   ├── ftg_node_obstaculos_respaldo.py   # respaldo: primer ajuste validado sin choques
│   ├── ftg_node_obstaculos_respaldo2.py  # respaldo: segundo ajuste validado sin choques
│   └── lap_timer.py                      # cronómetro + detección de colisión
├── launch/
│   ├── test1_static_obstacles_launch.py  # launch de la prueba 1 (bridge original + mapa con obstáculos)
│   ├── multiagent_bridge_launch.py       # launch de la prueba 2
│   ├── racecar.xacro                     # modelo del auto, compartido por todos los agentes
│   └── gym_bridge.rviz
├── config/
│   ├── sim.yaml                    # parámetros globales del bridge multi-agente
│   ├── agents_test2_budapest.yaml  # ego + 2 obstáculos dinámicos
│   └── sim_test1_budapest.yaml     # parámetros del bridge ORIGINAL, apuntando al mapa con obstáculos
├── maps/
│   └── Budapest_map_obstaculos.{png,yaml}  # copia de Budapest con 5 obstáculos dibujados
└── package.xml / setup.py / ...
```

## Parámetros principales

Todos los parámetros de `ftg_node_obstaculos.py` se declaran como
parámetros de ROS 2 (ver docstring de la clase para la lista completa).
Los que más lo distinguen del controlador base de la Parte 1:

| Parámetro | Efecto |
|---|---|
| `distancia_alerta_burbuja` | Radio de la burbuja de seguridad alrededor de cualquier punto cercano del escaneo; más grande que en la Parte 1 para reaccionar antes ante un obstáculo pequeño. |
| `cono_proximidad_grados` | Ancho angular del cono frontal usado para el frenado por proximidad — deliberadamente más angosto que los 180° completos del LiDAR, para no frenar de más en curvas cerradas normales donde la pared queda cerca a los costados sin peligro real. |
| `histeresis_hueco` | Fracción de ancho adicional que debe tener un hueco alternativo para "robarle" la elección al hueco del ciclo anterior — evita que el auto dude entre esquivar un obstáculo por la izquierda o la derecha. |
| `velocidad_max` | Velocidad máxima absoluta en recta. |
| `a_lat_max` | Aceleración lateral máxima admisible en curva. |
| `a_freno_max` | Desaceleración máxima de frenado, usada tanto por el frenado angosto como por el de proximidad. |

Los parámetros de `gym_bridge_multi.py` (`num_agents`, `agent_{i}_*`,
`scan_fov`, `scan_beams`, etc.) se documentan en el docstring de
`GymBridgeMulti` y se arman automáticamente a partir de
`config/agents_*.yaml` por `multiagent_bridge_launch.py` — no hace falta
tocarlos a mano.
