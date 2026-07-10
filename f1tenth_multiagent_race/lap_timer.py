#!/usr/bin/env python3
"""
F1 Timing Tower — F1TENTH Reactive Racing (Budapest)
=======================================================

Autor: George Gabriel Giler Vega
Repositorio: https://github.com/ggilerv/F1TENTH-Reactive-Racing-Budapest

Nodo ROS 2 de cronometraje de vueltas para el simulador F1TENTH. Se
suscribe a la odometría del vehículo (`odom_topic`) y usa una máquina de
estados espacial, basada en la distancia euclidiana al punto de arranque,
para detectar el cruce de la línea de meta en cada vuelta. Muestra en
terminal un tablero de tiempos en vivo al estilo de una torre de
cronometraje de Fórmula 1 (vuelta actual, mejor vuelta, historial de
vueltas, y el estado de la prueba: aprobada o fallida por colisión).

Detección de vuelta:
    1. Espera a que la velocidad del vehículo supere un umbral mínimo
       para descartar el ruido de odometría en reposo; en ese instante
       ancla la línea de meta a la posición actual y arranca el reloj.
    2. Espera a que el vehículo se aleje más de `DISTANCIA_SALIDA` del
       punto de arranque (evita registrar una vuelta falsa antes de que
       el vehículo realmente haya completado el circuito).
    3. Una vez alejado, cuando el vehículo vuelve a estar a menos de
       `DISTANCIA_META` del punto de arranque, se registra la vuelta y
       se reinicia el ciclo.

Detección de colisión (para las pruebas de evasión de obstáculos):
    - Si el tópico `/collisions` existe (bridge multi-agente de
      `f1tenth_multiagent_race`, usado en la prueba de obstáculos
      dinámicos), se usa como fuente exacta (ground truth): la prueba
      falla en cuanto `collisions[0]` (el ego) sea 1.
    - Si ese tópico no existe (bridge original de un solo agente, usado
      en la prueba de obstáculos estáticos), se usa una heurística de
      respaldo: si la velocidad del ego queda sostenidamente cerca de
      cero durante la carrera, se asume una colisión (un choque real
      contra un obstáculo o muro deja al vehículo con velocidad ~0).
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray


class F1TimingTower(Node):
    """
    Nodo de cronometraje de vueltas y detección de colisión.

    Attributes:
        odom_sub: suscripción al tópico de odometría del vehículo ego.
        collision_sub: suscripción a `/collisions` (puede no publicar
            nunca si se corre sobre el bridge original de un solo
            agente; en ese caso se usa la heurística de velocidad).
        race_started (bool): True una vez que el vehículo superó el
            umbral de velocidad de arranque.
        start_x, start_y (float | None): coordenadas de la línea de
            meta, ancladas a la posición del vehículo en el instante de
            arranque.
        lap_start_time: timestamp ROS del inicio de la vuelta actual.
        laps_completed (int): número de vueltas completadas.
        TOTAL_LAPS (int): número total de vueltas de la sesión.
        best_lap_time (float): mejor tiempo de vuelta registrado [s].
        lap_history (list[float]): tiempo de cada vuelta completada [s].
        is_outside_start_zone (bool): True si el vehículo ya se alejó lo
            suficiente de la línea de meta como para poder registrar el
            próximo cruce como una vuelta válida.
        DISTANCIA_SALIDA (float): distancia mínima [m] a la que debe
            alejarse el vehículo de la línea de meta antes de que un
            regreso cuente como vuelta completada.
        DISTANCIA_META (float): distancia máxima [m] a la línea de meta
            para considerar que el vehículo la cruzó.
        test_failed (bool): True desde el instante en que se detecta una
            colisión del ego (por `/collisions` o por la heurística).
        fail_lap (int | None): número de vuelta en la que se detectó la
            falla, o None si no ha fallado.
        live_timer: temporizador que refresca el tablero en pantalla.
    """

    def __init__(self):
        """Declara los parámetros ROS 2, crea las suscripciones a
        odometría y colisiones, e inicializa el estado de la carrera y
        el temporizador de refresco del tablero."""
        super().__init__('lap_timer')

        self.declare_parameter('odom_topic', '/ego_racecar/odom')
        self.declare_parameter('track_name', 'BUDAPEST')
        self.declare_parameter('total_laps', 10)
        self.declare_parameter('umbral_velocidad_atascado', 0.15)
        self.declare_parameter('tiempo_atascado_colision', 1.5)

        odom_topic = self.get_parameter('odom_topic').value
        self.TRACK_NAME = self.get_parameter('track_name').value
        self.TOTAL_LAPS = int(self.get_parameter('total_laps').value)
        self.UMBRAL_VELOCIDAD_ATASCADO = float(
            self.get_parameter('umbral_velocidad_atascado').value)
        self.TIEMPO_ATASCADO_COLISION = float(
            self.get_parameter('tiempo_atascado_colision').value)

        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10)
        self.collision_sub = self.create_subscription(
            Float32MultiArray,
            '/collisions',
            self.collision_callback,
            10)

        self.race_started = False
        self.start_x = None
        self.start_y = None
        self.lap_start_time = None

        self.laps_completed = 0
        self.best_lap_time = float('inf')
        self.lap_history = []

        self.is_outside_start_zone = False
        self.DISTANCIA_SALIDA = 6.0
        self.DISTANCIA_META = 2.5

        self.collision_topic_visto = False
        self.test_failed = False
        self.fail_lap = None
        self._atascado_desde = None

        self.live_timer = self.create_timer(0.1, self.print_dashboard)

    def format_time(self, seconds):
        """
        Convierte un tiempo en segundos a formato de cronómetro F1
        (minutos:segundos.milisegundos).

        Args:
            seconds (float): tiempo en segundos.

        Returns:
            str: tiempo formateado como "M:SS.mmm".
        """
        m = int(seconds // 60)
        s = seconds % 60
        return f"{m}:{s:06.3f}"

    def collision_callback(self, msg):
        """
        Callback de `/collisions` (ground truth del bridge multi-agente):
        marca la prueba como fallida en cuanto el ego (índice 0) reporte
        colisión. Este tópico solo existe cuando se corre sobre
        `gym_bridge_multi`; si nunca llega ningún mensaje, se usa la
        heurística de velocidad en `odom_callback` como respaldo.

        Args:
            msg (std_msgs.msg.Float32MultiArray): vector de colisión por
                agente (0.0/1.0), índice 0 = ego.
        """
        self.collision_topic_visto = True
        if len(msg.data) > 0 and msg.data[0] > 0.5:
            self._marcar_fallo()

    def _marcar_fallo(self):
        """Marca la prueba como fallida en la vuelta actual, si no lo estaba ya."""
        if not self.test_failed:
            self.test_failed = True
            self.fail_lap = self.laps_completed + 1

    def odom_callback(self, msg):
        """
        Callback de odometría: implementa la máquina de estados espacial
        de detección de vueltas descrita en el docstring del módulo, y
        la heurística de colisión de respaldo (velocidad sostenida ~0)
        para cuando no hay tópico `/collisions` disponible.

        Ignora los mensajes una vez completadas todas las vueltas de la
        sesión (`TOTAL_LAPS`).

        Args:
            msg (nav_msgs.msg.Odometry): odometría del vehículo ego.
        """
        if self.laps_completed >= self.TOTAL_LAPS:
            return

        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        speed = math.hypot(vx, vy)

        if not self.race_started:
            self.start_x = current_x
            self.start_y = current_y

            if speed > 0.15:
                self.race_started = True
                self.lap_start_time = self.get_clock().now()
            return

        if not self.collision_topic_visto:
            if speed < self.UMBRAL_VELOCIDAD_ATASCADO:
                if self._atascado_desde is None:
                    self._atascado_desde = self.get_clock().now()
                else:
                    atascado_seg = (self.get_clock().now()
                                     - self._atascado_desde).nanoseconds / 1e9
                    if atascado_seg > self.TIEMPO_ATASCADO_COLISION:
                        self._marcar_fallo()
            else:
                self._atascado_desde = None

        dist = math.hypot(current_x - self.start_x, current_y - self.start_y)

        if not self.is_outside_start_zone:
            if dist > self.DISTANCIA_SALIDA:
                self.is_outside_start_zone = True
        else:
            if dist < self.DISTANCIA_META:
                self.record_lap()

    def record_lap(self):
        """
        Registra el cierre de una vuelta: calcula su duración, actualiza
        el mejor tiempo y el historial, y reinicia el cronómetro para la
        siguiente vuelta. Si se completó `TOTAL_LAPS`, detiene el
        refresco del tablero y muestra el estado final.
        """
        current_time = self.get_clock().now()
        lap_duration = (current_time - self.lap_start_time).nanoseconds / 1e9

        self.laps_completed += 1
        self.lap_history.append(lap_duration)

        if lap_duration < self.best_lap_time:
            self.best_lap_time = lap_duration

        self.lap_start_time = current_time
        self.is_outside_start_zone = False

        if self.laps_completed >= self.TOTAL_LAPS:
            self.live_timer.cancel()
            self.print_dashboard()

    def print_dashboard(self):
        """
        Refresca el tablero de cronometraje en la terminal: limpia la
        pantalla y muestra el estado de la sesión, la vuelta en curso,
        el mejor tiempo registrado, el historial completo de vueltas, y
        el resultado de la prueba (aprobada o fallida por colisión).
        """
        print('\033c', end='')

        print("="*55)
        print(f" 🏎️  F1TENTH TIMING TOWER - {self.TRACK_NAME} ")
        print("="*55)

        if not self.race_started:
            print("\n 🔴 STATUS: WAITING FOR START")
            print(" 🛑 Esperando que el vehículo acelere...\n")
            print("="*55)
            return

        if self.test_failed:
            print(f"\n 💥 COLISIÓN DETECTADA EN LA VUELTA {self.fail_lap} — PRUEBA FALLIDA 💥")
        elif self.laps_completed >= self.TOTAL_LAPS:
            print(f"\n 🏁 ¡BANDERA A CUADROS! {self.TOTAL_LAPS}/{self.TOTAL_LAPS} VUELTAS SIN COLISIÓN — PRUEBA APROBADA 🏁")
        else:
            print(f"\n 🟢 STATUS: RACE LIVE")

        print(f" 🔄 LAP: {min(self.laps_completed + 1, self.TOTAL_LAPS)} / {self.TOTAL_LAPS}\n")

        if self.laps_completed < self.TOTAL_LAPS:
            now = self.get_clock().now()
            current_lap_time = (now - self.lap_start_time).nanoseconds / 1e9
            print(f" ⏱️  CURRENT LAP:  {self.format_time(current_lap_time)}")
        else:
            print(f" ⏱️  CURRENT LAP:  --:--.---")

        if self.best_lap_time != float('inf'):
            print(f" 🟣 BEST LAP:     {self.format_time(self.best_lap_time)}")
        else:
            print(f" ⚪ BEST LAP:     --:--.---")

        print("\n [ HISTORIAL DE VUELTAS ]")

        if len(self.lap_history) == 0:
            print("    Aún no hay tiempos registrados.")
        else:
            for i, t in enumerate(self.lap_history):
                marker = "🟣" if t == self.best_lap_time else "⚪"
                print(f"    {marker} Vuelta {i + 1:02d}:   {self.format_time(t)}")

        print("\n" + "="*55)


def main(args=None):
    """Punto de entrada del nodo: inicializa ROS 2, hace spin y cierra limpiamente."""
    rclpy.init(args=args)
    node = F1TimingTower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
