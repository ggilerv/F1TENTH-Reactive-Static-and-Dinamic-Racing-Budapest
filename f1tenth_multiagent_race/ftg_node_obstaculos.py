#!/usr/bin/env python3
"""
Follow The Gap + Disparity Extender — variante para obstáculos estáticos
==========================================================================

Autor: George Gabriel Giler Vega

Copia de `ftg_node.py` (el controlador de la Parte 1, afinado para seguir
las paredes del circuito de Budapest a máxima velocidad), con los valores
por defecto reajustados para la Prueba 1 de la Parte 2: 5 obstáculos
pequeños y aislados (círculos de ~0.4 m) colocados a un costado de la
pista. El algoritmo es idéntico; solo cambian los parámetros por
defecto, ya que un obstáculo pequeño y aislado necesita una burbuja de
seguridad y una distancia de alerta distintas a las de una pared continua.

Ver `ftg_node.py` para la documentación completa de cada parámetro y del
pipeline de control.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class FollowTheGapObstaculos(Node):
    """Nodo ROS 2 Follow The Gap, reajustado para obstáculos estáticos pequeños."""

    def __init__(self):
        """Declara los parámetros ROS 2, crea la suscripción a `/scan` y el
        publicador de `/drive`, e inicializa el estado interno del controlador."""
        super().__init__('follow_the_gap_obstaculos')

        self.declare_parameter('rango_maximo', 10.0)
        self.declare_parameter('radio_vehiculo', 0.40)
        self.declare_parameter('fov_grados', 180.0)
        self.declare_parameter('umbral_disparidad', 0.30)
        self.declare_parameter('distancia_alerta_burbuja', 2.2)
        self.declare_parameter('factor_umbral_libre', 1.6)
        self.declare_parameter('ventana_suavizado', 3)
        self.declare_parameter('histeresis_hueco', 0.25)
        self.declare_parameter('cono_proximidad_grados', 100.0)

        self.declare_parameter('velocidad_max', 18.0)
        self.declare_parameter('velocidad_min', 1.2)
        self.declare_parameter('alpha_steering', 0.60)

        self.declare_parameter('a_lat_max', 5.8)
        self.declare_parameter('a_freno_max', 8.5)
        self.declare_parameter('max_aceleracion', 8.5)
        self.declare_parameter('wheelbase', 0.3302)
        self.declare_parameter('max_steering_angle', 0.4189)
        self.declare_parameter('rango_frenado', 12.0)
        self.declare_parameter('cono_frenado_grados', 55.0)

        self.declare_parameter('beta_filtro_temporal', 0.40)
        self.declare_parameter('max_rate_steering', 400.0)
        self.declare_parameter('beta_filtro_temporal_max', 0.95)
        self.declare_parameter('max_rate_steering_alta', 420.0)
        self.declare_parameter('umbral_cambio_bajo', 0.05)
        self.declare_parameter('umbral_cambio_alto', 0.40)

        self.declare_parameter('max_delta_objetivo_grados', 20.0)
        self.declare_parameter('velocidad_steer_completo', 4.5)

        self._cargar_parametros()

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        self.angulo_anterior = 0.0
        self.velocidad_anterior = 0.0
        self.ultimo_tiempo = self.get_clock().now()
        self._ranges_ema = None
        self._factor_cambio = 0.0
        self._idx_objetivo_anterior = None

        self.get_logger().info(
            'Follow The Gap (obstaculos) node initialized. '
            f'velocidad_max={self.VELOCIDAD_MAX:.1f} m/s, '
            f'distancia_alerta_burbuja={self.DISTANCIA_ALERTA_BURBUJA:.2f} m')

    def _cargar_parametros(self):
        """Lee los parámetros ROS 2 declarados y los asigna como atributos de instancia."""
        gp = self.get_parameter
        self.RANGO_MAXIMO = float(gp('rango_maximo').value)
        self.RADIO_VEHICULO = float(gp('radio_vehiculo').value)
        self.FOV_RAD = math.radians(float(gp('fov_grados').value))
        self.UMBRAL_DISPARIDAD = float(gp('umbral_disparidad').value)
        self.DISTANCIA_ALERTA_BURBUJA = float(gp('distancia_alerta_burbuja').value)
        self.FACTOR_UMBRAL_LIBRE = float(gp('factor_umbral_libre').value)
        self.VENTANA_SUAVIZADO = max(1, int(gp('ventana_suavizado').value))
        self.HISTERESIS_HUECO = float(gp('histeresis_hueco').value)
        self.CONO_PROXIMIDAD_RAD = math.radians(float(gp('cono_proximidad_grados').value))
        self.VELOCIDAD_MAX = float(gp('velocidad_max').value)
        self.VELOCIDAD_MIN = float(gp('velocidad_min').value)
        self.ALPHA = float(gp('alpha_steering').value)
        self.A_LAT_MAX = float(gp('a_lat_max').value)
        self.A_FRENO_MAX = float(gp('a_freno_max').value)
        self.MAX_ACCEL = float(gp('max_aceleracion').value)
        self.WHEELBASE = float(gp('wheelbase').value)
        self.MAX_STEER = float(gp('max_steering_angle').value)
        self.RANGO_FRENADO = float(gp('rango_frenado').value)
        self.CONO_FRENADO_RAD = math.radians(float(gp('cono_frenado_grados').value))
        self.BETA_FILTRO_TEMPORAL = float(gp('beta_filtro_temporal').value)
        self.MAX_RATE_STEERING_RAD_S = math.radians(float(gp('max_rate_steering').value))
        self.BETA_FILTRO_TEMPORAL_MAX = float(gp('beta_filtro_temporal_max').value)
        self.MAX_RATE_STEERING_ALTA_RAD_S = math.radians(float(gp('max_rate_steering_alta').value))
        self.UMBRAL_CAMBIO_BAJO = float(gp('umbral_cambio_bajo').value)
        self.UMBRAL_CAMBIO_ALTO = float(gp('umbral_cambio_alto').value)
        self.MAX_DELTA_OBJETIVO_RAD = math.radians(float(gp('max_delta_objetivo_grados').value))
        self.V_STEER_COMPLETO = float(gp('velocidad_steer_completo').value)

    def scan_callback(self, msg: LaserScan):
        """Ciclo principal de control, ejecutado en cada escaneo del LiDAR
        (idéntico al de `ftg_node.py`; ver ese archivo para el detalle)."""
        ahora = self.get_clock().now()
        dt = (ahora - self.ultimo_tiempo).nanoseconds * 1e-9
        if dt <= 0.0 or dt > 0.5:
            dt = 0.02
        self.ultimo_tiempo = ahora

        angle_increment = max(msg.angle_increment, 1e-6)
        n_total = len(msg.ranges)
        if n_total < 10:
            return

        centro_idx_total = n_total // 2

        ranges_completos = np.array(msg.ranges, dtype=np.float64)
        ranges_completos = np.where(np.isfinite(ranges_completos),
                                     ranges_completos, 100.0)
        ranges_completos = np.clip(ranges_completos, 0.0, 100.0)

        medio_fov_idx = int(round((self.FOV_RAD / 2.0) / angle_increment))
        start_idx = max(0, centro_idx_total - medio_fov_idx)
        end_idx = min(n_total, centro_idx_total + medio_fov_idx)
        ranges = ranges_completos[start_idx:end_idx].copy()
        ranges = np.clip(ranges, 0.0, self.RANGO_MAXIMO)

        if self.VENTANA_SUAVIZADO > 1:
            kernel = np.ones(self.VENTANA_SUAVIZADO) / self.VENTANA_SUAVIZADO
            ranges_suaves = np.convolve(ranges, kernel, mode='same')
        else:
            ranges_suaves = ranges

        min_dist_bruto = float(np.min(ranges_suaves))

        idx_centro_local = len(ranges_suaves) // 2
        medio_cono_prox_idx = max(1, int(round(
            (self.CONO_PROXIMIDAD_RAD / 2.0) / angle_increment)))
        i0_prox = max(0, idx_centro_local - medio_cono_prox_idx)
        i1_prox = min(len(ranges_suaves), idx_centro_local + medio_cono_prox_idx + 1)
        min_dist_frontal = float(np.min(ranges_suaves[i0_prox:i1_prox]))

        ranges_ft = self._filtrar_temporalmente(ranges_suaves)
        ranges_seguros = self._extender_disparidades(ranges_ft, angle_increment)
        ranges_seguros = self._aplicar_burbuja_vectorizada(ranges_seguros,
                                                            angle_increment)

        idx_centro_fov = len(ranges_seguros) // 2
        umbral_libre = self.FACTOR_UMBRAL_LIBRE * self.RADIO_VEHICULO
        best_idx = self._elegir_objetivo(ranges_seguros, idx_centro_fov,
                                          umbral_libre)

        real_idx = best_idx + start_idx
        steering_angle = msg.angle_min + real_idx * angle_increment
        steering_angle = max(-self.MAX_STEER, min(self.MAX_STEER, steering_angle))

        steering_angle = max(
            self.angulo_anterior - self.MAX_DELTA_OBJETIVO_RAD,
            min(self.angulo_anterior + self.MAX_DELTA_OBJETIVO_RAD,
                steering_angle))

        smoothed_steering = (self.ALPHA * steering_angle
                             + (1.0 - self.ALPHA) * self.angulo_anterior)
        smoothed_steering = max(-self.MAX_STEER,
                                min(self.MAX_STEER, smoothed_steering))

        tasa_efectiva = (self.MAX_RATE_STEERING_RAD_S
                         + self._factor_cambio
                         * (self.MAX_RATE_STEERING_ALTA_RAD_S
                            - self.MAX_RATE_STEERING_RAD_S))
        max_delta_steer = tasa_efectiva * dt
        delta_steer = smoothed_steering - self.angulo_anterior
        delta_steer = max(-max_delta_steer, min(max_delta_steer, delta_steer))
        smoothed_steering = self.angulo_anterior + delta_steer
        smoothed_steering = max(-self.MAX_STEER,
                                min(self.MAX_STEER, smoothed_steering))

        v_actual = max(self.velocidad_anterior, 0.5)
        if v_actual > self.V_STEER_COMPLETO:
            max_steer_actual = min(
                self.MAX_STEER,
                self.MAX_STEER * self.V_STEER_COMPLETO / v_actual)
            smoothed_steering = max(-max_steer_actual,
                                    min(max_steer_actual, smoothed_steering))

        self.angulo_anterior = smoothed_steering

        distancia_frenado = self._distancia_frenado_extendida(
            ranges_completos, real_idx, angle_increment, centro_idx_total)

        velocidad_objetivo = self._planificar_velocidad(
            smoothed_steering, distancia_frenado, min_dist_bruto, min_dist_frontal)

        if velocidad_objetivo > self.velocidad_anterior:
            max_delta = self.MAX_ACCEL * dt
        else:
            max_delta = self.A_FRENO_MAX * dt
        delta = velocidad_objetivo - self.velocidad_anterior
        delta = max(-max_delta, min(max_delta, delta))
        velocidad_final = self.velocidad_anterior + delta
        velocidad_final = max(0.0, min(self.VELOCIDAD_MAX, velocidad_final))
        self.velocidad_anterior = velocidad_final

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = 'base_link'
        drive_msg.drive.steering_angle = smoothed_steering
        drive_msg.drive.speed = velocidad_final
        self.drive_pub.publish(drive_msg)

    def _filtrar_temporalmente(self, ranges_suaves):
        """Filtro EMA adaptativo sobre el escaneo (ver `ftg_node.py`)."""
        if (self._ranges_ema is None
                or len(self._ranges_ema) != len(ranges_suaves)):
            self._ranges_ema = ranges_suaves.copy()
            self._factor_cambio = 0.0
            return self._ranges_ema

        discrepancia = float(np.mean(np.abs(ranges_suaves - self._ranges_ema)))
        rango_umbral = max(self.UMBRAL_CAMBIO_ALTO - self.UMBRAL_CAMBIO_BAJO,
                           1e-6)
        factor = (discrepancia - self.UMBRAL_CAMBIO_BAJO) / rango_umbral
        factor = max(0.0, min(1.0, factor))
        self._factor_cambio = factor

        beta = (self.BETA_FILTRO_TEMPORAL
                + factor * (self.BETA_FILTRO_TEMPORAL_MAX
                            - self.BETA_FILTRO_TEMPORAL))
        self._ranges_ema = (beta * ranges_suaves
                            + (1.0 - beta) * self._ranges_ema)
        return self._ranges_ema

    def _distancia_frenado_extendida(self, ranges_completos, idx_objetivo_total,
                                      angle_increment, idx_centro_total):
        """Distancia libre de frenado, filtrada por ancho de carril (ver `ftg_node.py`)."""
        cono_idx = max(1, int(round(self.CONO_FRENADO_RAD / angle_increment)))
        n = len(ranges_completos)
        ancho_carril = self.RADIO_VEHICULO * 1.3

        def peor_en_carril(idx):
            i0 = max(0, idx - cono_idx)
            i1 = min(n, idx + cono_idx + 1)
            segmento = ranges_completos[i0:i1]
            angulos = (np.arange(i0, i1) - idx) * angle_increment
            lateral = segmento * np.sin(angulos)
            valido = np.abs(lateral) < ancho_carril
            if not np.any(valido):
                return self.RANGO_FRENADO
            return float(np.min(segmento[valido]))

        idx_objetivo_total = max(0, min(n - 1, idx_objetivo_total))
        idx_centro_total = max(0, min(n - 1, idx_centro_total))
        distancia = min(peor_en_carril(idx_objetivo_total),
                        peor_en_carril(idx_centro_total))
        return min(distancia, self.RANGO_FRENADO)

    def _extender_disparidades(self, ranges, angle_increment):
        """Disparity Extender (ver `ftg_node.py`)."""
        r = ranges.copy()
        n = len(ranges)
        diffs = np.diff(ranges)
        indices_disparidad = np.where(np.abs(diffs) > self.UMBRAL_DISPARIDAD)[0]

        for i in indices_disparidad:
            if ranges[i] < ranges[i + 1]:
                near_val = max(float(ranges[i]), 0.05)
                n_ext = int(math.ceil(
                    math.atan2(self.RADIO_VEHICULO, near_val) / angle_increment))
                r[i + 1:min(n, i + 1 + n_ext)] = np.minimum(
                    r[i + 1:min(n, i + 1 + n_ext)], near_val)
            else:
                near_val = max(float(ranges[i + 1]), 0.05)
                n_ext = int(math.ceil(
                    math.atan2(self.RADIO_VEHICULO, near_val) / angle_increment))
                inicio = max(0, i + 1 - n_ext)
                r[inicio:i + 1] = np.minimum(r[inicio:i + 1], near_val)
        return r

    def _aplicar_burbuja_vectorizada(self, ranges, angle_increment):
        """Burbuja de seguridad vectorizada (ver `ftg_node.py`)."""
        bajo_umbral = ranges < self.DISTANCIA_ALERTA_BURBUJA
        if not np.any(bajo_umbral):
            return ranges

        r = ranges.copy()
        n = len(ranges)
        distancias_seguras = np.maximum(ranges, 0.05)
        angulos_burbuja = np.arctan2(self.RADIO_VEHICULO, distancias_seguras)
        n_burbuja = np.ceil(angulos_burbuja / angle_increment).astype(int)

        cierre = np.zeros(n, dtype=bool)
        for idx in np.where(bajo_umbral)[0]:
            nb = int(n_burbuja[idx])
            cierre[max(0, idx - nb):min(n, idx + nb + 1)] = True
        r[cierre] = 0.0
        return r

    def _elegir_objetivo(self, ranges, idx_centro, umbral_libre):
        """
        Selección del hueco objetivo (ver `ftg_node.py`), con histéresis:
        si ya había un hueco elegido en el ciclo anterior, se mantiene
        mientras siga existiendo, a menos que otro hueco sea al menos
        `HISTERESIS_HUECO` (fracción) más ancho. Sin esto, un obstáculo
        pequeño cercano puede hacer que el hueco más ancho "salte" de un
        lado al otro del obstáculo de un ciclo a otro (el auto duda entre
        esquivarlo por la izquierda o la derecha en vez de comprometerse).
        """
        mask = ranges > umbral_libre
        cambios = np.diff(mask.astype(np.int8))
        inicios = list(np.where(cambios == 1)[0] + 1)
        finales = list(np.where(cambios == -1)[0])
        if mask[0]:
            inicios.insert(0, 0)
        if mask[-1]:
            finales.append(len(mask) - 1)

        if not inicios:
            self._idx_objetivo_anterior = None
            return int(np.argmax(ranges))

        mejor_inicio = inicios[0]
        mejor_fin = finales[0]
        mejor_largo = finales[0] - inicios[0] + 1
        for s, e in zip(inicios, finales):
            largo = e - s + 1
            if largo > mejor_largo:
                mejor_largo, mejor_inicio, mejor_fin = largo, s, e
            elif largo == mejor_largo:
                ca = (mejor_inicio + mejor_fin) / 2.0
                cn = (s + e) / 2.0
                if abs(cn - idx_centro) < abs(ca - idx_centro):
                    mejor_inicio, mejor_fin = s, e

        if self._idx_objetivo_anterior is not None:
            for s, e in zip(inicios, finales):
                if s <= self._idx_objetivo_anterior <= e:
                    largo_actual = e - s + 1
                    if largo_actual >= mejor_largo / (1.0 + self.HISTERESIS_HUECO):
                        mejor_inicio, mejor_fin = s, e
                    break

        segmento = ranges[mejor_inicio:mejor_fin + 1]
        indices_segmento = np.arange(mejor_inicio, mejor_fin + 1)
        centroide = float(np.sum(indices_segmento * segmento) / np.sum(segmento))
        objetivo = int(round(centroide))
        self._idx_objetivo_anterior = objetivo
        return objetivo

    def _planificar_velocidad(self, steering_angle, distancia_frenado,
                               min_dist_bruto, min_dist_frontal):
        """
        Planificación de velocidad (ver `ftg_node.py`), con un límite
        adicional por proximidad frontal (`v_proximidad`): a diferencia
        de `distancia_frenado` (que solo mira un cono angosto alrededor
        del objetivo y del frente), este límite usa el punto más cercano
        dentro de un cono más amplio (`cono_proximidad_grados`) centrado
        en el auto. Es necesario porque un obstáculo pequeño a un
        costado de la pista puede quedar fuera del cono de frenado
        angosto mientras el auto sigue acelerando, y solo lo detecta la
        burbuja de seguridad cuando ya está muy cerca. Se usa un cono
        (no los 180° completos de `min_dist_bruto`) para no frenar de
        más en curvas cerradas normales, donde la pared queda cerca a
        los costados sin que eso sea peligroso.
        """
        if abs(steering_angle) > 1e-3:
            curvatura = abs(math.tan(steering_angle)) / self.WHEELBASE
            v_curvatura = math.sqrt(self.A_LAT_MAX / curvatura)
        else:
            v_curvatura = self.VELOCIDAD_MAX

        distancia_segura = max(0.0, distancia_frenado - self.RADIO_VEHICULO)
        v_frenado = math.sqrt(2.0 * self.A_FRENO_MAX * distancia_segura)

        distancia_proximidad_segura = max(0.0, min_dist_frontal - self.RADIO_VEHICULO)
        v_proximidad = math.sqrt(2.0 * self.A_FRENO_MAX * distancia_proximidad_segura)

        velocidad = min(self.VELOCIDAD_MAX, v_curvatura, v_frenado, v_proximidad)
        velocidad = max(self.VELOCIDAD_MIN, velocidad)

        umbral_emergencia = self.RADIO_VEHICULO + 0.10
        if min_dist_bruto < umbral_emergencia:
            velocidad = min(velocidad, self.VELOCIDAD_MIN * 0.4)

        return velocidad


def main(args=None):
    """Punto de entrada del nodo: inicializa ROS 2, hace spin y cierra limpiamente."""
    rclpy.init(args=args)
    nodo = FollowTheGapObstaculos()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
