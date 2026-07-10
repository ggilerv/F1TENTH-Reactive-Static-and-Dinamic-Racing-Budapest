#!/usr/bin/env python3
"""
Bridge ROS2 multi-agente para el simulador F1TENTH (fork de gym_bridge.py)
============================================================================

Autor: George Gabriel Giler Vega

Extiende el bridge original de f1tenth_gym_ros (limitado a 2 agentes: ego +
1 oponente) para soportar N agentes arbitrarios. Se usa específicamente
para la Prueba 2 de la Parte 2 (obstáculos dinámicos): ego + 2 autos
adicionales, los 3 con su propio `ftg_control`, donde el ego tiene
`velocidad_max` mayor a los otros dos.

(La Prueba 1, de obstáculos estáticos, no usa este bridge: se resuelve con
una copia del mapa de Budapest con los obstáculos dibujados directamente en
la imagen, corriendo con el bridge original de un solo agente.)

El motor físico (f110_gym) ya soporta N agentes de forma nativa; este bridge
solo generaliza la capa ROS2 (antes escrita como "bloque ego + un solo
bloque de oponente") a un loop sobre una lista de agentes. Cada agente se
declara con parámetros indexados `agent_{i}_*` (namespace, tópicos de
scan/odom/drive, pose inicial) y tiene su propio subscriptor de drive.

Este paquete NO modifica f1tenth_gym_ros (que pertenece al repo base del
curso): es un fork independiente, pensado para convivir en el mismo
workspace sin alterar el bridge original de la Parte 1.
"""

import functools

import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformBroadcaster

import gym
from transforms3d import euler


class Agent:
    """Estado en tiempo de ejecución de un agente (ego u obstáculo dinámico)."""

    def __init__(self, index, namespace, scan_topic, odom_topic, drive_topic,
                 sx, sy, stheta):
        """Inicializa el estado del agente en su pose de arranque
        (`sx`, `sy`, `stheta`) con velocidad y comando de dirección
        nulos; los publicadores/suscriptor se asignan después, una vez
        creados los tópicos correspondientes."""
        self.index = index
        self.namespace = namespace
        self.scan_topic = scan_topic
        self.odom_topic = odom_topic
        self.drive_topic = drive_topic

        self.pose = [sx, sy, stheta]
        self.speed = [0.0, 0.0, 0.0]
        self.requested_speed = 0.0
        self.steer = 0.0
        self.scan = []

        self.scan_pub = None
        self.odom_pub = None
        self.drive_sub = None


class GymBridgeMulti(Node):
    """
    Bridge ROS2 <-> f110_gym para N agentes.

    Publica, por cada agente: LaserScan, Odometry y las transformadas TF
    (base_link, ruedas, láser). Publica además un tópico `/collisions`
    (std_msgs/Float32MultiArray, uno por agente, 0.0/1.0) para detección
    exacta de choques, usada por el `lap_timer` extendido de `f1_reactive`.
    """

    def __init__(self):
        """Declara los parámetros globales y por-agente, crea el entorno
        `f110_gym` con `num_agents` agentes, y arranca los timers de
        física y de publicación de tópicos."""
        super().__init__('gym_bridge_multi')

        self.declare_parameter('scan_distance_to_base_link', 0.0)
        self.declare_parameter('scan_fov', 0.0)
        self.declare_parameter('scan_beams', 0)
        self.declare_parameter('map_path', '')
        self.declare_parameter('map_img_ext', '')
        self.declare_parameter('kb_teleop', False)
        self.declare_parameter('num_agents', 1)

        num_agents = int(self.get_parameter('num_agents').value)
        if num_agents < 1 or num_agents > 8:
            raise ValueError('num_agents debe estar entre 1 y 8.')

        for i in range(num_agents):
            self.declare_parameter(f'agent_{i}_namespace', f'agent_{i}')
            self.declare_parameter(f'agent_{i}_scan_topic', f'agent_{i}/scan')
            self.declare_parameter(f'agent_{i}_odom_topic', f'agent_{i}/odom')
            self.declare_parameter(f'agent_{i}_drive_topic', f'agent_{i}/drive')
            self.declare_parameter(f'agent_{i}_sx', 0.0)
            self.declare_parameter(f'agent_{i}_sy', 0.0)
            self.declare_parameter(f'agent_{i}_stheta', 0.0)

        scan_fov = self.get_parameter('scan_fov').value
        scan_beams = self.get_parameter('scan_beams').value
        self.angle_min = -scan_fov / 2.
        self.angle_max = scan_fov / 2.
        self.angle_inc = scan_fov / scan_beams
        self.scan_distance_to_base_link = self.get_parameter('scan_distance_to_base_link').value

        self.agents = []
        for i in range(num_agents):
            gp = self.get_parameter
            agent = Agent(
                index=i,
                namespace=gp(f'agent_{i}_namespace').value,
                scan_topic=gp(f'agent_{i}_scan_topic').value,
                odom_topic=gp(f'agent_{i}_odom_topic').value,
                drive_topic=gp(f'agent_{i}_drive_topic').value,
                sx=gp(f'agent_{i}_sx').value,
                sy=gp(f'agent_{i}_sy').value,
                stheta=gp(f'agent_{i}_stheta').value,
            )
            self.agents.append(agent)

        self.env = gym.make('f110_gym:f110-v0',
                             map=self.get_parameter('map_path').value,
                             map_ext=self.get_parameter('map_img_ext').value,
                             num_agents=num_agents)

        poses = np.array([agent.pose for agent in self.agents])
        self.obs, _, self.done, _ = self.env.reset(poses)
        self._update_sim_state()

        self.drive_timer = self.create_timer(0.01, self.drive_timer_callback)
        self.timer = self.create_timer(0.004, self.timer_callback)

        self.br = TransformBroadcaster(self)

        self.collision_pub = self.create_publisher(Float32MultiArray, '/collisions', 10)

        for agent in self.agents:
            agent.scan_pub = self.create_publisher(LaserScan, agent.scan_topic, 10)
            agent.odom_pub = self.create_publisher(Odometry, agent.odom_topic, 10)
            agent.drive_sub = self.create_subscription(
                AckermannDriveStamped,
                agent.drive_topic,
                functools.partial(self._drive_callback, agent_idx=agent.index),
                10)

        self.ego_reset_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.ego_reset_callback,
            10)

        if self.get_parameter('kb_teleop').value:
            self.teleop_sub = self.create_subscription(
                Twist,
                '/cmd_vel',
                self.teleop_callback,
                10)

    def _drive_callback(self, drive_msg, agent_idx):
        """Callback de drive genérico, uno por agente no-estático (via functools.partial)."""
        agent = self.agents[agent_idx]
        agent.requested_speed = drive_msg.drive.speed
        agent.steer = drive_msg.drive.steering_angle

    def ego_reset_callback(self, pose_msg):
        """Reposiciona el agente 0 (ego); el resto de agentes conserva su pose actual."""
        rx = pose_msg.pose.pose.position.x
        ry = pose_msg.pose.pose.position.y
        rqx = pose_msg.pose.pose.orientation.x
        rqy = pose_msg.pose.pose.orientation.y
        rqz = pose_msg.pose.pose.orientation.z
        rqw = pose_msg.pose.pose.orientation.w
        _, _, rtheta = euler.quat2euler([rqw, rqx, rqy, rqz], axes='sxyz')

        poses = np.array([agent.pose for agent in self.agents])
        poses[0] = [rx, ry, rtheta]
        self.obs, _, self.done, _ = self.env.reset(poses)
        self._update_sim_state()

    def teleop_callback(self, twist_msg):
        """Teleoperación por teclado del agente 0 (ego)."""
        ego = self.agents[0]
        ego.requested_speed = twist_msg.linear.x
        if twist_msg.angular.z > 0.0:
            ego.steer = 0.3
        elif twist_msg.angular.z < 0.0:
            ego.steer = -0.3
        else:
            ego.steer = 0.0

    def drive_timer_callback(self):
        """
        Avanza la física un paso, con la acción actual de cada agente.
        No espera a que todos los agentes hayan publicado un comando: un
        agente cuyo controlador aún no arrancó simplemente aporta
        [0.0, 0.0] (acción neutra, no se mueve) hasta que empiece a
        publicar en su tópico de drive.
        """
        action = np.array([[a.steer, a.requested_speed] for a in self.agents])
        self.obs, _, self.done, _ = self.env.step(action)
        self._update_sim_state()

    def timer_callback(self):
        """Publica, para cada agente, su LaserScan y sus transformadas
        (base_link, ruedas, láser), además del vector de colisiones
        `/collisions` calculado por el motor físico en el último `step`."""
        ts = self.get_clock().now().to_msg()

        for agent in self.agents:
            scan = LaserScan()
            scan.header.stamp = ts
            scan.header.frame_id = agent.namespace + '/laser'
            scan.angle_min = self.angle_min
            scan.angle_max = self.angle_max
            scan.angle_increment = self.angle_inc
            scan.range_min = 0.
            scan.range_max = 30.
            scan.ranges = agent.scan
            agent.scan_pub.publish(scan)

        collisions_msg = Float32MultiArray()
        collisions_msg.data = [float(c) for c in self.obs['collisions']]
        self.collision_pub.publish(collisions_msg)

        self._publish_odom(ts)
        self._publish_transforms(ts)
        self._publish_laser_transforms(ts)
        self._publish_wheel_transforms(ts)

    def _update_sim_state(self):
        """Copia el último `obs` del motor físico al estado de cada agente
        (escaneo, pose y velocidades), tras cada `reset` o `step`."""
        for agent in self.agents:
            i = agent.index
            agent.scan = list(self.obs['scans'][i])
            agent.pose[0] = self.obs['poses_x'][i]
            agent.pose[1] = self.obs['poses_y'][i]
            agent.pose[2] = self.obs['poses_theta'][i]
            agent.speed[0] = self.obs['linear_vels_x'][i]
            agent.speed[1] = self.obs['linear_vels_y'][i]
            agent.speed[2] = self.obs['ang_vels_z'][i]

    def _publish_odom(self, ts):
        """Publica la odometría (pose + velocidades) de cada agente en su
        tópico `odom_topic`."""
        for agent in self.agents:
            odom = Odometry()
            odom.header.stamp = ts
            odom.header.frame_id = 'map'
            odom.child_frame_id = agent.namespace + '/base_link'
            odom.pose.pose.position.x = agent.pose[0]
            odom.pose.pose.position.y = agent.pose[1]
            quat = euler.euler2quat(0., 0., agent.pose[2], axes='sxyz')
            odom.pose.pose.orientation.x = quat[1]
            odom.pose.pose.orientation.y = quat[2]
            odom.pose.pose.orientation.z = quat[3]
            odom.pose.pose.orientation.w = quat[0]
            odom.twist.twist.linear.x = agent.speed[0]
            odom.twist.twist.linear.y = agent.speed[1]
            odom.twist.twist.angular.z = agent.speed[2]
            agent.odom_pub.publish(odom)

    def _publish_transforms(self, ts):
        """Publica la transformada `map -> {namespace}/base_link` de cada
        agente."""
        for agent in self.agents:
            t = Transform()
            t.translation.x = agent.pose[0]
            t.translation.y = agent.pose[1]
            t.translation.z = 0.0
            quat = euler.euler2quat(0.0, 0.0, agent.pose[2], axes='sxyz')
            t.rotation.x = quat[1]
            t.rotation.y = quat[2]
            t.rotation.z = quat[3]
            t.rotation.w = quat[0]

            tst = TransformStamped()
            tst.transform = t
            tst.header.stamp = ts
            tst.header.frame_id = 'map'
            tst.child_frame_id = agent.namespace + '/base_link'
            self.br.sendTransform(tst)

    def _publish_wheel_transforms(self, ts):
        """Publica la rotación de las ruedas delanteras de cada agente
        según su ángulo de dirección actual."""
        for agent in self.agents:
            wheel_ts = TransformStamped()
            quat = euler.euler2quat(0., 0., agent.steer, axes='sxyz')
            wheel_ts.transform.rotation.x = quat[1]
            wheel_ts.transform.rotation.y = quat[2]
            wheel_ts.transform.rotation.z = quat[3]
            wheel_ts.transform.rotation.w = quat[0]
            wheel_ts.header.stamp = ts
            wheel_ts.header.frame_id = agent.namespace + '/front_left_hinge'
            wheel_ts.child_frame_id = agent.namespace + '/front_left_wheel'
            self.br.sendTransform(wheel_ts)
            wheel_ts.header.frame_id = agent.namespace + '/front_right_hinge'
            wheel_ts.child_frame_id = agent.namespace + '/front_right_wheel'
            self.br.sendTransform(wheel_ts)

    def _publish_laser_transforms(self, ts):
        """Publica la transformada `{namespace}/base_link -> {namespace}/laser`
        de cada agente."""
        for agent in self.agents:
            scan_ts = TransformStamped()
            scan_ts.transform.translation.x = self.scan_distance_to_base_link
            scan_ts.transform.rotation.w = 1.
            scan_ts.header.stamp = ts
            scan_ts.header.frame_id = agent.namespace + '/base_link'
            scan_ts.child_frame_id = agent.namespace + '/laser'
            self.br.sendTransform(scan_ts)


def main(args=None):
    """Punto de entrada del nodo: inicializa ROS 2, hace spin y cierra limpiamente."""
    rclpy.init(args=args)
    bridge = GymBridgeMulti()
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
