#!/usr/bin/env python3
"""
Launch multi-agente del simulador F1TENTH (Parte 2).
=======================================================

Autor: George Gabriel Giler Vega

Levanta el bridge multi-agente (gym_bridge_multi), RViz, el servidor de
mapas, y por cada agente definido en `agents_config`: un
`robot_state_publisher` (con el xacro compartido `racecar.xacro`,
parametrizado por nombre/color). No lanza ningún `ftg_control`: cada
controlador se inicia a mano, por separado, para poder verificar el
comportamiento del bridge antes de que cualquier auto se mueva (ver
README de este paquete para los comandos de cada agente).

Uso:
  ros2 launch f1tenth_multiagent_race multiagent_bridge_launch.py \\
      agents_config:=<ruta a agents_testX_pista.yaml> \\
      map_path:=<ruta al mapa sin extensión>
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def _agent_defaults(i, agent):
    """Deriva namespace/tópicos/color por defecto a partir de la config humana del agente."""
    namespace = agent['namespace']
    return {
        'namespace': namespace,
        'scan_topic': agent.get('scan_topic', f'{namespace}/scan'),
        'odom_topic': agent.get('odom_topic', f'{namespace}/odom'),
        'drive_topic': agent.get('drive_topic', f'{namespace}/drive'),
        'color': agent.get('color', '0.3 0.57 1. 1.'),
        'sx': float(agent.get('sx', 0.0)),
        'sy': float(agent.get('sy', 0.0)),
        'stheta': float(agent.get('stheta', 0.0)),
    }


def _launch_setup(context, *args, **kwargs):
    """Arma la lista de acciones del launch: lee `sim.yaml` y el
    `agents_config` indicado, aplana la configuración de cada agente a
    parámetros indexados para `gym_bridge_multi`, y agrega RViz, el
    servidor de mapas, y un `robot_state_publisher` por agente."""
    pkg_share = get_package_share_directory('f1tenth_multiagent_race')

    sim_config_path = os.path.join(pkg_share, 'config', 'sim.yaml')
    with open(sim_config_path) as f:
        sim_params = yaml.safe_load(f)['bridge']['ros__parameters']

    agents_config_path = LaunchConfiguration('agents_config').perform(context)
    with open(agents_config_path) as f:
        agents_raw = yaml.safe_load(f)['agents']

    agents = [_agent_defaults(i, a) for i, a in enumerate(agents_raw)]

    map_path = LaunchConfiguration('map_path').perform(context)

    bridge_params = dict(sim_params)
    bridge_params['map_path'] = map_path
    bridge_params['num_agents'] = len(agents)
    for i, agent in enumerate(agents):
        bridge_params[f'agent_{i}_namespace'] = agent['namespace']
        bridge_params[f'agent_{i}_scan_topic'] = agent['scan_topic']
        bridge_params[f'agent_{i}_odom_topic'] = agent['odom_topic']
        bridge_params[f'agent_{i}_drive_topic'] = agent['drive_topic']
        bridge_params[f'agent_{i}_sx'] = agent['sx']
        bridge_params[f'agent_{i}_sy'] = agent['sy']
        bridge_params[f'agent_{i}_stheta'] = agent['stheta']

    actions = []

    actions.append(Node(
        package='f1tenth_multiagent_race',
        executable='gym_bridge_multi',
        name='bridge',
        parameters=[bridge_params],
    ))

    actions.append(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(pkg_share, 'launch', 'multiagent.rviz')],
    ))

    actions.append(Node(
        package='nav2_map_server',
        executable='map_server',
        parameters=[{'yaml_filename': map_path + '.yaml'},
                    {'topic': 'map'},
                    {'frame_id': 'map'},
                    {'use_sim_time': True}],
    ))

    actions.append(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['map_server']}],
    ))

    racecar_xacro = os.path.join(pkg_share, 'launch', 'racecar.xacro')

    for agent in agents:
        ns = agent['namespace']
        actions.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name=f'{ns}_robot_state_publisher',
            parameters=[{'robot_description': Command([
                'xacro ', racecar_xacro,
                ' car_name:=', ns,
                " body_color_rgba:='", agent['color'], "'",
            ])}],
            remappings=[('/robot_description', f'{ns}_robot_description')],
        ))

    return actions


def generate_launch_description():
    """Declara los argumentos de lanzamiento (`agents_config`, `map_path`)
    y delega el armado de nodos a `_launch_setup`."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'agents_config',
            description='Ruta al YAML de agentes (config/agents_testX_pista.yaml).',
        ),
        DeclareLaunchArgument(
            'map_path',
            description='Ruta al mapa (sin extensión .png/.yaml).',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
