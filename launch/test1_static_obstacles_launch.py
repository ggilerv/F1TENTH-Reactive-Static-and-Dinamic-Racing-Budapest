#!/usr/bin/env python3
"""
Launch de la Prueba 1 (obstáculos estáticos) con RViz.
=========================================================

Autor: George Gabriel Giler Vega

Réplica del `gym_bridge_launch.py` original de `f1tenth_gym_ros` (RViz +
servidor de mapas + modelo del auto), pero usando `config/sim_test1_budapest.yaml`
(que apunta a `maps/Budapest_map_obstaculos`) en vez del `sim.yaml` del
paquete base. Usa el bridge ORIGINAL de un solo agente (`f1tenth_gym_ros`,
sin ningún cambio) — este paquete solo aporta el launch y el mapa editado.

Uso:
  ros2 launch f1tenth_multiagent_race test1_static_obstacles_launch.py
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    pkg_share = get_package_share_directory('f1tenth_multiagent_race')
    config = os.path.join(pkg_share, 'config', 'sim_test1_budapest.yaml')
    config_dict = yaml.safe_load(open(config, 'r'))

    bridge_node = Node(
        package='f1tenth_gym_ros',
        executable='gym_bridge',
        name='bridge',
        parameters=[config],
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(pkg_share, 'launch', 'gym_bridge.rviz')],
    )
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        parameters=[{'yaml_filename': config_dict['bridge']['ros__parameters']['map_path'] + '.yaml'},
                    {'topic': 'map'},
                    {'frame_id': 'map'},
                    {'use_sim_time': True}],
    )
    nav_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['map_server']}],
    )
    ego_robot_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ego_robot_state_publisher',
        parameters=[{'robot_description': Command([
            'xacro ', os.path.join(pkg_share, 'launch', 'racecar.xacro'),
            ' car_name:=ego_racecar',
        ])}],
        remappings=[('/robot_description', 'ego_robot_description')],
    )

    ld.add_action(rviz_node)
    ld.add_action(bridge_node)
    ld.add_action(nav_lifecycle_node)
    ld.add_action(map_server_node)
    ld.add_action(ego_robot_publisher)

    return ld
