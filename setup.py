import os
from glob import glob
from setuptools import setup

package_name = 'f1tenth_multiagent_race'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.xacro')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.png')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='George Giler',
    maintainer_email='gg.gilerv@gmail.com',
    description='Bridge multi-agente para el simulador F1TENTH: evasión de obstáculos estáticos y dinámicos.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gym_bridge_multi = f1tenth_multiagent_race.gym_bridge_multi:main',
            'ftg_control_obstaculos = f1tenth_multiagent_race.ftg_node_obstaculos:main',
            'lap_timer = f1tenth_multiagent_race.lap_timer:main',
        ],
    },
)
