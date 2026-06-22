import os
from setuptools import find_packages, setup

package_name = 'swarm_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), [
            'launch/telemetry_launch.py',   # real robot mode
            'launch/sim_launch.py',          # simulation mode
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yekka',
    maintainer_email='yekka@todo.todo',
    description='IoT Level-4 Autonomous Spatial Mapping Rover',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'rover_bridge_node = swarm_bot.rover_bridge_node:main',
            'mapper_node       = swarm_bot.mapper_node:main',
            'room_simulator    = swarm_bot.room_simulator:main',
        ],
    },
)