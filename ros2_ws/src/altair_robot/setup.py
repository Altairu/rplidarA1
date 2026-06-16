import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'altair_robot'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'html'),
            glob(os.path.join('html', '*.html'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='altair',
    maintainer_email='Altairu@github.comu',
    description='差動二輪ロボット制御パッケージ (MDD CAN + WebSocket)',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'mdd_can_node = altair_robot.mdd_can_node:main',
            'websocket_bridge_node = altair_robot.websocket_bridge_node:main',
            'map_marker_node = altair_robot.map_marker_node:main',
            'spresense_imu_node = altair_robot.spresense_imu_node:main',
            'experiment_logger_node = altair_robot.experiment_logger_node:main',
        ],
    },
)
