"""
Bottom Aruco Detector launch file.

Lorenzo Bianchi <lnz.bnc@gmail.com>
Roberto Masocco <robmasocco@gmail.com>
Intelligent Systems Lab <isl.torvergata@gmail.com>

August 7, 2023
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Builds a LaunchDescription for the bottom Detector"""
    ld = LaunchDescription()

    # Build config file path
    config_file = os.path.join(
        get_package_share_directory('aruco_detector'),
        'config',
        'aruco_detector.yaml'
    )

    # Create node launch description
    node = Node(
        package='aruco_detector',
        executable='aruco_detector',
        exec_name='aruco_detector_app',
        # namespace='seppia',
        shell=True,
        emulate_tty=True,
        output='both',
        log_cmd=True,
        parameters=[config_file]
    )

    ld.add_action(node)

    return ld
