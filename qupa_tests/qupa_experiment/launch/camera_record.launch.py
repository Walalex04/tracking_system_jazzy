"""
camera_record.launch.py — Lanza la cámara y graba en rosbag durante el experimento.

Arranca cameraNode (camera_package) y camera_recorder_node.
La grabación empieza y para automáticamente al recibir /experiment/running,
que es publicado por experiment_timer_node cuando se llama el servicio
/experiment/start o /experiment/stop.

Requiere que el experiment_timer_node esté corriendo (timer.launch.py).

Uso:
  ros2 launch qupa_experiment camera_record.launch.py
  ros2 launch qupa_experiment camera_record.launch.py bag_dir:=/home/coral/bags

Para disparar la grabación:
  ros2 service call /experiment/start std_srvs/srv/Trigger
  ros2 service call /experiment/stop  std_srvs/srv/Trigger
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    cam_pkg = get_package_share_directory('camera_package')
    calibration_url = 'file://' + os.path.join(
        cam_pkg, 'config', 'camera_calibration_1280x720.yaml'
    )

    bag_dir = LaunchConfiguration('bag_dir')

    camera_node = Node(
        package='camera_package',
        executable='cameraNode',
        name='camera_node',
        output='screen',
        parameters=[{
            'camera_info_url': calibration_url,
        }],
    )

    recorder_node = Node(
        package='qupa_experiment',
        executable='camera_recorder',
        name='camera_recorder_node',
        output='screen',
        parameters=[{
            'bag_dir': bag_dir,
        }],
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'bag_dir',
            default_value='~/bags',
            description='Directorio donde se guardan los rosbags del experimento',
        ),

        camera_node,
        recorder_node,

    ])
