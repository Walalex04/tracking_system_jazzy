"""
camera_record_filtered.launch.py — Lanza el pipeline de cámara y graba image_cropped durante el experimento.

Pipeline:
  cameraNode -> /image -> rectify_node -> /image_rect -> cropperNode -> image_cropped

camera_recorder_node escucha /experiment/running (publicado por experiment_timer_node)
y graba image_cropped automáticamente al llamar /experiment/start.

Requiere que el experiment_timer_node esté corriendo (timer.launch.py).

Uso:
  ros2 launch qupa_experiment camera_record_filtered.launch.py
  ros2 launch qupa_experiment camera_record_filtered.launch.py bag_dir:=/home/coral/bags

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

    cam_pkg     = get_package_share_directory('camera_package')
    cropper_pkg = get_package_share_directory('cropper_package')

    calibration_url = 'file://' + os.path.join(
        cam_pkg, 'config', 'camera_calibration_1280x720.yaml'
    )
    cropper_config = os.path.join(cropper_pkg, 'config', 'config_cropper.yaml')

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

    rectify_node = Node(
        package='image_proc',
        executable='rectify_node',
        name='rectify',
        output='screen',
    )

    cropper_node = Node(
        package='cropper_package',
        executable='cropper_node',
        name='cropperNode',
        output='screen',
        parameters=[cropper_config],
    )

    recorder_node = Node(
        package='qupa_experiment',
        executable='camera_recorder',
        name='camera_recorder_node',
        output='screen',
        parameters=[{
            'bag_dir': bag_dir,
            'topics': ['image_cropped'],
        }],
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'bag_dir',
            default_value='~/bags',
            description='Directorio donde se guardan los rosbags del experimento',
        ),

        camera_node,
        rectify_node,
        cropper_node,
        recorder_node,

    ])
