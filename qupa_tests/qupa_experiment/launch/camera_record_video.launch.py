"""
camera_record_video.launch.py — Lanza el pipeline de cámara y graba video (.mp4)
durante el experimento, con la máscara de la arena (crop) opcional.

Pipeline:
  cameraNode -> /image -> rectify_node -> /image_rect -> video_recorder_node

video_recorder_node escucha /experiment/running (publicado por experiment_timer_node)
y graba automáticamente al llamar /experiment/start. Si apply_mask:=true, aplica
sobre cada frame la misma máscara poligonal de config_cropper.yaml (arena
recortada); si es false (default), graba la imagen completa sin recortar.

Requiere que el experiment_timer_node esté corriendo (timer.launch.py).

Uso:
  ros2 launch qupa_experiment camera_record_video.launch.py
  ros2 launch qupa_experiment camera_record_video.launch.py apply_mask:=true
  ros2 launch qupa_experiment camera_record_video.launch.py video_dir:=/home/coral/videos

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
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    cam_pkg     = get_package_share_directory('camera_package')
    cropper_pkg = get_package_share_directory('cropper_package')

    calibration_url = 'file://' + os.path.join(
        cam_pkg, 'config', 'camera_calibration_1280x720.yaml'
    )
    cropper_config = os.path.join(cropper_pkg, 'config', 'config_cropper.yaml')

    video_dir = LaunchConfiguration('video_dir')
    fps = LaunchConfiguration('fps')
    apply_mask = LaunchConfiguration('apply_mask')

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

    recorder_node = Node(
        package='qupa_experiment',
        executable='video_recorder',
        name='video_recorder_node',
        output='screen',
        parameters=[{
            'video_dir': video_dir,
            'image_topic': 'image_rect',
            'fps': fps,
            'apply_mask': ParameterValue(apply_mask, value_type=bool),
            'mask_config_path': cropper_config,
        }],
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'video_dir',
            default_value='~/videos',
            description='Directorio donde se guardan los videos del experimento',
        ),
        DeclareLaunchArgument(
            'fps',
            default_value='30.0',
            description='Cuadros por segundo del video de salida',
        ),
        DeclareLaunchArgument(
            'apply_mask',
            default_value='false',
            description='true = grabar solo el área de la arena (máscara de '
                        'config_cropper.yaml); false = grabar la imagen completa',
        ),

        camera_node,
        rectify_node,
        recorder_node,

    ])
