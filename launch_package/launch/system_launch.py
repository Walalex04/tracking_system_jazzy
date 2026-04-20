

from launch import LaunchDescription
from launch_ros.actions import Node
import yaml
import os 
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():

    system_package = get_package_share_directory("launch_package")
    config_system_path = os.path.join(system_package, "config", "system_configuration.yaml")

    #reading the yaml file configuration
    with open(config_system_path, 'r') as f:
        config = yaml.safe_load(f)


    camera_width = config['camera_width']
    camera_height = config['camera_height']

    print(f"The configuration of the camera is {camera_width} and {camera_height}")

    camera_package = get_package_share_directory("camera_package")  

    
    camera_info = os.path.join(
        camera_package,
        'config',
        f'camera_calibration_{camera_width}x{camera_height}.yaml'
    )

    print(f'camera_calibration {camera_info}')

    return LaunchDescription([
        Node(
            package="camera_package",
            executable="cameraNode",
            name="cameraNode",
            output="screen",
            parameters=[{
                'camera_info_url': f'file://{camera_info}'
            }]
        ), 
        Node(
            package='image_proc',
            executable='rectify_node',
            name='rectify',
            output='screen'
        ),
        Node(
            package="cropper_package",
            executable="cropperNode",
            name="cropperNode",
            output="screen"
        ),
        Node(
            package="tracker_package",
            executable="trackerNode",
            name="traclerNode",
            output="screen"
        )
    ])
