from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            parameters=[{
                'enable_infra1': False,
                'enable_infra2': False,
                'enable_gyro': False,
                'enable_accel': False,
                'pointcloud.enable': False,
                'colorizer.enable': False,
                'publish_tf': False,

                #launch resolution/fps can be configured!!!!
                'rgb_camera.color_profile': '848x480x60',
                'enable_color': True,
                'depth_module.depth_profile': '848x480x30',
                'enable_depth': True,

                #color compression level can be changed: 0-100 (100 is best quality but larger size)!!!!!
                '.camera.color.image_raw.jpeg_quality': 75,
            }],
            output='screen'
        ),
        Node(
            package='cam_package',
            executable='viewer_node.py',
            output='screen'
        )
    ])