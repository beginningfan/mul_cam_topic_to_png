from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mul_cam_to_img',
            executable='sub_cal',
            output='screen',
            parameters=[
                {
                    "img_filepath": "/home/fanliangliang/Desktop/test/",
                    "topic_list":["/gmsl_driver/front0", "/gmsl_driver/front1"],
                }
            ]
        ),
    ])