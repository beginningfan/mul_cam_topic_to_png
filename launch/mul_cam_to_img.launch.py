from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mul_cam_to_img',
            executable='sub',
            output='screen',
            parameters=[
                {
                    "img_filepath": "/home/fanliangliang/ROS2/2022-07-05/img/",
                    # "topic_list":["/image_raw", "/left_camera0", "/right_camera0"],
                    # "topic_list":["/image_raw", "/sensing/camera/traffic_light/image_raw"],
                    "topic_list":["/image_raw", "/image_raw2"],
                }
            ]
        ),
    ])