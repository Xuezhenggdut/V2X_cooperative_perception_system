from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
# from launch.actions import IncludeLaunchDescription
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
# import launch.substitutions

# import os


def generate_launch_description():
    pcd_path_launch_arg = DeclareLaunchArgument(
        "pcd_path", default_value=TextSubstitution(text='/home/thu/Downloads/2021_08_23_21_47_19/225')
    )
    rate_launch_arg = DeclareLaunchArgument(
        "rate", default_value=TextSubstitution(text='5')
    )

    return LaunchDescription([
        pcd_path_launch_arg,
        rate_launch_arg,
        Node(
            package='point_cloud_infer',
            executable='publisher',
            name='pcd_publisher',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'pcd_path': LaunchConfiguration('pcd_path'),
                'rate': LaunchConfiguration('rate')
            }]
        ),
        Node(
            package='point_cloud_infer',
            executable='visualization_subscriber',
            name='plot_pcd',
            output='screen',
            emulate_tty=True
        )
    ])
