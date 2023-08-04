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
    engine_file_launch_arg = DeclareLaunchArgument(
        "engine_path", default_value=TextSubstitution(text='/home/thu/Downloads/PointPillarNet/'
                                                           'checkpoint_epoch_80_test.engine')
    )

    return LaunchDescription([
        pcd_path_launch_arg,
        rate_launch_arg,
        engine_file_launch_arg,
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
        ),
        Node(
            package='pp_infer',
            executable='pp_infer',
            name='pcd_infer',
            parameters=[{
                'nms_iou_thresh': 0.01,
                'pre_nms_top_n': 4096,
                'class_names': ['Vehicle,', 'Pedestrian', 'Cyclist'],
                'model_path': '',
                'engine_path': LaunchConfiguration('engine_path'),
                'data_type': 'fp32',
                'intensity_scale': 255.0,
            }]
        ),
        Node(
            package='point_cloud_infer',
            executable='bounding_box_client',
            name='bbox_sender',
            output='screen',
            emulate_tty=True

        )
    ])
