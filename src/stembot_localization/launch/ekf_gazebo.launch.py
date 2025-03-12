from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, AppendEnvironmentVariable, IncludeLaunchDescription

import os
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name = 'stembot_localization'
    package_share = FindPackageShare(package=package_name).find(package_name)

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    ekf_param_file = os.path.join(package_share, 'config', 'ekf.yaml')
    ekf_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        parameters=[
            ekf_param_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/odometry/filtered', '/odom'),
        ]
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time)
    ld.add_action(ekf_localization)

    return ld