from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, AppendEnvironmentVariable, IncludeLaunchDescription, GroupAction

import os
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name = 'stembot_gazebo'
    package_share = FindPackageShare(package=package_name).find(package_name)

    use_ekf_odom = LaunchConfiguration('use_ekf_odom')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_name = LaunchConfiguration('world_name')
    rviz_config_path = LaunchConfiguration('rviz_config_path')

    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    roll = LaunchConfiguration('roll')
    pitch = LaunchConfiguration('pitch')
    yaw = LaunchConfiguration('yaw')

    declare_use_ekf_odom = DeclareLaunchArgument(
        name='use_ekf_odom',
        default_value='false',
        description='Whether to start ekf filter for odometry'
    )

    declare_world_name = DeclareLaunchArgument(
        name='world_name',
        default_value='empty.sdf',
        description='Name of the demo world to load'
    )

    declare_rviz_config_path = DeclareLaunchArgument(
        name='rviz_config_path',
        default_value=os.path.join(package_share, 'config', 'rviz_config.rviz'),
        description='Location of RViz config file'
    )

    declare_use_rviz = DeclareLaunchArgument(
        name='use_rviz',
        default_value='false',
        description='Whether to start RViz'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_x = DeclareLaunchArgument(
        name='x',
        default_value='2.0',
        description='x component of initial position, meters'
    )

    declare_y = DeclareLaunchArgument(
        name='y',
        default_value='2.0',
        description='y component of initial position, meters'
    )

    declare_z = DeclareLaunchArgument(
        name='z',
        default_value='0.05',
        description='z component of initial position, meters'
    )
        

    declare_roll = DeclareLaunchArgument(
        name='roll',
        default_value='0.0',
        description='roll angle of initial orientation, radians'
    )

    declare_pitch = DeclareLaunchArgument(
        name='pitch',
        default_value='0.0',
        description='pitch angle of initial orientation, radians'
    )

    declare_yaw = DeclareLaunchArgument(
        name='yaw',
        default_value='0.0',
        description='yaw angle of initial orientation, radians'
    )

    package_tb3_description = FindPackageShare(package='tb3_description').find('tb3_description')
    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_tb3_description, 'launch', 'description.launch.py')),
            launch_arguments={
                'use_rviz': use_rviz,
                'rviz_config_path': rviz_config_path,
                'use_jsp_gui': 'false',
                'use_sim_time': use_sim_time
        }.items()
    )

    package_ros_gz_sim = FindPackageShare(package='ros_gz_sim').find('ros_gz_sim')
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={
                'gz_args': ['-r ', world_name],
                'on_exit_shutdown': 'true',
                'use_sim_time': use_sim_time
        }.items()
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-allow_renaming', 'true',
            '-x', x,
            '-y', y,
            '-z', z,
            '-R', roll,
            '-P', pitch,
            '-Y', yaw
        ],
        output='screen'
    )

    bridge_param_file = os.path.join(package_share, 'config', 'bridge.yaml')

    main_bridge = Node(
        condition=UnlessCondition(use_ekf_odom),
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_param_file}],
        remappings=[
            ('/robot/odom', '/odom'),
            ('/robot/tf', '/tf')
        ],
        output='screen'
    )

    package_stembot_localization = FindPackageShare(package='stembot_localization').find('stembot_localization')
    ekf_gazebo_group = GroupAction(
        actions=[
            Node(
                condition=IfCondition(use_ekf_odom),
                package='ros_gz_bridge',
                executable='parameter_bridge',
                parameters=[{'config_file': bridge_param_file}],
                output='screen'
            ),

            Node(
                package='imu_filter_madgwick',
                executable='imu_filter_madgwick_node',
                parameters=[{
                    'gain': 0.01,
                    'use_sim_time': use_sim_time,
                    "use_mag": False,
                    "publish_tf": False,
                    "world_frame": "enu",
                    "fixed_frame": "odom"
                }],
                remappings=[
                    ('/imu/data_raw', '/imu'),
                ]
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(package_stembot_localization, 'launch', 'ekf_gazebo.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time
                }.items(),
            )
        ],
        condition=IfCondition(use_ekf_odom)
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_ekf_odom)
    ld.add_action(declare_use_rviz)
    ld.add_action(declare_rviz_config_path)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_use_sim_time)

    ld.add_action(declare_world_name)
    ld.add_action(declare_x)
    ld.add_action(declare_y)
    ld.add_action(declare_z)
    ld.add_action(declare_roll)
    ld.add_action(declare_pitch)
    ld.add_action(declare_yaw)

    ld.add_action(robot_description)
    ld.add_action(gz_sim)
    ld.add_action(gz_spawn_entity)
    ld.add_action(main_bridge)
    ld.add_action(ekf_gazebo_group)

    return ld
