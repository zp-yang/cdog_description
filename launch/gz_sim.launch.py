from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.actions import ExecuteProcess
from launch.conditions import LaunchConfigurationEquals, IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# TODO: take 'world', 'robot', 'pose' etc. as launch args
ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='Use sim time'),
    DeclareLaunchArgument('log_level', default_value='error',
                          choices=['info', 'warn', 'error'],
                          description='log level'),
    DeclareLaunchArgument('bridge', default_value='true',
                          choices=['true', 'false'],
                          description='Run bridges'),
]

def generate_launch_description():
    pkg_description = Path(get_package_share_directory('cdog_description'))
    urdf_file_path = pkg_description / 'urdf'/ 'robot.xacro'
    
    # with open(urdf_file, 'r') as f:
    #     robot_desc = f.read()
    robot_desc = ParameterValue(Command(['xacro ' , str(urdf_file_path)]), value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='cdog_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'robot_description': robot_desc},
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
            [get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])]),
        launch_arguments=[('gz_args', [
             'empty.sdf', ' -v 1', ' -r'
            ])]
    )

    create_model = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'empty',
            '-name', 'cdog',
            '-z', '0.5',
            '-topic', 'robot_description', # using parameter produces exception, seems not fixed yet? https://github.com/ros2/rclcpp/issues/1691
        ],
        ros_arguments=['--log-level', LaunchConfiguration('log_level')],
    )
    
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_gz_ros_clock',
        output='screen',
        condition=IfCondition(LaunchConfiguration('bridge')),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )

    joint_state_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_gz_ros_joint_state',
        output='screen',
        condition=IfCondition(LaunchConfiguration('bridge')),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
        arguments=['/world/empty/model/cdog/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model'],
        remappings=[(
            '/world/empty/model/cdog/joint_state', 'joint_states'
            )],
    )

    return LaunchDescription(ARGUMENTS + [
        robot_state_publisher,
        gz_sim,
        create_model,
        clock_bridge,
        joint_state_bridge,
    ])