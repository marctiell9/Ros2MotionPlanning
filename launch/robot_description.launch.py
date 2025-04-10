import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    robot_name = 'panda'
    
    urdf_path = os.path.join(
    get_package_share_directory('robot_arm_control_pkg'),
    'urdf',
    'panda_ros2.urdf'
    )

    with open(urdf_path, 'r') as file:
        robot_description = file.read()

    nodeRobotStatePublisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }],
        )

    gazebo_rosPackageLaunch = PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'))

    gazeboLaunch = IncludeLaunchDescription(gazebo_rosPackageLaunch, launch_arguments={'gz_args':['-r -v -v4 empty.sdf'], 'on_exit_shutdown': 'true'}.items())

    #Gazebo Node
    spawnModelNodeGazebo = Node(
        package = 'ros_gz_sim',
        executable= 'create',
        arguments=[
            '-name', robot_name,
            '-topic', 'robot_description' 
        ],
        output='screen',
    )

    nodeRobotStatePublisher = Node(
        package = 'robot_state_publisher',
        executable= 'robot_state_publisher',
        output = 'screen',
        parameters = [{'robot_description': robot_description,
                       'use_sim_time': True}],
    )

    bridge_params = os.path.join(
        get_package_share_directory('robot_arm_control_pkg'),
        'config',
        'bridge_parameters.yaml',
    )

    start_gazebo_ros_bridge_cmd = Node(
        package = 'ros_gz_bridge',
        executable= 'parameter_bridge',
        arguments = [
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )

    config_file = PathJoinSubstitution(
        [
            FindPackageShare('robot_arm_control_pkg'),
            'config',
            'control.yaml',
        ]
    )
  
    joint_state_broadcaster_spawner = Node(
    package='controller_manager',
    executable='spawner',
    arguments=['joint_state_broadcaster'],
    parameters=[config_file],  # optional
    output='screen'
    )

    '''
    joint_effort_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['effort_controller'],
        parameters=[config_file],
        output='screen'  
    )
    '''

    '''
    joint_effort_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['effort_controller', '--stopped'],
        parameters=[config_file],
        output='screen'
    )
    '''
    
    position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['position_controller'],
        parameters=[config_file],
        output='screen'  
    )
    

    LaunchDescriptionObject = LaunchDescription()
    LaunchDescriptionObject.add_action(gazeboLaunch)
    LaunchDescriptionObject.add_action(spawnModelNodeGazebo)
    LaunchDescriptionObject.add_action(nodeRobotStatePublisher)
    LaunchDescriptionObject.add_action(joint_state_broadcaster_spawner)
    #LaunchDescriptionObject.add_action(joint_effort_controller_spawner)
    LaunchDescriptionObject.add_action(position_controller_spawner)
    LaunchDescriptionObject.add_action(start_gazebo_ros_bridge_cmd)

    return LaunchDescriptionObject