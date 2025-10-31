import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    # This name has to match the robot name in the xacro file
    robotXacroName = 'tracker_robot'

    # This is the name of our package
    namePackage = 'my_bot'

    # this is the relative path to the xacro file defining the model
    modelFileRelativePath = 'description/robot.urdf.xacro'

    # this is the absolute path to the model
    pathModelFile = os.path.join(get_package_share_directory(namePackage), modelFileRelativePath)

    # get robot description from the xacro model file
    robotDescription = xacro.process_file(pathModelFile).toxml()

    # this is the launch file from the gazebo_ros package 
    gazebo_rosPackageLaunch = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    )

    # this is if you are using an empty world
    gazeboLaunch = IncludeLaunchDescription(
        gazebo_rosPackageLaunch, 
        launch_arguments={
            'gz_args': ['-r -v -v4 empty.sdf'], 
            'on_exit_shutdown': 'true'
        }.items()
    )

    # Gazebo node to spawn robot
    spawnModelNodeGazebo = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            'name', robotXacroName,
            '-topic', 'robot_description'
        ],
        output='screen',
    )

    # Robot State Publisher Node
    nodeRobotStatePublisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robotDescription,
            'use_sim_time': True,
            'publish_frequency': 30.0
        }]
    )

    # Controller Manager
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robotDescription},
            os.path.join(get_package_share_directory(namePackage), "config", "controllers.yaml"),
        ],
        output="screen",
    )

    # Joint State Broadcaster Spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Pan-Tilt Controller Spawner
    pan_tilt_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["pan_tilt_joint_trajectory_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Delay controller spawning to ensure robot is fully loaded
    delayed_joint_state_broadcaster_spawner = TimerAction(
        period=3.0,
        actions=[joint_state_broadcaster_spawner],
    )

    delayed_pan_tilt_controller_spawner = TimerAction(
        period=4.0,
        actions=[pan_tilt_controller_spawner],
    )

    # here we create the launch description object
    LaunchDescriptionObject = LaunchDescription()

    # Add all actions to launch description
    LaunchDescriptionObject.add_action(gazeboLaunch)
    LaunchDescriptionObject.add_action(nodeRobotStatePublisher)
    LaunchDescriptionObject.add_action(spawnModelNodeGazebo)
    LaunchDescriptionObject.add_action(controller_manager)
    LaunchDescriptionObject.add_action(delayed_joint_state_broadcaster_spawner)
    LaunchDescriptionObject.add_action(delayed_pan_tilt_controller_spawner)

    return LaunchDescriptionObject