import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
import xacro


def generate_launch_description():


    # This name has to match the robot name n the xacro file
    robotXacroName = 'tracker_robot'

    # This is the name of our package, at the same time this is the name of the 
    # folder that will be used to define paths
    namePackage = 'my_bot'

    # this is the relative path to the xacro file defining the model
    modelFileRelativePath = 'description/robot.urdf.xacro'

    #this is the absolute path to the model
    pathModelFile = os.path.join(get_package_share_directory(namePackage), modelFileRelativePath)

    # get robot description from the xacro model file
    robotDescrption = xacro.process_file(pathModelFile).toxml()


    #this is the launch file from the gazebo_ros package 
    gazebo_rosPackageLaunch = PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'),'launch','gz_sim.launch.py'))

    #this is the launch description

    # this is if you are using yourown world model
    #gazeboLaunch=IncludeLaunchDescription(gazebo_rosPackageLaunch, launch_arguments={''})    not finished

    #this is if you are using an empty world
    gazeboLaunch  = IncludeLaunchDescription(gazebo_rosPackageLaunch, launch_arguments={'gz_args': ['-r -v -v4 empty.sdf'], 'on_exit_shutdown': 'true'}.items())


    # Gazebo node
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
        parameters=[{'robot_description' : robotDescrption,
                     'use_sim_time': True}]
    )

    #bridge stuff need yaml file
    """
    # This is very important so we can control the robot from ros2
    bridge_params = os.path.join(
        get_package_share_directory(namePackage),
        'parameters',
        'bridge_parameters.yaml'
    )

    start_gazebo_ros_brdige_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
        '--ros-args',
        '-p',
        f'config_file:={bridge_params}'
        ],
        output='screen'
    )"""


    #here we create an empty launch description object
    LaunchDescriptionObject = LaunchDescription()

    # we add gazeboLaunch
    LaunchDescriptionObject.add_action(gazeboLaunch)

    #we add the two nodes
    LaunchDescriptionObject.add_action(spawnModelNodeGazebo)
    LaunchDescriptionObject.add_action(nodeRobotStatePublisher)
    #LaunchDescriptionObject.add_action(start_gazebo_ros_bridge_cmd)

    return LaunchDescriptionObject