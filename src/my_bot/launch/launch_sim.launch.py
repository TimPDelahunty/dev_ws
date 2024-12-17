import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Include the robot_state_publisher (rsp) launch file
    rsp_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('my_bot'),
                'launch',
                'rsp.launch.py'
            )
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include the Gazebo simulation launch file
    gazebo_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ]),
        launch_arguments={'gz_args': 'empty.sdf'}.items()
    )

    # Spawn the robot in Gazebo using ros_gz_sim's gz_spawn_model.launch.py
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_spawn_model.launch.py'
            )
        ]),
        launch_arguments={
            'world': 'empty',
            'file': os.path.join(
                get_package_share_directory('my_bot'),
                'description',
                'robot.urdf.xacro'
            ),
            'name': 'my_robot',
            'x': '0.0',
            'y': '0.0',
            'z': '1.0'
        }.items()
    )

    # Combine everything into a LaunchDescription
    return LaunchDescription([
        rsp_launch_file,
        gazebo_launch_file,
        spawn_robot
    ])
