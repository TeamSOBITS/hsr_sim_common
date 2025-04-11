import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import xacro


def generate_launch_description():
    description_pkg = "hsrb_description"
    rviz_config = os.path.join(get_package_share_directory(
        "hsr_sim_common"), "rviz", "hsr.rviz")
    robot_description = os.path.join(get_package_share_directory(
        description_pkg), "robots", "hsrb4s.urdf.xacro")
    robot_description_config = xacro.process_file(robot_description)

    generate_pointcloud = os.path.join(
        get_package_share_directory("hsr_sim_common"), "launch", "generate_pointcloud.launch.py"
    )
    library_server = os.path.join(
        get_package_share_directory("hsr_sim_common"), "launch", "library_server.launch.py"
    )
    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[
                {"robot_description": robot_description_config.toxml()}],
            output="screen",
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(generate_pointcloud)
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(library_server)
        ),
    ])