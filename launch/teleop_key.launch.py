import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pub_base_twist_topic_name = LaunchConfiguration('pub_base_twist_topic_name', default='/hsrb/command_velocity')
    pub_head_trajectory_topic_name = LaunchConfiguration('pub_arm_trajectory_topic_name', default='/hsrb/head_trajectory_controller/command')
    pub_arm_trajectory_topic_name = LaunchConfiguration('pub_arm_trajectory_topic_name', default='/hsrb/arm_trajectory_controller/command')
    pub_gripper_trajectory_topic_name = LaunchConfiguration('pub_gripper_trajectory_topic_name', default='/hsrb/gripper_controller/command')

    teleop_key_node = Node(
        package='hsr_sim_common',
        executable='hsr_sim_teleop_key',
        name='hsr_sim_teleop_key',
        output='screen',
        prefix='xterm -font r16 -fg floralwhite -bg darkslateblue -e',
        parameters=[
            {'pub_base_twist_topic_name': pub_base_twist_topic_name},
            {'pub_head_trajectory_topic_name': pub_head_trajectory_topic_name},
            {'pub_arm_trajectory_topic_name': pub_arm_trajectory_topic_name},
            {'pub_gripper_trajectory_topic_name': pub_gripper_trajectory_topic_name}
        ]
    )

    return LaunchDescription([
        teleop_key_node
    ])
