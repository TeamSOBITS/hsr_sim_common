import launch
import launch_ros.actions
import launch_ros.descriptions
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    namespace_param_name = "namespace"
    namespace = LaunchConfiguration(namespace_param_name)
    namespace_launch_arg = DeclareLaunchArgument(namespace_param_name, default_value='camera')

    container = launch_ros.actions.ComposableNodeContainer(
        name='container',
        namespace=namespace,
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # Create XYZRGB point cloud
            launch_ros.descriptions.ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::PointCloudXyzrgbNode',
                name='points_xyzrgb',
                namespace=namespace,
                parameters=[{'queue_size': 10}],
                remappings=[('rgb/image_rect_color', '/hsrb/head_rgbd_sensor/rgb/image_raw'),
                            ('rgb/camera_info', '/hsrb/head_rgbd_sensor/rgb/camera_info'),
                            ('depth_registered/image_rect', '/hsrb/head_rgbd_sensor/depth_registered/image_raw'),
                            ('points', '/hsrb/head_rgbd_sensor/depth_registered/points')],
            ),
        ],
        output='screen',
    )

    return launch.LaunchDescription([
        namespace_launch_arg,
        container,
    ])