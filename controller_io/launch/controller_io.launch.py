import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    node_params = os.path.join(get_package_share_directory("controller_io"),
                               "config", "serial_config.yaml")
    serial_node = ComposableNode(package="custom_serial_driver",
                                 plugin="custom_serial::SerialDriverNode",
                                 name="custom_serial_node",
                                 parameters=[node_params],
                                 extra_arguments=[{
                                     "user_intra_process_comms":
                                     True
                                 }])
    controller_io_node = ComposableNode(
        package="controller_io",
        plugin="armor_auto_aim::ControllerIONode",
        name="controller_io_node",
        extra_arguments=[{
            "user_intra_process_comms": True
        }])
    serial_container = ComposableNodeContainer(
        name="serial_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            serial_node,
            controller_io_node,
        ],
        output="both",
        emulate_tty=True)
    return LaunchDescription([serial_container])
