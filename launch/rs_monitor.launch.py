from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

from launch import LaunchDescription  # type: ignore
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

NAMESPACE = "robosense"
PACKAGE_NAME = "rs_monitor"
MONITOR_NODE_NAME = "rs_monitor"


def generate_launch_description() -> LaunchDescription:
    # return LaunchDescription(
    #     [
    #         ComposableNodeContainer(
    #             name=f"{MONITOR_NODE_NAME}_container",
    #             namespace=NAMESPACE,
    #             package="rclcpp_components",
    #             executable="component_container_mt",
    #             composable_node_descriptions=[
    #                 ComposableNode(
    #                     package=PACKAGE_NAME,
    #                     plugin="robosense::rs_monitor::MonitorManager",
    #                     namespace=NAMESPACE,
    #                     name=MONITOR_NODE_NAME,
    #                     parameters=[
    #                         {"package_name": PACKAGE_NAME},
    #                     ],
    #                 )
    #             ],
    #             output="screen",
    #         )
    #     ]
    # )

    config_file_path = LaunchConfiguration("config_file", default="")
    config_file_path_argument = DeclareLaunchArgument(
        "config_file",
        default_value="",
        description="The path to the config file",
    )

    return LaunchDescription(
        [
            config_file_path_argument,
            Node(
                package=PACKAGE_NAME,
                namespace=NAMESPACE,
                executable=MONITOR_NODE_NAME,
                name=MONITOR_NODE_NAME,
                parameters=[
                    {"package_name": PACKAGE_NAME},
                    {"config_file": config_file_path},
                ],
                output="screen",
            ),
        ]
    )
