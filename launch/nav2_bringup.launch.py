"""Nav2 bringup launch for GO2 navigation_rbnx.

Launches: lifecycle_manager, planner_server, controller_server,
bt_navigator, behavior_server, smoother_server, velocity_smoother.

Does NOT launch map_server or AMCL — localization and maps are
provided by mapping_rbnx via ROS2 topics.
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_params = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join("/ws", "config", "nav2_params.yaml"),
    )
    declare_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="false"
    )

    lifecycle_nodes = [
        "planner_server",
        "controller_server",
        "bt_navigator",
        "behavior_server",
        "smoother_server",
        "velocity_smoother",
    ]

    nav2_nodes = []
    for node_name in lifecycle_nodes:
        nav2_nodes.append(
            Node(
                package="nav2_" + node_name.replace("_server", "")
                if node_name
                not in (
                    "planner_server",
                    "controller_server",
                    "smoother_server",
                    "behavior_server",
                    "velocity_smoother",
                    "bt_navigator",
                )
                else {
                    "planner_server": "nav2_planner",
                    "controller_server": "nav2_controller",
                    "smoother_server": "nav2_smoother",
                    "behavior_server": "nav2_behaviors",
                    "velocity_smoother": "nav2_velocity_smoother",
                    "bt_navigator": "nav2_bt_navigator",
                }[node_name],
                executable=node_name,
                name=node_name,
                output="screen",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
            )
        )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": True},
            {"node_names": lifecycle_nodes},
        ],
    )

    return LaunchDescription(
        [declare_params, declare_sim_time] + nav2_nodes + [lifecycle_manager]
    )
