import os
import platform
import re
import shutil
import subprocess
from datetime import datetime
from typing import List

import yaml
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription  # type: ignore
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration

NAMESPACE = "robosense"
PACKAGE_NAME = "rs_monitor"
EXECUTABLE_NAME = f"{PACKAGE_NAME}"


def _host_based_node_name() -> str:
    """Node name unique per host. Safe for ROS 2 (alphanumeric + underscore)"""
    host = platform.node() or "unknown"
    safe = re.sub(r"[^a-zA-Z0-9_]", "_", host).strip("_") or "unknown"
    return f"{PACKAGE_NAME}_{safe}"


NODE_NAME = _host_based_node_name()


def _to_bool(s: str) -> bool:
    return s.lower() in ("1", "true", "yes", "on")


def setup_record(context: LaunchContext) -> List[ExecuteProcess]:
    enable = LaunchConfiguration("enable_self_recording").perform(context)
    if not _to_bool(enable):
        return []

    cfg_path = LaunchConfiguration("config_file").perform(context)
    self_recording_dir = LaunchConfiguration("self_recording_dir").perform(context)
    if not os.path.exists(self_recording_dir):
        os.makedirs(self_recording_dir, exist_ok=True)
    out_dir = os.path.join(
        self_recording_dir,
        f"rs_monitor_record_{datetime.now().strftime('%Y%m%d_%H%M%S')}",
    )

    topics = set()

    if cfg_path and os.path.exists(cfg_path):
        try:
            with open(cfg_path, "r") as f:
                monitor_config = yaml.safe_load(f) or {}
            for _, sub in monitor_config.items():
                if isinstance(sub, dict):
                    t = sub.get("publish_topic_name")
                    if t:
                        topics.add(t)
        except Exception:
            pass

    if topics:
        cmd = [
            "ros2",
            "bag",
            "record",
            *sorted(topics),
            "-o",
            out_dir,
        ]
        return [ExecuteProcess(name="monitor_recorder", cmd=cmd, output="screen")]
    else:
        return []


def generate_launch_description() -> LaunchDescription:
    config_file_path_argument = DeclareLaunchArgument(
        "config_file",
        default_value=os.path.join(
            get_package_share_directory(PACKAGE_NAME), "config", "config.yaml"
        ),
        description="The path to the config file",
    )
    node_name_argument = DeclareLaunchArgument(
        "node_name",
        default_value=NODE_NAME,
        description="Node name (default: rs_monitor_<hostname>). Override for multiple instances on same host",
    )
    enable_self_recording_argument = DeclareLaunchArgument(
        "enable_self_recording",
        default_value="false",
        description="Whether to launch ros bag record to save the results output by rs_monitor",
    )
    self_recording_dir_argument = DeclareLaunchArgument(
        "self_recording_dir",
        default_value=os.getcwd(),
        description="The directory to save the self recording",
    )

    actions = [
        config_file_path_argument,
        node_name_argument,
        enable_self_recording_argument,
        self_recording_dir_argument,
        Node(
            package=PACKAGE_NAME,
            namespace=NAMESPACE,
            executable=EXECUTABLE_NAME,
            name=LaunchConfiguration("node_name"),
            parameters=[
                {"package_name": PACKAGE_NAME},
                {"config_file": LaunchConfiguration("config_file")},
            ],
            output="screen",
            env={
                **os.environ,
                "RS_NODE_NAME": LaunchConfiguration("node_name"),
                "RS_PACKAGE_NAME": PACKAGE_NAME,
                "RS_HARDWARE_ID": f"{platform.machine()}@{platform.node()}",
            },
        ),
    ]

    # runtime-resolve and add recorder if enabled
    actions.append(OpaqueFunction(function=setup_record))

    return LaunchDescription(actions)
