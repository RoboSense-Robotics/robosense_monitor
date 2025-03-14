import os

IS_ROS1 = os.environ.get("ROS_VERSION") == "1"
IS_ROS2 = os.environ.get("ROS_VERSION") == "2"
