#!/usr/bin/env python3

import importlib.util

if importlib.util.find_spec("rospy") and importlib.util.find_spec("rclpy"):
    raise ImportError(f"Both rospy and rclpy are importable.")
if not importlib.util.find_spec("rospy") and not importlib.util.find_spec("rclpy"):
    raise ImportError(f"Neither rospy nor rclpy is importable.")

USING_ROS1 = importlib.util.find_spec("rospy") and not importlib.util.find_spec("rclpy")