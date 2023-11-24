#!/usr/bin/env python3.10

import os
import subprocess
from typing import Dict, Any, Optional, List

import rospy

from multiverse_launch import MultiverseLaunch, find_files, run_subprocess, get_urdf_str_from_ros_package


def is_roscore_running() -> bool:
    import rosgraph
    return rosgraph.is_master_online()


class MultiverseRosLaunch(MultiverseLaunch):
    multiverse_control_pkg_path: Optional[str]
    mesh_abspath_prefix: Optional[str]

    def __init__(self):
        super().__init__()

    def start_ros_socket(self):
        ros_dict = self.ros
        if ros_dict is None:
            print("No ROS nodes to run")
            return

        if not is_roscore_running():
            cmd = ["roscore"]
            run_subprocess(cmd)

        self.run_multiverse_ros()

        if "ros_control" in ros_dict or "ros_run" in ros_dict:
            import rospkg, rospy
            rospy.init_node(name="multiverse_launch")
            ros_package = rospkg.RosPack()
            self.multiverse_control_pkg_path = ros_package.get_path("multiverse_control")
            self.mesh_abspath_prefix = os.path.relpath("/", self.multiverse_control_pkg_path)
            self.mesh_abspath_prefix = os.path.join("package://multiverse_control", self.mesh_abspath_prefix)

            for ros_control in self.ros.get("ros_control", {}):
                self.run_ros_control(ros_control)

    def run_multiverse_ros(self) -> Optional[subprocess.Popen]:
        if not any([key in self.ros for key in ["services", "publishers", "subscribers"]]):
            return None

        cmd = [
            "multiverse_ros_run",
            f'--multiverse_server="{self.multiverse_server}"',
        ]
        for ros_node_type in ["services", "publishers", "subscribers"]:
            if ros_node_type in self.ros:
                cmd.append(f'--{ros_node_type}="{self.ros[ros_node_type]}"')

        return run_subprocess(cmd)

    def run_ros_control(self, ros_control: Dict[str, Any]) -> List[subprocess.Popen]:
        processes: List[subprocess.Popen] = []
        controller_manager = ros_control["controller_manager"]
        robot = controller_manager["robot"]
        robot_description = controller_manager["robot_description"]
        actuators = controller_manager["actuators"]
        robot_urdf_path = find_files(self.resources_paths, controller_manager["urdf"])
        robot_urdf_str = get_urdf_str_from_ros_package(self.mesh_abspath_prefix, self.multiverse_control_pkg_path,
                                                       robot_urdf_path)
        rospy.set_param(f"{robot_description}", f"{robot_urdf_str}")
        multiverse_dict = {
            "multiverse_server": self.multiverse_server,
            "multiverse_client": {
                "host": ros_control["host"],
                "port": ros_control["port"],
                "meta_data": ros_control["meta_data"],
            },
            "controller_manager": {"robot": robot, "robot_description": robot_description, "actuators": actuators},
        }
        cmd = [
            "rosrun",
            "multiverse_control",
            "multiverse_control_node",
            f"{multiverse_dict}".replace(" ", "").replace("'", '"').replace('"', '"'),
        ]
        process = run_subprocess(cmd)
        processes.append(process)

        control_config_path = find_files(self.resources_paths, controller_manager["config"])
        os.environ["ROS_NAMESPACE"] = f"{robot}"
        cmd = [
            "rosparam",
            "load",
            f"{control_config_path}",
        ]
        process = run_subprocess(cmd)
        process.wait()
        processes.append(process)

        for command, controllers in controller_manager["controllers"].items():
            cmd = [
                      "rosrun",
                      "controller_manager",
                      "controller_manager",
                      f"{command}",
                  ] + controllers[0].split()
            process = run_subprocess(cmd)
            processes.append(process)

        return processes

    def run_ros_nodes(self):
        ros_dict = self.ros
        if ros_dict is None:
            print("No ROS nodes to run")
            return

        if not is_roscore_running():
            cmd = ["roscore"]
            run_subprocess(cmd)

        if "ros_run" in ros_dict:
            ros_run = ros_dict["ros_run"]
            if "rviz" in ros_run:
                from ros_packges.rviz import run_rviz
                run_rviz(ros_run["rviz"], self.resources_paths, self.mesh_abspath_prefix,
                         self.multiverse_control_pkg_path)

            if "map_server" in ros_run:
                from ros_packges.map_server import run_map_server
                _, map_path = run_map_server(ros_run["map_server"], self.resources_paths)

                if "move_base" in ros_run:
                    from ros_packges.move_base import run_move_base
                    run_move_base(ros_run["move_base"], self.resources_paths, map_path)

    @property
    def ros(self) -> Optional[Dict[str, Any]]:
        return self.data.get("ros")


def main():
    multiverse_launch = MultiverseRosLaunch()
    multiverse_launch.start_ros_socket()


if __name__ == "__main__":
    main()
