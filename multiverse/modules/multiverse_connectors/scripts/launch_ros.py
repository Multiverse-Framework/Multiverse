#!/usr/bin/env python3.10

import os
import subprocess
from typing import Dict, Any, Optional, List

from multiverse_ros_socket import Interface, INTERFACE

from multiverse_launch import MultiverseLaunch
from utils import find_files, run_subprocess, get_urdf_str_from_ros_package

def is_roscore_running() -> bool:
    import rosgraph
    return rosgraph.is_master_online()


def run_controller_command(controller_manager: Dict[str, Any]) -> List[subprocess.Popen]:
    processes: List[subprocess.Popen] = []
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


class MultiverseRosLaunch(MultiverseLaunch):
    multiverse_control_pkg_path: Optional[str]
    mesh_abspath_prefix: Optional[str]

    def __init__(self):
        super().__init__()

    @property
    def multiverse_control_pkg_path(self) -> str:
        if INTERFACE == Interface.ROS1:
            import rospkg
            ros_package = rospkg.RosPack()
            return ros_package.get_path("multiverse_control")

    @property
    def mesh_abspath_prefix(self) -> str:
        mesh_abspath_prefix = os.path.relpath("/", self.multiverse_control_pkg_path)
        return os.path.join("package://multiverse_control", mesh_abspath_prefix)

    def start_ros_socket(self) -> List[subprocess.Popen]:
        processes: List[subprocess.Popen] = []
        ros_dict = self.ros
        if ros_dict is None:
            print("No ROS nodes to run")
            return processes

        if INTERFACE == Interface.ROS1 and not is_roscore_running():
            process = run_subprocess(["roscore"])
            processes.append(process)

        processes.append(self.run_multiverse_ros())

        if "ros_control" in ros_dict or "ros_run" in ros_dict:
            if INTERFACE == Interface.ROS1:
                import rospy

                rospy.init_node(name="multiverse_launch")

            elif INTERFACE == Interface.ROS2:
                pass
            else:
                raise ValueError(f"Invalid interface")

            for ros_control in self.ros.get("ros_control", {}):
                processes += self.run_ros_control(ros_control)

            processes += self.run_ros_nodes(ros_dict.get("ros_run", {}))

        return processes

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
        if INTERFACE == Interface.ROS1:
            return self._run_ros1_control(ros_control)
        elif INTERFACE == Interface.ROS2:
            return []
        else:
            raise ValueError(f"Invalid interface")

    def _run_ros1_control(self, ros_control: Dict[str, Any]) -> List[subprocess.Popen]:
        processes: List[subprocess.Popen] = []

        process = self._run_controller_node(ros_control)
        processes.append(process)

        process = self._load_config(ros_control["controller_manager"])
        processes.append(process)

        processes += run_controller_command(ros_control["controller_manager"])

        return processes

    def _run_controller_node(self, ros_control: Dict[str, Any]) -> subprocess.Popen:
        if INTERFACE != Interface.ROS1:
            raise NotImplementedError(f"Controller node for {INTERFACE} not implemented")
        import rospy
        controller_manager = ros_control["controller_manager"]
        robot_description = controller_manager["robot_description"]
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
            "controller_manager": {"robot": controller_manager["robot"], "robot_description": robot_description,
                                   "actuators": controller_manager["actuators"]},
        }
        cmd = [
            "rosrun",
            "multiverse_control",
            "multiverse_control_node",
            f"{multiverse_dict}".replace(" ", "").replace("'", '"').replace('"', '"'),
        ]
        return run_subprocess(cmd)

    def _load_config(self, controller_manager: Dict[str, Any]) -> subprocess.Popen:
        control_config_path = find_files(self.resources_paths, controller_manager["config"])
        os.environ["ROS_NAMESPACE"] = controller_manager["robot"]
        cmd = [
            "rosparam",
            "load",
            f"{control_config_path}",
        ]
        process = run_subprocess(cmd)
        process.wait()
        return process

    def run_ros_nodes(self, ros_run: Dict[str, Any]) -> List[subprocess.Popen]:
        processes: List[subprocess.Popen] = []
        if "rviz" in ros_run:
            if INTERFACE == Interface.ROS1:
                from utils import run_rviz
                process = run_rviz(ros_run["rviz"], self.resources_paths, self.mesh_abspath_prefix,
                                   self.multiverse_control_pkg_path)
                processes.append(process)
            elif INTERFACE == Interface.ROS2:
                from utils import run_rviz2
                process = run_rviz2(ros_run["rviz"], self.resources_paths)
                processes.append(process)

        if "map_server" in ros_run:
            from utils import run_map_server
            process, map_path = run_map_server(ros_run["map_server"], self.resources_paths)
            processes.append(process)

            if "move_base" in ros_run:
                from utils import run_move_base
                process = run_move_base(ros_run["move_base"], self.resources_paths, map_path)
                processes.append(process)

        return processes

    @property
    def ros(self) -> Optional[Dict[str, Any]]:
        if INTERFACE == Interface.ROS1:
            return self.multiverse_clients.get("ros")
        elif INTERFACE == Interface.ROS2:
            return self.multiverse_clients.get("ros2")
        else:
            raise ValueError(f"Invalid interface")


def main():
    multiverse_launch = MultiverseRosLaunch()
    multiverse_launch.start_ros_socket()


if __name__ == "__main__":
    main()
