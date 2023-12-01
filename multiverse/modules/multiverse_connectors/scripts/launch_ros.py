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
    robot_description = controller_manager["robot_description"]
    for command, controllers in controller_manager["controllers"].items():
        cmd = [
                  "rosrun",
                  "controller_manager",
                  "controller_manager",
                  f"robot_description:={robot_description}",
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

            processes += self.run_ros_nodes(ros_dict.get("ros_run", {}))

            for ros_control in self.ros.get("ros_control", {}):
                processes += self.run_ros_control(ros_control)

        return processes

    def run_multiverse_ros(self) -> Optional[subprocess.Popen]:
        ros_nodes = self.ros.get("ros_nodes")
        if ros_nodes is None or not any(
                [key in ros_nodes for key in ["services", "publishers", "subscribers"]]):
            return None

        cmd = [
            "multiverse_ros_run",
            f'--multiverse_server="{self.multiverse_server}"',
        ]
        for ros_node_type in ["services", "publishers", "subscribers"]:
            if ros_node_type in ros_nodes:
                cmd.append(f'--{ros_node_type}="{ros_nodes[ros_node_type]}"')

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

        process = self._load_config(ros_control["controller_manager"], ros_control["meta_data"]["world_name"])
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

        joint_state = {}
        world_name = ros_control["meta_data"]["world_name"]
        for simulation in self.simulations.values():
            if simulation.get("world") is not None and simulation["world"]["name"] == world_name:
                robot = simulation["robots"][controller_manager["robot"]]
                joint_state = robot.get("joint_state", {})
                break

        rospy.set_param(f"{robot_description}", f"{robot_urdf_str}")
        multiverse_dict = {
            "multiverse_server": self.multiverse_server,
            "multiverse_client": {
                "host": ros_control["host"],
                "port": ros_control["port"],
                "meta_data": ros_control["meta_data"],
            },
            "controller_manager": {"robot": controller_manager["robot"], "robot_description": robot_description,
                                   "actuators": controller_manager["actuators"], "init_joint_state": joint_state},
        }
        cmd = [
            "rosrun",
            "multiverse_control",
            "multiverse_control_node",
            f"robot_description:={robot_description}",
            f"{multiverse_dict}".replace(" ", "").replace("'", '"').replace('"', '"'),
        ]
        return run_subprocess(cmd)

    def _load_config(self, controller_manager: Dict[str, Any], world_name: str) -> subprocess.Popen:
        control_config_path = find_files(self.resources_paths, controller_manager["config"])
        os.environ["ROS_NAMESPACE"] = os.path.join(world_name, controller_manager["robot"])
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

        if "joint_state_publisher_gui" in ros_run:
            if INTERFACE == Interface.ROS1:
                from utils import run_joint_state_publisher_gui
                process = run_joint_state_publisher_gui(ros_run["joint_state_publisher_gui"].get("robot_description", "robot_description"))
                processes.append(process)

        if "robot_state_publisher" in ros_run:
            if INTERFACE == Interface.ROS1:
                from utils import run_robot_state_publisher
                process = run_robot_state_publisher(ros_run["robot_state_publisher"].get("robot_description", "robot_description"))
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
