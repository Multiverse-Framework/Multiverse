#!/usr/bin/env python3

import os, sys
import subprocess
import platform
from typing import Dict, Any, Optional, List

from multiverse_ros_socket import Interface, INTERFACE

from multiverse_launch import MultiverseLaunch
from utils import (
    find_files,
    run_subprocess,
    get_urdf_str_from_ros_package,
    get_urdf_str_abs,
    is_roscore_running,
)

ubuntu_version = platform.freedesktop_os_release().get("VERSION_ID")


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
            if INTERFACE != Interface.ROS1:
                raise ValueError(f"Invalid interface")

            import rospy

            rospy.init_node(name="multiverse_launch")

            processes += self.run_ros_nodes(ros_dict.get("ros_run", {}))

            for ros_control in self.ros.get("ros_control", {}):
                processes += self._run_ros1_control(ros_control)

        if "ros2_control" in ros_dict or "ros2_run" in ros_dict:
            if INTERFACE != Interface.ROS2:
                raise ValueError(f"Invalid interface")

            processes += self.run_ros_nodes(ros_dict.get("ros2_run", {}))

            for ros_control in self.ros.get("ros2_control", {}):
                if INTERFACE == Interface.ROS2:
                    processes += self._run_ros2_control(ros_control)
                else:
                    raise ValueError(f"Invalid interface")

        return processes

    def run_multiverse_ros(self) -> Optional[subprocess.Popen]:
        ros_nodes = self.ros.get("ros_nodes")
        if ros_nodes is None or not any(
                [key in ros_nodes for key in ["services", "publishers", "subscribers"]]
        ):
            return None
        
        python_executable = sys.executable
        if INTERFACE == Interface.ROS2 and os.environ.get("ROS_DISTRO") == "foxy":
            python_executable = "python3.8"

        if os.name == "posix":
            multiverse_ros_run_file_name = "multiverse_ros_run"
        elif os.name == "nt":
            multiverse_ros_run_file_name = "multiverse_ros_run.py"
        else:
            raise ValueError(f"Unsupported OS: {os.name}")
        
        # Find the path to multiverse_ros_run.py from PATH
        for path in os.environ["PATH"].split(os.pathsep):
            path = path.strip('"')
            multiverse_ros_run_path = os.path.join(path, multiverse_ros_run_file_name)
            if os.path.isfile(multiverse_ros_run_path):
                break
        else:
            raise FileNotFoundError(f"{multiverse_ros_run_file_name} not found in PATH")        
        cmd = [
            python_executable,
            multiverse_ros_run_path,
            f"--host={self.multiverse_server.get('host', 'tcp://127.0.0.1')}",
            f"--server_port={self.multiverse_server.get('port', '7000')}",
        ]
        for ros_node_type in ["services", "publishers", "subscribers"]:
            if ros_node_type in ros_nodes:
                cmd.append(f'--{ros_node_type}="{ros_nodes[ros_node_type]}"')

        return run_subprocess(cmd)

    def _run_ros1_control(self, ros_control: Dict[str, Any]) -> List[subprocess.Popen]:
        processes: List[subprocess.Popen] = []

        processes += self._run_controller_command(ros_control)

        process = self._run_controller_node(ros_control)
        processes.append(process)

        return processes

    def _run_ros2_control(self, ros_control: Dict[str, Any]) -> List[subprocess.Popen]:
        processes: List[subprocess.Popen] = []

        processes += self._run_controller_command(ros_control)

        process = self._run_controller_node(ros_control)
        processes.append(process)

        return processes

    def _run_controller_node(self, ros_control: Dict[str, Any]) -> subprocess.Popen:
        controller_manager = ros_control["controller_manager"]
        robot_description = controller_manager["robot_description"]
        joint_state = {}
        world_name = ros_control["meta_data"]["world_name"]
        for simulation in self.simulations.values():
            if (
                    simulation.get("world") is not None
                    and simulation["world"]["name"] == world_name
            ):
                robot = simulation["robots"][controller_manager["robot"]]
                joint_state = robot.get("joint_state", {})
                break
        if INTERFACE == Interface.ROS1:
            import rospy

            robot_urdf_path = find_files(
                self.resources_paths, controller_manager["urdf"]
            )
            robot_urdf_str = get_urdf_str_from_ros_package(
                self.mesh_abspath_prefix,
                self.multiverse_control_pkg_path,
                robot_urdf_path,
            )

            rospy.set_param(f"{robot_description}", f"{robot_urdf_str}")
            multiverse_dict = {
                "host": self.multiverse_server["host"],
                "server_port": self.multiverse_server["port"],
                "client_port": ros_control["port"],
                "meta_data": ros_control["meta_data"],
                "controller_manager": {
                    "robot": controller_manager["robot"],
                    "robot_description": robot_description,
                    "actuators": controller_manager["actuators"],
                    "init_joint_state": joint_state,
                },
            }
            cmd = [
                "rosrun",
                "multiverse_control",
                "multiverse_control_node",
                f"robot_description:={robot_description}",
                f"{multiverse_dict}".replace(" ", "")
                .replace("'", '"')
                .replace('"', '"'),
            ]
        elif INTERFACE == Interface.ROS2:
            robot_urdf_path = find_files(
                self.resources_paths, controller_manager["urdf"]
            )
            robot_urdf_str = get_urdf_str_abs(robot_urdf_path)

            from xml.etree import ElementTree as ET

            robot_element = ET.fromstring(robot_urdf_str)
            if robot_element.tag != "robot":
                raise ValueError("No robot tag found in URDF")

            ros2_control_elements = robot_element.findall("ros2_control")
            for ros2_control_element in ros2_control_elements:
                robot_element.remove(ros2_control_element)

            ros2_control_element = ET.Element("ros2_control")
            ros2_control_element.set("name", robot_element.attrib["name"])
            ros2_control_element.set("type", "system")

            hardware_element = ET.Element("hardware")
            plugin_element = ET.Element("plugin")
            plugin_element.text = "/MultiverseHWInterface"
            hardware_element.append(plugin_element)

            world_name_param_element = ET.Element("param")
            world_name_param_element.set("name", "world_name")
            world_name_param_element.text = ros_control["meta_data"]["world_name"]
            hardware_element.append(world_name_param_element)

            length_unit_param_element = ET.Element("param")
            length_unit_param_element.set("name", "length_unit")
            length_unit_param_element.text = ros_control["meta_data"]["length_unit"]
            hardware_element.append(length_unit_param_element)

            angle_unit_param_element = ET.Element("param")
            angle_unit_param_element.set("name", "angle_unit")
            angle_unit_param_element.text = ros_control["meta_data"]["angle_unit"]
            hardware_element.append(angle_unit_param_element)

            mass_unit_param_element = ET.Element("param")
            mass_unit_param_element.set("name", "mass_unit")
            mass_unit_param_element.text = ros_control["meta_data"]["mass_unit"]
            hardware_element.append(mass_unit_param_element)

            time_unit_param_element = ET.Element("param")
            time_unit_param_element.set("name", "time_unit")
            time_unit_param_element.text = ros_control["meta_data"]["time_unit"]
            hardware_element.append(time_unit_param_element)

            handedness_param_element = ET.Element("param")
            handedness_param_element.set("name", "handedness")
            handedness_param_element.text = ros_control["meta_data"]["handedness"]
            hardware_element.append(handedness_param_element)

            server_host_param_element = ET.Element("param")
            server_host_param_element.set("name", "host")
            server_host_param_element.text = self.multiverse_server["host"]
            hardware_element.append(server_host_param_element)

            server_port_param_element = ET.Element("param")
            server_port_param_element.set("name", "server_port")
            server_port_param_element.text = str(self.multiverse_server["port"])
            hardware_element.append(server_port_param_element)

            client_port_param_element = ET.Element("param")
            client_port_param_element.set("name", "client_port")
            client_port_param_element.text = str(ros_control["port"])
            hardware_element.append(client_port_param_element)

            ros2_control_element.append(hardware_element)

            from urdf_parser_py.urdf import URDF, Joint

            robot_urdf = URDF.from_xml_file(robot_urdf_path)
            joint: Joint
            for joint in robot_urdf.joints:
                if joint.type == "fixed":
                    continue

                joint_element = ET.Element("joint")
                ros2_control_element.append(joint_element)

                joint_element.set("name", joint.name)

                state_interface = ET.Element("state_interface")
                state_interface.set("name", "position")
                initial_value_param = ET.Element("param")
                initial_value_param.set("name", "initial_value")
                if joint.name in joint_state:
                    initial_value_param.text = str(joint_state[joint.name])
                else:
                    initial_value_param.text = "0.0"
                state_interface.append(initial_value_param)
                joint_element.append(state_interface)

                state_interface = ET.Element("state_interface")
                state_interface.set("name", "velocity")
                joint_element.append(state_interface)

                actuator_name = None
                for actuator_name, joint_name in controller_manager[
                    "actuators"
                ].items():
                    if joint_name == joint.name:
                        break
                else:
                    continue

                actuator_param = ET.Element("param")
                actuator_param.set("name", "actuator")
                actuator_param.text = actuator_name
                joint_element.append(actuator_param)

                state_interface = ET.Element("command_interface")
                state_interface.set("name", "position")
                if joint.limit is not None:
                    min_position_param = ET.Element("param")
                    min_position_param.set("name", "min")
                    min_position_param.text = str(joint.limit.lower)
                    state_interface.append(min_position_param)

                    max_position_param = ET.Element("param")
                    max_position_param.set("name", "max")
                    max_position_param.text = str(joint.limit.upper)
                    state_interface.append(max_position_param)

                    initial_value_param = ET.Element("param")
                    initial_value_param.set("name", "initial_value")
                    if joint.name in joint_state:
                        initial_value_param.text = str(joint_state[joint.name])
                    else:
                        initial_value_param.text = "0.0"
                    state_interface.append(initial_value_param)
                joint_element.append(state_interface)

                state_interface = ET.Element("command_interface")
                state_interface.set("name", "velocity")
                if joint.limit is not None:
                    min_velocity_param = ET.Element("param")
                    min_velocity_param.set("name", "min")
                    min_velocity_param.text = str(-joint.limit.velocity)
                    state_interface.append(min_velocity_param)

                    max_velocity_param = ET.Element("param")
                    max_velocity_param.set("name", "max")
                    max_velocity_param.text = str(joint.limit.velocity)
                    state_interface.append(max_velocity_param)
                joint_element.append(state_interface)

                state_interface = ET.Element("command_interface")
                state_interface.set("name", "effort")
                if joint.limit is not None:
                    min_effort_param = ET.Element("param")
                    min_effort_param.set("name", "min")
                    min_effort_param.text = str(-joint.limit.effort)
                    state_interface.append(min_effort_param)

                    max_effort_param = ET.Element("param")
                    max_effort_param.set("name", "max")
                    max_effort_param.text = str(joint.limit.effort)
                    state_interface.append(max_effort_param)
                joint_element.append(state_interface)

            robot_element.append(ros2_control_element)

            tmp_urdf_path = os.path.join("/tmp", os.path.basename(robot_urdf_path))
            tree = ET.ElementTree(robot_element)
            tree.write(tmp_urdf_path, encoding="utf-8", xml_declaration=True)

            robot_urdf_str = ET.tostring(robot_element, encoding="unicode")

            tf_topic = controller_manager.get("tf_topic", "/tf")
            if ubuntu_version == "22.04":
                ns = "~"
            else:
                ns = ""
            cmd = [
                "ros2",
                "run",
                "robot_state_publisher",
                "robot_state_publisher",
                "--ros-args",
                "--remap",
                f"{ns}/robot_description:={robot_description}",
                "-p",
                f"robot_description:={robot_urdf_str}",
                "-r",
                f"tf:={tf_topic}",
            ]

        return run_subprocess(cmd)

    def run_ros_nodes(self, ros_run: Dict[str, Any]) -> List[subprocess.Popen]:
        processes: List[subprocess.Popen] = []
        if "rviz" in ros_run:
            if INTERFACE != Interface.ROS1:
                raise ValueError(f"Invalid interface")

            from utils import run_rviz

            process = run_rviz(
                ros_run["rviz"],
                self.resources_paths,
                self.mesh_abspath_prefix,
                self.multiverse_control_pkg_path,
            )
            processes.append(process)

        if "rviz2" in ros_run:
            if INTERFACE != Interface.ROS2:
                raise ValueError(f"Invalid interface")

            from utils import run_rviz2

            process = run_rviz2(ros_run["rviz2"], self.resources_paths)
            processes.append(process)

        if "map_server" in ros_run:
            from utils import run_map_server

            process, map_path = run_map_server(
                ros_run["map_server"], self.resources_paths
            )
            processes.append(process)

            if "move_base" in ros_run:
                from utils import run_move_base

                process = run_move_base(
                    ros_run["move_base"], self.resources_paths, map_path
                )
                processes.append(process)

        if "joint_state_publisher_gui" in ros_run:
            if INTERFACE == Interface.ROS1:
                from utils import run_joint_state_publisher_gui

                process = run_joint_state_publisher_gui(
                    ros_run["joint_state_publisher_gui"].get(
                        "robot_description", "robot_description"
                    )
                )
                processes.append(process)

        if "robot_state_publisher" in ros_run:
            if INTERFACE == Interface.ROS1:
                from utils import run_robot_state_publisher

                process = run_robot_state_publisher(
                    ros_run["robot_state_publisher"].get(
                        "robot_description", "robot_description"
                    )
                )
                processes.append(process)

        return processes

    def _run_controller_command(
            self, ros_control: Dict[str, Any]
    ) -> List[subprocess.Popen]:
        processes: List[subprocess.Popen] = []

        controller_manager = ros_control["controller_manager"]
        control_config_path = find_files(
            self.resources_paths, controller_manager["config"]
        )
        if INTERFACE == Interface.ROS1:
            world_name = ros_control["meta_data"]["world_name"]
            os.environ["ROS_NAMESPACE"] = os.path.join(
                world_name, controller_manager["robot"]
            )
            cmd = [
                "rosparam",
                "load",
                f"{control_config_path}",
            ]
            process = run_subprocess(cmd)
            process.wait()

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

        elif INTERFACE == Interface.ROS2:
            robot_description = controller_manager["robot_description"]
            if ubuntu_version == "22.04":
                ns = "~"
            else:
                ns = ""
            cmd = [
                "ros2",
                "run",
                "controller_manager",
                "ros2_control_node",
                "--ros-args",
                "--remap",
                f"{ns}/robot_description:={robot_description}",
                "--params-file",
                f"{control_config_path}",
            ]
            process = run_subprocess(cmd)
            processes.append(process)

            for command, controllers in controller_manager["controllers"].items():
                cmd = [
                    "ros2",
                    "run",
                    "controller_manager",
                    f"{command}",
                ]
                cmd += controllers[0].split()
                cmd += [
                    "--param-file",
                    f"{control_config_path}",
                ]
                process = run_subprocess(cmd)
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
