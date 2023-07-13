#!/usr/bin/env python3

import rospy
from typing import Any, List
from geometry_msgs.msg import Twist
from .ros_subscriber import MultiverseRosSubscriber


class cmd_vel_subscriber(MultiverseRosSubscriber):
    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self.__body = kwargs.get("body")
        if self.__body is None:
            raise Exception("Body not found.")
        elif not isinstance(self.__body, str):
            raise TypeError("Body is not a string.")
        self._topic_name = "/cmd_vel"
        self._data_class = Twist

    def _construct_send_meta_data(self):
        self._send_meta_data_dict["send"] = {}
        self._send_meta_data_dict["send"][self.__body] = ["relative_velocity"]
        self._send_meta_data_dict["receive"] = {}
        self._send_meta_data_dict["receive"][self.__body] = ["joint_tvalue", "joint_rvalue"]

    def _init_send_data(self) -> None:
        self._send_data = [0.0 for _ in range(7)]

    def _bind_send_data(self, twist_msg: Twist):
        lin_x_pos = self._receive_data[1]
        lin_y_pos = self._receive_data[2]
        lin_z_pos = self._receive_data[3]
        ang_x_pos = self._receive_data[1]
        ang_y_pos = self._receive_data[2]
        ang_z_pos = self._receive_data[3]
        self._send_data[0] = rospy.Time.now().to_sec()
        self._send_data[1] = twist_msg.linear.x
        self._send_data[2] = twist_msg.linear.y
        self._send_data[3] = twist_msg.linear.z
        self._send_data[4] = twist_msg.angular.x
        self._send_data[5] = twist_msg.angular.y
        self._send_data[6] = twist_msg.angular.z

        # d->qvel[dof_id] = MjSim::odom_vels[robot + "_lin_odom_x_joint"] * mju_cos(odom_y_joint_pos) * mju_cos(odom_z_joint_pos) + MjSim::odom_vels[robot + "_lin_odom_y_joint"] * (mju_sin(odom_x_joint_pos) * mju_sin(odom_y_joint_pos) * mju_cos(odom_z_joint_pos) - mju_cos(odom_x_joint_pos) * mju_sin(odom_z_joint_pos)) + MjSim::odom_vels[robot + "_lin_odom_z_joint"] * (mju_cos(odom_x_joint_pos) * mju_sin(odom_y_joint_pos) * mju_cos(odom_z_joint_pos) + mju_sin(odom_x_joint_pos) * mju_sin(odom_z_joint_pos));
		# 	}
		# }
		# if (MjSim::add_odom_joints[robot]["lin_odom_y_joint"])
		# {
		# 	const std::string joint_name = robot + "_lin_odom_y_joint";
		# 	const int joint_id = mj_name2id(m, mjtObj::mjOBJ_JOINT, joint_name.c_str());
		# 	if (joint_id != -1)
		# 	{
		# 		const int dof_id = m->jnt_dofadr[joint_id];
		# 		d->qvel[dof_id] = MjSim::odom_vels[robot + "_lin_odom_x_joint"] * mju_cos(odom_y_joint_pos) * mju_sin(odom_z_joint_pos) + MjSim::odom_vels[robot + "_lin_odom_y_joint"] * (mju_sin(odom_x_joint_pos) * mju_sin(odom_y_joint_pos) * mju_sin(odom_z_joint_pos) + mju_cos(odom_x_joint_pos) * mju_cos(odom_z_joint_pos)) + MjSim::odom_vels[robot + "_lin_odom_z_joint"] * (mju_cos(odom_x_joint_pos) * mju_sin(odom_y_joint_pos) * mju_sin(odom_z_joint_pos) - mju_sin(odom_x_joint_pos) * mju_cos(odom_z_joint_pos));
