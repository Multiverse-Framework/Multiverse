// Copyright (c) 2023, Giang Hoang Nguyen - Institute for Artificial Intelligence, University Bremen

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "mj_multiverse_client.h"

#include <chrono>
#include <csignal>
#include <iostream>

std::mutex MjMultiverseClient::mutex;

void MjMultiverseClient::init(const std::string &server_host, const std::string &server_port, const std::string &client_port, const Json::Value &in_send_objects_json, const Json::Value &in_receive_objects_json, const std::string &in_world)
{
	send_objects_json = in_send_objects_json;
	receive_objects_json = in_receive_objects_json;
	world = in_world;

	host = server_host;
	server_socket_addr = host + ":" + server_port;

	port = client_port;

	connect();
}

bool MjMultiverseClient::init_objects()
{
	std::set<std::string> body_attributes = {"position", "quaternion", "relative_velocity", "force", "torque"};
	std::set<std::string> joint_attributes = {"joint_rvalue", "joint_tvalue", "joint_position", "joint_quaternion"};

	for (const std::string &object_name : receive_objects_json.getMemberNames())
	{
		receive_objects[object_name] = {};
		for (const Json::Value &attribute_json : receive_objects_json[object_name])
		{
			const std::string attribute_name = attribute_json.asString();
			const int body_id = mj_name2id(m, mjtObj::mjOBJ_BODY, object_name.c_str());
			const int joint_id = mj_name2id(m, mjtObj::mjOBJ_JOINT, object_name.c_str());
			if (body_attributes.count(attribute_name) != 0 && body_id != -1)
			{
				receive_objects[object_name].insert(attribute_name);
			}
			else if (joint_attributes.count(attribute_name) != 0 && joint_id != -1)
			{
				if ((m->jnt_type[joint_id] == mjtJoint::mjJNT_HINGE && strcmp(attribute_name.c_str(), "joint_rvalue") == 0) || (m->jnt_type[joint_id] == mjtJoint::mjJNT_SLIDE && strcmp(attribute_name.c_str(), "joint_tvalue") == 0))
				{
					receive_objects[object_name].insert(attribute_name);
				}
			}
		}
	}

	for (const std::string &object_name : send_objects_json.getMemberNames())
	{
		for (const Json::Value &attribute_json : send_objects_json[object_name])
		{
			const std::string attribute_name = attribute_json.asString();
			if (strcmp(object_name.c_str(), "body") == 0)
			{
				if (body_attributes.count(attribute_name) != 0)
				{
					for (int body_id = 1; body_id < m->nbody; body_id++)
					{
						send_objects[mj_id2name(m, mjtObj::mjOBJ_BODY, body_id)].insert(attribute_name);
					}
				}
			}
			else if (strcmp(object_name.c_str(), "joint") == 0)
			{
				if (strcmp(attribute_name.c_str(), "joint_rvalue") == 0)
				{
					for (int joint_id = 0; joint_id < m->njnt; joint_id++)
					{
						if (m->jnt_type[joint_id] == mjtJoint::mjJNT_HINGE)
						{
							send_objects[mj_id2name(m, mjtObj::mjOBJ_JOINT, joint_id)].insert(attribute_name);
						}
					}
				}
				else if (strcmp(attribute_name.c_str(), "joint_tvalue") == 0)
				{
					for (int joint_id = 0; joint_id < m->njnt; joint_id++)
					{
						if (m->jnt_type[joint_id] == mjtJoint::mjJNT_SLIDE)
						{
							send_objects[mj_id2name(m, mjtObj::mjOBJ_JOINT, joint_id)].insert(attribute_name);
						}
					}
				}
			}
			else
			{
				const int body_id = mj_name2id(m, mjtObj::mjOBJ_BODY, object_name.c_str());
				const int joint_id = mj_name2id(m, mjtObj::mjOBJ_JOINT, object_name.c_str());
				if (body_attributes.count(attribute_name) != 0 && body_id != -1)
				{
					send_objects[object_name].insert(attribute_name);
				}
				else if (joint_attributes.count(attribute_name) != 0 && joint_id != -1)
				{
					if ((m->jnt_type[joint_id] == mjtJoint::mjJNT_HINGE && strcmp(attribute_name.c_str(), "joint_rvalue") == 0) || (m->jnt_type[joint_id] == mjtJoint::mjJNT_SLIDE && strcmp(attribute_name.c_str(), "joint_tvalue") == 0))
					{
						send_objects[object_name].insert(attribute_name);
					}
				}
			}
		}
	}

	return send_objects.size() > 0 || receive_objects.size() > 0;
}

void MjMultiverseClient::start_connect_to_server_thread()
{
	connect_to_server_thread = std::thread(&MjMultiverseClient::connect_to_server, this);
}

void MjMultiverseClient::wait_for_connect_to_server_thread_finish()
{
	if (connect_to_server_thread.joinable())
	{
		connect_to_server_thread.join();
	}
}

void MjMultiverseClient::start_meta_data_thread()
{
	meta_data_thread = std::thread(&MjMultiverseClient::send_and_receive_meta_data, this);
}

void MjMultiverseClient::wait_for_meta_data_thread_finish()
{
	if (meta_data_thread.joinable())
	{
		meta_data_thread.join();
	}
}

void MjMultiverseClient::bind_request_meta_data()
{
	mtx.lock();
	// Create JSON object and populate it
	request_meta_data_json.clear();
	request_meta_data_json["world"] = world;
	request_meta_data_json["length_unit"] = "m";
	request_meta_data_json["angle_unit"] = "rad";
	request_meta_data_json["force_unit"] = "N";
	request_meta_data_json["time_unit"] = "s";
	request_meta_data_json["handedness"] = "rhs";

	for (const std::pair<std::string, std::set<std::string>> &send_object : send_objects)
	{
		const int body_id = mj_name2id(m, mjtObj::mjOBJ_BODY, send_object.first.c_str());
		const int joint_id = mj_name2id(m, mjtObj::mjOBJ_JOINT, send_object.first.c_str());
		if (body_id != -1)
		{
			const std::string body_name = send_object.first;
			for (const std::string &attribute_name : send_object.second)
			{
				request_meta_data_json["send"][body_name].append(attribute_name);
			}
		}
		else if (joint_id != -1)
		{
			const std::string joint_name = send_object.first;
			const int qpos_id = m->jnt_qposadr[joint_id];
			for (const std::string &attribute_name : send_object.second)
			{
				request_meta_data_json["send"][joint_name].append(attribute_name);
			}
		}
	}

	for (const std::pair<std::string, std::set<std::string>> &receive_object : receive_objects)
	{
		const int body_id = mj_name2id(m, mjtObj::mjOBJ_BODY, receive_object.first.c_str());
		const int joint_id = mj_name2id(m, mjtObj::mjOBJ_JOINT, receive_object.first.c_str());
		if (body_id != -1)
		{
			const std::string body_name = receive_object.first;
			for (const std::string &attribute_name : receive_object.second)
			{
				request_meta_data_json["receive"][body_name].append(attribute_name);
			}
		}
		else if (joint_id != -1)
		{
			const std::string joint_name = receive_object.first;
			const int qpos_id = m->jnt_qposadr[joint_id];
			for (const std::string &attribute_name : receive_object.second)
			{
				request_meta_data_json["receive"][joint_name].append(attribute_name);
			}
		}
	}

	mtx.unlock();

	request_meta_data_str = request_meta_data_json.toStyledString();
}

void MjMultiverseClient::bind_response_meta_data()
{
	mtx.lock();
	for (const std::pair<std::string, std::set<std::string>> &send_object : send_objects)
	{
		const int body_id = mj_name2id(m, mjtObj::mjOBJ_BODY, send_object.first.c_str());
		const int joint_id = mj_name2id(m, mjtObj::mjOBJ_JOINT, send_object.first.c_str());
		if (body_id != -1)
		{
			if (m->body_dofnum[body_id] == 6 && m->body_jntadr[body_id] != -1 && m->jnt_type[m->body_jntadr[body_id]] == mjtJoint::mjJNT_FREE)
			{
				mjtNum *xpos_desired = d->xpos + 3 * body_id;
				mjtNum *xquat_desired = d->xquat + 4 * body_id;

				for (const std::string &attribute_name : send_object.second)
				{
					if (strcmp(attribute_name.c_str(), "position") == 0)
					{
						const double x = response_meta_data_json["send"][send_object.first][attribute_name][0].asDouble();
						const double y = response_meta_data_json["send"][send_object.first][attribute_name][1].asDouble();
						const double z = response_meta_data_json["send"][send_object.first][attribute_name][2].asDouble();
						if (std::isnan(x) && std::isnan(y) && std::isnan(z))
						{
							xpos_desired[0] = x;
							xpos_desired[1] = y;
							xpos_desired[2] = z;
						}
					}
					else if (strcmp(attribute_name.c_str(), "quaternion") == 0)
					{
						const double w = response_meta_data_json["send"][send_object.first][attribute_name][0].asDouble();
						const double x = response_meta_data_json["send"][send_object.first][attribute_name][1].asDouble();
						const double y = response_meta_data_json["send"][send_object.first][attribute_name][2].asDouble();
						const double z = response_meta_data_json["send"][send_object.first][attribute_name][3].asDouble();
						if (std::isnan(w) && std::isnan(x) && std::isnan(y) && std::isnan(z))
						{
							xquat_desired[0] = w;
							xquat_desired[1] = x;
							xquat_desired[2] = y;
							xquat_desired[3] = z;
						}
					}
				}

				const int qpos_id = m->jnt_qposadr[m->body_jntadr[body_id]];
				d->qpos[qpos_id] = xpos_desired[0];
				d->qpos[qpos_id + 1] = xpos_desired[1];
				d->qpos[qpos_id + 2] = xpos_desired[2];
				d->qpos[qpos_id + 3] = xquat_desired[0];
				d->qpos[qpos_id + 4] = xquat_desired[1];
				d->qpos[qpos_id + 5] = xquat_desired[2];
				d->qpos[qpos_id + 6] = xquat_desired[3];
			}
			else if (m->body_dofnum[body_id] == 3 && m->body_jntadr[body_id] != -1 && m->jnt_type[m->body_jntadr[body_id]] == mjtJoint::mjJNT_BALL)
			{
				for (const std::string &attribute_name : send_object.second)
				{
					if (strcmp(attribute_name.c_str(), "quaternion") == 0)
					{
						const double w = response_meta_data_json["send"][send_object.first][attribute_name][0].asDouble();
						const double x = response_meta_data_json["send"][send_object.first][attribute_name][1].asDouble();
						const double y = response_meta_data_json["send"][send_object.first][attribute_name][2].asDouble();
						const double z = response_meta_data_json["send"][send_object.first][attribute_name][3].asDouble();

						if (std::isnan(w) && std::isnan(x) && std::isnan(y) && std::isnan(z))
						{
							const mjtNum xquat_desired[4] = {w, x, y, z};
							mjtNum *xquat_current_neg = d->xquat + 4 * body_id;
							mju_negQuat(xquat_current_neg, xquat_current_neg);

							const int qpos_id = m->jnt_qposadr[m->body_jntadr[body_id]];
							mju_mulQuat(d->qpos + qpos_id, xquat_current_neg, xquat_desired);
						}
					}
				}
			}
		}
		else if (joint_id != -1)
		{
			for (const std::string &attribute_name : send_object.second)
			{
				if ((strcmp(attribute_name.c_str(), "joint_rvalue") == 0 && m->jnt_type[joint_id] == mjtJoint::mjJNT_HINGE) ||
					(strcmp(attribute_name.c_str(), "joint_tvalue") == 0 && m->jnt_type[joint_id] == mjtJoint::mjJNT_SLIDE))
				{
					const double v = response_meta_data_json["send"][send_object.first][attribute_name][0].asDouble();
					if (std::isnan(v))
					{
						const int qpos_id = m->jnt_qposadr[joint_id];
						d->qpos[qpos_id] = v;
					}
				}
				else if ((strcmp(attribute_name.c_str(), "joint_quaternion") == 0 && m->jnt_type[joint_id] == mjtJoint::mjJNT_BALL))
				{
					const double w = response_meta_data_json["send"][send_object.first][attribute_name][0].asDouble();
					const double x = response_meta_data_json["send"][send_object.first][attribute_name][1].asDouble();
					const double y = response_meta_data_json["send"][send_object.first][attribute_name][2].asDouble();
					const double z = response_meta_data_json["send"][send_object.first][attribute_name][3].asDouble();

					if (std::isnan(w) && std::isnan(x) && std::isnan(y) && std::isnan(z))
					{
						const int qpos_id = m->jnt_qposadr[joint_id];
						d->qpos[qpos_id] = w;
						d->qpos[qpos_id + 1] = x;
						d->qpos[qpos_id + 2] = y;
						d->qpos[qpos_id + 3] = z;
					}
				}
			}
		}
	}
	mtx.unlock();
}

void MjMultiverseClient::init_send_and_receive_data()
{
	mtx.lock();
	for (const std::pair<std::string, std::set<std::string>> &send_object : send_objects)
	{
		const int body_id = mj_name2id(m, mjtObj::mjOBJ_BODY, send_object.first.c_str());
		const int joint_id = mj_name2id(m, mjtObj::mjOBJ_JOINT, send_object.first.c_str());
		if (body_id != -1)
		{
			const std::string body_name = send_object.first;
			const int body_ref_id = mj_name2id(m, mjtObj::mjOBJ_BODY, (body_name + "_ref").c_str());
			const int dof_id = m->body_dofadr[body_id];
			for (const std::string &attribute_name : send_object.second)
			{
				if (strcmp(attribute_name.c_str(), "position") == 0)
				{
					send_data_vec.emplace_back(&d->xpos[3 * body_id]);
					send_data_vec.emplace_back(&d->xpos[3 * body_id + 1]);
					send_data_vec.emplace_back(&d->xpos[3 * body_id + 2]);
				}
				else if (strcmp(attribute_name.c_str(), "quaternion") == 0)
				{
					send_data_vec.emplace_back(&d->xquat[4 * body_id]);
					send_data_vec.emplace_back(&d->xquat[4 * body_id + 1]);
					send_data_vec.emplace_back(&d->xquat[4 * body_id + 2]);
					send_data_vec.emplace_back(&d->xquat[4 * body_id + 3]);
				}
				else if (strcmp(attribute_name.c_str(), "force") == 0)
				{
					if (m->body_dofnum[body_id] == 6 && m->body_jntadr[body_id] != -1 && m->jnt_type[m->body_jntadr[body_id]] == mjtJoint::mjJNT_FREE)
					{
						if (contact_efforts.count(body_id) == 0)
						{
							contact_efforts[body_id] = (mjtNum *)calloc(6, sizeof(mjtNum));
						}

						send_data_vec.emplace_back(&contact_efforts[body_id][0]);
						send_data_vec.emplace_back(&contact_efforts[body_id][1]);
						send_data_vec.emplace_back(&contact_efforts[body_id][2]);
					}
					else
					{
						printf("%s for %s not supported", attribute_name.c_str(), body_name.c_str());
					}
				}
				else if (strcmp(attribute_name.c_str(), "torque") == 0)
				{
					if (m->body_dofnum[body_id] == 6 && m->body_jntadr[body_id] != -1 && m->jnt_type[m->body_jntadr[body_id]] == mjtJoint::mjJNT_FREE)
					{
						if (contact_efforts.count(body_id) == 0)
						{
							contact_efforts[body_id] = (mjtNum *)calloc(6, sizeof(mjtNum));
						}

						send_data_vec.emplace_back(&contact_efforts[body_id][3]);
						send_data_vec.emplace_back(&contact_efforts[body_id][4]);
						send_data_vec.emplace_back(&contact_efforts[body_id][5]);
					}
					else
					{
						printf("%s for %s not supported", attribute_name.c_str(), body_name.c_str());
					}
				}
				else if (strcmp(attribute_name.c_str(), "relative_velocity") == 0)
				{
					if (m->body_dofnum[body_id] == 6 && m->body_jntadr[body_id] != -1 && m->jnt_type[m->body_jntadr[body_id]] == mjtJoint::mjJNT_FREE)
					{
						send_data_vec.emplace_back(&d->qvel[dof_id]);
						send_data_vec.emplace_back(&d->qvel[dof_id + 1]);
						send_data_vec.emplace_back(&d->qvel[dof_id + 2]);
						send_data_vec.emplace_back(&d->qvel[dof_id + 3]);
						send_data_vec.emplace_back(&d->qvel[dof_id + 4]);
						send_data_vec.emplace_back(&d->qvel[dof_id + 5]);
					}
					else
					{
						printf("%s for %s not supported", attribute_name.c_str(), body_name.c_str());
					}
				}
			}
		}
		else if (joint_id != -1)
		{
			const std::string joint_name = send_object.first;
			const int qpos_id = m->jnt_qposadr[joint_id];
			for (const std::string &attribute_name : send_object.second)
			{
				if (strcmp(attribute_name.c_str(), "joint_rvalue") == 0)
				{
					if (m->jnt_type[joint_id] == mjtJoint::mjJNT_HINGE)
					{
						send_data_vec.emplace_back(&d->qpos[qpos_id]);
					}
					else
					{
						printf("%s for %s not supported", attribute_name.c_str(), joint_name.c_str());
					}
				}
				else if (strcmp(attribute_name.c_str(), "joint_tvalue") == 0)
				{
					if (m->jnt_type[joint_id] == mjtJoint::mjJNT_SLIDE)
					{
						send_data_vec.emplace_back(&d->qpos[qpos_id]);
					}
					else
					{
						printf("%s for %s not supported", attribute_name.c_str(), joint_name.c_str());
					}
				}
				else if (strcmp(attribute_name.c_str(), "joint_position") == 0)
				{
					printf("%s for %s not supported", attribute_name.c_str(), joint_name.c_str());
				}
				else if (strcmp(attribute_name.c_str(), "joint_quaternion") == 0)
				{
					if (m->jnt_type[joint_id] == mjtJoint::mjJNT_BALL)
					{
						send_data_vec.emplace_back(&d->qpos[qpos_id]);
						send_data_vec.emplace_back(&d->qpos[qpos_id + 1]);
						send_data_vec.emplace_back(&d->qpos[qpos_id + 2]);
						send_data_vec.emplace_back(&d->qpos[qpos_id + 3]);
					}
					else
					{
						printf("%s for %s not supported", attribute_name.c_str(), joint_name.c_str());
					}
				}
			}
		}
	}

	for (const std::pair<std::string, std::set<std::string>> &receive_object : receive_objects)
	{
		const int body_id = mj_name2id(m, mjtObj::mjOBJ_BODY, receive_object.first.c_str());
		const int joint_id = mj_name2id(m, mjtObj::mjOBJ_JOINT, receive_object.first.c_str());
		if (body_id != -1)
		{
			const std::string body_name = receive_object.first;
			const int body_ref_id = mj_name2id(m, mjtObj::mjOBJ_BODY, (body_name + "_ref").c_str());
			const int mocap_id = m->body_mocapid[body_ref_id];
			const int dof_id = m->body_dofadr[body_id];
			for (const std::string &attribute_name : receive_object.second)
			{
				if (strcmp(attribute_name.c_str(), "position") == 0)
				{
					if (body_ref_id == -1)
					{
						if (m->body_dofnum[body_id] == 6 && m->body_jntadr[body_id] != -1 && m->jnt_type[m->body_jntadr[body_id]] == mjtJoint::mjJNT_FREE)
						{
							int qpos_id = m->jnt_qposadr[m->body_jntadr[body_id]];
							receive_data_vec.emplace_back(&d->qpos[qpos_id]);
							receive_data_vec.emplace_back(&d->qpos[qpos_id + 1]);
							receive_data_vec.emplace_back(&d->qpos[qpos_id + 2]);
						}
						else
						{
							printf("%s for %s not supported", attribute_name.c_str(), body_name.c_str());
						}
					}
					else
					{
						receive_data_vec.emplace_back(&d->mocap_pos[3 * mocap_id]);
						receive_data_vec.emplace_back(&d->mocap_pos[3 * mocap_id + 1]);
						receive_data_vec.emplace_back(&d->mocap_pos[3 * mocap_id + 2]);
					}
				}
				else if (strcmp(attribute_name.c_str(), "quaternion") == 0)
				{
					if (body_ref_id == -1)
					{
						if (m->body_dofnum[body_id] == 6 && m->body_jntadr[body_id] != -1 && m->jnt_type[m->body_jntadr[body_id]] == mjtJoint::mjJNT_FREE)
						{
							int qpos_id = m->jnt_qposadr[m->body_jntadr[body_id]];
							receive_data_vec.emplace_back(&d->qpos[qpos_id + 3]);
							receive_data_vec.emplace_back(&d->qpos[qpos_id + 4]);
							receive_data_vec.emplace_back(&d->qpos[qpos_id + 5]);
							receive_data_vec.emplace_back(&d->qpos[qpos_id + 6]);
						}
						else if (m->body_dofnum[body_id] == 3 && m->body_jntadr[body_id] != -1 && m->jnt_type[m->body_jntadr[body_id]] == mjtJoint::mjJNT_BALL)
						{
							int qpos_id = m->jnt_qposadr[m->body_jntadr[body_id]];
							receive_data_vec.emplace_back(&d->qpos[qpos_id]);
							receive_data_vec.emplace_back(&d->qpos[qpos_id + 1]);
							receive_data_vec.emplace_back(&d->qpos[qpos_id + 2]);
							receive_data_vec.emplace_back(&d->qpos[qpos_id + 3]);
						}
						else
						{
							printf("%s for %s not supported", attribute_name.c_str(), body_name.c_str());
						}
					}
					else
					{
						receive_data_vec.emplace_back(&d->mocap_quat[4 * mocap_id]);
						receive_data_vec.emplace_back(&d->mocap_quat[4 * mocap_id + 1]);
						receive_data_vec.emplace_back(&d->mocap_quat[4 * mocap_id + 2]);
						receive_data_vec.emplace_back(&d->mocap_quat[4 * mocap_id + 3]);
					}
				}
				else if (strcmp(attribute_name.c_str(), "force") == 0)
				{
					receive_data_vec.emplace_back(&d->xfrc_applied[6 * body_id]);
					receive_data_vec.emplace_back(&d->xfrc_applied[6 * body_id + 1]);
					receive_data_vec.emplace_back(&d->xfrc_applied[6 * body_id + 2]);
				}
				else if (strcmp(attribute_name.c_str(), "torque") == 0)
				{
					receive_data_vec.emplace_back(&d->xfrc_applied[6 * body_id + 3]);
					receive_data_vec.emplace_back(&d->xfrc_applied[6 * body_id + 4]);
					receive_data_vec.emplace_back(&d->xfrc_applied[6 * body_id + 5]);
				}
				else if (strcmp(attribute_name.c_str(), "relative_velocity") == 0)
				{
					if (m->body_dofnum[body_id] == 6 && m->body_jntadr[body_id] != -1 && m->jnt_type[m->body_jntadr[body_id]] == mjtJoint::mjJNT_FREE)
					{
						if (odom_velocities.count(body_id) == 0)
						{
							odom_velocities[body_id] = (mjtNum *)calloc(6, sizeof(mjtNum));
							receive_data_vec.emplace_back(&odom_velocities[body_id][0]);
							receive_data_vec.emplace_back(&odom_velocities[body_id][1]);
							receive_data_vec.emplace_back(&odom_velocities[body_id][2]);
							receive_data_vec.emplace_back(&odom_velocities[body_id][3]);
							receive_data_vec.emplace_back(&odom_velocities[body_id][4]);
							receive_data_vec.emplace_back(&odom_velocities[body_id][5]);
						}
					}
					else if (m->body_dofnum[body_id] > 0 && m->body_jntadr[body_id] != 1)
					{
						// receive_data_vec.emplace_back(&MjSim::odom_vels[body_name + "_lin_odom_x_joint"]);
						// receive_data_vec.emplace_back(&MjSim::odom_vels[body_name + "_lin_odom_y_joint"]);
						// receive_data_vec.emplace_back(&MjSim::odom_vels[body_name + "_lin_odom_z_joint"]);
						// receive_data_vec.emplace_back(&MjSim::odom_vels[body_name + "_ang_odom_x_joint"]);
						// receive_data_vec.emplace_back(&MjSim::odom_vels[body_name + "_ang_odom_y_joint"]);
						// receive_data_vec.emplace_back(&MjSim::odom_vels[body_name + "_ang_odom_z_joint"]);
					}
					else
					{
						printf("%s for %s not supported", attribute_name.c_str(), body_name.c_str());
					}
				}
			}
		}
		else if (joint_id != -1)
		{
			const std::string joint_name = receive_object.first;
			const int qpos_id = m->jnt_qposadr[joint_id];
			for (const std::string &attribute_name : receive_object.second)
			{
				if (strcmp(attribute_name.c_str(), "joint_position") == 0)
				{
					printf("%s for %s not supported", attribute_name.c_str(), joint_name.c_str());
				}
				else if (strcmp(attribute_name.c_str(), "joint_quaternion") == 0)
				{
					if (m->jnt_type[joint_id] == mjtJoint::mjJNT_BALL)
					{
						receive_data_vec.emplace_back(&d->qpos[qpos_id]);
						receive_data_vec.emplace_back(&d->qpos[qpos_id + 1]);
						receive_data_vec.emplace_back(&d->qpos[qpos_id + 2]);
						receive_data_vec.emplace_back(&d->qpos[qpos_id + 3]);
					}
					else
					{
						printf("%s for %s not supported", attribute_name.c_str(), joint_name.c_str());
					}
				}
				else if (strcmp(attribute_name.c_str(), "joint_rvalue") == 0)
				{
					if (m->jnt_type[joint_id] == mjtJoint::mjJNT_HINGE)
					{
						receive_data_vec.emplace_back(&d->qpos[qpos_id]);
					}
					else
					{
						printf("%s for %s not supported", attribute_name.c_str(), joint_name.c_str());
					}
				}
				else if (strcmp(attribute_name.c_str(), "joint_tvalue") == 0)
				{
					if (m->jnt_type[joint_id] == mjtJoint::mjJNT_SLIDE)
					{
						receive_data_vec.emplace_back(&d->qpos[qpos_id]);
					}
					else
					{
						printf("%s for %s not supported", attribute_name.c_str(), joint_name.c_str());
					}
				}
			}
		}
	}
	mtx.unlock();
}

void MjMultiverseClient::bind_send_data()
{
	if (send_buffer_size - 1 != send_data_vec.size())
	{
		printf("The size of send_data_vec (%ld) does not match with send_buffer_size - 1 (%ld)", send_data_vec.size(), send_buffer_size);
		return;
	}

	mtx.lock();
	for (std::pair<const int, mjtNum *> &contact_effort : contact_efforts)
	{
		mjtNum jac[6 * m->nv];
		mj_jacBodyCom(m, d, jac, jac + 3 * m->nv, contact_effort.first);
		mju_mulMatVec(contact_effort.second, jac, d->qfrc_constraint, 6, m->nv);
	}

	*send_buffer = get_time_now();

	for (size_t i = 0; i < send_buffer_size - 1; i++)
	{
		send_buffer[i + 1] = *send_data_vec[i];
	}
	mtx.unlock();
}

void MjMultiverseClient::bind_receive_data()
{
	if (receive_buffer_size - 1 != receive_data_vec.size())
	{
		printf("The size of receive_data_vec (%ld) does not match with receive_buffer_size - 1 (%ld)", receive_data_vec.size(), receive_buffer_size - 1);
		return;
	}

	mtx.lock();
	for (std::pair<const int, mjtNum *> &odom_velocity : odom_velocities)
	{
		const int body_id = odom_velocity.first;
		const int dof_adr = m->body_dofadr[body_id];
		if (m->body_dofnum[body_id] == 6)
		{
			const int joint_id = m->body_jntadr[body_id];
			const int qpos_id = m->jnt_qposadr[joint_id];

			const mjtNum w = d->qpos[7 * qpos_id + 3];
			const mjtNum x = d->qpos[7 * qpos_id + 4];
			const mjtNum y = d->qpos[7 * qpos_id + 5];
			const mjtNum z = d->qpos[7 * qpos_id + 6];

			const mjtNum sinr_cosp = 2 * (w * x + y * z);
			const mjtNum cosr_cosp = 1 - 2 * (x * x + y * y);
			mjtNum odom_x_joint_pos = std::atan2(sinr_cosp, cosr_cosp);

			const mjtNum sinp = 2 * (w * y - z * x);
			mjtNum odom_y_joint_pos;
			if (std::abs(sinp) >= 1)
			{
				odom_y_joint_pos = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
			}
			else
			{
				odom_y_joint_pos = std::asin(sinp);
			}

			const mjtNum siny_cosp = 2 * (w * z + x * y);
			const mjtNum cosy_cosp = 1 - 2 * (y * y + z * z);
			mjtNum odom_z_joint_pos= std::atan2(siny_cosp, cosy_cosp);
			
			d->qvel[dof_adr] 	 = odom_velocity.second[0] * mju_cos(odom_y_joint_pos) * mju_cos(odom_z_joint_pos) + odom_velocity.second[1] * (mju_sin(odom_x_joint_pos) * mju_sin(odom_y_joint_pos) * mju_cos(odom_z_joint_pos) - mju_cos(odom_x_joint_pos) * mju_sin(odom_z_joint_pos)) + odom_velocity.second[2] * (mju_cos(odom_x_joint_pos) * mju_sin(odom_y_joint_pos) * mju_cos(odom_z_joint_pos) + mju_sin(odom_x_joint_pos) * mju_sin(odom_z_joint_pos));
			d->qvel[dof_adr + 1] = odom_velocity.second[0] * mju_cos(odom_y_joint_pos) * mju_sin(odom_z_joint_pos) + odom_velocity.second[1] * (mju_sin(odom_x_joint_pos) * mju_sin(odom_y_joint_pos) * mju_sin(odom_z_joint_pos) + mju_cos(odom_x_joint_pos) * mju_cos(odom_z_joint_pos)) + odom_velocity.second[2] * (mju_cos(odom_x_joint_pos) * mju_sin(odom_y_joint_pos) * mju_sin(odom_z_joint_pos) - mju_sin(odom_x_joint_pos) * mju_cos(odom_z_joint_pos));
			d->qvel[dof_adr + 2] = odom_velocity.second[0] * mju_sin(odom_y_joint_pos) + odom_velocity.second[1] * mju_sin(odom_x_joint_pos) * mju_cos(odom_y_joint_pos) + odom_velocity.second[2] * mju_cos(odom_x_joint_pos) * mju_cos(odom_y_joint_pos);
			d->qvel[dof_adr + 3] = odom_velocity.second[3];
			d->qvel[dof_adr + 4] = odom_velocity.second[4];
			d->qvel[dof_adr + 5] = odom_velocity.second[5];
		}
	}

	for (size_t i = 0; i < receive_buffer_size - 1; i++)
	{
		*receive_data_vec[i] = receive_buffer[i + 1];
	}
	mtx.unlock();
}

void MjMultiverseClient::clean_up()
{
	send_data_vec.clear();

	receive_data_vec.clear();

	for (std::pair<const int, mjtNum *> &contact_effort : contact_efforts)
	{
		free(contact_effort.second);
	}
}

void MjMultiverseClient::communicate(const bool resend_meta_data)
{
	MjMultiverseClient::mutex.lock();
	MultiverseClient::communicate(resend_meta_data);
	MjMultiverseClient::mutex.unlock();
}