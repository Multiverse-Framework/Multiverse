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

void MjMultiverseClient::init(const Json::Value &multiverse_params_json)
{
	std::map<std::string, std::string> multiverse_params;

	const Json::Value multiverse_server_params_json = multiverse_params_json["multiverse_server"];
	multiverse_params["server_host"] = multiverse_server_params_json.isMember("host") ? multiverse_server_params_json["host"].asString() : "tcp://127.0.0.1";
	multiverse_params["server_port"] = multiverse_server_params_json.isMember("port") ? multiverse_server_params_json["port"].asString() : "7000";

	const Json::Value multiverse_client_params_json = multiverse_params_json["multiverse_client"];
	multiverse_params["client_host"] = multiverse_client_params_json.isMember("host") ? multiverse_client_params_json["host"].asString() : "tcp://127.0.0.1";
	multiverse_params["client_port"] = multiverse_client_params_json["port"].asString();

	const Json::Value multiverse_client_meta_data_params_json = multiverse_client_params_json["meta_data"];
	multiverse_params["world"] = multiverse_client_meta_data_params_json["world"].asString();
	multiverse_params["name"] = multiverse_client_meta_data_params_json["name"].asString();

	send_objects_json = multiverse_params_json["multiverse_client"]["send"];
	receive_objects_json = multiverse_params_json["multiverse_client"]["receive"];
	world_name = multiverse_params["world"];
	simulation_name = multiverse_params["name"];

	server_socket_addr = multiverse_params["server_host"] + ":" + multiverse_params["server_port"];

	host = multiverse_params["client_host"];
	port = multiverse_params["client_port"];

	connect();
}

bool MjMultiverseClient::spawn_objects(const std::set<std::string> objects)
{
	if (objects.size() == 0)
	{
		return true;
	}

	return true;
}

bool MjMultiverseClient::destroy_objects(const std::set<std::string> objects)
{
	if (objects.size() == 0)
	{
		return true;
	}

	return true;
}

bool MjMultiverseClient::init_objects(bool from_server)
{
	if (from_server && reader.parse(request_meta_data_str, request_meta_data_json))
	{
		receive_objects_json = request_meta_data_json["receive"];
		send_objects_json = request_meta_data_json["send"];
	}

	std::set<std::string> objects_to_spawn;
	for (const std::string &object_name : send_objects_json.getMemberNames())
	{
		if (mj_name2id(m, mjtObj::mjOBJ_BODY, object_name.c_str()) == -1 ||
			mj_name2id(m, mjtObj::mjOBJ_JOINT, object_name.c_str()) == -1)
		{
			objects_to_spawn.insert(object_name);
		}
	}
	for (const std::string &object_name : receive_objects_json.getMemberNames())
	{
		if (mj_name2id(m, mjtObj::mjOBJ_BODY, object_name.c_str()) == -1 ||
			mj_name2id(m, mjtObj::mjOBJ_JOINT, object_name.c_str()) == -1)
		{
			objects_to_spawn.insert(object_name);
		}
	}
	if (!spawn_objects(objects_to_spawn))
	{
		return false;
	}

	std::set<std::string> body_attributes = {"position", "quaternion", "relative_velocity", "force", "torque"};
	std::set<std::string> receive_hinge_joint_attributes = {"cmd_joint_rvalue", "cmd_joint_angular_velocity", "cmd_joint_torque"};
	std::set<std::string> receive_slide_joint_attributes = {"cmd_joint_tvalue", "cmd_joint_linear_velocity", "cmd_joint_force"};

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
			else if (joint_id != -1 && ((receive_hinge_joint_attributes.count(attribute_name) != 0 && m->jnt_type[joint_id] == mjtJoint::mjJNT_HINGE) || (receive_slide_joint_attributes.count(attribute_name) != 0 && m->jnt_type[joint_id] == mjtJoint::mjJNT_SLIDE)))
			{
				receive_objects[object_name].insert(attribute_name);
			}
		}
	}

	std::set<std::string> send_hinge_joint_attributes = {"joint_rvalue", "joint_angular_velocity", "joint_torque"};
	std::set<std::string> send_slide_joint_attributes = {"joint_tvalue", "joint_linear_velocity", "joint_force"};
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
				if (send_hinge_joint_attributes.count(attribute_name) != 0)
				{
					for (int joint_id = 0; joint_id < m->njnt; joint_id++)
					{
						if (m->jnt_type[joint_id] == mjtJoint::mjJNT_HINGE)
						{
							send_objects[mj_id2name(m, mjtObj::mjOBJ_JOINT, joint_id)].insert(attribute_name);
						}
					}
				}
				else if (send_slide_joint_attributes.count(attribute_name) != 0)
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
				else if (joint_id != -1 && ((send_hinge_joint_attributes.count(attribute_name) != 0 && m->jnt_type[joint_id] == mjtJoint::mjJNT_HINGE) || (send_slide_joint_attributes.count(attribute_name) != 0 && m->jnt_type[joint_id] == mjtJoint::mjJNT_SLIDE)))
				{
					send_objects[object_name].insert(attribute_name);
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
	request_meta_data_json["world"] = world_name;
	request_meta_data_json["name"] = simulation_name;
	request_meta_data_json["length_unit"] = "m";
	request_meta_data_json["angle_unit"] = "rad";
	request_meta_data_json["mass_unit"] = "kg";
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
						const Json::Value x_json = response_meta_data_json["send"][send_object.first][attribute_name][0];
						const Json::Value y_json = response_meta_data_json["send"][send_object.first][attribute_name][1];
						const Json::Value z_json = response_meta_data_json["send"][send_object.first][attribute_name][2];
						if (!x_json.isNull() && !y_json.isNull() && !z_json.isNull())
						{
							xpos_desired[0] = x_json.asDouble();
							xpos_desired[1] = y_json.asDouble();
							xpos_desired[2] = z_json.asDouble();
						}
					}
					else if (strcmp(attribute_name.c_str(), "quaternion") == 0)
					{
						const Json::Value w_json = response_meta_data_json["send"][send_object.first][attribute_name][0];
						const Json::Value x_json = response_meta_data_json["send"][send_object.first][attribute_name][1];
						const Json::Value y_json = response_meta_data_json["send"][send_object.first][attribute_name][2];
						const Json::Value z_json = response_meta_data_json["send"][send_object.first][attribute_name][3];
						if (!w_json.isNull() && !x_json.isNull() && !y_json.isNull() && !z_json.isNull())
						{
							xquat_desired[0] = w_json.asDouble();
							xquat_desired[1] = x_json.asDouble();
							xquat_desired[2] = y_json.asDouble();
							xquat_desired[3] = z_json.asDouble();
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
						const Json::Value w_json = response_meta_data_json["send"][send_object.first][attribute_name][0];
						const Json::Value x_json = response_meta_data_json["send"][send_object.first][attribute_name][1];
						const Json::Value y_json = response_meta_data_json["send"][send_object.first][attribute_name][2];
						const Json::Value z_json = response_meta_data_json["send"][send_object.first][attribute_name][3];

						if (!w_json.isNull() && !x_json.isNull() && !y_json.isNull() && !z_json.isNull())
						{
							const mjtNum xquat_desired[4] = {w_json.asDouble(), x_json.asDouble(), y_json.asDouble(), z_json.asDouble()};
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
					const Json::Value v_json = response_meta_data_json["send"][send_object.first][attribute_name][0];
					if (!v_json.isNull())
					{
						const int qpos_id = m->jnt_qposadr[joint_id];
						d->qpos[qpos_id] = v_json.asDouble();
					}
				}
				else if ((strcmp(attribute_name.c_str(), "joint_angular_velocity") == 0 && m->jnt_type[joint_id] == mjtJoint::mjJNT_HINGE) ||
						 (strcmp(attribute_name.c_str(), "joint_linear_velocity") == 0 && m->jnt_type[joint_id] == mjtJoint::mjJNT_SLIDE))
				{
					const Json::Value v_json = response_meta_data_json["send"][send_object.first][attribute_name][0];
					if (!v_json.isNull())
					{
						const int dof_id = m->jnt_dofadr[joint_id];
						d->qvel[dof_id] = v_json.asDouble();
					}
				}
				else if ((strcmp(attribute_name.c_str(), "joint_quaternion") == 0 && m->jnt_type[joint_id] == mjtJoint::mjJNT_BALL))
				{
					const Json::Value w_json = response_meta_data_json["send"][send_object.first][attribute_name][0];
					const Json::Value x_json = response_meta_data_json["send"][send_object.first][attribute_name][1];
					const Json::Value y_json = response_meta_data_json["send"][send_object.first][attribute_name][2];
					const Json::Value z_json = response_meta_data_json["send"][send_object.first][attribute_name][3];

					if (!w_json.isNull() && !x_json.isNull() && !y_json.isNull() && !z_json.isNull())
					{
						const int qpos_id = m->jnt_qposadr[joint_id];
						d->qpos[qpos_id] = w_json.asDouble();
						d->qpos[qpos_id + 1] = x_json.asDouble();
						d->qpos[qpos_id + 2] = y_json.asDouble();
						d->qpos[qpos_id + 3] = z_json.asDouble();
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
	send_data_vec.emplace_back(&d->time);
	for (const std::pair<std::string, std::set<std::string>> &send_object : send_objects)
	{
		const int body_id = mj_name2id(m, mjtObj::mjOBJ_BODY, send_object.first.c_str());
		const int joint_id = mj_name2id(m, mjtObj::mjOBJ_JOINT, send_object.first.c_str());
		if (body_id != -1)
		{
			const std::string body_name = send_object.first;
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
				else if (strcmp(attribute_name.c_str(), "force") == 0 && m->body_dofnum[body_id] == 6 && m->body_jntadr[body_id] != -1 && m->jnt_type[m->body_jntadr[body_id]] == mjtJoint::mjJNT_FREE)
				{
					if (contact_efforts.count(body_id) == 0)
					{
						contact_efforts[body_id] = (mjtNum *)calloc(6, sizeof(mjtNum));
					}

					send_data_vec.emplace_back(&contact_efforts[body_id][0]);
					send_data_vec.emplace_back(&contact_efforts[body_id][1]);
					send_data_vec.emplace_back(&contact_efforts[body_id][2]);
				}
				else if (strcmp(attribute_name.c_str(), "torque") == 0 && m->body_dofnum[body_id] == 6 && m->body_jntadr[body_id] != -1 && m->jnt_type[m->body_jntadr[body_id]] == mjtJoint::mjJNT_FREE)
				{
					if (contact_efforts.count(body_id) == 0)
					{
						contact_efforts[body_id] = (mjtNum *)calloc(6, sizeof(mjtNum));
					}

					send_data_vec.emplace_back(&contact_efforts[body_id][3]);
					send_data_vec.emplace_back(&contact_efforts[body_id][4]);
					send_data_vec.emplace_back(&contact_efforts[body_id][5]);
				}
				else if (strcmp(attribute_name.c_str(), "relative_velocity") == 0 && m->body_dofnum[body_id] == 6 && m->body_jntadr[body_id] != -1 && m->jnt_type[m->body_jntadr[body_id]] == mjtJoint::mjJNT_FREE)
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
					printf("Send %s for %s not supported", attribute_name.c_str(), body_name.c_str());
				}
			}
		}
		else if (joint_id != -1)
		{
			const std::string joint_name = send_object.first;
			const int qpos_id = m->jnt_qposadr[joint_id];
			const int dof_id = m->jnt_dofadr[joint_id];
			for (const std::string &attribute_name : send_object.second)
			{
				if ((strcmp(attribute_name.c_str(), "joint_rvalue") == 0 && m->jnt_type[joint_id] == mjtJoint::mjJNT_HINGE) || (strcmp(attribute_name.c_str(), "joint_tvalue") == 0 && m->jnt_type[joint_id] == mjtJoint::mjJNT_SLIDE))
				{
					send_data_vec.emplace_back(&d->qpos[qpos_id]);
				}
				else if ((strcmp(attribute_name.c_str(), "joint_angular_velocity") == 0 && m->jnt_type[joint_id] == mjtJoint::mjJNT_HINGE) || (strcmp(attribute_name.c_str(), "joint_linear_velocity") == 0 && m->jnt_type[joint_id] == mjtJoint::mjJNT_SLIDE))
				{
					send_data_vec.emplace_back(&d->qvel[dof_id]);
				}
				else if ((strcmp(attribute_name.c_str(), "joint_torque") == 0 && m->jnt_type[joint_id] == mjtJoint::mjJNT_HINGE) || (strcmp(attribute_name.c_str(), "joint_force") == 0 && m->jnt_type[joint_id] == mjtJoint::mjJNT_SLIDE))
				{
					send_data_vec.emplace_back(&d->qfrc_inverse[dof_id]);
				}
				else if (strcmp(attribute_name.c_str(), "joint_position") == 0)
				{
					printf("Send %s for %s not supported yet", attribute_name.c_str(), joint_name.c_str());
				}
				else if (strcmp(attribute_name.c_str(), "joint_quaternion") == 0 && m->jnt_type[joint_id] == mjtJoint::mjJNT_BALL)
				{
					send_data_vec.emplace_back(&d->qpos[qpos_id]);
					send_data_vec.emplace_back(&d->qpos[qpos_id + 1]);
					send_data_vec.emplace_back(&d->qpos[qpos_id + 2]);
					send_data_vec.emplace_back(&d->qpos[qpos_id + 3]);
				}
				else
				{
					printf("Send %s for %s not supported", attribute_name.c_str(), joint_name.c_str());
				}
			}
		}
	}

	receive_data_vec.emplace_back(nullptr);
	for (const std::pair<std::string, std::set<std::string>> &receive_object : receive_objects)
	{
		const int body_id = mj_name2id(m, mjtObj::mjOBJ_BODY, receive_object.first.c_str());
		const int joint_id = mj_name2id(m, mjtObj::mjOBJ_JOINT, receive_object.first.c_str());
		if (body_id != -1)
		{
			const std::string body_name = receive_object.first;
			const int dof_id = m->body_dofadr[body_id];
			for (const std::string &attribute_name : receive_object.second)
			{
				if (strcmp(attribute_name.c_str(), "position") == 0 &&
					m->body_dofnum[body_id] == 6 && m->body_jntadr[body_id] != -1 &&
					m->jnt_type[m->body_jntadr[body_id]] == mjtJoint::mjJNT_FREE)
				{
					int qpos_id = m->jnt_qposadr[m->body_jntadr[body_id]];
					receive_data_vec.emplace_back(&d->qpos[qpos_id]);
					receive_data_vec.emplace_back(&d->qpos[qpos_id + 1]);
					receive_data_vec.emplace_back(&d->qpos[qpos_id + 2]);
				}
				else if (strcmp(attribute_name.c_str(), "quaternion") == 0 &&
							 (m->body_dofnum[body_id] == 6 && m->body_jntadr[body_id] != -1 && m->jnt_type[m->body_jntadr[body_id]] == mjtJoint::mjJNT_FREE) ||
						 (m->body_dofnum[body_id] == 3 && m->body_jntadr[body_id] != -1 && m->jnt_type[m->body_jntadr[body_id]] == mjtJoint::mjJNT_BALL))
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
				else if (strcmp(attribute_name.c_str(), "relative_velocity") == 0 &&
						 m->body_dofnum[body_id] == 6 && m->body_jntadr[body_id] != -1 && m->jnt_type[m->body_jntadr[body_id]] == mjtJoint::mjJNT_FREE &&
						 odom_velocities.count(body_id) == 0)
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
		}
		else if (joint_id != -1)
		{
			const std::string joint_name = receive_object.first;
			const int qpos_id = m->jnt_qposadr[joint_id];
			const int dof_id = m->jnt_dofadr[joint_id];
			for (const std::string &attribute_name : receive_object.second)
			{
				if ((strcmp(attribute_name.c_str(), "cmd_joint_rvalue") == 0 && m->jnt_type[joint_id] == mjtJoint::mjJNT_HINGE) ||
					(strcmp(attribute_name.c_str(), "cmd_joint_tvalue") == 0 && m->jnt_type[joint_id] == mjtJoint::mjJNT_SLIDE))
				{
					receive_data_vec.emplace_back(&d->qpos[qpos_id]);
				}
				else if ((strcmp(attribute_name.c_str(), "cmd_joint_angular_velocity") == 0 && m->jnt_type[joint_id] == mjtJoint::mjJNT_HINGE) ||
						 (strcmp(attribute_name.c_str(), "cmd_joint_linear_velocity") == 0 && m->jnt_type[joint_id] == mjtJoint::mjJNT_SLIDE))
				{
					receive_data_vec.emplace_back(&d->qvel[dof_id]);
				}
				else if ((strcmp(attribute_name.c_str(), "cmd_joint_torque") == 0 && m->jnt_type[joint_id] == mjtJoint::mjJNT_HINGE) ||
						 (strcmp(attribute_name.c_str(), "cmd_joint_force") == 0 && m->jnt_type[joint_id] == mjtJoint::mjJNT_SLIDE))
				{
					receive_data_vec.emplace_back(&d->qfrc_applied[dof_id]);
				}
				else if (strcmp(attribute_name.c_str(), "joint_quaternion") == 0 && m->jnt_type[joint_id] == mjtJoint::mjJNT_BALL)
				{
					receive_data_vec.emplace_back(&d->qpos[qpos_id]);
					receive_data_vec.emplace_back(&d->qpos[qpos_id + 1]);
					receive_data_vec.emplace_back(&d->qpos[qpos_id + 2]);
					receive_data_vec.emplace_back(&d->qpos[qpos_id + 3]);
				}
				else
				{
					printf("Receive %s for %s not supported\n", attribute_name.c_str(), joint_name.c_str());
				}
			}
		}
	}
	mtx.unlock();
}

void MjMultiverseClient::bind_send_data()
{
	if (send_buffer_size != send_data_vec.size())
	{
		printf("The size of send_data_vec (%ld) does not match with send_buffer_size (%ld)", send_data_vec.size(), send_buffer_size);
		return;
	}

	mtx.lock();
	for (std::pair<const int, mjtNum *> &contact_effort : contact_efforts)
	{
		mjtNum jac[6 * m->nv];
		mj_jacBodyCom(m, d, jac, jac + 3 * m->nv, contact_effort.first);
		mju_mulMatVec(contact_effort.second, jac, d->qfrc_constraint, 6, m->nv);
	}

	for (size_t i = 0; i < send_buffer_size; i++)
	{
		send_buffer[i] = *send_data_vec[i];
	}
	mtx.unlock();
}

void MjMultiverseClient::bind_receive_data()
{
	if (receive_buffer_size != receive_data_vec.size())
	{
		printf("[Client %s] The size of receive_data_vec (%ld) does not match with receive_buffer_size (%ld)\n", port.c_str(), receive_data_vec.size(), receive_buffer_size);
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

			const mjtNum w = (d->qpos + qpos_id)[3];
			const mjtNum x = (d->qpos + qpos_id)[4];
			const mjtNum y = (d->qpos + qpos_id)[5];
			const mjtNum z = (d->qpos + qpos_id)[6];

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
			mjtNum odom_z_joint_pos = std::atan2(siny_cosp, cosy_cosp);

			mjtNum *qvel = d->qvel + dof_adr;
			qvel[0] = odom_velocity.second[0] * mju_cos(odom_y_joint_pos) * mju_cos(odom_z_joint_pos) + odom_velocity.second[1] * (mju_sin(odom_x_joint_pos) * mju_sin(odom_y_joint_pos) * mju_cos(odom_z_joint_pos) - mju_cos(odom_x_joint_pos) * mju_sin(odom_z_joint_pos)) + odom_velocity.second[2] * (mju_cos(odom_x_joint_pos) * mju_sin(odom_y_joint_pos) * mju_cos(odom_z_joint_pos) + mju_sin(odom_x_joint_pos) * mju_sin(odom_z_joint_pos));
			qvel[1] = odom_velocity.second[0] * mju_cos(odom_y_joint_pos) * mju_sin(odom_z_joint_pos) + odom_velocity.second[1] * (mju_sin(odom_x_joint_pos) * mju_sin(odom_y_joint_pos) * mju_sin(odom_z_joint_pos) + mju_cos(odom_x_joint_pos) * mju_cos(odom_z_joint_pos)) + odom_velocity.second[2] * (mju_cos(odom_x_joint_pos) * mju_sin(odom_y_joint_pos) * mju_sin(odom_z_joint_pos) - mju_sin(odom_x_joint_pos) * mju_cos(odom_z_joint_pos));
			qvel[2] = odom_velocity.second[0] * mju_sin(odom_y_joint_pos) + odom_velocity.second[1] * mju_sin(odom_x_joint_pos) * mju_cos(odom_y_joint_pos) + odom_velocity.second[2] * mju_cos(odom_x_joint_pos) * mju_cos(odom_y_joint_pos);
			qvel[3] = odom_velocity.second[3];
			qvel[4] = odom_velocity.second[4];
			qvel[5] = odom_velocity.second[5];
		}
	}

	for (size_t i = 1; i < receive_buffer_size; i++)
	{
		*receive_data_vec[i] = receive_buffer[i];
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
	contact_efforts.clear();
	for (std::pair<const int, mjtNum *> &odom_velocity : odom_velocities)
	{
		free(odom_velocity.second);
	}
	odom_velocities.clear();
}

void MjMultiverseClient::communicate(const bool resend_meta_data)
{
	MjMultiverseClient::mutex.lock();
	MultiverseClient::communicate(resend_meta_data);
	MjMultiverseClient::mutex.unlock();
}