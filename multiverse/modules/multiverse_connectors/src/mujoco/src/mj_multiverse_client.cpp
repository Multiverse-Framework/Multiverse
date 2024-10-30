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

#define _USE_MATH_DEFINES
#include <cmath>

#include "mj_multiverse_client.h"
#include "mj_simulate.h"

#include <chrono>
#include <csignal>
#include <iostream>
#include <tinyxml2.h>
#include <sstream>
#include <algorithm>

std::mutex MjMultiverseClient::mutex;

bool MjMultiverseClient::pause = false;

bool contains_tag(const boost::filesystem::path &file_path, const std::string &model_name)
{
	tinyxml2::XMLDocument doc;

	if (doc.LoadFile(file_path.c_str()) == tinyxml2::XML_SUCCESS)
	{
		for (const tinyxml2::XMLElement *mujoco_element = doc.FirstChildElement("mujoco");
			 mujoco_element != nullptr;
			 mujoco_element = mujoco_element->NextSiblingElement("mujoco"))
		{
			const tinyxml2::XMLAttribute *model_attribute = mujoco_element->FindAttribute("model");
			if (model_attribute != nullptr && strcmp(model_attribute->Value(), model_name.c_str()) == 0)
			{
				return true;
			}
		}
	}
	else
	{
		printf("Could not load file: %s\n", file_path.c_str());
	}
	return false;
}

// Function to recursively find XML files containing the specific tag within the specified directories
std::vector<boost::filesystem::path> find_object_xml_paths(const std::set<std::string> &directories, const std::string &model_name)
{
	std::vector<boost::filesystem::path> matching_files;

	for (const std::string &dir : directories)
	{
		if (boost::filesystem::exists(dir) && boost::filesystem::is_directory(dir))
		{
			for (const auto &entry : boost::filesystem::recursive_directory_iterator(dir))
			{
				if (boost::filesystem::is_regular_file(entry) && entry.path().extension() == ".xml")
				{
					if (contains_tag(entry.path(), model_name))
					{
						matching_files.push_back(entry.path());
					}
				}
			}
		}
	}

	return matching_files;
}

void MjMultiverseClient::init(const Json::Value &multiverse_params_json)
{
	std::map<std::string, std::string> multiverse_params;

	const Json::Value &multiverse_server_params_json = multiverse_params_json["multiverse_server"];
	multiverse_params["server_host"] = multiverse_server_params_json.isMember("host") ? multiverse_server_params_json["host"].asString() : "tcp://127.0.0.1";
	multiverse_params["server_port"] = multiverse_server_params_json.isMember("port") ? multiverse_server_params_json["port"].asString() : "7000";

	const Json::Value &multiverse_client_params_json = multiverse_params_json["multiverse_client"];
	multiverse_params["client_port"] = multiverse_client_params_json["port"].asString();

	const Json::Value &multiverse_client_meta_data_params_json = multiverse_client_params_json["meta_data"];
	multiverse_params["world_name"] = multiverse_client_meta_data_params_json["world_name"].asString();
	multiverse_params["simulation_name"] = multiverse_client_meta_data_params_json["simulation_name"].asString();

	for (const Json::Value &resource : multiverse_client_params_json["resources"])
	{
		resources.insert(resource.asString());
	}

	send_objects_json = multiverse_params_json["multiverse_client"]["send"];
	receive_objects_json = multiverse_params_json["multiverse_client"]["receive"];
	world_name = multiverse_params["world_name"];
	simulation_name = multiverse_params["simulation_name"];

	server_socket_addr = multiverse_params["server_host"] + ":" + multiverse_params["server_port"];

	host = multiverse_params["server_host"];
	port = multiverse_params["client_port"];

	*world_time = d->time;

	connect();
}

bool MjMultiverseClient::spawn_objects(std::set<std::string> &object_names)
{
	boost::filesystem::path scene_xml_folder = scene_xml_path.parent_path();
	for (const std::string &object_name : object_names)
	{
		const std::vector<boost::filesystem::path> &object_xml_paths = find_object_xml_paths(resources, object_name);
		if (!object_xml_paths.empty())
		{
			const boost::filesystem::path object_xml_path = object_xml_paths[0];
			printf("Found XML file of [%s] at [%s].\n", object_name.c_str(), object_xml_path.c_str());
			printf("Trying to spawn object [%s] to the scene [%s].\n", object_name.c_str(), scene_xml_path.c_str());

			boost::filesystem::path new_object_xml_path = scene_xml_folder / object_xml_path.filename();
			{
#ifdef _WIN32
				boost::filesystem::copy_file(object_xml_path, new_object_xml_path, boost::filesystem::copy_options::overwrite_existing);
#else
				boost::filesystem::copy_file(object_xml_path, new_object_xml_path, boost::filesystem::copy_option::overwrite_if_exists);
#endif
				tinyxml2::XMLDocument doc;
				if (doc.LoadFile(new_object_xml_path.c_str()) == tinyxml2::XML_SUCCESS)
				{
					tinyxml2::XMLElement *mujoco_element = doc.FirstChildElement("mujoco");
					for (const std::string &asset_type : {"mesh", "texture"})
					{
						boost::filesystem::path asset_dir_path = object_xml_path;
						const char *asset_type_dir = (asset_type + "dir").c_str();
						for (tinyxml2::XMLElement *compiler_element = mujoco_element->FirstChildElement("compiler");
							 compiler_element != nullptr; compiler_element = compiler_element->NextSiblingElement("compiler"))
						{
							if (compiler_element->Attribute(asset_type_dir) == nullptr)
							{
								continue;
							}
							asset_dir_path = compiler_element->Attribute(asset_type_dir);
							if (asset_dir_path.is_relative())
							{
								asset_dir_path = object_xml_path.parent_path() / asset_dir_path;
							}
							compiler_element->DeleteAttribute(asset_type_dir);
						}
						for (tinyxml2::XMLElement *asset_element = mujoco_element->FirstChildElement("asset");
							 asset_element != nullptr; asset_element = asset_element->NextSiblingElement("asset"))
						{
							for (tinyxml2::XMLElement *asset_type_element = asset_element->FirstChildElement(asset_type.c_str());
								 asset_type_element != nullptr; asset_type_element = asset_type_element->NextSiblingElement(asset_type.c_str()))
							{
								if (asset_type_element->Attribute("file") == nullptr)
								{
									continue;
								}
								boost::filesystem::path asset_file_path = asset_type_element->Attribute("file");
								if (!asset_file_path.empty() && asset_file_path.is_relative())
								{
									asset_file_path = asset_dir_path / asset_file_path;
									asset_type_element->SetAttribute("file", asset_file_path.c_str());
								}
							}
						}

						std::string object_pos = "";
						if (!response_meta_data_json["send"][object_name]["position"].empty())
						{
							for (const Json::Value &pos : response_meta_data_json["send"][object_name]["position"])
							{
								object_pos += (pos.isNumeric() ? pos.asString() + " " : "");
							}
							if (!object_pos.empty())
							{
								object_pos.pop_back();
							}
						}
						else if (!response_meta_data_json["receive"][object_name]["position"].empty())
						{
							for (const Json::Value &pos : response_meta_data_json["receive"][object_name]["position"])
							{
								object_pos += (pos.isNumeric() ? pos.asString() + " " : "");
							}
							if (!object_pos.empty())
							{
								object_pos.pop_back();
							}
						}
						if (object_pos.empty())
						{
							object_pos = "0.0 0.0 0.0";
						}

						std::string object_quat = "";
						if (!response_meta_data_json["send"][object_name]["quaternion"].empty())
						{
							for (const Json::Value &quat : response_meta_data_json["send"][object_name]["quaternion"])
							{
								object_quat += (quat.isNumeric() ? quat.asString() + " " : "");
							}
							if (!object_quat.empty())
							{
								object_quat.pop_back();
							}
						}
						else if (!response_meta_data_json["receive"][object_name]["quaternion"].empty())
						{
							for (const Json::Value &quat : response_meta_data_json["receive"][object_name]["quaternion"])
							{
								object_quat += (quat.isNumeric() ? quat.asString() + " " : "");
							}
							if (!object_quat.empty())
							{
								object_quat.pop_back();
							}
						}
						if (object_quat.empty())
						{
							object_quat = "1.0 0.0 0.0 0.0";
						}

						for (tinyxml2::XMLElement *worldbody_element = mujoco_element->FirstChildElement("worldbody");
							 worldbody_element != nullptr; worldbody_element = worldbody_element->NextSiblingElement("worldbody"))
						{
							for (tinyxml2::XMLElement *body_element = worldbody_element->FirstChildElement("body");
								 body_element != nullptr; body_element = body_element->NextSiblingElement("body"))
							{
								if (strcmp(body_element->Attribute("name"), object_name.c_str()) == 0)
								{
									body_element->SetAttribute("pos", object_pos.c_str());
									body_element->SetAttribute("quat", object_quat.c_str());
									break;
								}
							}
						}

						for (tinyxml2::XMLElement *keyframe_element = mujoco_element->FirstChildElement("keyframe");
							 keyframe_element != nullptr; keyframe_element = keyframe_element->NextSiblingElement("keyframe"))
						{
							keyframe_element->DeleteChildren();
						}
					}

					doc.SaveFile(new_object_xml_path.c_str());
				}
				else
				{
					printf("Could not load file: %s\n", new_object_xml_path.c_str());
					return false;
				}
			}

			tinyxml2::XMLDocument doc;
			if (doc.LoadFile(scene_xml_path.c_str()) == tinyxml2::XML_SUCCESS)
			{
				tinyxml2::XMLElement *mujoco_element = doc.FirstChildElement("mujoco");
				tinyxml2::XMLElement *include_element = doc.NewElement("include");
				mujoco_element->InsertEndChild(include_element);

				include_element->SetAttribute("file", object_xml_path.filename().c_str());
				doc.SaveFile(scene_xml_path.c_str());
			}
			else
			{
				printf("Could not load file: %s\n", scene_xml_path.c_str());
				return false;
			}

			printf("Spawned object [%s] to the scene [%s].\n", object_name.c_str(), scene_xml_path.c_str());
		}
		else
		{
			printf("No XML files found with the tag <mujoco model=\"%s\">\n", object_name.c_str());
			return false;
		}
	}
	object_names.clear();
	return true;
}

bool MjMultiverseClient::destroy_objects(std::set<std::string> &object_names)
{
	tinyxml2::XMLDocument doc;
	if (doc.LoadFile(scene_xml_path.c_str()) == tinyxml2::XML_SUCCESS)
	{
		tinyxml2::XMLElement *mujoco_element = doc.FirstChildElement("mujoco");
		for (tinyxml2::XMLElement *worldbody_element = mujoco_element->FirstChildElement("worldbody");
			 worldbody_element != nullptr; worldbody_element = worldbody_element->NextSiblingElement("worldbody"))
		{
			for (tinyxml2::XMLElement *body_element = worldbody_element->FirstChildElement("body");
				 body_element != nullptr; body_element = body_element->NextSiblingElement("body"))
			{
				if (std::find(object_names.begin(), object_names.end(), body_element->Attribute("name")) != object_names.end())
				{
					worldbody_element->DeleteChild(body_element);
				}
			}
		}
		for (const std::string &object_name : std::set<std::string>(object_names))
		{
			printf("Trying to remove object [%s] from the scene [%s].\n", object_name.c_str(), scene_xml_path.c_str());
			for (tinyxml2::XMLElement *include_element = mujoco_element->FirstChildElement("include");
				 include_element != nullptr; include_element = include_element->NextSiblingElement("include"))
			{
				boost::filesystem::path include_file_path = include_element->Attribute("file");
				if (include_file_path.is_relative())
				{
					include_file_path = scene_xml_path.parent_path() / include_file_path;
				}

				if (contains_tag(include_file_path, object_name))
				{
					mujoco_element->DeleteChild(include_element);

					for (tinyxml2::XMLElement *equality_element = mujoco_element->FirstChildElement("equality");
						 equality_element != nullptr; equality_element = equality_element->NextSiblingElement("equality"))
					{
						std::vector<tinyxml2::XMLElement *> weld_elements_to_delete;
						for (tinyxml2::XMLElement *weld_element = equality_element->FirstChildElement("weld");
							 weld_element != nullptr; weld_element = weld_element->NextSiblingElement("weld"))
						{
							if (strcmp(weld_element->Attribute("body1"), object_name.c_str()) == 0 ||
								strcmp(weld_element->Attribute("body2"), object_name.c_str()) == 0)
							{
								weld_elements_to_delete.push_back(weld_element);
							}
						}
						for (tinyxml2::XMLElement *weld_element : weld_elements_to_delete)
						{
							equality_element->DeleteChild(weld_element);
						}
					}

					printf("Removed object [%s] from the scene [%s].\n", object_name.c_str(), scene_xml_path.c_str());

					if (cached_objects.find(include_file_path.string()) == cached_objects.end())
					{
						cached_objects[include_file_path.string()] = {};
						char error[1000] = "Could not load binary model";
						mjModel *m_include = mj_loadXML(include_file_path.c_str(), nullptr, error, 1000);
						printf("Trying to load include model [%s].\n", include_file_path.c_str());
						if (!m_include)
						{
							mju_error("Load include model error: %s", error);
						}
						for (int body_id = 1; body_id < m_include->nbody; body_id++)
						{
							const std::string body_name = mj_id2name(m_include, mjtObj::mjOBJ_BODY, body_id);
							cached_objects[include_file_path.string()].insert(body_name);
						}
						for (int joint_id = 0; joint_id < m_include->njnt; joint_id++)
						{
							if (mj_id2name(m_include, mjtObj::mjOBJ_JOINT, joint_id))
							{
								const std::string joint_name = mj_id2name(m_include, mjtObj::mjOBJ_JOINT, joint_id);
								cached_objects[include_file_path.string()].insert(joint_name);
							}
						}
					}
					for (const std::string &object_name : cached_objects[include_file_path.string()])
					{
						if (send_objects_json.isMember(object_name))
						{
							send_objects_json.removeMember(object_name);
							printf("Removed object [%s] from the request meta data.\n", object_name.c_str());
						}
					}
				}
				else
				{
					tinyxml2::XMLDocument include_doc;
					include_doc.LoadFile(include_file_path.c_str());
					tinyxml2::XMLElement *include_mujoco_element = include_doc.FirstChildElement("mujoco");
					for (tinyxml2::XMLElement *worldbody_element = include_mujoco_element->FirstChildElement("worldbody");
						 worldbody_element != nullptr; worldbody_element = worldbody_element->NextSiblingElement("worldbody"))
					{
						for (tinyxml2::XMLElement *body_element = worldbody_element->FirstChildElement("body");
							 body_element != nullptr; body_element = body_element->NextSiblingElement("body"))
						{
							if (std::find(object_names.begin(), object_names.end(), body_element->Attribute("name")) != object_names.end())
							{
								worldbody_element->DeleteChild(body_element);
							}
						}
					}
				}
			}
		}
		doc.SaveFile(scene_xml_path.c_str());
	}
	else
	{
		printf("Could not load file: %s\n", scene_xml_path.c_str());
		return false;
	}

	object_names.clear();
	return true;
}

bool MjMultiverseClient::init_objects(bool from_request_meta_data)
{
	if (from_request_meta_data)
	{
		if (request_meta_data_json["receive"].empty())
		{
			receive_objects_json.clear();
		}
		if (request_meta_data_json["send"].empty())
		{
			send_objects_json.clear();
		}
		for (const std::string &object_name : request_meta_data_json["receive"].getMemberNames())
		{
			receive_objects_json[object_name] = request_meta_data_json["receive"][object_name];
		}
		for (const std::string &object_name : request_meta_data_json["send"].getMemberNames())
		{
			send_objects_json[object_name] = request_meta_data_json["send"][object_name];
		}
	}

	std::set<std::string> objects_to_spawn;
	std::set<std::string> objects_to_destroy;
	for (const std::string &object_name : send_objects_json.getMemberNames())
	{
		if (strcmp(object_name.c_str(), "body") == 0 || strcmp(object_name.c_str(), "joint") == 0)
		{
			continue;
		}
		bool stop = !send_objects_json[object_name].empty();
		for (const Json::Value &attribute_json : send_objects_json[object_name])
		{
			const std::string attribute_name = attribute_json.asString();
			if (strcmp(attribute_name.c_str(), "position") == 0 || strcmp(attribute_name.c_str(), "quaternion") == 0)
			{
				stop = false;
			}
		}
		if (stop)
		{
			continue;
		}

		if (mj_name2id(m, mjtObj::mjOBJ_BODY, object_name.c_str()) == -1 &&
			mj_name2id(m, mjtObj::mjOBJ_JOINT, object_name.c_str()) == -1 &&
			!(receive_objects_json.isMember(object_name) &&
			  receive_objects_json[object_name].empty()))
		{
			objects_to_spawn.insert(object_name);
		}
		else if (send_objects_json[object_name].empty() &&
				 receive_objects_json.isMember(object_name) &&
				 receive_objects_json[object_name].empty())
		{
			objects_to_destroy.insert(object_name);
		}
	}
	for (const std::string &object_name : receive_objects_json.getMemberNames())
	{
		if (strcmp(object_name.c_str(), "body") == 0 || strcmp(object_name.c_str(), "joint") == 0)
		{
			continue;
		}
		bool stop = true;
		for (const Json::Value &attribute_json : receive_objects_json[object_name])
		{
			const std::string attribute_name = attribute_json.asString();
			if (strcmp(attribute_name.c_str(), "position") == 0 || strcmp(attribute_name.c_str(), "quaternion") == 0)
			{
				stop = false;
			}
		}
		if (stop)
		{
			continue;
		}

		if (mj_name2id(m, mjtObj::mjOBJ_BODY, object_name.c_str()) == -1 &&
			mj_name2id(m, mjtObj::mjOBJ_JOINT, object_name.c_str()) == -1 &&
			mj_name2id(m, mjtObj::mjOBJ_ACTUATOR, object_name.c_str()) == -1 &&
			!(send_objects_json.isMember(object_name) &&
			  send_objects_json[object_name].empty()))
		{
			objects_to_spawn.insert(object_name);
		}
	}
	for (const std::string &object_name : objects_to_destroy)
	{
		receive_objects_json.removeMember(object_name);
		send_objects_json.removeMember(object_name);
	}

	if (objects_to_spawn.size() > 0 || objects_to_destroy.size() > 0)
	{
		tinyxml2::XMLDocument doc;
		if (doc.LoadFile(scene_xml_path.c_str()) == tinyxml2::XML_SUCCESS)
		{
			tinyxml2::XMLElement *mujoco_element = doc.FirstChildElement("mujoco");
			for (tinyxml2::XMLElement *keyframe_element = mujoco_element->FirstChildElement("keyframe");
				 keyframe_element != nullptr; keyframe_element = keyframe_element->NextSiblingElement("keyframe"))
			{
				for (tinyxml2::XMLElement *key_element = keyframe_element->FirstChildElement("key");
					 key_element != nullptr; key_element = key_element->NextSiblingElement("key"))
				{
					key_element->DeleteAttribute("qpos");
					key_element->DeleteAttribute("qvel");
					key_element->DeleteAttribute("act");
					key_element->DeleteAttribute("ctrl");
					key_element->DeleteAttribute("mpos");
					key_element->DeleteAttribute("mquat");
				}
			}
			doc.SaveFile(scene_xml_path.c_str());
		}
		else
		{
			printf("Could not load file: %s\n", scene_xml_path.c_str());
			return false;
		}

		if (!spawn_objects(objects_to_spawn) || !destroy_objects(objects_to_destroy))
		{
			return false;
		}

		MjSimulate::load_new_model_and_keep_old_data();
	}

	std::set<std::string> body_attributes = {"position", "quaternion", "odometric_velocity", "relative_velocity", "force", "torque"};
	std::set<std::string> receive_hinge_joint_attributes = {"cmd_joint_rvalue", "cmd_joint_angular_velocity", "cmd_joint_torque"};
	std::set<std::string> receive_slide_joint_attributes = {"cmd_joint_tvalue", "cmd_joint_linear_velocity", "cmd_joint_force"};

	receive_objects.clear();
	for (const std::string &object_name : receive_objects_json.getMemberNames())
	{
		receive_objects[object_name] = {};
		for (const Json::Value &attribute_json : receive_objects_json[object_name])
		{
			const std::string attribute_name = attribute_json.asString();
			const int body_id = mj_name2id(m, mjtObj::mjOBJ_BODY, object_name.c_str());
			const int joint_id = mj_name2id(m, mjtObj::mjOBJ_JOINT, object_name.c_str());
			const int actuator_id = mj_name2id(m, mjtObj::mjOBJ_ACTUATOR, object_name.c_str());
			if (body_attributes.count(attribute_name) != 0 && body_id != -1)
			{
				receive_objects[object_name].insert(attribute_name);
			}
			else if (joint_id != -1 && ((receive_hinge_joint_attributes.count(attribute_name) != 0 && m->jnt_type[joint_id] == mjtJoint::mjJNT_HINGE) || (receive_slide_joint_attributes.count(attribute_name) != 0 && m->jnt_type[joint_id] == mjtJoint::mjJNT_SLIDE)))
			{
				receive_objects[object_name].insert(attribute_name);
			}
			else if (actuator_id != -1)
			{
				receive_objects[object_name].insert(attribute_name);
			}
		}
	}

	std::set<std::string> send_hinge_joint_attributes = {"joint_rvalue", "joint_angular_velocity", "joint_torque"};
	std::set<std::string> send_slide_joint_attributes = {"joint_tvalue", "joint_linear_velocity", "joint_force"};

	send_objects.clear();
	for (const std::string &object_name : send_objects_json.getMemberNames())
	{
		for (const Json::Value &attribute_json : send_objects_json[object_name])
		{
			const std::string attribute_name = attribute_json.asString();
			if (strcmp(object_name.c_str(), "body") == 0 || strcmp(object_name.c_str(), "joint") == 0)
			{
				continue;
			}
			else
			{
				if ((receive_objects.find(object_name) != receive_objects.end()) && (receive_objects[object_name].find(attribute_name) != receive_objects[object_name].end()))
				{
					continue;
				}
				const int body_id = mj_name2id(m, mjtObj::mjOBJ_BODY, object_name.c_str());
				const int joint_id = mj_name2id(m, mjtObj::mjOBJ_JOINT, object_name.c_str());
				if (body_attributes.count(attribute_name) != 0 && body_id != -1)
				{
					if ((m->body_dofnum[body_id] == 6 &&
						 m->body_jntadr[body_id] != -1 &&
						 m->jnt_type[m->body_jntadr[body_id]] == mjtJoint::mjJNT_FREE) ||
						strcmp(attribute_name.c_str(), "relative_velocity") != 0)
					{
						send_objects[object_name].insert(attribute_name);
					}
				}
				else if (joint_id != -1 &&
						 ((send_hinge_joint_attributes.count(attribute_name) != 0 && m->jnt_type[joint_id] == mjtJoint::mjJNT_HINGE) ||
						  (send_slide_joint_attributes.count(attribute_name) != 0 && m->jnt_type[joint_id] == mjtJoint::mjJNT_SLIDE)))
				{
					send_objects[object_name].insert(attribute_name);
				}
			}
		}
	}

	for (const std::string &object_name : send_objects_json.getMemberNames())
	{
		if (strcmp(object_name.c_str(), "body") == 0)
		{
			for (int body_id = 1; body_id < m->nbody; body_id++)
			{
				if (mj_id2name(m, mjtObj::mjOBJ_BODY, body_id) == nullptr)
				{
					continue;
				}
				const std::string body_name = mj_id2name(m, mjtObj::mjOBJ_BODY, body_id);
				if (send_objects.find(body_name) != send_objects.end())
				{
					continue;
				}
				for (const Json::Value &attribute_json : send_objects_json[object_name])
				{
					const std::string attribute_name = attribute_json.asString();
					if (body_attributes.count(attribute_name) != 0)
					{
						if ((receive_objects.find(body_name) != receive_objects.end()) && (receive_objects[body_name].find(attribute_name) != receive_objects[body_name].end()))
						{
							continue;
						}
						if (strcmp(attribute_name.c_str(), "relative_velocity") == 0)
						{
							if ((m->body_dofnum[body_id] == 6 &&
								 m->body_jntadr[body_id] != -1 &&
								 m->jnt_type[m->body_jntadr[body_id]] == mjtJoint::mjJNT_FREE))
							{
								send_objects[body_name].insert(attribute_name);
							}
						}
						else if (strcmp(attribute_name.c_str(), "position") == 0 ||
								 strcmp(attribute_name.c_str(), "quaternion") == 0)
						{
							send_objects[body_name].insert(attribute_name);
						}
					}
				}
			}
		}
		else if (strcmp(object_name.c_str(), "joint") == 0)
		{
			for (int joint_id = 0; joint_id < m->njnt; joint_id++)
			{
				if (mj_id2name(m, mjtObj::mjOBJ_JOINT, joint_id) == nullptr)
				{
					continue;
				}
				const std::string joint_name = mj_id2name(m, mjtObj::mjOBJ_JOINT, joint_id);
				if (send_objects.find(joint_name) != send_objects.end())
				{
					continue;
				}
				for (const Json::Value &attribute_json : send_objects_json[object_name])
				{
					const std::string attribute_name = attribute_json.asString();
					if (send_hinge_joint_attributes.count(attribute_name) != 0)
					{
						if (m->jnt_type[joint_id] == mjtJoint::mjJNT_HINGE)
						{
							if ((receive_objects.find(joint_name) != receive_objects.end()) && (receive_objects[joint_name].find(attribute_name) != receive_objects[joint_name].end()))
							{
								continue;
							}
							send_objects[joint_name].insert(attribute_name);
						}
					}
					else if (send_slide_joint_attributes.count(attribute_name) != 0)
					{
						for (int joint_id = 0; joint_id < m->njnt; joint_id++)
						{
							if (m->jnt_type[joint_id] == mjtJoint::mjJNT_SLIDE)
							{
								const std::string joint_name = mj_id2name(m, mjtObj::mjOBJ_JOINT, joint_id);
								if ((receive_objects.find(joint_name) != receive_objects.end()) && (receive_objects[joint_name].find(attribute_name) != receive_objects[joint_name].end()))
								{
									continue;
								}
								send_objects[joint_name].insert(attribute_name);
							}
						}
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
	const Json::Value api_callbacks = request_meta_data_json["api_callbacks"];
	const Json::Value api_callbacks_response = request_meta_data_json["api_callbacks_response"];

	request_meta_data_json.clear();

	if (!api_callbacks.isNull())
	{
		request_meta_data_json["api_callbacks"] = api_callbacks;
	}
	if (!api_callbacks_response.isNull())
	{
		request_meta_data_json["api_callbacks_response"] = api_callbacks_response;
	}

	request_meta_data_json["meta_data"]["world_name"] = world_name;
	request_meta_data_json["meta_data"]["simulation_name"] = simulation_name;
	request_meta_data_json["meta_data"]["length_unit"] = "m";
	request_meta_data_json["meta_data"]["angle_unit"] = "rad";
	request_meta_data_json["meta_data"]["mass_unit"] = "kg";
	request_meta_data_json["meta_data"]["time_unit"] = "s";
	request_meta_data_json["meta_data"]["handedness"] = "rhs";

	for (const std::pair<std::string, std::set<std::string>> &send_object : send_objects)
	{
		const int body_id = mj_name2id(m, mjtObj::mjOBJ_BODY, send_object.first.c_str());
		const int joint_id = mj_name2id(m, mjtObj::mjOBJ_JOINT, send_object.first.c_str());
		const int actuator_id = mj_name2id(m, mjtObj::mjOBJ_ACTUATOR, send_object.first.c_str());
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
		else if (actuator_id != -1)
		{
			const std::string actuator_name = send_object.first;
			for (const std::string &attribute_name : send_object.second)
			{
				request_meta_data_json["send"][actuator_name].append(attribute_name);
			}
		}
	}

	for (const std::pair<std::string, std::set<std::string>> &receive_object : receive_objects)
	{
		const int body_id = mj_name2id(m, mjtObj::mjOBJ_BODY, receive_object.first.c_str());
		const int joint_id = mj_name2id(m, mjtObj::mjOBJ_JOINT, receive_object.first.c_str());
		const int actuator_id = mj_name2id(m, mjtObj::mjOBJ_ACTUATOR, receive_object.first.c_str());
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
		else if (actuator_id != -1)
		{
			const std::string actuator_name = receive_object.first;
			for (const std::string &attribute_name : receive_object.second)
			{
				request_meta_data_json["receive"][actuator_name].append(attribute_name);
			}
		}
	}

	request_meta_data_str = request_meta_data_json.toStyledString();
}

void MjMultiverseClient::bind_response_meta_data()
{
	for (const std::pair<std::string, std::set<std::string>> &send_object : send_objects)
	{
		const int body_id = mj_name2id(m, mjtObj::mjOBJ_BODY, send_object.first.c_str());
		const int joint_id = mj_name2id(m, mjtObj::mjOBJ_JOINT, send_object.first.c_str());
		const int mocap_id = m->body_mocapid[body_id];
		const int actuator_id = mj_name2id(m, mjtObj::mjOBJ_ACTUATOR, send_object.first.c_str());
		if (body_id != -1)
		{
			if (mocap_id != -1)
			{
				for (const std::string &attribute_name : send_object.second)
				{
					if (strcmp(attribute_name.c_str(), "position") == 0)
					{
						const Json::Value x_json = response_meta_data_json["send"][send_object.first][attribute_name][0];
						const Json::Value y_json = response_meta_data_json["send"][send_object.first][attribute_name][1];
						const Json::Value z_json = response_meta_data_json["send"][send_object.first][attribute_name][2];
						if (!x_json.isNull() && !y_json.isNull() && !z_json.isNull())
						{
							d->mocap_pos[3 * mocap_id] = x_json.asDouble();
							d->mocap_pos[3 * mocap_id + 1] = y_json.asDouble();
							d->mocap_pos[3 * mocap_id + 2] = z_json.asDouble();
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
							d->mocap_quat[4 * mocap_id] = w_json.asDouble();
							d->mocap_quat[4 * mocap_id + 1] = x_json.asDouble();
							d->mocap_quat[4 * mocap_id + 2] = y_json.asDouble();
							d->mocap_quat[4 * mocap_id + 3] = z_json.asDouble();
						}
					}
				}
			}
			else if (m->body_dofnum[body_id] == 6 &&
					 m->body_jntadr[body_id] != -1 &&
					 m->jnt_type[m->body_jntadr[body_id]] == mjtJoint::mjJNT_FREE)
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
					else if (strcmp(attribute_name.c_str(), "relative_velocity") == 0)
					{
						const Json::Value qvel_lin_x = response_meta_data_json["send"][send_object.first][attribute_name][0];
						const Json::Value qvel_lin_y = response_meta_data_json["send"][send_object.first][attribute_name][1];
						const Json::Value qvel_lin_z = response_meta_data_json["send"][send_object.first][attribute_name][2];
						const Json::Value qvel_ang_x = response_meta_data_json["send"][send_object.first][attribute_name][3];
						const Json::Value qvel_ang_y = response_meta_data_json["send"][send_object.first][attribute_name][4];
						const Json::Value qvel_ang_z = response_meta_data_json["send"][send_object.first][attribute_name][5];
						if (!qvel_lin_x.isNull() && !qvel_lin_y.isNull() && !qvel_lin_z.isNull() && !qvel_ang_x.isNull() && !qvel_ang_y.isNull() && !qvel_ang_z.isNull())
						{
							const int qvel_adr = m->body_dofadr[body_id];
							d->qvel[qvel_adr] = qvel_lin_x.asDouble();
							d->qvel[qvel_adr + 1] = qvel_lin_y.asDouble();
							d->qvel[qvel_adr + 2] = qvel_lin_z.asDouble();
							d->qvel[qvel_adr + 3] = qvel_ang_x.asDouble();
							d->qvel[qvel_adr + 4] = qvel_ang_y.asDouble();
							d->qvel[qvel_adr + 5] = qvel_ang_z.asDouble();
						}
					}
				}

				const int qpos_adr = m->jnt_qposadr[m->body_jntadr[body_id]];
				d->qpos[qpos_adr] = xpos_desired[0];
				d->qpos[qpos_adr + 1] = xpos_desired[1];
				d->qpos[qpos_adr + 2] = xpos_desired[2];
				d->qpos[qpos_adr + 3] = xquat_desired[0];
				d->qpos[qpos_adr + 4] = xquat_desired[1];
				d->qpos[qpos_adr + 5] = xquat_desired[2];
				d->qpos[qpos_adr + 6] = xquat_desired[3];
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
						const int qpos_adr = m->jnt_qposadr[joint_id];
						d->qpos[qpos_adr] = w_json.asDouble();
						d->qpos[qpos_adr + 1] = x_json.asDouble();
						d->qpos[qpos_adr + 2] = y_json.asDouble();
						d->qpos[qpos_adr + 3] = z_json.asDouble();
					}
				}
			}
		}
		else if (actuator_id != -1)
		{
			for (const std::string &attribute_name : send_object.second)
			{
				if (strcmp(attribute_name.c_str(), "cmd_joint_rvalue") == 0 ||
					strcmp(attribute_name.c_str(), "cmd_joint_tvalue") == 0 ||
					strcmp(attribute_name.c_str(), "cmd_joint_angular_velocity") == 0 ||
					strcmp(attribute_name.c_str(), "cmd_joint_linear_velocity") == 0 ||
					strcmp(attribute_name.c_str(), "cmd_joint_torque") == 0 ||
					strcmp(attribute_name.c_str(), "cmd_joint_force") == 0)
				{
					d->ctrl[actuator_id] = response_meta_data_json["send"][send_object.first][attribute_name][0].asDouble();
				}
			}
		}
	}

	tinyxml2::XMLDocument doc;
	if (doc.LoadFile(scene_xml_path.c_str()) == tinyxml2::XML_SUCCESS)
	{
		tinyxml2::XMLElement *mujoco_element = doc.FirstChildElement("mujoco");
		tinyxml2::XMLElement *keyframe_element = mujoco_element->FirstChildElement("keyframe");
		if (keyframe_element == nullptr)
		{
			return;
		}
		tinyxml2::XMLElement *key_element = keyframe_element->FirstChildElement("key");

		key_element->SetAttribute("time", d->time);
		std::string qpos_str;
		for (int qpos_id = 0; qpos_id < m->nq; qpos_id++)
		{
			qpos_str += std::to_string(d->qpos[qpos_id]) + " ";
			m->key_qpos[qpos_id] = d->qpos[qpos_id];
		}
		if (qpos_str.size() > 0)
		{
			qpos_str.pop_back();
			key_element->SetAttribute("qpos", qpos_str.c_str());
		}

		std::string qvel_str;
		for (int dof_id = 0; dof_id < m->nv; dof_id++)
		{
			qvel_str += std::to_string(d->qvel[dof_id]) + " ";
			m->key_qvel[dof_id] = d->qvel[dof_id];
		}
		if (qvel_str.size() > 0)
		{
			qvel_str.pop_back();
			key_element->SetAttribute("qvel", qvel_str.c_str());
		}

		std::string act_str;
		for (int act_id = 0; act_id < m->na; act_id++)
		{
			act_str += std::to_string(d->act[act_id]) + " ";
			m->key_act[act_id] = d->act[act_id];
		}
		if (act_str.size() > 0)
		{
			act_str.pop_back();
			key_element->SetAttribute("act", act_str.c_str());
		}

		std::string ctrl_str;
		for (int ctrl_id = 0; ctrl_id < m->nu; ctrl_id++)
		{
			ctrl_str += std::to_string(d->ctrl[ctrl_id]) + " ";
			m->key_ctrl[ctrl_id] = d->ctrl[ctrl_id];
		}
		if (ctrl_str.size() > 0)
		{
			ctrl_str.pop_back();
			key_element->SetAttribute("ctrl", ctrl_str.c_str());
		}

		std::string mocap_pos_str;
		std::string mocap_quat_str;
		for (int mocap_id = 0; mocap_id < m->nmocap; mocap_id++)
		{
			mocap_pos_str += std::to_string(d->mocap_pos[3 * mocap_id]) + " ";
			mocap_pos_str += std::to_string(d->mocap_pos[3 * mocap_id + 1]) + " ";
			mocap_pos_str += std::to_string(d->mocap_pos[3 * mocap_id + 2]) + " ";
			m->key_mpos[3 * mocap_id] = d->mocap_pos[3 * mocap_id];
			m->key_mpos[3 * mocap_id + 1] = d->mocap_pos[3 * mocap_id + 1];
			m->key_mpos[3 * mocap_id + 2] = d->mocap_pos[3 * mocap_id + 2];

			mocap_quat_str += std::to_string(d->mocap_quat[4 * mocap_id]) + " ";
			mocap_quat_str += std::to_string(d->mocap_quat[4 * mocap_id + 1]) + " ";
			mocap_quat_str += std::to_string(d->mocap_quat[4 * mocap_id + 2]) + " ";
			mocap_quat_str += std::to_string(d->mocap_quat[4 * mocap_id + 3]) + " ";
			m->key_mquat[4 * mocap_id] = d->mocap_quat[4 * mocap_id];
			m->key_mquat[4 * mocap_id + 1] = d->mocap_quat[4 * mocap_id + 1];
			m->key_mquat[4 * mocap_id + 2] = d->mocap_quat[4 * mocap_id + 2];
			m->key_mquat[4 * mocap_id + 3] = d->mocap_quat[4 * mocap_id + 3];
		}
		if (mocap_pos_str.size() > 0)
		{
			mocap_pos_str = mocap_pos_str.substr(0, mocap_pos_str.size() - 1);
			key_element->SetAttribute("mpos", mocap_pos_str.c_str());
		}
		if (mocap_quat_str.size() > 0)
		{
			mocap_quat_str = mocap_quat_str.substr(0, mocap_quat_str.size() - 1);
			key_element->SetAttribute("mquat", mocap_quat_str.c_str());
		}

		doc.SaveFile(scene_xml_path.c_str());
	}
	else
	{
		printf("Could not load file: %s\n", scene_xml_path.c_str());
	}
}

void MjMultiverseClient::weld(const Json::Value &arguments)
{
	const std::string weld_response = get_weld_response(arguments);
	if (strcmp(weld_response.c_str(), "success") == 0)
	{
		printf("Attachment already exists.\n");
		return;
	}
	if (strcmp(weld_response.c_str(), "failed (Equality not found)") != 0 &&
		strcmp(weld_response.c_str(), "failed (Relative pose are different)") != 0)
	{
		printf("%s\n", weld_response.c_str());
		return;
	}

	const std::string object_1_name = arguments[0].asString();
	const std::string object_2_name = arguments[1].asString();

	const int body_1_id = mj_name2id(m, mjtObj::mjOBJ_BODY, object_2_name.c_str());
	const int body_2_id = mj_name2id(m, mjtObj::mjOBJ_BODY, object_1_name.c_str());

	mjtNum *body_1_pos = d->xpos + 3 * body_1_id;
	mjtNum *body_1_quat = d->xquat + 4 * body_1_id;
	mjtNum body_1_neg_quat[4];
	mju_negQuat(body_1_neg_quat, body_1_quat);

	mjtNum *body_2_pos = d->xpos + 3 * body_2_id;
	mjtNum *body_2_quat = d->xquat + 4 * body_2_id;

	mjtNum body_2_in_1_pos[3];
	mjtNum body_2_in_1_quat[4];

	mju_sub3(body_2_in_1_pos, body_2_pos, body_1_pos);
	mju_rotVecQuat(body_2_in_1_pos, body_2_in_1_pos, body_1_neg_quat);

	mju_mulQuat(body_2_in_1_quat, body_1_neg_quat, body_2_quat);

	std::string relative_pose = std::to_string(body_2_in_1_pos[0]) + " " + std::to_string(body_2_in_1_pos[1]) + " " + std::to_string(body_2_in_1_pos[2]) + " " + std::to_string(body_2_in_1_quat[0]) + " " + std::to_string(body_2_in_1_quat[1]) + " " + std::to_string(body_2_in_1_quat[2]) + " " + std::to_string(body_2_in_1_quat[3]);
	if (arguments.size() == 3)
	{
		relative_pose = arguments[2].asString();
	}

	tinyxml2::XMLDocument doc;
	if (doc.LoadFile(scene_xml_path.c_str()) == tinyxml2::XML_SUCCESS)
	{
		tinyxml2::XMLElement *mujoco_element = doc.FirstChildElement("mujoco");

		if (strcmp(weld_response.c_str(), "failed (Relative pose are different)") == 0)
		{
			bool found_weld_element = false;
			for (tinyxml2::XMLElement *equality_element = mujoco_element->FirstChildElement("equality");
				 equality_element != nullptr;
				 equality_element = equality_element->NextSiblingElement("equality"))
			{
				for (tinyxml2::XMLElement *weld_element = equality_element->FirstChildElement("weld");
					 weld_element != nullptr;
					 weld_element = weld_element->NextSiblingElement("weld"))
				{
					if (strcmp(weld_element->Attribute("body1"), object_2_name.c_str()) == 0 &&
						strcmp(weld_element->Attribute("body2"), object_1_name.c_str()) == 0)
					{
						weld_element->SetAttribute("relpose", relative_pose.c_str());
						found_weld_element = true;
						break;
					}
				}
				if (found_weld_element)
				{
					break;
				}
			}
		}
		else
		{
			tinyxml2::XMLElement *equality_element = doc.NewElement("equality");
			mujoco_element->InsertEndChild(equality_element);

			tinyxml2::XMLElement *weld_element = doc.NewElement("weld");
			equality_element->InsertEndChild(weld_element);

			weld_element->SetAttribute("name", (object_1_name + "_" + object_2_name).c_str());
			weld_element->SetAttribute("body1", object_2_name.c_str());
			weld_element->SetAttribute("body2", object_1_name.c_str());
			weld_element->SetAttribute("relpose", relative_pose.c_str());
		}

		doc.SaveFile(scene_xml_path.c_str());
	}
	else
	{
		printf("Could not load file: %s\n", scene_xml_path.c_str());
		return;
	}

	printf("Weld %s to %s at %s\n", object_1_name.c_str(), object_2_name.c_str(), relative_pose.c_str());
	MjSimulate::load_new_model_and_keep_old_data();
}

std::string MjMultiverseClient::get_weld_response(const Json::Value &arguments) const
{
	if (!arguments.isArray())
	{
		return "failed (Arguments for weld should be an array of strings.)";
	}
	if (arguments.size() < 2 || arguments.size() > 3)
	{
		return "failed (Arguments for weld should be an array of strings with 2 or 3 elements.)";
	}

	const std::string object_1_name = arguments[0].asString();
	const std::string object_2_name = arguments[1].asString();

	const int body_1_id = mj_name2id(m, mjtObj::mjOBJ_BODY, object_2_name.c_str());
	const int body_2_id = mj_name2id(m, mjtObj::mjOBJ_BODY, object_1_name.c_str());

	if (body_2_id == -1)
	{
		return "failed (Object " + object_1_name + " does not exist.)";
	}
	if (body_1_id == -1)
	{
		return "failed (Object " + object_2_name + " does not exist.)";
	}

	std::vector<mjtNum> relative_pose;
	if (arguments.size() == 3)
	{
		std::istringstream iss(arguments[2].asString());
		relative_pose = std::vector<mjtNum>(std::istream_iterator<mjtNum>(iss), std::istream_iterator<mjtNum>());
	}

	for (int equality_id = 0; equality_id < m->neq; equality_id++)
	{
		if (m->eq_type[equality_id] == mjtEq::mjEQ_WELD)
		{
			const int body1_id = m->eq_obj1id[equality_id];
			const std::string body_1_name = mj_id2name(m, mjtObj::mjOBJ_BODY, body1_id);
			if (strcmp(body_1_name.c_str(), object_2_name.c_str()) != 0)
			{
				continue;
			}

			const int body2_id = m->eq_obj2id[equality_id];
			const std::string body_2_name = mj_id2name(m, mjtObj::mjOBJ_BODY, body2_id);
			if (strcmp(body_2_name.c_str(), object_1_name.c_str()) != 0)
			{
				continue;
			}

			if (arguments.size() == 3)
			{
				for (int i = 0; i < 7; i++)
				{
					if (mju_abs(relative_pose[i] - m->eq_data[mjNEQDATA * equality_id + i + 3]) > 1e-3)
					{
						return "failed (Relative pose are different)";
					}
				}
			}

			return "success";
		}
	}

	return "failed (Equality not found)";
}

void MjMultiverseClient::unweld(const Json::Value &arguments)
{
	const std::string unweld_response = get_unweld_response(arguments);
	if (strcmp(unweld_response.c_str(), "success") == 0)
	{
		printf("Attachment not found, already unwelded.\n");
		return;
	}
	if (strcmp(unweld_response.c_str(), "failed (equality found)") != 0)
	{
		printf("%s\n", unweld_response.c_str());
		return;
	}

	const std::string object_1_name = arguments[0].asString();
	const std::string object_2_name = arguments[1].asString();

	tinyxml2::XMLDocument doc;
	if (doc.LoadFile(scene_xml_path.c_str()) == tinyxml2::XML_SUCCESS)
	{
		tinyxml2::XMLElement *mujoco_element = doc.FirstChildElement("mujoco");

		tinyxml2::XMLElement *found_weld_element = nullptr;
		for (tinyxml2::XMLElement *equality_element = mujoco_element->FirstChildElement("equality");
			 equality_element != nullptr;
			 equality_element = equality_element->NextSiblingElement("equality"))
		{
			for (tinyxml2::XMLElement *weld_element = equality_element->FirstChildElement("weld");
				 weld_element != nullptr;
				 weld_element = weld_element->NextSiblingElement("weld"))
			{
				if (strcmp(weld_element->Attribute("body1"), object_2_name.c_str()) == 0 &&
					strcmp(weld_element->Attribute("body2"), object_1_name.c_str()) == 0)
				{
					found_weld_element = weld_element;
					break;
				}
			}
			if (found_weld_element != nullptr)
			{
				equality_element->DeleteChild(found_weld_element);
				break;
			}
		}

		doc.SaveFile(scene_xml_path.c_str());
	}
	else
	{
		printf("Could not load file: %s\n", scene_xml_path.c_str());
		return;
	}

	printf("Unweld %s from %s\n", object_1_name.c_str(), object_2_name.c_str());
	MjSimulate::load_new_model_and_keep_old_data();
}

std::string MjMultiverseClient::get_unweld_response(const Json::Value &arguments) const
{
	if (!arguments.isArray())
	{
		return "failed (Arguments for weld should be an array of strings.)";
	}
	if (arguments.size() != 2)
	{
		return "failed (Arguments for weld should be an array of strings with 2 elements.)";
	}

	const std::string object_1_name = arguments[0].asString();
	const std::string object_2_name = arguments[1].asString();

	if (mj_name2id(m, mjtObj::mjOBJ_BODY, object_1_name.c_str()) == -1)
	{
		return "failed (Object " + object_1_name + " does not exist.)";
	}
	if (mj_name2id(m, mjtObj::mjOBJ_BODY, object_2_name.c_str()) == -1)
	{
		return "failed (Object " + object_2_name + " does not exist.)";
	}

	for (int equality_id = 0; equality_id < m->neq; equality_id++)
	{
		if (m->eq_type[equality_id] == mjtEq::mjEQ_WELD)
		{
			const int body1_id = m->eq_obj1id[equality_id];
			const std::string body_1_name = mj_id2name(m, mjtObj::mjOBJ_BODY, body1_id);
			if (strcmp(body_1_name.c_str(), object_2_name.c_str()) != 0)
			{
				continue;
			}

			const int body2_id = m->eq_obj2id[equality_id];
			const std::string body_2_name = mj_id2name(m, mjtObj::mjOBJ_BODY, body2_id);
			if (strcmp(body_2_name.c_str(), object_1_name.c_str()) != 0)
			{
				continue;
			}

			return "failed (equality found)";
		}
	}
	return "success";
}

void get_body_element(tinyxml2::XMLElement *parent_body_element, tinyxml2::XMLElement *&found_body_element, const std::string &body_name)
{
	for (tinyxml2::XMLElement *body_element = parent_body_element->FirstChildElement("body");
		 body_element != nullptr;
		 body_element = body_element->NextSiblingElement("body"))
	{
		if (strcmp(body_element->Attribute("name"), body_name.c_str()) == 0)
		{
			found_body_element = body_element;
			if (found_body_element != nullptr)
			{
				return;
			}
		}
		get_body_element(body_element, found_body_element, body_name);
	}
}

void get_body_element(tinyxml2::XMLDocument &doc, std::string &doc_file_path, tinyxml2::XMLElement *&found_body_element, const std::string &body_name)
{
	tinyxml2::XMLElement *mujoco_element = doc.FirstChildElement("mujoco");
	for (tinyxml2::XMLElement *worldbody_element = mujoco_element->FirstChildElement("worldbody");
		 worldbody_element != nullptr;
		 worldbody_element = worldbody_element->NextSiblingElement("worldbody"))
	{
		get_body_element(worldbody_element, found_body_element, body_name);
		if (found_body_element != nullptr)
		{
			doc_file_path = scene_xml_path.string();
			return;
		}
	}

	const boost::filesystem::path scene_dir = scene_xml_path.parent_path();
	std::vector<std::string> file_paths;
	for (tinyxml2::XMLElement *include_element = mujoco_element->FirstChildElement("include");
		 include_element != nullptr;
		 include_element = include_element->NextSiblingElement("include"))
	{
		const std::string file_name = include_element->Attribute("file");
		file_paths.push_back((scene_dir / file_name).string());
	}
	for (const std::string &file_path : file_paths)
	{
		if (doc.LoadFile(file_path.c_str()) == tinyxml2::XML_SUCCESS)
		{
			get_body_element(doc, doc_file_path, found_body_element, body_name);
			if (found_body_element != nullptr)
			{
				doc_file_path = file_path;
				return;
			}
		}
		else
		{
			printf("Could not load file: %s\n", file_path.c_str());
			return;
		}
	}
}

void MjMultiverseClient::attach(const Json::Value &arguments)
{
	const std::string attach_response = get_attach_response(arguments);
	if (strcmp(attach_response.c_str(), "success") == 0)
	{
		printf("Attachment already exists.\n");
		return;
	}
	if (strcmp(attach_response.c_str(), "failed (Relative pose are different)") != 0 &&
		strcmp(attach_response.c_str(), "failed (attachment not found)") != 0)
	{
		printf("%s\n", attach_response.c_str());
		return;
	}

	const std::string object_1_name = arguments[0].asString();
	const std::string object_2_name = arguments[1].asString();

	const int body_1_id = mj_name2id(m, mjtObj::mjOBJ_BODY, object_2_name.c_str());
	const int body_2_id = mj_name2id(m, mjtObj::mjOBJ_BODY, object_1_name.c_str());

	mjtNum *body_1_pos = d->xpos + 3 * body_1_id;
	mjtNum *body_1_quat = d->xquat + 4 * body_1_id;
	mjtNum body_1_neg_quat[4];
	mju_negQuat(body_1_neg_quat, body_1_quat);

	mjtNum *body_2_pos = d->xpos + 3 * body_2_id;
	mjtNum *body_2_quat = d->xquat + 4 * body_2_id;

	mjtNum body_2_in_1_pos[3];
	mjtNum body_2_in_1_quat[4];

	mju_sub3(body_2_in_1_pos, body_2_pos, body_1_pos);
	mju_rotVecQuat(body_2_in_1_pos, body_2_in_1_pos, body_1_neg_quat);

	mju_mulQuat(body_2_in_1_quat, body_1_neg_quat, body_2_quat);
	std::vector<mjtNum> relative_pose = {body_2_in_1_pos[0], body_2_in_1_pos[1], body_2_in_1_pos[2], body_2_in_1_quat[0], body_2_in_1_quat[1], body_2_in_1_quat[2], body_2_in_1_quat[3]};
	if (arguments.size() == 3)
	{
		std::istringstream iss(arguments[2].asString());
		relative_pose = std::vector<mjtNum>(std::istream_iterator<mjtNum>(iss), std::istream_iterator<mjtNum>());
	}
	const std::string relative_pos = std::to_string(relative_pose[0]) + " " + std::to_string(relative_pose[1]) + " " + std::to_string(relative_pose[2]);
	const std::string relative_quat = std::to_string(relative_pose[3]) + " " + std::to_string(relative_pose[4]) + " " + std::to_string(relative_pose[5]) + " " + std::to_string(relative_pose[6]);

	tinyxml2::XMLDocument doc_1;
	std::string doc_file_path_1;
	doc_1.LoadFile(scene_xml_path.c_str());
	tinyxml2::XMLElement *body_1_element = nullptr;
	get_body_element(doc_1, doc_file_path_1, body_1_element, object_1_name);

	tinyxml2::XMLDocument doc_2;
	std::string doc_file_path_2;
	doc_2.LoadFile(scene_xml_path.c_str());
	tinyxml2::XMLElement *body_2_element = nullptr;
	get_body_element(doc_2, doc_file_path_2, body_2_element, object_2_name);

	body_1_element->SetAttribute("pos", relative_pos.c_str());
	body_1_element->SetAttribute("quat", relative_quat.c_str());
	if (strcmp(attach_response.c_str(), "failed (attachment not found)") == 0)
	{
		tinyxml2::XMLElement *body_1_element_copy = body_1_element->DeepClone(&doc_2)->ToElement();
		std::vector<tinyxml2::XMLElement *> body_1_joint_elements;
		for (tinyxml2::XMLElement *joint_element = body_1_element_copy->FirstChildElement("joint");
			 joint_element != nullptr;
			 joint_element = joint_element->NextSiblingElement("joint"))
		{
			body_1_joint_elements.push_back(joint_element);
		}
		for (tinyxml2::XMLElement *joint_element = body_1_element_copy->FirstChildElement("freejoint");
			 joint_element != nullptr;
			 joint_element = joint_element->NextSiblingElement("freejoint"))
		{
			body_1_joint_elements.push_back(joint_element);
		}
		for (tinyxml2::XMLElement *joint_element : body_1_joint_elements)
		{
			body_1_element_copy->DeleteChild(joint_element);
		}
		body_2_element->InsertEndChild(body_1_element_copy);

		if (strcmp(doc_file_path_1.c_str(), doc_file_path_2.c_str()) == 0)
		{
			doc_2.SaveFile(doc_file_path_2.c_str());

			tinyxml2::XMLDocument doc;
			doc.LoadFile(scene_xml_path.c_str());

			tinyxml2::XMLElement *found_body_element = nullptr;
			for (tinyxml2::XMLElement *worldbody_element = doc.FirstChildElement("mujoco")->FirstChildElement("worldbody");
				worldbody_element != nullptr;
				worldbody_element = worldbody_element->NextSiblingElement("worldbody"))
			{
				get_body_element(worldbody_element, found_body_element, object_1_name);
			}

			tinyxml2::XMLElement *parent_body_1_element = found_body_element->Parent()->ToElement();
			parent_body_1_element->DeleteChildren();
			doc.SaveFile(scene_xml_path.c_str());
		}
		else
		{
			tinyxml2::XMLElement *parent_body_1_element = body_1_element->Parent()->ToElement();
			parent_body_1_element->DeleteChild(body_1_element);
			doc_1.SaveFile(doc_file_path_1.c_str());
			doc_2.SaveFile(doc_file_path_2.c_str());
		}

		const int body_1_id = mj_name2id(m, mjtObj::mjOBJ_BODY, object_1_name.c_str());
		const int body_1_dof_num = m->body_dofnum[body_1_id];
		if (body_1_dof_num != 0)
		{
			tinyxml2::XMLDocument doc;
			doc.LoadFile(scene_xml_path.c_str());
			tinyxml2::XMLElement *mujoco_element = doc.FirstChildElement("mujoco");
			for (tinyxml2::XMLElement *keyframe_element = mujoco_element->FirstChildElement("keyframe");
				 keyframe_element != nullptr; keyframe_element = keyframe_element->NextSiblingElement("keyframe"))
			{
				for (tinyxml2::XMLElement *key_element = keyframe_element->FirstChildElement("key");
					 key_element != nullptr; key_element = key_element->NextSiblingElement("key"))
				{
					if (key_element->Attribute("qpos") != nullptr)
					{
						std::istringstream iss(key_element->Attribute("qpos"));
						std::vector<mjtNum> qpos = std::vector<mjtNum>(std::istream_iterator<mjtNum>(iss), std::istream_iterator<mjtNum>());
						const int qpos_adr = m->jnt_qposadr[m->body_jntadr[body_1_id]];
						if (body_1_dof_num == 6)
						{
							for (int i = 0; i < 7; i++)
							{
								qpos.erase(qpos.begin() + qpos_adr);
							}
						}
						else
						{
							mju_error("Unsupported body dof num: %d", body_1_dof_num);
						}
						std::string qpos_str;
						for (mjtNum qpos_val : qpos)
						{
							qpos_str += std::to_string(qpos_val) + " ";
						}
						qpos_str.pop_back();
						key_element->SetAttribute("qpos", qpos_str.c_str());
					}
					if (key_element->Attribute("qvel") != nullptr)
					{
						std::istringstream iss(key_element->Attribute("qvel"));
						std::vector<mjtNum> qvel = std::vector<mjtNum>(std::istream_iterator<mjtNum>(iss), std::istream_iterator<mjtNum>());
						const int qvel_adr = m->jnt_dofadr[m->body_jntadr[body_1_id]];
						if (body_1_dof_num == 6)
						{
							for (int i = 0; i < 6; i++)
							{
								qvel.erase(qvel.begin() + qvel_adr);
							}
						}
						else
						{
							mju_error("Unsupported body dof num: %d", body_1_dof_num);
						}
						std::string qvel_str;
						for (mjtNum qvel_val : qvel)
						{
							qvel_str += std::to_string(qvel_val) + " ";
						}
						qvel_str.pop_back();
						key_element->SetAttribute("qvel", qvel_str.c_str());
					}
				}
			}
			doc.SaveFile(scene_xml_path.c_str());
		}
	}
	else
	{
		doc_1.SaveFile(doc_file_path_1.c_str());
	}

	printf("Attach %s to %s at %s %s\n", object_1_name.c_str(), object_2_name.c_str(), relative_pos.c_str(), relative_quat.c_str());
	MjSimulate::load_new_model_and_keep_old_data();
}

std::string MjMultiverseClient::get_attach_response(const Json::Value &arguments) const
{
	if (!arguments.isArray() || arguments.size() < 2 || arguments.size() > 3)
	{
		return "failed (Arguments for attach should be an array of strings with 2 or 3 elements.)";
	}

	const std::string object_1_name = arguments[0].asString();
	const std::string object_2_name = arguments[1].asString();

	const int body_1_id = mj_name2id(m, mjtObj::mjOBJ_BODY, object_1_name.c_str());
	if (body_1_id == -1)
	{
		return "failed (Object " + object_1_name + " does not exist.)";
	}

	const int body_2_id = mj_name2id(m, mjtObj::mjOBJ_BODY, object_2_name.c_str());
	if (body_2_id == -1)
	{
		return "failed (Object " + object_2_name + " does not exist.)";
	}

	if (m->body_parentid[body_1_id] == body_2_id)
	{
		if (arguments.size() == 3)
		{
			std::istringstream iss(arguments[2].asString());
			std::vector<mjtNum> relative_pose = std::vector<mjtNum>(std::istream_iterator<mjtNum>(iss), std::istream_iterator<mjtNum>());
			if (m->body_pos[3 * body_1_id] != relative_pose[0] ||
				m->body_pos[3 * body_1_id + 1] != relative_pose[1] ||
				m->body_pos[3 * body_1_id + 2] != relative_pose[2] ||
				m->body_quat[4 * body_1_id] != relative_pose[3] ||
				m->body_quat[4 * body_1_id + 1] != relative_pose[4] ||
				m->body_quat[4 * body_1_id + 2] != relative_pose[5] ||
				m->body_quat[4 * body_1_id + 3] != relative_pose[6])
			{
				return "failed (Relative pose are different)";
			}
		}
		return "success";
	}
	else
	{
		return "failed (Attachment not found)";
	}
}

void MjMultiverseClient::detach(const Json::Value &arguments)
{
	const std::string detach_response = get_detach_response(arguments);
	if (strcmp(detach_response.c_str(), "success") == 0)
	{
		printf("Attachment not found, already detached.\n");
		return;
	}
	if (strcmp(detach_response.c_str(), "failed (Attachment found)") != 0)
	{
		printf("%s\n", detach_response.c_str());
		return;
	}

	const std::string object_1_name = arguments[0].asString();
	const std::string object_2_name = arguments[1].asString();

	tinyxml2::XMLDocument object_doc;
	std::string object_doc_file_path;
	object_doc.LoadFile(scene_xml_path.c_str());
	tinyxml2::XMLElement *body_1_element = nullptr;
	get_body_element(object_doc, object_doc_file_path, body_1_element, object_1_name);

	tinyxml2::XMLDocument scene_doc;
	tinyxml2::XMLElement *mujoco_element;
	tinyxml2::XMLElement *worldbody_element;
	tinyxml2::XMLElement *freejoint_element;
	tinyxml2::XMLElement *body_1_element_copy;
	if (strcmp(scene_xml_path.c_str(), object_doc_file_path.c_str()) != 0)
	{
		scene_doc.LoadFile(scene_xml_path.c_str());
		mujoco_element = scene_doc.FirstChildElement("mujoco");
		worldbody_element = scene_doc.NewElement("worldbody");
		freejoint_element = scene_doc.NewElement("freejoint");
		body_1_element_copy = body_1_element->DeepClone(&scene_doc)->ToElement();
	}
	else
	{
		mujoco_element = object_doc.FirstChildElement("mujoco");
		worldbody_element = object_doc.NewElement("worldbody");
		freejoint_element = object_doc.NewElement("freejoint");
		body_1_element_copy = body_1_element->DeepClone(&object_doc)->ToElement();
	}

	mujoco_element->InsertEndChild(worldbody_element);
	worldbody_element->InsertEndChild(body_1_element_copy);
	body_1_element_copy->InsertFirstChild(freejoint_element);

	tinyxml2::XMLElement *parent_body_1_element = body_1_element->Parent()->ToElement();
	parent_body_1_element->DeleteChild(body_1_element);

	const int body_1_id = mj_name2id(m, mjtObj::mjOBJ_BODY, object_1_name.c_str());
	const int body_1_dof_num = m->body_dofnum[body_1_id];

	for (tinyxml2::XMLElement *keyframe_element = mujoco_element->FirstChildElement("keyframe");
		 keyframe_element != nullptr; keyframe_element = keyframe_element->NextSiblingElement("keyframe"))
	{
		for (tinyxml2::XMLElement *key_element = keyframe_element->FirstChildElement("key");
			 key_element != nullptr; key_element = key_element->NextSiblingElement("key"))
		{
			if (key_element->Attribute("qpos") != nullptr)
			{
				std::istringstream iss(key_element->Attribute("qpos"));
				std::vector<mjtNum> qpos = std::vector<mjtNum>(std::istream_iterator<mjtNum>(iss), std::istream_iterator<mjtNum>());
				const int qpos_adr = m->jnt_qposadr[m->body_jntadr[body_1_id]];
				for (int i = 2; i >= 0; i--)
				{
					qpos.insert(qpos.begin() + qpos_adr, d->xpos[3 * body_1_id + i]);
				}
				for (int i = 3; i >= 0; i--)
				{
					qpos.insert(qpos.begin() + qpos_adr + 3, d->xquat[4 * body_1_id + i]);
				}
				std::string qpos_str;
				for (mjtNum qpos_val : qpos)
				{
					qpos_str += std::to_string(qpos_val) + " ";
				}
				qpos_str.pop_back();
				key_element->SetAttribute("qpos", qpos_str.c_str());
			}
			if (key_element->Attribute("qvel") != nullptr)
			{
				std::istringstream iss(key_element->Attribute("qvel"));
				std::vector<mjtNum> qvel = std::vector<mjtNum>(std::istream_iterator<mjtNum>(iss), std::istream_iterator<mjtNum>());
				const int qvel_adr = m->jnt_dofadr[m->body_jntadr[body_1_id]];
				for (int i = 0; i < 6; i++)
				{
					qvel.insert(qvel.begin() + qvel_adr, 0.0);
				}
				std::string qvel_str;
				for (mjtNum qvel_val : qvel)
				{
					qvel_str += std::to_string(qvel_val) + " ";
				}
				qvel_str.pop_back();
				key_element->SetAttribute("qvel", qvel_str.c_str());
			}
		}
	}

	if (strcmp(scene_xml_path.c_str(), object_doc_file_path.c_str()) != 0)
	{
		scene_doc.SaveFile(scene_xml_path.c_str());
		object_doc.SaveFile(object_doc_file_path.c_str());
	}
	else
	{
		object_doc.SaveFile(scene_xml_path.c_str());
	}

	printf("Detach %s from %s\n", object_1_name.c_str(), object_2_name.c_str());
	MjSimulate::load_new_model_and_keep_old_data();
}

std::string MjMultiverseClient::get_save_response(const Json::Value &arguments) const
{
	if (!arguments.isArray() || arguments.size() != 1)
	{
		return "failed (Arguments for save should be an array of strings with 1 element.)";
	}

	boost::filesystem::path save_path = arguments[0].asString();
	if (!save_path.has_extension())
	{
		save_path /= scene_xml_path.filename();
	}
	if (save_path.is_relative())
	{
		save_path = scene_xml_path.parent_path() / save_path;
	}

	boost::filesystem::path save_dir = save_path.parent_path();
	if (!boost::filesystem::exists(save_dir))
	{
		boost::filesystem::create_directories(save_dir);
	}

	tinyxml2::XMLDocument doc;
	if (doc.LoadFile(scene_xml_path.c_str()) == tinyxml2::XML_SUCCESS)
	{
		tinyxml2::XMLElement *mujoco_element = doc.FirstChildElement("mujoco");
		for (tinyxml2::XMLElement *include_element = mujoco_element->FirstChildElement("include");
			 include_element != nullptr;
			 include_element = include_element->NextSiblingElement("include"))
		{
			boost::filesystem::path file_path = include_element->Attribute("file");
			if (file_path.is_relative())
			{
				file_path = scene_xml_path.parent_path() / file_path;
			}
			boost::filesystem::copy_file(file_path, save_path.parent_path() / file_path.filename(), boost::filesystem::copy_option::overwrite_if_exists);
		}
		doc.SaveFile(save_path.c_str());
		printf("Saved scene to %s\n", save_path.c_str());
	}
	else
	{
		printf("Could not load file: %s\n", scene_xml_path.c_str());
	}

	return save_path.string();
}

void MjMultiverseClient::load(const Json::Value &arguments)
{
	const std::string load_response = get_load_response(arguments);
	if (strcmp(load_response.c_str(), "failed (Path does not exist.)") == 0)
	{
		printf("Path does not exist.\n");
		return;
	}

	boost::filesystem::path load_path = load_response;
	tinyxml2::XMLDocument doc;
	if (doc.LoadFile(load_path.c_str()) == tinyxml2::XML_SUCCESS)
	{
		tinyxml2::XMLElement *mujoco_element = doc.FirstChildElement("mujoco");
		for (tinyxml2::XMLElement *include_element = mujoco_element->FirstChildElement("include");
			 include_element != nullptr;
			 include_element = include_element->NextSiblingElement("include"))
		{
			boost::filesystem::path file_path = include_element->Attribute("file");
			if (file_path.is_relative())
			{
				file_path = load_path.parent_path() / file_path;
			}
			boost::filesystem::copy_file(load_path, scene_xml_path.parent_path() / load_path.filename(), boost::filesystem::copy_option::overwrite_if_exists);
		}
		doc.SaveFile(scene_xml_path.c_str());
	}

	printf("Loaded scene from %s\n", load_response.c_str());
	MjSimulate::load_new_model_and_keep_old_data();

	std::set<std::string> exclude_objects;
	for (const std::string &object_name : request_meta_data_json["send"].getMemberNames())
	{
		exclude_objects.insert(object_name);
	}
	for (const std::string &object_name : request_meta_data_json["receive"].getMemberNames())
	{
		exclude_objects.insert(object_name);
	}
	for (const std::string &object_name : send_objects_json.getMemberNames())
	{
		exclude_objects.insert(object_name);
	}
	for (const std::string &object_name : receive_objects_json.getMemberNames())
	{
		exclude_objects.insert(object_name);
	}
	exclude_objects.erase("body");
	exclude_objects.erase("joint");
	for (const std::string &object_name : exclude_objects)
	{
		if (mj_name2id(m, mjtObj::mjOBJ_BODY, object_name.c_str()) == -1 && mj_name2id(m, mjtObj::mjOBJ_BODY, object_name.c_str()) == -1 && mj_name2id(m, mjtObj::mjOBJ_JOINT, object_name.c_str()) == -1 && mj_name2id(m, mjtObj::mjOBJ_ACTUATOR, object_name.c_str()) == -1)
		{
			request_meta_data_json["send"].removeMember(object_name);
			request_meta_data_json["receive"].removeMember(object_name);
			send_objects_json.removeMember(object_name);
			receive_objects_json.removeMember(object_name);
		}
	}
}

std::string MjMultiverseClient::get_load_response(const Json::Value &arguments) const
{
	if (!arguments.isArray() || arguments.size() != 1)
	{
		return "failed (Arguments for load should be an array of strings with 1 element.)";
	}

	boost::filesystem::path load_path = arguments[0].asString();
	if (!load_path.has_extension())
	{
		load_path /= scene_xml_path.filename();
	}
	if (load_path.is_relative())
	{
		load_path = scene_xml_path.parent_path() / load_path;
	}

	if (!boost::filesystem::exists(load_path))
	{
		return "failed (Path does not exist.)";
	}

	return load_path.string();
}

std::string MjMultiverseClient::get_detach_response(const Json::Value &arguments) const
{
	if (!arguments.isArray() || arguments.size() != 2)
	{
		return "failed (Arguments for detach should be an array of strings with 2 elements.)";
	}

	const std::string object_1_name = arguments[0].asString();
	const std::string object_2_name = arguments[1].asString();

	const int body_1_id = mj_name2id(m, mjtObj::mjOBJ_BODY, object_1_name.c_str());
	if (body_1_id == -1)
	{
		return "failed (Object " + object_1_name + " does not exist.)";
	}

	const int body_2_id = mj_name2id(m, mjtObj::mjOBJ_BODY, object_2_name.c_str());
	if (body_2_id == -1)
	{
		return "failed (Object " + object_2_name + " does not exist.)";
	}

	return m->body_parentid[body_1_id] == body_2_id ? "failed (Attachment found)" : "success";
}

std::set<std::string> MjMultiverseClient::get_get_contact_bodies_response(const Json::Value &arguments) const
{
	if (!arguments.isArray() || (arguments.size() != 1 && arguments.size() != 2))
	{
		return {"failed (Arguments for get_contact_bodies should be an array of strings with 1 or 2 elements.)"};
	}

	const std::string object_name = arguments[0].asString();
	const int body_id = mj_name2id(m, mjtObj::mjOBJ_BODY, object_name.c_str());
	if (body_id == -1)
	{
		return {"failed (Object " + object_name + " does not exist.)"};
	}

	std::set<int> body_ids = {body_id};
	bool with_children = false;
	if (arguments.size() == 2)
	{
		if (arguments[1].asString() != "with_children")
		{
			return {"failed (Second argument for get_contact_bodies should be \"with_children\".)"};
		}
		with_children = true;
	}

	if (with_children)
	{
		for (int child_body_id = body_id + 1; child_body_id < m->nbody; child_body_id++)
		{
			if (m->body_parentid[child_body_id] == body_id)
			{
				body_ids.insert(child_body_id);
			}
			else
			{
				break;
			}
		}
	}

	std::set<int> contact_body_ids;
	for (int contact_id = 0; contact_id < d->ncon; contact_id++)
	{
		const mjContact contact = d->contact[contact_id];
		if (contact.exclude != 0 && contact.exclude != 1)
		{
			continue;
		}
		const int geom_1_id = contact.geom[0];
		const int geom_2_id = contact.geom[1];
		const int body_1_id = m->geom_bodyid[geom_1_id];
		const int body_2_id = m->geom_bodyid[geom_2_id];

		if (body_ids.find(body_1_id) != body_ids.end() && body_ids.find(body_2_id) == body_ids.end())
		{
			contact_body_ids.insert(body_2_id);
		}
		else if (body_ids.find(body_2_id) != body_ids.end() && body_ids.find(body_1_id) == body_ids.end())
		{
			contact_body_ids.insert(body_1_id);
		}
	}

	std::set<std::string> contact_results;
	for (const int &contact_body_id : contact_body_ids)
	{
		contact_results.insert(mj_id2name(m, mjtObj::mjOBJ_BODY, contact_body_id));
	}
	return contact_results;
}

std::set<std::string> MjMultiverseClient::get_get_contact_points_response(const Json::Value &arguments) const
{
	if (!arguments.isArray() || (arguments.size() != 1 && arguments.size() != 2))
	{
		return {"failed (Arguments for get_contact_points should be an array of strings with 1 or 2 elements.)"};
	}

	const std::string object_1_name = arguments[0].asString();
	const int body_1_id = mj_name2id(m, mjtObj::mjOBJ_BODY, object_1_name.c_str());
	if (body_1_id == -1)
	{
		return {"failed (Object " + object_1_name + " does not exist.)"};
	}

	std::vector<std::vector<double>> contact_results;
	if (arguments.size() == 2)
	{
		const std::string object_2_name = arguments[1].asString();
		const int body_2_id = mj_name2id(m, mjtObj::mjOBJ_BODY, object_2_name.c_str());
		if (body_2_id == -1)
		{
			return {"failed (Object " + object_2_name + " does not exist.)"};
		}

		for (int contact_id = 0; contact_id < d->ncon; contact_id++)
		{
			const mjContact contact = d->contact[contact_id];
			if (contact.exclude != 0 && contact.exclude != 1)
			{
				continue;
			}
			const int geom_1_id = contact.geom[0];
			const int geom_2_id = contact.geom[1];

			if ((body_1_id == m->geom_bodyid[geom_1_id] && body_2_id == m->geom_bodyid[geom_2_id]) || (body_1_id == m->geom_bodyid[geom_2_id] && body_2_id == m->geom_bodyid[geom_1_id]))
			{
				std::vector<double> contact_point = {contact.pos[0], contact.pos[1], contact.pos[2]};
				if (body_1_id == m->geom_bodyid[geom_2_id])
				{
					contact_point.push_back(contact.frame[0]);
					contact_point.push_back(contact.frame[1]);
					contact_point.push_back(contact.frame[2]);
				}
				else
				{
					contact_point.push_back(-contact.frame[0]);
					contact_point.push_back(-contact.frame[1]);
					contact_point.push_back(-contact.frame[2]);
				}
				contact_results.push_back(contact_point);
			}
		}
	}
	else
	{
		for (int contact_id = 0; contact_id < d->ncon; contact_id++)
		{
			const mjContact contact = d->contact[contact_id];
			if (contact.exclude != 0 && contact.exclude != 1)
			{
				continue;
			}
			const int geom_1_id = contact.geom[0];
			const int geom_2_id = contact.geom[1];

			if (body_1_id == m->geom_bodyid[geom_1_id] || body_1_id == m->geom_bodyid[geom_2_id])
			{
				std::vector<double> contact_point = {contact.pos[0], contact.pos[1], contact.pos[2]};
				if (body_1_id == m->geom_bodyid[geom_2_id])
				{
					contact_point.push_back(contact.frame[0]);
					contact_point.push_back(contact.frame[1]);
					contact_point.push_back(contact.frame[2]);
				}
				else
				{
					contact_point.push_back(-contact.frame[0]);
					contact_point.push_back(-contact.frame[1]);
					contact_point.push_back(-contact.frame[2]);
				}
				contact_results.push_back(contact_point);
			}
		}
	}

	std::set<std::string> contact_results_str;
	for (const std::vector<double> &contact_result : contact_results)
	{
		std::string contact_result_str = "";
		for (const double &contact_result_val : contact_result)
		{
			contact_result_str += std::to_string(contact_result_val) + " ";
		}
		contact_result_str.pop_back();
		contact_results_str.insert(contact_result_str);
	}

	return contact_results_str;
}

std::set<std::string> MjMultiverseClient::get_get_contact_islands_response(const Json::Value &arguments) const
{
	if (!arguments.isArray() || (arguments.size() != 1 && arguments.size() != 2))
	{
		return {"failed (Arguments for get_contact_islands should be an array of strings with 1 or 2 elements.)"};
	}

	const std::string object_name = arguments[0].asString();
	const int object_body_id = mj_name2id(m, mjtObj::mjOBJ_BODY, object_name.c_str());
	if (object_body_id == -1)
	{
		return {"failed (Object " + object_name + " does not exist.)"};
	}

	std::set<int> body_ids = {object_body_id};
	bool with_children = false;
	if (arguments.size() == 2)
	{
		if (arguments[1].asString() != "with_children")
		{
			return {"failed (Second argument for get_contact_islands should be \"with_children\".)"};
		}
		with_children = true;
	}

	if (with_children)
	{
		for (int child_body_id = object_body_id + 1; child_body_id < m->nbody; child_body_id++)
		{
			if (m->body_parentid[child_body_id] == object_body_id)
			{
				body_ids.insert(child_body_id);
			}
			else
			{
				break;
			}
		}
	}

	mj_island(m, d);

	std::map<int, std::set<int>> islands;
	for (const int body_id : body_ids)
	{
		const int dof_adr = m->body_dofadr[body_id];
		const int dof_num = m->body_dofnum[body_id];
		for (int dof_id = dof_adr; dof_id < dof_adr + dof_num; dof_id++)
		{
			const int island_id = d->dof_island[dof_id];

			if (island_id == -1 || islands.count(island_id) > 0)
			{
				continue;
			}

			islands[island_id] = {};
			const int island_dofadr = d->island_dofadr[island_id];
			const int island_dofnum = d->island_dofnum[island_id];
			for (int island_dof_id = island_dofadr; island_dof_id < island_dofadr + island_dofnum; island_dof_id++)
			{
				const int island_body_id = m->dof_bodyid[d->island_dofind[island_dof_id]];
				if (body_ids.find(island_body_id) == body_ids.end())
				{
					islands[island_id].insert(island_body_id);
				}
			}
		}
	}

	std::set<std::string> contact_island_results;
	for (const std::pair<const int, std::set<int>> &island : islands)
	{
		std::string contact_island_result = "";
		for (const int &contact_body_id : island.second)
		{
			contact_island_result += std::string(mj_id2name(m, mjtObj::mjOBJ_BODY, contact_body_id)) + " ";
		}
		if (!contact_island_result.empty())
		{
			contact_island_result.pop_back();
			contact_island_results.insert(contact_island_result);
		}
	}
	return contact_island_results;
}

std::string MjMultiverseClient::get_get_constraint_effort_response(const Json::Value &arguments) const
{
	if (!arguments.isArray() || arguments.size() != 1)
	{
		return {"failed (Arguments for get_constraint_effort should be an array of strings with 1 element.)"};
	}

	const std::string object_name = arguments[0].asString();
	const int object_body_id = mj_name2id(m, mjtObj::mjOBJ_BODY, object_name.c_str());
	if (object_body_id == -1)
	{
		return {"failed (Object " + object_name + " does not exist.)"};
	}

	mjtNum *contact_effort = (mjtNum *)calloc(6, sizeof(mjtNum));
	mjtNum *jac = mj_stackAllocNum(d, 6 * m->nv);
	mj_jacBodyCom(m, d, jac, jac + 3 * m->nv, object_body_id);
	mju_mulMatVec(contact_effort, jac, d->qfrc_constraint, 6, m->nv);

	std::string contact_effort_result = "";
	for (int i = 0; i < 6; i++)
	{
		contact_effort_result += std::to_string(contact_effort[i]) + " ";
	}
	contact_effort_result.pop_back();

	return contact_effort_result;
}

std::vector<std::string> MjMultiverseClient::get_get_rays_response(const Json::Value &arguments) const
{
	if (!arguments.isArray() || arguments.size() != 2)
	{
		return {"failed (Arguments for get_rays should be an array of strings with 2 elements.)"};
	}

	std::istringstream starting_points_iss(arguments[0].asString());
	std::vector<mjtNum> starting_points = std::vector<mjtNum>(std::istream_iterator<mjtNum>(starting_points_iss), std::istream_iterator<mjtNum>());
	if (starting_points.size() % 3 != 0)
	{
		return {"failed (Starting points should have a multiple of 3 float elements.)"};
	}

	std::istringstream ending_points_iss(arguments[1].asString());
	std::vector<mjtNum> ending_points = std::vector<mjtNum>(std::istream_iterator<mjtNum>(ending_points_iss), std::istream_iterator<mjtNum>());
	if (ending_points.size() % 3 != 0)
	{
		return {"failed (Ending points should have a multiple of 3 float elements.)"};
	}

	if (starting_points.size() / 3 != ending_points.size() / 3)
	{
		return {"failed (Starting points and ending points should have the same number of points.)"};
	}

	std::vector<std::string> multi_ray_results;
	for (int i = 0; i < starting_points.size() / 3; i++)
	{
		const mjtNum pnt[3] = {starting_points[3 * i], starting_points[3 * i + 1], starting_points[3 * i + 2]};
		mjtNum vec[3] = {ending_points[3 * i] - pnt[0], ending_points[3 * i + 1] - pnt[1], ending_points[3 * i + 2] - pnt[2]};
		const mjtNum vec_len = mju_normalize3(vec);

		const mjtByte *geomgroup = NULL;
		const mjtByte flg_static = 1;
		const int bodyexclude = -1;

		int geomid;
		const mjtNum dist = mj_ray(m, d, pnt, vec, geomgroup, flg_static, bodyexclude, &geomid);

		if (geomid != -1 && dist <= vec_len)
		{
			const int body_id = m->geom_bodyid[geomid];
			const std::string body_name = mj_id2name(m, mjtObj::mjOBJ_BODY, body_id);
			const std::string dist_str = std::to_string(dist);
			multi_ray_results.push_back(body_name + " " + dist_str);
		}
		else
		{
			multi_ray_results.push_back({"None"});
		}
	}

	return multi_ray_results;
}

std::vector<std::string> MjMultiverseClient::get_exist_response(const Json::Value &arguments) const
{
	if (!arguments.isArray())
	{
		return {"failed (Arguments for exist should be an array of strings.)"};
	}

	std::vector<std::string> exist_results;
	for (const Json::Value &argument : arguments)
	{
		const std::string object_name = argument.asString();
		const int object_id = mj_name2id(m, mjtObj::mjOBJ_BODY, object_name.c_str());
		exist_results.push_back(object_id != -1 ? "yes" : "no");
	}

	return exist_results;
}

std::string MjMultiverseClient::get_set_control_value_response(const Json::Value &arguments) const
{
	if (!arguments.isArray())
	{
		return "failed (Arguments for set_control_value should be an array of strings.)";
	}
	if (arguments.size() % 2 != 0)
	{
		return "failed (Arguments for set_control_value should be an array of strings with an even number of elements.)";
	}

	for (int i = 0; i < arguments.size(); i += 2)
	{
		if (!arguments[i].isString())
		{
			return "failed (Control name should be a string.)";
		}
		if (!arguments[i + 1].isString())
		{
			return "failed (Control value should be a number.)";
		}

		const std::string control_name = arguments[i].asString();
		const mjtNum control_value = std::stod(arguments[i + 1].asString());

		const int control_id = mj_name2id(m, mjtObj::mjOBJ_ACTUATOR, control_name.c_str());
		if (control_id == -1)
		{
			return "failed (Control " + control_name + " does not exist.)";
		}

		d->ctrl[control_id] = control_value;
		m->key_ctrl[control_id] = control_value;
	}

	return "success";
}

std::vector<std::string> MjMultiverseClient::get_get_bounding_box_response(const Json::Value &arguments) const
{
	if (!arguments.isArray() || (arguments.size() != 1 && arguments.size() != 2))
	{
		return {"failed (Arguments for get_bounding_box should be an array of strings with 1 or 2 elements.)"};
	}

	const std::string object_name = arguments[0].asString();
	const int body_id = mj_name2id(m, mjtObj::mjOBJ_BODY, object_name.c_str());
	if (body_id == -1)
	{
		return {"failed (Object " + object_name + " does not exist.)"};
	}

	std::set<int> body_ids = {body_id};
	bool with_children = false;
	if (arguments.size() == 2)
	{
		if (arguments[1].asString() != "with_children")
		{
			return {"failed (Second argument for get_contact_bodies should be \"with_children\".)"};
		}
		with_children = true;
	}

	if (with_children)
	{
		for (int child_body_id = body_id + 1; child_body_id < m->nbody; child_body_id++)
		{
			if (m->body_parentid[child_body_id] == body_id)
			{
				body_ids.insert(child_body_id);
			}
			else
			{
				break;
			}
		}
	}

	std::vector<std::string> bounding_box_results;
	for (const int &body_id : body_ids)
	{
		for (int geom_id = m->body_geomadr[body_id]; geom_id < m->body_geomadr[body_id] + m->body_geomnum[body_id]; geom_id++)
		{
			mjtNum *aabb = m->geom_aabb + 6 * geom_id;
			std::string bounding_box_result = "";
			for (int i = 0; i < 6; i++)
			{
				bounding_box_result += std::to_string(aabb[i]) + " ";
			}
			bounding_box_result.pop_back();
			bounding_box_results.push_back(bounding_box_result);
		}
	}

	return bounding_box_results;
}

void MjMultiverseClient::bind_api_callbacks()
{
	const Json::Value &api_callbacks_json = response_meta_data_json["api_callbacks"];
	for (const Json::Value &api_callback_json : api_callbacks_json)
	{
		for (const std::string &api_callback_name : api_callback_json.getMemberNames())
		{
			if (strcmp(api_callback_name.c_str(), "weld") == 0)
			{
				weld(api_callback_json[api_callback_name]);
			}
			else if (strcmp(api_callback_name.c_str(), "unweld") == 0)
			{
				unweld(api_callback_json[api_callback_name]);
			}
			else if (strcmp(api_callback_name.c_str(), "attach") == 0)
			{
				attach(api_callback_json[api_callback_name]);
			}
			else if (strcmp(api_callback_name.c_str(), "detach") == 0)
			{
				detach(api_callback_json[api_callback_name]);
			}
			else if (strcmp(api_callback_name.c_str(), "load") == 0)
			{
				load(api_callback_json[api_callback_name]);
			}
		}
	}
}

void MjMultiverseClient::bind_api_callbacks_response()
{
	const Json::Value &api_callbacks_json = response_meta_data_json["api_callbacks"];
	request_meta_data_json["api_callbacks_response"] = Json::arrayValue;
	for (const Json::Value &api_callback_json : api_callbacks_json)
	{
		for (const std::string &api_callback_name : api_callback_json.getMemberNames())
		{
			Json::Value api_callback_response;
			api_callback_response[api_callback_name] = Json::arrayValue;
			if (strcmp(api_callback_name.c_str(), "is_mujoco") == 0)
			{
				api_callback_response[api_callback_name].append("yes");
			}
			else if (strcmp(api_callback_name.c_str(), "weld") == 0)
			{
				const std::string weld_response = get_weld_response(api_callback_json[api_callback_name]);
				api_callback_response[api_callback_name].append(weld_response);
			}
			else if (strcmp(api_callback_name.c_str(), "unweld") == 0)
			{
				const std::string unweld_response = get_unweld_response(api_callback_json[api_callback_name]);
				api_callback_response[api_callback_name].append(unweld_response);
			}
			else if (strcmp(api_callback_name.c_str(), "attach") == 0)
			{
				const std::string attach_response = get_attach_response(api_callback_json[api_callback_name]);
				api_callback_response[api_callback_name].append(attach_response);
			}
			else if (strcmp(api_callback_name.c_str(), "detach") == 0)
			{
				const std::string detach_response = get_detach_response(api_callback_json[api_callback_name]);
				api_callback_response[api_callback_name].append(detach_response);
			}
			else if (strcmp(api_callback_name.c_str(), "get_contact_bodies") == 0)
			{
				for (const std::string &get_contact_body_response : get_get_contact_bodies_response(api_callback_json[api_callback_name]))
				{
					api_callback_response[api_callback_name].append(get_contact_body_response);
				}
			}
			else if (strcmp(api_callback_name.c_str(), "get_contact_points") == 0)
			{
				for (const std::string &get_contact_point_response : get_get_contact_points_response(api_callback_json[api_callback_name]))
				{
					api_callback_response[api_callback_name].append(get_contact_point_response);
				}
			}
			else if (strcmp(api_callback_name.c_str(), "get_contact_islands") == 0)
			{
				for (const std::string &get_contact_islands_response : get_get_contact_islands_response(api_callback_json[api_callback_name]))
				{
					api_callback_response[api_callback_name].append(get_contact_islands_response);
				}
			}
			else if (strcmp(api_callback_name.c_str(), "get_constraint_effort") == 0)
			{
				const std::string get_contact_effort_response = get_get_constraint_effort_response(api_callback_json[api_callback_name]);
				api_callback_response[api_callback_name].append(get_contact_effort_response);
			}
			else if (strcmp(api_callback_name.c_str(), "get_rays") == 0)
			{
				for (const std::string &get_rays_response : get_get_rays_response(api_callback_json[api_callback_name]))
				{
					api_callback_response[api_callback_name].append(get_rays_response);
				}
			}
			else if (strcmp(api_callback_name.c_str(), "exist") == 0)
			{
				for (const std::string &exist_response : get_exist_response(api_callback_json[api_callback_name]))
				{
					api_callback_response[api_callback_name].append(exist_response);
				}
			}
			else if (strcmp(api_callback_name.c_str(), "get_bounding_box") == 0)
			{
				for (const std::string &get_bounding_box_response : get_get_bounding_box_response(api_callback_json[api_callback_name]))
				{
					api_callback_response[api_callback_name].append(get_bounding_box_response);
				}
			}
			else if (strcmp(api_callback_name.c_str(), "set_control_value") == 0)
			{
				api_callback_response[api_callback_name].append(get_set_control_value_response(api_callback_json[api_callback_name]));
			}
			else if (strcmp(api_callback_name.c_str(), "pause") == 0)
			{
				pause = true;
				api_callback_response[api_callback_name].append("paused");
			}
			else if (strcmp(api_callback_name.c_str(), "unpause") == 0)
			{
				pause = false;
				api_callback_response[api_callback_name].append("unpaused");
			}
			else if (strcmp(api_callback_name.c_str(), "save") == 0)
			{
				const std::string save_response = get_save_response(api_callback_json[api_callback_name]);
				api_callback_response[api_callback_name].append(save_response);
			}
			else if (strcmp(api_callback_name.c_str(), "load") == 0)
			{
				const std::string load_response = get_load_response(api_callback_json[api_callback_name]);
				api_callback_response[api_callback_name].append(load_response);
			}
			else
			{
				api_callback_response[api_callback_name].append("not implemented");
			}
			request_meta_data_json["api_callbacks_response"].append(api_callback_response);
		}
	}
}

void MjMultiverseClient::init_send_and_receive_data()
{
	for (const std::pair<std::string, std::set<std::string>> &send_object : send_objects)
	{
		const int body_id = mj_name2id(m, mjtObj::mjOBJ_BODY, send_object.first.c_str());
		const int mocap_id = m->body_mocapid[body_id];
		const int joint_id = mj_name2id(m, mjtObj::mjOBJ_JOINT, send_object.first.c_str());
		if (body_id != -1)
		{
			const std::string body_name = send_object.first;
			if (mocap_id != -1)
			{
				for (const std::string &attribute_name : send_object.second)
				{
					if (strcmp(attribute_name.c_str(), "position") == 0)
					{
						send_data_vec.emplace_back(&d->mocap_pos[3 * mocap_id]);
						send_data_vec.emplace_back(&d->mocap_pos[3 * mocap_id + 1]);
						send_data_vec.emplace_back(&d->mocap_pos[3 * mocap_id + 2]);
					}
					else if (strcmp(attribute_name.c_str(), "quaternion") == 0)
					{
						send_data_vec.emplace_back(&d->mocap_quat[4 * mocap_id]);
						send_data_vec.emplace_back(&d->mocap_quat[4 * mocap_id + 1]);
						send_data_vec.emplace_back(&d->mocap_quat[4 * mocap_id + 2]);
						send_data_vec.emplace_back(&d->mocap_quat[4 * mocap_id + 3]);
					}
					else
					{
						printf("Send %s for %s not supported", attribute_name.c_str(), body_name.c_str());
					}
				}
			}
			else
			{
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
					else if (strcmp(attribute_name.c_str(), "force") == 0 &&
							 m->body_dofnum[body_id] == 6 &&
							 m->body_jntadr[body_id] != -1 &&
							 m->jnt_type[m->body_jntadr[body_id]] == mjtJoint::mjJNT_FREE)
					{
						if (contact_efforts.count(body_id) == 0)
						{
							contact_efforts[body_id] = (mjtNum *)calloc(6, sizeof(mjtNum));
						}

						send_data_vec.emplace_back(&contact_efforts[body_id][0]);
						send_data_vec.emplace_back(&contact_efforts[body_id][1]);
						send_data_vec.emplace_back(&contact_efforts[body_id][2]);
					}
					else if (strcmp(attribute_name.c_str(), "torque") == 0 &&
							 m->body_dofnum[body_id] == 6 &&
							 m->body_jntadr[body_id] != -1 &&
							 m->jnt_type[m->body_jntadr[body_id]] == mjtJoint::mjJNT_FREE)
					{
						if (contact_efforts.count(body_id) == 0)
						{
							contact_efforts[body_id] = (mjtNum *)calloc(6, sizeof(mjtNum));
						}

						send_data_vec.emplace_back(&contact_efforts[body_id][3]);
						send_data_vec.emplace_back(&contact_efforts[body_id][4]);
						send_data_vec.emplace_back(&contact_efforts[body_id][5]);
					}
					else if (strcmp(attribute_name.c_str(), "relative_velocity") == 0 &&
							 m->body_dofnum[body_id] == 6 &&
							 m->body_jntadr[body_id] != -1 &&
							 m->jnt_type[m->body_jntadr[body_id]] == mjtJoint::mjJNT_FREE)
					{
						send_data_vec.emplace_back(&d->qvel[dof_id]);
						send_data_vec.emplace_back(&d->qvel[dof_id + 1]);
						send_data_vec.emplace_back(&d->qvel[dof_id + 2]);
						send_data_vec.emplace_back(&d->qvel[dof_id + 3]);
						send_data_vec.emplace_back(&d->qvel[dof_id + 4]);
						send_data_vec.emplace_back(&d->qvel[dof_id + 5]);
					}
					else if (strcmp(attribute_name.c_str(), "odometric_velocity") == 0 &&
							 m->body_dofnum[body_id] <= 6 &&
							 m->body_jntadr[body_id] != -1)
					{
						odom_velocities[body_id] = (mjtNum *)calloc(6, sizeof(mjtNum));
						send_data_vec.emplace_back(&odom_velocities[body_id][0]);
						send_data_vec.emplace_back(&odom_velocities[body_id][1]);
						send_data_vec.emplace_back(&odom_velocities[body_id][2]);
						send_data_vec.emplace_back(&odom_velocities[body_id][3]);
						send_data_vec.emplace_back(&odom_velocities[body_id][4]);
						send_data_vec.emplace_back(&odom_velocities[body_id][5]);
					}
					else
					{
						printf("Send %s for %s not supported", attribute_name.c_str(), body_name.c_str());
					}
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
				if ((strcmp(attribute_name.c_str(), "joint_rvalue") == 0 &&
					 m->jnt_type[joint_id] == mjtJoint::mjJNT_HINGE) ||
					(strcmp(attribute_name.c_str(), "joint_tvalue") == 0 &&
					 m->jnt_type[joint_id] == mjtJoint::mjJNT_SLIDE))
				{
					send_data_vec.emplace_back(&d->qpos[qpos_id]);
				}
				else if ((strcmp(attribute_name.c_str(), "joint_angular_velocity") == 0 &&
						  m->jnt_type[joint_id] == mjtJoint::mjJNT_HINGE) ||
						 (strcmp(attribute_name.c_str(), "joint_linear_velocity") == 0 &&
						  m->jnt_type[joint_id] == mjtJoint::mjJNT_SLIDE))
				{
					send_data_vec.emplace_back(&d->qvel[dof_id]);
				}
				else if ((strcmp(attribute_name.c_str(), "joint_torque") == 0 &&
						  m->jnt_type[joint_id] == mjtJoint::mjJNT_HINGE) ||
						 (strcmp(attribute_name.c_str(), "joint_force") == 0 &&
						  m->jnt_type[joint_id] == mjtJoint::mjJNT_SLIDE))
				{
					send_data_vec.emplace_back(&d->qfrc_inverse[dof_id]);
				}
				else if (strcmp(attribute_name.c_str(), "joint_position") == 0)
				{
					printf("Send %s for %s not supported yet", attribute_name.c_str(), joint_name.c_str());
				}
				else if (strcmp(attribute_name.c_str(), "joint_quaternion") == 0 &&
						 m->jnt_type[joint_id] == mjtJoint::mjJNT_BALL)
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

	for (const std::pair<std::string, std::set<std::string>> &receive_object : receive_objects)
	{
		const int body_id = mj_name2id(m, mjtObj::mjOBJ_BODY, receive_object.first.c_str());
		const int mocap_id = m->body_mocapid[body_id];
		const int joint_id = mj_name2id(m, mjtObj::mjOBJ_JOINT, receive_object.first.c_str());
		const int actuator_id = mj_name2id(m, mjtObj::mjOBJ_ACTUATOR, receive_object.first.c_str());
		if (body_id != -1)
		{
			const std::string body_name = receive_object.first;
			if (mocap_id != -1)
			{
				for (const std::string &attribute_name : receive_object.second)
				{
					if (strcmp(attribute_name.c_str(), "position") == 0)
					{
						receive_data_vec.emplace_back(&d->mocap_pos[3 * mocap_id]);
						receive_data_vec.emplace_back(&d->mocap_pos[3 * mocap_id + 1]);
						receive_data_vec.emplace_back(&d->mocap_pos[3 * mocap_id + 2]);
					}
					else if (strcmp(attribute_name.c_str(), "quaternion") == 0)
					{
						receive_data_vec.emplace_back(&d->mocap_quat[4 * mocap_id]);
						receive_data_vec.emplace_back(&d->mocap_quat[4 * mocap_id + 1]);
						receive_data_vec.emplace_back(&d->mocap_quat[4 * mocap_id + 2]);
						receive_data_vec.emplace_back(&d->mocap_quat[4 * mocap_id + 3]);
					}
					else
					{
						printf("Send %s for %s not supported", attribute_name.c_str(), body_name.c_str());
					}
				}
			}
			else
			{
				const int dof_id = m->body_dofadr[body_id];
				for (const std::string &attribute_name : receive_object.second)
				{
					if (strcmp(attribute_name.c_str(), "position") == 0 &&
						m->body_dofnum[body_id] == 6 &&
						m->body_jntadr[body_id] != -1 &&
						m->jnt_type[m->body_jntadr[body_id]] == mjtJoint::mjJNT_FREE)
					{
						int qpos_id = m->jnt_qposadr[m->body_jntadr[body_id]];
						receive_data_vec.emplace_back(&d->qpos[qpos_id]);
						receive_data_vec.emplace_back(&d->qpos[qpos_id + 1]);
						receive_data_vec.emplace_back(&d->qpos[qpos_id + 2]);
					}
					else if (strcmp(attribute_name.c_str(), "quaternion") == 0)
					{
						if (m->body_dofnum[body_id] == 6 &&
							m->body_jntadr[body_id] != -1 &&
							m->jnt_type[m->body_jntadr[body_id]] == mjtJoint::mjJNT_FREE)
						{
							int qpos_id = m->jnt_qposadr[m->body_jntadr[body_id]];
							receive_data_vec.emplace_back(&d->qpos[qpos_id + 3]);
							receive_data_vec.emplace_back(&d->qpos[qpos_id + 4]);
							receive_data_vec.emplace_back(&d->qpos[qpos_id + 5]);
							receive_data_vec.emplace_back(&d->qpos[qpos_id + 6]);
						}
						else if (m->body_dofnum[body_id] == 3 &&
								 m->body_jntadr[body_id] != -1 &&
								 m->jnt_type[m->body_jntadr[body_id]] == mjtJoint::mjJNT_BALL)
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
							 m->body_dofnum[body_id] == 6 &&
							 m->body_jntadr[body_id] != -1 &&
							 m->jnt_type[m->body_jntadr[body_id]] == mjtJoint::mjJNT_FREE &&
							 odom_velocities.count(body_id) == 0)
					{
						receive_data_vec.emplace_back(&d->qvel[dof_id]);
						receive_data_vec.emplace_back(&d->qvel[dof_id + 1]);
						receive_data_vec.emplace_back(&d->qvel[dof_id + 2]);
						receive_data_vec.emplace_back(&d->qvel[dof_id + 3]);
						receive_data_vec.emplace_back(&d->qvel[dof_id + 4]);
						receive_data_vec.emplace_back(&d->qvel[dof_id + 5]);
					}
					else if (strcmp(attribute_name.c_str(), "odometric_velocity") == 0)
					{
						if (m->body_dofnum[body_id] <= 6 &&
							m->body_jntadr[body_id] != -1 &&
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
			}
		}
		else if (joint_id != -1)
		{
			const std::string joint_name = receive_object.first;
			const int qpos_id = m->jnt_qposadr[joint_id];
			const int dof_id = m->jnt_dofadr[joint_id];
			for (const std::string &attribute_name : receive_object.second)
			{
				if ((strcmp(attribute_name.c_str(), "cmd_joint_torque") == 0 && m->jnt_type[joint_id] == mjtJoint::mjJNT_HINGE) ||
					(strcmp(attribute_name.c_str(), "cmd_joint_force") == 0 && m->jnt_type[joint_id] == mjtJoint::mjJNT_SLIDE))
				{
					receive_data_vec.emplace_back(&d->qfrc_applied[dof_id]);
				}
				else
				{
					printf("Receive %s for %s not supported\n", attribute_name.c_str(), joint_name.c_str());
				}
			}
		}
		else if (actuator_id != -1)
		{
			const std::string actuator_name = receive_object.first;
			for (const std::string &attribute_name : receive_object.second)
			{
				if (strcmp(attribute_name.c_str(), "cmd_joint_rvalue") == 0 ||
					strcmp(attribute_name.c_str(), "cmd_joint_tvalue") == 0 ||
					strcmp(attribute_name.c_str(), "cmd_joint_angular_velocity") == 0 ||
					strcmp(attribute_name.c_str(), "cmd_joint_linear_velocity") == 0 ||
					strcmp(attribute_name.c_str(), "cmd_joint_torque") == 0 ||
					strcmp(attribute_name.c_str(), "cmd_joint_force") == 0)
				{
					receive_data_vec.emplace_back(&d->ctrl[actuator_id]);
				}
				else
				{
					printf("Receive %s for %s not supported\n", attribute_name.c_str(), actuator_name.c_str());
				}
			}
		}
	}
}

void MjMultiverseClient::bind_send_data()
{
	if (send_data_vec.size() != send_buffer.buffer_double.size)
	{
		printf("The size of send_data_vec (%zd) does not match with send_buffer_size (%zd).\n", send_data_vec.size(), send_buffer.buffer_double.size);
		return;
	}

	for (std::pair<const int, mjtNum *> &contact_effort : contact_efforts)
	{
		mjtNum *jac = mj_stackAllocNum(d, 6 * m->nv);
		mj_jacBodyCom(m, d, jac, jac + 3 * m->nv, contact_effort.first);
		mju_mulMatVec(contact_effort.second, jac, d->qfrc_constraint, 6, m->nv);
	}

	*world_time = d->time;
	for (size_t i = 0; i < send_buffer.buffer_double.size; i++)
	{
		send_buffer.buffer_double.data[i] = *send_data_vec[i];
	}
}

void MjMultiverseClient::bind_receive_data()
{
	*world_time = d->time + m->opt.timestep;
	if (receive_data_vec.size() != receive_buffer.buffer_double.size)
	{
		printf("[Client %s] The size of receive_data_vec (%zd) does not match with receive_buffer_size (%zd)\n", port.c_str(), receive_data_vec.size(), receive_buffer.buffer_double.size);
		return;
	}

	for (std::pair<const int, mjtNum *> &odom_velocity : odom_velocities)
	{
		const int body_id = odom_velocity.first;
		const int dof_adr = m->body_dofadr[body_id];
		if (m->body_dofnum[body_id] == 6 &&
			m->body_jntadr[body_id] != -1 &&
			m->jnt_type[m->body_jntadr[body_id]] == mjtJoint::mjJNT_FREE)
		{
			const int joint_adr = m->body_jntadr[body_id];
			const int qpos_adr = m->jnt_qposadr[joint_adr];

			const mjtNum w = (d->qpos + qpos_adr)[3];
			const mjtNum x = (d->qpos + qpos_adr)[4];
			const mjtNum y = (d->qpos + qpos_adr)[5];
			const mjtNum z = (d->qpos + qpos_adr)[6];

			const mjtNum sinr_cosp = 2 * (w * x + y * z);
			const mjtNum cosr_cosp = 1 - 2 * (x * x + y * y);
			mjtNum odom_ang_x_joint_pos = std::atan2(sinr_cosp, cosr_cosp);

			const mjtNum sinp = 2 * (w * y - z * x);
			mjtNum odom_ang_y_joint_pos;
			if (std::abs(sinp) >= 1)
			{
				odom_ang_y_joint_pos = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
			}
			else
			{
				odom_ang_y_joint_pos = std::asin(sinp);
			}

			const mjtNum siny_cosp = 2 * (w * z + x * y);
			const mjtNum cosy_cosp = 1 - 2 * (y * y + z * z);
			mjtNum odom_ang_z_joint_pos = std::atan2(siny_cosp, cosy_cosp);

			mjtNum *qvel = d->qvel + dof_adr;
			qvel[0] = odom_velocity.second[0] * mju_cos(odom_ang_y_joint_pos) * mju_cos(odom_ang_z_joint_pos) + odom_velocity.second[1] * (mju_sin(odom_ang_x_joint_pos) * mju_sin(odom_ang_y_joint_pos) * mju_cos(odom_ang_z_joint_pos) - mju_cos(odom_ang_x_joint_pos) * mju_sin(odom_ang_z_joint_pos)) + odom_velocity.second[2] * (mju_cos(odom_ang_x_joint_pos) * mju_sin(odom_ang_y_joint_pos) * mju_cos(odom_ang_z_joint_pos) + mju_sin(odom_ang_x_joint_pos) * mju_sin(odom_ang_z_joint_pos));
			qvel[1] = odom_velocity.second[0] * mju_cos(odom_ang_y_joint_pos) * mju_sin(odom_ang_z_joint_pos) + odom_velocity.second[1] * (mju_sin(odom_ang_x_joint_pos) * mju_sin(odom_ang_y_joint_pos) * mju_sin(odom_ang_z_joint_pos) + mju_cos(odom_ang_x_joint_pos) * mju_cos(odom_ang_z_joint_pos)) + odom_velocity.second[2] * (mju_cos(odom_ang_x_joint_pos) * mju_sin(odom_ang_y_joint_pos) * mju_sin(odom_ang_z_joint_pos) - mju_sin(odom_ang_x_joint_pos) * mju_cos(odom_ang_z_joint_pos));
			qvel[2] = odom_velocity.second[0] * mju_sin(odom_ang_y_joint_pos) + odom_velocity.second[1] * mju_sin(odom_ang_x_joint_pos) * mju_cos(odom_ang_y_joint_pos) + odom_velocity.second[2] * mju_cos(odom_ang_x_joint_pos) * mju_cos(odom_ang_y_joint_pos);
			qvel[3] = odom_velocity.second[3];
			qvel[4] = odom_velocity.second[4];
			qvel[5] = odom_velocity.second[5];
		}
		else if (m->body_dofnum[body_id] <= 6)
		{
			mjtNum odom_ang_x_joint_pos = 0.0;
			mjtNum odom_ang_y_joint_pos = 0.0;
			mjtNum odom_ang_z_joint_pos = 0.0;

			const int joint_adr = m->body_jntadr[body_id];
			for (int joint_id = joint_adr; joint_id < joint_adr + m->body_jntnum[body_id]; joint_id++)
			{
				const int qpos_adr = m->jnt_qposadr[joint_id];
				if (m->jnt_type[joint_id] == mjtJoint::mjJNT_HINGE)
				{
					if (mju_abs(m->jnt_axis[3 * joint_id] - 1.0) < mjMINVAL && mju_abs(m->jnt_axis[3 * joint_id + 1]) < mjMINVAL && mju_abs(m->jnt_axis[3 * joint_id + 2]) < mjMINVAL)
					{
						odom_ang_x_joint_pos = d->qpos[qpos_adr];
					}
					else if (mju_abs(m->jnt_axis[3 * joint_id + 1] - 1.0) < mjMINVAL && mju_abs(m->jnt_axis[3 * joint_id + 2]) < mjMINVAL && mju_abs(m->jnt_axis[3 * joint_id]) < mjMINVAL)
					{
						odom_ang_y_joint_pos = d->qpos[qpos_adr];
					}
					else if (mju_abs(m->jnt_axis[3 * joint_id + 2] - 1.0) < mjMINVAL && mju_abs(m->jnt_axis[3 * joint_id]) < mjMINVAL && mju_abs(m->jnt_axis[3 * joint_id + 1]) < mjMINVAL)
					{
						odom_ang_z_joint_pos = d->qpos[qpos_adr];
					}
				}
			}
			for (int joint_num = 0; joint_num < m->body_jntnum[body_id]; joint_num++)
			{
				const int joint_id = joint_adr + joint_num;
				if (m->jnt_type[joint_id] != mjtJoint::mjJNT_SLIDE && m->jnt_type[joint_id] != mjtJoint::mjJNT_HINGE)
				{
					continue;
				}
				const int dof_id = m->jnt_dofadr[joint_id];
				if (m->jnt_type[joint_id] == mjtJoint::mjJNT_SLIDE)
				{
					if (mju_abs(m->jnt_axis[3 * joint_id] - 1.0) < mjMINVAL && mju_abs(m->jnt_axis[3 * joint_id + 1]) < mjMINVAL && mju_abs(m->jnt_axis[3 * joint_id + 2]) < mjMINVAL)
					{
						d->qvel[dof_id] = odom_velocity.second[0] * mju_cos(odom_ang_y_joint_pos) * mju_cos(odom_ang_z_joint_pos) + odom_velocity.second[1] * (mju_sin(odom_ang_x_joint_pos) * mju_sin(odom_ang_y_joint_pos) * mju_cos(odom_ang_z_joint_pos) - mju_cos(odom_ang_x_joint_pos) * mju_sin(odom_ang_z_joint_pos)) + odom_velocity.second[2] * (mju_cos(odom_ang_x_joint_pos) * mju_sin(odom_ang_y_joint_pos) * mju_cos(odom_ang_z_joint_pos) + mju_sin(odom_ang_x_joint_pos) * mju_sin(odom_ang_z_joint_pos));
					}
					else if (mju_abs(m->jnt_axis[3 * joint_id + 1] - 1.0) < mjMINVAL && mju_abs(m->jnt_axis[3 * joint_id + 2]) < mjMINVAL && mju_abs(m->jnt_axis[3 * joint_id]) < mjMINVAL)
					{
						d->qvel[dof_id] = odom_velocity.second[0] * mju_cos(odom_ang_y_joint_pos) * mju_sin(odom_ang_z_joint_pos) + odom_velocity.second[1] * (mju_sin(odom_ang_x_joint_pos) * mju_sin(odom_ang_y_joint_pos) * mju_sin(odom_ang_z_joint_pos) + mju_cos(odom_ang_x_joint_pos) * mju_cos(odom_ang_z_joint_pos)) + odom_velocity.second[2] * (mju_cos(odom_ang_x_joint_pos) * mju_sin(odom_ang_y_joint_pos) * mju_sin(odom_ang_z_joint_pos) - mju_sin(odom_ang_x_joint_pos) * mju_cos(odom_ang_z_joint_pos));
					}
					else if (mju_abs(m->jnt_axis[3 * joint_id + 2] - 1.0) < mjMINVAL && mju_abs(m->jnt_axis[3 * joint_id]) < mjMINVAL && mju_abs(m->jnt_axis[3 * joint_id + 1]) < mjMINVAL)
					{
						d->qvel[dof_id] = odom_velocity.second[0] * mju_sin(odom_ang_y_joint_pos) + odom_velocity.second[1] * mju_sin(odom_ang_x_joint_pos) * mju_cos(odom_ang_y_joint_pos) + odom_velocity.second[2] * mju_cos(odom_ang_x_joint_pos) * mju_cos(odom_ang_y_joint_pos);
					}
				}
				else if (m->jnt_type[joint_id] == mjtJoint::mjJNT_HINGE)
				{
					if (mju_abs(m->jnt_axis[3 * joint_id] - 1.0) < mjMINVAL && mju_abs(m->jnt_axis[3 * joint_id + 1]) < mjMINVAL && mju_abs(m->jnt_axis[3 * joint_id + 2]) < mjMINVAL)
					{
						d->qvel[dof_id] = odom_velocity.second[3];
					}
					else if (mju_abs(m->jnt_axis[3 * joint_id + 1] - 1.0) < mjMINVAL && mju_abs(m->jnt_axis[3 * joint_id + 2]) < mjMINVAL && mju_abs(m->jnt_axis[3 * joint_id]) < mjMINVAL)
					{
						d->qvel[dof_id] = odom_velocity.second[4];
					}
					else if (mju_abs(m->jnt_axis[3 * joint_id + 2] - 1.0) < mjMINVAL && mju_abs(m->jnt_axis[3 * joint_id]) < mjMINVAL && mju_abs(m->jnt_axis[3 * joint_id + 1]) < mjMINVAL)
					{
						d->qvel[dof_id] = odom_velocity.second[5];
					}
				}
			}
		}
	}

	for (size_t i = 0; i < receive_buffer.buffer_double.size; i++)
	{
		*receive_data_vec[i] = receive_buffer.buffer_double.data[i];
	}
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

void MjMultiverseClient::reset()
{
	start_time += real_time;
	const int cursor_body_id = mj_name2id(m, mjtObj::mjOBJ_BODY, "cursor");
	const int cursor_id = m->body_mocapid[cursor_body_id];
	if (cursor_body_id != -1 && cursor_id != -1)
	{
		const mjtNum cursor_pos[3] = {d->mocap_pos[3 * cursor_id], d->mocap_pos[3 * cursor_id + 1], d->mocap_pos[3 * cursor_id + 2]};
		mj_resetDataKeyframe(m, d, 0);
		d->mocap_pos[3 * cursor_id] = cursor_pos[0];
		d->mocap_pos[3 * cursor_id + 1] = cursor_pos[1];
		d->mocap_pos[3 * cursor_id + 2] = cursor_pos[2];
	}
	else
	{
		mj_resetDataKeyframe(m, d, 0);
	}
	d->time = 0.0;
	mj_step(m, d);
}

bool MjMultiverseClient::communicate(const bool resend_meta_data)
{
	MjMultiverseClient::mutex.lock();
	const bool success = MultiverseClient::communicate(resend_meta_data);
	MjMultiverseClient::mutex.unlock();
	return success;
}