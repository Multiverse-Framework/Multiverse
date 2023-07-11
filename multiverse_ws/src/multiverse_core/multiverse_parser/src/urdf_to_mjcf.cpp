// Copyright (c) 2022, Hoang Giang Nguyen - Institute for Artificial Intelligence, University Bremen

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

#include "mujoco/mujoco.h"

#include <boost/filesystem.hpp>
#include <cctype>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <ros/package.h>
#include <urdf/model.h>

// help
const char helpstring[] =
    "\n Usage:  compile infile outfile\n"
    "   infile must be in urdf format\n"
    "   outfile must be in xml format\n\n"
    " Example:  compile model.urdf model.xml\n";

char error[1000];

std::string meshes_path_string;

// model and error
mjModel *m = 0;

// urdf model
urdf::Model model;

// deallocate and print message
int finish(const char *msg = 0)
{
    // deallocated everything
    if (m)
    {
        mj_deleteModel(m);
    }

    // print message
    if (msg)
    {
        std::printf("%s\n", msg);
    }

    return 0;
}

// possible file types
enum
{
    typeUNKNOWN = 0,
    typeURDF,
    typeXML
};

// determine file type
int filetype(const char *filename)
{
    // convert to lower case for string comparison
    char lower[1000];
    std::size_t i = 0;
    while (i < std::strlen(filename) && i < 999)
    {
        lower[i] = (char)tolower(filename[i]);
        i++;
    }
    lower[i] = 0;

    // find last dot
    int dot = (int)std::strlen(lower);
    while (dot >= 0 && lower[dot] != '.')
    {
        dot--;
    }

    // no dot found
    if (dot < 0)
    {
        return typeUNKNOWN;
    }

    // check extension
    if (!std::strcmp(lower + dot, ".xml"))
    {
        return typeXML;
    }
    else if (!std::strcmp(lower + dot, ".urdf"))
    {
        return typeURDF;
    }
    else
    {
        return typeUNKNOWN;
    }
}

// add_mujoco_tags
void add_mujoco_tags(const boost::filesystem::path &model_path)
{
    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(model_path.c_str()) != tinyxml2::XML_SUCCESS)
    {
        mju_error("Couldn't read file in [%s]\n", model_path.c_str());
    }

    tinyxml2::XMLElement *mujoco_element;
    if (model_path.extension().compare(".urdf") == 0)
    {
        if (doc.FirstChildElement("robot") != nullptr && doc.FirstChildElement("robot")->FirstChildElement("mujoco") != nullptr)
        {
            mujoco_element = doc.FirstChildElement("robot")->FirstChildElement("mujoco");
        }
        else
        {
            mujoco_element = doc.NewElement("mujoco");
        }

        doc.FirstChildElement("robot")->InsertFirstChild(mujoco_element);
    }
    else if (model_path.extension().compare(".xml") == 0)
    {
        if (doc.FirstChildElement("mujoco") != nullptr)
        {
            mujoco_element = doc.FirstChildElement("mujoco");
        }
    }

    tinyxml2::XMLElement *compiler_element = doc.NewElement("compiler");
    if (mujoco_element->FirstChildElement("compiler") != nullptr)
    {
        compiler_element = mujoco_element->FirstChildElement("compiler");
    }
    else
    {
        compiler_element = doc.NewElement("compiler");
        mujoco_element->LinkEndChild(compiler_element);
    }

    compiler_element->SetAttribute("fusestatic", false);
    compiler_element->SetAttribute("balanceinertia", true);
    compiler_element->SetAttribute("discardvisual", true);
    compiler_element->SetAttribute("boundmass", "0.000001");
    compiler_element->SetAttribute("boundinertia", "0.000001");

    if (model_path.extension().compare(".urdf") == 0)
    {
        compiler_element->SetAttribute("meshdir", model_path.parent_path().c_str());
        compiler_element->SetAttribute("strippath", false);

        for (tinyxml2::XMLElement *link_element = doc.FirstChildElement()->FirstChildElement("link");
             link_element != nullptr;
             link_element = link_element->NextSiblingElement("link"))
        {
            for (tinyxml2::XMLElement *collision_element = link_element->FirstChildElement("collision");
                 collision_element != nullptr;
                 collision_element = collision_element->NextSiblingElement("collision"))
            {
                for (tinyxml2::XMLElement *geometry_element = collision_element->FirstChildElement("geometry");
                     geometry_element != nullptr;
                     geometry_element = geometry_element->NextSiblingElement("geometry"))
                {
                    for (tinyxml2::XMLElement *mesh_element = geometry_element->FirstChildElement("mesh");
                         mesh_element != nullptr;
                         mesh_element = mesh_element->NextSiblingElement("mesh"))
                    {
                        boost::filesystem::path mesh_path = mesh_element->Attribute("filename");

                        mesh_element->SetAttribute("filename", mesh_path.filename().c_str());
                    }
                }
            }
        }
    }

    doc.SaveFile(model_path.c_str());
}

void add_robot_body(const boost::filesystem::path &model_path)
{
    tinyxml2::XMLDocument model_xml_doc;
    if (model_xml_doc.LoadFile(model_path.c_str()) != tinyxml2::XML_SUCCESS)
    {
        mju_warning("Failed to load file \"%s\"\n", model_path.c_str());
        return;
    }

    if (tinyxml2::XMLElement *worldbody_element = model_xml_doc.FirstChildElement()->FirstChildElement("worldbody"))
    {
        tinyxml2::XMLElement *robot_element = model_xml_doc.NewElement("body");

        robot_element->SetAttribute("name", model.getName().c_str());
        while (tinyxml2::XMLElement *body_element = worldbody_element->FirstChildElement())
        {
            robot_element->LinkEndChild(body_element);
        }
        worldbody_element->LinkEndChild(robot_element);
    }

    model_xml_doc.SaveFile(model_path.c_str());
}

void add_mimic_joints(const boost::filesystem::path &model_path)
{
    tinyxml2::XMLDocument model_xml_doc;
    if (model_xml_doc.LoadFile(model_path.c_str()) != tinyxml2::XML_SUCCESS)
    {
        mju_warning("Failed to load file \"%s\"\n", model_path.c_str());
        return;
    }

    tinyxml2::XMLElement *equality_element = model_xml_doc.NewElement("equality");
    model_xml_doc.FirstChildElement()->LinkEndChild(equality_element);

    for (const std::pair<std::string, urdf::JointSharedPtr> &joint : model.joints_)
    {
        if (joint.second->mimic != nullptr)
        {
            tinyxml2::XMLElement *joint_element = model_xml_doc.NewElement("joint");
            joint_element->SetAttribute("joint1", joint.first.c_str());
            joint_element->SetAttribute("joint2", joint.second->mimic->joint_name.c_str());
            joint_element->SetAttribute("polycoef",
                                        (std::to_string(joint.second->mimic->offset) + " " +
                                         std::to_string(joint.second->mimic->multiplier) + " " +
                                         " 0 0 0")
                                            .c_str());
            equality_element->LinkEndChild(joint_element);
        }
    }

    model_xml_doc.SaveFile(model_path.c_str());
}

void disable_parent_child_collision(const boost::filesystem::path &model_path, const int disable_parent_child_collision_level)
{
    tinyxml2::XMLDocument model_xml_doc;
    if (model_xml_doc.LoadFile(model_path.c_str()) != tinyxml2::XML_SUCCESS)
    {
        mju_warning("Failed to load file \"%s\"\n", model_path.c_str());
        return;
    }

    tinyxml2::XMLElement *contact_element = model_xml_doc.NewElement("contact");
    model_xml_doc.FirstChildElement()->LinkEndChild(contact_element);

    if (disable_parent_child_collision_level >= 0)
    {
        for (int body_id = 1; body_id < m->nbody; body_id++)
        {
            int body_parent_id = body_id;
            const std::string body_name = mj_id2name(m, mjOBJ_BODY, body_id);
            for (int k = 0; k < disable_parent_child_collision_level; k++)
            {
                body_parent_id = m->body_parentid[body_parent_id];

                tinyxml2::XMLElement *exclude_element = model_xml_doc.NewElement("exclude");
                if (body_parent_id == 0)
                {
                    exclude_element->SetAttribute("body1", model.getName().c_str());
                }
                else
                {
                    const std::string parent_name = mj_id2name(m, mjOBJ_BODY, body_parent_id);
                    exclude_element->SetAttribute("body1", parent_name.c_str());
                }
                exclude_element->SetAttribute("body2", body_name.c_str());
                contact_element->LinkEndChild(exclude_element);
                if (body_parent_id == 0)
                {
                    break;
                }
            }
        }
    }
    else if (disable_parent_child_collision_level < m->nbody)
    {
        mju_warning("Disable self collision of %s", model.getName().c_str());
        for (int body_1_id = 0; body_1_id < m->nbody; body_1_id++)
        {
            for (int body_2_id = body_1_id + 1; body_2_id < m->nbody; body_2_id++)
            {
                tinyxml2::XMLElement *exclude_element = model_xml_doc.NewElement("exclude");
                if (body_1_id == 0)
                {
                    exclude_element->SetAttribute("body1", model.getName().c_str());
                }
                else
                {
                    exclude_element->SetAttribute("body1", mj_id2name(m, mjOBJ_BODY, body_1_id));
                }
                exclude_element->SetAttribute("body2", mj_id2name(m, mjOBJ_BODY, body_2_id));
                contact_element->LinkEndChild(exclude_element);
            }
        }
    }

    model_xml_doc.SaveFile(model_path.c_str());
}

// modify input file
void load_urdf(const char *input, const char *output)
{
    boost::filesystem::path input_file_path = input;
    boost::filesystem::path output_file_path = output;

    if (!model.initFile(input_file_path.c_str()))
    {
        ROS_ERROR("Couldn't read file in [%s]\n", input);
    }

    meshes_path_string = output_file_path.stem().string() + "/stl";

    boost::filesystem::create_directories(output_file_path.parent_path() / meshes_path_string);
    boost::filesystem::path meshes_path = output_file_path.parent_path() / meshes_path_string;

    boost::filesystem::path model_urdf_path = meshes_path / input_file_path.filename();
    if (boost::filesystem::exists(model_urdf_path))
    {
        ROS_INFO("File [%s] exists, replace...", model_urdf_path.c_str());
        boost::filesystem::remove(model_urdf_path);
    }
    boost::filesystem::copy_file(input_file_path, model_urdf_path);

    std::vector<urdf::LinkSharedPtr> links;
    model.getLinks(links);
    for (const urdf::LinkSharedPtr &link : links)
    {
        for (const urdf::VisualConstSharedPtr &visual : link->visual_array)
        {
            if (visual->geometry->type == urdf::Geometry::MESH)
            {
                urdf::Mesh &mesh = dynamic_cast<urdf::Mesh &>(*visual->geometry);
                mesh.filename.erase(0, 9);
                boost::filesystem::path mesh_path = mesh.filename;

                std::string file_name = mesh_path.filename().string();
                boost::filesystem::path ros_pkg = mesh_path;
                while (ros_pkg.has_parent_path() && ros_pkg.parent_path().has_parent_path())
                {
                    ros_pkg = ros_pkg.parent_path();
                }
                ros_pkg = ros_pkg.relative_path();
                boost::filesystem::path ros_pkg_path = ros::package::getPath(ros_pkg.string());
                mesh_path = ros_pkg_path.parent_path() / mesh_path.string();

                if (boost::filesystem::exists(meshes_path / file_name))
                {
                    ROS_INFO("File [%s] from [%s] already exists in [%s], ignore", file_name.c_str(), mesh_path.c_str(), meshes_path.c_str());
                }
                else
                {
                    boost::filesystem::copy_file(mesh_path, meshes_path / mesh_path.filename());
                }
            }
        }
        for (const urdf::CollisionSharedPtr &collision : link->collision_array)
        {
            if (collision->geometry->type == urdf::Geometry::MESH)
            {
                urdf::Mesh &mesh = dynamic_cast<urdf::Mesh &>(*collision->geometry);
                mesh.filename.erase(0, 9);
                boost::filesystem::path mesh_path = mesh.filename;
                std::string file_name = mesh_path.filename().string();
                boost::filesystem::path ros_pkg = mesh_path;
                while (ros_pkg.has_parent_path() && ros_pkg.parent_path().has_parent_path())
                {
                    ros_pkg = ros_pkg.parent_path();
                }
                ros_pkg = ros_pkg.relative_path();
                boost::filesystem::path ros_pkg_path = ros::package::getPath(ros_pkg.string());
                mesh_path = ros_pkg_path.parent_path() / mesh_path.string();

                if (boost::filesystem::exists(meshes_path / file_name))
                {
                    ROS_INFO("File [%s] from [%s] already exists in [%s], ignore", file_name.c_str(), mesh_path.c_str(), meshes_path.c_str());
                }
                else
                {
                    boost::filesystem::copy_file(mesh_path, meshes_path / mesh_path.filename());
                }
            }
        }
    }

    add_mujoco_tags(model_urdf_path);

    // load model
    m = mj_loadXML(model_urdf_path.c_str(), 0, error, 1000);
}

// main function
int main(int argc, char **argv)
{
    ros::init(argc, argv, "urdf_to_mjcf");

    // print help if arguments are missing
    if (argc < 2)
    {
        return finish(helpstring);
    }

    // determine file types
    int type1 = filetype(argv[1]);
    int type2;
    std::string output;
    int disable_parent_child_collision_level = 1;
    if (argc == 2)
    {
        type2 = typeXML;
        boost::filesystem::path input_file_path = argv[1];
        output = ros::package::getPath("mujoco_sim") + "/model/tmp/" + input_file_path.stem().string() + ".xml";
    }
    else
    {
        if (atoi(argv[2]) != 0 || strcmp(argv[2], "0") == 0)
        {
            type2 = typeXML;
            boost::filesystem::path input_file_path = argv[1];
            output = ros::package::getPath("mujoco_sim") + "/model/tmp/" + input_file_path.stem().string() + ".xml";
            disable_parent_child_collision_level = atoi(argv[2]);
        }
        else
        {
            type2 = filetype(argv[2]);
            output = argv[2];
            if (argc == 4)
            {
                disable_parent_child_collision_level = atoi(argv[3]);
            }
        }
    }

    // check types
    if (type1 != typeURDF || type2 != typeXML)
    {
        return finish("Illegal combination of file formats");
    }

    // override output file if it exists
    if (boost::filesystem::exists(output))
    {
        boost::filesystem::remove(output);
    }

    load_urdf(argv[1], output.c_str());

    // check error
    if (!m)
    {
        return finish(error);
    }

    // save model
    if (!mj_saveLastXML(output.c_str(), m, error, 1000))
    {
        return finish(error);
    }

    add_robot_body(output);

    add_mimic_joints(output);

    disable_parent_child_collision(output, disable_parent_child_collision_level);

    add_mujoco_tags(output);

    // finalize
    return finish("Done");
}
