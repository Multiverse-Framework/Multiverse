// Copyright (c) 2022, Giang Hoang Nguyen - Institute for Artificial Intelligence, University Bremen

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

#include "mj_visual.h"

#include <iomanip>
#include <sstream>

mjvCamera MjVisual::cam;  // abstract camera
mjvOption MjVisual::opt;  // visualization options
mjvScene MjVisual::scn;   // abstract scene
mjrContext MjVisual::con; // custom GPU context

bool MjVisual::button_left = false;
bool MjVisual::button_middle = false;
bool MjVisual::button_right = false;
double MjVisual::lastx = 0;
double MjVisual::lasty = 0;

int MjVisual::cursor_body_id = -1;
int MjVisual::cursor_id = -1;

mjtNum MjVisual::cam_distance_0 = 10.0;

double MjVisual::sim_start = 0.0;

// allocate lists from https://github.com/deepmind/mujoco/blob/main/src/render/render_context.c
static void listAllocate(GLuint *base, GLsizei *range, GLsizei newrange)
{
    // allocate lists
    *range = newrange;
    if (newrange)
    {
        *base = glGenLists(*range);
        if (*base <= 0)
        {
            mju_error("Could not allocate display lists");
        }
    }
}

MjVisual::~MjVisual()
{
    terminate();
}

void MjVisual::init()
{
    // init GLFW
    if (!glfwInit())
    {
        mju_error("Could not initialize GLFW");
    }

    // create window, make OpenGL context current, request v-sync
    window = glfwCreateWindow(960, 540, scene_xml_path.stem().c_str(), NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);              // space for 2000 objects
    mjr_makeContext(m, &con, mjFONTSCALE_150); // model-specific context

    cam.type = mjCAMERA_FREE;
    cam.distance = MjVisual::cam_distance_0;
    cam.elevation = m->vis.global.elevation;
    cam.azimuth = m->vis.global.azimuth;

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, &MjVisual::keyboard);
    glfwSetCursorPosCallback(window, &MjVisual::mouse_move);
    glfwSetMouseButtonCallback(window, &MjVisual::mouse_button);
    glfwSetScrollCallback(window, &MjVisual::scroll);

    cursor_body_id = mj_name2id(m, mjtObj::mjOBJ_BODY, "cursor");
    cursor_id = m->body_mocapid[cursor_body_id];
}

void MjVisual::mouse_move(GLFWwindow *window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if (!button_left && !button_middle && !button_right)
    {
        return;
    }

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    if (button_left)
    {
        cam.elevation += (-dy / height) / M_PI * 180.f;
        cam.azimuth += (-dx / width) / M_PI * 180.f;
    }
    else if (button_middle)
    {
        if (cursor_id != -1 && mj_name2id(m, mjtObj::mjOBJ_SITE, "cursor_x") != -1 && mj_name2id(m, mjtObj::mjOBJ_SITE, "cursor_y") != -1 && mj_name2id(m, mjtObj::mjOBJ_SITE, "cursor_z") != -1)
        {
            double offset = 2.0 * cam.distance * mju_tan(m->vis.global.fovy / 360.0 * M_PI);
            double dxx = dx / height * offset;
            double dyy = -dy / height * offset * mju_sin(cam.elevation / 180.0 * M_PI);
            double dzz = dy / height * offset * mju_cos(cam.elevation / 180.0 * M_PI);
            d->mocap_pos[3 * cursor_id] += -dxx * mju_sin(cam.azimuth / 180.0 * M_PI) + dyy * mju_cos(cam.azimuth / 180.0 * M_PI);
            d->mocap_pos[3 * cursor_id + 1] += dxx * mju_cos(cam.azimuth / 180.0 * M_PI) + dyy * mju_sin(cam.azimuth / 180.0 * M_PI);
            d->mocap_pos[3 * cursor_id + 2] += dzz;
        }
    }
    else if (button_right)
    {
        cam.distance *= 1 - 0.5 * dy / height;
    }
}

void MjVisual::keyboard(GLFWwindow *window, int key, int scancode, int act, int mods)
{
    // backspace: reset simulation
    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE)
    {
        mtx.lock();
        d->time = 0.0;
        sim_start = 0.0;
        start_time += real_time;
        mj_resetDataKeyframe(m, d, 0);
        mj_forward(m, d);
        cam.distance = cam_distance_0;
        cam.elevation = m->vis.global.elevation;
        cam.azimuth = m->vis.global.azimuth;
        mtx.unlock();
    }

    // plus: speed up simulation
    if (act == GLFW_PRESS && key == GLFW_KEY_KP_ADD)
    {
        mtx.lock();
        rtf_desired *= 2;
        d->time = 0.0;
        start_time += real_time;
        mj_resetDataKeyframe(m, d, 0);
        mj_forward(m, d);
        cam.distance = cam_distance_0;
        cam.elevation = m->vis.global.elevation;
        cam.azimuth = m->vis.global.azimuth;
        mtx.unlock();
    }

    // minus: slow down simulation
    if (act == GLFW_PRESS && key == GLFW_KEY_KP_SUBTRACT)
    {
        mtx.lock();
        rtf_desired /= 2;
        d->time = 0.0;
        start_time += real_time;
        mj_resetDataKeyframe(m, d, 0);
        mj_forward(m, d);
        cam.distance = cam_distance_0;
        cam.elevation = m->vis.global.elevation;
        cam.azimuth = m->vis.global.azimuth;
        mtx.unlock();
    }

    // s: save simulation
    if (act == GLFW_PRESS && key == GLFW_KEY_S)
    {
        printf("Save as %s\n", scene_xml_path.c_str());
        char error[1000] = "Could not save XML model";
        mj_saveLastXML(scene_xml_path.c_str(), m, error, 1000);
    }
}

void MjVisual::mouse_button(GLFWwindow *window, int button, int act, int mods)
{
    // update button state
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    // if (button_left || button_middle || button_right)
    // {
    //     printf("Mouse pressed!\n");
    // }
    // else
    // {
    //     printf("Mouse released!\n");
    // }

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}

// scroll callback
void MjVisual::scroll(GLFWwindow *window, double xoffset, double yoffset)
{
    cam.distance *= 1 + 0.05 * yoffset;
}

bool MjVisual::is_window_closed()
{
    return glfwWindowShouldClose(window);
}

void MjVisual::run()
{
    sim_start = d->time;
    while (!stop)
    {
        if (d->time <= m->opt.timestep)
        {
            listAllocate(&con.baseMesh, &con.rangeMesh, 2 * m->nmesh);

            // process meshes
            for (int i = 0; i < m->nmesh; i++)
            {
                mjr_uploadMesh(m, &con, i);
            }
            for (int i = 0; i < m->ntex; i++)
            {
                mjr_uploadTexture(m, &con, i);
            }
        }

        if (is_window_closed())
        {
            stop = true;
            break;
        }

        if (d->time - sim_start > 1.0 / 60.0)
        {
            render();
            sim_start = d->time;
        }
    }
}

void MjVisual::render()
{
    if (cursor_id != -1)
    {
        cam.lookat[0] = d->mocap_pos[3 * cursor_id];
        cam.lookat[1] = d->mocap_pos[3 * cursor_id + 1];
        cam.lookat[2] = d->mocap_pos[3 * cursor_id + 2];
    }

    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    // update scene and render
    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);

    mjr_render(viewport, &scn, &con);

    if (cursor_id != -1)
    {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(4);
        oss << "[" << d->mocap_pos[3 * cursor_id] << ", " << d->mocap_pos[3 * cursor_id + 1] << ", " << d->mocap_pos[3 * cursor_id + 2] << "]";
        mjr_text(mjFONT_NORMAL, oss.str().c_str(), &con, 0.5, 0.5, 1, 1, 1);
    }

    // print simulation time
    mjrRect rect1 = {0, viewport.height - 50, 300, 50};
    mjrRect rect2 = {0, viewport.height - 100, 300, 50};
    mjrRect rect3 = {0, viewport.height - 150, 300, 50};
    mjrRect rect4 = {0, viewport.height - 200, 300, 50};
    mjrRect rect5 = {0, viewport.height - 250, 300, 50};
    mjrRect rect6 = {viewport.width - 100, 0, 100, 50};

    std::string real_time_text = "Real time: " + std::to_string(real_time);
    std::string sim_time_text = "Simulation time: " + std::to_string(d->time);
    std::string rtf_text = "Real-time factor: " + std::to_string(rtf);
    std::string time_step_text = "Time step: " + std::to_string(m->opt.timestep);
    std::string energy = "Total energy: " + std::to_string(d->energy[0] + d->energy[1]);
    std::ostringstream rtf_ss;
    rtf_ss << std::fixed << std::setprecision(2);
    rtf_ss << rtf_desired << " X";

    mjr_label(rect1, 0, real_time_text.c_str(), 1, 1, 1, 0.2, 1, 1, 1, &con);
    mjr_label(rect2, 0, sim_time_text.c_str(), 1, 1, 1, 0.2, 1, 1, 1, &con);
    mjr_label(rect3, 0, rtf_text.c_str(), 1, 1, 1, 0.2, 1, 1, 1, &con);
    mjr_label(rect4, 0, time_step_text.c_str(), 1, 1, 1, 0.2, 1, 1, 1, &con);
    mjr_label(rect5, 0, energy.c_str(), 1, 1, 1, 0.2, 1, 1, 1, &con);
    mjr_label(rect6, 0, rtf_ss.str().c_str(), 1, 1, 1, 0.2, 1, 1, 1, &con);

    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();
}

void MjVisual::terminate()
{
    // free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);
}
