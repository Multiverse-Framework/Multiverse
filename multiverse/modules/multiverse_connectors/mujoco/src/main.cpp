// Copyright (c) 2023, Hoang Giang Nguyen - Institute for Artificial Intelligence, University Bremen

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

#include <GLFW/glfw3.h>
#include <cstdio>
#include <cstring>
#include <string>
#ifdef __linux__
#include <jsoncpp/json/json.h>
#include <jsoncpp/json/reader.h>
#elif _WIN32
#include <json/json.h>
#include <json/reader.h>
#endif
#include <algorithm>
#include <getopt.h>
#include <map>
#include <mujoco/mujoco.h>
#include <sstream>
#include <unistd.h>

// MuJoCo data structures
mjModel *m = NULL; // MuJoCo model
mjData *d = NULL;  // MuJoCo data
mjvCamera cam;     // abstract camera
mjvOption opt;     // visualization options
mjvScene scn;      // abstract scene
mjrContext con;    // custom GPU context

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx = 0.0;
double lasty = 0.0;

int cursor_body_id = -1;
int cursor_id = -1;
int cursor_site_id = -1;

mjtNum cursor_roll = 0.0;
mjtNum cursor_yaw = 0.0;

double cam_distance_0 = 2.0;

// keyboard callback
void keyboard(GLFWwindow *window, int key, int scancode, int act, int mods)
{
    // backspace: reset simulation
    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE)
    {
        mj_resetData(m, d);
        mj_forward(m, d);
        cam.distance = cam_distance_0;
        cam.elevation = m->vis.global.elevation;
        cam.azimuth = m->vis.global.azimuth;
    }

    // s: save simulation
    if (act == GLFW_PRESS && key == GLFW_KEY_S)
    {
        const char *filename = "save.mjb";
        printf("Save as %s\n", filename);
        mj_saveModel(m, filename, NULL, 1000);
    }
}

// mouse button callback
void mouse_button(GLFWwindow *window, int button, int act, int mods)
{
    // update button state
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    if (button_left || button_middle || button_right)
    {
        printf("Mouse pressed!\n");
        if (cursor_site_id != -1)
        {
            m->site_rgba[4 * cursor_site_id + 3] = 0.5;
        }
    }
    else
    {
        printf("Mouse released!\n");
        if (cursor_site_id != -1)
        {
            m->site_rgba[4 * cursor_site_id + 3] = 0;
        }
    }

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}

// mouse move callback
void mouse_move(GLFWwindow *window, double xpos, double ypos)
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

        // double cy = mju_cos(cursor_yaw * 0.5);
        // double sy = mju_sin(cursor_yaw * 0.5);
        // double cr = mju_cos(cursor_roll * 0.5);
        // double sr = mju_sin(cursor_roll * 0.5);

        // mjtNum quat[4] = {cr * cy, sr * cy, sr * sy, cr * sy};
        // d->mocap_quat[0] = quat[0];
        // d->mocap_quat[1] = quat[1];
        // d->mocap_quat[2] = quat[2];
        // d->mocap_quat[3] = quat[3];
    }
    else if (button_middle)
    {
        double offset =  0.828f;
        double dxx = dx / height * cam.distance * offset;
        double dyy = -dy / height * cam.distance * offset * mju_sin(cam.elevation / 180.f * M_PI);
        double dzz = dy / height * cam.distance * offset * mju_cos(cam.elevation / 180.f * M_PI) ;
        d->mocap_pos[0] += -dxx * mju_sin(cam.azimuth / 180.f * M_PI) + dyy * mju_cos(cam.azimuth / 180.f * M_PI);
        d->mocap_pos[1] += dxx * mju_cos(cam.azimuth / 180.f * M_PI) + dyy * mju_sin(cam.azimuth / 180.f * M_PI);
        d->mocap_pos[2] += dzz;
    }
}

// scroll callback
void scroll(GLFWwindow *window, double xoffset, double yoffset)
{
    cam.distance *= 1 + 0.05 * yoffset;
}

int main(int argc, char **argv)
{
    // print version, check compatibility
    printf("MuJoCo version %s\n", mj_versionString());

    // load and compile model
    char error[1000] = "Could not load binary model";
    if (argc > 1)
    {
        m = mj_loadXML(argv[1], 0, error, 1000);
    }
    if (!m)
    {
        mju_error("Load model error: %s", error);
    }

    cursor_body_id = mj_name2id(m, mjtObj::mjOBJ_BODY, "cursor");
    cursor_id = m->body_mocapid[cursor_body_id];
    cursor_site_id = mj_name2id(m, mjtObj::mjOBJ_SITE, "cursor");

    // make data
    d = mj_makeData(m);

    // init GLFW
    if (!glfwInit())
    {
        mju_error("Could not initialize GLFW");
    }

    // create window, make OpenGL context current, request v-sync
    GLFWwindow *window = glfwCreateWindow(960, 540, "MuJoCo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    cam.type = mjCAMERA_FREE;

    // mjtNum w = d->mocap_quat[4*cursor_id];
    // mjtNum x = d->mocap_quat[4*cursor_id + 1];
    // mjtNum y = d->mocap_quat[4*cursor_id + 2];
    // mjtNum z = d->mocap_quat[4*cursor_id + 3];

    // cam_elevation_0 = mju_atan2(2.f * (w * x + y * z), 1.f - 2.f * (x * x + y * y)) / M_PI * 180.f;
    // cam_azimuth_0 = mju_atan2(2.f * (w * z + x * y), 1.f - 2.f * (y * y + z * z)) / M_PI * 180.f + 90.f;

    cam.distance = cam_distance_0;
    cam.elevation = m->vis.global.elevation;
    cam.azimuth = m->vis.global.azimuth;

    // cursor_roll = cam.elevation / 180.f * M_PI;
    // cursor_yaw = cam.azimuth / 180.f * M_PI;

    // double cy = mju_cos(cursor_yaw * 0.5);
    // double sy = mju_sin(cursor_yaw * 0.5);
    // double cr = mju_cos(cursor_roll * 0.5);
    // double sr = mju_sin(cursor_roll * 0.5);

    // mjtNum quat[4] = {cr * cy, sr * cy, sr * sy, cr * sy};
    // d->mocap_quat[0] = quat[0];
    // d->mocap_quat[1] = quat[1];
    // d->mocap_quat[2] = quat[2];
    // d->mocap_quat[3] = quat[3];

    // create scene and context
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    // run main loop, target real-time simulation and 60 fps rendering
    while (!glfwWindowShouldClose(window))
    {
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        mjtNum simstart = d->time;
        while (d->time - simstart < 1.0 / 60.0)
        {
            mj_step(m, d);
        }

        cam.lookat[0] = d->mocap_pos[3*cursor_id];
        cam.lookat[1] = d->mocap_pos[3*cursor_id+1];
        cam.lookat[2] = d->mocap_pos[3*cursor_id+2];
        
        // mjtNum w = d->mocap_quat[4*cursor_id];
        // mjtNum x = d->mocap_quat[4*cursor_id + 1];
        // mjtNum y = d->mocap_quat[4*cursor_id + 2];
        // mjtNum z = d->mocap_quat[4*cursor_id + 3];

        // cam.elevation = mju_atan2(2.f * (w * x + y * z), 1.f - 2.f * (x * x + y * y)) / M_PI * 180.f;
        // cam.azimuth = mju_atan2(2.f * (w * z + x * y), 1.f - 2.f * (y * y + z * z)) / M_PI * 180.f + 90.f;
        
        // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        // update scene and render
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();
    }

    // free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model and data
    mj_deleteData(d);
    mj_deleteModel(m);

    return 0;
}