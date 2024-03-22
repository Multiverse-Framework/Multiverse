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

#pragma once

#include "mj_model.h"

#include <GLFW/glfw3.h>

class MjVisual
{
public:
    MjVisual(const MjVisual &) = delete;

    void operator=(MjVisual const &) = delete;

    static MjVisual &get_instance()
    {
        static MjVisual mj_visual;
        return mj_visual;
    }

    /**
     * @brief Initialize the window
     *
     */
    void init();

    /**
     * @brief keyboard callback
     * 
     * @param window 
     * @param key 
     * @param scancode 
     * @param act 
     * @param mods 
     */
    static void keyboard(GLFWwindow *window, int key, int scancode, int act, int mods);

    /**
     * @brief mouse button callback
     *
     * @param window
     * @param button
     * @param act
     * @param mods
     */
    static void mouse_button(GLFWwindow *window, int button, int act, int mods);

    /**
     * @brief mouse move callback
     *
     * @param window
     * @param xpos
     * @param ypos
     */
    static void mouse_move(GLFWwindow *window, double xpos, double ypos);

    /**
     * @brief scroll callback
     *
     * @param window
     * @param xoffset
     * @param yoffset
     */
    static void scroll(GLFWwindow *window, double xoffset, double yoffset);

    /**
     * @brief Reload the model
     * 
     */
    static void reload_model();

    /**
     * @brief Return true if window is closed
     *
     */
    bool is_window_closed();

    /**
     * @brief Run the visualization
     * 
     */
    void run();

    /**
     * @brief Render the simulation with OpenGL
     *
     */
    void render();

    /**
     * @brief Free visualization storage
     *
     */
    void terminate();

public:
    GLFWwindow *window = NULL;

private:
    MjVisual() = default; // Singleton

    ~MjVisual();

private:
    static mjvCamera cam;  // abstract camera
    static mjvOption opt;  // visualization options
    static mjvScene scn;   // abstract scene
    static mjrContext con; // custom GPU context

    // mouse interaction
    static bool button_left;
    static bool button_middle;
    static bool button_right;
    static double lastx;
    static double lasty;

    // cursor data
    static int cursor_body_id;
    static int cursor_id;

    // init cam distance
    static mjtNum cam_distance_0;

    static double sim_start;
};