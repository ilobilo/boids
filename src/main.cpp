// Copyright (C) 2024 ilobilo

#include <fmt/core.h>
#include <cstdlib>
#include <cmath>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <GLFW/glfw3.h>

#include <trackball.hpp>
#include <boids.hpp>

#define CAMERA_ARGS { 0, 0, 0 }, 15
trackball::camera camera { CAMERA_ARGS };

enum targets
{
    first_boid = 0,
    random_boid = 1
};
int target = first_boid;
bool follow = false;

boid *target_boid = nullptr;

boids boids;
int window_w, window_h;

boid *get_random_boid()
{
    auto &lst = boids.boids_list;
    std::uniform_int_distribution<> dist(0, lst.size() - 1);

    auto start = lst.begin();
    std::advance(start, dist(boids.re));

    return std::addressof((target == first_boid) ? boids.boids_list.front() : *start);
}

void set_target_boid()
{
    if (target == first_boid)
        target_boid = std::addressof(boids.boids_list.front());
    else if (target == random_boid)
        target_boid = get_random_boid();
    else
        assert(false && "WHYY");
}

namespace callbacks
{
    void error(int error, const char *description)
    {
        fmt::println(stderr, "GLFW Error {}: {}", error, description);
    }

    void scroll(GLFWwindow *, double, double yoffset)
    {
        if (yoffset != 0)
            camera.zoom(-yoffset);
    }

    double mouse_x = 0, mouse_y = 0;
    int mouse_buttons[GLFW_MOUSE_BUTTON_LAST + 1];
    void mousebutton(GLFWwindow *, int button, int action, int)
    {
        auto &io = ImGui::GetIO();
        if (io.WantCaptureMouse)
            return;

        mouse_buttons[button] = action;
    }

    void mousemotion(GLFWwindow *, double xpos, double ypos)
    {
        auto &io = ImGui::GetIO();
        if (io.WantCaptureMouse)
            return;

        auto dxn = (xpos - mouse_x) / window_w;
        auto dyn = (ypos - mouse_y) / window_h;

        mouse_x = xpos;
        mouse_y = ypos;

        if (mouse_buttons[GLFW_MOUSE_BUTTON_LEFT] == GLFW_PRESS)
            camera.rotate(dxn, dyn);

        if (mouse_buttons[GLFW_MOUSE_BUTTON_RIGHT] == GLFW_PRESS && follow == false)
            camera.shift_centre(dxn, dyn);
    }
} // namespace callbacks

GLFWwindow *init_glfw()
{
    glfwSetErrorCallback(callbacks::error);
    if (!glfwInit())
    {
        fmt::println(stderr, "Could not initialise GLFW");
        return nullptr;
    }

    auto videomode = glfwGetVideoMode(glfwGetPrimaryMonitor());

    auto *window = glfwCreateWindow(videomode->width / 3 * 2, videomode->height / 3 * 2, "Boids", nullptr, nullptr);
    if (window == nullptr)
    {
        fmt::println(stderr, "Could not create GLFW window");
        glfwTerminate();
        return nullptr;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync

    for (auto &button : callbacks::mouse_buttons)
        button = GLFW_RELEASE;

    glfwSetScrollCallback(window, callbacks::scroll);
    glfwSetMouseButtonCallback(window, callbacks::mousebutton);
    glfwSetCursorPosCallback(window, callbacks::mousemotion);

    return window;
}

auto main() -> int
{
    auto window = init_glfw();
    if (window == nullptr)
        return EXIT_FAILURE;

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();

    auto &io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;

    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init();

    glEnable(GL_DEPTH_TEST | GL_LIGHTING | GL_LIGHT0 | GL_COLOR_MATERIAL | GL_NORMALIZE);
    glShadeModel(GL_SMOOTH);

    auto before = glfwGetTime();
    while (!glfwWindowShouldClose(window))
    {
        auto now = glfwGetTime();
        auto elapsed = now - before;

        if (elapsed * trackball::fps >= 1.0)
        {
            ImGui_ImplOpenGL3_NewFrame();
            ImGui_ImplGlfw_NewFrame();
            ImGui::NewFrame();
            {
                glfwGetWindowSize(window, &window_w, &window_h);
                glViewport(0, 0, window_w, window_h);

                glClearColor(0.18f, 0.18f, 0.18f, 1.0f);
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                ImGui_ImplGlfw_ScrollCallback(window, 0.0, 0.0);

                glMatrixMode(GL_PROJECTION);
                glLoadIdentity();

                const GLdouble fh = std::tan(trackball::fovy / 360 * M_PI) * trackball::nearclip;
                const GLdouble fw = fh * static_cast<double>(window_w) / static_cast<double>(window_h);
                glFrustum(-fw, fw, -fh, fh, trackball::nearclip, trackball::farclip);

                ImGui::Begin("Controls");
                {
                    ImGui::SliderFloat("Neighbour Distance", &boids::neighbourhood_max_dist, 0.0f, 12.f);
                    ImGui::SliderFloat("Separation", &boids::separation_factor, 0.0f, 0.1f);
                    ImGui::SliderFloat("Cohesion", &boids::cohesion_factor, 0.0f, 0.1f);
                    ImGui::SliderFloat("Alignment", &boids::alignment_factor, 0.0f, 0.02f);
                    ImGui::SliderFloat("Randomness", &boids::randomness, 0.0f, 2.0f);
                    ImGui::SliderFloat("Min cos angle", &boids::min_cos_angle, -1.f, 1.f);
                    ImGui::SliderFloat("Max speed", &boids::max_speed, 0.0f, 20.0f);
                    ImGui::SliderInt("Reset num boids", &num_boids, 2, 2000);

                    if (ImGui::RadioButton("First", &target, first_boid) || target_boid == nullptr)
                        target_boid = std::addressof(boids.boids_list.front());

                    ImGui::SameLine();
                    if (ImGui::RadioButton("Random", &target, random_boid))
                        target_boid = get_random_boid();

                    ImGui::Checkbox("Follow", &follow);

                    if (ImGui::Button("Recentre") || follow)
                        camera.recentre(target_boid->position);

                    ImGui::SameLine();

                    if (ImGui::Button("Add"))
                        boids.add();

                    ImGui::SameLine();

                    if (ImGui::Button("Reset"))
                    {
                        boids.reset();
                        camera.reset(CAMERA_ARGS);

                        set_target_boid();
                    }
                }
                ImGui::End();

                camera.lookat();
                boids.update(elapsed);

                before = now;
            }
            ImGui::Render();
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

            glfwSwapBuffers(window);
            glfwPollEvents();
        }
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    return EXIT_SUCCESS;
}
