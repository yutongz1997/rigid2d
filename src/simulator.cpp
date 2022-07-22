#include <string>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <fmt/core.h>

#include "rigid2d/common.h"
#include "rigid2d/simulator.h"


RIGID2D_NAMESPACE_BEGIN

void FramebufferSizeCallback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
}


bool Simulator::InitWindow(int width, int height) {
    window_ = glfwCreateWindow(width, height, "Simulation", nullptr, nullptr);
    if (!window_) {
        fmt::print(stderr, "[Simulator] Error: Failed to create GLFW window.\n");
        glfwTerminate();
        return false;
    }
    glfwMakeContextCurrent(window_);
    glfwSetFramebufferSizeCallback(window_, FramebufferSizeCallback);

    // Initialize GLAD before calling any OpenGL function
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        fmt::print(stderr, "[Simulator] Error: Failed to initialize GLAD.\n");
        return false;
    }

    glViewport(0, 0, width, height);

    return true;
}


void Simulator::DestroyWindow() {
    glfwDestroyWindow(window_);
}


void Simulator::ProcessKeyboardInput() {

}


Simulator::Simulator(int width, int height, const std::string& scene_path) {
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
    glfwWindowHint(GLFW_SAMPLES, 4);
#if defined(__APPLE__)
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
#endif

    window_ = nullptr;
    InitWindow(width, height);

    scene_ = std::make_unique<Scene>(scene_path);
}


Simulator::~Simulator() {
    DestroyWindow();
    glfwTerminate();
}


void Simulator::Run() {
    if (!window_) {
        return;
    }

    while (!glfwWindowShouldClose(window_)) {
        glfwPollEvents();

        ProcessKeyboardInput();

        glEnable(GL_DEPTH_TEST);
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        scene_->Render();

        glfwSwapBuffers(window_);
    }
}

RIGID2D_NAMESPACE_END
