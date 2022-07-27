#include <string>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

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
    if (glfwGetKey(window_, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        glfwSetWindowShouldClose(window_, true);
    } else if (glfwGetKey(window_, GLFW_KEY_SPACE) == GLFW_PRESS) {
        running_ = !running_;
    } else if (glfwGetKey(window_, GLFW_KEY_R) == GLFW_PRESS) {
        running_ = false;
        scene_->body_system_.ResetState();
    }
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

    width_ = width;
    height_ = height;
    window_ = nullptr;
    InitWindow(width, height);

    scene_ = std::make_unique<Scene>(scene_path);
    glfwSetWindowTitle(window_,
                       fmt::format("Simulation (Scene: {})", scene_->Name()).c_str());
    running_ = false;

    glm::mat4 ortho = glm::ortho(-4.0f, 4.0f, -3.0f, 3.0f, -1.0f, 1.0f);
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            ortho_(i, j) = ortho[i][j];
        }
    }
}


Simulator::~Simulator() {
    DestroyWindow();
    glfwTerminate();
}


void Simulator::Run() {
    if (!window_ || !scene_) {
        return;
    }
    // Only to display the "render is failed" message
    if (!scene_->IsValid()) {
        scene_->Render(ortho_);
        return;
    }

    while (!glfwWindowShouldClose(window_)) {
        glfwPollEvents();

        ProcessKeyboardInput();

        glEnable(GL_DEPTH_TEST);
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        // glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

        if (running_) {
            scene_->body_system_.Step(0.01);
        }
        scene_->Render(ortho_);

        glfwSetWindowTitle(window_,
                           fmt::format("Simulation (Scene: {}) - {:.2f} s",
                                       scene_->Name(), scene_->body_system_.SimulationTime()).c_str());

        glfwSwapBuffers(window_);
    }
}

RIGID2D_NAMESPACE_END
