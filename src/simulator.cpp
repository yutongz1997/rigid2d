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


void Simulator::Render(const std::shared_ptr<TriangleMesh>& geometry) {
    geometry->Render(*shader_);
}


Simulator::Simulator(int width, int height) {
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

    std::string vert = "#version 410 core\n"
                       "layout (location = 0) in vec3 v_position;\n"
                       "void main() {\n"
                       "    gl_Position = vec4(v_position, 1.0f);\n"
                       "}";
    std::string frag = "#version 410 core\n"
                       "out vec4 frag_color;\n"
                       "void main() {\n"
                       "    frag_color = vec4(1.0f, 0.5f, 0.2f, 1.0f);\n"
                       "}";
    shader_ = std::make_unique<Shader>(vert, frag);
}


Simulator::~Simulator() {
    DestroyWindow();
    glfwTerminate();
}


void Simulator::Run(const std::shared_ptr<TriangleMesh>& geometry) {
    if (!window_) {
        return;
    }

    while (!glfwWindowShouldClose(window_)) {
        glfwPollEvents();

        ProcessKeyboardInput();

        glEnable(GL_DEPTH_TEST);
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        Render(geometry);

        glfwSwapBuffers(window_);
    }
}

RIGID2D_NAMESPACE_END
