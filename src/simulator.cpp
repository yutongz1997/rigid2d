#include <string>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

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


void Simulator::InitImGui() {
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui_ImplGlfw_InitForOpenGL(window_, true);
    ImGui_ImplOpenGL3_Init();
}


void Simulator::DestroyImGui() {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
}


void Simulator::ShowControlPanel() {
    ImGui::Begin("Control Panel");

    ImGui::Checkbox("Wireframe", &wireframe_);
    ImGui::Checkbox("Draw BVH", &draw_bvh_);
    ImGui::Checkbox("Draw Contact Normal", &draw_contact_normal_);

    ImGui::InputFloat("Step Size", &dt_, 0.001f, 0.1f, "%.3f");
    if (ImGui::Button("Start / Pause")) {
        running_ = !running_;
    }
    ImGui::SameLine();
    if (ImGui::Button("Step")) {
        scene_->body_system_.Step(dt_);
    }
    ImGui::SameLine();
    if (ImGui::Button("Reset")) {
        running_ = false;
        scene_->body_system_.ResetState();
    }

    ImGui::End();
}


void Simulator::ProcessKeyboardInput() {
    ImGuiIO& io = ImGui::GetIO();
    if (ImGui::IsKeyDown(ImGuiKey_Escape)) {
        glfwSetWindowShouldClose(window_, true);
    } else if (ImGui::IsKeyDown(ImGuiKey_Space)) {
        running_ = !running_;
    } else if (ImGui::IsKeyDown(ImGuiKey_R)) {
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
    InitImGui();

    scene_ = std::make_unique<Scene>(scene_path);
    glfwSetWindowTitle(window_,
                       fmt::format("Simulation (Scene: {})", scene_->Name()).c_str());

    running_ = false;
    wireframe_ = false;
    draw_bvh_ = true;
    draw_contact_normal_ = true;
    dt_ = 0.005f;

    glm::mat4 ortho = glm::ortho(-4.0f, 4.0f, -3.0f, 3.0f, -1.0f, 1.0f);
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            ortho_(i, j) = ortho[i][j];
        }
    }
}


Simulator::~Simulator() {
    DestroyImGui();
    DestroyWindow();
    glfwTerminate();
}


void Simulator::Run() {
    if (!window_ || !scene_) {
        return;
    }
    // Only to display the "render is failed" message
    if (!scene_->IsValid()) {
        scene_->Render(ortho_, draw_bvh_, draw_contact_normal_);
        return;
    }

    while (!glfwWindowShouldClose(window_)) {
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();

        ProcessKeyboardInput();

        ImGui::NewFrame();
        ShowControlPanel();
        ImGui::EndFrame();

        glEnable(GL_DEPTH_TEST);
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glPolygonMode(GL_FRONT_AND_BACK, wireframe_ ? GL_LINE : GL_FILL);

        if (running_) {
            scene_->body_system_.Step(dt_);
        }
        scene_->Render(ortho_, draw_bvh_, draw_contact_normal_);
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSetWindowTitle(window_,
                           fmt::format("Simulation (Scene: {}) - {:.2f} s",
                                       scene_->Name(), scene_->body_system_.SimulationTime()).c_str());

        glfwSwapBuffers(window_);
    }
}

RIGID2D_NAMESPACE_END
