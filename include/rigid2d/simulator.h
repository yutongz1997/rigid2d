#ifndef RIGID2D_SIMULATOR_H
#define RIGID2D_SIMULATOR_H

#include <memory>

#include <Eigen/Core>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "rigid2d/common.h"
#include "rigid2d/scene.h"
#include "rigid2d/shader.h"
#include "rigid2d/geometry.h"


RIGID2D_NAMESPACE_BEGIN

class Simulator {
private:
    int width_;
    int height_;
    GLFWwindow* window_;

    std::unique_ptr<Scene> scene_;
    Eigen::Matrix4f ortho_;

    bool running_;
    bool wireframe_;
    bool draw_bvh_;
    bool draw_contact_normal_;
    float dt_;

private:
    bool InitWindow(int width, int height);
    void DestroyWindow();

    void InitImGui();
    void DestroyImGui();

    void ShowControlPanel();

    void ProcessKeyboardInput();

public:
    Simulator(int width, int height, const std::string& scene_path);

    ~Simulator();

    void Run();
};

RIGID2D_NAMESPACE_END

#endif // RIGID2D_SIMULATOR_H
