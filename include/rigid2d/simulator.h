#ifndef RIGID2D_SIMULATOR_H
#define RIGID2D_SIMULATOR_H

#include <memory>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "rigid2d/common.h"
#include "rigid2d/scene.h"
#include "rigid2d/shader.h"
#include "rigid2d/geometry.h"


RIGID2D_NAMESPACE_BEGIN

class Simulator {
private:
    GLFWwindow* window_;

    std::unique_ptr<Scene> scene_;

    std::shared_ptr<Shader> circle_shader_;

private:
    bool InitWindow(int width, int height);
    void DestroyWindow();

    void ProcessKeyboardInput();

public:
    Simulator(int width, int height, const std::string& scene_path);

    ~Simulator();

    void Run();
};

RIGID2D_NAMESPACE_END

#endif // RIGID2D_SIMULATOR_H
