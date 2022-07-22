#ifndef RIGID2D_SIMULATOR_H
#define RIGID2D_SIMULATOR_H

#include <memory>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "rigid2d/common.h"
#include "rigid2d/shader.h"
#include "rigid2d/geometry.h"


RIGID2D_NAMESPACE_BEGIN

class Simulator {
private:
    GLFWwindow* window_;

    std::unique_ptr<Shader> shader_;

    std::shared_ptr<TriangleMesh> geometry_;

private:
    bool InitWindow(int width, int height);
    void DestroyWindow();

    void ProcessKeyboardInput();

    void Render(const std::shared_ptr<TriangleMesh>& geometry);

public:
    Simulator(int width, int height);

    ~Simulator();

    void Run(const std::shared_ptr<TriangleMesh>& geometry);
};

RIGID2D_NAMESPACE_END

#endif // RIGID2D_SIMULATOR_H
