#ifndef RIGID2D_SCENE_H
#define RIGID2D_SCENE_H

#include <string>
#include <vector>
#include <map>
#include <unordered_map>
#include <memory>
#include <random>
#include <utility>

#include <fmt/core.h>

#include <nlohmann/json.hpp>

#include <glad/glad.h>

#include "rigid2d/common.h"
#include "rigid2d/shader.h"
#include "rigid2d/geometry.h"
#include "rigid2d/bodysystem.h"


RIGID2D_NAMESPACE_BEGIN

using scene_json = nlohmann::basic_json<
    std::map,        // Object type
    std::vector,     // Array type
    std::string,     // String type
    bool,            // Boolean type
    std::int32_t,    // Integer type
    std::uint32_t,   // Unsigned integer type
    float            // Floating number type
>;


class Scene {
private:
    // Scene descriptor in parsed JSON format
    scene_json descriptor_;

    // Name of the scene
    std::string name_;
    // Flag to indicate if the parsed scene is valid to simulate/render
    bool valid_;

    // Storage of underlying triangle meshes of each rigid body
    std::unordered_map<std::string, std::shared_ptr<TriangleMesh>> meshes_;
    // Storage of vertex array objects (VAOs) corresponding to each triangle mesh
    std::unordered_map<std::string, GLuint> vaos_;
    // Storage of shaders used to render each rigid body
    std::unordered_map<std::string, std::shared_ptr<Shader>> shaders_;
    //
    std::shared_ptr<Shader> circ_shader_;
    //
    std::shared_ptr<Shader> contact_shader_;
    // System of rigid bodies in the scene
    RigidBodySystem body_system_;

    friend class Simulator;

private:
    // Load all triangle meshes. Should only be called by Load()
    void LoadMeshes();
    // Load all vertex and fragment shaders. Should only be called by Load()
    void LoadShaders();
    // Load all rigid bodies. Should only be called by Load()
    void LoadBodies();

    // Load the given scene description file
    bool Load(const std::string& path);

public:
    explicit Scene(const std::string& path);

    [[nodiscard]] inline std::string Name() const {
        return name_;
    }

    [[nodiscard]] inline bool IsValid() const {
        return valid_;
    }

    void Render(const Eigen::Matrix4f& ortho,
                bool draw_bvh, bool draw_normal);
};

RIGID2D_NAMESPACE_END

#endif // RIGID2D_SCENE_H
