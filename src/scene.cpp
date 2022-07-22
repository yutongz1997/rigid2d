#include <fstream>
#include <vector>
#include <map>
#include <memory>

#include <glad/glad.h>

#include "rigid2d/common.h"
#include "rigid2d/scene.h"


RIGID2D_NAMESPACE_BEGIN

void Scene::LoadMeshes() {
    std::vector<scene_json> json_meshes;
    descriptor_.at("meshes").get_to(json_meshes);

    for (auto& json_mesh : json_meshes) {
        // Retrieve the JSON object of each triangle mesh
        std::map<std::string, scene_json> obj_mesh;
        json_mesh.get_to(obj_mesh);

        auto name = obj_mesh["name"].get<std::string>();
        auto path = obj_mesh["path"].get<std::string>();
        meshes_[name] = std::make_shared<TriangleMesh>(name, path);

        // Generate the OpenGL vertex array (VAO) and buffers (VBO, IBO)
        GLuint vao;
        glGenVertexArrays(1, &vao);
        GLuint vbo, ibo;
        glGenBuffers(1, &vbo);
        glGenBuffers(1, &ibo);
        // Populate vertices and indices data into the VBO and IBO
        const auto& buffer_vertices = meshes_[name]->VertexBufferData();
        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(buffer_vertices.size() * sizeof(GLfloat)),
                     &buffer_vertices[0], GL_STATIC_DRAW);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(GLfloat), nullptr);
        glEnableVertexAttribArray(0);
        const auto& buffer_indices = meshes_[name]->IndexBufferData();
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(buffer_indices.size() * sizeof(GLuint)),
                     &buffer_indices[0], GL_STATIC_DRAW);
        glBindVertexArray(0);
        // Store the generated VAO for future rendering
        vaos_[name] = vao;
    }
}


void Scene::LoadShaders() {
    std::vector<scene_json> json_shaders;
    descriptor_.at("shaders").get_to(json_shaders);

    for (auto& json_shader : json_shaders) {
        std::map<std::string, scene_json> obj_shader;
        json_shader.get_to(obj_shader);

        auto name = obj_shader["name"].get<std::string>();
        auto vertex_shader = obj_shader["vertex_path"].get<std::string>();
        auto fragment_shader = obj_shader["fragment_path"].get<std::string>();
        shaders_[name] = std::make_shared<Shader>(vertex_shader, fragment_shader, 'f');
    }
}


bool Scene::Load(const std::string &path) {
    // Read the scene description file into a JSON object
    std::ifstream file(path);
    file >> descriptor_;
    file.close();

    try {
        // Read the name of the scene
        name_ = descriptor_.at("name").get<std::string>();

        LoadMeshes();
        LoadShaders();

        return true;
    } catch (const std::exception& e) {
        fmt::print(stderr,
                   "[Scene] Error: Failed to parse the scene description file {}. "
                   "Reason: {}.\n",
                   path, e.what());
        return false;
    }
}


Scene::Scene(const std::string &path) {
    valid_ = Load(path);
}


void Scene::Render() {
    if (!valid_) {
        fmt::print(stderr,
                   "[Scene] Error: Failed to render as the scene is invalid.\n");
        return;
    }

    for (const auto& [name, mesh] : meshes_) {
        shaders_[name]->Use();
        glBindVertexArray(vaos_[name]);
        glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(mesh->NumVertices() * 2),
                       GL_UNSIGNED_INT, nullptr);
        glBindVertexArray(0);
    }
}

RIGID2D_NAMESPACE_END
