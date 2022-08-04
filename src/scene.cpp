#include <fstream>
#include <vector>
#include <map>
#include <memory>

#include <glad/glad.h>

#include "rigid2d/common.h"
#include "rigid2d/scene.h"


RIGID2D_NAMESPACE_BEGIN


Eigen::Vector2f JsonArray2ToVector2f(const scene_json& json_arr) {
    auto arr = json_arr.get<std::vector<float>>();
    assert(arr.size() >= 2);
    return { arr[0], arr[1] };
}


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
        const auto& vertex_buffer = meshes_[name]->VertexBufferData();
        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(vertex_buffer.size() * sizeof(GLfloat)),
                     &vertex_buffer[0], GL_STATIC_DRAW);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(GLfloat), nullptr);
        glEnableVertexAttribArray(0);
        const auto& index_buffer = meshes_[name]->IndexBufferData();
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(index_buffer.size() * sizeof(GLuint)),
                     &index_buffer[0], GL_STATIC_DRAW);
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


    std::string circ_vert = "#version 410 core\n"
                            "layout (location = 0) in vec2 v_position;\n"
                            "uniform mat4 model;\n"
                            "uniform mat4 projection;\n"
                            "void main() {\n"
                            "   gl_Position = projection * model * vec4(v_position, 0.0f, 1.0f);\n"
                            "}";
    std::string circ_frag = "#version 410 core\n"
                            "out vec4 frag_color;\n"
                            "void main() {\n"
                            "    frag_color = vec4(1.0f, 0.0f, 0.0f, 1.0f);\n"
                            "}";
    circ_shader_ = std::make_shared<Shader>(circ_vert, circ_frag, 's');
}


void Scene::LoadBodies() {
    std::vector<scene_json> json_bodies;
    descriptor_.at("bodies").get_to(json_bodies);

    for (auto& json_body : json_bodies) {
        std::map<std::string, scene_json> obj_body;
        json_body.get_to(obj_body);

        auto position = JsonArray2ToVector2f(json_body.at("position"));
        auto orientation = json_body.at("orientation").get<float>();
        float mass;
        if (json_body.contains("pinned") && json_body["pinned"].get<bool>()) {
            mass = kInf;
        } else {
            mass = json_body.at("mass").get<float>();
        }
        auto mesh_name = json_body.at("mesh").get<std::string>();
        auto shader_name = json_body.at("shader").get<std::string>();

        body_system_.AddBody(meshes_[mesh_name], shaders_[shader_name],
                             position, orientation, mass);
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
        LoadBodies();
        return true;
    } catch (const std::exception& e) {
        fmt::print(stderr,
                   "[Scene] Error: Failed to parse the scene description file '{}'. "
                   "Reason: {}.\n",
                   path, e.what());
        return false;
    }
}


Scene::Scene(const std::string &path) {
    valid_ = Load(path);
}


void Scene::Render(const Eigen::Matrix4f& ortho) {
    if (!valid_) {
        fmt::print(stderr,
                   "[Scene] Error: Failed to render as the scene is invalid.\n");
        return;
    }

    for (int i = 0; i < body_system_.bodies_.size(); ++i) {
        auto& body = body_system_.bodies_[i];
        body->shader_->Use();
        body->shader_->SetMatrix4f("model", body->local_to_world_.TransformationMatrix());
        body->shader_->SetMatrix4f("projection", ortho);
        body->shader_->SetVector3f("color", body_system_.colors_[i]);
        glBindVertexArray(vaos_[body->geometry_->Name()]);
        glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(body->geometry_->IndexBufferData().size()),
                       GL_UNSIGNED_INT, nullptr);
        glBindVertexArray(0);

        // body->bvh_root_->Render(circ_shader_, ortho);
        body->bvh_root_->RenderVisitBoundary(circ_shader_, ortho, body_system_.visit_id_);
    }
}

RIGID2D_NAMESPACE_END
