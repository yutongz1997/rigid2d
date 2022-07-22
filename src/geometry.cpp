#include <cstdio>
#include <fstream>
#include <vector>
#include <memory>
#include <algorithm>
#include <numeric>

#include <fmt/core.h>

#include <glad/glad.h>

#include "rigid2d/common.h"
#include "rigid2d/geometry.h"


RIGID2D_NAMESPACE_BEGIN

TriangleMesh::TriangleMesh(const std::string& name, const std::string& path) {
    name_ = name;
    loaded_ = true;
    vao_ = 0;

    std::ifstream file(path);
    std::string line;
    int line_count = 1;
    while (std::getline(file, line)) {
        // Skip empty lines
        if (line.empty()) {
            ++line_count;
            continue;
        }

        // Read the first token and the remaining part of the current line
        char *type = new char[line.length()];
        char *data = new char[line.length()];
        // Make sure the parsed line is in the correct format
        if (std::sscanf(line.c_str(), "%s%[^\n]", type, data) != 2) {
            fmt::print(stderr, "[TriangleMesh] Error: Cannot parse line {} of '{}'.\n",
                       line_count, name_);
            file.close();
            CleanUp();
            loaded_ = false;
            break;
        }

        // The current line contains a vertex
        if (std::strcmp(type, "v") == 0) {
            // Make sure the line contains at least 2 coordinates and read each of them
            GLfloat x, y;
            int num_coordinates = std::sscanf(data, "%f %f", &x, &y);
            if (num_coordinates != 2) {
                fmt::print(stderr,
                           "[TriangleMesh] Error: Incorrect number of vertex coordinates on line {} of '{}'. "
                           "Expected: >= 2, Received: {}.\n",
                           line_count, name_, num_coordinates);
                file.close();
                CleanUp();
                loaded_ = false;
                break;
            }
            // Record the new vertex coordinates
            buffer_vertices_.push_back(x);
            buffer_vertices_.push_back(y);
        }

            // The current line contains a face
        else if (std::strcmp(type, "f") == 0) {
            GLuint idx_v1, idx_v2, idx_v3;
            int num_vertices = std::sscanf(data, "%u %u %u", &idx_v1, &idx_v2, &idx_v3);
            if (num_vertices != 3) {
                fmt::print(stderr,
                           "[TriangleMesh] Error: Incorrect number of vertices on line {} of '{}'. "
                           "Expected: >= 3, Received: {}.\n",
                           line_count, name_, num_vertices);
                file.close();
                CleanUp();
                loaded_ = false;
                break;
            }
            // Record the new triangle vertex indices (also need to convert to 0-indexed)
            buffer_indices_.push_back(idx_v1 - 1);
            buffer_indices_.push_back(idx_v2 - 1);
            buffer_indices_.push_back(idx_v3 - 1);
        }

        ++line_count;
        delete[] type;
        delete[] data;
    }

    if (loaded_) {
        file.close();

        // Fill in arrays of vertices, triangles, triangle centroids and areas
        MakeVertices();
        MakeTriangles();
        centroids_ = std::vector<Eigen::Vector2f>(triangles_.size());
        std::transform(triangles_.begin(), triangles_.end(), centroids_.begin(),
                       [](const std::shared_ptr<Triangle> &trig) { return trig->centroid; });
        areas_ = std::vector<float>(triangles_.size());
        std::transform(triangles_.begin(), triangles_.end(), areas_.begin(),
                       [](const std::shared_ptr<Triangle> &trig) { return trig->area; });
        // Compute the total area of the triangle mesh
        area_ = std::accumulate(areas_.begin(), areas_.end(), 0.0f);

        // Generate the OpenGL vertex array (VAO) and buffers (VBO, IBO)
        glGenVertexArrays(1, &vao_);
        GLuint vbo, ibo;
        glGenBuffers(1, &vbo);
        glGenBuffers(1, &ibo);
        // Populate vertices and indices data into the VBO and IBO
        glBindVertexArray(vao_);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(buffer_vertices_.size() * sizeof(GLfloat)),
                     &buffer_vertices_[0], GL_STATIC_DRAW);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(GLfloat), nullptr);
        glEnableVertexAttribArray(0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(buffer_indices_.size() * sizeof(GLuint)),
                     &buffer_indices_[0], GL_STATIC_DRAW);
        glBindVertexArray(0);
    }
}


void TriangleMesh::MakeVertices() {
    std::size_t num_vertices = buffer_vertices_.size() / 2;
    vertices_ = std::vector<std::shared_ptr<Vertex>>(num_vertices);
    for (int i = 0; i < num_vertices; ++i) {
        float x = buffer_vertices_[2 * i];
        float y = buffer_vertices_[2 * i + 1];
        vertices_[i] = std::make_shared<Vertex>(x, y);
        vertices_[i]->index = i;
    }
}


void TriangleMesh::MakeTriangles() {
    for (int i = 0; i < buffer_indices_.size(); i += 3) {
        std::shared_ptr<Vertex> v1 = vertices_[buffer_indices_[i]];
        std::shared_ptr<Vertex> v2 = vertices_[buffer_indices_[i + 1]];
        std::shared_ptr<Vertex> v3 = vertices_[buffer_indices_[i + 2]];
        triangles_.push_back(std::make_shared<Triangle>(v1, v2, v3));
    }
}


void TriangleMesh::CleanUp() {
    name_ = "";
    loaded_ = false;

    vertices_.clear();
    triangles_.clear();
    centroids_.clear();
    areas_.clear();
    area_ = 0.0f;

    buffer_vertices_.clear();
    buffer_indices_.clear();
}


void TriangleMesh::Render(const Shader& shader) const {
    shader.Use();
    glBindVertexArray(vao_);
    glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(buffer_indices_.size()),
                   GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);
}

RIGID2D_NAMESPACE_END
