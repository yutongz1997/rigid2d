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

RIGID2D_NAMESPACE_END
