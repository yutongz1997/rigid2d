#include <cstdio>
#include <fstream>
#include <vector>
#include <memory>
#include <algorithm>
#include <numeric>
#include <random>

#include <fmt/core.h>

#include <glad/glad.h>

#include "rigid2d/common.h"
#include "rigid2d/geometry.h"


RIGID2D_NAMESPACE_BEGIN

Circle TwoVerticesCircle(const std::shared_ptr<Vertex>& v1,
                         const std::shared_ptr<Vertex>& v2) {
    Eigen::Vector2f center { 0.5f * (v1->x + v2->x),
                             0.5f * (v1->y + v2->y) };
    float radius = v1->distance(center);
    return { center, radius };
}


Circle ThreeVerticesCircle(const std::shared_ptr<Vertex>& v1,
                           const std::shared_ptr<Vertex>& v2,
                           const std::shared_ptr<Vertex>& v3) {
    float a = v2->x - v1->x;
    float b = v2->y - v1->y;
    float c = v3->x - v1->x;
    float d = v3->y - v1->y;
    float e = a * (v2->x + v1->x) * 0.5f + b * (v2->y + v1->y) * 0.5f;
    float f = c * (v3->x + v1->x) * 0.5f + d * (v3->y + v1->y) * 0.5f;
    float det = a * d - b * c;
    // In case the three given vertices are co-linear, the enclosing circle is
    // degenerate and only passes through two vertices
    if (det == 0) {
        float d12 = v1->distance(*v2);
        float d23 = v2->distance(*v3);
        float d13 = v1->distance(*v3);
        if (d12 > d23 && d12 > d13) {
            return TwoVerticesCircle(v1, v2);
        } else if (d23 > d13) {
            return TwoVerticesCircle(v2, v3);
        }
        return TwoVerticesCircle(v1, v3);
    } else {
        Eigen::Vector2f center { (d * e - b * f) / det,
                                 (-c * e + a * f) / det };
        float radius = v1->distance(center);
        return { center, radius };
    }
}

Circle MinimumEnclosingCircle(const std::vector<std::shared_ptr<Vertex>>& inputs,
                              std::size_t num_inputs,
                              std::vector<std::shared_ptr<Vertex>>& support,
                              std::size_t support_size) {
    // Compute an exact circle in terminal cases
    if (support_size == 3) {
        return ThreeVerticesCircle(support[0], support[1], support[2]);
    } else if (num_inputs == 1 && support_size == 0) {
        Eigen::Vector2f center { inputs[0]->x, inputs[0]->y };
        return { center, 0.0f };
    } else if (num_inputs == 0 && support_size == 2) {
        return TwoVerticesCircle(support[0], support[1]);
    } else if (num_inputs == 1 && support_size == 1) {
        return TwoVerticesCircle(support[0], inputs[0]);
    } else {
        // Recursively compute the minimum enclosing circle of the remaining vertices
        Circle mec = MinimumEnclosingCircle(inputs, num_inputs - 1, support, support_size);
        // Pick the last vertex of the input set, and if that vertex lies inside the
        // circle, it is indeed the minimum
        std::size_t idx = num_inputs - 1;
        if (mec.IsInCircle(*inputs[idx])) {
            return mec;
        }
        // Otherwise, update the set of support to additionally contain the new vertex
        support[support_size] = inputs[idx];
        return MinimumEnclosingCircle(inputs, num_inputs - 1, support, support_size + 1);
    }
}


Circle Circle::WelzlCircle(const std::vector<std::shared_ptr<Vertex>> &vertices) {
    // Copy and shuffle the list of vertex pointers first
    std::random_device rd;
    std::mt19937 rng(rd());
    std::vector<std::shared_ptr<Vertex>> vertices_copy(vertices);
    std::shuffle(vertices_copy.begin(), vertices_copy.end(), rng);
    // Recursively compute the minimum enclosing circle of the given vertices
    std::vector<std::shared_ptr<Vertex>> support(3);
    return MinimumEnclosingCircle(vertices_copy, vertices_copy.size(), support, 0);
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

RIGID2D_NAMESPACE_END
