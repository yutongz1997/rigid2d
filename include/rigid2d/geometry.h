#ifndef RIGID2D_GEOMETRY_H
#define RIGID2D_GEOMETRY_H

#include <iostream>
#include <string>
#include <vector>
#include <memory>

#include <fmt/core.h>

#include <Eigen/Core>

#include <glad/glad.h>

#include "rigid2d/common.h"
#include "rigid2d/shader.h"


RIGID2D_NAMESPACE_BEGIN

struct Vertex {
    // X-coordinate of the vertex
    float x;
    // Y-coordinate of the vertex
    float y;
    // Index of the vertex in an array of vertices (optional)
    int index;

    explicit Vertex(float x, float y) : x(x), y(y) {
        index = -1;
    }

    inline explicit operator Eigen::Vector2f() const {
        return { x, y };
    }
};


struct Triangle {
    std::shared_ptr<Vertex> v1;
    std::shared_ptr<Vertex> v2;
    std::shared_ptr<Vertex> v3;
    Eigen::Vector2f centroid;
    float area;

    explicit Triangle(const std::shared_ptr<Vertex>& v1,
                      const std::shared_ptr<Vertex>& v2,
                      const std::shared_ptr<Vertex>& v3)
                      : v1(v1), v2(v2), v3(v3) {
        // Compute the centroid of this triangle
        centroid.x() = (v1->x + v2->x + v3->x) / 3.0f;
        centroid.y() = (v1->y + v2->y + v3->y) / 3.0f;
        // Compute the area of this triangle
        Eigen::Vector2f e1 { v2->x - v1->x, v2->y - v1->y };
        Eigen::Vector2f e2 { v3->x - v1->x, v3->y - v1->y };
        area = 0.5f * std::fabs(Cross2(e1, e2));
    }
};


class TriangleMesh {
private:
    std::string name_;
    bool loaded_;

    std::vector<std::shared_ptr<Vertex>> vertices_;
    std::vector<std::shared_ptr<Triangle>> triangles_;
    std::vector<Eigen::Vector2f> centroids_;
    std::vector<float> areas_;
    float area_;

    std::vector<GLfloat> buffer_vertices_;
    std::vector<GLuint> buffer_indices_;

private:
    void MakeVertices();
    void MakeTriangles();
    void CleanUp();

public:
    TriangleMesh(const std::string& name, const std::string& path);

    ~TriangleMesh() = default;

    [[nodiscard]] inline std::string Name() const {
        return name_;
    }

    [[nodiscard]] inline std::size_t NumVertices() const {
        return vertices_.size();
    }

    [[nodiscard]] inline std::size_t NumTriangles() const {
        return triangles_.size();
    }

    [[nodiscard]] inline float Area() const {
        return area_;
    }

    [[nodiscard]] inline std::vector<GLfloat> VertexBufferData() const {
        return buffer_vertices_;
    }

    [[nodiscard]] inline std::vector<GLuint> IndexBufferData() const {
        return buffer_indices_;
    }

    friend std::ostream& operator<<(std::ostream& os,
                                    const TriangleMesh& mesh) {
        os << fmt::format("Mesh ({}) - Vertices: {}, Triangles: {}.",
                          mesh.name_, mesh.NumVertices(), mesh.NumTriangles());
        return os;
    }
};

RIGID2D_NAMESPACE_END

#endif // RIGID2D_GEOMETRY_H
