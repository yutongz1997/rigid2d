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

// Forward declarations
class RigidBody;


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

    [[nodiscard]] inline float Dot(const Vertex& other) const {
        return x * other.x + y * other.y;
    }

    [[nodiscard]] inline float Dot(const Eigen::Vector2f& other) const {
        return x * other.x() + y * other.y();
    }

    [[nodiscard]] inline float DistanceSquared(const Vertex& other) const {
        return Dot(other);
    }

    [[nodiscard]] inline float DistanceSquared(const Eigen::Vector2f& other) const {
        float dx = other.x() - x;
        float dy = other.y() - y;
        return dx * dx + dy * dy;
    }

    [[nodiscard]] inline float Distance(const Vertex& other) const {
        return std::sqrt(DistanceSquared(other));
    }

    [[nodiscard]] inline float Distance(const Eigen::Vector2f& other) const {
        return std::sqrt(DistanceSquared(other));
    }

    friend Eigen::Vector2f operator-(const Vertex& v1, const Vertex& v2) {
        return { v1.x - v2.x, v1.y - v2.y };
    }
};


class RigidTransformation {
private:
    // Translational component of the rigid transformation
    Eigen::Vector2f translation_;
    // Rotational component (in radians) of the rigid transformation
    float rotation_;

    // Internally used transformation matrix
    Eigen::Matrix4f transformation_;

public:
    RigidTransformation() {
        translation_.setZero();
        rotation_ = 0.0f;
        transformation_.setIdentity();
    }

    inline void Set(const Eigen::Vector2f& translation,
                    float rotation) {
        translation_ = translation;
        rotation_ = rotation;

        float c = std::cos(rotation_);
        float s = std::sin(rotation_);
        transformation_ <<    c,   -s, 0.0f, translation_.x(),
                              s,    c, 0.0f, translation_.y(),
                           0.0f, 0.0f, 1.0f,             0.0f,
                           0.0f, 0.0f, 0.0f,             1.0f;
    }

    inline void SetInverse(const Eigen::Vector2f& translation,
                           float rotation) {
        Set(-translation, -rotation);
    }

    [[nodiscard]] inline Eigen::Vector2f TransformPoint(const Eigen::Vector2f& p) const {
        Eigen::Vector4f p_homogeneous { p.x(), p.y(), 0.0f, 1.0f };
        Eigen::Vector4f q = transformation_ * p_homogeneous;
        return { q.x(), q.y() };
    }

    [[nodiscard]] inline Vertex TransformPoint(const Vertex& p) const {
        Eigen::Vector4f p_homogenous { p.x, p.y, 0.0f, 1.0f };
        Eigen::Vector4f q = transformation_ * p_homogenous;
        return Vertex(q.x(), q.y());
    }

    [[nodiscard]] inline Eigen::Vector2f TransformVector(const Eigen::Vector2f& v) const {
        Eigen::Vector4f v_homogeneous { v.x(), v.y(), 0.0f, 0.0f };
        Eigen::Vector4f w = transformation_ * v_homogeneous;
        return { w.x(), w.y() };
    }

    [[nodiscard]] inline Eigen::Matrix4f TransformationMatrix() const {
        return transformation_;
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
        area = 0.5f * std::abs(Cross2(e1, e2));
    }

    [[nodiscard]] bool IsInside(const Eigen::Vector2f& p) const;
};


struct Circle {
    // Position of the center of the circle
    Eigen::Vector2f center;
    // Radius of the circle
    float radius;

    Circle() : center({ 0.0f, 0.0f }), radius(1.0f) { }

    Circle(const Eigen::Vector2f& center, float radius)
        : center(center), radius(radius) { }

    [[nodiscard]] inline bool IsInside(const Vertex& v) const {
        return v.DistanceSquared(center) <= radius * radius;
    }

    static Circle WelzlCircle(const std::vector<std::shared_ptr<Vertex>>& vertices);
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

    std::vector<GLfloat> vertex_buffer_;
    std::vector<GLuint> index_buffer_;

    friend class RigidBody;

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
        return vertex_buffer_;
    }

    [[nodiscard]] inline std::vector<GLuint> IndexBufferData() const {
        return index_buffer_;
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
