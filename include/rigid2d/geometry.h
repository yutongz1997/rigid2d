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

class RigidTransformation {
private:
    // Translational component of the rigid transformation
    Eigen::Vector2f translation_;
    // Rotational component (in radians) of the rigid transformation
    float rotation_;

    // Internally used transformation matrix
    Eigen::Matrix3f transformation_;

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
        transformation_ << c, -s, translation_.x(),
                           s,  c, translation_.y(),
                           0,  0,                1;
    }

    inline void SetInverse(const Eigen::Vector2f& translation,
                           float rotation) {
        Set(-translation, -rotation);
    }

    inline Eigen::Vector2f TransformPoint(const Eigen::Vector2f& p) {
        Eigen::Vector3f p_homogeneous { p.x(), p.y(), 1 };
        Eigen::Vector3f q = transformation_ * p_homogeneous;
        return { p.x(), p.y() };
    }

    inline Eigen::Vector2f TransformVector(const Eigen::Vector2f& v) {
        Eigen::Vector3f v_homogeneous { v.x(), v.y(), 0 };
        Eigen::Vector3f w = transformation_ * v_homogeneous;
        return { w.x(), w.y() };
    }

    [[nodiscard]] inline Eigen::Matrix3f TransformationMatrix() const {
        return transformation_;
    }
};


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

    [[nodiscard]] inline float dot(const Vertex& other) const {
        return x * other.x + y * other.y;
    }

    [[nodiscard]] inline float distanceSquared(const Vertex& other) const {
        return dot(other);
    }

    [[nodiscard]] inline float distanceSquared(const Eigen::Vector2f& other) const {
        float dx = other.x() - x;
        float dy = other.y() - y;
        return dx * dx + dy * dy;
    }

    [[nodiscard]] inline float distance(const Vertex& other) const {
        return std::sqrt(distanceSquared(other));
    }

    [[nodiscard]] inline float distance(const Eigen::Vector2f& other) const {
        return std::sqrt(distanceSquared(other));
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
};


struct Circle {
    // Center position (in world coordinates) of the circle
    Eigen::Vector2f center;
    // Radius of the circle
    float radius;

    Circle() : center({ 0.0f, 0.0f }), radius(1.0f) { }

    Circle(const Eigen::Vector2f& center, float radius)
        : center(center), radius(radius) { }

    [[nodiscard]] inline bool IsInCircle(const Vertex& p) const {
        return p.distanceSquared(center) <= radius * radius;
    }

    [[nodiscard]] inline bool IsInCircle(const Eigen::Vector2f& p) const {
        return (p - center).squaredNorm() <= radius * radius;
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
