#ifndef RIGID2D_BVH_H
#define RIGID2D_BVH_H

#include <vector>
#include <memory>

#include "rigid2d/common.h"
#include "rigid2d/geometry.h"
#include "rigid2d/shader.h"


RIGID2D_NAMESPACE_BEGIN

class RigidBody;

class RigidBodySystem;


class BoundingDisc {
private:
    // Rigid body attached to the bounding disc
    RigidBody* body_;

    // Position of the center (in local coordinates) of the bounding disc
    Eigen::Vector2f center_local_;
    // Position of the center (in world coordinates) of the bounding disc
    Eigen::Vector2f center_world_;
    // Radius of the bounding disc
    float radius_;

    // Vertex array object (VAO) of the bounding disc
    GLuint vao_;
    // Vertex buffer of the bounding disc
    std::vector<GLfloat> vertex_buffer_;
    // Index buffer of the bounding disc
    std::vector<GLuint> index_buffer_;

private:
    void BuildDrawingBuffer(unsigned int num_segments = 120);

    void GenerateVAO();

public:
    explicit BoundingDisc(const std::vector<std::shared_ptr<Vertex>>& vertices,
                          RigidBody* body,
                          unsigned int num_segments = 120);

    [[nodiscard]] inline float Radius() const {
        return radius_;
    }

    void UpdateCenterWorld();

    [[nodiscard]] inline bool IsInside(const Vertex& p_local) const {
        return p_local.distanceSquared(center_local_) <= radius_ * radius_;
    }

    [[nodiscard]] inline bool IsInside(const Eigen::Vector2f& p_local) const {
        return (p_local - center_local_).squaredNorm() <= radius_ * radius_;
    }

    [[nodiscard]] inline bool Intersect(const BoundingDisc& other) const {
        return (center_world_ - other.center_world_).norm() < radius_ + other.radius_;
    }

    void Render(const std::shared_ptr<Shader>& disc_shader, const Eigen::Matrix4f& ortho) const;
};


class BVHNode {
private:
    // Bounding volume attached to the node
    std::unique_ptr<BoundingDisc> volume_;
    // Triangle attached to the node (only non-null if the node is a leaf)
    std::shared_ptr<Triangle> leaf_triangle_;
    // ID that keeps track of when the node was last visited
    int visit_id_;

    // Left child of the node
    std::shared_ptr<BVHNode> left_child_;
    // Right child of the node
    std::shared_ptr<BVHNode> right_child_;

    friend class RigidBody;
    friend class RigidBodySystem;

public:
    explicit BVHNode(const std::vector<std::shared_ptr<Triangle>>& triangles,
                     RigidBody* body);

    [[nodiscard]] inline bool IsLeaf() const {
        return !leaf_triangle_;
    }

    void Render(const std::shared_ptr<Shader>& disc_shader, const Eigen::Matrix4f& ortho) const;

    void RenderVisitBoundary(const std::shared_ptr<Shader>& disc_shader,
                             const Eigen::Matrix4f& ortho,
                             int visit) const;
};

RIGID2D_NAMESPACE_END

#endif // RIGID2D_BVH_H
