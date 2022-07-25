#ifndef RIGID2D_BODY_H
#define RIGID2D_BODY_H

#include <memory>

#include <Eigen/Core>

#include "rigid2d/common.h"
#include "rigid2d/geometry.h"
#include "rigid2d/shader.h"
#include "rigid2d/bvh.h"


RIGID2D_NAMESPACE_BEGIN

class RigidBody {
private:
    // Underlying geometry (triangle mesh) of the body
    std::shared_ptr<TriangleMesh> geometry_;
    // Vertex and fragment shaders associated with the mesh
    std::shared_ptr<Shader> shader_;

    //
    std::shared_ptr<BVHNode> bvh_root_;
    //
    std::vector<GLfloat> bvh_vertex_buffer_;
    std::vector<GLuint> bvh_index_buffer_;
    //
    GLuint bvh_vao_;

    // Mass of the body
    float mass_;
    // Inverse of the body's mass
    float inverse_mass_;
    // Rotational inertia of the body
    float rotational_inertia_;
    // Inverse of the body's rotational inertia
    float inverse_rotational_inertia_;

    // Current position of the center of mass in world coordinates
    Eigen::Vector2f position_;
    // Initial position of the center of mass in world coordinates
    Eigen::Vector2f initial_position_;
    // Current orientation angle of the body (in radians)
    float orientation_;
    // Initial orientation angle of the body (in radians)
    float initial_orientation_;

    // Linear velocity of the body
    Eigen::Vector2f linear_velocity_;
    // Angular velocity of the body (in radians per second)
    float angular_velocity_;
    // Total forces applied to the body
    Eigen::Vector2f force_;
    // Total torque applied to the body
    float torque_;

    // Rigid transformation to bring vertices/vectors in body coordinates to
    // world coordinates
    RigidTransformation body_to_world;
    // Rigid transformation to bring vertices/vectors in world coordinates to
    // body coordinates (inverse of the above transformation)
    RigidTransformation world_to_body;

    friend class Scene;

private:
    void PopulateBVHBuffer(int n = 360);

    void ComputeRotationalInertia();

    void UpdateTransformation();

public:
    explicit RigidBody(const std::shared_ptr<TriangleMesh>& geometry,
                       const std::shared_ptr<Shader>& shader,
                       const Eigen::Vector2f& position,
                       float orientation,
                       float mass);

    ~RigidBody() = default;

    void Reset();

    inline void AddForce(const Eigen::Vector2f& force) {
        force_ += force;
    }

    void Step(float duration);
};

RIGID2D_NAMESPACE_END

#endif // RIGID2D_BODY_H
