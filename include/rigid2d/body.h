#ifndef RIGID2D_BODY_H
#define RIGID2D_BODY_H

#include <memory>

#include <Eigen/Core>

#include "rigid2d/common.h"
#include "rigid2d/geometry.h"
#include "rigid2d/shader.h"
#include "rigid2d/bvh.h"


RIGID2D_NAMESPACE_BEGIN

class RigidBody : std::enable_shared_from_this<RigidBody> {
private:
    // Underlying geometry (triangle mesh) of the body
    std::shared_ptr<TriangleMesh> geometry_;
    // Vertex and fragment shaders associated with the mesh
    std::shared_ptr<Shader> shader_;

    //
    std::shared_ptr<BVHNode> bvh_root_;

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

    // Rigid transformation to bring vertices/vectors in local coordinates to
    // world coordinates
    RigidTransformation local_to_world_;
    // Rigid transformation to bring vertices/vectors in world coordinates to
    // local coordinates (inverse of the above transformation)
    RigidTransformation world_to_local_;

    //
    int index_;

    friend class BoundingDisc;
    friend class MouseSpringForce;
    friend class RigidBodySystem;
    friend class Scene;

private:
    void ComputeRotationalInertia();

    void UpdateTransformation();

public:
    explicit RigidBody(const std::shared_ptr<TriangleMesh>& geometry,
                       const std::shared_ptr<Shader>& shader,
                       const Eigen::Vector2f& position,
                       float orientation,
                       float mass);

    ~RigidBody() = default;

    [[nodiscard]] inline float Mass() const {
        return mass_;
    }

    [[nodiscard]] inline Eigen::Vector2f PointToWorld(const Eigen::Vector2f& p_local) const {
        return local_to_world_.TransformPoint(p_local);
    }

    [[nodiscard]] inline Eigen::Vector2f VectorToWorld(const Eigen::Vector2f& v_local) const {
        return local_to_world_.TransformVector(v_local);
    }

    [[nodiscard]] inline Eigen::Vector2f PointToLocal(const Eigen::Vector2f& p_world) const {
        return world_to_local_.TransformPoint(p_world);
    }

    [[nodiscard]] inline Eigen::Vector2f VectorToLocal(const Eigen::Vector2f& v_world) const {
        return world_to_local_.TransformVector(v_world);
    }

    void AddForce(const Eigen::Vector2f& force);

    void AddContactForce(const Eigen::Vector2f& contact_point,
                         const Eigen::Vector2f& contact_force);

    [[nodiscard]] Eigen::Vector2f SpatialVelocity(const Eigen::Vector2f& contact_point) const;

    [[nodiscard]] bool IsInside(const Eigen::Vector2f& p_world) const;

    void ResetState();

    void Step(float dt);
};

RIGID2D_NAMESPACE_END

#endif // RIGID2D_BODY_H
