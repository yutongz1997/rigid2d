#include <queue>
#include <numeric>

#include "rigid2d/common.h"
#include "rigid2d/body.h"


RIGID2D_NAMESPACE_BEGIN

/**
 * See https://physics.stackexchange.com/questions/708936/how-to-calculate-the-moment-of-inertia-of-convex-polygon-two-dimensions
 */
void RigidBody::ComputeRotationalInertia() {
    // Compute the mass moment of inertia (MMOI) per unit mass for each triangle
    // and accumulate the contribution by the area of each triangle
    std::size_t num_triangles = geometry_->NumTriangles();
    for (int i = 0; i < num_triangles; ++i) {
        auto& v1 = geometry_->triangles_[i]->v1;
        auto& v2 = geometry_->triangles_[i]->v2;
        auto& v3 = geometry_->triangles_[i]->v3;
        float unit_mmoi = (v1->dot(*v1) + v2->dot(*v2) + v3->dot(*v3) +
                v1->dot(*v2) + v2->dot(*v3) + v1->dot(*v3)) / 6.0f;
        rotational_inertia_ += geometry_->areas_[i] * unit_mmoi;
    }
    rotational_inertia_ *= (mass_ / geometry_->area_);
}


void RigidBody::UpdateTransformation() {
    local_to_world_.Set(position_, orientation_);
    world_to_local_.SetInverse(position_, orientation_);
}


RigidBody::RigidBody(const std::shared_ptr<TriangleMesh>& geometry,
                     const std::shared_ptr<Shader>& shader,
                     const Eigen::Vector2f& position,
                     float orientation,
                     float mass) {
    geometry_ = geometry;
    shader_ = shader;

    bvh_root_ = std::make_shared<BVHNode>(geometry_->triangles_, this);

    mass_ = mass;
    inverse_mass_ = (mass == kInf || mass == -kInf) ? 0.0f : 1.0f / mass;
    rotational_inertia_ = 0.0f;
    ComputeRotationalInertia();
    inverse_rotational_inertia_ = 1.0f / rotational_inertia_;

    initial_position_ = position;
    position_ = position;
    initial_orientation_ = orientation;
    orientation_ = orientation;
    UpdateTransformation();

    linear_velocity_.setZero();
    angular_velocity_ = 0.0f;
    force_.setZero();
    torque_ = 0.0f;

    index_ = -1;
}


void RigidBody::AddForce(const Eigen::Vector2f& force) {
    force_ += force;
}


void RigidBody::AddContactForce(const Eigen::Vector2f& contact_point,
                                const Eigen::Vector2f& contact_force) {
    force_ += contact_force;
    torque_ += Cross2(contact_point - position_, contact_force);
}


Eigen::Vector2f RigidBody::SpatialVelocity(const Eigen::Vector2f& contact_point) const {
    Eigen::Vector2f tmp = angular_velocity_ * (contact_point - position_);
    Eigen::Vector2f vel { -tmp.y(), tmp.x() };
    return vel + linear_velocity_;
}


bool RigidBody::IsInside(const Eigen::Vector2f& p_world) const {
    Eigen::Vector2f p_local = world_to_local_.TransformPoint(p_world);
    if (bvh_root_->volume_->IsInside(p_local)) {
        for (const auto& trig : geometry_->triangles_) {
            if (trig->IsInside(p_local)) {
                return true;
            }
        }
    }
    return false;
}


void RigidBody::ResetState() {
    position_ = initial_position_;
    orientation_ = initial_orientation_;
    linear_velocity_.setZero();
    angular_velocity_ = 0.0f;
    force_.setZero();
    torque_ = 0.0f;
    UpdateTransformation();
}


void RigidBody::Step(float dt) {
    if (inverse_mass_ > 0.0f) {
        // Advance the state of angular velocity and orientation angle
        angular_velocity_ += inverse_rotational_inertia_ * torque_ * dt;
        orientation_ += angular_velocity_ * dt;
        // Advance the state of linear velocity and position
        linear_velocity_ += inverse_mass_ * force_ * dt;
        position_ += linear_velocity_ * dt;
        // Update rigid transformation matrices after stepping
        UpdateTransformation();
    }
    // Reset the force and torque accumulators
    force_.setZero();
    torque_ = 0.0f;
}

RIGID2D_NAMESPACE_END
