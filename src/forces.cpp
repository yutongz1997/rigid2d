#include <memory>

#include <Eigen/Core>

#include "rigid2d/common.h"
#include "rigid2d/forces.h"
#include "rigid2d/body.h"


RIGID2D_NAMESPACE_BEGIN

void Gravity::Apply(const std::shared_ptr<RigidBody> &body) {
    if (body->Mass() < kInf) {
        Eigen::Vector2f gravity { 0.0f, -k_gravity_ };
        body->AddForce(body->Mass() * gravity);
    }
}


void MouseSpringForce::Apply(const std::shared_ptr<RigidBody> &body) {
    if (!body) {
        return;
    }

    Eigen::Vector2f grab_point_world = body->PointToWorld(grab_point_body_);
    Eigen::Vector2f direction = -grab_point_world;
    if (direction.norm() < 1.0e-3f) {
        return;
    }
    direction.normalize();
    body->AddContactForce(grab_point_world,
                          stiffness_ * grab_point_world.norm() * direction);

    Eigen::Vector2f grab_point_vel = body->SpatialVelocity(grab_point_world);
    body->AddContactForce(grab_point_world,
                          -damping_ * grab_point_vel.dot(direction) * direction);
}

RIGID2D_NAMESPACE_END
