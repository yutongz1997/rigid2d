#ifndef RIGID2D_FORCES_H
#define RIGID2D_FORCES_H

#include <memory>

#include "rigid2d/common.h"
#include "rigid2d/body.h"


RIGID2D_NAMESPACE_BEGIN

class Force {
public:
    virtual void Apply(const std::shared_ptr<RigidBody>& body) = 0;
};


class Gravity : public Force {
private:
    float k_gravity_;

public:
    explicit Gravity(float k_gravity = 9.8) : k_gravity_(k_gravity) { }

    inline void SetGravitationalConstant(float k_gravity) {
        k_gravity_ = k_gravity;
    }

    void Apply(const std::shared_ptr<RigidBody>& body) override;
};


class MouseSpringForce : public Force {
private:
    // Stiffness of the mouse spring
    float stiffness_;
    // Viscous damping coefficient for the mouse spring
    float damping_;

    //
    Eigen::Vector2f grab_point_body_;

public:
    // TODO
    explicit MouseSpringForce(float stiffness = 10.0f, float damping = 0.0f)
        : stiffness_(stiffness), damping_(damping) { }

    inline void SetStiffness(float stiffness) {
        stiffness_ = stiffness;
    }

    inline void SetDamping(float damping) {
        damping_ = damping;
    }

    // TODO
    void Apply(const std::shared_ptr<RigidBody>& body) override;
};

RIGID2D_NAMESPACE_END

#endif // RIGID2D_FORCES_H
