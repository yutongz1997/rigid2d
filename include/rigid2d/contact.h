#ifndef RIGID2D_CONTACT_H
#define RIGID2D_CONTACT_H

#include <utility>
#include <vector>
#include <memory>

#include <Eigen/Core>

#include "rigid2d/common.h"
#include "rigid2d/body.h"
#include "rigid2d/bvh.h"
#include "rigid2d/geometry.h"


RIGID2D_NAMESPACE_BEGIN

struct Contact {
    // First rigid body in contact
    std::shared_ptr<RigidBody> body1;
    // Second rigid body in contact
    std::shared_ptr<RigidBody> body2;

    // Triangle of the first body in contact
    std::shared_ptr<Triangle> triangle1;
    // Triangle of the second body in contact
    std::shared_ptr<Triangle> triangle2;

    // Contact normal (in world coordinates)
    Eigen::Vector2f normal;
    // Position of the contact point (in world coordinates)
    Eigen::Vector2f point;

    explicit Contact(const std::shared_ptr<RigidBody>& body1,
                     const std::shared_ptr<RigidBody>& body2,
                     const std::shared_ptr<Triangle>& triangle1,
                     const std::shared_ptr<Triangle>& triangle2,
                     const Eigen::Vector2f& normal,
                     const Eigen::Vector2f& point) {
        this->body1 = body1;
        this->body2 = body2;
        this->triangle1 = triangle1;
        this->triangle2 = triangle2;
        this->normal = normal;
        this->point = point;
    }
};

RIGID2D_NAMESPACE_END

#endif // RIGID2D_CONTACT_H
