#ifndef RIGID2D_BODYSYSTEM_H
#define RIGID2D_BODYSYSTEM_H

#include <vector>
#include <memory>

#include "rigid2d/common.h"
#include "rigid2d/body.h"
#include "rigid2d/geometry.h"
#include "rigid2d/shader.h"


RIGID2D_NAMESPACE_BEGIN

class RigidBodySystem {
private:
    std::vector<std::shared_ptr<RigidBody>> bodies_;

    friend class Scene;

public:
    RigidBodySystem() = default;

    ~RigidBodySystem() = default;

    inline void AddBody(const std::shared_ptr<TriangleMesh>& geometry,
                        const std::shared_ptr<Shader>& shader,
                        const Eigen::Vector2f& position,
                        float orientation,
                        float mass) {
        bodies_.push_back(
            std::make_shared<RigidBody>(geometry, shader, position, orientation, mass)
        );
    }

    inline void Step(float duration) {
        for (auto& body : bodies_) {
            body->Step(duration);
        }
    }
};

RIGID2D_NAMESPACE_END

#endif // RIGID2D_BODYSYSTEM_H
