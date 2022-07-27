#ifndef RIGID2D_BODYSYSTEM_H
#define RIGID2D_BODYSYSTEM_H

#include <vector>
#include <memory>

#include <Eigen/Core>

#include "rigid2d/common.h"
#include "rigid2d/body.h"
#include "rigid2d/contact.h"
#include "rigid2d/forces.h"
#include "rigid2d/geometry.h"
#include "rigid2d/shader.h"


RIGID2D_NAMESPACE_BEGIN

class RigidBodySystem {
private:
    std::vector<std::shared_ptr<RigidBody>> bodies_;
    std::vector<Contact> contacts_;

    int visit_id_;

    std::vector<Eigen::Vector3f> colors_;

    Gravity gravity_;
    MouseSpringForce mouse_spring_;

    float simulation_time_;

    friend class Scene;

private:
    void CheckCollision(const std::shared_ptr<RigidBody>& body1,
                        const std::shared_ptr<BVHNode>& node1,
                        const std::shared_ptr<RigidBody>& body2,
                        const std::shared_ptr<BVHNode>& node2);

    void ProcessCollision(const std::shared_ptr<RigidBody>& body1,
                          const std::shared_ptr<Triangle>& triangle1,
                          const std::shared_ptr<RigidBody>& body2,
                          const std::shared_ptr<Triangle>& triangle2);

    void BroadPhase();

public:
    RigidBodySystem();

    ~RigidBodySystem() = default;

    [[nodiscard]] inline float SimulationTime() const {
        return simulation_time_;
    }

    void AddBody(const std::shared_ptr<TriangleMesh>& geometry,
                 const std::shared_ptr<Shader>& shader,
                 const Eigen::Vector2f& position,
                 float orientation,
                 float mass);

    void ResetState();

    /**
     * Find the rigid body which has a triangle that intersects the provided point.
     * @param p_world
     * @return
     */
    [[nodiscard]] std::shared_ptr<RigidBody> PickBody(const Eigen::Vector2f& p_world) const;

    void Step(float dt);
};

RIGID2D_NAMESPACE_END

#endif // RIGID2D_BODYSYSTEM_H
