#include <random>

#include "rigid2d/common.h"
#include "rigid2d/bodysystem.h"


RIGID2D_NAMESPACE_BEGIN

void RigidBodySystem::CheckCollision(const std::shared_ptr<RigidBody> &body1,
                                     const std::shared_ptr<BVHNode>& node1,
                                     const std::shared_ptr<RigidBody>& body2,
                                     const std::shared_ptr<BVHNode>& node2) {
    node1->volume_->UpdateCenterWorld();
    node1->visit_id_ = visit_id_;
    node2->volume_->UpdateCenterWorld();
    node2->visit_id_ = visit_id_;

    if (node1->volume_->Intersect(*node2->volume_)) {
        if (node1->IsLeaf() && node2->IsLeaf()) {
            ProcessCollision(body1, node1->leaf_triangle_, body2, node2->leaf_triangle_);
        } else {
            if (node2->IsLeaf() || (!node1->IsLeaf() && node1->volume_->Radius() >= node2->volume_->Radius())) {
                CheckCollision(body1, node1->left_child_, body2, node2);
                CheckCollision(body1, node1->right_child_, body2, node2);
            } else {
                CheckCollision(body1, node1, body2, node2->left_child_);
                CheckCollision(body1, node1, body2, node2->right_child_);
            }
        }
    }
}


void RigidBodySystem::ProcessCollision(const std::shared_ptr<RigidBody>& body1,
                                       const std::shared_ptr<Triangle>& trig1,
                                       const std::shared_ptr<RigidBody>& body2,
                                       const std::shared_ptr<Triangle>& trig2) {
    ContactManifold ct;
    if (contact_solver_.Intersect(body1, trig1, body2, trig2, ct)) {
        // TODO: Collision response
        float k = 1000.0f;
        float c1 = 10.0f;
        float threshold = 0.001f;

        Eigen::Vector2f v1 = body1->SpatialVelocity(ct.points[0].point);
        Eigen::Vector2f v2 = body2->SpatialVelocity(ct.points[0].point);
        Eigen::Vector2f rel_vel = v1 - v2;

        Eigen::Vector2f force = k * ct.points[0].penetration_depth * ct.normal;
        body1->AddContactForce(ct.points[0].point, force);
        body2->AddContactForce(ct.points[0].point, -force);
        force = c1 * rel_vel.dot(ct.normal) * ct.normal;
        body2->AddContactForce(ct.points[0].point, force);
        body1->AddContactForce(ct.points[0].point, -force);
    }
}


void RigidBodySystem::BroadPhase() {
    ++visit_id_;
    for (auto& body1 : bodies_) {
        for (auto& body2 : bodies_) {
            if (body1->index_ >= body2->index_) {
                continue;
            }
            if (body1->mass_ == kInf && body2->mass_ == kInf) {
                continue;
            }
            CheckCollision(body1, body1->bvh_root_, body2, body2->bvh_root_);
        }
    }
}


RigidBodySystem::RigidBodySystem() {
    simulation_time_ = 0.0f;
    visit_id_ = 0;
}


void RigidBodySystem::AddBody(const std::shared_ptr<TriangleMesh>& geometry,
                              const std::shared_ptr<Shader>& shader,
                              const Eigen::Vector2f& position,
                              float orientation,
                              float mass) {
    auto body = std::make_shared<RigidBody>(geometry, shader, position, orientation, mass);
    body->index_ = static_cast<int>(bodies_.size());
    bodies_.push_back(body);

    std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<float> dist(0.0f, 1.0f);
    float r = dist(rng);
    float g = dist(rng);
    float b = dist(rng);
    colors_.emplace_back(r, g, b);
}


void RigidBodySystem::ResetState() {
    for (const auto& body : bodies_) {
        body->ResetState();
    }
    visit_id_ = 0;
    simulation_time_ = 0.0f;
}


std::shared_ptr<RigidBody> RigidBodySystem::PickBody(const Eigen::Vector2f& p_world) const {
    for (auto& body : bodies_) {
        if (body->IsInside(p_world)) {
            return body;
        }
    }
    return nullptr;
}


void RigidBodySystem::Step(float dt) {
    for (const auto& body : bodies_) {
        gravity_.Apply(body);
    }

    contacts_.clear();
    BroadPhase();

    for (const auto& body : bodies_) {
        body->Step(dt);
    }
    simulation_time_ += dt;
}

RIGID2D_NAMESPACE_END
