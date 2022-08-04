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

    Contact() = default;

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


class GJKSimplex {
private:
    std::vector<Eigen::Vector2f> vertices_;

public:
    GJKSimplex() = default;

    inline Eigen::Vector2f LastVertex() const {
        return vertices_[vertices_.size() - 1];
    }

    inline void Add(const Eigen::Vector2f& vert) {
        vertices_.push_back(vert);
    }

    inline void Clear() {
        vertices_.clear();
    }

    bool ContainsOrigin(Eigen::Vector2f& direction);
};


class GJKSolver {
private:
    int max_intersect_iterations_;
    int max_distance_iterations_;
    float distance_epsilon_;

    GJKSimplex simplex_;

public:
    explicit GJKSolver(int max_intersect_iterations = 30,
                       int max_distance_iterations = 30,
                       float distance_epsilon = std::sqrt(kEpsilon)) {
        max_intersect_iterations_ = max_intersect_iterations;
        max_distance_iterations_ = max_distance_iterations;
        distance_epsilon_ = distance_epsilon;
    }

    inline void SetMaxIntersectIterations(int max_intersect_iterations) {
        max_intersect_iterations_ = max_intersect_iterations;
    }

    inline void SetMaxDistanceIterations(int max_distance_iterations) {
        max_distance_iterations_ = max_distance_iterations;
    }

    bool Intersect(const std::shared_ptr<RigidBody>& body1,
                   const std::shared_ptr<Triangle>& trig1,
                   const std::shared_ptr<RigidBody>& body2,
                   const std::shared_ptr<Triangle>& trig2,
                   Contact& contact);

    bool DistanceBetween(const std::shared_ptr<RigidBody>& body1,
                         const std::shared_ptr<Triangle>& trig1,
                         const std::shared_ptr<RigidBody>& body2,
                         const std::shared_ptr<Triangle>& trig2,
                         float& distance);
};

RIGID2D_NAMESPACE_END

#endif // RIGID2D_CONTACT_H
