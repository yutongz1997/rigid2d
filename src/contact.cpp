#include <memory>

#include <Eigen/Core>

#include "rigid2d/common.h"
#include "rigid2d/contact.h"


RIGID2D_NAMESPACE_BEGIN

Eigen::Vector2f ComputeInitialDirection(const Triangle& trig1,
                                        const Triangle& trig2) {
    return trig2.centroid - trig1.centroid;
}


Vertex FindFarthestPointInDirection(const Triangle& trig,
                                   const Eigen::Vector2f& direction) {
    float dot_prod1 = trig.v1->Dot(direction);
    float dot_prod2 = trig.v2->Dot(direction);
    float dot_prod3 = trig.v3->Dot(direction);
    if (dot_prod1 >= dot_prod2 && dot_prod2 >= dot_prod3) {
        return *trig.v1;
    } else if (dot_prod2 >= dot_prod1 && dot_prod2 >= dot_prod3) {
        return *trig.v2;
    } else {
        return *trig.v3;
    }
}


Eigen::Vector2f FindSupportPoint(const Triangle& trig1,
                                 const Triangle& trig2,
                                 const Eigen::Vector2f &direction) {
    Vertex p1 = FindFarthestPointInDirection(trig1, direction);
    Vertex p2 = FindFarthestPointInDirection(trig2, -direction);
    return { p1.x - p2.x, p1.y - p2.y };
}


bool GJKSimplex::ContainsOrigin(Eigen::Vector2f &direction) {
    // Reference: https://github.com/dyn4j/dyn4j/blob/master/src/main/java/org/dyn4j/collision/narrowphase/Gjk.java
    Eigen::Vector2f v1_o = -LastVertex();
    if (vertices_.size() == 3) {
        Eigen::Vector2f v1_v2 = vertices_[1] - LastVertex();
        Eigen::Vector2f v1_v3 = vertices_[0] - LastVertex();
        Eigen::Vector2f v1_v3_perp = VectorTripleProduct(v1_v2, v1_v3, v1_v3);
        if (v1_v3_perp.dot(v1_o) >= 0.0f) {
            vertices_.erase(vertices_.begin() + 1);
            direction = v1_v3_perp;
        } else {
            Eigen::Vector2f v1_v2_perp = VectorTripleProduct(v1_v3, v1_v2, v1_v2);
            if (v1_v2_perp.dot(v1_o) < 0.0f) {
                return true;
            } else {
                vertices_.erase(vertices_.begin());
                direction = v1_v2_perp;
            }
        }
    } else {
        Eigen::Vector2f v1_v2 = vertices_[0] - LastVertex();
        direction = VectorTripleProduct(v1_v2, v1_o, v1_v2);
        if (direction.squaredNorm() <= kEpsilon) {
            direction = { -direction.y(), direction.x() };
        }
    }
    return false;
}


Triangle TransformTriangle(const std::shared_ptr<Triangle>& trig,
                           const std::shared_ptr<RigidBody>& body) {
    Vertex v1 = body->PointToWorld(*trig->v1);
    Vertex v2 = body->PointToWorld(*trig->v2);
    Vertex v3 = body->PointToWorld(*trig->v3);
    return Triangle(std::make_shared<Vertex>(v1.x, v1.y),
                    std::make_shared<Vertex>(v2.x, v2.y),
                    std::make_shared<Vertex>(v3.x, v3.y));
}


bool GJKSolver::Intersect(const std::shared_ptr<RigidBody>& body1,
                          const std::shared_ptr<Triangle>& trig1,
                          const std::shared_ptr<RigidBody>& body2,
                          const std::shared_ptr<Triangle>& trig2,
                          Contact& contact) {
    // Reference: https://github.com/dyn4j/dyn4j/blob/master/src/main/java/org/dyn4j/collision/narrowphase/Gjk.java

    //
    Triangle trig1_transformed = TransformTriangle(trig1, body1);
    Triangle trig2_transformed = TransformTriangle(trig2, body2);
    Eigen::Vector2f d = ComputeInitialDirection(trig1_transformed, trig2_transformed);
    if (d.isZero()) {
        d = { 1.0f, 0.0f };
    }

    simplex_.Clear();
    simplex_.Add(FindSupportPoint(trig1_transformed, trig2_transformed, d));
    if (simplex_.LastVertex().dot(d) <= 0.0f) {
        return false;
    }

    d = -d;
    for (int i = 0; i < max_intersect_iterations_; ++i) {
        simplex_.Add(FindSupportPoint(trig1_transformed, trig2_transformed, d));
        if (simplex_.LastVertex().dot(d) <= 0.0f) {
            return false;
        } else {
            if (simplex_.ContainsOrigin(d)) {
                // TODO: Fill in contact data here
                return true;
            }
        }
    }

    return false;
}


Eigen::Vector2f FindClosestPointToOrigin(const Eigen::Vector2f& p1,
                                         const Eigen::Vector2f& p2) {
    Eigen::Vector2f p1_p2 = p2 - p1;
    float t = Clamp(-p1.dot(p1_p2) / p1_p2.squaredNorm(), 0.0f, 1.0f);
    return p1 + t * p1_p2;
}


bool ContainsOrigin(const Eigen::Vector2f& p1,
                    const Eigen::Vector2f& p2,
                    const Eigen::Vector2f& p3) {
    float sa = Cross2(p1, p2);
    float sb = Cross2(p2, p3);
    float sc = Cross2(p3, p1);
    return sa * sb > 0.0f && sa * sc > 0.0f;
}


bool GJKSolver::DistanceBetween(const std::shared_ptr<RigidBody>& body1,
                                const std::shared_ptr<Triangle>& trig1,
                                const std::shared_ptr<RigidBody>& body2,
                                const std::shared_ptr<Triangle>& trig2,
                                float& distance) {
    // Reference: https://github.com/dyn4j/dyn4j/blob/master/src/main/java/org/dyn4j/collision/narrowphase/Gjk.java

    // TODO: Check validity
    Triangle trig1_transformed = TransformTriangle(trig1, body1);
    Triangle trig2_transformed = TransformTriangle(trig2, body2);
    Eigen::Vector2f d = ComputeInitialDirection(trig1_transformed, trig2_transformed);

    if (d.isZero()) {
        return false;
    }

    simplex_.Clear();
    Eigen::Vector2f v1 = FindSupportPoint(trig1_transformed, trig2_transformed, d);
    Eigen::Vector2f v2 = FindSupportPoint(trig1_transformed, trig2_transformed, -d);
    d = FindClosestPointToOrigin(v1, v2);

    for (int i = 0; i < max_distance_iterations_; ++i) {
        d = -d;
        if (d.squaredNorm() <= kEpsilon) {
            return false;
        }

        Eigen::Vector2f v3 = FindSupportPoint(trig1_transformed, trig2_transformed, d);
        if (ContainsOrigin(v1, v2, v3)) {
            return false;
        }

        float dc = v3.dot(d);
        float da = v1.dot(d);
        if (dc - da < distance_epsilon_) {
            distance = d.norm();
            return true;
        }

        Eigen::Vector2f p1 = FindClosestPointToOrigin(v1, v3);
        Eigen::Vector2f p2 = FindClosestPointToOrigin(v3, v2);
        if (p1.squaredNorm() < p2.squaredNorm()) {
            v2 = v3;
            d = p1;
        } else {
            v1 = v3;
            d = p2;
        }
    }

    distance = d.norm();
    return true;
}

RIGID2D_NAMESPACE_END
