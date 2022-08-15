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


Triangle TransformTriangle(const std::shared_ptr<Triangle>& trig,
                           const std::shared_ptr<RigidBody>& body) {
    Vertex v1 = body->PointToWorld(*trig->v1);
    Vertex v2 = body->PointToWorld(*trig->v2);
    Vertex v3 = body->PointToWorld(*trig->v3);
    return Triangle(std::make_shared<Vertex>(v1.x, v1.y),
                    std::make_shared<Vertex>(v2.x, v2.y),
                    std::make_shared<Vertex>(v3.x, v3.y));
}


bool ContainsOrigin(const Eigen::Vector2f& p1,
                    const Eigen::Vector2f& p2,
                    const Eigen::Vector2f& p3) {
    float sa = Cross2(p1, p2);
    float sb = Cross2(p2, p3);
    float sc = Cross2(p3, p1);
    return sa * sb > 0.0f && sa * sc > 0.0f;
}


Eigen::Vector2f FindClosestPointToOrigin(const Eigen::Vector2f& p1,
                                         const Eigen::Vector2f& p2) {
    Eigen::Vector2f p1_p2 = p2 - p1;
    float t = Clamp(-p1.dot(p1_p2) / p1_p2.squaredNorm(), 0.0f, 1.0f);
    return p1 + t * p1_p2;
}


WindingDirection FindWindingDirection(const GJKSimplex& simplex) {
    for (int i = 0; i < simplex.Size(); ++i) {
        Eigen::Vector2f o_v1 = simplex[i];
        Eigen::Vector2f o_v2 = simplex[(i + 1) % simplex.Size()];
        float cross_prod = Cross2(o_v1, o_v2);
        if (cross_prod > 0.0f) {
            return CounterClockwise;
        } else if (cross_prod < 0.0f) {
            return Clockwise;
        }
    }
    return Any;
}


ExpandingSimplex::ExpandingSimplex(const GJKSimplex& simplex) {
    winding_direction_ = FindWindingDirection(simplex);
    for (int i = 0; i < simplex.Size(); ++i) {
        Eigen::Vector2f v1 = simplex[i];
        Eigen::Vector2f v2 = simplex[(i + 1) % simplex.Size()];
        edges_.emplace(v1, v2, winding_direction_);
    }
}


void ExpandingSimplex::Expand(const Eigen::Vector2f& p) {
    ExpandingSimplexEdge e = edges_.top();
    edges_.pop();
    edges_.emplace(e.v1, p, winding_direction_);
    edges_.emplace(p, e.v2, winding_direction_);
}


void ExpandingPolygonSolver::FindPenetration(const GJKSimplex& simplex,
                                             ContactManifold& contact) const {
    Triangle trig1_transformed = TransformTriangle(contact.triangle1, contact.body1);
    Triangle trig2_transformed = TransformTriangle(contact.triangle2, contact.body2);
    ExpandingSimplex exp_simplex(simplex);

    for (int i = 0; i < max_iterations_; ++i) {
        ExpandingSimplexEdge e = exp_simplex.FindClosestEdgeToOrigin();
        Eigen::Vector2f p = FindSupportPoint(trig1_transformed, trig2_transformed, e.normal);
        float dist = p.dot(e.normal);

        //
        contact.normal = e.normal;
        contact.penetration_depth = dist;
        if (dist - e.distance < distance_epsilon_) {
            break;
        }

        exp_simplex.Expand(p);
    }
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


bool GJKSolver::Intersect(const std::shared_ptr<RigidBody>& body1,
                          const std::shared_ptr<Triangle>& trig1,
                          const std::shared_ptr<RigidBody>& body2,
                          const std::shared_ptr<Triangle>& trig2,
                          ContactManifold& contact) {
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
                contact.body1 = body1;
                contact.triangle1 = trig1;
                contact.body2 = body2;
                contact.triangle2 = trig2;
                epa_solver_.FindPenetration(simplex_, contact);
                SutherlandHodgmanSolver::FindContactPoints(contact);
                return true;
            }
        }
    }

    return false;
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


EdgeFeature FindFarthestEdgeFeature(const std::shared_ptr<RigidBody>& body,
                                    const std::shared_ptr<Triangle>& trig,
                                    const Eigen::Vector2f& normal) {
    //
    Eigen::Vector2f normal_local = body->VectorToLocal(normal);
    float v1_proj = trig->v1->Dot(normal_local);
    float v2_proj = trig->v2->Dot(normal_local);
    float v3_proj = trig->v3->Dot(normal_local);

    std::shared_ptr<Vertex> curr, prev, next;
    if (v1_proj >= v2_proj && v2_proj >= v3_proj) {
        curr = trig->v1;
        prev = trig->v3;
        next = trig->v2;
    } else if (v2_proj >= v1_proj && v2_proj >= v3_proj) {
        curr = trig->v2;
        prev = trig->v1;
        next = trig->v3;
    } else {
        curr = trig->v3;
        prev = trig->v2;
        next = trig->v1;
    }

    //
    Eigen::Vector2f left = (*curr - *next).normalized();
    Eigen::Vector2f right = (*curr - *prev).normalized();
    Eigen::Vector2f curr_world = body->PointToWorld({ curr->x, curr->y });
    if (left.dot(normal_local) < right.dot(normal_local)) {
        Eigen::Vector2f next_world = body->PointToWorld({ next->x, next->y });
        return EdgeFeature(curr_world, next_world, curr_world);
    } else {
        Eigen::Vector2f prev_world = body->PointToWorld({ prev->x, prev->y });
        return EdgeFeature(prev_world, curr_world, curr_world);
    }
}


std::vector<Eigen::Vector2f> Clip(const Eigen::Vector2f& v1,
                                  const Eigen::Vector2f& v2,
                                  const Eigen::Vector2f& normal,
                                  float offset) {
    std::vector<Eigen::Vector2f> points;

    //
    float d1 = v1.dot(normal) - offset;
    float d2 = v2.dot(normal) - offset;
    //
    if (d1 >= 0.0f) {
        points.push_back(v1);
    }
    if (d2 >= 0.0f) {
        points.push_back(v2);
    }
    //
    if (d1 * d2 < 0.0f) {
        float alpha = d1 / (d1 - d2);
        points.emplace_back(v1 + alpha * (v2 - v1));
    }

    return points;
}


bool SutherlandHodgmanSolver::FindContactPoints(ContactManifold& contact) {
    EdgeFeature e1 = FindFarthestEdgeFeature(contact.body1, contact.triangle1, contact.normal);
    EdgeFeature e2 = FindFarthestEdgeFeature(contact.body2, contact.triangle2, -contact.normal);

    EdgeFeature reference = e1, incident = e2;
    bool flipped = false;
    if (std::abs(e1.AsVector().dot(contact.normal)) > std::abs(e2.AsVector().dot(contact.normal))) {
        reference = e2;
        incident = e1;
        flipped = true;
    }

    Eigen::Vector2f ref_vect = reference.AsVector().normalized();
    float offset1 = ref_vect.dot(reference.v1);
    std::vector<Eigen::Vector2f> clip1 = Clip(incident.v1, incident.v2, ref_vect, offset1);
    if (clip1.size() < 2) {
        return false;
    }

    float offset2 = ref_vect.dot(reference.v2);
    std::vector<Eigen::Vector2f> clip2 = Clip(clip1[0], clip1[1], -ref_vect, -offset2);
    if (clip2.size() < 2) {
        return false;
    }

    Eigen::Vector2f front_normal = RightHandedNormal(ref_vect);
    float front_offset = reference.max.dot(front_normal);
    contact.normal = flipped ? -front_normal : front_normal;

    for (Eigen::Vector2f& point : clip2) {
        float depth = point.dot(front_normal) - front_offset;
        if (depth >= 0.0f) {
            contact.points.emplace_back(point, depth);
        }
    }

    if (contact.points.empty()) {
        return false;
    }
    return true;
}

RIGID2D_NAMESPACE_END
