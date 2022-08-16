#ifndef RIGID2D_CONTACT_H
#define RIGID2D_CONTACT_H

#include <utility>
#include <vector>
#include <memory>
#include <queue>

#include <Eigen/Core>

#include "rigid2d/common.h"
#include "rigid2d/body.h"
#include "rigid2d/bvh.h"
#include "rigid2d/geometry.h"


RIGID2D_NAMESPACE_BEGIN

struct ContactPair {
    // First rigid body in contact
    std::shared_ptr<RigidBody> body1;
    // Corresponding triangle of the first body in contact
    std::shared_ptr<Triangle> triangle1;

    // Second rigid body in contact
    std::shared_ptr<RigidBody> body2;
    // Corresponding triangle of the second body in contact
    std::shared_ptr<Triangle> triangle2;

    ContactPair() = default;

    explicit ContactPair(std::shared_ptr<RigidBody> body1,
                         std::shared_ptr<Triangle> triangle1,
                         std::shared_ptr<RigidBody> body2,
                         std::shared_ptr<Triangle> triangle2) {
        this->body1 = std::move(body1);
        this->triangle1 = std::move(triangle1);
        this->body2 = std::move(body2);
        this->triangle2 = std::move(triangle2);
    }
};


struct Contact {
    //
    GLuint vao_;
    //
    std::vector<GLfloat> vertex_buffer_;

    //
    ContactPair pair;
    // Contact normal (in world coordinates)
    Eigen::Vector2f normal;
    // Penetration depth
    float penetration_depth;
    // Position of the contact point (in world coordinates)
    Eigen::Vector2f point;

    explicit Contact(const ContactPair& pair,
                     Eigen::Vector2f normal,
                     float penetration_depth,
                     Eigen::Vector2f point) {
        this->pair = pair;
        this->normal = std::move(normal);
        this->penetration_depth = penetration_depth;
        this->point = std::move(point);

        vao_ = 0;
        BuildDrawingBuffer();
    }

    void BuildDrawingBuffer();

    void Render(const std::shared_ptr<Shader>& contact_shader,
                const Eigen::Matrix4f& ortho) const;
};


struct Penetration {
    Eigen::Vector2f normal;
    float depth;
};


class GJKSimplex {
private:
    std::vector<Eigen::Vector2f> vertices_;

public:
    GJKSimplex() = default;

    [[nodiscard]] inline std::size_t Size() const {
        return vertices_.size();
    }

    inline Eigen::Vector2f operator[](std::size_t idx) const {
        return vertices_[idx];
    }

    [[nodiscard]] inline Eigen::Vector2f LastVertex() const {
        return vertices_[vertices_.size() - 1];
    }

    inline void Add(const Eigen::Vector2f& v) {
        vertices_.push_back(v);
    }

    inline void Clear() {
        vertices_.clear();
    }

    bool ContainsOrigin(Eigen::Vector2f& direction);
};


enum WindingDirection {
    Clockwise,
    CounterClockwise,
    Any
};


struct ExpandingSimplexEdge {
    Eigen::Vector2f v1;
    Eigen::Vector2f v2;
    Eigen::Vector2f normal;
    float distance;

    explicit ExpandingSimplexEdge(const Eigen::Vector2f& v1,
                                  const Eigen::Vector2f& v2,
                                  WindingDirection winding_direction)
        : v1(v1), v2(v2) {
        if (winding_direction == Clockwise) {
            normal = LeftHandedNormal(v2 - v1);
        } else {
            normal = RightHandedNormal(v2 - v1);
        }
        normal.normalize();
        //
        distance = std::abs(v1.dot(normal));
    }

    struct EdgeComparator {
        bool operator()(const ExpandingSimplexEdge& e1,
                        const ExpandingSimplexEdge& e2) {
            return e1.distance > e2.distance;
        }
    };
};


class ExpandingSimplex {
private:
    WindingDirection winding_direction_;

    std::priority_queue<ExpandingSimplexEdge,
                        std::vector<ExpandingSimplexEdge>,
                        ExpandingSimplexEdge::EdgeComparator> edges_;

public:
    explicit ExpandingSimplex(const GJKSimplex& simplex);

    [[nodiscard]] inline ExpandingSimplexEdge FindClosestEdgeToOrigin() const {
        return edges_.top();
    }

    void Expand(const Eigen::Vector2f& v);
};


class ExpandingPolygonSolver {
private:
    int max_iterations_;
    float distance_epsilon_;

public:
    explicit ExpandingPolygonSolver(int max_iterations = 100,
                                    float distance_epsilon = std::sqrt(kEpsilon))
         : max_iterations_(max_iterations), distance_epsilon_(distance_epsilon) { }

    void FindPenetration(const GJKSimplex& simplex,
                         const ContactPair& pair,
                         Penetration& penetration) const;
};


class GJKSolver {
private:
    int max_intersect_iterations_;
    int max_distance_iterations_;
    float distance_epsilon_;

    GJKSimplex simplex_;

    ExpandingPolygonSolver epa_solver_;

public:
    explicit GJKSolver(int max_intersect_iterations = 50,
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

    bool Intersect(const ContactPair& pair,
                   std::vector<Contact>& contacts);

    bool DistanceBetween(const ContactPair& pair,
                         float& distance);
};


struct EdgeFeature {
    Eigen::Vector2f v1;
    Eigen::Vector2f v2;
    Eigen::Vector2f max;

    explicit EdgeFeature(Eigen::Vector2f v1,
                         Eigen::Vector2f v2,
                         Eigen::Vector2f max) {
        this->v1 = std::move(v1);
        this->v2 = std::move(v2);
        this->max = std::move(max);
    }

    [[nodiscard]] inline Eigen::Vector2f AsVector() const {
        return v2 - v1;
    }
};


class SutherlandHodgmanSolver {
public:
    static bool FindContactPoints(const Penetration& penetration,
                                  const ContactPair& pair,
                                  std::vector<Contact>& contacts);
};

RIGID2D_NAMESPACE_END

#endif // RIGID2D_CONTACT_H
