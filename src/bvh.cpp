#include <vector>
#include <memory>
#include <unordered_set>
#include <queue>

#include "rigid2d/common.h"
#include "rigid2d/bvh.h"
#include "rigid2d/body.h"
#include "rigid2d/geometry.h"


RIGID2D_NAMESPACE_BEGIN

BVHNode::BVHNode(const std::vector<std::shared_ptr<Triangle>>& triangles) {
    // Determine the list of (unique) vertices the given triangles contain
    std::unordered_set<std::shared_ptr<Vertex>> set_vertices;
    for (const auto& trig : triangles) {
        set_vertices.insert(trig->v1);
        set_vertices.insert(trig->v2);
        set_vertices.insert(trig->v3);
    }
    std::vector<std::shared_ptr<Vertex>> vertices(set_vertices.begin(),
                                                  set_vertices.end());
    // Compute the bounding disc using Welzl's algorithm
    bounding_disc_ = Circle::WelzlCircle(vertices);

    leaf_triangle_ = (triangles.size() == 1) ? triangles[0] : nullptr;

    if (!leaf_triangle_) {
        // Compute the maximum spread of all vertices in each axis-aligned
        // direction
        Eigen::Vector2f min_corner { kInf, kInf };
        Eigen::Vector2f max_corner { -kInf, -kInf };
        for (const auto& vert : vertices) {
            min_corner.x() = std::min(vert->x, min_corner.x());
            min_corner.y() = std::min(vert->y, min_corner.y());
            max_corner.x() = std::max(vert->x, max_corner.x());
            max_corner.y() = std::max(vert->y, max_corner.y());
        }
        Eigen::Vector2f spread = max_corner - min_corner;

        // Compute the average centroid of all given triangles
        Eigen::Vector2f avg_centroid;
        avg_centroid.setZero();
        for (const auto& trig : triangles) {
            avg_centroid += trig->centroid;
        }
        avg_centroid /= static_cast<float>(triangles.size());

        // Split the triangles in the axis-aligned direction in which they
        // are most spread out by comparing each triangle's centroid with
        // the average centroid
        std::vector<std::shared_ptr<Triangle>> left_triangles, right_triangles;
        for (const auto& trig : triangles) {
            if (spread.x() >= spread.y()) {
                if (trig->centroid.x() < avg_centroid.x()) {
                    left_triangles.push_back(trig);
                } else {
                    right_triangles.push_back(trig);
                }
            } else {
                if (trig->centroid.y() < avg_centroid.y()) {
                    left_triangles.push_back(trig);
                } else {
                    right_triangles.push_back(trig);
                }
            }
        }

        // Make left and right child nodes using the above triangle lists
        left_child_ = std::make_shared<BVHNode>(left_triangles);
        right_child_ = std::make_shared<BVHNode>(right_triangles);
    }
}

RIGID2D_NAMESPACE_END
