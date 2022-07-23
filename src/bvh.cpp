#include <vector>
#include <memory>
#include <unordered_set>

#include "rigid2d/common.h"
#include "rigid2d/bvh.h"


RIGID2D_NAMESPACE_BEGIN

BVHNode::BVHNode(const std::vector<std::shared_ptr<Triangle>>& triangles) {
    leaf_triangle_ = (triangles.size() == 1) ? triangles[0] : nullptr;

    // Determine the list of vertices the given triangles contain
    std::unordered_set<std::shared_ptr<Vertex>> set;
    for (const auto& trig : triangles) {
        set.insert(trig->v1);
        set.insert(trig->v2);
        set.insert(trig->v3);
    }
    std::vector<std::shared_ptr<Vertex>> vertices(set.begin(), set.end());

    // Compute the bounding disc using Welzl's algorithm
    bounding_disc_ = Circle::WelzlCircle(vertices);
}

RIGID2D_NAMESPACE_END
