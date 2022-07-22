#include <vector>
#include <memory>
#include <unordered_set>

#include "rigid2d/common.h"
#include "rigid2d/bvh.h"


RIGID2D_NAMESPACE_BEGIN

BVHNode::BVHNode(std::vector<std::shared_ptr<Triangle>> triangles) {
    std::unordered_set<std::shared_ptr<Vertex>> set;
    for (const auto& trig : triangles) {
        set.insert(trig->v1);
        set.insert(trig->v2);
        set.insert(trig->v3);
    }
    std::vector<std::shared_ptr<Vertex>> vertices(set.begin(), set.end());
}

RIGID2D_NAMESPACE_END
