#ifndef RIGID2D_BVH_H
#define RIGID2D_BVH_H

#include <vector>
#include <memory>

#include "rigid2d/common.h"
#include "rigid2d/geometry.h"


RIGID2D_NAMESPACE_BEGIN

class BVHNode {
private:
    Circle bounding_disc_;
    std::shared_ptr<Triangle> leaf_triangle_;

    std::unique_ptr<BVHNode> left_child_;
    std::unique_ptr<BVHNode> right_child_;

public:
    explicit BVHNode(const std::vector<std::shared_ptr<Triangle>>& triangles);
};

RIGID2D_NAMESPACE_END

#endif // RIGID2D_BVH_H
