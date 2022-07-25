#ifndef RIGID2D_BVH_H
#define RIGID2D_BVH_H

#include <vector>
#include <memory>

#include "rigid2d/common.h"
#include "rigid2d/geometry.h"
#include "rigid2d/shader.h"


RIGID2D_NAMESPACE_BEGIN

class RigidBody;


class BVHNode {
private:
    // Bouding volume attached to the node
    Circle bounding_disc_;
    // Rigid body attached to the node
    std::shared_ptr<RigidBody> body_;
    // Triangle attached to the node (only non-null if the node is a leaf)
    std::shared_ptr<Triangle> leaf_triangle_;

    // Left child of the node
    std::shared_ptr<BVHNode> left_child_;
    // Right child of the node
    std::shared_ptr<BVHNode> right_child_;

    friend class RigidBody;

public:
    explicit BVHNode(const std::vector<std::shared_ptr<Triangle>>& triangles);

    [[nodiscard]] inline bool IsLeaf() const {
        return !leaf_triangle_;
    }
};

RIGID2D_NAMESPACE_END

#endif // RIGID2D_BVH_H
