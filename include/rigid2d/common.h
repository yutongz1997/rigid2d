#ifndef RIGID2D_COMMON_H
#define RIGID2D_COMMON_H

#define RIGID2D_NAMESPACE_BEGIN namespace rigid2d {
#define RIGID2D_NAMESPACE_END   }

#include <Eigen/Core>


static float Cross2(float x1, float y1, float x2, float y2) {
    return x1 * y2 - x2 * y1;
}

static float Cross2(const Eigen::Vector2f& v1, const Eigen::Vector2f& v2) {
    return Cross2(v1.x(), v1.y(), v2.x(), v2.y());
}

#endif // RIGID2D_COMMON_H
