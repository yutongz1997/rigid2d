#ifndef RIGID2D_COMMON_H
#define RIGID2D_COMMON_H

#include <limits>

#include <Eigen/Core>


#define RIGID2D_NAMESPACE_BEGIN namespace rigid2d {
#define RIGID2D_NAMESPACE_END   }


RIGID2D_NAMESPACE_BEGIN

// Constant of machine epsilon for single precision floating numbers
constexpr float kEpsilon = std::numeric_limits<float>::epsilon();
// Constant of (positive) infinity
constexpr float kInf = std::numeric_limits<float>::infinity();


template <typename T>
static T Clamp(T x, T lower, T upper) {
    return std::max(lower, std::min(x, upper));
}


static float Cross2(float x1, float y1, float x2, float y2) {
    return x1 * y2 - x2 * y1;
}


static float Cross2(const Eigen::Vector2f& v1, const Eigen::Vector2f& v2) {
    return Cross2(v1.x(), v1.y(), v2.x(), v2.y());
}


static Eigen::Vector2f VectorTripleProduct(const Eigen::Vector2f& v1,
                                           const Eigen::Vector2f& v2,
                                           const Eigen::Vector2f& v3) {
    return v1.dot(v3) * v2 - v2.dot(v3) * v1;
}

RIGID2D_NAMESPACE_END

#endif // RIGID2D_COMMON_H
