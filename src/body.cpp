#include <queue>
#include <numeric>

#include "rigid2d/common.h"
#include "rigid2d/body.h"


RIGID2D_NAMESPACE_BEGIN

void RigidBody::PopulateBVHBuffer(int n) {
    std::queue<std::shared_ptr<BVHNode>> queue;
    queue.push(bvh_root_);
    while (!queue.empty()) {
        auto node = queue.front();
        queue.pop();

        Eigen::Vector2f center = node->bounding_disc_.center;
        float radius = node->bounding_disc_.radius;
        GLuint idx_start = bvh_vertex_buffer_.size() / 2;
        for (int i = 0; i < n; ++i) {
            auto angle = static_cast<float>(2 * M_PI * i / n);
            bvh_vertex_buffer_.push_back(center.x() + radius * std::cos(angle));
            bvh_vertex_buffer_.push_back(center.y() + radius * std::sin(angle));
            bvh_index_buffer_.push_back(idx_start + i % n);
            bvh_index_buffer_.push_back(idx_start + (i + 1) % n);
        }

        if (node->left_child_) {
            queue.push(node->left_child_);
        }
        if (node->right_child_) {
            queue.push(node->right_child_);
        }
    }
}


/**
 * See https://physics.stackexchange.com/questions/708936/how-to-calculate-the-moment-of-inertia-of-convex-polygon-two-dimensions
 */
void RigidBody::ComputeRotationalInertia() {
    // Compute the mass moment of inertia (MMOI) per unit mass for each triangle
    // and accumulate the contribution by the area of each triangle
    std::size_t num_triangles = geometry_->NumTriangles();
    for (int i = 0; i < num_triangles; ++i) {
        auto& v1 = geometry_->triangles_[i]->v1;
        auto& v2 = geometry_->triangles_[i]->v2;
        auto& v3 = geometry_->triangles_[i]->v3;
        float unit_mmoi = (v1->dot(*v1) + v2->dot(*v2) + v3->dot(*v3) +
                v1->dot(*v2) + v2->dot(*v3) + v1->dot(*v3)) / 6.0f;
        rotational_inertia_ += geometry_->areas_[i] * unit_mmoi;
    }
    rotational_inertia_ *= (mass_ / geometry_->area_);
}


void RigidBody::UpdateTransformation() {
    body_to_world.Set(position_, orientation_);
    world_to_body.SetInverse(position_, orientation_);
}


RigidBody::RigidBody(const std::shared_ptr<TriangleMesh> &geometry,
                     const std::shared_ptr<Shader> &shader,
                     const Eigen::Vector2f &position,
                     float orientation,
                     float mass) {
    geometry_ = geometry;
    shader_ = shader;

    bvh_root_ = std::make_shared<BVHNode>(geometry_->triangles_);
    PopulateBVHBuffer();

    bvh_vao_ = 0;
    glGenVertexArrays(1, &bvh_vao_);
    GLuint bvh_vbo, bvh_ibo;
    glGenBuffers(1, &bvh_vbo);
    glGenBuffers(1, &bvh_ibo);
    // Populate vertices and indices data into the VBO and IBO
    const auto& vertex_buffer = bvh_vertex_buffer_;
    glBindVertexArray(bvh_vao_);
    glBindBuffer(GL_ARRAY_BUFFER, bvh_vbo);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(bvh_vertex_buffer_.size() * sizeof(GLfloat)),
                 &bvh_vertex_buffer_[0], GL_STATIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(GLfloat), nullptr);
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, bvh_ibo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(bvh_index_buffer_.size() * sizeof(GLuint)),
                 &bvh_index_buffer_[0], GL_STATIC_DRAW);
    glBindVertexArray(0);

    mass_ = mass;
    inverse_mass_ = (mass == kInf || mass == -kInf) ? 0.0f : 1.0f / mass;
    rotational_inertia_ = 0.0f;
    ComputeRotationalInertia();
    inverse_rotational_inertia_ = 1.0f / rotational_inertia_;

    initial_position_ = position;
    position_ = position;
    initial_orientation_ = orientation;
    orientation_ = orientation;
    UpdateTransformation();

    linear_velocity_.setZero();
    angular_velocity_ = 0.0f;
    force_.setZero();
    torque_ = 0.0f;
}


void RigidBody::Reset() {
    position_ = initial_position_;
    orientation_ = initial_orientation_;
    linear_velocity_.setZero();
    angular_velocity_ = 0.0f;
    force_.setZero();
    torque_ = 0.0f;
    UpdateTransformation();
}


void RigidBody::Step(float duration) {
    if (inverse_mass_ > 0.0f) {
        // Advance the state of angular velocity and orientation angle
        angular_velocity_ += inverse_rotational_inertia_ * torque_ * duration;
        orientation_ += angular_velocity_ * duration;
        // Advance the state of linear velocity and position
        linear_velocity_ += inverse_mass_ * force_ * duration;
        position_ += linear_velocity_ * duration;
        // Update rigid transformation matrices after stepping
        UpdateTransformation();
    }
    // Reset the force and torque accumulators
    force_.setZero();
    torque_ = 0.0f;
}

RIGID2D_NAMESPACE_END
