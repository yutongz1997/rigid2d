#include <vector>
#include <memory>
#include <unordered_set>

#include "rigid2d/common.h"
#include "rigid2d/bvh.h"
#include "rigid2d/body.h"
#include "rigid2d/geometry.h"


RIGID2D_NAMESPACE_BEGIN

void BoundingDisc::BuildDrawingBuffer(unsigned int num_segments) {
    for (int i = 0; i < num_segments; ++i) {
        auto angle = static_cast<float>(2 * M_PI * i / num_segments);
        vertex_buffer_.push_back(center_local_.x() + radius_ * std::cos(angle));
        vertex_buffer_.push_back(center_local_.y() + radius_ * std::sin(angle));
        index_buffer_.push_back(i % num_segments);
        index_buffer_.push_back((i + 1) % num_segments);
    }
}


void BoundingDisc::GenerateVAO() {
    glGenVertexArrays(1, &vao_);
    GLuint vbo, ibo;
    glGenBuffers(1, &vbo);
    glGenBuffers(1, &ibo);

    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(vertex_buffer_.size() * sizeof(GLfloat)),
                 &vertex_buffer_[0], GL_STATIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(GLfloat), nullptr);
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(index_buffer_.size() * sizeof(GLuint)),
                 &index_buffer_[0], GL_STATIC_DRAW);
    glBindVertexArray(0);
}


BoundingDisc::BoundingDisc(const std::vector<std::shared_ptr<Vertex>>& vertices,
                           RigidBody* body,
                           unsigned int num_segments) {
    body_ = body;

    Circle mec = Circle::WelzlCircle(vertices);
    center_local_ = mec.center;
    radius_ = mec.radius;
    center_world_ = body->PointToWorld(center_local_);

    BuildDrawingBuffer(num_segments);
    vao_ = 0;
    GenerateVAO();
}


void BoundingDisc::UpdateCenterWorld() {
    center_world_ = body_->PointToWorld(center_local_);
}


void BoundingDisc::Render(const std::shared_ptr<Shader>& disc_shader, const Eigen::Matrix4f& ortho) const {
    disc_shader->Use();
    disc_shader->SetMatrix4f("model", body_->local_to_world_.TransformationMatrix());
    disc_shader->SetMatrix4f("projection", ortho);
    glBindVertexArray(vao_);
    glDrawElements(GL_LINES, static_cast<GLsizei>(index_buffer_.size()),
                   GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);
}


BVHNode::BVHNode(const std::vector<std::shared_ptr<Triangle>>& triangles,
                 RigidBody* body) {
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
    volume_ = std::make_unique<BoundingDisc>(vertices, body);

    leaf_triangle_ = (triangles.size() == 1) ? triangles[0] : nullptr;
    visit_id_ = -1;

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
        left_child_ = std::make_shared<BVHNode>(left_triangles, body);
        right_child_ = std::make_shared<BVHNode>(right_triangles, body);
    }
}


void BVHNode::Render(const std::shared_ptr<Shader>& disc_shader,
                     const Eigen::Matrix4f& ortho) const {
    volume_->Render(disc_shader, ortho);
    if (left_child_) {
        left_child_->Render(disc_shader, ortho);
    }
    if (right_child_) {
        right_child_->Render(disc_shader, ortho);
    }
}


void BVHNode::RenderVisitBoundary(const std::shared_ptr<Shader>& disc_shader,
                                  const Eigen::Matrix4f& ortho,
                                  int visit) const {
    if (IsLeaf() || left_child_->visit_id_ != visit) {
        volume_->Render(disc_shader, ortho);
    } else {
        left_child_->RenderVisitBoundary(disc_shader, ortho, visit);
        right_child_->RenderVisitBoundary(disc_shader, ortho, visit);
    }
}

RIGID2D_NAMESPACE_END
