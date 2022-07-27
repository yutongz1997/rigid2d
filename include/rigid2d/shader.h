#ifndef RIGID2D_SHADER_H
#define RIGID2D_SHADER_H

#include <string>

#include <glad/glad.h>

#include "rigid2d/common.h"


RIGID2D_NAMESPACE_BEGIN

class Shader {
private:
    // Current shader program's ID
    GLuint id_;

private:
    static GLuint LoadShader(const char* vertex_source,
                             const char* fragment_source);

public:
    explicit Shader(const std::string& vertex_shader,
                    const std::string& fragment_shader,
                    char mode = 's');

    inline void Use() const {
        glUseProgram(id_);
    }

    inline void SetVector3f(const std::string& name,
                            const Eigen::Vector3f& vec) const {
        GLint location = glGetUniformLocation(id_, name.c_str());
        glUniform3f(location, vec.x(), vec.y(), vec.z());
    }

    inline void SetMatrix4f(const std::string& name,
                            const Eigen::Matrix4f& mat) const {
        GLint location = glGetUniformLocation(id_, name.c_str());
        glUniformMatrix4fv(location, 1, GL_FALSE, mat.data());
    }
};

RIGID2D_NAMESPACE_END

#endif // RIGID2D_SHADER_H
