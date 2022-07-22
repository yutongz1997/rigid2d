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
};

RIGID2D_NAMESPACE_END

#endif // RIGID2D_SHADER_H
