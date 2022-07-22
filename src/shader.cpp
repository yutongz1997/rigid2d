#include <iostream>
#include <fstream>
#include <string>

#include <fmt/core.h>

#include "rigid2d/common.h"
#include "rigid2d/shader.h"


RIGID2D_NAMESPACE_BEGIN

GLuint Shader::LoadShader(const char *vertex_source,
                          const char *fragment_source) {
    // For error message recording purposes
    int status;
    char info_log[512];

    // Load the vertex shader and check if there is any error
    GLuint vertex_shader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertex_shader, 1, &vertex_source, nullptr);
    glCompileShader(vertex_shader);
    glGetShaderiv(vertex_shader, GL_COMPILE_STATUS, &status);
    if (!status) {
        glGetShaderInfoLog(vertex_shader, 512, nullptr, info_log);
        fmt::print(stderr,
                   "[Shader] Error: Vertex shader compilation failed. Reason: {}.\n",
                   info_log);
        return 0;
    }

    // Load the fragment shader and check if there is any error
    GLuint fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragment_shader, 1, &fragment_source, nullptr);
    glCompileShader(fragment_shader);
    glGetShaderiv(fragment_shader, GL_COMPILE_STATUS, &status);
    if (!status) {
        glGetShaderInfoLog(fragment_shader, 512, nullptr, info_log);
        fmt::print(stderr,
                   "Error: Fragment shader compilation failed. Reason: {}.\n",
                   info_log);
        return 0;
    }

    // Create the shader program and check if there is any error
    GLuint program = glCreateProgram();
    glAttachShader(program, vertex_shader);
    glAttachShader(program, fragment_shader);
    glLinkProgram(program);
    glGetProgramiv(program, GL_LINK_STATUS, &status);
    if (!status) {
        glGetProgramInfoLog(program, 512, nullptr, info_log);
        fmt::print(stderr,
                   "[Shader] Error: Shaders linking failed. Reason: {}.\n",
                   info_log);
        return 0;
    }

    glDetachShader(program, vertex_shader);
    glDetachShader(program, fragment_shader);
    glDeleteShader(vertex_shader);
    glDeleteShader(fragment_shader);

    return program;
}


Shader::Shader(const std::string &vertex_shader,
               const std::string &fragment_shader,
               char mode) {
    id_ = 0;

    // Identify given shader strings as source codes
    if (mode == 's') {
        id_ = Shader::LoadShader(vertex_shader.c_str(), fragment_shader.c_str());
    }

    // Identify given shader strings as file paths
    else if (mode == 'f') {
        // Ensure ifstream objects can throw exceptions
        std::ifstream vertex_file, fragment_file;
        vertex_file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
        fragment_file.exceptions(std::ifstream::failbit | std::ifstream::badbit);

        std::string vertex_source, fragment_source;
        bool success = true;
        try {
            // Open the given vertex and fragment shader files
            vertex_file.open(vertex_shader);
            fragment_file.open(fragment_shader);
            // Read each shader file's buffer contents into streams
            std::stringstream vertex_stream, fragment_stream;
            vertex_stream << vertex_file.rdbuf();
            fragment_stream << fragment_file.rdbuf();
            // Convert streams into strings
            vertex_source = vertex_stream.str();
            fragment_source = fragment_stream.str();
            // Close the file handlers
            vertex_file.close();
            fragment_file.close();
        }
        catch (const std::ifstream::failure& e) {
            fmt::print(stderr,
                       "[Shader] Error: Failed to read the shader file(s). Reason: {}.\n",
                       e.what());
            success = false;
        }

        if (success) {
            id_ = Shader::LoadShader(vertex_source.c_str(), fragment_source.c_str());
        }
    }

    // Invalid shader mode is given
    else {
        fmt::print(stderr,
                   "[Shader] Error: Invalid shader mode '{}'.\n",
                   mode);
    }
}

RIGID2D_NAMESPACE_END
