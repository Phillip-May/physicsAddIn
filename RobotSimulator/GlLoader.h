#pragma once

// Modern GL entry points for the scene renderer.

#ifdef __EMSCRIPTEN__

#include <GLES3/gl3.h>

namespace GlLoader {
// Nothing to resolve; GLES3 symbols are linked directly.
inline bool initialize() { return true; }
} // namespace GlLoader

#else

#ifdef _WIN32
// GL/gl.h on Windows does not define APIENTRY or WINGDIAPI itself and fails to compile
// unless windows.h has been included first. NOMINMAX keeps its min/max macros from
// colliding with std::min/std::max, which this codebase uses heavily.
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#endif

#include <GL/gl.h>
#include <cstddef>

// Types GL/gl.h predates.
typedef char GLchar;
typedef ptrdiff_t GLsizeiptr;
typedef ptrdiff_t GLintptr;

#ifndef GL_ARRAY_BUFFER
#define GL_ARRAY_BUFFER 0x8892
#define GL_STATIC_DRAW 0x88E4
#define GL_DYNAMIC_DRAW 0x88E8
#define GL_FRAGMENT_SHADER 0x8B30
#define GL_VERTEX_SHADER 0x8B31
#define GL_COMPILE_STATUS 0x8B81
#define GL_LINK_STATUS 0x8B82
#define GL_INFO_LOG_LENGTH 0x8B84
#define GL_FRAMEBUFFER 0x8D40
#define GL_RENDERBUFFER 0x8D41
#define GL_COLOR_ATTACHMENT0 0x8CE0
#define GL_DEPTH_ATTACHMENT 0x8D00
#define GL_DEPTH_COMPONENT24 0x81A6
#define GL_DEPTH_STENCIL_ATTACHMENT 0x821A
#define GL_DEPTH24_STENCIL8 0x88F0
#define GL_FRAMEBUFFER_COMPLETE 0x8CD5
#define GL_CLAMP_TO_EDGE 0x812F
#endif

// name, return type, parameter list. Kept as one list so declaration, definition and
// resolution cannot drift apart.
#define GL_FUNCTION_LIST(X) \
    X(glCreateShader, GLuint, (GLenum)) \
    X(glShaderSource, void, (GLuint, GLsizei, const GLchar* const*, const GLint*)) \
    X(glCompileShader, void, (GLuint)) \
    X(glGetShaderiv, void, (GLuint, GLenum, GLint*)) \
    X(glGetShaderInfoLog, void, (GLuint, GLsizei, GLsizei*, GLchar*)) \
    X(glDeleteShader, void, (GLuint)) \
    X(glCreateProgram, GLuint, (void)) \
    X(glAttachShader, void, (GLuint, GLuint)) \
    X(glLinkProgram, void, (GLuint)) \
    X(glGetProgramiv, void, (GLuint, GLenum, GLint*)) \
    X(glGetProgramInfoLog, void, (GLuint, GLsizei, GLsizei*, GLchar*)) \
    X(glUseProgram, void, (GLuint)) \
    X(glDeleteProgram, void, (GLuint)) \
    X(glGetUniformLocation, GLint, (GLuint, const GLchar*)) \
    X(glGetAttribLocation, GLint, (GLuint, const GLchar*)) \
    X(glUniformMatrix4fv, void, (GLint, GLsizei, GLboolean, const GLfloat*)) \
    X(glUniform4fv, void, (GLint, GLsizei, const GLfloat*)) \
    X(glUniform1f, void, (GLint, GLfloat)) \
    X(glUniform1i, void, (GLint, GLint)) \
    X(glGenBuffers, void, (GLsizei, GLuint*)) \
    X(glBindBuffer, void, (GLenum, GLuint)) \
    X(glBufferData, void, (GLenum, GLsizeiptr, const void*, GLenum)) \
    X(glDeleteBuffers, void, (GLsizei, const GLuint*)) \
    X(glEnableVertexAttribArray, void, (GLuint)) \
    X(glDisableVertexAttribArray, void, (GLuint)) \
    X(glVertexAttribPointer, void, (GLuint, GLint, GLenum, GLboolean, GLsizei, const void*)) \
    X(glGenVertexArrays, void, (GLsizei, GLuint*)) \
    X(glBindVertexArray, void, (GLuint)) \
    X(glDeleteVertexArrays, void, (GLsizei, const GLuint*)) \
    X(glGenFramebuffers, void, (GLsizei, GLuint*)) \
    X(glBindFramebuffer, void, (GLenum, GLuint)) \
    X(glFramebufferTexture2D, void, (GLenum, GLenum, GLenum, GLuint, GLint)) \
    X(glDeleteFramebuffers, void, (GLsizei, const GLuint*)) \
    X(glCheckFramebufferStatus, GLenum, (GLenum)) \
    X(glGenRenderbuffers, void, (GLsizei, GLuint*)) \
    X(glBindRenderbuffer, void, (GLenum, GLuint)) \
    X(glRenderbufferStorage, void, (GLenum, GLenum, GLsizei, GLsizei)) \
    X(glFramebufferRenderbuffer, void, (GLenum, GLenum, GLenum, GLuint)) \
    X(glDeleteRenderbuffers, void, (GLsizei, const GLuint*))

// The pointers are declared at global scope under the real GL names so call sites read the
// same on both platforms.
#define GL_DECLARE_POINTER(name, ret, args) extern ret(APIENTRY* name) args;
GL_FUNCTION_LIST(GL_DECLARE_POINTER)
#undef GL_DECLARE_POINTER

namespace GlLoader {
// Resolves every entry point through glfwGetProcAddress. Returns false if any is missing,
// which means the context is not the 3.3 core profile the renderer needs.
bool initialize();
} // namespace GlLoader

#endif // __EMSCRIPTEN__
