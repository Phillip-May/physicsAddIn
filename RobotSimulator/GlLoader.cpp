#include "GlLoader.h"

#ifndef __EMSCRIPTEN__

#include <GLFW/glfw3.h>

#include <cstdio>

#define GL_DEFINE_POINTER(name, ret, args) ret(APIENTRY* name) args = nullptr;
GL_FUNCTION_LIST(GL_DEFINE_POINTER)
#undef GL_DEFINE_POINTER

namespace GlLoader {

bool initialize() {
    bool complete = true;

#define GL_RESOLVE_POINTER(name, ret, args)                                        \
    name = reinterpret_cast<ret(APIENTRY*) args>(glfwGetProcAddress(#name));        \
    if (name == nullptr) {                                                          \
        std::fprintf(stderr, "OpenGL entry point not available: %s\n", #name);       \
        complete = false;                                                           \
    }
    GL_FUNCTION_LIST(GL_RESOLVE_POINTER)
#undef GL_RESOLVE_POINTER

    return complete;
}

} // namespace GlLoader

#endif // __EMSCRIPTEN__
