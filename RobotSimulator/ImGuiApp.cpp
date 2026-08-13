#include "ImGuiApp.h"

#include "GlLoader.h"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "implot.h"

#include <GLFW/glfw3.h>

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif

#include <cstdio>

namespace {

// GLSL version string handed to the ImGui GL3 backend. It must agree with the context
// requested below, and with the scene shader's own version header.
#ifdef __EMSCRIPTEN__
constexpr const char* kGlslVersion = "#version 300 es";
#else
constexpr const char* kGlslVersion = "#version 330 core";
#endif

void glfwErrorCallback(int error, const char* description) {
    std::fprintf(stderr, "GLFW error %d: %s\n", error, description ? description : "(none)");
}

} // namespace

// If IMGUI_USER_CONFIG fails to reach ImGui's headers the build silently keeps 16-bit
// indices and the web build asserts at runtime the first time a trajectory is plotted, since
// WebGL cannot split draw lists past 64K vertices. Fail here instead, at compile time.
static_assert(sizeof(ImDrawIdx) == 4,
              "ImDrawIdx must be 32-bit. Check IMGUI_USER_CONFIG in build/imgui.pri.");

ImGuiApp::~ImGuiApp() {
    shutdown();
}

bool ImGuiApp::initialize(const std::string& title, int width, int height,
                          std::string* errorMessage, bool visible) {
    const auto fail = [&](const char* message) {
        if (errorMessage) *errorMessage = message;
        shutdown();
        return false;
    };

    glfwSetErrorCallback(glfwErrorCallback);
    if (!glfwInit()) return fail("Failed to initialise GLFW.");

#ifdef __EMSCRIPTEN__
    // Emscripten maps this onto a WebGL2 context.
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
#else
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#endif
    glfwWindowHint(GLFW_DEPTH_BITS, 24);
#ifndef __EMSCRIPTEN__
    // Command-line renders use the exact production OpenGL path without flashing a window or
    // touching desktop input. GLFW still creates a context; it simply never shows the surface.
    glfwWindowHint(GLFW_VISIBLE, visible ? GLFW_TRUE : GLFW_FALSE);
#else
    (void)visible;
#endif

    m_window = glfwCreateWindow(width, height, title.c_str(), nullptr, nullptr);
    if (!m_window) return fail("Failed to create a GLFW window. An OpenGL 3.3 context is required.");

    glfwMakeContextCurrent(m_window);
    glfwSwapInterval(1);

    // Must follow makeContextCurrent: glfwGetProcAddress needs a current context. The scene
    // renderer calls shader, buffer, VAO and framebuffer entry points that opengl32.lib does
    // not export, so a failure here means the context is not the profile we asked for.
    if (!GlLoader::initialize()) {
        return fail("Required OpenGL 3.3 entry points are unavailable. See stderr for the list.");
    }

#ifndef __EMSCRIPTEN__
    // Browsers report a scale via devicePixelRatio and ImGui already accounts for it, so
    // this is only consulted on desktop.
    float scaleX = 1.0f;
    float scaleY = 1.0f;
    glfwGetWindowContentScale(m_window, &scaleX, &scaleY);
    m_dpiScale = scaleX > 0.0f ? scaleX : 1.0f;
#endif

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    m_imguiReady = true;
    ImPlot::CreateContext();
    m_implotReady = true;

    ImGuiIO& io = ImGui::GetIO();
    // Docking gives the dockable-panel layout. Viewports are deliberately NOT enabled:
    // they need real OS windows, which do not exist in a browser tab.
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    loadFonts();
    applyNativeStyle();

    if (!ImGui_ImplGlfw_InitForOpenGL(m_window, true)) return fail("ImGui GLFW backend failed to initialise.");
    if (!ImGui_ImplOpenGL3_Init(kGlslVersion)) return fail("ImGui OpenGL3 backend failed to initialise.");

#ifdef __EMSCRIPTEN__
    ImGui_ImplGlfw_InstallEmscriptenCallbacks(m_window, "#canvas");
#endif

    if (errorMessage) errorMessage->clear();
    return true;
}

void ImGuiApp::loadFonts() {
    ImGuiIO& io = ImGui::GetIO();
    // 13px matches Segoe UI 9pt, the Windows default.
    const float fontSize = 13.0f * m_dpiScale;

#ifdef __EMSCRIPTEN__
    // Preloaded into the Emscripten filesystem by build/imgui.pri. Segoe UI is not
    // redistributable and there is no system font path in a browser, so this stands in for
    // it; the proportions are close enough that the layout metrics still hold.
    if (io.Fonts->AddFontFromFileTTF("/fonts/DroidSans.ttf", fontSize) != nullptr) {
        return;
    }
#else
    if (io.Fonts->AddFontFromFileTTF("C:/Windows/Fonts/segoeui.ttf", fontSize) != nullptr) {
        return;
    }
#endif
    ImFontConfig config;
    config.SizePixels = fontSize;
    io.Fonts->AddFontDefault(&config);
}

void ImGuiApp::applyNativeStyle() {
    ImGui::StyleColorsLight();
    ImGuiStyle& style = ImGui::GetStyle();

    style.FrameRounding = 2.0f;
    style.GrabRounding = 2.0f;
    style.WindowRounding = 0.0f;
    style.ChildRounding = 2.0f;
    style.PopupRounding = 2.0f;
    style.ScrollbarRounding = 2.0f;
    style.TabRounding = 2.0f;

    style.WindowBorderSize = 1.0f;
    style.FrameBorderSize = 1.0f;
    style.PopupBorderSize = 1.0f;

    style.FramePadding = ImVec2(6.0f, 4.0f);
    style.ItemSpacing = ImVec2(8.0f, 6.0f);
    style.ItemInnerSpacing = ImVec2(6.0f, 4.0f);
    style.WindowPadding = ImVec2(8.0f, 8.0f);
    style.ScrollbarSize = 14.0f;
    style.GrabMinSize = 10.0f;

    ImVec4* colors = style.Colors;
    const ImVec4 accent(0.0f, 0.47f, 0.84f, 1.0f);        // Windows selection blue
    const ImVec4 accentHovered(0.10f, 0.55f, 0.90f, 1.0f);
    const ImVec4 surface(0.98f, 0.98f, 0.98f, 1.0f);
    const ImVec4 border(0.72f, 0.72f, 0.72f, 1.0f);

    colors[ImGuiCol_WindowBg] = ImVec4(0.94f, 0.94f, 0.94f, 1.0f);
    colors[ImGuiCol_ChildBg] = surface;
    colors[ImGuiCol_FrameBg] = surface;
    colors[ImGuiCol_FrameBgHovered] = ImVec4(0.93f, 0.96f, 0.99f, 1.0f);
    colors[ImGuiCol_FrameBgActive] = ImVec4(0.88f, 0.93f, 0.98f, 1.0f);
    colors[ImGuiCol_Border] = border;
    colors[ImGuiCol_CheckMark] = accent;
    colors[ImGuiCol_SliderGrab] = accent;
    colors[ImGuiCol_SliderGrabActive] = accentHovered;
    colors[ImGuiCol_Button] = ImVec4(0.90f, 0.90f, 0.90f, 1.0f);
    colors[ImGuiCol_ButtonHovered] = ImVec4(0.87f, 0.93f, 0.98f, 1.0f);
    colors[ImGuiCol_ButtonActive] = ImVec4(0.80f, 0.88f, 0.96f, 1.0f);
    colors[ImGuiCol_Header] = ImVec4(0.90f, 0.90f, 0.90f, 1.0f);
    colors[ImGuiCol_HeaderHovered] = ImVec4(0.87f, 0.93f, 0.98f, 1.0f);
    colors[ImGuiCol_HeaderActive] = accent;
    colors[ImGuiCol_Tab] = ImVec4(0.90f, 0.90f, 0.90f, 1.0f);
    colors[ImGuiCol_TabHovered] = ImVec4(0.87f, 0.93f, 0.98f, 1.0f);
    colors[ImGuiCol_TabActive] = surface;
    colors[ImGuiCol_TitleBgActive] = ImVec4(0.90f, 0.90f, 0.90f, 1.0f);
    colors[ImGuiCol_TextSelectedBg] = ImVec4(accent.x, accent.y, accent.z, 0.35f);

    // Deliberately not ScaleAllSizes(m_dpiScale): the font is already sized by the scale, and
    // doing both compounds the scaling. Metrics above are in logical pixels.
}

#ifdef __EMSCRIPTEN__
void ImGuiApp::syncCanvasToBrowserSize() {
    // GLFW created the canvas at the size main() asked for and nothing resizes it afterwards,
    // so a browser window that is not exactly that size leaves the UI letterboxed or clipped.
    const int width = EM_ASM_INT({ return window.innerWidth; });
    const int height = EM_ASM_INT({ return window.innerHeight; });
    if (width <= 0 || height <= 0) return;
    if (width == m_canvasWidth && height == m_canvasHeight) return;
    m_canvasWidth = width;
    m_canvasHeight = height;
    glfwSetWindowSize(m_window, width, height);
}
#endif

void ImGuiApp::run(const std::function<void()>& drawFrame,
                   const std::function<void()>& beforeFrame) {
    if (!m_window) return;

    const auto renderOneFrame = [this, &drawFrame, &beforeFrame]() {
        glfwPollEvents();
#ifdef __EMSCRIPTEN__
        syncCanvasToBrowserSize();
#endif

        if (beforeFrame) beforeFrame();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        if (drawFrame) drawFrame();

        ImGui::Render();

        int displayWidth = 0;
        int displayHeight = 0;
        glfwGetFramebufferSize(m_window, &displayWidth, &displayHeight);
        glViewport(0, 0, displayWidth, displayHeight);
        const ImVec4 clear = ImGui::GetStyle().Colors[ImGuiCol_WindowBg];
        glClearColor(clear.x, clear.y, clear.z, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(m_window);
    };

#ifdef __EMSCRIPTEN__
    // The browser owns the frame clock, so the loop is inverted: hand the callback over and
    // return. Asyncify lets beforeFrame yield without ending the frame; see the header for why the
    // yield has to be there and not inside drawFrame.
    static std::function<void()> frameCallback;
    frameCallback = renderOneFrame;
    emscripten_set_main_loop([]() { frameCallback(); }, 0, /*simulate_infinite_loop=*/1);
#else
    while (!glfwWindowShouldClose(m_window)) {
        renderOneFrame();
    }
#endif
}

void ImGuiApp::shutdown() {
    if (m_imguiReady) {
        // Only tear down the backends if they got as far as being initialised; calling these
        // after a failed init would dereference null backend state.
        if (ImGui::GetIO().BackendRendererUserData) ImGui_ImplOpenGL3_Shutdown();
        if (ImGui::GetIO().BackendPlatformUserData) ImGui_ImplGlfw_Shutdown();
    }
    if (m_implotReady) {
        ImPlot::DestroyContext();
        m_implotReady = false;
    }
    if (m_imguiReady) {
        ImGui::DestroyContext();
        m_imguiReady = false;
    }
    if (m_window) {
        glfwDestroyWindow(m_window);
        m_window = nullptr;
    }
    glfwTerminate();
}
