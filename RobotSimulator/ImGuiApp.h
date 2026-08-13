#pragma once

#include <functional>
#include <string>

struct GLFWwindow;

// Window, GL context, ImGui/ImPlot lifetime and the frame loop.
class ImGuiApp {
public:
    ImGuiApp() = default;
    ~ImGuiApp();

    ImGuiApp(const ImGuiApp&) = delete;
    ImGuiApp& operator=(const ImGuiApp&) = delete;

    // Creates the window and initialises ImGui/ImPlot. Returns false and fills
    // errorMessage on failure.
    bool initialize(const std::string& title, int width, int height, std::string* errorMessage,
                    bool visible = true);

    // Runs until the window closes. drawFrame is called once per frame, between
    // NewFrame and Render, and should emit the dockspace and all panels.
    void run(const std::function<void()>& drawFrame,
             const std::function<void()>& beforeFrame = nullptr);

    GLFWwindow* window() const { return m_window; }

    float dpiScale() const { return m_dpiScale; }

private:
    void applyNativeStyle();
    void loadFonts();
    void shutdown();
#ifdef __EMSCRIPTEN__
    // Resizes the canvas to the browser viewport when it changes. Called once per frame.
    void syncCanvasToBrowserSize();
#endif

    GLFWwindow* m_window = nullptr;
    float m_dpiScale = 1.0f;
    bool m_imguiReady = false;
    bool m_implotReady = false;
#ifdef __EMSCRIPTEN__
    int m_canvasWidth = 0;
    int m_canvasHeight = 0;
#endif
};
