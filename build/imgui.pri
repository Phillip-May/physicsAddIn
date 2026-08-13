# Dear ImGui + ImPlot + GLFW configuration for the ImGui-based UI.
#
# Deliberately separate from deps.pri: that file requires OpenCascade and PhysX, which
# RobotSimulator does not link. Machine-specific overrides belong in build/local-env.pri,
# which is ignored by git.
#
# These three live under external/ and are NOT committed, matching external/CoACD.
#   git clone --depth 1 --branch docking https://github.com/ocornut/imgui.git external/imgui
#   git clone --depth 1 https://github.com/epezent/implot.git external/implot
#   GLFW 3.4 Windows binaries unzipped to external/glfw (desktop only; the web build
#   gets GLFW from Emscripten).
#
# The docking branch is required: docking is not in ImGui master.

IMGUI_REPO_ROOT = $$clean_path($$PWD/..)

isEmpty(IMGUI_ROOT):  IMGUI_ROOT  = $${IMGUI_REPO_ROOT}/external/imgui
isEmpty(IMPLOT_ROOT): IMPLOT_ROOT = $${IMGUI_REPO_ROOT}/external/implot
isEmpty(GLFW_ROOT):   GLFW_ROOT   = $${IMGUI_REPO_ROOT}/external/glfw

isEmpty(LOCAL_ENV_PRI_INCLUDED) {
    LOCAL_ENV_PRI_INCLUDED = 1
    exists($$PWD/local-env.pri): include($$PWD/local-env.pri)
}

!exists($${IMGUI_ROOT}/imgui.cpp) {
    error("Dear ImGui not found at $${IMGUI_ROOT}. Clone the docking branch into external/imgui.")
}
!exists($${IMGUI_ROOT}/backends/imgui_impl_glfw.cpp) {
    error("ImGui GLFW backend not found. Clone the full imgui repository, not just the headers.")
}
!exists($${IMPLOT_ROOT}/implot.cpp) {
    error("ImPlot not found at $${IMPLOT_ROOT}. Clone it into external/implot.")
}

INCLUDEPATH += $${IMGUI_ROOT} $${IMGUI_ROOT}/backends $${IMPLOT_ROOT}

# Applies to every target that includes this file, ImGui's own sources included, so the whole
# build agrees on sizeof(ImDrawIdx). See RobotSimulator/imgui_user_config.h for why it is
# widened to 32 bits.
INCLUDEPATH += $${IMGUI_REPO_ROOT}/RobotSimulator
DEFINES += IMGUI_USER_CONFIG=\\\"imgui_user_config.h\\\"

SOURCES += \
    $${IMGUI_ROOT}/imgui.cpp \
    $${IMGUI_ROOT}/imgui_draw.cpp \
    $${IMGUI_ROOT}/imgui_tables.cpp \
    $${IMGUI_ROOT}/imgui_widgets.cpp \
    $${IMGUI_ROOT}/backends/imgui_impl_glfw.cpp \
    $${IMGUI_ROOT}/backends/imgui_impl_opengl3.cpp \
    $${IMPLOT_ROOT}/implot.cpp \
    $${IMPLOT_ROOT}/implot_items.cpp

# Desktop only: the web build is scripts/build_wasm.ps1, which gets GLFW from Emscripten's own
# port and never reaches this file (RobotSimulator.pro error()s under wasm).
!exists($${GLFW_ROOT}/include/GLFW/glfw3.h) {
    error("GLFW not found at $${GLFW_ROOT}. Unzip the GLFW Windows binaries into external/glfw.")
}
INCLUDEPATH += $${GLFW_ROOT}/include
# lib-vc2019 matches the MSVC v142 toolchain the rest of the project is built with.
LIBS += -L$${GLFW_ROOT}/lib-vc2019 -lglfw3
# comdlg32 supplies GetOpenFileName/GetSaveFileName, used for the native file dialogs
# since ImGui has none of its own.
# advapi32 supplies the registry calls SerialPort uses to enumerate COM ports.
LIBS += -lopengl32 -lgdi32 -lshell32 -luser32 -lcomdlg32 -ladvapi32
