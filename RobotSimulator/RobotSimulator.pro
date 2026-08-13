TEMPLATE = app
CONFIG += c++17
# No Qt. Strings and formatting are Common/StringUtil.h, JSON is nlohmann via
# Common/JsonCompat.h, zip is third_party/miniz, the maths types are RobotSimulator/SceneMath.h,
# and file I/O is <fstream> and <filesystem>.
#
# qmake still generates the Makefiles: it is a build-time tool here, not a runtime dependency.
CONFIG -= qt

# The renderer is shader/VBO based, so no GLU and no desktop-only GL entry points are needed.
wasm {
    # Do not let the legacy Qt 5.15 Emscripten mkspec emit a plausible but backend-free build.
    # It pins Emscripten 1.39 and injects pthread flags incompatible with the current CPU PhysX
    # archives. scripts/build_wasm.ps1 is the one supported WebAssembly target and owns the whole
    # web configuration (defines, link flags, preloaded image).
    error("RobotSimulator WASM must be built with scripts/build_wasm.ps1 (includes CPU PhysX 5.9)")
} else {
    CONFIG += console
    LIBS += -lopengl32
    include(../build/physx-runtime.pri)

    # Every native invocation writes the executable to the same repo-owned location. Shadow build
    # directories may hold compiler intermediates, but they must never each publish a plausible
    # RobotSimulator.exe: that makes it impossible to tell which binary contains the current code.
    CONFIG(debug, debug|release) {
        ROBOTSIM_OUTPUT_CONFIG = debug
    } else {
        ROBOTSIM_OUTPUT_CONFIG = release
    }
    DESTDIR = $$clean_path($$PWD/../dist/RobotSimulator/$$ROBOTSIM_OUTPUT_CONFIG)
    message(RobotSimulator executable: $$DESTDIR/RobotSimulator.exe)
}

TARGET = RobotSimulator
INCLUDEPATH += . ../Common

# Dear ImGui + ImPlot + GLFW. Kept out of deps.pri, which requires OCCT and PhysX that
# this target does not link.
include(../build/imgui.pri)

# nlohmann/json and miniz, replacing QJson* and Qt's private zip reader.
include(../build/thirdparty.pri)

SOURCES += \
    main.cpp \
    AccessoryBuilders.cpp \
    ../Common/AccessoryGeometry.cpp \
    ../Common/AccessoryPropertySchema.cpp \
    ../Common/CadNodeDraw.cpp \
    ../Common/ConveyorGeometry.cpp \
    ../Common/LibraryCatalogue.cpp \
    ../Common/MountingSnap.cpp \
    ../Common/PlacedMechanismSchema.cpp \
    ../Common/PlacementSession.cpp \
    ../Common/ViewRay.cpp \
    AppState.cpp \
    ConveyorRuntime.cpp \
    SceneConveyorHost.cpp \
    FirmwareProgram.cpp \
    MasteringIo.cpp \
    UiFormHelpers.cpp \
    CliHardwareCommands.cpp \
    CliInspectCommands.cpp \
    CliProgramCommands.cpp \
    CliStationCommands.cpp \
    ProgramTextIo.cpp \
    StationParameterLinks.cpp \
    StationSceneLoad.cpp \
    LibraryPlacement.cpp \
    ViewerBridge.cpp \
    TrajectoryAndLiveRun.cpp \
    ProgramEditing.cpp \
    ProgramWidgets.cpp \
    HardwareIoPanel.cpp \
    ProgramPanel.cpp \
    SimulationInfoPanel.cpp \
    TimelinePanel.cpp \
    RobotPanel.cpp \
    PlacementWindows.cpp \
    StationPanels.cpp \
    ContextBar.cpp \
    AppFrame.cpp \
    ImGuiApp.cpp \
    GlLoader.cpp \
    HardwareIo.cpp \
    ConveyorPhysics.cpp \
    DragChainPhysics.cpp \
    LiveRunDriver.cpp \
    MeshRobotViewer.cpp \
    RobotLibraryPanel.cpp \
    RobotProgramModel.cpp \
    RobotProgramSimulator.cpp \
    WebFiles.cpp \
    ../Common/CadNodePackage.cpp \
    ../Common/ConveyorCore.cpp \
    ../Common/RobotRuntime.cpp \
    ../Common/SerialPort.cpp \
    ../Common/StationPackage.cpp

HEADERS += \
    AccessoryBuilders.h \
    ../Common/AccessoryGeometry.h \
    ../Common/AccessoryPropertySchema.h \
    ../Common/CadNodeDraw.h \
    ../Common/ConveyorGeometry.h \
    ../Common/LibraryCatalogue.h \
    ../Common/MountingSnap.h \
    ../Common/PlacedMechanismSchema.h \
    ../Common/PlacementSession.h \
    ../Common/SelectionStyle.h \
    ../Common/ViewRay.h \
    AppState.h \
    ConveyorRuntime.h \
    FirmwareProgram.h \
    MasteringIo.h \
    UiFormHelpers.h \
    ../Common/UnitsMath.h \
    CliHardwareCommands.h \
    CliInspectCommands.h \
    CliProgramCommands.h \
    CliStationCommands.h \
    ProgramTextIo.h \
    StationParameterLinks.h \
    StationSceneLoad.h \
    LibraryPlacement.h \
    ViewerBridge.h \
    TrajectoryAndLiveRun.h \
    ProgramEditing.h \
    ProgramWidgets.h \
    HardwareIoPanel.h \
    ProgramPanel.h \
    SimulationInfoPanel.h \
    TimelinePanel.h \
    RobotPanel.h \
    PlacementWindows.h \
    StationPanels.h \
    ContextBar.h \
    AppFrame.h \
    ImGuiApp.h \
    GlLoader.h \
    HardwareIo.h \
    ConveyorPhysics.h \
    DragChainPhysics.h \
    LiveRunDriver.h \
    RollingRateWindow.h \
    MeshRobotViewer.h \
    RobotLibraryPanel.h \
    RobotProgramModel.h \
    RobotProgramSimulator.h \
    SceneMath.h \
    WebFiles.h \
    ../Common/CadNode.h \
    ../Common/CadNodePackage.h \
    ../Common/RobotMotionCore.h \
    ../Common/RobotRuntime.h \
    ../Common/SerialPort.h \
    ../Common/StationPackage.h

FORMS +=
