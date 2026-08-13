#include "AppFrame.h"
#include "AppState.h"
#include "CliHardwareCommands.h"
#include "CliInspectCommands.h"
#include "CliProgramCommands.h"
#include "CliStationCommands.h"
#include "ImGuiApp.h"
#include "RobotLibraryPanel.h"
#include "StationPackage.h"
#include "TrajectoryAndLiveRun.h"

#include <filesystem>
#include <iostream>
#include <string>
#include <system_error>
#include <thread>
#include <vector>
int main(int argc, char* argv[]) {
    // Plain argc/argv. There is no QCoreApplication: nothing here needs a Qt event loop, and on
    // WebAssembly constructing one would pull in the platform plugin that competes with GLFW for
    // the page canvas.
    std::vector<std::string> args;
    args.reserve(static_cast<size_t>(argc));
    for (int i = 0; i < argc; ++i) args.push_back(argv[i]);

    // A binary can identify itself without loading graphics, packages or hardware. The canonical
    // output path prevents duplicate current builds; this makes support logs prove which actual
    // file was launched and when it was compiled instead of relying on its shared filename.
    if (args.size() >= 2 && args[1] == "--build-info") {
        std::error_code ignored;
        const std::filesystem::path executable =
            std::filesystem::absolute(std::filesystem::path(args[0]), ignored);
        std::cout << "RobotSimulator built " << __DATE__ << " " << __TIME__
                  << " executable=" << executable.string() << std::endl;
        return 0;
    }

#ifndef __EMSCRIPTEN__
    {
        std::error_code ignored;
        std::filesystem::path exeDirectory =
            std::filesystem::absolute(std::filesystem::path(args[0]), ignored).parent_path();
        if (!exeDirectory.empty()) {
            setBuiltinRobotCatalogueRoot((exeDirectory / "packages").string());
        }
    }
#endif

    if (args.size() >= 2 && args[1] == "--validate-gripper-demo") {
        return validateGripperDemoCommand(args);
    }
    if (args.size() >= 2 && args[1] == "--validate-station-program-loop") {
        return validateStationProgramLoopCommand(args);
    }
    if (args.size() >= 2 && args[1] == "--dump-station-program-ik") {
        return dumpStationProgramIkCommand(args);
    }
    if (args.size() >= 2 && args[1] == "--dump-conveyor-trace") {
        return dumpConveyorTraceCommand(args);
    }

#ifndef ROBOTSIM_NO_SERIAL
    if (args.size() >= 2 && args[1] == "--hardware-io-list") {
        return hardwareIoListCommand();
    }

    if (args.size() >= 2 && args[1] == "--hardware-io-status") {
        return hardwareIoStatusCommand(args);
    }

    if (args.size() >= 2 && args[1] == "--hardware-io-load-mastering") {
        return hardwareIoLoadMasteringCommand(args);
    }

    if (args.size() >= 2 && args[1] == "--hardware-io-send") {
        return hardwareIoSendCommand(args);
    }

    if (args.size() >= 2 && args[1] == "--hardware-io-monitor") {
        return hardwareIoMonitorCommand(args);
    }
#endif

    if (args.size() >= 2 && args[1] == "--simulate-program") {
        return simulateProgramCommand(args);
    }

    if (args.size() >= 2 && args[1] == "--render-station") {
        return renderStationCommand(args);
    }

    if (args.size() >= 2 && args[1] == "--live-run") {
        return liveRunCommand(args, /*threaded=*/false);
    }

    if (args.size() >= 2 && args[1] == "--live-run-threaded") {
        return liveRunCommand(args, /*threaded=*/true);
    }

    if (args.size() >= 3 && args[1] == "--dump-package-programs") {
        return dumpPackageProgramsCommand(args);
    }

    if (args.size() >= 2 && args[1] == "--generate-rect-program") {
        return generateRectProgramCommand(args);
    }

    if (args.size() >= 2 && args[1] == "--generate-triangle-weave-program") {
        return generateRectProgramCommand(args, true);
    }

    if (args.size() >= 2 && args[1] == "--collision-pose") {
        return collisionPoseCommand(args);
    }

    if (args.size() >= 3 && args[1] == "--validate-package") return validatePackageCommand(args);
    if (args.size() >= 3 && args[1] == "--validate-station") return validateStationCommand(args);
    if (args.size() >= 5 && args[1] == "--orientation") return orientationCommand(args);
    if (args.size() >= 2 && args[1] == "--list-robots") return listRobotsCommand(args);
    if (args.size() >= 4 && args[1] == "--resave-station") return resaveStationCommand(args);
    if (args.size() >= 3 && args[1] == "--dump-axes") return dumpAxesCommand(args);
    if (args.size() >= 3 && args[1] == "--tool-pose") return toolPoseCommand(args);
    if (args.size() >= 3 && args[1] == "--dump-robot-model") return dumpRobotModelCommand(args);
    if (args.size() >= 8 && args[1] == "--station-ik-position") return stationIkPositionCommand(args);
    if (args.size() >= 3 && args[1] == "--ik-smoke") return ikSmokeCommand(args);
    if (args.size() >= 4 && args[1] == "--bake-hulls") return bakeHullsCommand(args);
    std::string initialPackage;
    if (args.size() >= 2) initialPackage = args[1];
#ifdef __EMSCRIPTEN__
    // There is no command line in a browser tab, so open the shipped machine-tending cell. Its
    // builtin references resolve against the complete /packages catalogue preloaded by the WASM
    // build, including the FAIRINO robot, Gudel rail, Robotiq tool and conveyor accessories.
    if (initialPackage.empty()) {
        const std::string station = "/stations/fairino_gudel_machine_tending.station.json";
        if (std::filesystem::exists(station)) initialPackage = station;
    }
    if (initialPackage.empty()) {
        const std::vector<BuiltinRobot> catalogue = listBuiltinRobots();
        if (!catalogue.empty()) initialPackage = catalogue.front().path;
    }
#endif

    ImGuiApp gui;
    std::string guiError;
    if (!gui.initialize("RobotSimulator", 1400, 900, &guiError)) {
        std::cerr << "Failed to start the user interface: " << guiError << std::endl;
        return 1;
    }
    // Keep one live-run worker for the UI session; scene changes replace its arm list.
#ifndef __EMSCRIPTEN__
    g_scene.liveRunThread = std::thread(liveRunThreadMain);
#endif

    gui.run([&]() { drawRobotSimulatorUi(initialPackage); },
            []() {
                // Outside the ImGui frame: this mutates the scene graph, and on the web it yields.
#ifdef __EMSCRIPTEN__
                pumpLiveRunsForFrame();
#endif
                applyLiveRunResults();
            });

    // These own GL buffers and textures, so release them before the local ImGuiApp destroys the
    // GLFW context. The scene viewer historically survived until global teardown; the library adds
    // several more renderers, making the correct lifetime worth stating and enforcing here.
    g_robotLibrary.releaseGraphics();
    g_scene.viewer.reset();

#ifndef __EMSCRIPTEN__
    // Before g_scene's arms go anywhere. The window has closed, so nothing is going to read what the
    // last slice published.
    g_scene.liveRunThreadStop.store(true);
    if (g_scene.liveRunThread.joinable()) g_scene.liveRunThread.join();
#endif
    return 0;
}
