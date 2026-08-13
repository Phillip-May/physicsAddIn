#include "CliHardwareCommands.h"

#ifndef ROBOTSIM_NO_SERIAL

#include <chrono>
#include <cstdio>
#include <fstream>
#include <functional>
#include <iostream>
#include <thread>

#include "CliProgramCommands.h"
#include "HardwareIo.h"
#include "JsonCompat.h"
#include "ProgramTextIo.h"
#include "StringUtil.h"


int hardwareIoListCommand() {
    const std::vector<SerialPort::PortInfo> ports = SerialPort::availablePorts();
    if (ports.empty()) {
        std::cout << "No serial ports found." << std::endl;
        return 1;
    }

    // Narrower than the QSerialPortInfo version, which also printed the manufacturer and
    // the USB VID/PID. SerialPort enumerates the registry's SERIALCOMM map, which carries
    // the name and description only; the description still identifies a USB serial device.
    for (const SerialPort::PortInfo& info : ports) {
        std::cout << info.portName;
        if (!info.description.empty()) std::cout << "  " << info.description;
        std::cout << std::endl;
    }
    return 0;
}

// Opens the link for a CLI command. HardwareIo::connectTo sends hello and get_status
// immediately, but opening the port asserts DTR and resets the board, so those two land in
// the bootloader window and are dropped. The settle below covers the reset; each command
// asks for what it needs afterwards.
namespace {

bool openHardwareIo(HardwareIo* io, const std::string& portName, std::string* errorMessage) {
    if (!io->connectTo(portName)) {
        if (errorMessage) *errorMessage = io->lastError();
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(750));
    io->poll();
    return true;
}

bool pumpHardwareIo(HardwareIo* io, int timeoutMs, const std::function<bool()>& done,
                    std::string* errorMessage) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
    while (true) {
        io->poll();
        if (done()) return true;
        if (!io->isConnected()) {
            if (errorMessage) *errorMessage = "Serial port closed unexpectedly.";
            return false;
        }
        if (std::chrono::steady_clock::now() >= deadline) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    if (errorMessage) *errorMessage = "Timed out waiting for serial response.";
    return false;
}

void printHardwareStatus(const Json& object) {
    const Json poll = jsoncompat::fieldObject(object, "limit_poll");
    std::cout << "robot_status" << std::endl;
    std::cout << "  limit_poll_period_ms=" << jsoncompat::fieldInt(poll, "period_ms") << std::endl;
    std::cout << "  measured_hz=" << jsoncompat::fieldInt(poll, "measured_hz_x100") / 100.0 << std::endl;
    std::cout << "  poll_count=" << jsoncompat::fieldInt(poll, "count") << std::endl;
    std::cout << "  limits_raw=" << jsoncompat::fieldArray(object, "limits_raw").dump().c_str()
              << "  (raw digitalRead values)" << std::endl;
    std::cout << "  limits_pressed=" << jsoncompat::fieldArray(object, "limits_active").dump().c_str()
              << "  (1=pressed)" << std::endl;
    std::cout << "  limit_changes=" << jsoncompat::fieldArray(object, "limit_changes").dump().c_str() << std::endl;
    const Json jog = jsoncompat::fieldObject(object, "jog");
    std::cout << "  jog_armed=" << (jsoncompat::fieldInt(jog, "armed") == 1 ? "true" : "false")
              << " allowed_joints=" << jsoncompat::fieldArray(jog, "allowed_joints").dump().c_str()
              << " max_steps=" << jsoncompat::fieldInt(jog, "max_steps") << std::endl;
    const Json mastering = jsoncompat::fieldObject(object, "mastering");
    std::cout << "  mastering_active=" << (jsoncompat::fieldInt(mastering, "active") == 1 ? "true" : "false")
              << " active_joint=" << jsoncompat::fieldInt(mastering, "active_joint") << std::endl;
    std::cout << "  mastered=" << jsoncompat::fieldArray(mastering, "mastered").dump().c_str() << std::endl;
    std::cout << "  master_dir=" << jsoncompat::fieldArray(mastering, "master_dir").dump().c_str() << std::endl;
    std::cout << "  current_steps=" << jsoncompat::fieldArray(mastering, "current_steps").dump().c_str() << std::endl;
    std::cout << "  zero_steps=" << jsoncompat::fieldArray(mastering, "zero_steps").dump().c_str() << std::endl;
    std::cout << "  master_limit_steps=" << jsoncompat::fieldArray(mastering, "master_limit_steps").dump().c_str() << std::endl;
    std::cout << "  position_deg_x100=" << jsoncompat::fieldArray(mastering, "position_deg_x100").dump().c_str() << std::endl;
    std::cout << "  position_deg_x10000000=" << jsoncompat::fieldArray(mastering, "position_deg_x10000000").dump().c_str() << std::endl;
    std::cout << "  offset_deg_x100=" << jsoncompat::fieldArray(mastering, "offset_deg_x100").dump().c_str() << std::endl;
    std::cout << "  offset_deg_x10000000=" << jsoncompat::fieldArray(mastering, "offset_deg_x10000000").dump().c_str() << std::endl;
    std::cout << "  steps_per_deg_x1000=" << jsoncompat::fieldArray(mastering, "steps_per_deg_x1000").dump().c_str() << std::endl;
    std::cout << "  steps_per_deg_x10000000=" << jsoncompat::fieldArray(mastering, "steps_per_deg_x10000000").dump().c_str() << std::endl;
}

} // namespace

int hardwareIoStatusCommand(const std::vector<std::string>& args) {
    if (args.size() < 3) {
        std::cerr << "Usage: RobotSimulator.exe --hardware-io-status COM5 [timeout_ms]" << std::endl;
        return 1;
    }

    const std::string portName = args[2];
    const int timeoutMs = args.size() >= 4 ? std::max(500, strutil::parseIntOr(args[3], 0)) : 5000;
    std::string errorMessage;
    HardwareIo io;
    if (!openHardwareIo(&io, portName, &errorMessage)) {
        // SerialPort's message already names the port, so it is not prefixed again here.
        std::cerr << errorMessage << std::endl;
        return 1;
    }

    bool printed = false;
    io.setMessageCallback([&printed](const std::string& line, const Json& object) {
        if (printed) return;
        if (object.empty() || jsoncompat::fieldString(object, "msg") != "robot_status") {
            // Anything else the firmware volunteers is echoed, as the raw-serial version did.
            std::cout << line << std::endl;
            return;
        }
        printHardwareStatus(object);
        printed = true;
    });

    // period_ms is sent even when disabling, unlike the raw-serial version which omitted it.
    // The firmware defaults the field to 250 when absent and stores it either way, so the
    // resulting state is the same.
    io.setStatusStreaming(false, 250);
    io.requestStatus();

    if (!pumpHardwareIo(&io, timeoutMs, [&printed] { return printed; }, &errorMessage)) {
        if (!io.isConnected()) {
            std::cerr << errorMessage << std::endl;
            return 1;
        }
        std::cerr << "Timed out waiting for robot_status." << std::endl;
        return 1;
    }
    return 0;
}

namespace {

Json jsonArrayOfSix(const Json& object, const char* key, double fallback) {
    Json values = jsoncompat::fieldArray(object, key);
    while (values.size() < 6) values.push_back(fallback);
    return values;
}

Json masteringLoadCommandFromFile(const std::string& path, bool restoreReference, std::string* errorMessage) {
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        if (errorMessage) *errorMessage = "Failed to open file: " + path;
        return {};
    }

    const std::string bytes((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    const Json doc = Json::parse(bytes.data(), bytes.data() + bytes.size(),
                                 nullptr, /*allow_exceptions=*/false);
    if (doc.is_discarded() || !doc.is_object()) {
        if (errorMessage) *errorMessage = "bad JSON";
        return {};
    }

    const Json& root = doc;
    Json command = Json::object();
    command["cmd"] = "load_mastering";

    if (jsoncompat::member(root, "joints").is_array()) {
        Json mastered = Json::array();
        Json currentSteps = Json::array();
        Json zeroSteps = Json::array();
        Json masterLimitSteps = Json::array();
        Json stepsPerDegree = Json::array();
        Json offsetDeg = Json::array();
        Json masterDir = Json::array();
        const Json joints = jsoncompat::fieldArray(root, "joints");
        for (int i = 0; i < 6; ++i) {
            const Json& joint = static_cast<size_t>(i) < joints.size() ? jsoncompat::toObject(joints[static_cast<size_t>(i)]) : jsoncompat::emptyObject();
            mastered.push_back(jsoncompat::fieldBool(joint, "mastered", false) ? 1 : 0);
            if (jsoncompat::contains(joint, "master_dir")) {
                masterDir.push_back(jsoncompat::fieldInt(joint, "master_dir", 1) >= 0 ? 1 : -1);
            }
            const int current = jsoncompat::fieldInt(joint, "current_steps", 0);
            currentSteps.push_back(current);
            zeroSteps.push_back(jsoncompat::fieldInt(joint, "zero_steps", current));
            masterLimitSteps.push_back(jsoncompat::fieldInt(joint, "master_limit_steps", 0));
            stepsPerDegree.push_back(jsoncompat::fieldDouble(joint, "steps_per_deg", 1.0));
            offsetDeg.push_back(jsoncompat::fieldDouble(joint, "offset_deg", 0.0));
        }
        if (restoreReference) {
            command["mastered"] = mastered;
            command["current_steps"] = currentSteps;
            command["zero_steps"] = zeroSteps;
            command["master_limit_steps"] = masterLimitSteps;
        }
        command["steps_per_deg"] = stepsPerDegree;
        command["offset_deg"] = offsetDeg;
        if (masterDir.size() == 6) command["master_dir"] = masterDir;
        return command;
    }

    if (restoreReference) {
        command["mastered"] = jsonArrayOfSix(root, "mastered", 0);
        command["current_steps"] = jsonArrayOfSix(root, "current_steps", 0);
        command["zero_steps"] = jsonArrayOfSix(root, "zero_steps", 0);
        command["master_limit_steps"] = jsonArrayOfSix(root, "master_limit_steps", 0);
    }
    command["steps_per_deg"] = jsonArrayOfSix(root, "steps_per_deg", 1.0);
    command["offset_deg"] = jsonArrayOfSix(root, "offset_deg", 0.0);
    if (jsoncompat::contains(root, "master_dir")) {
        command["master_dir"] = jsonArrayOfSix(root, "master_dir", 1);
    }
    return command;
}

} // namespace

int hardwareIoLoadMasteringCommand(const std::vector<std::string>& args) {
    if (args.size() < 4) {
        std::cerr << "Usage: RobotSimulator.exe --hardware-io-load-mastering COM5 mastering.json [--calibration-only] [timeout_ms]" << std::endl;
        return 1;
    }

    const std::string portName = args[2];
    const std::string filePath = args[3];
    bool restoreReference = true;
    int timeoutMs = 6000;
    for (int i = 4; i < args.size(); ++i) {
        if (args[i] == "--calibration-only") {
            restoreReference = false;
        } else {
            timeoutMs = std::max(500, strutil::parseIntOr(args[i], 0));
        }
    }

    std::string errorMessage;
    const Json command = masteringLoadCommandFromFile(filePath, restoreReference, &errorMessage);
    if (command.empty()) {
        std::cerr << "Failed to read mastering file: " << errorMessage << std::endl;
        return 1;
    }

    HardwareIo io;
    if (!openHardwareIo(&io, portName, &errorMessage)) {
        // SerialPort's message already names the port, so it is not prefixed again here.
        std::cerr << errorMessage << std::endl;
        return 1;
    }

    // Watches both stages: first the load acknowledgement, then the status that follows it.
    bool loaded = false;
    bool failed = false;
    std::string failureCode;
    bool statusSeen = false;
    io.setMessageCallback([&](const std::string& line, const Json& object) {
        if (object.empty()) {
            std::cout << line << std::endl;
            return;
        }
        const std::string msg = jsoncompat::fieldString(object, "msg");
        if (msg == "mastering_load_failed") {
            failureCode = jsoncompat::fieldString(object, "code", "unknown");
            failed = true;
        } else if (msg == "mastering_load_done") {
            loaded = true;
        } else if (msg == "robot_status") {
            if (loaded && !statusSeen) {
                std::cout << "mastering_load_done" << std::endl;
                printHardwareStatus(object);
                statusSeen = true;
            }
        } else if (!loaded) {
            std::cout << line << std::endl;
        }
    });

    io.setStatusStreaming(false, 250);
    io.sendCommand(command);

    if (!pumpHardwareIo(&io, timeoutMs, [&] { return loaded || failed; }, &errorMessage)) {
        if (!io.isConnected()) {
            std::cerr << errorMessage << std::endl;
            return 1;
        }
        std::cerr << "Timed out waiting for mastering_load_done." << std::endl;
        return 1;
    }
    if (failed) {
        std::cerr << "Load mastering failed: " << failureCode << std::endl;
        return 1;
    }

    io.requestStatus();
    if (!pumpHardwareIo(&io, timeoutMs, [&statusSeen] { return statusSeen; }, &errorMessage)) {
        if (!io.isConnected()) {
            std::cerr << errorMessage << std::endl;
            return 1;
        }
        std::cerr << "Timed out waiting for robot_status after load." << std::endl;
        return 1;
    }
    return 0;
}

// Sends one command line and prints whatever comes back for a few seconds.
int hardwareIoSendCommand(const std::vector<std::string>& args) {
    if (args.size() < 4) {
        std::cerr << "Usage: RobotSimulator.exe --hardware-io-send COM5 '{\"cmd\":\"save_config\"}' [seconds]"
                  << std::endl;
        return 1;
    }

    const std::string portName = args[2];
    const std::string commandLine = args[3];
    const int seconds = args.size() >= 5 ? std::max(1, strutil::parseIntOr(args[4], 3)) : 3;

    // Rejected before opening the port: a malformed line would be sent and silently ignored by the
    // firmware, which looks identical to the command not existing.
    const Json parsed = Json::parse(commandLine, nullptr, /*allow_exceptions=*/false);
    if (parsed.is_discarded() || !parsed.is_object()) {
        std::cerr << "Command must be a JSON object." << std::endl;
        return 1;
    }

    std::string errorMessage;
    HardwareIo io;
    if (!openHardwareIo(&io, portName, &errorMessage)) {
        std::cerr << errorMessage << std::endl;
        return 1;
    }

    io.setMessageCallback([](const std::string& line, const Json&) {
        std::cout << line << std::endl;
    });
    io.sendCommand(parsed);

    const auto start = std::chrono::steady_clock::now();
    const auto limit = std::chrono::milliseconds(static_cast<long long>(seconds) * 1000);
    while ((std::chrono::steady_clock::now() - start) < limit) {
        io.poll();
        if (!io.isConnected()) {
            std::cerr << "Serial port closed unexpectedly." << std::endl;
            return 1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return 0;
}

int hardwareIoMonitorCommand(const std::vector<std::string>& args) {
    if (args.size() < 3) {
        std::cerr << "Usage: RobotSimulator.exe --hardware-io-monitor COM5 [seconds]" << std::endl;
        return 1;
    }

    const std::string portName = args[2];
    const int seconds = args.size() >= 4 ? std::max(0, strutil::parseIntOr(args[3], 0)) : 0;
    std::string errorMessage;
    HardwareIo io;
    if (!openHardwareIo(&io, portName, &errorMessage)) {
        // SerialPort's message already names the port, so it is not prefixed again here.
        std::cerr << errorMessage << std::endl;
        return 1;
    }

    io.setMessageCallback([](const std::string& line, const Json&) {
        // Echoes the firmware verbatim, parsed or not, which is the point of monitoring.
        std::cout << line << std::endl;
    });
    io.setStatusStreaming(true, 250);

    const auto start = std::chrono::steady_clock::now();
    const auto limit = std::chrono::milliseconds(static_cast<long long>(seconds) * 1000);
    while (seconds <= 0 || (std::chrono::steady_clock::now() - start) < limit) {
        io.poll();
        if (!io.isConnected()) {
            std::cerr << "Serial port closed unexpectedly." << std::endl;
            return 1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    io.setStatusStreaming(false, 250);
    return 0;
}


#endif // ROBOTSIM_NO_SERIAL
