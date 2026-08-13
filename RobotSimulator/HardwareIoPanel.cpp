#include "HardwareIoPanel.h"
#include "StringUtil.h"

#include "AppState.h"
#include "FirmwareProgram.h"
#include "HardwareIo.h"
#include "JsonCompat.h"
#include "MasteringIo.h"
#include "ProgramEditing.h"
#include "RobotMotionCore.h"
#include "RobotRuntime.h"
#include "TrajectoryAndLiveRun.h"
#include "UiFormHelpers.h"
#include "ViewerBridge.h"
#include "WebFiles.h"

#include "imgui.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

void saveMasteringToFile() {
#ifdef __EMSCRIPTEN__
    WebFiles::saveText("ar4_mastering.json", masteringDocumentFromEditors().dump(4));
    activeRobot().hardwareStatus = "Mastering handed to the browser to save.";
#else
    const std::string path = runFileDialog("Save mastering", true, kMasteringFilter, L"json");
    if (path.empty()) return;
    std::ofstream file(path, std::ios::trunc);
    if (!file) {
        activeRobot().hardwareStatus = "Save failed: could not open " + path;
        return;
    }
    file << masteringDocumentFromEditors().dump(4);
    activeRobot().hardwareStatus = "Saved mastering: " + path;
#endif
}


void loadMasteringFromFile() {
#ifdef __EMSCRIPTEN__
    WebFiles::requestOpen(WebFiles::Purpose::Mastering, ".json");
    activeRobot().hardwareStatus = "Choose a mastering file...";
#else
    const std::string path = runFileDialog("Load mastering", false, kMasteringFilter, L"json");
    if (path.empty()) return;
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        activeRobot().hardwareStatus = "Load failed: could not open " + path;
        return;
    }
    const std::string bytes((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    const Json document = Json::parse(bytes, nullptr, /*allow_exceptions=*/false);
    if (document.is_discarded() || !document.is_object()) {
        activeRobot().hardwareStatus = "Load failed: not a JSON object.";
        return;
    }
    applyLoadedMasteringDocument(document, path);
#endif
}


// Records where the real robot actually was, from the joint angles in robot_status.
void appendHardwareTraceSample() {
    const HardwareIo::Status& status = activeRobot().hardware.status();
    if (!status.valid || !std::isfinite(status.firmwareSeconds)) return;
    if (status.statusCount == activeRobot().hardwareTraceLastStatusCount) return;
    activeRobot().hardwareTraceLastStatusCount = status.statusCount;

    const RobotMotionCore::RobotModel model = currentRobotModelFor(activeRobot());
    if (!RobotMotionCore::modelIsValid(model)) return;

    const double firmwareSeconds = status.firmwareSeconds;
    HardwareTraceSample sample;
    for (size_t i = 0; i < 6; ++i) sample.joints[i] = status.jointsDeg[i] * kDegToRad;
    sample.tcpPose = RobotMotionCore::toolPoseForJoints(model, sample.joints.data());

    // A firmware timestamp that went backwards means the board was reset, so the old trace no
    // longer shares a clock with the new samples.
    if (std::isfinite(activeRobot().hardwareTraceStartSeconds) && !activeRobot().hardwareTrace.empty() &&
        firmwareSeconds < activeRobot().hardwareTraceStartSeconds) {
        activeRobot().hardwareTrace.clear();
        activeRobot().hardwareTraceStartSeconds = std::numeric_limits<double>::quiet_NaN();
        activeRobot().hardwareTracePendingValid = false;
    }

    if (!std::isfinite(activeRobot().hardwareTraceStartSeconds)) {
        // Still waiting for the first step. Hold this frame back as the candidate departure point
        // and compare the step counts; until they change there is nothing to plot but a flat line
        // the user did not ask to see.
        if (!activeRobot().hardwareTracePendingValid) {
            activeRobot().hardwareTracePendingSample = sample;
            activeRobot().hardwareTracePendingSample.seconds = firmwareSeconds;
            activeRobot().hardwareTracePendingSteps = status.currentSteps;
            activeRobot().hardwareTracePendingValid = true;
            return;
        }
        bool moved = false;
        for (size_t i = 0; i < 6 && !moved; ++i) {
            if (status.currentSteps[i] != activeRobot().hardwareTracePendingSteps[i]) moved = true;
        }
        if (!moved) {
            activeRobot().hardwareTracePendingSample = sample;
            activeRobot().hardwareTracePendingSample.seconds = firmwareSeconds;
            activeRobot().hardwareTracePendingSteps = status.currentSteps;
            return;
        }
        // Motion, somewhere between the held-back frame and this one. The held-back frame is the
        // last time the arm was known to be still, so the trace starts there at zero speed.
        activeRobot().hardwareTraceStartSeconds = activeRobot().hardwareTracePendingSample.seconds;
        activeRobot().hardwareTracePendingValid = false;
        HardwareTraceSample first = activeRobot().hardwareTracePendingSample;
        first.seconds = 0.0;
        activeRobot().hardwareTrace.clear();
        activeRobot().hardwareTrace.push_back(first);
    }

    const double seconds = std::max(0.0, firmwareSeconds - activeRobot().hardwareTraceStartSeconds);
    // Duplicate or out-of-order timestamps would divide by zero in the speed derivation.
    if (!activeRobot().hardwareTrace.empty() && seconds <= activeRobot().hardwareTrace.back().seconds + 1.0e-6) {
        return;
    }

    sample.seconds = seconds;
    activeRobot().hardwareTrace.push_back(sample);

    // Bounded, since a streamed status at 20 ms would otherwise grow without limit over a long
    // session. Dropping from the front keeps the most recent motion, and the start time moves with
    // it so the remaining samples keep their spacing.
    constexpr size_t kMaxHardwareTraceSamples = 4096;
    if (activeRobot().hardwareTrace.size() > kMaxHardwareTraceSamples) {
        const size_t removeCount = activeRobot().hardwareTrace.size() - kMaxHardwareTraceSamples;
        activeRobot().hardwareTrace.erase(activeRobot().hardwareTrace.begin(),
                                    activeRobot().hardwareTrace.begin() + static_cast<std::ptrdiff_t>(removeCount));
        // Retained samples remain relative to the original trace origin.
    }
}

void drawHardwareIoPanel() {
    // This arm's link, not a shared one that had to be told whose it was. No poll here: every
    // connection is drained once a frame by pollAllHardware, so an arm keeps being serviced while
    // you are looking at a different one.
    HardwareIo& hardware = activeRobot().hardware;

    if (hardware.moveInFlight() && hardware.allJointsMastered()) {
        appendHardwareTraceSample();
    } else if (!hardware.moveInFlight()) {
        // Armed for the next move: it starts a fresh trace, timed from its own first step, rather
        // than continuing the last one. The samples already collected stay on the graph until then,
        // so a finished move remains readable.
        activeRobot().hardwareTraceLastStatusCount = hardware.status().statusCount;
        activeRobot().hardwareTraceStartSeconds = std::numeric_limits<double>::quiet_NaN();
        activeRobot().hardwareTracePendingValid = false;
    }

    // Track during move: drive the shown pose from the robot's reported joints while it is
    // moving, so the 3D view follows the real arm instead of the commanded target. Gated on all
    // six joints being mastered, since otherwise the reported degrees are meaningless, and on a
    // move being in flight, so idle status frames do not fight with the user dragging the gimbal.
    const bool syncRequested = activeRobot().poseSyncPending &&
                               hardware.status().statusCount != activeRobot().poseSyncAwaitCount;
    if ((syncRequested || (activeRobot().hardwareTrackDuringMove && hardware.moveInFlight())) &&
        hardware.allJointsMastered() && hardware.status().valid && activeRobot().poseController.isBound()) {
        std::array<double, 6> jointsRad{};
        for (size_t i = 0; i < 6; ++i) jointsRad[i] = hardware.status().jointsDeg[i] * kDegToRad;
        activeRobot().poseController.setJoints(jointsRad);
        if (syncRequested) {
            activeRobot().poseSyncPending = false;
            activeRobot().hardwareStatus = "View moved to the robot's reported position.";
        }
    } else if (syncRequested && hardware.status().valid && !hardware.allJointsMastered()) {
        // The reported degrees are derived from the step counts against a zero that does not exist
        // yet, so moving the view to them would be moving it to a number, not to the arm.
        activeRobot().poseSyncPending = false;
        activeRobot().hardwareStatus = "Cannot follow the robot: the joints are not mastered.";
    }

    // Connection strip.
    if (hardware.ports().empty()) hardware.refreshPorts();

    if (SerialPort::needsPortRequest()) {
        if (ImGui::Button("Request port...")) {
            SerialPort::requestPort();
            hardware.refreshPorts();
        }
        ImGui::SameLine();
        ImGui::TextDisabled("(browser asks which device to allow)");
        const std::string requestError = SerialPort::lastRequestError();
        if (!requestError.empty()) {
            ImGui::TextColored(ImVec4(0.75f, 0.15f, 0.15f, 1.0f), "%s", requestError.c_str());
        }
        ImGui::Spacing();
    }

    ImGui::SetNextItemWidth(240.0f);
    const std::vector<SerialPort::PortInfo>& ports = hardware.ports();
    if (activeRobot().selectedPort < 0 || activeRobot().selectedPort >= static_cast<int>(ports.size())) {
        activeRobot().selectedPort = hardware.suggestedPortIndex();
    }
    const std::string preview = (activeRobot().selectedPort >= 0 && activeRobot().selectedPort < static_cast<int>(ports.size()))
        ? ports[static_cast<size_t>(activeRobot().selectedPort)].portName + " - " +
              ports[static_cast<size_t>(activeRobot().selectedPort)].description
        : std::string("No ports");
    if (ImGui::BeginCombo("##port", preview.c_str())) {
        for (size_t i = 0; i < ports.size(); ++i) {
            const std::string label = ports[i].portName + " - " + ports[i].description;
            if (ImGui::Selectable(label.c_str(), static_cast<int>(i) == activeRobot().selectedPort)) {
                activeRobot().selectedPort = static_cast<int>(i);
            }
        }
        ImGui::EndCombo();
    }
    ImGui::SameLine();
    if (ImGui::Button("Refresh")) hardware.refreshPorts();
    ImGui::SameLine();
    if (!hardware.isConnected()) {
        if (ImGui::Button("Connect") && activeRobot().selectedPort >= 0 &&
            activeRobot().selectedPort < static_cast<int>(ports.size())) {
            hardware.connectTo(ports[static_cast<size_t>(activeRobot().selectedPort)].portName);
        }
    } else if (ImGui::Button("Disconnect")) {
        hardware.disconnect();
    }
    ImGui::SameLine();
    if (ImGui::Button("Get status")) {
        hardware.requestStatus();
        // The reply also moves the view to wherever the arm actually is. Asking where the robot is
        // and then not being shown it was a strange thing for this button to do.
        activeRobot().poseSyncPending = true;
        activeRobot().poseSyncAwaitCount = hardware.status().statusCount;
    }

    ImGui::SameLine();
    ImGui::Dummy(ImVec2(12.0f, 0.0f));
    ImGui::SameLine();
    if (ImGui::Button("Save config")) saveMasteringToFile();
    ImGui::SameLine();
    if (ImGui::Button("Load config")) loadMasteringFromFile();

    // No Stream checkbox: streaming is turned on for the whole session by connectTo. It was a
    // switch for something nothing benefits from having off - every readout on this panel is only
    // as current as the last status frame - and it read as a duplicate of "Track during move",
    // which is the control that actually decides whether the robot drives the view.

    const HardwareIo::Status& status = hardware.status();
    // The model state is on this line because it is a precondition for every move and there is
    // otherwise no way to tell it apart from a robot that is simply refusing to move: the firmware
    // holds the model in RAM, so it is absent after a power cycle even though everything else about
    // the connection looks healthy.
    const char* modelState = !hardware.hasRobotModel() ? "no package"
                             : !hardware.isConnected() ? "pending"
                             : hardware.robotModelAccepted() ? "loaded"
                                                             : "sent";
    ImGui::Text("%s | status msgs %u | firmware %.1f s | model %s",
                hardware.isConnected() ? "Connected" : "Disconnected",
                status.statusCount, status.firmwareSeconds, modelState);
    if (!hardware.lastError().empty()) {
        ImGui::TextWrapped("Error: %s", hardware.lastError().c_str());
    }
    ImGui::Separator();

    // Arm, disarm, step size and Stop sit above the tab bar, at panel level: they gate
    // mastering and Run as well as jogging, so burying them under one tab would hide the
    // control the other tabs depend on.
    const bool armed = status.jogArmed;
    const bool connected = hardware.isConnected();
    // A mastering sweep owns the joint until it finishes, so arming, disarming and
    // jogging stay disabled through it.
    const bool busy = status.masteringActive;
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted("State");
    ImGui::SameLine();
    if (armed) {
        ImGui::TextColored(ImVec4(0.69f, 0.0f, 0.13f, 1.0f), "Armed");
    } else {
        ImGui::TextUnformatted("Disarmed");
    }
    ImGui::SameLine();

    ImGui::BeginDisabled(!connected || armed || busy);
    if (ImGui::Button("Arm jog")) hardware.armJog(true);
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::BeginDisabled(!connected || !armed || busy);
    if (ImGui::Button("Disarm")) hardware.armJog(false);
    ImGui::EndDisabled();
    ImGui::SameLine(0.0f, 16.0f);
    nativeSpinBox("##jogSteps", &activeRobot().hardwareJogSteps, 1.0, "%.0f steps", 1.0, 500.0,
                  90.0f, "Step");
    ImGui::SameLine(0.0f, 16.0f);
    // Stop is deliberately always enabled, including while disarmed, and coloured so it
    // is findable without reading.
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.78f, 0.16f, 0.12f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.88f, 0.24f, 0.18f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.66f, 0.10f, 0.08f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
    if (ImGui::Button("STOP", ImVec2(70.0f, 0.0f))) hardware.stopMotion();
    ImGui::PopStyleColor(4);
    ImGui::Separator();

    // Panel level, not inside a tab: these span the Jog, Mastering and Motion Planner tabs, so
    // tucking them under one of the three implied they only applied to that one.
    if (hardware.motionSettingsReceived()) {
        applyMotionSettingsToEditors(activeRobot(), hardware.lastMotionSettings(), "Read from robot.");
        hardware.clearMotionSettingsReceived();
    }

    if (hardware.weaveSchedulesReceived()) {
        activeRobot().weaveSchedules = hardware.readWeaveSchedules();
        activeRobot().weaveScheduleStatus = "Read from robot.";
        hardware.clearWeaveSchedulesReceived();
    }

    if (robotModelOverrideDiffers(activeRobot())) {
        ImGui::TextColored(ImVec4(0.85f, 0.6f, 0.15f, 1.0f),
                           "Kinematics overridden by the loaded config; the 3D view still shows the package.");
        ImGui::SameLine();
        if (ImGui::SmallButton("Use package")) {
            activeRobot().robotModelOverrideValid = false;
            activeRobot().robotModelOverrideSource.clear();
            // The firmware is holding the override, so it has to be told about the reversion.
            activeRobot().hardware.setRobotModelCommand(
                robotModelCommandForPackage(activeRobot().poseController, currentRobotModelFor(activeRobot())));
            activeRobot().hardwareStatus = "Reverted to the package's kinematics.";
        }
    }

    const bool configBusy = !hardware.isConnected() || status.masteringActive || hardware.moveInFlight();
    // Each label names both ends of the move, because "Send to robot" and "Restore from EEPROM"
    // did not say which of the robot's two stores they touched, and the RAM pair reads as the
    // obvious one to reach for when the intent is only to try a value.
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted("Editors");
    ImGui::SameLine();
    ImGui::BeginDisabled(configBusy);
    if (ImGui::Button("Write to robot RAM")) {
        // Calibration only: this pushes what the editors hold, and the zero reference is not the
        // editors' to move - restoring that is Load mastering's job, behind its own question.
        applyMasteringToFirmware(masteringDocumentFromEditors(), false);
        hardware.sendMotionSettings(motionSettingsFromEditors());
        // The welding schedules travel with the rest of the editors. A program refers to them by
        // index, so leaving them behind would have the robot weld a different pattern from the one
        // the simulation just showed - or refuse the move outright.
        hardware.sendWeaveSchedules(activeRobot().weaveSchedules);
        activeRobot().hardwareStatus = "Written to the robot's RAM. Not saved: use Write RAM to EEPROM for that.";
    }
    ImGui::SameLine();
    if (ImGui::Button("Read from robot RAM")) {
        // Reads what the robot is running now, not what it has stored. The mastering editors
        // already follow status frames; this drops the sticky edited flags so they resume doing so,
        // and asks for the motion settings, which only arrive when requested.
        hardware.requestStatus();
        hardware.requestMotionSettings();
        hardware.requestWeaveSchedules();
        for (size_t i = 0; i < 6; ++i) {
            activeRobot().masterDirectionEdited[i] = false;
            activeRobot().jogDirInvertEdited[i] = false;
        }
        activeRobot().hardwareStatus = "Read the robot's live values into the editors.";
    }
    ImGui::EndDisabled();

    // These two are editor-relative, with the trip through the robot's RAM implicit. Saving to
    // EEPROM only ever stores what the robot holds, so writing the editors there means pushing them
    // to RAM first; loading only ever lands in RAM, so the editors pick it up from the status that
    // follows. Making the caller do those halves would be a way to save one thing and observe
    // another.
    ImGui::SameLine(0.0f, 18.0f);
    ImGui::BeginDisabled(configBusy);
    if (ImGui::Button("Write to robot EEPROM")) {
        applyMasteringToFirmware(masteringDocumentFromEditors(), false);
        hardware.sendMotionSettings(motionSettingsFromEditors());
        hardware.sendWeaveSchedules(activeRobot().weaveSchedules);
        Json command = Json::object();
        command["cmd"] = "save_config";
        hardware.sendCommand(command);
        activeRobot().hardwareStatus = "Written to the robot and saved to its EEPROM.";
    }
    ImGui::SameLine();
    if (ImGui::Button("Read from robot EEPROM")) {
        Json command = Json::object();
        command["cmd"] = "load_config";
        hardware.sendCommand(command);
        hardware.requestMotionSettings();
        hardware.requestWeaveSchedules();
        for (size_t i = 0; i < 6; ++i) {
            activeRobot().masterDirectionEdited[i] = false;
            activeRobot().jogDirInvertEdited[i] = false;
        }
        activeRobot().hardwareStatus = "Loaded the saved configuration into the robot and the editors.";
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::TextUnformatted(status.configStored ? "(saved configuration present)" : "(nothing saved)");
    if (activeRobot().masteringConfirmPending) {
        ImGui::OpenPopup("Confirm robot zero");
        activeRobot().masteringConfirmPending = false;
    }
    if (ImGui::BeginPopupModal("Confirm robot zero", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::TextUnformatted("Is the physical robot already at the saved 0 position?");
        ImGui::Spacing();
        ImGui::TextWrapped("Restore: put back the saved step reference and mark the joints "
                           "mastered. Only correct if the arm is standing at that pose.");
        ImGui::TextWrapped("Calibration only: send Steps/deg and Offset deg, leaving the "
                           "robot's own zero reference alone.");
        ImGui::Spacing();
        if (ImGui::Button("Restore reference")) {
            applyMasteringToFirmware(activeRobot().pendingMasteringDocument, true);
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Calibration only")) {
            applyMasteringToFirmware(activeRobot().pendingMasteringDocument, false);
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel")) {
            activeRobot().hardwareStatus = "Load config cancelled; nothing sent to the robot.";
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }

    if (!activeRobot().hardwareStatus.empty() && activeRobot().hardwareStatus != "-") {
        ImGui::TextWrapped("%s", activeRobot().hardwareStatus.c_str());
    }
    ImGui::Separator();

    if (ImGui::BeginTabBar("hardwareTabs")) {
        if (ImGui::BeginTabItem("Jog")) {

            // The jog buttons are fixed narrow columns; without WidthFixed every column takes an
            // equal share and the two button columns swallow half the table.
            if (ImGui::BeginTable("jog", 5, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                                                ImGuiTableFlags_SizingStretchProp)) {
                const float jogButtonWidth = 44.0f;
                ImGui::TableSetupColumn("Joint", ImGuiTableColumnFlags_WidthStretch);
                ImGui::TableSetupColumn("Limit", ImGuiTableColumnFlags_WidthFixed, 70.0f);
                ImGui::TableSetupColumn("-", ImGuiTableColumnFlags_WidthFixed, jogButtonWidth);
                ImGui::TableSetupColumn("+", ImGuiTableColumnFlags_WidthFixed, jogButtonWidth);
                ImGui::TableSetupColumn("Invert", ImGuiTableColumnFlags_WidthFixed, 52.0f);
                ImGui::TableHeadersRow();
                for (int joint = 0; joint < 6; ++joint) {
                    ImGui::TableNextRow();
                    ImGui::PushID(joint);
                    ImGui::TableSetColumnIndex(0);
                    ImGui::AlignTextToFramePadding();
                    ImGui::Text("J%d  %.2f deg", joint + 1, status.jointsDeg[static_cast<size_t>(joint)]);
                    ImGui::TableSetColumnIndex(1);
                    ImGui::AlignTextToFramePadding();
                    const bool pressed = status.limitPressed[static_cast<size_t>(joint)];
                    if (pressed) {
                        ImGui::TextColored(ImVec4(0.85f, 0.25f, 0.20f, 1.0f), "pressed");
                    } else {
                        ImGui::TextDisabled("-");
                    }
                    // Disabled unless connected and armed: pressing them while disarmed only earns
                    // a firmware rejection.
                    ImGui::BeginDisabled(!connected || !armed || busy);
                    // Suffixes avoid ImGui ID collisions across headers and joint rows.
                    ImGui::TableSetColumnIndex(2);
                    if (ImGui::Button("-##jogMinus", ImVec2(-1.0f, 0.0f))) {
                        hardware.jogJoint(joint, -1, static_cast<int>(activeRobot().hardwareJogSteps));
                    }
                    ImGui::TableSetColumnIndex(3);
                    if (ImGui::Button("+##jogPlus", ImVec2(-1.0f, 0.0f))) {
                        hardware.jogJoint(joint, +1, static_cast<int>(activeRobot().hardwareJogSteps));
                    }
                    ImGui::EndDisabled();
                    // Pulse direction. This is the wiring polarity of the joint's direction pin, a
                    // different thing from the Reverse box in the Mastering tab: that one picks
                    // which way the joint sweeps to find its switch, this one decides which way the
                    // motor turns for a given commanded sign. Getting it wrong sends every jog, and
                    // every move, the wrong way on that joint.
                    const size_t jogIndex = static_cast<size_t>(joint);
                    ImGui::TableSetColumnIndex(4);
                    bool inverted = activeRobot().jogDirInvert[jogIndex];
                    if (status.valid && !activeRobot().jogDirInvertEdited[jogIndex]) {
                        inverted = status.reportedJogDirInvert[jogIndex];
                        activeRobot().jogDirInvert[jogIndex] = inverted;
                    }
                    ImGui::BeginDisabled(!connected || busy);
                    if (ImGui::Checkbox("##pulseInvert", &inverted)) {
                        activeRobot().jogDirInvert[jogIndex] = inverted;
                        activeRobot().jogDirInvertEdited[jogIndex] = true;
                        hardware.sendJogDirectionInvert(activeRobot().jogDirInvert);
                        activeRobot().jogDirInvertEdited[jogIndex] = false;
                    }
                    ImGui::EndDisabled();

                    ImGui::PopID();
                }
                ImGui::EndTable();
            }
            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Mastering")) {
            ImGui::AlignTextToFramePadding();
            ImGui::TextUnformatted("Backoff");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(110.0f);
            ImGui::InputInt("##masterBackoff", &activeRobot().masterBackoffSteps, 10, 100);
            activeRobot().masterBackoffSteps = std::max(10, std::min(5000, activeRobot().masterBackoffSteps));

            const HardwareIo::Status& masterStatus = hardware.status();
            if (ImGui::BeginTable("mastering", 9,
                                  ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                                      ImGuiTableFlags_SizingStretchProp)) {
                ImGui::TableSetupColumn("Joint");
                ImGui::TableSetupColumn("Mastered");
                ImGui::TableSetupColumn("Position deg");
                ImGui::TableSetupColumn("Current steps");
                ImGui::TableSetupColumn("Steps/deg");
                ImGui::TableSetupColumn("Offset deg");
                ImGui::TableSetupColumn("Reverse", ImGuiTableColumnFlags_WidthFixed, 60.0f);
                ImGui::TableSetupColumn("Master");
                ImGui::TableSetupColumn("Set 0");
                ImGui::TableHeadersRow();

                for (int joint = 0; joint < 6; ++joint) {
                    const size_t index = static_cast<size_t>(joint);
                    ImGui::TableNextRow();
                    ImGui::PushID(joint);

                    ImGui::TableSetColumnIndex(0);
                    ImGui::Text("J%d", joint + 1);

                    ImGui::TableSetColumnIndex(1);
                    const bool mastered = masterStatus.jointMastered[index];
                    ImGui::TextUnformatted(masterStatus.valid ? (mastered ? "Yes" : "No") : "-");

                    // Seven decimals: the firmware reports position to a
                    // ten-millionth of a degree and mastering is judged against it. An
                    // unmastered joint has no meaningful angle, so it shows a dash.
                    ImGui::TableSetColumnIndex(2);
                    ImGui::TextUnformatted(masterStatus.valid && mastered
                        ? strutil::formatFixed(masterStatus.jointsDeg[index], 7).c_str()
                        : "-");

                    ImGui::TableSetColumnIndex(3);
                    ImGui::TextUnformatted(std::to_string(masterStatus.currentSteps[index]).c_str());

                    // Adopt the robot's own calibration: what the
                    // firmware is actually using beats the compiled-in defaults. Skipped while
                    // the field has keyboard focus, otherwise a status frame would overwrite the
                    // number mid-edit; the flag is from the previous frame, since a widget's
                    // active state is only known after it is drawn.
                    ImGui::TableSetColumnIndex(4);
                    if (!activeRobot().masterStepsPerDegreeEditing[index] &&
                        masterStatus.reportedStepsPerDegree[index] > 0.0) {
                        activeRobot().masterStepsPerDegree[index] = masterStatus.reportedStepsPerDegree[index];
                    }
                    ImGui::SetNextItemWidth(-1.0f);
                    ImGui::InputDouble("##steps", &activeRobot().masterStepsPerDegree[index], 0.0, 0.0, "%.7f");
                    activeRobot().masterStepsPerDegreeEditing[index] = ImGui::IsItemActive();

                    ImGui::TableSetColumnIndex(5);
                    if (!activeRobot().masterOffsetEditing[index] && masterStatus.valid) {
                        activeRobot().masterOffsetDeg[index] = masterStatus.reportedOffsetDeg[index];
                    }
                    ImGui::SetNextItemWidth(-1.0f);
                    ImGui::InputDouble("##offset", &activeRobot().masterOffsetDeg[index], 0.0, 0.0, "%.7f");
                    activeRobot().masterOffsetEditing[index] = ImGui::IsItemActive();

                    // Adopt the robot's own direction unless this row is being changed, the same
                    // way the two editors above do, so the box shows what the firmware will use.
                    ImGui::TableSetColumnIndex(6);
                    bool reversed = activeRobot().masterDirection[index] < 0;
                    if (masterStatus.valid && !activeRobot().masterDirectionEdited[index]) {
                        reversed = masterStatus.reportedMasterDirection[index] < 0;
                        activeRobot().masterDirection[index] = reversed ? -1 : 1;
                    }
                    if (ImGui::Checkbox("##reverse", &reversed)) {
                        activeRobot().masterDirection[index] = reversed ? -1 : 1;
                        // Sticky, so an arriving status frame does not undo the tick before the
                        // firmware has been told about it.
                        activeRobot().masterDirectionEdited[index] = true;
                    }

                    ImGui::TableSetColumnIndex(7);
                    if (ImGui::Button(strutil::format("Master J%1").arg(joint + 1).str().c_str())) {
                        if (!hardware.isConnected()) {
                            activeRobot().hardwareStatus = "Connect to the robot first.";
                        } else if (!masterStatus.jogArmed) {
                            activeRobot().hardwareStatus = "Arm jog before mastering.";
                        } else {
                            hardware.masterJoint(joint + 1, activeRobot().masterStepsPerDegree[index],
                                                 activeRobot().masterOffsetDeg[index], activeRobot().masterBackoffSteps,
                                                 activeRobot().masterDirection[index]);
                            // The firmware now holds this direction, so the display can go back to
                            // following the robot.
                            activeRobot().masterDirectionEdited[index] = false;
                            activeRobot().hardwareStatus = "Mastering J" + std::to_string(joint + 1) + "...";
                        }
                    }

                    ImGui::TableSetColumnIndex(8);
                    // Suffixed for the same reason as the jog buttons: this label matches the name
                    // of column 8, and TableHeadersRow seeds each header with its column index. Six
                    // joints means PushID(7) never happens and it cannot collide today, but the
                    // collision is one added row away and invisible until something stops working.
                    if (ImGui::Button("Set 0##setZero")) {
                        if (!hardware.isConnected()) {
                            activeRobot().hardwareStatus = "Connect to the robot first.";
                        } else if (!masterStatus.jogArmed) {
                            activeRobot().hardwareStatus = "Arm jog before confirming zero.";
                        } else {
                            hardware.setJointZero(joint + 1, activeRobot().masterStepsPerDegree[index],
                                                  activeRobot().masterOffsetDeg[index]);
                            activeRobot().hardwareStatus = "Zero set on J" + std::to_string(joint + 1) + ".";
                        }
                    }
                    ImGui::PopID();
                }
                ImGui::EndTable();
            }

            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Motion Planner")) {
            // Separate physical capability limits from controller policy.
            if (ImGui::BeginTabBar("motionSettingsTabs")) {
                if (ImGui::BeginTabItem("Dynamics")) {
                    nativeSpinBox("##motionJointAccel", &activeRobot().motionJointAccelDegS2, 10.0, "%.3f deg/s^2",
                                  1.0, 1.0e9, 150.0f, "Joint acceleration");
                    nativeSpinBox("##motionJointJerk", &activeRobot().motionJointJerkDegS3, 100.0, "%.3f deg/s^3",
                                  1.0, 1.0e12, 150.0f, "Joint jerk");
                    nativeSpinBox("##motionLinearAccel", &activeRobot().motionLinearAccelMmS2, 100.0, "%.3f mm/s^2",
                                  1.0, 1.0e12, 150.0f, "Linear acceleration");
                    nativeSpinBox("##motionLinearJerk", &activeRobot().motionLinearJerkMmS3, 1000.0, "%.3f mm/s^3",
                                  1.0, 1.0e15, 150.0f, "Linear jerk");
                    ImGui::TextDisabled("Per-joint acceleration and jerk scaling is in the Dynamic Limits tab.");
                    ImGui::EndTabItem();
                }
                if (ImGui::BeginTabItem("Tool")) {
                    nativeSpinBox("##motionToolSpeed", &activeRobot().motionToolAngularSpeedDegS, 10.0, "%.3f deg/s",
                                  0.1, 1.0e9, 150.0f, "Tool angular speed");
                    nativeSpinBox("##motionToolAccel", &activeRobot().motionToolAngularAccelDegS2, 100.0, "%.3f deg/s^2",
                                  1.0, 1.0e12, 150.0f, "Tool angular acceleration");
                    nativeSpinBox("##motionToolJerk", &activeRobot().motionToolAngularJerkDegS3, 1000.0, "%.3f deg/s^3",
                                  1.0, 1.0e15, 150.0f, "Tool angular jerk");
                    ImGui::EndTabItem();
                }
                if (ImGui::BeginTabItem("Controller")) {
                    nativeSpinBox("##motionControlPeriod", &activeRobot().motionControlPeriodMs, 0.5, "%.3f ms",
                                  0.1, 100.0, 150.0f, "Control period");
                    nativeSpinBox("##motionSingularity", &activeRobot().motionSingularityDeg, 0.5, "%.3f deg",
                                  0.01, 90.0, 150.0f, "Singularity threshold");
                    ImGui::EndTabItem();
                }
                if (ImGui::BeginTabItem("Dynamic Limits")) {
                    // Named for what they are. "Joint limits" reads as the travel range - the q_min
                    // and q_max saying how far a joint turns - where these are the speed,
                    // acceleration and jerk it may use getting there. Two different things, and
                    // confusing them shows up as an arm moving faster than anyone intended.
                    ImGui::TextDisabled("Ceilings from the package - not the +/- travel range. "
                                        "Scale multiplies all three for that joint.");
                    ImGui::SameLine();
                    if (ImGui::SmallButton("All to 1")) activeRobot().motionDynamicLimitScale.fill(1.0);
                    if (ImGui::BeginTable("dynamicLimits", 5,
                                          ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                                              ImGuiTableFlags_SizingStretchProp)) {
                        ImGui::TableSetupColumn("Joint");
                        ImGui::TableSetupColumn("Scale", ImGuiTableColumnFlags_WidthFixed, 90.0f);
                        ImGui::TableSetupColumn("Max velocity deg/s");
                        ImGui::TableSetupColumn("Max acceleration deg/s^2");
                        ImGui::TableSetupColumn("Max jerk deg/s^3");
                        ImGui::TableHeadersRow();

                        const auto cell = [](double baseValue, double factor) {
                            if (!(baseValue > 0.0)) return std::string("-");
                            if (factor == 1.0) return strutil::formatFixed(baseValue, 2);
                            return strutil::formatFixed(baseValue, 2) + " x " +
                                   strutil::formatFixed(factor, 3) + " = " +
                                   strutil::formatFixed(baseValue * factor, 2);
                        };
                        for (int joint = 0; joint < 6; ++joint) {
                            const size_t index = static_cast<size_t>(joint);
                            const bool valid = activeRobot().packageJointLimitsValid;
                            const double scale = activeRobot().motionDynamicLimitScale[index];
                            ImGui::TableNextRow();
                            ImGui::PushID(joint);
                            ImGui::TableSetColumnIndex(0);
                            ImGui::AlignTextToFramePadding();
                            ImGui::Text("J%d", joint + 1);
                            ImGui::TableSetColumnIndex(1);
                            ImGui::SetNextItemWidth(-1.0f);
                            // Suffixed for the same reason as the jog buttons: the row pushes the
                            // joint index and TableHeadersRow pushes the column index, so a plain
                            // label would collide with the header of the column at the same number.
                            ImGui::InputDouble("##limitScale", &activeRobot().motionDynamicLimitScale[index],
                                               0.0, 0.0, "%.3f");
                            if (!(activeRobot().motionDynamicLimitScale[index] > 0.0)) {
                                // Zero or negative would silently mean "no scaling" to the
                                // simulator and would be nonsense to the firmware.
                                activeRobot().motionDynamicLimitScale[index] = 1.0;
                            }
                            // Pushed when the edit finishes, not on each keystroke: the limits
                            // travel inside load_robot_model, which is over a kilobyte.
                            if (ImGui::IsItemDeactivatedAfterEdit() && hardware.isConnected()) {
                                hardware.setRobotModelCommand(
                                    robotModelCommandForPackage(activeRobot().poseController, currentRobotModelFor(activeRobot())));
                                activeRobot().motionSettingsStatus = "Joint limit scales sent to the robot.";
                            }
                            ImGui::TableSetColumnIndex(2);
                            ImGui::TextUnformatted(valid
                                ? cell(activeRobot().packageJointVelocityMaxRadS[index] * kRadToDeg, scale).c_str()
                                : "-");
                            ImGui::TableSetColumnIndex(3);
                            ImGui::TextUnformatted(valid
                                ? cell(activeRobot().packageJointAccelMaxRadS2[index] * kRadToDeg, scale).c_str()
                                : "-");
                            ImGui::TableSetColumnIndex(4);
                            ImGui::TextUnformatted(valid
                                ? cell(activeRobot().packageJointJerkMaxRadS3[index] * kRadToDeg, scale).c_str()
                                : "-");
                            ImGui::PopID();
                        }
                        ImGui::EndTable();
                    }
                    ImGui::EndTabItem();
                }
                ImGui::EndTabBar();
            }

            ImGui::Separator();
            if (ImGui::Button("Load from package")) {
                if (activeRobot().packageMotionSettings.empty()) {
                    activeRobot().motionSettingsStatus = "No package motion settings are loaded.";
                } else {
                    activeRobot().motionDynamicLimitScale.fill(1.0);
                    applyMotionSettingsToEditors(activeRobot(), activeRobot().packageMotionSettings, "Package settings loaded.");
                }
            }
            // No Send or Read here: the panel row above does both, and more. Its Write to robot RAM
            // sends these settings and the mastering calibration together, and its Read pulls the
            // settings and the status in one go, so the pair that lived here could only ever move
            // half of what the other moves - two buttons that look like a choice and are not.
            ImGui::SameLine();
            ImGui::TextUnformatted(activeRobot().motionSettingsStatus.c_str());

            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Welding")) {
            // Programs and the wire protocol reference eight schedules by index.
            ImGui::TextDisabled("Weave schedules. A program selects one with WeaveOn <index>.");

            static const char* kShapeNames[] = {"none", "sine", "zigzag", "trapezoid",
                                                "circular", "figure8", "lshape"};
            // Labelled with the vendors each convention comes from, because the choice is not a
            // preference: it decides whether the pattern stretches or the bead geometry holds when
            // the planner slows the seam down.
            static const char* kRateNames[] = {"(choose one)",
                                               "Frequency Hz - FANUC / Yaskawa",
                                               "Wavelength mm - ABB / KUKA"};

            // Split the twelve fields into aligned pattern and dimension tables.
            const float halfWidth = (ImGui::GetContentRegionAvail().x - 8.0f) * 0.5f;
            const float tableHeight = ImGui::GetFrameHeightWithSpacing() *
                                      (RobotMotionCore::kMaxWeaveSchedules + 1.6f);
            constexpr ImGuiTableFlags kHalfFlags =
                ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp;

            if (ImGui::BeginChild("weavePatternHalf", ImVec2(halfWidth, tableHeight), false)) {
                if (ImGui::BeginTable("weaveSchedulesPattern", 6, kHalfFlags)) {
                    ImGui::TableSetupColumn("#", ImGuiTableColumnFlags_WidthFixed, 24.0f);
                    ImGui::TableSetupColumn("On", ImGuiTableColumnFlags_WidthFixed, 28.0f);
                    ImGui::TableSetupColumn("Name", ImGuiTableColumnFlags_WidthStretch, 1.0f);
                    ImGui::TableSetupColumn("Shape", ImGuiTableColumnFlags_WidthStretch, 1.1f);
                    ImGui::TableSetupColumn("Rate mode", ImGuiTableColumnFlags_WidthStretch, 2.0f);
                    ImGui::TableSetupColumn("Rate", ImGuiTableColumnFlags_WidthStretch, 0.9f);
                    ImGui::TableHeadersRow();

                    for (int index = 0; index < static_cast<int>(RobotMotionCore::kMaxWeaveSchedules); ++index) {
                        RobotMotionCore::WeaveParams& weave = activeRobot().weaveSchedules.schedules[index];
                        ImGui::TableNextRow();
                        ImGui::PushID(index);

                        ImGui::TableSetColumnIndex(0);
                        ImGui::AlignTextToFramePadding();
                        ImGui::Text("%d", index);

                        ImGui::TableSetColumnIndex(1);
                        bool valid = activeRobot().weaveSchedules.valid[index] != 0;
                        if (ImGui::Checkbox("##valid", &valid)) {
                            activeRobot().weaveSchedules.valid[index] = valid ? 1 : 0;
                        }

                        ImGui::TableSetColumnIndex(2);
                        ImGui::SetNextItemWidth(-1.0f);
                        char name[32] = {};
                        const std::string& current = activeRobot().weaveScheduleNames[static_cast<size_t>(index)];
                        std::snprintf(name, sizeof(name), "%s", current.c_str());
                        if (ImGui::InputText("##name", name, sizeof(name))) {
                            activeRobot().weaveScheduleNames[static_cast<size_t>(index)] = name;
                        }

                        ImGui::TableSetColumnIndex(3);
                        ImGui::SetNextItemWidth(-1.0f);
                        int shape = static_cast<int>(weave.shape);
                        if (ImGui::Combo("##shape", &shape, kShapeNames, IM_ARRAYSIZE(kShapeNames))) {
                            weave.shape = static_cast<RobotMotionCore::WeaveShape>(shape);
                        }

                        ImGui::TableSetColumnIndex(4);
                        ImGui::SetNextItemWidth(-1.0f);
                        int rateMode = static_cast<int>(weave.rateMode);
                        if (ImGui::Combo("##rate", &rateMode, kRateNames, IM_ARRAYSIZE(kRateNames))) {
                            weave.rateMode = static_cast<RobotMotionCore::WeaveRateMode>(rateMode);
                        }

                        // The rate cell follows whichever mode was chosen, and stays blank until one
                        // is. There is deliberately no default: a schedule that silently picked Hz
                        // would weld at a pattern nobody asked for, so the planner refuses it by
                        // index instead and this is where that shows.
                        ImGui::TableSetColumnIndex(5);
                        ImGui::SetNextItemWidth(-1.0f);
                        if (weave.rateMode == RobotMotionCore::WeaveRateMode::Frequency) {
                            ImGui::InputDouble("##rateValue", &weave.frequencyHz, 0.0, 0.0, "%.2f Hz");
                        } else if (weave.rateMode == RobotMotionCore::WeaveRateMode::Wavelength) {
                            ImGui::InputDouble("##rateValue", &weave.wavelengthMm, 0.0, 0.0, "%.2f mm");
                        } else {
                            ImGui::AlignTextToFramePadding();
                            ImGui::TextDisabled("set mode");
                        }

                        ImGui::PopID();
                    }
                    ImGui::EndTable();
                }
            }
            ImGui::EndChild();

            ImGui::SameLine(0.0f, 8.0f);

            if (ImGui::BeginChild("weaveGeometryHalf", ImVec2(halfWidth, tableHeight), false)) {
                if (ImGui::BeginTable("weaveSchedulesGeometry", 6, kHalfFlags)) {
                    ImGui::TableSetupColumn("#", ImGuiTableColumnFlags_WidthFixed, 24.0f);
                    ImGui::TableSetupColumn("Amp L/R mm", ImGuiTableColumnFlags_WidthStretch, 1.8f);
                    ImGui::TableSetupColumn("Elev mm", ImGuiTableColumnFlags_WidthStretch, 1.0f);
                    ImGui::TableSetupColumn("Angle deg", ImGuiTableColumnFlags_WidthStretch, 1.0f);
                    ImGui::TableSetupColumn("Bias mm", ImGuiTableColumnFlags_WidthStretch, 1.0f);
                    ImGui::TableSetupColumn("Dwell L/C/R", ImGuiTableColumnFlags_WidthStretch, 2.4f);
                    ImGui::TableHeadersRow();

                    for (int index = 0; index < static_cast<int>(RobotMotionCore::kMaxWeaveSchedules); ++index) {
                        RobotMotionCore::WeaveParams& weave = activeRobot().weaveSchedules.schedules[index];
                        ImGui::TableNextRow();
                        ImGui::PushID(index);

                        ImGui::TableSetColumnIndex(0);
                        ImGui::AlignTextToFramePadding();
                        ImGui::Text("%d", index);

                        // Paired in one cell, because the two sides of a weave are read together and
                        // are equal in every schedule that is not deliberately unsymmetrical.
                        ImGui::TableSetColumnIndex(1);
                        const float ampWidth = (ImGui::GetContentRegionAvail().x - 4.0f) * 0.5f;
                        ImGui::SetNextItemWidth(ampWidth);
                        ImGui::InputDouble("##ampL", &weave.amplitudeLeftMm, 0.0, 0.0, "%.2f");
                        ImGui::SameLine(0.0f, 4.0f);
                        ImGui::SetNextItemWidth(ampWidth);
                        ImGui::InputDouble("##ampR", &weave.amplitudeRightMm, 0.0, 0.0, "%.2f");

                        ImGui::TableSetColumnIndex(2);
                        ImGui::SetNextItemWidth(-1.0f);
                        ImGui::InputDouble("##elev", &weave.elevationMm, 0.0, 0.0, "%.2f");
                        ImGui::TableSetColumnIndex(3);
                        ImGui::SetNextItemWidth(-1.0f);
                        ImGui::InputDouble("##angle", &weave.planeAngleDeg, 0.0, 0.0, "%.1f");
                        ImGui::TableSetColumnIndex(4);
                        ImGui::SetNextItemWidth(-1.0f);
                        ImGui::InputDouble("##bias", &weave.biasMm, 0.0, 0.0, "%.2f");

                        ImGui::TableSetColumnIndex(5);
                        const float dwellWidth = (ImGui::GetContentRegionAvail().x - 8.0f) / 3.0f;
                        ImGui::SetNextItemWidth(dwellWidth);
                        ImGui::InputDouble("##dwellL", &weave.dwellLeft, 0.0, 0.0, "%.2f");
                        ImGui::SameLine(0.0f, 4.0f);
                        ImGui::SetNextItemWidth(dwellWidth);
                        ImGui::InputDouble("##dwellC", &weave.dwellCenter, 0.0, 0.0, "%.2f");
                        ImGui::SameLine(0.0f, 4.0f);
                        ImGui::SetNextItemWidth(dwellWidth);
                        ImGui::InputDouble("##dwellR", &weave.dwellRight, 0.0, 0.0, "%.2f");

                        ImGui::PopID();
                    }
                    ImGui::EndTable();
                }
            }
            ImGui::EndChild();

            // Dwell units follow the rate mode, so saying so once here beats three ambiguous
            // column headers.
            ImGui::TextDisabled("Dwell is in seconds for a frequency schedule and millimetres for "
                                "a wavelength one, matching its rate.");

            ImGui::Separator();
            if (ImGui::Button("Send schedules to robot")) {
                if (!hardware.isConnected()) {
                    activeRobot().weaveScheduleStatus = "Not connected.";
                } else {
                    hardware.sendWeaveSchedules(activeRobot().weaveSchedules);
                    activeRobot().weaveScheduleStatus = "Schedules sent to the robot.";
                }
            }
            ImGui::SameLine();
            if (ImGui::Button("Read schedules from robot")) {
                if (!hardware.isConnected()) {
                    activeRobot().weaveScheduleStatus = "Not connected.";
                } else {
                    hardware.requestWeaveSchedules();
                    activeRobot().weaveScheduleStatus = "Reading schedules from the robot...";
                }
            }
            ImGui::SameLine();
            if (ImGui::Button("Clear row states")) {
                for (int index = 0; index < static_cast<int>(RobotMotionCore::kMaxWeaveSchedules); ++index) {
                    activeRobot().weaveSchedules.schedules[index] = RobotMotionCore::defaultWeaveParams();
                    activeRobot().weaveSchedules.valid[index] = 0;
                    activeRobot().weaveScheduleNames[static_cast<size_t>(index)].clear();
                }
                activeRobot().weaveScheduleStatus = "All schedules cleared.";
            }
            ImGui::SameLine();
            ImGui::TextUnformatted(activeRobot().weaveScheduleStatus.c_str());

            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Run")) {
            // The speeds come from the program State fields under the instruction list, not from a
            // pair of their own. Run program always used those - it builds its commands from
            // currentProgramState - so a second pair here meant the same button row moved the arm
            // at one speed and the program at another.
            const RobotProgramSimulator::ProgramState runState = currentProgramState();
            const double runJointSpeedDegPerSec = runState.jointSpeedRadPerSec * kRadToDeg;
            ImGui::Text("Speed %.2f deg/s, linear %.2f mm/s (from the program State fields)",
                        runJointSpeedDegPerSec, runState.linearSpeedMmPerSec);
            ImGui::Checkbox("Track during move", &activeRobot().hardwareTrackDuringMove);
            ImGui::Spacing();
            // Everything here drives a real arm from a real pose, so all of it needs the same
            // preconditions: connected, armed, not mid-mastering, and all six joints mastered,
            // since an unmastered joint has no step-to-degree reference.
            const bool programRunning = hardware.programActive();
            const bool canMove = hardware.isConnected() && status.jogArmed && !status.masteringActive &&
                                 hardware.allJointsMastered() && !programRunning;
            // A disabled button that says nothing is indistinguishable from a broken one. Each of
            // these is also enforced by the firmware, which would answer not_armed, not_mastered,
            // busy or model_not_loaded; saying it here means not having to read the log to find out.
            const char* blockedBecause =
                !hardware.isConnected()        ? "Not connected."
                : !hardware.hasRobotModel()    ? "No robot package loaded."
                : programRunning               ? "A program is running."
                : status.masteringActive       ? "Mastering is in progress."
                : !status.jogArmed             ? "Arm jog first (the button is above the tabs)."
                : !hardware.allJointsMastered() ? "All six joints must be mastered: master them, or "
                                                  "load a saved calibration and restore the reference."
                                                : nullptr;
            if (blockedBecause) ImGui::TextWrapped("Cannot move: %s", blockedBecause);

            // Targets come from the simulated pose, so the panel drives the real arm to
            // whatever the 3D view is showing.
            ImGui::BeginDisabled(!canMove);
            if (ImGui::Button("MoveJ to sim pose")) {
                hardware.moveJointsTo(activeRobot().poseController.joints(), runJointSpeedDegPerSec);
            }
            ImGui::SameLine();
            if (ImGui::Button("MoveL to sim pose")) {
                // The tool pose the firmware plans the straight line along, taken from the same
                // kinematics it was handed in load_robot_model.
                hardware.moveLinearTo(activeRobot().poseController.joints(),
                                      RobotMotionCore::toolPoseForJoints(
                                          currentRobotModelFor(activeRobot()),
                                          activeRobot().poseController.joints().data()),
                                      runState.linearSpeedMmPerSec);
            }
            ImGui::EndDisabled();

            ImGui::SameLine(0.0f, 16.0f);
            ImGui::BeginDisabled(!canMove || !activeRobot().program().root ||
                                 activeRobot().program().root->children.empty());
            if (ImGui::Button("Run program")) {
                std::vector<Json> commands;
                std::string errorMessage;
                // The program runs from where the robot is now, not from the simulated pose, so
                // the first move is planned against reality.
                std::array<double, 6> startDeg{};
                for (size_t i = 0; i < 6; ++i) startDeg[i] = status.jointsDeg[i];
                std::map<int, std::string> triggerMessages;
                if (!buildHardwareProgramCommands(activeRobot(), startDeg, currentProgramState(), &commands,
                                                  &triggerMessages, &errorMessage)) {
                    activeRobot().buildStatus = errorMessage;
                } else {
                    // Handed over before the program starts, so the first trigger to fire already
                    // has its text waiting.
                    hardware.setTriggerMessages(std::move(triggerMessages));
                    if (!hardware.startProgram(std::move(commands), &errorMessage)) {
                        activeRobot().buildStatus = errorMessage;
                    }
                }
            }
            ImGui::EndDisabled();
            if (programRunning) {
                ImGui::SameLine();
                if (ImGui::Button("Cancel run")) hardware.cancelProgram("Program cancelled.", true);
                ImGui::Text("Running %d/%d (row %d)", hardware.programCompletedCount() + 1,
                            hardware.programTotalCount(), hardware.programExecutingRow() + 1);
            }
            ImGui::Spacing();
            ImGui::TextUnformatted("Simulated pose:");
            const std::array<double, 6> simJoints = activeRobot().poseController.joints();
            for (int joint = 0; joint < 6; ++joint) {
                ImGui::Text("J%d  sim %.2f   robot %.2f", joint + 1,
                            simJoints[static_cast<size_t>(joint)] * kRadToDeg,
                            status.jointsDeg[static_cast<size_t>(joint)]);
            }
            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Log")) {
            if (ImGui::Button("Clear")) hardware.clearLog();
            ImGui::Separator();
            if (ImGui::BeginChild("logScroll", ImVec2(0, 0), ImGuiChildFlags_None)) {
                for (const std::string& entry : hardware.log()) {
                    ImGui::TextUnformatted(entry.c_str());
                }
                // Follow the tail while the newest line is in view.
                if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 1.0f) ImGui::SetScrollHereY(1.0f);
            }
            ImGui::EndChild();
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }
}

