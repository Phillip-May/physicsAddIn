#include "SimulationInfoPanel.h"

#include "AppState.h"
#include "StringUtil.h"
#include "TrajectoryAndLiveRun.h"

#include "imgui.h"
#include "implot.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>
// Joint and TCP speeds of the real robot, differentiated from the recorded positions.
struct HardwareTraceSpeeds {
    std::vector<double> times;
    std::array<std::vector<double>, 6> jointDegPerSec;
    std::vector<double> tcpMmPerSec;
};

HardwareTraceSpeeds hardwareTraceSpeeds() {
    HardwareTraceSpeeds result;
    const std::vector<HardwareTraceSample>& trace = activeRobot().hardwareTrace;
    constexpr double kWindowSeconds = 0.12;
    constexpr double kMinIntervalSeconds = 0.04;
    // A revolute joint that crosses the wrap point reports a jump of nearly a full turn; taking the
    // shorter way round keeps that from reading as an enormous speed.
    const auto wrapJointDelta = [](double value) {
        constexpr double kPi = 3.14159265358979323846;
        while (value > kPi) value -= 2.0 * kPi;
        while (value < -kPi) value += 2.0 * kPi;
        return value;
    };

    size_t previousIndex = 0;
    for (size_t i = 1; i < trace.size(); ++i) {
        while (previousIndex + 1 < i &&
               trace[i].seconds - trace[previousIndex + 1].seconds >= kWindowSeconds) {
            ++previousIndex;
        }
        const HardwareTraceSample& previous = trace[previousIndex];
        const HardwareTraceSample& current = trace[i];
        const double dt = current.seconds - previous.seconds;
        if (dt < kMinIntervalSeconds) continue;

        result.times.push_back(current.seconds);
        for (size_t joint = 0; joint < 6; ++joint) {
            const double delta = wrapJointDelta(current.joints[joint] - previous.joints[joint]);
            result.jointDegPerSec[joint].push_back((delta / dt) * kRadToDeg);
        }
        const double dx = current.tcpPose.values[3] - previous.tcpPose.values[3];
        const double dy = current.tcpPose.values[7] - previous.tcpPose.values[7];
        const double dz = current.tcpPose.values[11] - previous.tcpPose.values[11];
        result.tcpMmPerSec.push_back(std::sqrt(dx * dx + dy * dy + dz * dz) / dt);
    }
    return result;
}

// A second palette for the real robot, warm where the simulated series are cool, so a robot trace
// is not mistaken for the simulation it is being compared against. Each series is also named with
// a "robot" suffix in the legend, since colour alone is a poor distinction for anyone who cannot
// separate these hues, and ImPlot has no dashed line style to fall back on.
const ImVec4 kRobotJointColors[6] = {
    ImVec4(0.96f, 0.53f, 0.13f, 1.0f), ImVec4(0.98f, 0.75f, 0.18f, 1.0f),
    ImVec4(0.91f, 0.30f, 0.24f, 1.0f), ImVec4(0.85f, 0.44f, 0.84f, 1.0f),
    ImVec4(0.99f, 0.62f, 0.55f, 1.0f), ImVec4(0.60f, 0.80f, 0.20f, 1.0f),
};

void drawSimulationInfoPanel() {
    if (ImGui::BeginTabBar("simInfo")) {
        if (ImGui::BeginTabItem("Joint speed")) {
            const HardwareTraceSpeeds robot = hardwareTraceSpeeds();
            if (activeRobot().timelineSamples.size() < 2 && robot.times.empty()) {
                ImGui::TextDisabled("No simulation trace.");
            } else if (ImPlot::BeginPlot("##jointSpeed", ImVec2(-1, -1))) {
                ImPlot::SetupAxes("s", "deg/s", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
                std::vector<double> times;
                std::array<std::vector<double>, 6> speeds;
                times.reserve(activeRobot().timelineSamples.size());
                for (size_t i = 1; i < activeRobot().timelineSamples.size(); ++i) {
                    const TimelineSample& previous = activeRobot().timelineSamples[i - 1];
                    const TimelineSample& current = activeRobot().timelineSamples[i];
                    const double dt = current.seconds - previous.seconds;
                    if (dt <= 0.0) continue;
                    times.push_back(current.seconds);
                    for (int joint = 0; joint < 6; ++joint) {
                        const size_t index = static_cast<size_t>(joint);
                        const double delta = current.joints[index] - previous.joints[index];
                        speeds[index].push_back((delta / dt) * kRadToDeg);
                    }
                }
                for (int joint = 0; joint < 6; ++joint) {
                    const std::string name = "J" + std::to_string(joint + 1);
                    ImPlot::PlotLine(name.c_str(), times.data(),
                                     speeds[static_cast<size_t>(joint)].data(),
                                     static_cast<int>(times.size()));
                }
                for (size_t joint = 0; joint < 6 && !robot.times.empty(); ++joint) {
                    const std::string name = "J" + std::to_string(joint + 1) + " robot";
                    ImPlotSpec spec;
                    spec.LineColor = kRobotJointColors[joint];
                    ImPlot::PlotLine(name.c_str(), robot.times.data(),
                                     robot.jointDegPerSec[joint].data(),
                                     static_cast<int>(robot.times.size()), spec);
                }
                ImPlot::EndPlot();
            }
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("TCP speed")) {
            const HardwareTraceSpeeds robot = hardwareTraceSpeeds();
            if (activeRobot().timelineSamples.empty() && robot.times.empty()) {
                ImGui::TextDisabled("No simulation trace.");
            } else if (ImPlot::BeginPlot("##tcpSpeed", ImVec2(-1, -1))) {
                ImPlot::SetupAxes("s", "mm/s", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
                std::vector<double> times;
                std::vector<double> actual;
                std::vector<double> desired;
                for (const TimelineSample& sample : activeRobot().timelineSamples) {
                    if (!std::isfinite(sample.tcpSpeedMmPerSec)) continue;
                    times.push_back(sample.seconds);
                    actual.push_back(sample.tcpSpeedMmPerSec);
                    desired.push_back(std::isfinite(sample.desiredTcpSpeedMmPerSec)
                                          ? sample.desiredTcpSpeedMmPerSec
                                          : 0.0);
                }
                ImPlot::PlotLine("Actual", times.data(), actual.data(), static_cast<int>(times.size()));
                ImPlot::PlotLine("Desired", times.data(), desired.data(), static_cast<int>(times.size()));
                if (!robot.times.empty()) {
                    ImPlotSpec spec;
                    spec.LineColor = kRobotJointColors[0];
                    spec.LineWeight = 2.0f;
                    ImPlot::PlotLine("Robot", robot.times.data(), robot.tcpMmPerSec.data(),
                                     static_cast<int>(robot.times.size()), spec);
                }
                ImPlot::EndPlot();
            }
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Events")) {
            ImGui::Checkbox("Planner caps", &g_scene.activeViewState().showPlannerCaps);
            ImGui::SameLine();
            ImGui::Checkbox("Singularities", &g_scene.activeViewState().showSingularities);
            ImGui::SameLine();
            ImGui::Checkbox("Joint flips", &g_scene.activeViewState().showJointFlips);
            ImGui::Separator();
            bool any = false;
            for (const TimelineMarker& marker : activeRobot().timelineMarkers) {
                if (!timelineMarkerVisible(marker.type)) continue;
                any = true;
                const std::string text = strutil::format("%1 s  %2").arg(marker.seconds, 0, 'f', 2).arg(marker.message);
                if (ImGui::Selectable(text.c_str())) {
                    activeRobot().elapsedSeconds = marker.seconds;
                    activeRobot().playing = false;
                }
            }
            if (!any) ImGui::TextDisabled("No visible planner markers.");
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Limits")) {
            // Ranked by how many segments each limit bound. Peak/limit ratios cannot answer
            // this: a limit that throttles the ramp keeps its own peak under its ceiling.
            std::vector<std::pair<std::string, uint32_t>> ranked;
            // std::map iterates in sorted key order, exactly as QMap did, which the ranking
            // below relies on for a stable order among equal counts.
            for (const auto& entry : activeRobot().limitReasonCounts) {
                ranked.emplace_back(entry.first, entry.second);
            }
            std::sort(ranked.begin(), ranked.end(), [](const auto& a, const auto& b) {
                if (a.second != b.second) return a.second > b.second;
                return a.first < b.first;
            });
            for (const auto& entry : ranked) {
                ImGui::Text("%s  -  %u segment%s", entry.first.c_str(), entry.second,
                            entry.second == 1 ? "" : "s");
            }
            if (ranked.empty()) ImGui::TextDisabled("Simulate a program to see which limits bound it.");
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }
}

