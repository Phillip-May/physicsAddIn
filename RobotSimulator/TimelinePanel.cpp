#include "TimelinePanel.h"

#include "AppState.h"
#include "TrajectoryAndLiveRun.h"

#include "imgui.h"
void drawTimelinePanel() {
    if (activeRobot().buildRunning) {
        ImGui::Text("Planning... %zu samples", activeRobot().buildSentSamples);
        return;
    }
    if (activeRobot().timelineSamples.empty()) {
        ImGui::TextDisabled("Simulate a program to populate the timeline.");
        return;
    }

    float seconds = static_cast<float>(activeRobot().elapsedSeconds);
    ImGui::SetNextItemWidth(-140.0f);
    const ImVec2 sliderOrigin = ImGui::GetCursorScreenPos();
    const float sliderWidth = ImGui::CalcItemWidth();
    if (ImGui::SliderFloat("##timeline", &seconds, 0.0f, static_cast<float>(activeRobot().durationSeconds), "%.3f s")) {
        activeRobot().elapsedSeconds = static_cast<double>(seconds);
        activeRobot().playing = false;
    }

    // Markers drawn over the groove, as the QSlider subclass did with subControlRect.
    if (activeRobot().durationSeconds > 0.0) {
        ImDrawList* drawList = ImGui::GetWindowDrawList();
        const float height = ImGui::GetItemRectSize().y;
        for (const TimelineMarker& marker : activeRobot().timelineMarkers) {
            if (!timelineMarkerVisible(marker.type)) continue;
            const float t = static_cast<float>(marker.seconds / activeRobot().durationSeconds);
            const float x = sliderOrigin.x + t * sliderWidth;
            ImU32 color = IM_COL32(220, 45, 45, 255);
            if (marker.type == TimelineMarkerType::PlannerCap) color = IM_COL32(90, 160, 230, 255);
            else if (marker.type == TimelineMarkerType::JointFlip) color = IM_COL32(245, 166, 35, 255);
            drawList->AddLine(ImVec2(x, sliderOrigin.y), ImVec2(x, sliderOrigin.y + height), color, 2.0f);
        }
    }

    ImGui::SameLine();
    ImGui::Text("%.2f / %.2f s", activeRobot().elapsedSeconds, activeRobot().durationSeconds);
}

