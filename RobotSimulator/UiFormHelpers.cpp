#include "UiFormHelpers.h"

#include <array>
#include <cstring>
#include <string>

#include "CadNode.h"
#include "PlacedMechanismSchema.h"
#include "StringUtil.h"
// ArrowButtonEx, for the spin buttons.
#include "imgui_internal.h"

#if defined(_WIN32) && !defined(__EMSCRIPTEN__)
#include <windows.h>
#include <commdlg.h>
#endif




// Uniform button grid: every button in a row comes out equal width and the row fills the panel;
// ImGui buttons size to their text unless told otherwise.
bool formButton(const char* label, int columns, bool disabled) {
    const ImGuiStyle& style = ImGui::GetStyle();
    const float spacing = style.ItemSpacing.x * static_cast<float>(columns - 1);
    // Measured from the panel's full width, not the space left on the current line. Using the
    // remaining width made the second button in a row half of what the first had left over, so
    // a two-button row stopped short of the edge and a full-width button below it looked
    // roughly a button wider.
    const float panelWidth = ImGui::GetContentRegionMax().x - ImGui::GetCursorStartPos().x;
    const float width = (panelWidth - spacing) / static_cast<float>(columns);
    ImGui::BeginDisabled(disabled);
    const bool pressed = ImGui::Button(label, ImVec2(width, 0.0f));
    ImGui::EndDisabled();
    return pressed;
}

// A spin box with a value field and small stacked up/down arrows on its right edge.
// ImGui::InputDouble's built-in stepper renders wide "+" and "-" push buttons instead, so the
// step buttons are disabled and the arrows drawn manually.
bool nativeSpinBox(const char* label,
                   double* value,
                   double step,
                   const char* format,
                   double minValue,
                   double maxValue,
                   float fieldWidth,
                   const char* leadingLabel) {
    ImGui::PushID(label);
    if (leadingLabel != nullptr) {
        // Caption to the left of the field, as QFormLayout placed it. The column is measured
        // from wherever this row starts, not from the window edge: ImGui::SameLine(offset)
        // positions absolutely, so a spin box placed after SameLine would throw its label back
        // to the window's left margin and draw on top of whatever is already there.
        const float rowStartX = ImGui::GetCursorPosX();
        ImGui::AlignTextToFramePadding();
        ImGui::TextUnformatted(leadingLabel);
        ImGui::SameLine(0.0f, 0.0f);
        ImGui::SetCursorPosX(rowStartX + kFormLabelWidth);
    }
    const float rowHeight = ImGui::GetFrameHeight();
    const float arrowWidth = std::floor(rowHeight * 0.62f);
    bool changed = false;

    ImGui::BeginGroup();
    ImGui::SetNextItemWidth(fieldWidth);
    // Zero step suppresses the +/- buttons; the field itself still accepts typed input.
    if (ImGui::InputDouble("##value", value, 0.0, 0.0, format)) changed = true;

    ImGui::SameLine(0.0f, 1.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0.0f, 0.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 0.0f);
    ImGui::BeginGroup();
    const float halfHeight = std::floor((rowHeight - 1.0f) * 0.5f);
    if (ImGui::ArrowButtonEx("##up", ImGuiDir_Up, ImVec2(arrowWidth, halfHeight))) {
        *value += step;
        changed = true;
    }
    if (ImGui::ArrowButtonEx("##down", ImGuiDir_Down, ImVec2(arrowWidth, halfHeight))) {
        *value -= step;
        changed = true;
    }
    ImGui::EndGroup();
    ImGui::PopStyleVar(2);

    // No trailing caption: the label argument is identity only. Captions come from
    // leadingLabel, on the left, as QFormLayout placed them. Printing both is what produced
    // the duplicated X/Y/Z/W/P/R columns.
    ImGui::EndGroup();

    if (changed && minValue < maxValue) {
        *value = std::max(minValue, std::min(maxValue, *value));
    }
    ImGui::PopID();
    return changed;
}

// ImGui has no file dialog. On Windows the native common dialog is used, which is both the
// least code and the closest match to the native feel the rest of the styling aims at. On the
// web there is no filesystem to browse, so the buttons report that instead.
#if defined(_WIN32) && !defined(__EMSCRIPTEN__)
std::string runFileDialog(const char* title, bool saving, const wchar_t* filter, const wchar_t* defaultExt,
                          const std::string& suggestedName) {
    wchar_t buffer[MAX_PATH] = {0};
    if (!suggestedName.empty()) {
        std::wstring wide;
        wide.reserve(suggestedName.size());
        for (char c : suggestedName) {
            if (std::strchr("\\/:*?\"<>|", c) != nullptr) continue;
            wide += static_cast<wchar_t>(static_cast<unsigned char>(c));
        }
        wide = wide.substr(0, MAX_PATH - 8);
        std::wmemcpy(buffer, wide.c_str(), wide.size() + 1);
    }

    OPENFILENAMEW dialog = {};
    dialog.lStructSize = sizeof(dialog);
    dialog.hwndOwner = nullptr;
    dialog.lpstrFilter = filter;
    dialog.lpstrFile = buffer;
    dialog.nMaxFile = MAX_PATH;
    dialog.lpstrDefExt = defaultExt;
    const std::wstring wideTitle(title, title + std::strlen(title));
    dialog.lpstrTitle = wideTitle.c_str();
    dialog.Flags = OFN_EXPLORER | OFN_NOCHANGEDIR | (saving ? OFN_OVERWRITEPROMPT : OFN_FILEMUSTEXIST);

    const BOOL chosen = saving ? GetSaveFileNameW(&dialog) : GetOpenFileNameW(&dialog);
    if (!chosen) return std::string();
    return strutil::wideToUtf8(buffer);
}
#else
std::string runFileDialog(const char*, bool, const wchar_t*, const wchar_t*, const std::string&) { return std::string(); }
#endif

// A row of spin boxes across the panel's width, each captioned beside its own field.
bool vectorSpinRow(const char* id, const char* const* labels, double* values, int count, double step,
                   const char* format) {
    if (count <= 0) return false;
    ImGui::PushID(id);
    const float startX = ImGui::GetCursorPosX();
    const float cell = ImGui::GetContentRegionAvail().x / static_cast<float>(count);
    // nativeSpinBox draws its arrows outside the field it is given, so the field has to be the cell
    // less the arrows or the last box overflows the row.
    const float arrowWidth = std::floor(ImGui::GetFrameHeight() * 0.62f) + 2.0f;
    // One caption column for the whole row, taken from the widest label in it. Sizing each cell to
    // its own caption would put the three boxes at three different offsets, which is the staggering
    // the fixed origins exist to avoid.
    float captionWidth = 0.0f;
    for (int i = 0; i < count; ++i) {
        captionWidth = std::max(captionWidth, ImGui::CalcTextSize(labels[i]).x);
    }
    captionWidth += ImGui::GetStyle().ItemInnerSpacing.x * 2.0f;
    const float fieldWidth = std::max(20.0f, cell - captionWidth - arrowWidth - 2.0f);

    bool changed = false;
    for (int i = 0; i < count; ++i) {
        const float columnX = startX + cell * static_cast<float>(i);
        if (i > 0) ImGui::SameLine();
        ImGui::SetCursorPosX(columnX);
        ImGui::PushID(i);
        ImGui::AlignTextToFramePadding();
        ImGui::TextUnformatted(labels[i]);
        // SameLine before the absolute set, for the reason nativeSpinBox's leading caption gives:
        // SameLine(offset) positions from the window edge, so a bare SetCursorPosX after a wrapped
        // row would throw the field back to the left margin.
        ImGui::SameLine(0.0f, 0.0f);
        ImGui::SetCursorPosX(columnX + captionWidth);
        if (nativeSpinBox("##v", &values[i], step, format, 0.0, 0.0, fieldWidth)) changed = true;
        ImGui::PopID();
    }
    ImGui::PopID();
    return changed;
}

// Properties use a visible caption column before the editor. ImGui's default InputDouble and
// SliderScalar captions are drawn after the control, where the dock's right edge clipped them as
// soon as the input consumed the available width.
void beginStationPropertyField(const char* caption) {
    constexpr float kPreferredLabelWidth = 120.0f;
    constexpr float kMinimumInlineWidth = 260.0f;
    const float available = ImGui::GetContentRegionAvail().x;
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted(caption);
    if (available >= kMinimumInlineWidth) {
        const float rowStart = ImGui::GetCursorPosX();
        // TextUnformatted advanced to the next row; CursorStartPos is the stable left edge of the
        // current panel, including its padding.
        const float fieldX = ImGui::GetCursorStartPos().x + kPreferredLabelWidth;
        ImGui::SameLine();
        ImGui::SetCursorPosX(std::max(rowStart, fieldX));
    }
    ImGui::SetNextItemWidth(-1.0f);
}

bool stationPropertyInputDouble(const char* id, const char* caption, double* value,
                                double step, double fastStep, const char* format,
                                ImGuiInputTextFlags flags) {
    ImGui::PushID(id);
    beginStationPropertyField(caption);
    const bool changed = ImGui::InputDouble("##value", value, step, fastStep, format, flags);
    ImGui::PopID();
    return changed;
}

bool stationPropertyInputInt(const char* id, const char* caption, int* value,
                             int step, int fastStep,
                             ImGuiInputTextFlags flags) {
    ImGui::PushID(id);
    beginStationPropertyField(caption);
    const bool changed = ImGui::InputInt("##value", value, step, fastStep, flags);
    ImGui::PopID();
    return changed;
}

bool stationPropertySliderDouble(const char* id, const char* caption, double* value,
                                 const double* minimum, const double* maximum,
                                 const char* format) {
    ImGui::PushID(id);
    beginStationPropertyField(caption);
    const bool changed = ImGui::SliderScalar("##value", ImGuiDataType_Double, value,
                                             minimum, maximum, format);
    ImGui::PopID();
    return changed;
}

bool drawStationPositionEditor(CadNode* node) {
    if (!node) return false;
    bool changed = false;
    ImGui::TextDisabled("Placement (station coordinates)");
    // A rendering of `Common/PlacedMechanismSchema`, which is also what the RoboDK plug-in's properties
    // window renders. Which fields exist, what they are called, what unit each is in and how coarsely
    // each steps all come off that table, so the two cells cannot come to offer different numbers for
    // the same placement.
    std::array<double, 6> values = placedmechanism::placementValues(node->loc);
    const std::array<placedmechanism::PlacementField, 6>& fields = placedmechanism::placementFields();
    for (size_t at = 0; at < fields.size(); ++at) {
        const placedmechanism::PlacementField& field = fields[at];
        const std::string caption = std::string(field.label) + " (" + field.unit + ")";
        changed |= stationPropertyInputDouble(field.key, caption.c_str(), &values[at],
                                              field.step * 0.1, field.step, "%.2f",
                                              ImGuiInputTextFlags_EnterReturnsTrue);
    }
    // Recomposed only when something was typed. The round trip is exact to a part in 1e15 - the smoke
    // holds it there - so writing every frame would be harmless arithmetic; it would not be harmless to
    // the *station*, which would be marked edited by the panel merely being open.
    if (changed) {
        node->loc = placedmechanism::placementPose(values);
        node->needsGlobalLocUpdate = true;
    }
    return changed;
}
