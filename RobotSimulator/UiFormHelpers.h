#pragma once

#include <string>

#include "imgui.h"

struct CadNode;

// Label column width for form rows where the caption sits to the left of the control.
inline constexpr float kFormLabelWidth = 58.0f;
// The border drawn around the whole window while inside a robot.
inline const ImVec4 kContextFrameColor(0.16f, 0.20f, 0.28f, 1.0f);

inline constexpr const wchar_t* kProgramFilter = L"Robot Program Text\0*.robotprog.txt;*.txt\0All Files\0*.*\0";
inline constexpr const wchar_t* kMasteringFilter = L"Mastering Calibration\0*.json\0All Files\0*.*\0";
// A station saves as an archive by default: that is the form that carries the per-instance folders
// and the base packages together. The loose JSON is offered too, for a cell kept in version control
// beside the packages it names.
inline constexpr const wchar_t* kStationFilter = L"Station Archive\0*.zip\0Station JSON\0*.json\0All Files\0*.*\0";

bool formButton(const char* label, int columns = 2, bool disabled = false);
bool nativeSpinBox(const char* label,
                   double* value,
                   double step,
                   const char* format,
                   double minValue,
                   double maxValue,
                   float fieldWidth = 96.0f,
                   const char* leadingLabel = nullptr);
// The filter and default extension are per call site: the dialog is shared by the program text
// and the mastering JSON.
std::string runFileDialog(const char* title, bool saving, const wchar_t* filter, const wchar_t* defaultExt,
                          const std::string& suggestedName = std::string());
bool vectorSpinRow(const char* id, const char* const* labels, double* values, int count, double step,
                   const char* format);
void beginStationPropertyField(const char* caption);
bool stationPropertyInputDouble(const char* id, const char* caption, double* value,
                                double step, double fastStep, const char* format,
                                ImGuiInputTextFlags flags = 0);
bool stationPropertyInputInt(const char* id, const char* caption, int* value,
                             int step, int fastStep,
                             ImGuiInputTextFlags flags = 0);
bool stationPropertySliderDouble(const char* id, const char* caption, double* value,
                                 const double* minimum, const double* maximum,
                                 const char* format);
bool drawStationPositionEditor(CadNode* node);
