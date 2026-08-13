#include "RobotLibraryPanel.h"

#include "CadNodePackage.h"
#include "MeshRobotViewer.h"
#include "RobotRuntime.h"

#include "imgui.h"

#include <algorithm>
#include <filesystem>
#include <string>
#include <utility>

namespace {

constexpr float kPreviewSize = 72.0f;
constexpr float kRowHeight = 84.0f;
constexpr int kPreviewPixels = 144;

void drawFallbackRobotIcon(ImDrawList* drawList, const ImVec2& min, const ImVec2& max) {
    const ImU32 line = IM_COL32(150, 157, 166, 255);
    const ImU32 joint = IM_COL32(42, 126, 196, 255);
    const float thickness = 4.0f;
    const ImVec2 base(min.x + 16.0f, max.y - 15.0f);
    const ImVec2 shoulder(min.x + 25.0f, min.y + 42.0f);
    const ImVec2 elbow(min.x + 44.0f, min.y + 27.0f);
    const ImVec2 tool(max.x - 12.0f, min.y + 18.0f);
    drawList->AddLine(base, shoulder, line, thickness);
    drawList->AddLine(shoulder, elbow, line, thickness);
    drawList->AddLine(elbow, tool, line, thickness);
    drawList->AddLine(ImVec2(base.x - 10.0f, base.y + 5.0f),
                      ImVec2(base.x + 12.0f, base.y + 5.0f), line, thickness);
    for (const ImVec2& point : {shoulder, elbow, tool}) {
        drawList->AddCircleFilled(point, 4.5f, joint);
    }
}

void drawFallbackRailIcon(ImDrawList* drawList, const ImVec2& min, const ImVec2& max) {
    const ImU32 rail = IM_COL32(150, 157, 166, 255);
    const ImU32 carriage = IM_COL32(42, 126, 196, 255);
    const float left = min.x + 9.0f;
    const float right = max.x - 9.0f;
    const float middle = (min.y + max.y) * 0.5f;
    drawList->AddLine(ImVec2(left, middle - 10.0f), ImVec2(right, middle - 10.0f), rail, 4.0f);
    drawList->AddLine(ImVec2(left, middle + 10.0f), ImVec2(right, middle + 10.0f), rail, 4.0f);
    drawList->AddRectFilled(ImVec2(min.x + 25.0f, middle - 20.0f),
                            ImVec2(max.x - 18.0f, middle + 20.0f), carriage, 2.0f);
}

void drawFallbackAccessoryIcon(ImDrawList* drawList, const ImVec2& min, const ImVec2& max) {
    const ImU32 steel = IM_COL32(150, 157, 166, 255);
    const ImU32 accent = IM_COL32(42, 126, 196, 255);
    const float left = min.x + 12.0f;
    const float right = max.x - 12.0f;
    const float top = min.y + 23.0f;
    const float bottom = max.y - 14.0f;
    drawList->AddRectFilled(ImVec2(left, top), ImVec2(right, top + 10.0f), steel, 2.0f);
    drawList->AddLine(ImVec2(left + 7.0f, top + 10.0f), ImVec2(left + 7.0f, bottom), steel, 4.0f);
    drawList->AddLine(ImVec2(right - 7.0f, top + 10.0f), ImVec2(right - 7.0f, bottom), steel, 4.0f);
    for (int i = 0; i < 5; ++i) {
        const float x = left + 7.0f + i * (right - left - 14.0f) / 4.0f;
        drawList->AddCircleFilled(ImVec2(x, top + 5.0f), 2.5f, accent);
    }
}

} // namespace

RobotLibraryPanel::RobotLibraryPanel() = default;
RobotLibraryPanel::~RobotLibraryPanel() = default;

void RobotLibraryPanel::releaseGraphics() {
    m_entries.clear();
    m_selected = -1;
    m_loaded = false;
}

RobotLibraryPanel::AssetRequest RobotLibraryPanel::requestFor(const librarycatalogue::Entry& entry) {
    AssetRequest request;
    request.packagePath = entry.path;
    request.displayName = entry.name;
    request.variantId = entry.variantId;
    request.assetKind = entry.assetKind;
    request.parameters = entry.parameters;
    return request;
}

void RobotLibraryPanel::loadCatalogue(const ConfigureAssetCallback& configureAsset) {
    if (m_loaded) return;
    m_loaded = true;
    m_entries.clear();

    librarycatalogue::Configure configure;
    if (configureAsset) {
        configure = [&configureAsset](CadNode* root, const librarycatalogue::Entry& scanned) {
            configureAsset(root, requestFor(scanned));
        };
    }
    for (librarycatalogue::Entry& scanned :
         librarycatalogue::scan(builtinRobotCatalogueRoot(), configure)) {
        Entry entry;
        entry.request = requestFor(scanned);
        static_cast<librarycatalogue::Entry&>(entry) = std::move(scanned);
        if (entry.root && entry.resolves) {
            entry.preview = std::make_unique<MeshRobotViewer>(entry.root.get());
            entry.preview->reframeCamera();
        }
        m_entries.push_back(std::move(entry));
    }

    if (m_entries.empty()) {
        m_selected = -1;
    } else if (m_selected < 0 || m_selected >= static_cast<int>(m_entries.size())) {
        m_selected = 0;
    }
}

void RobotLibraryPanel::refreshCatalogue(const ConfigureAssetCallback& configureAsset) {
    // Called from inside the UI frame, while the GL context is current, so destroying the cached
    // preview renderers here can release their textures and buffers safely.
    m_entries.clear();
    m_selected = -1;
    m_loaded = false;
    loadCatalogue(configureAsset);
}

bool RobotLibraryPanel::decodeAssetPayload(const void* data, int size, AssetRequest* request) {
    if (!data || size <= 1 || !request) return false;
    const char* bytes = static_cast<const char*>(data);
    const std::string text(bytes, bytes + size - (bytes[size - 1] == '\0' ? 1 : 0));
    const Json payload = Json::parse(text, nullptr, false);
    if (payload.is_discarded() || !payload.is_object()) return false;
    request->packagePath = jsoncompat::fieldString(payload, "packagePath");
    request->displayName = jsoncompat::fieldString(payload, "displayName");
    request->variantId = jsoncompat::fieldString(payload, "variantId");
    request->assetKind = jsoncompat::fieldString(payload, "assetKind");
    request->parameters = jsoncompat::fieldObject(payload, "parameters");
    return !request->packagePath.empty();
}

void RobotLibraryPanel::renderPreview(Entry& entry) {
    if (entry.previewTexture != 0 || !entry.preview) return;
    entry.previewTexture = entry.preview->render(kPreviewPixels, kPreviewPixels);
    if (entry.previewTexture == 0 && entry.error.empty()) {
        entry.error = "Preview renderer failed; see stderr.";
    }
}

void RobotLibraryPanel::drawEntry(int index, Entry& entry,
                                  const OpenPackageCallback& openPackage) {
    ImGui::PushID(index);
    renderPreview(entry);

    const bool selected = index == m_selected;
    ImGui::Selectable("##robotLibraryEntry", selected,
                      ImGuiSelectableFlags_AllowDoubleClick | ImGuiSelectableFlags_SpanAllColumns,
                      ImVec2(0.0f, kRowHeight));
    if (ImGui::IsItemClicked()) m_selected = index;
    const bool hovered = ImGui::IsItemHovered();
    if (hovered && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left) &&
        entry.category == Category::Robot && entry.resolves && openPackage) {
        openPackage(entry.request);
    }

    const ImVec2 rowMin = ImGui::GetItemRectMin();
    const ImVec2 rowMax = ImGui::GetItemRectMax();
    const ImVec2 previewMin(rowMin.x + 6.0f, rowMin.y + 6.0f);
    const ImVec2 previewMax(previewMin.x + kPreviewSize, previewMin.y + kPreviewSize);
    ImDrawList* drawList = ImGui::GetWindowDrawList();
    drawList->AddRectFilled(previewMin, previewMax, IM_COL32(20, 23, 26, 255), 2.0f);
    if (entry.previewTexture != 0) {
        drawList->AddImage(static_cast<ImTextureID>(entry.previewTexture), previewMin, previewMax,
                           ImVec2(0.0f, 1.0f), ImVec2(1.0f, 0.0f));
    } else {
        if (entry.category == Category::Robot) {
            drawFallbackRobotIcon(drawList, previewMin, previewMax);
        } else if (entry.category == Category::LinearRail) {
            drawFallbackRailIcon(drawList, previewMin, previewMax);
        } else {
            drawFallbackAccessoryIcon(drawList, previewMin, previewMax);
        }
    }
    drawList->AddRect(previewMin, previewMax, IM_COL32(145, 145, 145, 255), 2.0f);

    const float textX = previewMax.x + 10.0f;
    const float textWidth = std::max(1.0f, rowMax.x - textX - 6.0f);
    const std::string& name = entry.name;
    drawList->AddText(ImGui::GetFont(), ImGui::GetFontSize(), ImVec2(textX, rowMin.y + 10.0f),
                      ImGui::GetColorU32(ImGuiCol_Text), name.c_str(), nullptr, textWidth);
    const char* typeLabel = entry.category == Category::Robot
        ? (entry.variantLabel.empty() ? "Robot | " : "Robot variant | ")
        : (entry.category == Category::LinearRail
               ? "Linear rail | "
               : (entry.category == Category::Tool
                      ? "Tool | "
                      : (entry.variantLabel.empty() ? "Accessory | " : "Accessory preset | ")));
    const std::string typeLine = std::string(typeLabel) +
        (entry.variantLabel.empty()
             ? entry.id
             : (entry.category == Category::Accessory ? entry.request.variantId
                                                       : entry.variantLabel));
    drawList->AddText(ImVec2(textX, rowMin.y + 48.0f),
                      ImGui::GetColorU32(ImGuiCol_TextDisabled), typeLine.c_str());
    if (!entry.error.empty()) {
        drawList->AddText(ImVec2(textX, rowMin.y + 65.0f), IM_COL32(190, 65, 55, 255),
                          "Preview unavailable");
    }

    if (entry.resolves && ImGui::BeginDragDropSource()) {
        Json payload = Json::object();
        payload["packagePath"] = entry.request.packagePath;
        payload["displayName"] = entry.request.displayName;
        payload["variantId"] = entry.request.variantId;
        payload["assetKind"] = entry.request.assetKind;
        if (!entry.request.parameters.empty()) payload["parameters"] = entry.request.parameters;
        const std::string payloadText = payload.dump();
        ImGui::SetDragDropPayload(kLibraryAssetPayload, payloadText.c_str(),
                                  payloadText.size() + 1);
        if (entry.previewTexture != 0) {
            ImGui::Image(static_cast<ImTextureID>(entry.previewTexture), ImVec2(72.0f, 72.0f),
                         ImVec2(0.0f, 1.0f), ImVec2(1.0f, 0.0f));
            ImGui::SameLine();
        }
        ImGui::BeginGroup();
        ImGui::TextUnformatted(entry.request.displayName.c_str());
        ImGui::TextDisabled("Drag into the 3D view");
        ImGui::EndGroup();
        ImGui::EndDragDropSource();
    }

    if (hovered) {
        ImGui::BeginTooltip();
        ImGui::TextUnformatted(name.c_str());
        if (!entry.variantLabel.empty()) ImGui::TextDisabled("Variant: %s", entry.variantLabel.c_str());
        ImGui::TextDisabled("%s", entry.path.c_str());
        if (entry.error.empty()) {
            if (entry.category == Category::Robot) {
                ImGui::TextDisabled("Double-click to open this robot.");
            } else if (entry.category == Category::LinearRail) {
                ImGui::TextDisabled("Drag this linear rail into the 3D view.");
            } else if (entry.category == Category::Tool) {
                ImGui::TextDisabled("Drag this tool onto a compatible robot flange.");
            } else {
                ImGui::TextDisabled("Drag this accessory into the 3D view.");
            }
        } else {
            ImGui::Separator();
            ImGui::TextWrapped("%s", entry.error.c_str());
        }
        ImGui::EndTooltip();
    }

    ImGui::PopID();
}

bool RobotLibraryPanel::entryMatchesFilter(const Entry& entry) const {
    Category only = Category::Robot;
    switch (m_filter) {
    case Filter::Robots:      only = Category::Robot; break;
    case Filter::LinearRails: only = Category::LinearRail; break;
    case Filter::Accessories: only = Category::Accessory; break;
    case Filter::Tools:       only = Category::Tool; break;
    case Filter::All:         return librarycatalogue::matches(entry, nullptr, m_search.data());
    }
    return librarycatalogue::matches(entry, &only, m_search.data());
}

void RobotLibraryPanel::draw(const OpenPackageCallback& openPackage, float* snapDistancePercent,
                             const ConfigureAssetCallback& configureAsset) {
    loadCatalogue(configureAsset);

    ImGui::SetNextItemWidth(-1.0f);
    ImGui::InputTextWithHint("##librarySearch", "Search library...", m_search.data(), m_search.size());

    if (ImGui::BeginTabBar("libraryCategoryTabs")) {
        if (ImGui::BeginTabItem("ALL")) {
            m_filter = Filter::All;
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Robots")) {
            m_filter = Filter::Robots;
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Linear rails")) {
            m_filter = Filter::LinearRails;
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Accessories")) {
            m_filter = Filter::Accessories;
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Tools")) {
            m_filter = Filter::Tools;
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }

    if (snapDistancePercent) {
        ImGui::SetNextItemWidth(-1.0f);
        ImGui::SliderFloat("##mountingSnapDistance", snapDistancePercent, 0.5f, 10.0f,
                           "Snap distance %.1f%%");
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Maximum preview pull and mounting-hole alignment error as a\n"
                              "percentage of the shorter 3D viewport dimension. Robots require\n"
                              "75%% of their base-hole pattern; large accessories and rails use\n"
                              "one declared footprint reference.");
        }
    }

    const auto collectVisibleEntries = [this]() {
        std::vector<int> result;
        result.reserve(m_entries.size());
        for (int index = 0; index < static_cast<int>(m_entries.size()); ++index) {
            if (entryMatchesFilter(m_entries[static_cast<size_t>(index)])) {
                result.push_back(index);
            }
        }
        return result;
    };
    std::vector<int> visibleEntries = collectVisibleEntries();

    ImGui::TextDisabled("%zu of %zu item%s", visibleEntries.size(), m_entries.size(),
                        m_entries.size() == 1 ? "" : "s");
    ImGui::SameLine();
    const float refreshWidth = ImGui::CalcTextSize("Refresh").x +
                               ImGui::GetStyle().FramePadding.x * 2.0f;
    ImGui::SetCursorPosX(std::max(ImGui::GetCursorPosX(),
                                 ImGui::GetWindowContentRegionMax().x - refreshWidth));
    if (ImGui::SmallButton("Refresh")) {
        refreshCatalogue(configureAsset);
        visibleEntries = collectVisibleEntries();
    }
    ImGui::Separator();

    const float footerHeight = ImGui::GetFrameHeightWithSpacing() +
                               ImGui::GetTextLineHeightWithSpacing();
    if (ImGui::BeginChild("robotLibraryTree", ImVec2(0.0f, -footerHeight),
                          ImGuiChildFlags_Borders)) {
        if (m_entries.empty()) {
            ImGui::TextWrapped("No library packages were found in:\n%s",
                               builtinRobotCatalogueRoot().c_str());
        } else if (visibleEntries.empty()) {
            ImGui::TextDisabled("No library items match this filter.");
        } else if (ImGui::TreeNodeEx("Library items", ImGuiTreeNodeFlags_DefaultOpen |
                                                         ImGuiTreeNodeFlags_SpanAvailWidth)) {
            ImGuiListClipper clipper;
            clipper.Begin(static_cast<int>(visibleEntries.size()),
                          kRowHeight + ImGui::GetStyle().ItemSpacing.y);
            while (clipper.Step()) {
                for (int visibleIndex = clipper.DisplayStart;
                     visibleIndex < clipper.DisplayEnd; ++visibleIndex) {
                    const int entryIndex = visibleEntries[static_cast<size_t>(visibleIndex)];
                    drawEntry(entryIndex, m_entries[static_cast<size_t>(entryIndex)], openPackage);
                }
            }
            ImGui::TreePop();
        }
    }
    ImGui::EndChild();

    const bool selectedIsVisible =
        std::find(visibleEntries.begin(), visibleEntries.end(), m_selected) != visibleEntries.end();
    const bool canOpen = selectedIsVisible && m_selected >= 0 &&
                         m_selected < static_cast<int>(m_entries.size()) &&
                         m_entries[static_cast<size_t>(m_selected)].category == Category::Robot &&
                         m_entries[static_cast<size_t>(m_selected)].resolves;
    ImGui::BeginDisabled(!canOpen);
    if (ImGui::Button("Open selected robot", ImVec2(-1.0f, 0.0f)) && openPackage) {
        openPackage(m_entries[static_cast<size_t>(m_selected)].request);
    }
    ImGui::EndDisabled();
    ImGui::TextDisabled("Drag into 3D; use the wheel to rotate the preview.");
}
