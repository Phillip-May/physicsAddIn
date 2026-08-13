#pragma once

#include "CadNode.h"
#include "LibraryCatalogue.h"
#include "StationPackage.h"

#include <array>
#include <functional>
#include <memory>
#include <string>
#include <vector>

class MeshRobotViewer;

// The left-panel catalogue of robot and mechanism packages shipped beside RobotSimulator.
class RobotLibraryPanel {
public:
    struct AssetRequest {
        std::string packagePath;
        std::string displayName;
        std::string variantId;
        std::string assetKind;
        Json parameters = Json::object();
    };
    using OpenPackageCallback = std::function<void(const AssetRequest& request)>;
    using ConfigureAssetCallback =
        std::function<void(CadNode* root, const AssetRequest& request)>;
    static constexpr const char* kLibraryAssetPayload = "ROBOT_LIBRARY_ASSET";

    RobotLibraryPanel();
    ~RobotLibraryPanel();

    RobotLibraryPanel(const RobotLibraryPanel&) = delete;
    RobotLibraryPanel& operator=(const RobotLibraryPanel&) = delete;

    void draw(const OpenPackageCallback& openPackage, float* snapDistancePercent = nullptr,
              const ConfigureAssetCallback& configureAsset = {});
    static bool decodeAssetPayload(const void* data, int size, AssetRequest* request);
    // Releases cached GL textures while the application context is still current.
    void releaseGraphics();

private:
    using Category = librarycatalogue::Category;

    enum class Filter {
        All,
        Robots,
        LinearRails,
        Accessories,
        Tools
    };

    // The catalogue's entry, plus the two things that are only a rendered list's: the request this row
    // hands to a placement, and the little offscreen renderer that gives it a thumbnail.
    struct Entry : librarycatalogue::Entry {
        std::unique_ptr<MeshRobotViewer> preview;
        unsigned int previewTexture = 0;
        AssetRequest request;
    };

    static AssetRequest requestFor(const librarycatalogue::Entry& entry);
    void loadCatalogue(const ConfigureAssetCallback& configureAsset);
    void refreshCatalogue(const ConfigureAssetCallback& configureAsset);
    void renderPreview(Entry& entry);
    void drawEntry(int index, Entry& entry, const OpenPackageCallback& openPackage);
    bool entryMatchesFilter(const Entry& entry) const;

    std::vector<Entry> m_entries;
    bool m_loaded = false;
    int m_selected = -1;
    Filter m_filter = Filter::All;
    std::array<char, 160> m_search{};
};
