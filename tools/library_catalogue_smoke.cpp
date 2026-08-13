
#include "LibraryCatalogue.h"
#include "MountingSnap.h"
#include "RobotRuntime.h"

#include <algorithm>
#include <cstdio>
#include <map>
#include <string>
#include <vector>

namespace {

int g_failures = 0;

void check(bool condition, const std::string& what) {
    if (condition) return;
    std::printf("  FAIL: %s\n", what.c_str());
    ++g_failures;
}

const char* kDefaultRoot = "library/packages";

const librarycatalogue::Entry* entryWithId(const std::vector<librarycatalogue::Entry>& entries,
                                           const std::string& id) {
    for (const librarycatalogue::Entry& entry : entries) {
        if (entry.id == id) return &entry;
    }
    return nullptr;
}

std::vector<const librarycatalogue::Entry*> entriesWithId(
    const std::vector<librarycatalogue::Entry>& entries, const std::string& id) {
    std::vector<const librarycatalogue::Entry*> found;
    for (const librarycatalogue::Entry& entry : entries) {
        if (entry.id == id) found.push_back(&entry);
    }
    return found;
}

std::string categoryOf(const librarycatalogue::Entry& entry) {
    return librarycatalogue::categoryLabel(entry.category);
}

// Validate every package's declared category.
void everyPackageIsTheKindItSaysItIs(const std::vector<librarycatalogue::Entry>& entries) {
    const std::map<std::string, librarycatalogue::Category> expected = {
        {"ar4_6dof_robot", librarycatalogue::Category::Robot},
        {"fairino_fr20_robot", librarycatalogue::Category::Robot},
        {"fanuc_m800ia_60_robot", librarycatalogue::Category::Robot},
        {"gudel_tmf1_6m_gantry", librarycatalogue::Category::LinearRail},
        {"ws2000_gantry", librarycatalogue::Category::LinearRail},
        {"robotiq_hand_e_dual_tool", librarycatalogue::Category::Tool},
        {"ar4_table_100mm_pitch", librarycatalogue::Category::Accessory},
        {"pedestal_h12in_type1", librarycatalogue::Category::Accessory},
        {"box_workpiece", librarycatalogue::Category::Accessory},
        {"roller_conveyor", librarycatalogue::Category::Accessory},
    };
    for (const auto& wanted : expected) {
        const librarycatalogue::Entry* entry = entryWithId(entries, wanted.first);
        check(entry != nullptr, wanted.first + " is in the catalogue");
        if (!entry) continue;
        check(entry->category == wanted.second,
              wanted.first + " is a " + librarycatalogue::categoryLabel(wanted.second) +
                  ", not a " + categoryOf(*entry));
        check(entry->resolves, wanted.first + " loaded: " + entry->error);
        check(entry->root != nullptr, wanted.first + " has a tree to draw or place");
        check(!entry->name.empty() && !entry->assetKind.empty(),
              wanted.first + " has a name and an asset kind");
    }
}

void aPackagesPresetsAreItsAssets(const std::vector<librarycatalogue::Entry>& entries) {
    const std::vector<const librarycatalogue::Entry*> conveyors =
        entriesWithId(entries, "roller_conveyor");
    check(conveyors.size() >= 5,
          "the roller conveyor's presets are entries of their own; there are " +
              std::to_string(conveyors.size()));

    std::vector<std::string> names;
    std::vector<std::string> variantIds;
    for (const librarycatalogue::Entry* entry : conveyors) {
        check(!entry->variantLabel.empty(), "each preset carries its label");
        check(entry->name == entry->variantLabel,
              "the preset's label is the name an operator reads, not '" + entry->name + "'");
        check(entry->assetKind == "accessory", "a conveyor preset is placed as an accessory");
        names.push_back(entry->name);
        variantIds.push_back(entry->variantId);
    }
    std::sort(names.begin(), names.end());
    std::sort(variantIds.begin(), variantIds.end());
    check(std::adjacent_find(names.begin(), names.end()) == names.end(),
          "no two presets are called the same thing");
    check(std::adjacent_find(variantIds.begin(), variantIds.end()) == variantIds.end(),
          "and no two share a variant id");

    const librarycatalogue::Entry* straight = nullptr;
    const librarycatalogue::Entry* spiral = nullptr;
    for (const librarycatalogue::Entry* entry : conveyors) {
        if (entry->variantId == "straight-890") straight = entry;
        if (entry->variantId == "spiral-ascending-720") spiral = entry;
    }
    check(straight && spiral, "the straight and the spiral presets are both there");
    if (!straight || !spiral || !straight->root || !spiral->root) return;
    const TransformNodeData* straightData = straight->root->asTransform();
    const TransformNodeData* spiralData = spiral->root->asTransform();
    check(straightData && spiralData, "both presets' roots carry accessory parameters");
    if (!straightData || !spiralData) return;
    check(std::abs(spiralData->accessoryTurnAngleDeg) > std::abs(straightData->accessoryTurnAngleDeg),
          "the spiral preset turns and the straight one does not: " +
              std::to_string(spiralData->accessoryTurnAngleDeg) + " against " +
              std::to_string(straightData->accessoryTurnAngleDeg) + " deg");
    check(spiralData->accessoryEndHeightMm > straightData->accessoryEndHeightMm + 100.0,
          "and it climbs: it ends at " + std::to_string(spiralData->accessoryEndHeightMm) +
              " mm against " + std::to_string(straightData->accessoryEndHeightMm));
}

void everyPackageCanBePointedAt(const std::vector<librarycatalogue::Entry>& entries) {
    std::printf("--- every package declares a pattern to be placed by\n");
    for (const librarycatalogue::Entry& entry : entries) {
        if (!entry.resolves || !entry.root) continue; // said already, by the gate above
        std::vector<mountingsnap::Interface> sources;
        mountingsnap::collectPlacementSources(entry.root.get(), &sources);
        size_t points = 0;
        for (const mountingsnap::Interface& source : sources) points += source.pointsMm.size();
        check(!sources.empty(),
              entry.name + " declares a pattern it can be placed by; it declares none, so nothing can "
                           "point at it");
        check(points >= 3, entry.name + " offers " + std::to_string(points) +
                               " hole(s) to be placed by, which is too few to seat it");
    }
}

void theOrderIsTheSameEveryTime(const std::string& root) {
    const std::vector<librarycatalogue::Entry> first = librarycatalogue::scan(root);
    const std::vector<librarycatalogue::Entry> again = librarycatalogue::scan(root);
    check(first.size() == again.size(), "two scans find the same number of entries");
    if (first.size() != again.size()) return;
    for (size_t at = 0; at < first.size(); ++at) {
        if (first[at].id == again[at].id && first[at].variantId == again[at].variantId) continue;
        check(false, "entry " + std::to_string(at) + " moved between scans");
        return;
    }
}

void filteringAndSearching(const std::vector<librarycatalogue::Entry>& entries) {
    const librarycatalogue::Category robot = librarycatalogue::Category::Robot;
    int robots = 0;
    for (const librarycatalogue::Entry& entry : entries) {
        if (librarycatalogue::matches(entry, &robot, std::string())) ++robots;
    }
    check(robots >= 3, "the Robots filter finds the robots, and found " + std::to_string(robots));

    const librarycatalogue::Entry* pedestal = entryWithId(entries, "pedestal_h12in_type1");
    check(pedestal != nullptr, "the pedestal is there to search for");
    if (!pedestal) return;
    check(librarycatalogue::matches(*pedestal, nullptr, "PEDESTAL"),
          "search is case-insensitive over the id");
    check(!librarycatalogue::matches(*pedestal, &robot, std::string()),
          "and a pedestal is not a robot");
    check(!librarycatalogue::matches(*pedestal, nullptr, "not-in-this-library"),
          "a search that matches nothing matches nothing");
}

void amissingCatalogueIsEmpty() {
    check(librarycatalogue::scan("no/such/directory/at/all").empty(),
          "scanning a directory that is not there gives an empty catalogue");
    check(librarycatalogue::scan(std::string()).empty(), "and so does scanning nothing");
}

void ar4PackageSolvesToolPoses(const std::vector<librarycatalogue::Entry>& entries) {
    const librarycatalogue::Entry* ar4 = entryWithId(entries, "ar4_6dof_robot");
    if (!ar4 || !ar4->root) return;

    RobotPoseController controller;
    std::string error;
    const bool bound = controller.bind(ar4->root.get(), &error);
    check(bound, "the AR4 pose controller binds: " + error);
    if (!bound) return;

    CadTransform target = controller.toolPose();
    target.values[3] += 50.0;
    check(controller.setToolPose(target, &error), "the AR4 solves a nearby tool pose: " + error);
}

} // namespace

int main(int argc, char** argv) {
    const std::string root = argc > 1 ? argv[1] : kDefaultRoot;
    const std::vector<librarycatalogue::Entry> entries = librarycatalogue::scan(root);
    if (entries.empty()) {
        std::printf("library catalogue smoke FAILED: no packages under %s\n", root.c_str());
        return 1;
    }

    everyPackageIsTheKindItSaysItIs(entries);
    aPackagesPresetsAreItsAssets(entries);
    everyPackageCanBePointedAt(entries);
    theOrderIsTheSameEveryTime(root);
    filteringAndSearching(entries);
    amissingCatalogueIsEmpty();
    ar4PackageSolvesToolPoses(entries);

    std::printf("  %zu entries under %s:\n", entries.size(), root.c_str());
    for (const librarycatalogue::Entry& entry : entries) {
        std::printf("    %-13s %-28s %s%s\n", categoryOf(entry).c_str(), entry.name.c_str(),
                    entry.id.c_str(),
                    entry.variantLabel.empty() ? "" : (" / " + entry.variantId).c_str());
    }

    if (g_failures > 0) {
        std::printf("library catalogue smoke FAILED with %d problem(s)\n", g_failures);
        return 1;
    }
    std::printf("library catalogue smoke passed\n");
    return 0;
}
