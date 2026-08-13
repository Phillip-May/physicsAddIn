#include "LibraryCatalogue.h"

#include "CadNodePackage.h"
#include "RobotRuntime.h"
#include "StringUtil.h"

#include <algorithm>
#include <filesystem>
#include <system_error>
#include <utility>

namespace librarycatalogue {
namespace {

const char* assetKindOf(Category category) {
    switch (category) {
    case Category::Tool:        return "tool";
    case Category::Accessory:   return "accessory";
    case Category::LinearRail:  return "linear_rail";
    case Category::Robot:       break;
    }
    return "robot";
}

// One package, before its presets are expanded. Empty `category`-defining manifests mean this is not a
// library package at all, which is not an error - a catalogue directory may hold other things.
bool readEntry(const std::filesystem::path& package, Entry* entry, Json* manifest) {
    const std::string packagePath = package.string();
    std::string robotManifest;
    std::string mechanismManifest;
    std::string accessoryManifest;
    const bool hasRobot = readCadPackageEntry(packagePath, "robot.json", &robotManifest, nullptr);
    const bool hasMechanism =
        readCadPackageEntry(packagePath, "mechanism.json", &mechanismManifest, nullptr);
    const bool hasAccessory =
        readCadPackageEntry(packagePath, "accessory.json", &accessoryManifest, nullptr);
    if (!hasRobot && !hasMechanism && !hasAccessory) return false;

    *manifest = Json::object();
    if (hasAccessory) {
        *manifest = Json::parse(accessoryManifest, nullptr, false);
        if (manifest->is_discarded()) *manifest = Json::object();
    }
    const bool toolPackage =
        hasAccessory &&
        strutil::equalsCaseInsensitive(jsoncompat::fieldString(*manifest, "libraryCategory"), "tool");

    entry->id = package.stem().string();
    entry->path = packagePath;
    entry->category = hasRobot
        ? Category::Robot
        : (hasMechanism ? Category::LinearRail
                        : (toolPackage ? Category::Tool : Category::Accessory));
    return true;
}

bool resolveTree(Entry* entry) {
    std::string packageError;
    entry->root = loadCadNodePackage(entry->path, &packageError);
    if (!entry->root) {
        entry->error = packageError.empty() ? "Package could not be loaded." : packageError;
        return true;
    }
    if (entry->category == Category::Robot) {
        if (!validateRobotPackage(entry->root.get(), &packageError)) {
            entry->error = packageError.empty() ? "Robot package is invalid." : packageError;
        } else if (const CadNode* robotNode = findOPW6RobotNode(entry->root.get())) {
            entry->name = robotNode->name;
            entry->resolves = true;
        }
        return true;
    }
    if (entry->category == Category::LinearRail) {
        const CadNode* gantryNode = findGantryMechanismNode(entry->root.get());
        if (!gantryNode) return false;
        entry->name = gantryNode->name;
        entry->resolves = true;
        return true;
    }
    if (findOPW6RobotNode(entry->root.get()) || findGantryMechanismNode(entry->root.get())) {
        return false;
    }
    entry->name = entry->root->name;
    entry->resolves = true;
    return true;
}

void finishEntry(Entry* entry, const Configure& configure) {
    if (entry->name.empty()) entry->name = entry->id;
    entry->assetKind = assetKindOf(entry->category);
    if (entry->root && entry->resolves && configure) configure(entry->root.get(), *entry);
}

} // namespace

std::vector<Entry> scan(const std::string& catalogueRoot, const Configure& configure) {
    std::vector<Entry> entries;
    std::error_code ignored;
    const std::filesystem::path root(catalogueRoot);
    if (!std::filesystem::is_directory(root, ignored)) return entries;

    std::vector<std::filesystem::path> packages;
    for (const std::filesystem::directory_entry& found :
         std::filesystem::directory_iterator(root, ignored)) {
        if (found.is_regular_file(ignored) && found.path().extension() == ".zip") {
            packages.push_back(found.path());
        }
    }
    // Sorted, so a catalogue reads the same on every machine and in every host.
    std::sort(packages.begin(), packages.end());

    for (const std::filesystem::path& package : packages) {
        Entry entry;
        Json manifest;
        if (!readEntry(package, &entry, &manifest)) continue;
        if (!resolveTree(&entry)) continue;

        const Json& variants = jsoncompat::fieldArray(manifest, "libraryVariants");
        if (entry.category != Category::Accessory || variants.empty()) {
            finishEntry(&entry, configure);
            entries.push_back(std::move(entry));
            continue;
        }

        // Presets. Each is its own entry with its own tree, because each is a different object as far
        // as anyone placing one is concerned - six identically named conveyors in a list is not a
        // catalogue. The first reuses the tree already loaded; the rest load their own, since applying
        // a preset's parameters rebuilds the geometry.
        bool first = true;
        for (const Json& variantData : variants) {
            if (!variantData.is_object()) continue;
            Entry variant;
            if (first) {
                variant = std::move(entry);
                first = false;
            } else {
                variant.id = package.stem().string();
                variant.path = package.string();
                variant.category = Category::Accessory;
                std::string variantError;
                variant.root = loadCadNodePackage(variant.path, &variantError);
                variant.resolves = variant.root != nullptr;
                if (!variant.root) variant.error = variantError;
            }
            variant.variantLabel = jsoncompat::fieldString(variantData, "name");
            variant.variantId = jsoncompat::fieldString(variantData, "id");
            variant.parameters = jsoncompat::fieldObject(variantData, "parameters");
            // The preset is the operator-facing asset name, not merely secondary metadata. This keeps
            // the tree, the drag tooltip, the placement ghost, the selection and the saved station
            // name consistent instead of showing six indistinguishable default conveyors.
            variant.name = variant.variantLabel;
            if (variant.root && variant.resolves) {
                if (TransformNodeData* transform = variant.root->asTransform()) {
                    transform->applyAccessoryParametersJson(variant.parameters);
                }
            }
            finishEntry(&variant, configure);
            entries.push_back(std::move(variant));
        }
    }
    return entries;
}

bool matches(const Entry& entry, const Category* only, const std::string& search) {
    if (only && entry.category != *only) return false;
    if (search.empty()) return true;
    return strutil::containsCaseInsensitive(entry.name, search) ||
           strutil::containsCaseInsensitive(entry.id, search);
}

const char* categoryLabel(Category category) {
    switch (category) {
    case Category::Robot:       return "Robot";
    case Category::LinearRail:  return "Linear rail";
    case Category::Tool:        return "Tool";
    case Category::Accessory:   break;
    }
    return "Accessory";
}

} // namespace librarycatalogue
