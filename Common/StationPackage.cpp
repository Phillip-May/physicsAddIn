#include "StationPackage.h"

#include "CadNodePackage.h"
#include "JsonCompat.h"
#include "StringUtil.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <system_error>
#include <utility>

namespace {

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

constexpr const char* kBuiltinPrefix = "builtin:";

CadNode* findNodeOfType(CadNode* node, CadNodeType type) {
    if (!node) return nullptr;
    if (node->type == type) return node;
    for (auto& child : node->children) {
        if (CadNode* found = findNodeOfType(child.get(), type)) return found;
    }
    return nullptr;
}

void collectRobotNodesInStation(CadNode* node, std::vector<CadNode*>& out) {
    if (!node) return;
    if (node->type == CadNodeType::OPW6Robot) {
        out.push_back(node);
        return;
    }
    for (auto& child : node->children) collectRobotNodesInStation(child.get(), out);
}

std::shared_ptr<CadNode> detachSharedNode(CadNode* node) {
    if (!node || !node->parent) return {};
    CadNode* parent = node->parent;
    const auto found = std::find_if(parent->children.begin(), parent->children.end(),
                                    [node](const std::shared_ptr<CadNode>& child) {
                                        return child.get() == node;
                                    });
    if (found == parent->children.end()) return {};
    std::shared_ptr<CadNode> owned = *found;
    parent->children.erase(found);
    owned->parent = nullptr;
    return owned;
}

// Defaults to the Emscripten filesystem path the --preload-file list writes to, so the web build
// works without anyone remembering to set it. A desktop main() overwrites this at startup.
std::string& builtinCatalogueRootStorage() {
    static std::string root = "/packages";
    return root;
}

// The station JSON, whether it came from a loose file or from an archive entry.
bool readStationJson(const std::string& stationFile, std::string* bytes, std::string* errorMessage) {
    if (!bytes) return false;
    bytes->clear();

    if (!cadPackageIsZip(stationFile)) {
        std::ifstream file(stationFile, std::ios::binary);
        if (!file) {
            if (errorMessage) *errorMessage = "Failed to open station file: " + stationFile;
            return false;
        }
        bytes->assign((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
        if (bytes->empty()) {
            if (errorMessage) *errorMessage = "Station file is empty: " + stationFile;
            return false;
        }
        return true;
    }

    // station.json by name first, as the package loader prefers robot.json, so a station that also
    // carries embedded base packages is not confused by their JSON.
    if (readCadPackageEntry(stationFile, "station.json", bytes, nullptr)) return true;
    if (errorMessage) *errorMessage = "Station archive does not contain station.json.";
    return false;
}

bool writeTemporaryPackage(const std::string& bytes, const std::string& hint, std::string* pathOut,
                           std::string* errorMessage) {
    std::error_code code;
    std::filesystem::path directory = std::filesystem::temp_directory_path(code);
    if (code) {
        if (errorMessage) *errorMessage = "No temporary directory available for the embedded package.";
        return false;
    }
    // Named from the entry so a failure part way through leaves something recognisable behind.
    std::string leaf = std::filesystem::path(hint).filename().string();
    if (leaf.empty()) leaf = "embedded.zip";
    directory /= "robotsim_station_" + std::to_string(std::hash<std::string>{}(hint)) + "_" + leaf;

    std::ofstream file(directory, std::ios::binary | std::ios::trunc);
    if (!file) {
        if (errorMessage) *errorMessage = "Failed to unpack the embedded package: " + hint;
        return false;
    }
    file.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    file.close();
    *pathOut = directory.string();
    return true;
}

std::array<double, 3> readVec3(const Json& object, const char* key) {
    std::array<double, 3> out{{0.0, 0.0, 0.0}};
    const Json& array = jsoncompat::fieldArray(object, key);
    for (size_t i = 0; i < 3 && i < array.size(); ++i) out[i] = jsoncompat::toDouble(array[i]);
    return out;
}

// Reads a document that may be an archive entry or a file beside the station, in that order - the
// same rule base packages follow, and for the same reason: an archive that carries its own copy
// means that copy, and a stale one next to it cannot silently win.
Json readStationSideDocument(const std::string& stationFile, bool stationIsZip,
                             const std::filesystem::path& stationDirectory,
                             const std::string& reference) {
    std::string bytes;
    if (stationIsZip) {
        readCadPackageEntry(stationFile, reference, &bytes, nullptr);
    }
    if (bytes.empty()) {
        std::ifstream file(stationDirectory / reference, std::ios::binary);
        if (!file) return Json::object();
        bytes.assign((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    }
    if (bytes.empty()) return Json::object();
    const Json document = Json::parse(bytes.data(), bytes.data() + bytes.size(), nullptr,
                                      /*allow_exceptions=*/false);
    if (document.is_discarded() || !document.is_object()) return Json::object();
    return document;
}

bool writeFileUnder(const std::filesystem::path& path, const std::string& bytes,
                    std::string* errorMessage) {
    std::error_code ignored;
    std::filesystem::create_directories(path.parent_path(), ignored);
    std::ofstream file(path, std::ios::binary | std::ios::trunc);
    if (!file) {
        if (errorMessage) *errorMessage = "Failed to write: " + path.string();
        return false;
    }
    file.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    return true;
}

} // namespace

std::string stationInstanceIdFromName(const std::string& name, int index) {
    // Lowercased, with anything that is not a letter, digit or dash turned into a dash, because the
    // id becomes a folder name and travels through a zip entry path. Runs of dashes collapse so a
    // name like "Left arm (welding)" does not produce a folder full of them.
    std::string id;
    bool lastWasDash = false;
    for (char c : name) {
        const bool keep = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9');
        if (keep) {
            id += static_cast<char>(c >= 'A' && c <= 'Z' ? c - 'A' + 'a' : c);
            lastWasDash = false;
        } else if (!id.empty() && !lastWasDash) {
            id += '-';
            lastWasDash = true;
        }
    }
    while (!id.empty() && id.back() == '-') id.pop_back();
    if (id.empty()) id = "robot" + std::to_string(index + 1);
    return id;
}

void StationInstanceIds::reserve(const std::string& id) {
    if (id.empty()) return;
    if (std::find(m_taken.begin(), m_taken.end(), id) == m_taken.end()) m_taken.push_back(id);
}

std::string StationInstanceIds::derive(const std::string& name, int index) {
    const std::string base = stationInstanceIdFromName(name, index);
    if (std::find(m_taken.begin(), m_taken.end(), base) == m_taken.end()) {
        m_taken.push_back(base);
        return base;
    }
    for (int suffix = index + 1;; ++suffix) {
        const std::string candidate = base + "-" + std::to_string(suffix);
        if (std::find(m_taken.begin(), m_taken.end(), candidate) == m_taken.end()) {
            m_taken.push_back(candidate);
            return candidate;
        }
    }
}

CadTransform stationPlacementFromPositionRotation(const std::array<double, 3>& positionMm,
                                                  const std::array<double, 3>& rotationDeg) {
    const double roll = rotationDeg[0] * kDegToRad;   // about X
    const double pitch = rotationDeg[1] * kDegToRad;  // about Y
    const double yaw = rotationDeg[2] * kDegToRad;    // about Z

    const double cr = std::cos(roll), sr = std::sin(roll);
    const double cp = std::cos(pitch), sp = std::sin(pitch);
    const double cy = std::cos(yaw), sy = std::sin(yaw);

    CadTransform placement;
    placement.values[0] = cy * cp;
    placement.values[1] = cy * sp * sr - sy * cr;
    placement.values[2] = cy * sp * cr + sy * sr;
    placement.values[3] = positionMm[0];
    placement.values[4] = sy * cp;
    placement.values[5] = sy * sp * sr + cy * cr;
    placement.values[6] = sy * sp * cr - cy * sr;
    placement.values[7] = positionMm[1];
    placement.values[8] = -sp;
    placement.values[9] = cp * sr;
    placement.values[10] = cp * cr;
    placement.values[11] = positionMm[2];
    return placement;
}

void setBuiltinRobotCatalogueRoot(const std::string& directory) {
    builtinCatalogueRootStorage() = directory;
}

std::string builtinRobotCatalogueRoot() {
    return builtinCatalogueRootStorage();
}

bool packageRefIsBuiltin(const std::string& packageRef) {
    const std::string prefix(kBuiltinPrefix);
    return packageRef.size() > prefix.size() && packageRef.compare(0, prefix.size(), prefix) == 0;
}

std::string builtinRobotId(const std::string& packageRef) {
    if (!packageRefIsBuiltin(packageRef)) return std::string();
    return packageRef.substr(std::string(kBuiltinPrefix).size());
}

std::string builtinRobotPackageRef(const std::string& id) {
    return std::string(kBuiltinPrefix) + id;
}

std::string resolveBuiltinRobotPath(const std::string& id) {
    // An id names a file in the catalogue directory, so anything that could climb out of it is not
    // an id. A station is a document from elsewhere and `builtin:../../secrets` must not read one.
    if (id.empty() || id.find('/') != std::string::npos || id.find('\\') != std::string::npos ||
        id.find("..") != std::string::npos) {
        return std::string();
    }
    std::error_code ignored;
    const std::filesystem::path candidate =
        std::filesystem::path(builtinRobotCatalogueRoot()) / (id + ".zip");
    if (!std::filesystem::exists(candidate, ignored)) return std::string();
    return candidate.string();
}

std::vector<BuiltinRobot> listBuiltinRobots(bool loadNames) {
    std::vector<BuiltinRobot> found;
    std::error_code ignored;
    const std::filesystem::path root(builtinRobotCatalogueRoot());
    if (!std::filesystem::is_directory(root, ignored)) return found;

    for (const std::filesystem::directory_entry& entry :
         std::filesystem::directory_iterator(root, ignored)) {
        if (!entry.is_regular_file(ignored)) continue;
        const std::filesystem::path& file = entry.path();
        if (file.extension() != ".zip") continue;

        BuiltinRobot robot;
        robot.id = file.stem().string();
        robot.path = file.string();
        // A station in the catalogue directory is not a robot, and reporting one as a built-in
        // would offer an id that cannot be put in a station.
        if (fileIsStationPackage(robot.path)) continue;
        // A mechanism archive deliberately uses mechanism.json. The general CadNode package loader
        // can open it, but the robot catalogue must only offer archives that identify themselves
        // with robot.json.
        std::string robotManifest;
        if (!readCadPackageEntry(robot.path, "robot.json", &robotManifest, nullptr)) continue;
        robot.resolves = true;
        if (loadNames) {
            std::string packageError;
            if (std::shared_ptr<CadNode> node = loadCadNodePackage(robot.path, &packageError)) {
                if (findNodeOfType(node.get(), CadNodeType::OPW6Robot)) {
                    robot.name = node->name;
                } else {
                    // Mechanism packages share the same archive format and directory but are not
                    // choices for a station's robot package field.
                    robot.resolves = false;
                }
            } else {
                robot.resolves = false;
            }
        }
        found.push_back(robot);
    }

    std::sort(found.begin(), found.end(),
              [](const BuiltinRobot& a, const BuiltinRobot& b) { return a.id < b.id; });
    return found;
}

int promoteBuiltinPackageRefs(StationDocument* document, std::vector<std::string>* promoted) {
    if (!document) return 0;
    int changed = 0;
    const auto promote = [&](std::string& packageRef) {
        if (packageRef.empty() || packageRefIsBuiltin(packageRef)) return;
        const std::string id = std::filesystem::path(packageRef).stem().string();
        if (resolveBuiltinRobotPath(id).empty()) return;
        const std::string replacement = builtinRobotPackageRef(id);
        if (promoted) promoted->push_back(packageRef + " -> " + replacement);
        packageRef = replacement;
        ++changed;
    };
    for (StationMechanismInstance& instance : document->mechanisms) {
        promote(instance.packageRef);
    }
    for (StationAccessoryInstance& instance : document->accessories) {
        promote(instance.packageRef);
    }
    for (StationRobotInstance& instance : document->robots) {
        promote(instance.packageRef);
    }
    return changed;
}

bool fileIsStationPackage(const std::string& file) {
    std::string bytes;
    if (!readStationJson(file, &bytes, nullptr)) return false;
    const Json document = Json::parse(bytes.data(), bytes.data() + bytes.size(), nullptr,
                                      /*allow_exceptions=*/false);
    if (document.is_discarded() || !document.is_object()) return false;
    return jsoncompat::contains(document, "station_schema");
}

std::shared_ptr<CadNode> loadStationPackage(const std::string& stationFile,
                                            StationDocument* documentOut,
                                            std::string* errorMessage) {
    std::string bytes;
    if (!readStationJson(stationFile, &bytes, errorMessage)) return nullptr;

    const Json document = Json::parse(bytes.data(), bytes.data() + bytes.size(), nullptr,
                                      /*allow_exceptions=*/false);
    if (document.is_discarded() || !document.is_object()) {
        if (errorMessage) *errorMessage = "Invalid station JSON.";
        return nullptr;
    }

    // Only a *newer* schema is refused. That one may place arms by a convention this build does not
    // implement, and a cell laid out wrongly is worse than one that will not open. An older one is
    // read: every version so far has added optional sections, so a schema 1 file is a schema 2 file
    // with no per-instance data, which is exactly what it means.
    const int schema = jsoncompat::fieldInt(document, "station_schema", 0);
    if (schema < 1 || schema > kStationSchemaVersion) {
        if (errorMessage) {
            *errorMessage = strutil::format("Unsupported station schema %1; this build reads 1 to %2.")
                                .arg(schema).arg(kStationSchemaVersion).str();
        }
        return nullptr;
    }

    StationDocument parsed;
    parsed.name = jsoncompat::fieldString(document, "name");
    if (parsed.name.empty()) parsed.name = std::filesystem::path(stationFile).stem().string();
    parsed.conveyorSimulationMode =
        jsoncompat::fieldString(document, "conveyor_simulation_mode", "logical");
    if (parsed.conveyorSimulationMode != "logical" &&
        parsed.conveyorSimulationMode != "physx") {
        parsed.conveyorSimulationMode = "logical";
    }

    const Json& mechanisms = jsoncompat::fieldArray(document, "mechanisms");
    const Json& accessories = jsoncompat::fieldArray(document, "accessories");
    const Json& parameterLinks = jsoncompat::fieldArray(document, "parameter_links");
    const Json& robots = jsoncompat::fieldArray(document, "robots");

    const bool stationIsZip = cadPackageIsZip(stationFile);
    const std::filesystem::path stationDirectory =
        std::filesystem::absolute(std::filesystem::path(stationFile)).parent_path();

    auto stationRoot = std::make_shared<CadNode>();
    stationRoot->name = parsed.name;
    stationRoot->type = CadNodeType::Custom;

    // Robots, mechanisms and accessories use the same package resolution rules. `builtin:<id>`
    // names any package in the shipped catalogue; otherwise an embedded package wins over a file
    // beside the station, keeping a saved station archive self-contained.
    auto loadReferencedPackage = [&](const std::string& packageRef,
                                     const char* kind,
                                     std::string* resolvedPackagePath) -> std::shared_ptr<CadNode> {
        std::string resolvedPath;
        std::string temporaryPath;
        if (packageRefIsBuiltin(packageRef)) {
            const std::string id = builtinRobotId(packageRef);
            resolvedPath = resolveBuiltinRobotPath(id);
            if (resolvedPath.empty()) {
                if (errorMessage) {
                    *errorMessage = "Station references a built-in asset this build does not have: '" +
                                    id + "'. Catalogue: " + builtinRobotCatalogueRoot();
                }
                return nullptr;
            }
        } else {
            if (stationIsZip) {
                std::string embedded;
                if (readCadPackageEntry(stationFile, packageRef, &embedded, nullptr)) {
                    if (!writeTemporaryPackage(embedded, packageRef, &temporaryPath, errorMessage)) {
                        return nullptr;
                    }
                    resolvedPath = temporaryPath;
                }
            }
            if (resolvedPath.empty()) {
                const std::filesystem::path candidate = stationDirectory / packageRef;
                if (!std::filesystem::exists(candidate)) {
                    if (errorMessage) {
                        *errorMessage = "Station references a package that is neither embedded nor beside it: " +
                                        packageRef;
                    }
                    return nullptr;
                }
                resolvedPath = candidate.string();
            }
        }

        if (resolvedPackagePath) {
            *resolvedPackagePath = temporaryPath.empty() ? resolvedPath : std::string();
        }
        std::string packageError;
        std::shared_ptr<CadNode> loaded = loadCadNodePackage(resolvedPath, &packageError);
        if (!temporaryPath.empty()) {
            std::error_code ignored;
            std::filesystem::remove(temporaryPath, ignored);
        }
        if (!loaded && errorMessage) {
            *errorMessage = std::string("Station ") + kind + " package '" + packageRef +
                            "' failed to load: " + packageError;
        }
        return loaded;
    };

    struct LoadedMechanism {
        StationMechanismInstance instance;
        CadNode* movingFrame = nullptr;
    };
    std::vector<LoadedMechanism> loadedMechanisms;
    StationInstanceIds mechanismIds;
    for (const Json& entry : mechanisms) {
        if (!entry.is_object()) continue;
        mechanismIds.reserve(jsoncompat::fieldString(entry, "id"));
    }
    int mechanismIndex = -1;
    for (const Json& entry : mechanisms) {
        if (!entry.is_object()) continue;
        ++mechanismIndex;

        StationMechanismInstance instance;
        instance.name = jsoncompat::fieldString(entry, "name");
        instance.id = jsoncompat::fieldString(entry, "id");
        if (instance.id.empty()) instance.id = mechanismIds.derive(instance.name, mechanismIndex);
        instance.packageRef = jsoncompat::fieldString(entry, "package");
        if (instance.packageRef.empty()) {
            if (errorMessage) *errorMessage = "Station mechanism entry has no package reference.";
            return nullptr;
        }
        const Json& locArray = jsoncompat::fieldArray(entry, "loc");
        if (locArray.size() == 12) {
            for (size_t i = 0; i < 12; ++i) instance.placement.values[i] = jsoncompat::toDouble(locArray[i]);
        } else {
            instance.placement = stationPlacementFromPositionRotation(readVec3(entry, "position_mm"),
                                                                      readVec3(entry, "rotation_deg"));
        }
        instance.positionMm = jsoncompat::fieldDouble(entry, "joint_position_mm");
        instance.motionLink = jsoncompat::fieldString(entry, "motion_link");

        std::shared_ptr<CadNode> mechanism = loadReferencedPackage(
            instance.packageRef, "mechanism", &instance.resolvedPackagePath);
        if (!mechanism) return nullptr;
        CadNode* gantryNode = findNodeOfType(mechanism.get(), CadNodeType::GantryMechanism);
        GantryMechanismData* gantryData = gantryNode ? gantryNode->asGantryMechanism() : nullptr;
        if (!gantryData || !gantryData->movingFrame) {
            if (errorMessage) *errorMessage = "Station mechanism package does not contain a valid gantry moving frame.";
            return nullptr;
        }
        gantryData->positionMm = instance.positionMm;
        if (!instance.name.empty()) mechanism->name = instance.name;
        else instance.name = mechanism->name;
        mechanism->loc = instance.placement;
        mechanism->parent = stationRoot.get();
        stationRoot->children.push_back(mechanism);
        loadedMechanisms.push_back({instance, gantryData->movingFrame});
        parsed.mechanisms.push_back(instance);
    }

    StationInstanceIds accessoryIds;
    for (const Json& entry : accessories) {
        if (!entry.is_object()) continue;
        accessoryIds.reserve(jsoncompat::fieldString(entry, "id"));
    }
    int accessoryIndex = -1;
    for (const Json& entry : accessories) {
        if (!entry.is_object()) continue;
        ++accessoryIndex;

        StationAccessoryInstance instance;
        instance.name = jsoncompat::fieldString(entry, "name");
        instance.id = jsoncompat::fieldString(entry, "id");
        if (instance.id.empty()) {
            instance.id = accessoryIds.derive(instance.name, accessoryIndex);
        }
        instance.packageRef = jsoncompat::fieldString(entry, "package");
        instance.parentMechanismId = jsoncompat::fieldString(entry, "parent_mechanism");
        instance.parentRobotId = jsoncompat::fieldString(entry, "parent_robot");
        instance.activeTcpIndex = jsoncompat::fieldInt(entry, "active_tcp", 0);
        instance.hidden = jsoncompat::fieldBool(entry, "hidden", false);
        instance.folder = jsoncompat::fieldString(entry, "folder");
        if (instance.hidden && instance.folder.empty()) instance.folder = "Hidden items";
        instance.parameters = jsoncompat::fieldObject(entry, "parameters");
        if (instance.packageRef.empty()) {
            if (errorMessage) *errorMessage = "Station accessory entry has no package reference.";
            return nullptr;
        }
        if (!instance.parentMechanismId.empty() && !instance.parentRobotId.empty()) {
            if (errorMessage) {
                *errorMessage = "Station accessory cannot have both a mechanism and robot parent.";
            }
            return nullptr;
        }
        const Json& locArray = jsoncompat::fieldArray(entry, "loc");
        if (locArray.size() == 12) {
            for (size_t i = 0; i < 12; ++i) {
                instance.placement.values[i] = jsoncompat::toDouble(locArray[i]);
            }
        } else {
            instance.placement = stationPlacementFromPositionRotation(
                readVec3(entry, "position_mm"), readVec3(entry, "rotation_deg"));
        }

        std::shared_ptr<CadNode> accessory = loadReferencedPackage(
            instance.packageRef, "accessory", &instance.resolvedPackagePath);
        if (!accessory) return nullptr;
        if (findNodeOfType(accessory.get(), CadNodeType::OPW6Robot) ||
            findNodeOfType(accessory.get(), CadNodeType::GantryMechanism)) {
            if (errorMessage) {
                *errorMessage = "Station accessory package contains a robot or gantry mechanism.";
            }
            return nullptr;
        }
        if (TransformNodeData* transform = accessory->asTransform()) {
            transform->applyAccessoryParametersJson(instance.parameters);
        }
        if (!instance.name.empty()) accessory->name = instance.name;
        else instance.name = accessory->name;
        accessory->loc = instance.placement;
        CadNode* parent = stationRoot.get();
        if (!instance.parentMechanismId.empty()) {
            const auto mechanism = std::find_if(
                loadedMechanisms.begin(), loadedMechanisms.end(),
                [&](const LoadedMechanism& candidate) {
                    return candidate.instance.id == instance.parentMechanismId;
                });
            if (mechanism == loadedMechanisms.end() || !mechanism->movingFrame) {
                if (errorMessage) {
                    *errorMessage = "Station accessory references an unknown parent mechanism: " +
                                    instance.parentMechanismId;
                }
                return nullptr;
            }
            parent = mechanism->movingFrame;
        }
        if (instance.hidden) {
            auto folder = std::find_if(
                stationRoot->children.begin(), stationRoot->children.end(),
                [&](const std::shared_ptr<CadNode>& candidate) {
                    return candidate && candidate->name == instance.folder &&
                           candidate->type == CadNodeType::Custom;
                });
            if (folder == stationRoot->children.end()) {
                auto newFolder = std::make_shared<CadNode>();
                newFolder->name = instance.folder;
                newFolder->type = CadNodeType::Custom;
                newFolder->parent = stationRoot.get();
                stationRoot->children.push_back(newFolder);
                parent = newFolder.get();
            } else {
                parent = folder->get();
            }
            accessory->setVisibleRecursive(false);
        }
        accessory->parent = parent;
        parent->children.push_back(accessory);
        instance.node = accessory.get();
        parsed.accessories.push_back(instance);
    }

    for (const StationAccessoryInstance& instance : parsed.accessories) {
        const TransformNodeData* parameters = instance.node ? instance.node->asTransform() : nullptr;
        if (!parameters || parameters->accessoryConveyorRole != "spawner" ||
            parameters->accessorySpawnObjectId.empty()) {
            continue;
        }
        const auto source = std::find_if(
            parsed.accessories.begin(), parsed.accessories.end(),
            [&](const StationAccessoryInstance& candidate) {
                return candidate.id == parameters->accessorySpawnObjectId;
            });
        if (source == parsed.accessories.end() || !source->hidden || !source->node) {
            if (errorMessage) {
                *errorMessage = "Station spawner '" + instance.id +
                    "' references a missing or non-hidden source object: " +
                    parameters->accessorySpawnObjectId;
            }
            return nullptr;
        }
    }

    for (const Json& entry : parameterLinks) {
        if (!entry.is_object()) continue;
        StationParameterLink link;
        link.id = jsoncompat::fieldString(entry, "id");
        link.channel = jsoncompat::fieldString(entry, "channel");
        link.scale = jsoncompat::fieldDouble(entry, "scale", 1.0);
        link.offset = jsoncompat::fieldDouble(entry, "offset", 0.0);
        const Json& a = jsoncompat::fieldObject(entry, "a");
        const Json& b = jsoncompat::fieldObject(entry, "b");
        link.a.accessoryId = jsoncompat::fieldString(a, "accessory_id");
        link.a.port = jsoncompat::fieldString(a, "port");
        link.a.parameter = jsoncompat::fieldString(a, "parameter");
        link.b.accessoryId = jsoncompat::fieldString(b, "accessory_id");
        link.b.port = jsoncompat::fieldString(b, "port");
        link.b.parameter = jsoncompat::fieldString(b, "parameter");
        // Preserve only complete equations. Semantic validation (instance and parameter existence)
        // happens after packages are resolved, in the station validator and runtime solver.
        if (!link.a.accessoryId.empty() && !link.a.parameter.empty() &&
            !link.b.accessoryId.empty() && !link.b.parameter.empty()) {
            parsed.parameterLinks.push_back(std::move(link));
        }
    }

    // Every id the file states, before any is derived from a name: a derived id has to avoid the
    // explicit ones further down the list as well as the ones already assigned.
    StationInstanceIds instanceIds;
    for (const Json& entry : robots) {
        if (!entry.is_object()) continue;
        instanceIds.reserve(jsoncompat::fieldString(entry, "id"));
    }

    struct LoadedRobot {
        CadNode* robotNode = nullptr;
        StationRobotInstance instance;
        CadNode* packageTool = nullptr;
        CadNode* requestedTool = nullptr;
    };
    std::vector<LoadedRobot> loadedRobots;
    int entryIndex = -1;
    for (const Json& entry : robots) {
        if (!entry.is_object()) continue;
        ++entryIndex;

        StationRobotInstance instance;
        instance.name = jsoncompat::fieldString(entry, "name");
        instance.id = jsoncompat::fieldString(entry, "id");
        if (instance.id.empty()) {
            instance.id = instanceIds.derive(instance.name, entryIndex);
        }
        instance.packageRef = jsoncompat::fieldString(entry, "package");
        if (instance.packageRef.empty()) {
            if (errorMessage) *errorMessage = "Station robot entry has no package reference.";
            return nullptr;
        }
        instance.parentMechanismId = jsoncompat::fieldString(entry, "parent_mechanism");
        instance.activeToolId = jsoncompat::fieldString(entry, "active_tool");
        instance.motionLink = jsoncompat::fieldString(entry, "motion_link");
        for (const auto& programValue : jsoncompat::fieldArray(entry, "programs")) {
            if (!programValue.is_object()) continue;
            const Json programObject = jsoncompat::toObject(programValue);
            StationRobotInstance::ProgramText program;
            program.name = jsoncompat::fieldString(programObject, "name", "Program");
            program.text = jsoncompat::fieldString(programObject, "text");
            if (!program.text.empty()) instance.programs.push_back(std::move(program));
        }

        instance.configRef = jsoncompat::fieldString(entry, "config");
        if (instance.configRef.empty()) instance.configRef = "instances/" + instance.id + "/config.json";
        instance.config = readStationSideDocument(stationFile, cadPackageIsZip(stationFile),
                                                  std::filesystem::absolute(
                                                      std::filesystem::path(stationFile)).parent_path(),
                                                  instance.configRef);

        // An explicit twelve-value loc wins, for a station written by a tool that already has the
        // matrix. Everything hand-written uses the position and rotation pair.
        const Json& locArray = jsoncompat::fieldArray(entry, "loc");
        if (locArray.size() == 12) {
            for (size_t i = 0; i < 12; ++i) instance.placement.values[i] = jsoncompat::toDouble(locArray[i]);
        } else {
            instance.placement = stationPlacementFromPositionRotation(readVec3(entry, "position_mm"),
                                                                      readVec3(entry, "rotation_deg"));
        }

        std::shared_ptr<CadNode> robot = loadReferencedPackage(
            instance.packageRef, "robot", &instance.resolvedPackagePath);
        if (!robot) return nullptr;
        CadNode* robotNode = findNodeOfType(robot.get(), CadNodeType::OPW6Robot);
        if (!robotNode) {
            if (errorMessage) *errorMessage = "Station robot package does not contain an OPW6Robot node.";
            return nullptr;
        }

        // The base package's own name stands when the station does not rename the instance, so a
        // one-arm station reads the same as opening that package directly.
        if (!instance.name.empty()) robot->name = instance.name;
        else instance.name = robot->name;
        robot->loc = instance.placement;
        CadNode* parent = stationRoot.get();
        if (!instance.parentMechanismId.empty()) {
            const auto mechanism = std::find_if(
                loadedMechanisms.begin(), loadedMechanisms.end(),
                [&](const LoadedMechanism& candidate) {
                    return candidate.instance.id == instance.parentMechanismId;
                });
            if (mechanism == loadedMechanisms.end() || !mechanism->movingFrame) {
                if (errorMessage) {
                    *errorMessage = "Station robot references an unknown parent mechanism: " +
                                    instance.parentMechanismId;
                }
                return nullptr;
            }
            parent = mechanism->movingFrame;
        }
        robot->parent = parent;
        parent->children.push_back(robot);
        OPW6RobotData* robotData = robotNode->asOPW6Robot();
        loadedRobots.push_back({robotNode, instance,
                                robotData ? robotData->activeTool : nullptr,
                                nullptr});
    }

    // Tool accessories are loaded before robots so ordinary accessories can still be referenced by
    // spawners. Once the robots exist, move each tool beneath its robot root, then resolve the
    // robot's explicit active-tool state. Keeping RobotTool nodes flat beneath OPW6Robot is required
    // by RobotPoseController:
    // nesting one below the built-in flange marker would apply the link-6 delta twice.
    for (StationAccessoryInstance& accessory : parsed.accessories) {
        if (accessory.parentRobotId.empty()) continue;
        const auto robot = std::find_if(
            loadedRobots.begin(), loadedRobots.end(),
            [&](const LoadedRobot& candidate) {
                return candidate.instance.id == accessory.parentRobotId;
            });
        if (robot == loadedRobots.end() || !robot->robotNode || !accessory.node) {
            if (errorMessage) {
                *errorMessage = "Station tool references an unknown parent robot: " +
                                accessory.parentRobotId;
            }
            return nullptr;
        }
        RobotToolData* tool = accessory.node->asRobotTool();
        OPW6RobotData* robotData = robot->robotNode->asOPW6Robot();
        if (!tool || !robotData) {
            if (errorMessage) {
                *errorMessage = "A robot-parented accessory must have a RobotTool package root.";
            }
            return nullptr;
        }
        if (!tool->tcps.empty()) {
            tool->activeTcpIndex = std::max(
                0, std::min(accessory.activeTcpIndex,
                            static_cast<int>(tool->tcps.size()) - 1));
            accessory.activeTcpIndex = tool->activeTcpIndex;
        } else {
            tool->activeTcpIndex = 0;
            accessory.activeTcpIndex = 0;
        }
        std::shared_ptr<CadNode> owned = detachSharedNode(accessory.node);
        if (!owned) {
            if (errorMessage) *errorMessage = "Could not attach tool to robot flange.";
            return nullptr;
        }
        owned->parent = robot->robotNode;
        robot->robotNode->children.push_back(owned);
        if (robot->instance.activeToolId == accessory.id) {
            robot->requestedTool = owned.get();
        }
    }

    // Resolve active tool after every attachment is in the tree. This makes the selection
    // independent of accessory ordering. Merely attaching geometry does not change the active
    // kinematic frame: the package flange remains the default until the station selects a tool.
    for (LoadedRobot& robot : loadedRobots) {
        OPW6RobotData* robotData = robot.robotNode ? robot.robotNode->asOPW6Robot() : nullptr;
        if (!robotData) continue;
        if (robot.instance.activeToolId.empty() || robot.instance.activeToolId == "@package") {
            robotData->activeTool = robot.packageTool;
        } else if (!robot.instance.activeToolId.empty()) {
            if (!robot.requestedTool) {
                if (errorMessage) {
                    *errorMessage = "Station robot '" + robot.instance.id +
                        "' references an unknown active tool: " + robot.instance.activeToolId;
                }
                return nullptr;
            }
            robotData->activeTool = robot.requestedTool;
        }
    }

    // A motion link is a one-robot/one-mechanism station relationship. Refuse ambiguous or dangling
    // tags here so runtime code never has to guess which carriage a J7 value means.
    for (const LoadedRobot& robot : loadedRobots) {
        if (robot.instance.motionLink.empty()) continue;
        const int mechanismMatches = static_cast<int>(std::count_if(
            parsed.mechanisms.begin(), parsed.mechanisms.end(),
            [&](const StationMechanismInstance& mechanism) {
                return mechanism.motionLink == robot.instance.motionLink;
            }));
        const int robotMatches = static_cast<int>(std::count_if(
            loadedRobots.begin(), loadedRobots.end(),
            [&](const LoadedRobot& candidate) {
                return candidate.instance.motionLink == robot.instance.motionLink;
            }));
        if (mechanismMatches != 1 || robotMatches != 1) {
            if (errorMessage) {
                *errorMessage = "Station motion_link '" + robot.instance.motionLink +
                    "' must identify exactly one robot and one mechanism.";
            }
            return nullptr;
        }
    }
    for (const StationMechanismInstance& mechanism : parsed.mechanisms) {
        if (mechanism.motionLink.empty()) continue;
        const bool hasRobot = std::any_of(
            loadedRobots.begin(), loadedRobots.end(),
            [&](const LoadedRobot& robot) {
                return robot.instance.motionLink == mechanism.motionLink;
            });
        if (!hasRobot) {
            if (errorMessage) {
                *errorMessage = "Station mechanism motion_link '" + mechanism.motionLink +
                    "' has no matching robot.";
            }
            return nullptr;
        }
    }

    // Runtime robot instances follow depth-first scene order. Reorder the parsed entries to match
    // after attachment, since robots on different mechanisms need not appear in that order in JSON.
    std::vector<CadNode*> treeRobots;
    collectRobotNodesInStation(stationRoot.get(), treeRobots);
    for (CadNode* robotNode : treeRobots) {
        const auto loaded = std::find_if(loadedRobots.begin(), loadedRobots.end(),
                                         [robotNode](const LoadedRobot& item) {
                                             return item.robotNode == robotNode;
                                         });
        if (loaded != loadedRobots.end()) parsed.robots.push_back(loaded->instance);
    }

    if (documentOut) *documentOut = parsed;
    if (errorMessage) errorMessage->clear();
    return stationRoot;
}

bool saveStationPackage(const std::string& stationFile,
                        const StationDocument& document,
                        const std::string& sourceStationFile,
                        std::string* errorMessage) {
    const bool destinationIsZip = cadPackageIsZip(stationFile);
    const std::filesystem::path stationDirectory =
        std::filesystem::absolute(std::filesystem::path(stationFile)).parent_path();
    const bool sourceIsZip = !sourceStationFile.empty() && cadPackageIsZip(sourceStationFile);
    const std::filesystem::path sourceDirectory = sourceStationFile.empty()
        ? stationDirectory
        : std::filesystem::absolute(std::filesystem::path(sourceStationFile)).parent_path();

    Json mechanisms = Json::array();
    Json accessories = Json::array();
    Json parameterLinks = Json::array();
    Json robots = Json::array();
    std::vector<std::pair<std::string, std::string>> archiveEntries;

    StationInstanceIds mechanismIds;
    for (const StationMechanismInstance& instance : document.mechanisms) mechanismIds.reserve(instance.id);
    for (size_t i = 0; i < document.mechanisms.size(); ++i) {
        const StationMechanismInstance& instance = document.mechanisms[i];
        const std::string id = instance.id.empty()
            ? mechanismIds.derive(instance.name, static_cast<int>(i))
            : instance.id;
        Json entry = Json::object();
        entry["id"] = id;
        entry["name"] = instance.name;
        entry["package"] = instance.packageRef;
        Json loc = Json::array();
        for (double value : instance.placement.values) loc.push_back(value);
        entry["loc"] = loc;
        entry["joint_position_mm"] = instance.positionMm;
        if (!instance.motionLink.empty()) entry["motion_link"] = instance.motionLink;
        mechanisms.push_back(entry);
    }

    StationInstanceIds accessoryIds;
    for (const StationAccessoryInstance& instance : document.accessories) {
        accessoryIds.reserve(instance.id);
    }
    for (size_t i = 0; i < document.accessories.size(); ++i) {
        const StationAccessoryInstance& instance = document.accessories[i];
        const std::string id = instance.id.empty()
            ? accessoryIds.derive(instance.name, static_cast<int>(i))
            : instance.id;
        Json entry = Json::object();
        entry["id"] = id;
        entry["name"] = instance.name;
        entry["package"] = instance.packageRef;
        if (!instance.parentMechanismId.empty()) {
            entry["parent_mechanism"] = instance.parentMechanismId;
        }
        if (!instance.parentRobotId.empty()) {
            entry["parent_robot"] = instance.parentRobotId;
            if (instance.activeTcpIndex != 0) entry["active_tcp"] = instance.activeTcpIndex;
        }
        if (instance.hidden) {
            entry["hidden"] = true;
            entry["folder"] = instance.folder.empty() ? "Hidden items" : instance.folder;
        }
        Json loc = Json::array();
        for (double value : instance.placement.values) loc.push_back(value);
        entry["loc"] = loc;
        if (!instance.parameters.empty()) {
            entry["parameters"] = instance.parameters;
        } else if (instance.node) {
            if (const TransformNodeData* transform = instance.node->asTransform()) {
                if (transform->hasParametricAccessory()) {
                    entry["parameters"] = transform->accessoryParametersJson();
                }
            }
        }
        accessories.push_back(entry);
    }

    for (const StationParameterLink& link : document.parameterLinks) {
        Json entry = Json::object();
        if (!link.id.empty()) entry["id"] = link.id;
        if (!link.channel.empty()) entry["channel"] = link.channel;
        entry["scale"] = link.scale;
        entry["offset"] = link.offset;
        entry["a"] = Json::object({
            {"accessory_id", link.a.accessoryId},
            {"port", link.a.port},
            {"parameter", link.a.parameter}});
        entry["b"] = Json::object({
            {"accessory_id", link.b.accessoryId},
            {"port", link.b.port},
            {"parameter", link.b.parameter}});
        parameterLinks.push_back(std::move(entry));
    }

    // Reserved before the loop derives anything, and through the same assigner the loader uses, so
    // the two agree about which folder an arm's config lives in.
    StationInstanceIds instanceIds;
    for (const StationRobotInstance& instance : document.robots) instanceIds.reserve(instance.id);

    for (size_t i = 0; i < document.robots.size(); ++i) {
        const StationRobotInstance& instance = document.robots[i];
        const std::string id = instance.id.empty()
            ? instanceIds.derive(instance.name, static_cast<int>(i))
            : instance.id;
        const std::string configRef = instance.configRef.empty()
            ? "instances/" + id + "/config.json"
            : instance.configRef;

        Json entry = Json::object();
        entry["id"] = id;
        entry["name"] = instance.name;
        entry["package"] = instance.packageRef;
        if (!instance.parentMechanismId.empty()) {
            entry["parent_mechanism"] = instance.parentMechanismId;
        }
        if (!instance.activeToolId.empty()) entry["active_tool"] = instance.activeToolId;
        if (!instance.motionLink.empty()) entry["motion_link"] = instance.motionLink;
        Json programs = Json::array();
        for (const StationRobotInstance::ProgramText& program : instance.programs) {
            programs.push_back(Json{{"name", program.name}, {"text", program.text}});
        }
        if (!programs.empty()) entry["programs"] = programs;
        Json loc = Json::array();
        for (double value : instance.placement.values) loc.push_back(value);
        entry["loc"] = loc;
        // Only claimed when there is something in the folder. A station with no calibration for an
        // arm should not point at a file that is not there.
        if (!instance.config.empty()) entry["config"] = configRef;
        robots.push_back(entry);

        if (instance.config.empty()) continue;
        const std::string configBytes = instance.config.dump(4);
        if (destinationIsZip) {
            archiveEntries.emplace_back(configRef, configBytes);
        } else if (!writeFileUnder(stationDirectory / configRef, configBytes, errorMessage)) {
            return false;
        }
    }

    Json root = Json::object();
    root["station_schema"] = kStationSchemaVersion;
    root["name"] = document.name;
    root["conveyor_simulation_mode"] =
        document.conveyorSimulationMode == "physx" ? "physx" : "logical";
    if (!mechanisms.empty()) root["mechanisms"] = mechanisms;
    if (!accessories.empty()) root["accessories"] = accessories;
    if (!parameterLinks.empty()) root["parameter_links"] = parameterLinks;
    root["robots"] = robots;
    const std::string stationBytes = root.dump(4);

    if (!destinationIsZip) {
        if (!writeFileUnder(std::filesystem::path(stationFile), stationBytes, errorMessage)) return false;
        if (errorMessage) errorMessage->clear();
        return true;
    }

    // station.json first so the entry order reads the way the archive is meant to be understood.
    archiveEntries.emplace(archiveEntries.begin(), "station.json", stationBytes);

    // Robot and mechanism packages travel with the archive. Each distinct reference is copied once
    // however many instances share it.
    struct PackageToCopy {
        std::string packageRef;
        std::string resolvedPackagePath;
    };
    std::vector<PackageToCopy> packagesToCopy;
    for (const StationMechanismInstance& instance : document.mechanisms) {
        packagesToCopy.push_back({instance.packageRef, instance.resolvedPackagePath});
    }
    for (const StationAccessoryInstance& instance : document.accessories) {
        packagesToCopy.push_back({instance.packageRef, instance.resolvedPackagePath});
    }
    for (const StationRobotInstance& instance : document.robots) {
        packagesToCopy.push_back({instance.packageRef, instance.resolvedPackagePath});
    }
    std::vector<std::string> copied;
    for (const PackageToCopy& instance : packagesToCopy) {
        if (packageRefIsBuiltin(instance.packageRef)) continue;
        if (std::find(copied.begin(), copied.end(), instance.packageRef) != copied.end()) continue;
        copied.push_back(instance.packageRef);

        const auto readInto = [&](const std::filesystem::path& from, std::string* bytes) {
            std::ifstream file(from, std::ios::binary);
            if (!file) return;
            bytes->assign((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
        };

        std::string packageBytes;
        if (sourceIsZip) readCadPackageEntry(sourceStationFile, instance.packageRef, &packageBytes, nullptr);
        // Resolve a bare package reference relative to the file that was actually loaded.
        if (packageBytes.empty() && !instance.resolvedPackagePath.empty()) {
            readInto(std::filesystem::path(instance.resolvedPackagePath), &packageBytes);
        }
        if (packageBytes.empty()) readInto(sourceDirectory / instance.packageRef, &packageBytes);
        if (packageBytes.empty()) {
            if (errorMessage) {
                *errorMessage = "Cannot embed package '" + instance.packageRef +
                                "': it was not found in the source station or beside it.";
            }
            return false;
        }
        archiveEntries.emplace_back(instance.packageRef, packageBytes);
    }

    if (!writeCadPackageArchive(stationFile, archiveEntries, errorMessage)) return false;
    if (errorMessage) errorMessage->clear();
    return true;
}
