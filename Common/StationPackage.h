#pragma once

#include "CadNode.h"
#include "JsonCompat.h"

#include <memory>
#include <string>
#include <vector>

// Station files compose reusable robot, mechanism, and accessory packages into one cell.
// Older schemas are upgraded on read; newer schemas are rejected.
constexpr int kStationSchemaVersion = 12;

struct StationMechanismInstance {
    std::string id;
    std::string name;
    std::string packageRef;
    std::string resolvedPackagePath;
    CadTransform placement;
    double positionMm = 0.0;
    // Matches a robot's motionLink when exposed as an external axis.
    std::string motionLink;
};

struct StationAccessoryInstance {
    std::string id;
    std::string name;
    std::string packageRef;
    std::string resolvedPackagePath;
    CadTransform placement;
    // Empty parent ids mean station-rooted.
    std::string parentMechanismId;
    std::string parentRobotId;
    int activeTcpIndex = 0;
    bool hidden = false;
    std::string folder;
    // Per-instance overrides; the runtime node is not serialized.
    Json parameters;
    CadNode* node = nullptr;
};

struct StationParameterEndpoint {
    std::string accessoryId;
    std::string port;
    std::string parameter;
};

// Persistent bidirectional relation: b = a * scale + offset.
struct StationParameterLink {
    std::string id;
    std::string channel;
    StationParameterEndpoint a;
    StationParameterEndpoint b;
    double scale = 1.0;
    double offset = 0.0;
};

struct StationRobotInstance {
    // Stable storage key. Name is operator-facing and may change.
    std::string id;
    std::string name;
    // builtin:<id>, an embedded archive entry, or a path relative to the station.
    std::string packageRef;
    // Empty for embedded packages.
    std::string resolvedPackagePath;
    CadTransform placement;
    std::string parentMechanismId;
    // Empty or "@package" selects the package's built-in tool.
    std::string activeToolId;
    std::string motionLink;

    struct ProgramText {
        std::string name;
        std::string text;
    };
    std::vector<ProgramText> programs;

    // Defaults to instances/<id>/config.json.
    std::string configRef;
    // Per-installation calibration, planning, and kinematics settings.
    Json config;
};

struct StationDocument {
    std::string name;
    std::string conveyorSimulationMode = "logical";
    std::vector<StationMechanismInstance> mechanisms;
    std::vector<StationAccessoryInstance> accessories;
    std::vector<StationParameterLink> parameterLinks;
    std::vector<StationRobotInstance> robots;
};

struct BuiltinRobot {
    std::string id;
    std::string name;
    std::string path;
    bool resolves = false;
};

// Defaults to /packages for Emscripten.
void setBuiltinRobotCatalogueRoot(const std::string& directory);
std::string builtinRobotCatalogueRoot();

bool packageRefIsBuiltin(const std::string& packageRef);
std::string builtinRobotId(const std::string& packageRef);
std::string builtinRobotPackageRef(const std::string& id);

std::string resolveBuiltinRobotPath(const std::string& id);

// Sorted by id. loadNames opens each package to read its display name.
std::vector<BuiltinRobot> listBuiltinRobots(bool loadNames = false);

// Rewrites matching package paths to builtin:<id> references.
int promoteBuiltinPackageRefs(StationDocument* document,
                              std::vector<std::string>* promoted = nullptr);

// Identifies stations by station_schema, not by extension.
bool fileIsStationPackage(const std::string& file);

std::shared_ptr<CadNode> loadStationPackage(const std::string& stationFile,
                                            StationDocument* document = nullptr,
                                            std::string* errorMessage = nullptr);

// A .zip destination embeds instance configuration and referenced packages.
bool saveStationPackage(const std::string& stationFile,
                        const StationDocument& document,
                        const std::string& sourceStationFile = std::string(),
                        std::string* errorMessage = nullptr);

std::string stationInstanceIdFromName(const std::string& name, int index);

class StationInstanceIds {
public:
    void reserve(const std::string& id);
    std::string derive(const std::string& name, int index);

private:
    std::vector<std::string> m_taken;
};

// rotationDeg is applied Rz(yaw) * Ry(pitch) * Rx(roll).
CadTransform stationPlacementFromPositionRotation(const std::array<double, 3>& positionMm,
                                                  const std::array<double, 3>& rotationDeg);
