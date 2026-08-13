#include "AccessoryBuilders.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "AccessoryGeometry.h"
#include "ConveyorGeometry.h"
#include "MountingSnap.h"
#include "StringUtil.h"

bool rebuildStationRollerConveyor(CadNode* root, const StationDocument& station) {
    const TransformNodeData* parameters = root ? root->asTransform() : nullptr;
    if (!parameters) return false;
    // Resolving the spawner's prototype is the only thing the conveyor builder ever needed a
    // station for, so it is done here and the node handed over.
    const CadNode* spawnPrototype = nullptr;
    if (!parameters->accessorySpawnObjectId.empty()) {
        for (const StationAccessoryInstance& accessory : station.accessories) {
            if (accessory.id == parameters->accessorySpawnObjectId && accessory.hidden &&
                accessory.node) {
                spawnPrototype = accessory.node;
                break;
            }
        }
    }
    return rebuildRollerConveyor(root, spawnPrototype);
}

bool rebuildAr4Table(CadNode* root) {
    TransformNodeData* parameters = root ? root->asTransform() : nullptr;
    if (!parameters || parameters->accessoryGenerator != "ar4_table" ||
        root->children.size() < 3) {
        return false;
    }
    MeshGeometryData* frame = root->children[0]->asMeshGeometry();
    MeshGeometryData* feet = root->children[1]->asMeshGeometry();
    MeshGeometryData* top = root->children[2]->asMeshGeometry();
    if (!frame || !feet || !top) return false;

    parameters->accessoryWidthMm =
        std::max(400.0, std::min(3000.0, parameters->accessoryWidthMm));
    parameters->accessoryLengthMm =
        std::max(400.0, std::min(3000.0, parameters->accessoryLengthMm));
    parameters->accessoryHeightMm =
        std::max(300.0, std::min(2000.0, parameters->accessoryHeightMm));
    parameters->accessoryHolePitchMm =
        std::max(50.0, std::min(300.0, parameters->accessoryHolePitchMm));
    const float width = static_cast<float>(parameters->accessoryWidthMm);
    const float length = static_cast<float>(parameters->accessoryLengthMm);
    const float height = static_cast<float>(parameters->accessoryHeightMm);
    const float pitch = static_cast<float>(parameters->accessoryHolePitchMm);
    const float legX = length * 0.5f - 70.0f;
    const float legZ = width * 0.5f - 70.0f;

    frame->vertices.clear(); frame->indices.clear();
    feet->vertices.clear(); feet->indices.clear();
    top->vertices.clear(); top->indices.clear();
    for (float x : {-legX, legX}) {
        for (float z : {-legZ, legZ}) {
            accessoryAppendBox(*frame, x, (height - 40.0f) * 0.5f + 10.0f, z,
                               60.0f, height - 60.0f, 60.0f);
            accessoryAppendBox(*feet, x, 5.0f, z, 120.0f, 10.0f, 120.0f);
        }
    }
    for (float y : {120.0f, height - 130.0f}) {
        for (float z : {-legZ, legZ}) {
            accessoryAppendBox(*frame, 0.0f, y, z, length - 80.0f, 45.0f, 45.0f);
        }
        for (float x : {-legX, legX}) {
            accessoryAppendBox(*frame, x, y, 0.0f, 45.0f, 45.0f, width - 80.0f);
        }
    }

    const int xCount = accessoryHoleCount(length, pitch);
    const int zCount = accessoryHoleCount(width, pitch);
    const float firstX = -0.5f * static_cast<float>(xCount - 1) * pitch;
    const float firstZ = -0.5f * static_cast<float>(zCount - 1) * pitch;
    for (int x = 0; x < xCount; ++x) {
        for (int z = 0; z < zCount; ++z) {
            accessoryAppendPerforatedCell(*top, firstX + x * pitch, firstZ + z * pitch,
                                          height - 20.0f, height, pitch, 7.0f);
        }
    }
    const float activeLength = xCount * pitch;
    const float activeWidth = zCount * pitch;
    const float borderX = std::max(0.0f, (length - activeLength) * 0.5f);
    const float borderZ = std::max(0.0f, (width - activeWidth) * 0.5f);
    if (borderX > 0.0f) {
        accessoryAppendBox(*top, -(activeLength + borderX) * 0.5f, height - 10.0f, 0.0f,
                           borderX, 20.0f, width);
        accessoryAppendBox(*top, (activeLength + borderX) * 0.5f, height - 10.0f, 0.0f,
                           borderX, 20.0f, width);
    }
    if (borderZ > 0.0f) {
        accessoryAppendBox(*top, 0.0f, height - 10.0f,
                           -(activeWidth + borderZ) * 0.5f,
                           activeLength, 20.0f, borderZ);
        accessoryAppendBox(*top, 0.0f, height - 10.0f,
                           (activeWidth + borderZ) * 0.5f,
                           activeLength, 20.0f, borderZ);
    }
    accessoryFinalizeMesh(*frame);
    accessoryFinalizeMesh(*feet);
    accessoryFinalizeMesh(*top);

    root->mountingHoles.pointsMm.clear();
    root->mountingHoles.grids.clear();
    MountingHoleGridData feetGrid;
    feetGrid.originMm = CadVec3(-legX, 0.0, -legZ);
    feetGrid.uStepMm = CadVec3(legX * 2.0, 0.0, 0.0);
    feetGrid.vStepMm = CadVec3(0.0, 0.0, legZ * 2.0);
    feetGrid.uCount = 2;
    feetGrid.vCount = 2;
    root->mountingHoles.grids.push_back(feetGrid);

    CadNode* topNode = root->children[2].get();
    topNode->mountingHoles.pointsMm.clear();
    topNode->mountingHoles.grids.clear();
    MountingHoleGridData topGrid;
    topGrid.originMm = CadVec3(firstX, height, firstZ);
    topGrid.uStepMm = CadVec3(pitch, 0.0, 0.0);
    topGrid.vStepMm = CadVec3(0.0, 0.0, pitch);
    topGrid.uCount = static_cast<uint32_t>(xCount);
    topGrid.vCount = static_cast<uint32_t>(zCount);
    topNode->mountingHoles.grids.push_back(topGrid);
    const int roundedPitch = static_cast<int>(std::lround(pitch));
    root->name = "AR4 Table — " + std::to_string(roundedPitch) + " mm pitch";
    topNode->name = std::to_string(roundedPitch) + " mm pitch perforated steel top";
    return true;
}

bool rebuildParametricAccessory(CadNode* root, const StationDocument& station) {
    return rebuildAr4Table(root) || rebuildStationRollerConveyor(root, station);
}

void rebuildParametricAccessories(CadNode* node, const StationDocument& station) {
    if (!node) return;
    rebuildParametricAccessory(node, station);
    for (const std::shared_ptr<CadNode>& child : node->children) {
        rebuildParametricAccessories(child.get(), station);
    }
}

void includeSceneVisualMinY(const CadNode* node, const CadTransform& parentTransform,
                            bool parentVisible, double& minimumY, double& maximumAbsXZ,
                            bool& foundVisual) {
    if (!node) return;
    const bool visible = parentVisible && node->visible;
    const CadTransform world = parentTransform * node->loc;
    if (visible) {
        if (const MeshGeometryData* mesh = node->asMeshGeometry()) {
            if (mesh->loaded || !mesh->vertices.empty()) {
                const float xs[2] = {mesh->bounds[0], mesh->bounds[3]};
                const float ys[2] = {mesh->bounds[1], mesh->bounds[4]};
                const float zs[2] = {mesh->bounds[2], mesh->bounds[5]};
                for (float x : xs) {
                    for (float y : ys) {
                        for (float z : zs) {
                            const CadVec3 corner = world * CadVec3(x, y, z);
                            minimumY = foundVisual ? std::min(minimumY, corner.y) : corner.y;
                            maximumAbsXZ = std::max(
                                maximumAbsXZ, std::max(std::abs(corner.x), std::abs(corner.z)));
                            foundVisual = true;
                        }
                    }
                }
            }
        }
    }
    for (const std::shared_ptr<CadNode>& child : node->children) {
        includeSceneVisualMinY(child.get(), world, visible, minimumY, maximumAbsXZ,
                               foundVisual);
    }
}

std::shared_ptr<CadNode> makeDefaultFloor(double topY, double requestedHalfExtentMm) {
    const float halfExtentMm = static_cast<float>(
        std::max(2000.0, std::min(25000.0, requestedHalfExtentMm)));
    constexpr float kThicknessMm = 20.0f;

    auto floor = std::make_shared<CadNode>();
    floor->name = "Floor";
    floor->type = CadNodeType::Custom;
    floor->loc.values[7] = topY;
    floor->mountingHoles.comment =
        "Built-in 100 mm floor placement grid; generated at runtime and not serialized.";
    // A window onto the lattice, moved to the cursor by `recentreFloorGrid` before each placement solve -
    // not a sheet spanning the cell. The visual below still covers the whole floor, because that is what an
    // operator wants to see; the *grid* only ever needs to be around what is being placed, and a sheet at
    // this half-extent was up to 501 by 501 points every one of which the snap search projected on every
    // mouse move.
    floor->mountingHoles.grids.push_back(mountingsnap::floorGridWindow(CadVec3()));

    auto visual = std::make_shared<CadNode>();
    visual->name = "Floor visual";
    visual->type = CadNodeType::MeshGeometry;
    visual->color = CADNodeColor::fromSRGB(96, 103, 109);
    visual->parent = floor.get();
    auto mesh = std::make_shared<MeshGeometryData>();
    mesh->meshSource = "builtin://floor-box";
    mesh->vertices = {
        -halfExtentMm, -kThicknessMm, -halfExtentMm,
         halfExtentMm, -kThicknessMm, -halfExtentMm,
         halfExtentMm,  0.0f,         -halfExtentMm,
        -halfExtentMm,  0.0f,         -halfExtentMm,
        -halfExtentMm, -kThicknessMm,  halfExtentMm,
         halfExtentMm, -kThicknessMm,  halfExtentMm,
         halfExtentMm,  0.0f,          halfExtentMm,
        -halfExtentMm,  0.0f,          halfExtentMm,
    };
    mesh->indices = {
        0, 1, 5, 0, 5, 4,
        3, 7, 6, 3, 6, 2,
        0, 3, 2, 0, 2, 1,
        4, 5, 6, 4, 6, 7,
        0, 4, 7, 0, 7, 3,
        1, 2, 6, 1, 6, 5,
    };
    mesh->bounds = {{-halfExtentMm, -kThicknessMm, -halfExtentMm,
                      halfExtentMm, 0.0f, halfExtentMm}};
    mesh->loaded = true;
    visual->data = mesh;
    floor->children.push_back(visual);
    return floor;
}

CadNode* appendDefaultFloor(const std::shared_ptr<CadNode>& root) {
    if (!root) return nullptr;
    CadTransform identity;
    double minimumY = 0.0;
    double maximumAbsXZ = 0.0;
    bool foundVisual = false;
    includeSceneVisualMinY(root.get(), identity, true, minimumY, maximumAbsXZ, foundVisual);
    std::shared_ptr<CadNode> floor = makeDefaultFloor(
        foundVisual ? minimumY : 0.0, foundVisual ? maximumAbsXZ + 500.0 : 2000.0);
    floor->parent = root.get();
    root->children.insert(root->children.begin(), floor);
    return floor.get();
}

