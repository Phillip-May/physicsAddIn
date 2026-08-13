#ifndef RODOKBRIDGE_H
#define RODOKBRIDGE_H

#include "CadNode.h"
#include "robodktypes.h"

#include <QString>

#include <cstdint>
#include <vector>

// Geometry and pose translation between a RoboDK station and the CadNode/PhysX world.
namespace rdkbridge {

struct MeshData {
    // Item-local millimetres, which is the space RoboDK exports in and the space PhysX shapes are
    // built in. The item's pose is applied to the body, never baked into these.
    std::vector<CadVec3> verticesMm;
    std::vector<uint32_t> indices;
    CadVec3 minimumMm;
    CadVec3 maximumMm;

    bool valid() const { return verticesMm.size() >= 4 && indices.size() >= 3; }
    CadVec3 halfExtentsMm() const;
    CadVec3 centerMm() const;
    size_t triangleCount() const { return indices.size() / 3; }
};

CadTransform toCadTransform(const Mat& pose);
Mat toRoboDkPose(const CadTransform& transform);

bool exportItemMesh(RoboDK* rdk, Item item, MeshData* out, QString* errorMessage);

// Both STL flavours, because which one RoboDK writes is its choice and not something the caller can
// specify. Exposed for the sake of being testable without a running RoboDK.
bool readStlFile(const QString& path, MeshData* out, QString* errorMessage);

std::vector<CadVec3> reducedHullPoints(const std::vector<CadVec3>& verticesMm,
                                       size_t maximum = 512);

} // namespace rdkbridge

#endif // RODOKBRIDGE_H
