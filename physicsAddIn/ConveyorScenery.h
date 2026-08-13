#ifndef CONVEYORSCENERY_H
#define CONVEYORSCENERY_H

#include "CadNode.h"
#include "CadNodeDraw.h"

#include <string>
#include <vector>

using ConveyorDrawGroup = cadnodedraw::DrawGroup;

struct ConveyorSceneryRequest {
    TransformNodeData parameters;
    // Station-space placement used while baking the draw buffers.
    CadTransform world;
    const CadNode* spawnPrototype = nullptr;
};

struct ConveyorScenery : cadnodedraw::Scenery {
    // Retained for mounting interfaces and later re-bakes.
    std::shared_ptr<CadNode> root;
};

ConveyorScenery buildConveyorScenery(const ConveyorSceneryRequest& request);

// Converts the generated conveyor's Y-up frame to RoboDK's Z-up frame.
const CadTransform& conveyorAccessoryFrame();

std::shared_ptr<CadNode> conveyorPrototypeNode(const std::vector<CadVec3>& verticesMm,
                                               const std::vector<uint32_t>& indices);

#endif // CONVEYORSCENERY_H
