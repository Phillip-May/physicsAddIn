#include "ConveyorScenery.h"

#include "AccessoryGeometry.h"
#include "ConveyorGeometry.h"

#include <algorithm>
#include <utility>

namespace {

// One mesh child of the generated conveyor, as the builder expects to find it: it regenerates every
// vertex from the parameters, so only the node's identity and colour matter here.
CadNode* appendMesh(const std::shared_ptr<CadNode>& root, const char* name,
                    const CADNodeColor& color) {
    auto node = std::make_shared<CadNode>();
    node->name = name;
    node->type = CadNodeType::MeshGeometry;
    node->color = color;
    node->data = std::make_shared<MeshGeometryData>();
    node->parent = root.get();
    root->children.push_back(node);
    return node.get();
}

void appendTransform(const std::shared_ptr<CadNode>& root, const char* name) {
    auto node = std::make_shared<CadNode>();
    node->name = name;
    node->type = CadNodeType::Transform;
    node->data = std::make_shared<TransformNodeData>();
    node->parent = root.get();
    root->children.push_back(node);
}

} // namespace

std::shared_ptr<CadNode> conveyorPrototypeNode(const std::vector<CadVec3>& verticesMm,
                                               const std::vector<uint32_t>& indices) {
    auto node = std::make_shared<CadNode>();
    node->name = "Spawner product";
    node->type = CadNodeType::MeshGeometry;
    auto mesh = std::make_shared<MeshGeometryData>();
    mesh->vertices.reserve(verticesMm.size() * 3);
    for (const CadVec3& vertex : verticesMm) {
        mesh->vertices.push_back(static_cast<float>(vertex.x));
        mesh->vertices.push_back(static_cast<float>(vertex.y));
        mesh->vertices.push_back(static_cast<float>(vertex.z));
    }
    mesh->indices = indices;
    mesh->loaded = true;
    node->data = mesh;
    return node;
}

const CadTransform& conveyorAccessoryFrame() {
    // Maps an accessory-local (x along, y up, z across) onto a RoboDK frame's (x along, y across,
    // z up): itemX = accX, itemZ = accY, and itemY = itemZ x itemX = -accZ.
    static const CadTransform frame = [] {
        CadTransform transform;
        transform.values[0] = 1.0; transform.values[1] = 0.0;  transform.values[2] = 0.0;
        transform.values[4] = 0.0; transform.values[5] = 0.0;  transform.values[6] = -1.0;
        transform.values[8] = 0.0; transform.values[9] = 1.0;  transform.values[10] = 0.0;
        return transform;
    }();
    return frame;
}

ConveyorScenery buildConveyorScenery(const ConveyorSceneryRequest& request) {
    ConveyorScenery scenery;

    auto root = std::make_shared<CadNode>();
    root->name = "Conveyor";
    root->type = CadNodeType::Transform;
    auto parameters = std::make_shared<TransformNodeData>(request.parameters);
    root->data = parameters;

    // The five children the shared builder expects, in the order the accessory package has them.
    appendMesh(root, "Powder-coated frame", CADNodeColor(0.16f, 0.19f, 0.22f));
    appendMesh(root, "Adjustable feet", CADNodeColor(0.08f, 0.09f, 0.10f));
    appendMesh(root, "Steel rollers", CADNodeColor(0.68f, 0.71f, 0.73f));
    appendTransform(root, "Left end connection");
    appendTransform(root, "Right end connection");
    if (!rebuildRollerConveyor(root.get(), request.spawnPrototype)) return scenery;
    root->mountingHoles.placementSource = true;

    // The item's pose, and the axis convention between the two cells. No derivation, no anchoring on
    // the path's start point, and no sideways shift onto the load: the deck is drawn where the
    // conveyor stands, and the lane the products ride is the same expression through the same pose,
    // so the two cannot land in different places. There is nothing left for an `ends N mm off the
    // lane` figure to measure.
    static_cast<cadnodedraw::Scenery&>(scenery) =
        cadnodedraw::flatten(root.get(), request.world * conveyorAccessoryFrame());
    scenery.root = std::move(root);
    return scenery;
}
