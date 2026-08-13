#include "CadNodeDraw.h"

#include <cmath>

namespace cadnodedraw {
namespace {

bool isIdentity(const CadTransform& transform) {
    static const CadTransform identity;
    return transform.values == identity.values;
}

// Emits one mesh instance at one placement.
void emit(const MeshGeometryData& mesh, const CADNodeColor& colour, const CadTransform& placement,
          std::vector<DrawGroup>& groups) {
    if (mesh.indices.size() < 3) return;
    DrawGroup group;
    group.rgba[0] = colour.r;
    group.rgba[1] = colour.g;
    group.rgba[2] = colour.b;
    group.rgba[3] = colour.a;
    group.verticesMm.reserve(mesh.indices.size() * 3);
    group.normals.reserve(mesh.indices.size() * 3);
    for (size_t face = 0; face + 2 < mesh.indices.size(); face += 3) {
        CadVec3 corner[3];
        bool complete = true;
        for (int vertex = 0; vertex < 3; ++vertex) {
            const size_t at = static_cast<size_t>(mesh.indices[face + vertex]) * 3;
            if (at + 2 >= mesh.vertices.size()) { complete = false; break; }
            corner[vertex] = placement * CadVec3(mesh.vertices[at], mesh.vertices[at + 1],
                                                mesh.vertices[at + 2]);
        }
        if (!complete) continue;
        // Flat per-face normals. The mesh carries none - it is a vertex and index buffer -
        // and generated geometry is boxes and cylinder facets, which faceted shading suits.
        CadVec3 normal = cross(difference(corner[1], corner[0]),
                               difference(corner[2], corner[0]));
        const double magnitude = lengthOf(normal);
        normal = magnitude > 1.0e-9 ? scaled(normal, 1.0 / magnitude) : CadVec3(0.0, 0.0, 1.0);
        for (int vertex = 0; vertex < 3; ++vertex) {
            group.verticesMm.push_back(static_cast<float>(corner[vertex].x));
            group.verticesMm.push_back(static_cast<float>(corner[vertex].y));
            group.verticesMm.push_back(static_cast<float>(corner[vertex].z));
            group.normals.push_back(static_cast<float>(normal.x));
            group.normals.push_back(static_cast<float>(normal.y));
            group.normals.push_back(static_cast<float>(normal.z));
        }
        ++group.triangles;
    }
    if (group.triangles > 0) groups.push_back(std::move(group));
}

void collect(const CadNode* node, const CadTransform& placement, std::vector<DrawGroup>& groups) {
    if (!node) return;
    // Hidden prototype nodes provide geometry but are not themselves drawn.
    if (!node->visible) return;
    if (const MeshGeometryData* mesh = node->asMeshGeometry()) {
        emit(*mesh, node->color, placement, groups);
    } else if (const DragChainMechanismData* chain = node->asDragChainMechanism()) {
        // Draw the canonical chain member at each solved link frame.
        const CadNode* prototypeNode = chain->prototypeGeometry;
        const MeshGeometryData* prototype =
            prototypeNode ? prototypeNode->asMeshGeometry() : nullptr;
        if (prototype) {
            for (const CadNode* frame : chain->linkFrames) {
                if (!frame || !frame->visible) continue;
                emit(*prototype, prototypeNode->color, placement * frame->loc, groups);
            }
        }
    }
    for (const std::shared_ptr<CadNode>& child : node->children) {
        if (!child) continue;
        collect(child.get(), isIdentity(child->loc) ? placement : placement * child->loc, groups);
    }
}

} // namespace

Scenery flatten(const CadNode* root, const CadTransform& placement) {
    Scenery scenery;
    collect(root, placement, scenery.groups);
    for (const DrawGroup& group : scenery.groups) scenery.triangles += group.triangles;
    return scenery;
}

} // namespace cadnodedraw
