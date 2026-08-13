#ifndef CADNODEDRAW_H
#define CADNODEDRAW_H

#include "CadNode.h"

#include <vector>

// A CadNode tree, flattened into triangle soup a host can hand straight to a draw call.
namespace cadnodedraw {

// One mesh's triangles, in whatever frame `flatten` was given.
struct DrawGroup {
    // Three floats per vertex and nine per triangle, in millimetres. Indexed meshes are expanded
    // because the flat normals below are per face, and because a draw call for painted geometry has
    // no index buffer.
    std::vector<float> verticesMm;
    std::vector<float> normals;
    int triangles = 0;
    float rgba[4] = {1.0f, 1.0f, 1.0f, 1.0f};
};

struct Scenery {
    std::vector<DrawGroup> groups;
    int triangles = 0;
};

Scenery flatten(const CadNode* root, const CadTransform& placement);

} // namespace cadnodedraw

#endif // CADNODEDRAW_H
