#pragma once

#include "CadNode.h"


// Regenerates the roller conveyor's meshes from its own parameters: the frame, the feet, the
// rollers and their cover, the role enclosure over a spawner's or deleter's half of the deck, the
// deleter's X, and the cyan spawner product-and-infinity marking.
bool rebuildRollerConveyor(CadNode* root, const CadNode* spawnPrototype);

// Applies shared parameter bounds and inherited endpoint heights in place.
void clampRollerConveyorParameters(TransformNodeData& parameters);
