#pragma once

#include "CadNode.h"

// conveyorcore::Host over g_scene, and the one Runtime the station's conveyors advance on.
void resetSceneConveyorCore();
void addSceneConveyorSpawner(CadNode* conveyor);

// Rules 1-13 and 15 over the scene. The physics step and the body-pose writeback are the caller's,
// because they are not rules - they are what a PhysX host does around them.
void stepSceneConveyorRules(double seconds);
