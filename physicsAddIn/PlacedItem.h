#ifndef PLACEDITEM_H
#define PLACEDITEM_H

#include "ConveyorScenery.h"
#include "robodktypes.h"

#include <QString>

#include <memory>

class PlacedItemAxes;

// Geometry owned and drawn by the plugin. The RoboDK item stores identity and placement only.
struct PlacedItem {
    Item item = nullptr;
    QString name;
    // Relative to the item's RoboDK parent.
    CadTransform poseLocal;
    bool hasPose = false;
    // Baked in station coordinates.
    ConveyorScenery scenery;
    CadTransform sceneryAt;
    CadVec3 sceneryMinMm;
    CadVec3 sceneryMaxMm;
    CadTransform treeToWorld;
    // Present for robots and rails.
    std::shared_ptr<PlacedItemAxes> axes;
};

CadTransform placedItemParentWorld(RoboDK* rdk, const PlacedItem& placed);

// Resolves the local placement into station coordinates.
bool placedItemStand(RoboDK* rdk, const PlacedItem& placed, CadTransform* world);

bool placedItemHasDrifted(const PlacedItem& placed, const CadTransform& stand);

#endif // PLACEDITEM_H
