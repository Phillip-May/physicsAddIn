#include "PlacedItem.h"

#include "RoboDkBridge.h"

#include "iitem.h"
#include "irobodk.h"

CadTransform placedItemParentWorld(RoboDK* rdk, const PlacedItem& placed) {
    CadTransform parent;
    if (!placed.item || (rdk && !rdk->Valid(placed.item))) return parent;
    Item above = placed.item->Parent();
    // A rail stands on a RoboDK mechanism, and `BuildMechanism` gives a mechanism a base frame of its own
    // and parents it to that. So for a rail the item's parent is *its own placement* - the frame the
    // plugin poses to `poseLocal * railBaseInTree()` - and composing the stored placement onto it again
    // would place the rail by its own offset twice. What the placement is measured against is the frame
    // above that one.
    if (above && placed.item->Type() == IItem::ITEM_TYPE_ROBOT &&
        (!rdk || rdk->Valid(above)) && above->Type() == IItem::ITEM_TYPE_FRAME) {
        above = above->Parent();
    }
    if (above && (!rdk || rdk->Valid(above))) {
        parent = rdkbridge::toCadTransform(above->PoseAbs());
    }
    return parent;
}

bool placedItemStand(RoboDK* rdk, const PlacedItem& placed, CadTransform* world) {
    if (!placed.item || !world) return false; // the station owns placement here
    if (rdk && !rdk->Valid(placed.item)) return false;
    // The stored placement, composed with wherever its item hangs in the tree. Not the node's
    // `PoseAbs()`, which a generic item answers as identity however its pose is set.
    *world = placedItemParentWorld(rdk, placed) * placed.poseLocal;
    return true;
}

bool placedItemHasDrifted(const PlacedItem& placed, const CadTransform& stand) {
    for (int cell = 0; cell < 12; ++cell) {
        if (stand.values[cell] != placed.sceneryAt.values[cell]) return true;
    }
    return false;
}
