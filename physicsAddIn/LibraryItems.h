#ifndef LIBRARYITEMS_H
#define LIBRARYITEMS_H

#include "PlacedItem.h"

#include "JsonCompat.h"

#include <QByteArray>
#include <QString>

#include <vector>

extern const char* const kLibraryItemParam;

struct LibraryItemSpec {
    // Bare names resolve under the library root; paths are used directly.
    QString packageRef;
    QString variantId;
    Json parameters = Json::object();
    CadTransform poseLocal;
    bool hasPose = false;
    // Robot axes are degrees; rail position is millimetres.
    std::vector<double> axes;
};

struct PlacedLibraryItem : PlacedItem {
    LibraryItemSpec spec;
    // Cached serialized form, used to avoid no-op station edits.
    QByteArray blob;
};

Item createLibraryItem(RoboDK* rdk, const QString& name, QString* error);

Item createRailMechanismItem(RoboDK* rdk, const QString& name, double lowerMm, double upperMm,
                             double homeMm, QString* error);

// Returns the <name> Base frame created by BuildMechanism.
Item railMechanismBaseFrame(RoboDK* rdk, Item mechanism);

bool placedItemIsRailMechanism(RoboDK* rdk, Item item);

bool libraryItemSpecFromItem(RoboDK* rdk, Item item, LibraryItemSpec* out, QString* error);

// Returns true only when the item parameter changed.
bool writeLibraryItemSpec(RoboDK* rdk, Item item, const LibraryItemSpec& spec, QByteArray* blob);

std::shared_ptr<CadNode> loadLibraryItemTree(const LibraryItemSpec& spec, const QString& libraryRoot,
                                             QString* error);

void bindPlacedItemAxes(PlacedLibraryItem& placed, QString* warning = nullptr);

#endif // LIBRARYITEMS_H
