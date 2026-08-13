#ifndef PHYSICSICONS_H
#define PHYSICSICONS_H

#include "LibraryCatalogue.h"
#include "PlacedMechanismSchema.h"

#include <QIcon>


// One per catalogue row, so a rail is identifiable in a list of eighteen packages without reading the
// Type column.
QIcon libraryCategoryIcon(librarycatalogue::Category category);

QIcon categoryIcon(placedmechanism::AxisKind kind);

// A conveyor, for the Add Conveyor action the menu and the toolbar share.
QIcon conveyorIcon();

QIcon deleteIcon();

#endif // PHYSICSICONS_H
