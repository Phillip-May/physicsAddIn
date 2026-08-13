#ifndef LIBRARYPLACER_H
#define LIBRARYPLACER_H

#include "LibraryCatalogue.h"
#include "MountingSnap.h"
#include "PlacedItem.h"

#include <QString>

#include <memory>
#include <vector>

// A loaded catalogue entry ready for placement.
struct ArmedPackage {
    librarycatalogue::Entry entry;
    std::shared_ptr<CadNode> tree;
    // Parametric conveyors use the plugin's existing conveyor path.
    bool asConveyor = false;
    TransformNodeData conveyorParameters;
    std::vector<mountingsnap::Interface> sourceInterfaces;
    QString refusal;

    bool armed() const { return tree != nullptr && refusal.isEmpty() && !sourceInterfaces.empty(); }
    CadTransform treeFrame() const;
};

ArmedPackage armLibraryEntry(const librarycatalogue::Entry& entry);

// Package may be a file name, path, or bare stem. An empty variant selects the base package.
const librarycatalogue::Entry* findLibraryEntry(const std::vector<librarycatalogue::Entry>& entries,
                                               const QString& package, const QString& variant);

// Returns an empty string when the dock may offer the entry.
QString dockRefusal(const librarycatalogue::Entry& entry);

void collectNodeTargets(CadNode* root, const CadTransform& toWorld,
                        std::vector<mountingsnap::Interface>* targets);

void collectPlacedItemTargets(const PlacedItem& placed, std::vector<mountingsnap::Interface>* targets);

// Deterministic mating for headless RoboDK, where no camera is available. Named interfaces win;
// remaining ties prefer the placement with the greatest separation.
struct CursorlessMate {
    bool found = false;
    CadTransform worldPose;
    int matchedHoles = 0;
    int requiredHoles = 0;
    double worstErrorMm = 0.0;
    int quarterTurn = 0;
    QString sourceId;
    QString targetId;
    double apartMm = 0.0;
};

CursorlessMate mateWithoutACursor(const std::vector<mountingsnap::Interface>& sources,
                                  const std::vector<mountingsnap::Interface>& targets,
                                  const QString& sourceId, const QString& targetId);

#endif // LIBRARYPLACER_H
