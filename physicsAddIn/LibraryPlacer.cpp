#include "LibraryPlacer.h"

#include "ConveyorScenery.h"

#include <QFileInfo>

#include <cmath>

namespace {

// Every pattern a placed item declares is a target. What is host-specific about eligibility is that the
// plug-in's targets are its own placed items - it never offers RoboDK's own objects, which carry no
// mounting metadata - and that choice is made by which trees are walked, not by this.
struct EveryPattern final : mountingsnap::TargetFilter {
    bool eligible(const CadNode*, bool) const override { return true; }
};

CadVec3 centreOf(const std::vector<CadVec3>& points) {
    CadVec3 centre;
    if (points.empty()) return centre;
    for (const CadVec3& point : points) {
        centre = CadVec3(centre.x + point.x, centre.y + point.y, centre.z + point.z);
    }
    const double count = static_cast<double>(points.size());
    return CadVec3(centre.x / count, centre.y / count, centre.z / count);
}

// The pattern a free drag follows: an object's own placement holes, which is the unnamed
// non-directional one. Null when it declares none, and then there is nothing to measure apartness by.
const mountingsnap::Interface* placementPatternOf(
    const std::vector<mountingsnap::Interface>& interfaces) {
    for (const mountingsnap::Interface& entry : interfaces) {
        if (!entry.mateOpposite && entry.interfaceId.empty()) return &entry;
    }
    return nullptr;
}

bool declaresPlacementPattern(const CadNode* tree) {
    std::vector<mountingsnap::Interface> sources;
    mountingsnap::collectPlacementSources(tree, &sources);
    return !sources.empty();
}

QString noPlacementPatternRefusal(const librarycatalogue::Entry& entry) {
    return QStringLiteral("'%1' declares no mounting pattern to be placed by.")
        .arg(QString::fromStdString(entry.name));
}

} // namespace

CadTransform ArmedPackage::treeFrame() const {
    return asConveyor ? conveyorAccessoryFrame() : CadTransform();
}

ArmedPackage armLibraryEntry(const librarycatalogue::Entry& entry) {
    ArmedPackage armed;
    armed.entry = entry;
    if (!entry.resolves || !entry.root) {
        armed.refusal = entry.error.empty()
            ? QStringLiteral("'%1' does not load.").arg(QString::fromStdString(entry.id))
            : QString::fromStdString(entry.error);
        return armed;
    }
    const TransformNodeData* parameters = entry.root->asTransform();
    armed.asConveyor = parameters && parameters->hasParametricAccessory();
    if (armed.asConveyor) {
        armed.conveyorParameters = *parameters;
        ConveyorSceneryRequest request;
        request.parameters = armed.conveyorParameters;
        armed.tree = buildConveyorScenery(request).root;
        if (!armed.tree) {
            armed.refusal = QStringLiteral("'%1' describes no conveyor geometry.")
                                .arg(QString::fromStdString(entry.name));
            return armed;
        }
    } else {
        armed.tree = entry.root;
    }
    // Collect only interfaces that may place the package root.
    mountingsnap::collectPlacementSources(armed.tree.get(), &armed.sourceInterfaces);
    if (armed.sourceInterfaces.empty()) armed.refusal = noPlacementPatternRefusal(entry);
    return armed;
}

const librarycatalogue::Entry* findLibraryEntry(const std::vector<librarycatalogue::Entry>& entries,
                                               const QString& package, const QString& variant) {
    const QString wantedFile = QFileInfo(package).fileName();
    const QString wantedStem = QFileInfo(package).completeBaseName();
    for (const librarycatalogue::Entry& entry : entries) {
        const QString path = QString::fromStdString(entry.path);
        if (QFileInfo(path).fileName() != wantedFile &&
            QString::fromStdString(entry.id) != wantedStem) {
            continue;
        }
        const QString id = QString::fromStdString(entry.variantId);
        if (variant.isEmpty() ? id == QLatin1String("default") : id == variant) return &entry;
    }
    return nullptr;
}

QString dockRefusal(const librarycatalogue::Entry& entry) {
    if (!entry.resolves || !entry.root) {
        return entry.error.empty() ? QStringLiteral("does not load") : QString::fromStdString(entry.error);
    }
    const TransformNodeData* parameters = entry.root->asTransform();
    if (parameters && parameters->hasParametricAccessory()) return QString();
    if (!declaresPlacementPattern(entry.root.get())) {
        return QStringLiteral("declares no mounting pattern to be placed by - place it at a stated pose "
                              "instead");
    }
    return QString();
}

void collectNodeTargets(CadNode* root, const CadTransform& toWorld,
                        std::vector<mountingsnap::Interface>* targets) {
    if (!root || !targets) return;
    const EveryPattern everything;
    mountingsnap::collectTargets(root, toWorld, /*insideRobot=*/false, nullptr, everything, targets);
}

void collectPlacedItemTargets(const PlacedItem& placed, std::vector<mountingsnap::Interface>* targets) {
    if (!placed.scenery.root) return;
    collectNodeTargets(placed.scenery.root.get(), placed.treeToWorld, targets);
}

namespace {

// Which of several complete mates is the one a cursor would have produced, in order. First wins any
// remaining tie, which is collection order and therefore the same on every run.
struct Rank {
    // A declared connection - `start`, `end` - over a general mounting surface. "Join this to that"
    // means the connection, and a foot pattern meeting another foot pattern is two objects in one place.
    bool namedPair = false;
    // The wheel left where it was. Every quarter turn of a symmetric pattern satisfies every hole, and
    // the ones that are not zero are the object rolled about the joint - upside down, for a conveyor.
    // Nothing asked for the wheel to be turned, so nothing turns it.
    int quarterTurn = 0;
    double apartMm = 0.0;
    double worstErrorMm = 0.0;

    bool betterThan(const Rank& other) const {
        if (namedPair != other.namedPair) return namedPair;
        if (quarterTurn != other.quarterTurn) return quarterTurn < other.quarterTurn;
        if (std::abs(apartMm - other.apartMm) > 1.0e-6) return apartMm > other.apartMm;
        return worstErrorMm < other.worstErrorMm;
    }
};

} // namespace

CursorlessMate mateWithoutACursor(const std::vector<mountingsnap::Interface>& sources,
                                  const std::vector<mountingsnap::Interface>& targets,
                                  const QString& sourceId, const QString& targetId) {
    CursorlessMate best;
    const mountingsnap::Interface* sourcePlacement = placementPatternOf(sources);
    const mountingsnap::Interface* targetPlacement = placementPatternOf(targets);
    const CadVec3 targetStands = targetPlacement ? centreOf(targetPlacement->pointsMm) : CadVec3();
    Rank bestRank;

    for (const mountingsnap::Interface& source : sources) {
        if (!sourceId.isEmpty() && QString::fromStdString(source.interfaceId) != sourceId) continue;
        for (const mountingsnap::Interface& target : targets) {
            if (!targetId.isEmpty() && QString::fromStdString(target.interfaceId) != targetId) continue;
            // A directional end mates only another directional end, and an ordinary pattern only an
            // ordinary one. Without this a leg or a floor grid wins on one coincident point at a seam.
            if (source.mateOpposite != target.mateOpposite) continue;
            for (int quarterTurn = 0; quarterTurn < 4; ++quarterTurn) {
                const mountingsnap::Mate mate = mountingsnap::mate(source, target, quarterTurn);
                if (!mate.complete) continue;
                Rank rank;
                rank.namedPair = !source.interfaceId.empty() && !target.interfaceId.empty();
                rank.quarterTurn = quarterTurn;
                rank.worstErrorMm = mate.worstErrorMm;
                if (sourcePlacement && targetPlacement) {
                    const CadVec3 stood = mate.worldPose * centreOf(sourcePlacement->pointsMm);
                    rank.apartMm = lengthOf(CadVec3(stood.x - targetStands.x, stood.y - targetStands.y,
                                                    stood.z - targetStands.z));
                }
                if (best.found && !rank.betterThan(bestRank)) continue;
                bestRank = rank;
                best.found = true;
                best.worldPose = mate.worldPose;
                best.matchedHoles = mate.matchedHoles;
                best.requiredHoles = mate.requiredHoles;
                best.worstErrorMm = mate.worstErrorMm;
                best.quarterTurn = quarterTurn;
                best.sourceId = QString::fromStdString(source.interfaceId);
                best.targetId = QString::fromStdString(target.interfaceId);
                best.apartMm = rank.apartMm;
            }
        }
    }
    return best;
}
