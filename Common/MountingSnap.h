#ifndef MOUNTINGSNAP_H
#define MOUNTINGSNAP_H

#include "CadNode.h"

#include <map>
#include <string>
#include <vector>

namespace mountingsnap {

struct View {
    virtual bool project(const CadVec3& worldMm, double* pixelX, double* pixelY) const = 0;
    // Pixel coordinates use a top-left origin; direction is normalized.
    virtual bool rayThrough(double pixelX, double pixelY, CadVec3* originMm,
                            CadVec3* direction) const = 0;
    virtual ~View() = default;
};

struct Interface {
    CadNode* node = nullptr;
    CadTransform transform;
    std::vector<CadVec3> pointsMm;
    // Connection ends face each other; mounting surfaces align in the same direction.
    bool mateOpposite = false;
    std::string interfaceId;
    std::map<std::string, std::string> parameterBindings;
};

void collectInterfaces(const CadNode* node, const CadTransform& parentTransform,
                       bool placementSourcesOnly, std::vector<Interface>* interfaces);

// Treats root as the placement origin, excluding its existing scene transform.
void collectLocalInterfaces(const CadNode* root, bool placementSourcesOnly,
                            std::vector<Interface>* interfaces);

void collectPlacementSources(const CadNode* root, std::vector<Interface>* interfaces);

struct TargetFilter {
    virtual bool eligible(const CadNode* node, bool insideRobot) const = 0;
    virtual ~TargetFilter() = default;
};

// Walks eligible, visible targets in world millimetres and prunes the excluded subtree.
void collectTargets(CadNode* node, const CadTransform& parentTransform, bool insideRobot,
                    const CadNode* exclude, const TargetFilter& filter,
                    std::vector<Interface>* targets);

constexpr double kFloorGridPitchMm = 100.0;
constexpr double kFloorGridHalfExtentMm = 1500.0;

// Returns a finite, lattice-aligned window over the unbounded floor grid.
MountingHoleGridData floorGridWindow(const CadVec3& centreMm,
                                     double halfExtentMm = kFloorGridHalfExtentMm,
                                     double pitchMm = kFloorGridPitchMm);

bool recentreFloorGrid(CadNode* floor, const CadTransform& floorToWorld, const View& view,
                       double cursorX, double cursorY,
                       double halfExtentMm = kFloorGridHalfExtentMm,
                       double pitchMm = kFloorGridPitchMm);

int rotationQuarterStep(const Interface& interface);

struct Mate {
    CadTransform worldPose;
    int matchedHoles = 0;
    int requiredHoles = 0;
    bool complete = false;
    double worstErrorMm = 0.0;
};

Mate mate(const Interface& source, const Interface& target, int quarterTurn);

struct Request {
    const std::vector<Interface>* sourceInterfaces = nullptr;
    const std::vector<Interface>* targets = nullptr;
    // Floor grids permit single-point placement; other targets require a complete pattern.
    const CadNode* floor = nullptr;
    bool toolPackage = false;
    double cursorX = 0.0, cursorY = 0.0;
    int viewportWidthPx = 0, viewportHeightPx = 0;
    // Percentage of the shorter viewport dimension.
    double snapScreenPercent = 1.5;
    int quarterTurn = 0;
    // Retains the active mate while cycling through rotational symmetry.
    bool retainSnapAfterRotation = false;
    const CadNode* retainedTarget = nullptr;
    int retainedSourceInterface = -1;
};

struct Result {
    CadTransform worldPose;
    bool snapped = false;
    int matchedHoles = 0;
    int requiredHoles = 0;
    const CadNode* targetNode = nullptr;
    int sourceInterface = -1;
    // Local guide subset in world millimetres.
    std::vector<CadVec3> sourceGuidesMm, targetGuidesMm;
    std::string status;
};

Result solve(const Request& request, const View& view);

} // namespace mountingsnap

#endif // MOUNTINGSNAP_H
