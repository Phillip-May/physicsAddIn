// Does a placement land where its mounting holes say it does?

#include "MountingSnap.cpp"

#include "PlacementSession.cpp"

#include "ViewRay.cpp"
#include "ConveyorScenery.cpp"
#include "AccessoryGeometry.cpp"
#include "AccessoryPropertySchema.cpp"
#include "CadNodeDraw.cpp"
#include "ConveyorGeometry.cpp"

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace {

int g_failures = 0;

void check(bool condition, const std::string& what) {
    if (condition) return;
    std::printf("  FAIL: %s\n", what.c_str());
    ++g_failures;
}

CadVec3 minus(const CadVec3& a, const CadVec3& b) {
    return CadVec3(a.x - b.x, a.y - b.y, a.z - b.z);
}

double dotp(const CadVec3& a, const CadVec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

CadVec3 middleOf(const std::vector<CadVec3>& points) {
    CadVec3 centre;
    for (const CadVec3& point : points) {
        centre = CadVec3(centre.x + point.x, centre.y + point.y, centre.z + point.z);
    }
    const double count = static_cast<double>(points.size());
    return CadVec3(centre.x / count, centre.y / count, centre.z / count);
}

struct RayCamera final : mountingsnap::View {
    viewray::Camera camera;

    bool project(const CadVec3& worldMm, double* pixelX, double* pixelY) const override {
        return viewray::projectPoint(camera, worldMm, pixelX, pixelY);
    }
    bool rayThrough(double pixelX, double pixelY, CadVec3* originMm,
                    CadVec3* direction) const override {
        const viewray::Ray ray = viewray::rayThroughPixel(camera, pixelX, pixelY);
        *originMm = ray.origin;
        *direction = ray.direction;
        return true;
    }
};

// Columns are the camera's own axes, which is what `viewray` reads a world-from-camera pose as.
CadTransform lookAt(const CadVec3& eye, const CadVec3& at) {
    CadVec3 forward = minus(at, eye);
    forward = CadVec3(forward.x / lengthOf(forward), forward.y / lengthOf(forward),
                      forward.z / lengthOf(forward));
    const CadVec3 worldUp(0.0, 1.0, 0.0);
    CadVec3 right(forward.y * worldUp.z - forward.z * worldUp.y,
                  forward.z * worldUp.x - forward.x * worldUp.z,
                  forward.x * worldUp.y - forward.y * worldUp.x);
    right = CadVec3(right.x / lengthOf(right), right.y / lengthOf(right), right.z / lengthOf(right));
    const CadVec3 up(right.y * forward.z - right.z * forward.y,
                     right.z * forward.x - right.x * forward.z,
                     right.x * forward.y - right.y * forward.x);
    CadTransform pose;
    // The camera looks down its own -Z, so its third column is the direction it looks away from.
    pose.values = {{right.x, up.x, -forward.x, eye.x,
                    right.y, up.y, -forward.y, eye.y,
                    right.z, up.z, -forward.z, eye.z}};
    return pose;
}

constexpr int kViewWidth = 1280;
constexpr int kViewHeight = 800;
constexpr double kSnapPercent = 1.5;
constexpr double kLaneLength = 1400.0;

RayCamera sceneCamera() {
    RayCamera view;
    view.camera.pose = lookAt(CadVec3(3600.0, 2400.0, 3000.0), CadVec3(1400.0, 300.0, 0.0));
    view.camera.widthPx = kViewWidth;
    view.camera.heightPx = kViewHeight;
    view.camera.viewAngleDeg = 30.0;
    return view;
}

double thresholdPixels() {
    return std::max(2.0, static_cast<double>(std::min(kViewWidth, kViewHeight)) *
                             kSnapPercent / 100.0);
}

std::shared_ptr<CadNode> conveyorTree(double widthMm, bool endStop) {
    ConveyorSceneryRequest request;
    request.parameters.accessoryGenerator = "roller_conveyor";
    request.parameters.accessoryLengthMm = kLaneLength;
    request.parameters.accessoryWidthMm = widthMm;
    request.parameters.accessoryHeightMm = 866.0;
    request.parameters.accessoryStartHeightMm = 866.0;
    request.parameters.accessoryEndHeightMm = 866.0;
    request.parameters.accessoryStartLeftHeightMm = 866.0;
    request.parameters.accessoryStartRightHeightMm = 866.0;
    request.parameters.accessoryEndLeftHeightMm = 866.0;
    request.parameters.accessoryEndRightHeightMm = 866.0;
    request.parameters.accessoryConveyorRole = "normal";
    request.parameters.accessoryEndStopEnabled = endStop;
    return buildConveyorScenery(request).root;
}

std::shared_ptr<CadNode> floorGrid() {
    auto floor = std::make_shared<CadNode>();
    floor->name = "Floor";
    floor->type = CadNodeType::Transform;
    floor->data = std::make_shared<TransformNodeData>();
    MountingHoleGridData grid;
    grid.originMm = CadVec3(-2000.0, 0.0, -2000.0);
    grid.uStepMm = CadVec3(100.0, 0.0, 0.0);
    grid.vStepMm = CadVec3(0.0, 0.0, 100.0);
    grid.uCount = 61;
    grid.vCount = 41;
    floor->mountingHoles.grids.push_back(grid);
    return floor;
}

struct EveryPattern final : mountingsnap::TargetFilter {
    bool eligible(const CadNode*, bool) const override { return true; }
};

const mountingsnap::Interface* interfaceNamed(const std::vector<mountingsnap::Interface>& list,
                                              const std::string& id) {
    for (const mountingsnap::Interface& entry : list) {
        if (entry.interfaceId == id) return &entry;
    }
    return nullptr;
}

// Computes a cursor target independently of the solver.
bool exactMate(const mountingsnap::Interface& source, const mountingsnap::Interface& target,
               CadTransform* pose) {
    for (const CadVec3& anchor : source.pointsMm) {
        for (const CadVec3& onto : target.pointsMm) {
            const CadVec3 shift = minus(onto, anchor);
            bool all = true;
            for (const CadVec3& point : source.pointsMm) {
                const CadVec3 moved(point.x + shift.x, point.y + shift.y, point.z + shift.z);
                double nearest = 1.0e30;
                for (const CadVec3& candidate : target.pointsMm) {
                    nearest = std::min(nearest, lengthOf(minus(moved, candidate)));
                }
                if (nearest > 1.0e-3) { all = false; break; }
            }
            if (!all) continue;
            *pose = CadTransform();
            pose->values[3] = shift.x;
            pose->values[7] = shift.y;
            pose->values[11] = shift.z;
            return true;
        }
    }
    return false;
}

// How far every point of the mated source interface ends up from the target hole it claimed.
double worstResidualMm(const mountingsnap::Result& result,
                       const std::vector<mountingsnap::Interface>& sources,
                       const std::vector<mountingsnap::Interface>& targets) {
    if (result.sourceInterface < 0 || !result.targetNode) return 1.0e30;
    const mountingsnap::Interface& source =
        sources[static_cast<size_t>(result.sourceInterface)];
    const mountingsnap::Interface* target = nullptr;
    for (const mountingsnap::Interface& entry : targets) {
        if (entry.node == result.targetNode) target = &entry;
    }
    if (!target) return 1.0e30;
    double worst = 0.0;
    for (const CadVec3& point : source.pointsMm) {
        const CadVec3 placed = result.worldPose * point;
        double nearest = 1.0e30;
        for (const CadVec3& candidate : target->pointsMm) {
            nearest = std::min(nearest, lengthOf(minus(placed, candidate)));
        }
        worst = std::max(worst, nearest);
    }
    return worst;
}

// One scene: a conveyor standing where it stands, and the floor under it.
struct Scene {
    std::shared_ptr<CadNode> standing = conveyorTree(700.0, /*endStop=*/false);
    std::shared_ptr<CadNode> floor = floorGrid();
    std::vector<mountingsnap::Interface> targets;

    Scene() {
        const EveryPattern everything;
        mountingsnap::collectTargets(standing.get(), CadTransform(), false, nullptr, everything,
                                     &targets);
        mountingsnap::collectTargets(floor.get(), CadTransform(), false, nullptr, everything,
                                     &targets);
    }
};

mountingsnap::Request requestAt(const Scene& scene,
                                const std::vector<mountingsnap::Interface>& sources,
                                double cursorX, double cursorY) {
    mountingsnap::Request request;
    request.sourceInterfaces = &sources;
    request.targets = &scene.targets;
    request.floor = scene.floor.get();
    request.cursorX = cursorX;
    request.cursorY = cursorY;
    request.viewportWidthPx = kViewWidth;
    request.viewportHeightPx = kViewHeight;
    request.snapScreenPercent = kSnapPercent;
    return request;
}

void anEndMatesTheNextEnd() {
    const Scene scene;
    const RayCamera view = sceneCamera();
    const std::shared_ptr<CadNode> placed = conveyorTree(700.0, /*endStop=*/false);
    std::vector<mountingsnap::Interface> sources;
    mountingsnap::collectInterfaces(placed.get(), CadTransform(), /*placementSourcesOnly=*/true,
                                    &sources);
    check(interfaceNamed(sources, "") != nullptr,
          "the conveyor being placed offers its foot pattern, which is what a free drag follows");
    const mountingsnap::Interface* start = interfaceNamed(sources, "start");
    const mountingsnap::Interface* standingEnd = interfaceNamed(scene.targets, "end");
    check(start != nullptr && standingEnd != nullptr,
          "both conveyors declare the end interfaces this is about");
    if (!start || !standingEnd) return;

    CadTransform mated;
    check(exactMate(*start, *standingEnd, &mated),
          "a start and an end can be brought together by a translation alone");

    const CadVec3 dragCentre = middleOf(sources.front().pointsMm);
    double cursorX = 0.0;
    double cursorY = 0.0;
    check(view.project(mated * dragCentre, &cursorX, &cursorY),
          "the mated conveyor's middle is in front of the camera");

    const mountingsnap::Result result =
        mountingsnap::solve(requestAt(scene, sources, cursorX, cursorY), view);
    check(result.snapped, "it snapped: " + result.status);
    check(result.matchedHoles == 4 && result.requiredHoles == 4,
          "every hole of the end pattern matched, not " + std::to_string(result.matchedHoles) +
              " of " + std::to_string(result.requiredHoles));
    check(result.targetNode == standingEnd->node,
          "it mated the standing conveyor's end rather than its feet or the floor");
    check(result.sourceInterface >= 0 &&
              sources[static_cast<size_t>(result.sourceInterface)].interfaceId == "start",
          "and it is the near end that met it, so the belts run on rather than back");
    const CadVec3 stood = result.worldPose * dragCentre;
    const CadVec3 along = minus(stood, middleOf(scene.targets.front().pointsMm));
    check(std::abs(along.x - kLaneLength) < 1.0 && std::abs(along.y) < 1.0 &&
              std::abs(along.z) < 1.0,
          "it stands one lane length on from the conveyor it joined, not " +
              std::to_string(along.x) + ", " + std::to_string(along.y) + ", " +
              std::to_string(along.z) + " mm");
    const double residual = worstResidualMm(result, sources, scene.targets);
    check(residual < 1.0e-3,
          "the snapped pose puts every source hole on a target hole; the worst is " +
              std::to_string(residual) + " mm out");
    std::printf("  end to end: %s\n    stands at %.3f, %.3f, %.3f mm, worst hole %.6f mm\n",
                result.status.c_str(), stood.x, stood.y, stood.z, residual);
}

void oneMorePixelDoesNotSnap() {
    const Scene scene;
    const RayCamera view = sceneCamera();
    const std::shared_ptr<CadNode> placed = conveyorTree(700.0, /*endStop=*/false);
    std::vector<mountingsnap::Interface> sources;
    mountingsnap::collectInterfaces(placed.get(), CadTransform(), true, &sources);
    const mountingsnap::Interface* start = interfaceNamed(sources, "start");
    const mountingsnap::Interface* standingEnd = interfaceNamed(scene.targets, "end");
    if (!start || !standingEnd) { check(false, "the end interfaces are there"); return; }
    CadTransform mated;
    if (!exactMate(*start, *standingEnd, &mated)) { check(false, "the mate exists"); return; }

    const CadVec3 dragCentre = middleOf(sources.front().pointsMm);
    double cursorX = 0.0;
    double cursorY = 0.0;
    view.project(mated * dragCentre, &cursorX, &cursorY);

    const auto matesAt = [&](double offsetPixels) {
        const mountingsnap::Result result = mountingsnap::solve(
            requestAt(scene, sources, cursorX + offsetPixels, cursorY), view);
        return result.snapped && result.targetNode == standingEnd->node;
    };
    check(matesAt(0.0), "it mates with the cursor where the hand would leave it");

    int reach = 0;
    while (reach < 400 && matesAt(static_cast<double>(reach + 1))) ++reach;
    check(reach > 0 && reach < 400, "the reach is a number of pixels, measured at " +
                                        std::to_string(reach));
    check(!matesAt(static_cast<double>(reach + 1)),
          "one pixel past the reach it does not snap");
    const double declared = thresholdPixels() * 2.0;
    check(std::abs(static_cast<double>(reach) - declared) <= declared * 0.35,
          "the measured reach " + std::to_string(reach) + " px is the declared " +
              std::to_string(declared) + " px for a directional end");
    std::printf("  reach: snaps out to %d px, not at %d; declared %.1f px\n", reach, reach + 1,
                declared);
}

void anEndRefusesTheFloor() {
    Scene scene;
    // The floor alone, so there is no other end anywhere to mate with.
    scene.targets.clear();
    const EveryPattern everything;
    mountingsnap::collectTargets(scene.floor.get(), CadTransform(), false, nullptr, everything,
                                 &scene.targets);
    const RayCamera view = sceneCamera();
    const std::shared_ptr<CadNode> placed = conveyorTree(700.0, /*endStop=*/false);
    std::vector<mountingsnap::Interface> whole;
    mountingsnap::collectInterfaces(placed.get(), CadTransform(), true, &whole);

    double cursorX = 0.0;
    double cursorY = 0.0;
    view.project(CadVec3(1000.0, 0.0, 0.0), &cursorX, &cursorY);

    const mountingsnap::Result onItsFeet =
        mountingsnap::solve(requestAt(scene, whole, cursorX, cursorY), view);
    check(onItsFeet.snapped && onItsFeet.matchedHoles == 1 && onItsFeet.requiredHoles == 1,
          "a conveyor dropped on the floor stands on it: " + onItsFeet.status);
    check(onItsFeet.sourceInterface >= 0 &&
              !whole[static_cast<size_t>(onItsFeet.sourceInterface)].mateOpposite,
          "and it is the feet that met the floor, not an end");

    std::vector<mountingsnap::Interface> endsOnly;
    for (const mountingsnap::Interface& entry : whole) {
        if (entry.mateOpposite) endsOnly.push_back(entry);
    }
    check(endsOnly.size() == 2, "the conveyor has two end interfaces, not " +
                                    std::to_string(endsOnly.size()));
    const mountingsnap::Result refused =
        mountingsnap::solve(requestAt(scene, endsOnly, cursorX, cursorY), view);
    check(!refused.snapped, "an end will not mate a floor pattern: " + refused.status);
    std::printf("  floor: feet %s | ends %s\n", onItsFeet.status.c_str(), refused.status.c_str());
}

mountingsnap::Interface patternOf(const std::vector<CadVec3>& points) {
    mountingsnap::Interface interface;
    interface.pointsMm = points;
    return interface;
}

void aSquareTurnsByOneAndALineByTwo() {
    const mountingsnap::Interface square = patternOf({CadVec3(0.0, 0.0, 0.0),
                                                      CadVec3(100.0, 0.0, 0.0),
                                                      CadVec3(0.0, 0.0, 100.0),
                                                      CadVec3(100.0, 0.0, 100.0)});
    const mountingsnap::Interface line = patternOf({CadVec3(0.0, 0.0, 0.0),
                                                    CadVec3(100.0, 0.0, 0.0),
                                                    CadVec3(200.0, 0.0, 0.0)});
    const mountingsnap::Interface single = patternOf({CadVec3(0.0, 0.0, 0.0)});
    check(mountingsnap::rotationQuarterStep(square) == 1, "a square pattern steps by one quarter");
    check(mountingsnap::rotationQuarterStep(line) == 2, "a line pattern steps by two");
    check(mountingsnap::rotationQuarterStep(single) == 1, "a single hole steps by one");

    const std::shared_ptr<CadNode> conveyor = conveyorTree(700.0, /*endStop=*/false);
    std::vector<mountingsnap::Interface> sources;
    mountingsnap::collectInterfaces(conveyor.get(), CadTransform(), true, &sources);
    check(mountingsnap::rotationQuarterStep(sources.front()) == 2,
          "a 1400 by 700 foot pattern steps by two");
    std::printf("  quarter step: square 1, line 2, conveyor feet %d\n",
                mountingsnap::rotationQuarterStep(sources.front()));
}

// The state between two mouse moves, driven the way both hosts drive it: arm, move, rotate, read.
void aSessionArmsWalksAndTurnsTheWheel() {
    const Scene scene;
    const RayCamera view = sceneCamera();
    const std::shared_ptr<CadNode> placed = conveyorTree(700.0, /*endStop=*/false);
    std::vector<mountingsnap::Interface> sources;
    mountingsnap::collectInterfaces(placed.get(), CadTransform(), true, &sources);
    const mountingsnap::Interface* start = interfaceNamed(sources, "start");
    const mountingsnap::Interface* standingEnd = interfaceNamed(scene.targets, "end");
    if (!start || !standingEnd) { check(false, "the end interfaces are there"); return; }
    CadTransform mated;
    if (!exactMate(*start, *standingEnd, &mated)) { check(false, "the mate exists"); return; }

    placementsession::Session session;
    check(!session.armed(), "a session nothing has armed is not armed");
    check(session.activeInterface() == nullptr, "and offers no interface to commit by");
    session.arm(sources);
    check(session.armed(), "armed with what the conveyor being placed offers");
    check(session.quarterTurn() == 0, "a fresh placement has not been turned");

    const CadVec3 dragCentre = middleOf(sources.front().pointsMm);
    double onX = 0.0;
    double onY = 0.0;
    check(view.project(mated * dragCentre, &onX, &onY),
          "the mated conveyor's middle is in front of the camera");
    const auto moveTo = [&](double x, double y) {
        return session.moved(x, y, scene.targets, view, kViewWidth, kViewHeight, scene.floor.get(),
                             kSnapPercent);
    };

    const mountingsnap::Result onto = moveTo(onX, onY);
    check(onto.snapped && onto.targetNode == standingEnd->node,
          "the cursor walked onto the target end and mated it: " + onto.status);
    check(session.activeInterface() != nullptr &&
              session.activeInterface()->interfaceId == "start",
          "and the session names the near end as what a commit would link by");
    check(session.last().snapped && session.last().targetNode == standingEnd->node,
          "the answer is held rather than handed back once, which is what a host reads between moves");

    const mountingsnap::Result off = moveTo(onX + 400.0, onY);
    check(off.targetNode != standingEnd->node,
          "walking the cursor off the end lets go of it: " + off.status);
    check(session.activeInterface() != nullptr && !session.activeInterface()->mateOpposite,
          "and what it is being dragged by is its feet again, not the end it left");
    check(moveTo(onX, onY).targetNode == standingEnd->node,
          "and walking back on mates it again, from the same session");

    const int step = mountingsnap::rotationQuarterStep(*session.activeInterface());
    check(step == 2, "a conveyor end pattern steps by two quarters, not " + std::to_string(step));
    int expected = session.quarterTurn();
    for (int notch = 1; notch <= 4; ++notch) {
        session.rotated(1.0f);
        expected = (expected + step) % 4;
        check(session.quarterTurn() == expected,
              "notch " + std::to_string(notch) + " stepped to " +
                  std::to_string(session.quarterTurn()) + " rather than " +
                  std::to_string(expected));
        // Retain the mate while rotating about the free-drag centre.
        const mountingsnap::Result turned = moveTo(onX, onY);
        check(turned.snapped && turned.targetNode == standingEnd->node,
              "notch " + std::to_string(notch) + " kept the mate: " + turned.status);
        check(session.activeInterface() != nullptr &&
                  session.activeInterface()->interfaceId == "start",
              "notch " + std::to_string(notch) +
                  " kept the *near* end mated rather than swapping to the far one");
        check(turned.matchedHoles == 4,
              "notch " + std::to_string(notch) + " matched " +
                  std::to_string(turned.matchedHoles) + " of four holes");
        const double residual = worstResidualMm(turned, sources, scene.targets);
        check(residual < 1.0e-3, "notch " + std::to_string(notch) +
                                     " left the worst hole " + std::to_string(residual) + " mm out");
    }
    check(session.quarterTurn() == 0,
          "four notches of a two-quarter step bring the pattern back to the holes it started in");

    session.rotated(1.0f);
    mountingsnap::Request unretained = requestAt(scene, sources, onX, onY);
    unretained.quarterTurn = session.quarterTurn();
    const mountingsnap::Result blind = mountingsnap::solve(unretained, view);
    const std::string blindId = blind.sourceInterface >= 0
        ? sources[static_cast<size_t>(blind.sourceInterface)].interfaceId : std::string("(none)");
    check(blind.snapped && blind.targetNode == standingEnd->node && blindId == "end",
          "asked cold, a half turn mates the far end to the same target - it mated '" + blindId +
              "': " + blind.status);
    std::printf("  session: mates, lets go, re-mates; wheel steps by %d and holds 'start' where a "
                "cold solve takes '%s'\n", step, blindId.c_str());
}

void partNotchesAccumulateRatherThanRound() {
    const std::shared_ptr<CadNode> conveyor = conveyorTree(700.0, /*endStop=*/false);
    std::vector<mountingsnap::Interface> sources;
    mountingsnap::collectInterfaces(conveyor.get(), CadTransform(), true, &sources);
    placementsession::Session session;
    session.arm(sources);
    session.rotated(0.4f);
    check(session.quarterTurn() == 0, "four tenths of a notch turns nothing");
    session.rotated(0.4f);
    check(session.quarterTurn() == 0, "and eight tenths still turns nothing");
    session.rotated(0.4f);
    check(session.quarterTurn() == 1, "the notch that completes turns exactly one quarter");
    session.rotated(-1.2f);
    check(session.quarterTurn() == 0, "and it accumulates backwards the same way");
    session.cancel();
    check(!session.armed() && session.quarterTurn() == 0,
          "cancelling forgets what was armed and how far it had turned");
    std::printf("  wheel: fractions accumulate, and cancelling forgets the turn\n");
}

void theMateWithoutACameraIsTheSamePose() {
    const Scene scene;
    const RayCamera view = sceneCamera();
    const std::shared_ptr<CadNode> placed = conveyorTree(700.0, /*endStop=*/false);
    std::vector<mountingsnap::Interface> sources;
    mountingsnap::collectInterfaces(placed.get(), CadTransform(), true, &sources);
    const mountingsnap::Interface* start = interfaceNamed(sources, "start");
    const mountingsnap::Interface* standingEnd = interfaceNamed(scene.targets, "end");
    if (!start || !standingEnd) { check(false, "the end interfaces are there"); return; }

    CadTransform matedByHand;
    if (!exactMate(*start, *standingEnd, &matedByHand)) { check(false, "the mate exists"); return; }
    const CadVec3 dragCentre = middleOf(sources.front().pointsMm);
    double cursorX = 0.0;
    double cursorY = 0.0;
    view.project(matedByHand * dragCentre, &cursorX, &cursorY);
    const mountingsnap::Result withCursor =
        mountingsnap::solve(requestAt(scene, sources, cursorX, cursorY), view);
    if (!withCursor.snapped) { check(false, "the cursor path snapped"); return; }

    const mountingsnap::Mate blind = mountingsnap::mate(*start, *standingEnd, /*quarterTurn=*/0);
    check(blind.complete && blind.matchedHoles == 4 && blind.requiredHoles == 4,
          "the camera-free mate satisfies the whole pattern, not " +
              std::to_string(blind.matchedHoles) + " of " + std::to_string(blind.requiredHoles));
    double worstCell = 0.0;
    for (int cell = 0; cell < 12; ++cell) {
        worstCell = std::max(worstCell, std::abs(blind.worldPose.values[cell] -
                                                withCursor.worldPose.values[cell]));
    }
    check(worstCell < 1.0e-3,
          "the camera-free pose is the cursor's pose; the worst cell is " +
              std::to_string(worstCell) + " mm out");
    check(blind.worstErrorMm < 1.0e-3,
          "and it puts every hole on a hole; the worst is " + std::to_string(blind.worstErrorMm) +
              " mm out");

    const mountingsnap::Mate refused =
        mountingsnap::mate(sources.front(), *standingEnd, /*quarterTurn=*/0);
    check(!refused.complete, "feet do not mate an end grid, and the answer says so");
    std::printf("  camera-free mate: %.6f mm from the cursor's pose, worst hole %.6f mm\n",
                worstCell, blind.worstErrorMm);
}

void theFloorWindowIsALatticeAndNotAPatch() {
    const double centres[] = {0.0, 37.0, -1234.5, 12000.0, -48000.0};
    for (double centre : centres) {
        const MountingHoleGridData grid = mountingsnap::floorGridWindow(CadVec3(centre, 0.0, centre));
        const double offU = std::fmod(std::fabs(grid.originMm.x), mountingsnap::kFloorGridPitchMm);
        const double offV = std::fmod(std::fabs(grid.originMm.z), mountingsnap::kFloorGridPitchMm);
        check(std::min(offU, mountingsnap::kFloorGridPitchMm - offU) < 1.0e-6 &&
                  std::min(offV, mountingsnap::kFloorGridPitchMm - offV) < 1.0e-6,
              "a window centred at " + std::to_string(centre) +
                  " starts on the lattice, not at the cursor");
        check(grid.uCount > 2 && grid.uCount == grid.vCount,
              "the window has points in both directions");
    }

    const CadVec3 farOut(12000.0, 0.0, -48000.0);
    const MountingHoleGridData grid = mountingsnap::floorGridWindow(farOut);
    bool found = false;
    for (uint32_t u = 0; u < grid.uCount && !found; ++u) {
        for (uint32_t v = 0; v < grid.vCount && !found; ++v) {
            const CadVec3 point(grid.originMm.x + grid.uStepMm.x * u + grid.vStepMm.x * v,
                                0.0,
                                grid.originMm.z + grid.uStepMm.z * u + grid.vStepMm.z * v);
            found = std::fabs(point.x - farOut.x) < 1.0e-6 && std::fabs(point.z - farOut.z) < 1.0e-6;
        }
    }
    check(found, "a window twelve metres out contains the lattice point under the cursor");

    const RayCamera view = sceneCamera();
    auto floor = floorGrid();
    const double halfway = static_cast<double>(kViewWidth) * 0.5;
    check(mountingsnap::recentreFloorGrid(floor.get(), floor->loc, view, halfway, halfway * 0.9),
          "the cursor's ray meets the floor and the window moves");
    CadVec3 aimed;
    CadVec3 direction;
    view.rayThrough(halfway, halfway * 0.9, &aimed, &direction);
    const double distance = -aimed.y / direction.y;
    const CadVec3 hit(aimed.x + direction.x * distance, 0.0, aimed.z + direction.z * distance);
    const MountingHoleGridData& moved = floor->mountingHoles.grids.front();
    const CadVec3 windowCentre(
        moved.originMm.x + moved.uStepMm.x * (moved.uCount - 1) * 0.5,
        0.0,
        moved.originMm.z + moved.vStepMm.z * (moved.vCount - 1) * 0.5);
    // Within one step of the aimed point: the window is quantised, so it cannot be nearer than that.
    check(std::fabs(windowCentre.x - hit.x) <= mountingsnap::kFloorGridPitchMm &&
              std::fabs(windowCentre.z - hit.z) <= mountingsnap::kFloorGridPitchMm,
          "the window centres on what the cursor is pointing at, not on the origin");
}

} // namespace

int main() {
    anEndMatesTheNextEnd();
    oneMorePixelDoesNotSnap();
    anEndRefusesTheFloor();
    aSquareTurnsByOneAndALineByTwo();
    aSessionArmsWalksAndTurnsTheWheel();
    partNotchesAccumulateRatherThanRound();
    theMateWithoutACameraIsTheSamePose();
    theFloorWindowIsALatticeAndNotAPatch();

    if (g_failures > 0) {
        std::printf("mounting snap smoke FAILED with %d problem(s)\n", g_failures);
        return 1;
    }
    std::printf("mounting snap smoke passed\n");
    return 0;
}
