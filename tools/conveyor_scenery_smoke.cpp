// Where the drawn conveyor actually is, in numbers.

#include "ConveyorScenery.cpp"

#include "AccessoryGeometry.cpp"
#include "CadNodeDraw.cpp"
#include "AccessoryPropertySchema.cpp"
#include "ConveyorGeometry.cpp"

#include <cmath>
#include <cstdio>
#include <string>

namespace {

int g_failures = 0;

void check(bool condition, const std::string& what) {
    if (condition) return;
    ++g_failures;
    std::printf("FAIL: %s\n", what.c_str());
}

struct Bounds {
    double low[3] = {1e30, 1e30, 1e30};
    double high[3] = {-1e30, -1e30, -1e30};
    void add(float x, float y, float z) {
        const double point[3] = {x, y, z};
        for (int axis = 0; axis < 3; ++axis) {
            low[axis] = std::min(low[axis], point[axis]);
            high[axis] = std::max(high[axis], point[axis]);
        }
    }
    double centre(int axis) const { return (low[axis] + high[axis]) * 0.5; }
    double size(int axis) const { return high[axis] - low[axis]; }
};

Bounds boundsOf(const ConveyorScenery& scenery) {
    Bounds bounds;
    for (const ConveyorDrawGroup& group : scenery.groups) {
        for (size_t at = 0; at + 2 < group.verticesMm.size(); at += 3) {
            bounds.add(group.verticesMm[at], group.verticesMm[at + 1], group.verticesMm[at + 2]);
        }
    }
    return bounds;
}

constexpr double kLaneX = 10.0;
constexpr double kLaneZ = 370.0;
constexpr double kDeckHeight = 866.0;
constexpr double kLaneLength = 1400.0;
constexpr double kWorkpieceAcross = 398.1;

TransformNodeData showcaseParameters(double widthMm, const std::string& role) {
    TransformNodeData parameters;
    parameters.accessoryGenerator = "roller_conveyor";
    parameters.accessoryLengthMm = kLaneLength;
    parameters.accessoryWidthMm = widthMm;
    parameters.accessoryHeightMm = kDeckHeight;
    parameters.accessoryStartHeightMm = kDeckHeight;
    parameters.accessoryEndHeightMm = kDeckHeight;
    parameters.accessoryStartLeftHeightMm = kDeckHeight;
    parameters.accessoryStartRightHeightMm = kDeckHeight;
    parameters.accessoryEndLeftHeightMm = kDeckHeight;
    parameters.accessoryEndRightHeightMm = kDeckHeight;
    parameters.accessoryConveyorRole = role;
    parameters.accessoryEndStopEnabled = true;
    return parameters;
}

CadTransform standingAt(double acrossMm) {
    CadTransform pose;
    // Columns: along = -Y, across = +X, up = +Z.
    pose.values[0] = 0.0;  pose.values[1] = 1.0; pose.values[2] = 0.0;
    pose.values[4] = -1.0; pose.values[5] = 0.0; pose.values[6] = 0.0;
    pose.values[8] = 0.0;  pose.values[9] = 0.0; pose.values[10] = 1.0;
    pose.values[3] = kLaneX + acrossMm;
    pose.values[7] = 550.0 + kLaneLength * 0.5;
    pose.values[11] = kLaneZ - kDeckHeight;
    return pose;
}

ConveyorScenery showcaseLane(double widthMm, const std::string& role, double acrossMm = 0.0) {
    ConveyorSceneryRequest request;
    request.parameters = showcaseParameters(widthMm, role);
    request.world = standingAt(acrossMm);
    return buildConveyorScenery(request);
}

// Use the same parameters and pose as the scenery generator.
CadVec3 lanePoint(double widthMm, const std::string& role, double acrossMm, double progress) {
    const TransformNodeData parameters = showcaseParameters(widthMm, role);
    return conveyorTransformPoint(standingAt(acrossMm) * conveyorAccessoryFrame(),
                                  conveyorPathPoseAt(parameters, progress));
}

void theDeckIsCentredOnTheLane() {
    const ConveyorScenery scenery = showcaseLane(700.0, "normal");
    check(scenery.triangles > 0, "the conveyor was drawn at all");

    const Bounds bounds = boundsOf(scenery);
    // Across the lane is x here, because the lane runs down y.
    check(std::abs(bounds.centre(0) - kLaneX) < 1.0,
          "the deck is centred across the lane at x=" + std::to_string(kLaneX) + "; its own centre "
          "is x=" + std::to_string(bounds.centre(0)));
    check(std::abs(bounds.centre(1) - 1250.0) < 10.0,
          "the deck is centred along the lane; its centre is y=" + std::to_string(bounds.centre(1)));
    check(bounds.size(1) >= 1400.0 - 1.0 && bounds.size(1) <= 1400.0 + 200.0,
          "the deck is the lane's length, not " + std::to_string(bounds.size(1)) + " mm");
    check(bounds.high[2] >= kLaneZ - 1.0,
          "the deck reaches the lane at z=" + std::to_string(kLaneZ) + "; its highest point is z=" +
              std::to_string(bounds.high[2]));
    check(std::abs(bounds.low[2] - (kLaneZ - kDeckHeight)) < 20.0,
          "its feet reach the floor at z=" + std::to_string(kLaneZ - kDeckHeight) + "; they are at "
          "z=" + std::to_string(bounds.low[2]));
}

void theDeckCarriesTheWorkpiece() {
    const ConveyorScenery scenery = showcaseLane(kWorkpieceAcross + 150.0, "normal");
    const Bounds bounds = boundsOf(scenery);
    check(bounds.size(0) >= kWorkpieceAcross,
          "a deck " + std::to_string(bounds.size(0)) + " mm across cannot carry a workpiece " +
              std::to_string(kWorkpieceAcross) + " mm across");
}

void theDeclaredWidthIsTheDrawnWidth() {
    const Bounds narrow = boundsOf(showcaseLane(600.0, "normal"));
    const Bounds wide = boundsOf(showcaseLane(1000.0, "normal"));
    check(wide.size(0) > narrow.size(0) + 300.0,
          "400 mm more declared width drew " + std::to_string(wide.size(0) - narrow.size(0)) +
              " mm more deck");
    check(std::abs(wide.centre(0) - narrow.centre(0)) < 1.0,
          "widening the deck moved its centre, so the extra width went to one side");
}

void aSpawnerDrawsMoreThanAPlainConveyor() {
    const ConveyorScenery plain = showcaseLane(700.0, "normal");
    const ConveyorScenery spawner = showcaseLane(700.0, "spawner");
    check(spawner.triangles > plain.triangles,
          "a spawner draws its enclosure and marking on top of the deck: " +
              std::to_string(spawner.triangles) + " triangles against " +
              std::to_string(plain.triangles));

    for (const ConveyorDrawGroup& group : spawner.groups) {
        Bounds bounds;
        for (size_t at = 0; at + 2 < group.verticesMm.size(); at += 3) {
            bounds.add(group.verticesMm[at], group.verticesMm[at + 1], group.verticesMm[at + 2]);
        }
        check(std::abs(bounds.centre(0) - kLaneX) < 1.0,
              "one drawn part is centred at x=" + std::to_string(bounds.centre(0)) + " rather than "
              "on the lane at x=" + std::to_string(kLaneX));
    }
}

void theLaneIsTheDecksCentreline() {
    constexpr double kAcross = 198.8;
    const Bounds square = boundsOf(showcaseLane(700.0, "spawner"));
    const Bounds shifted = boundsOf(showcaseLane(700.0, "spawner", kAcross));

    check(std::abs(shifted.centre(0) - (kLaneX + kAcross)) < 1.0,
          "the drawn deck stands where the conveyor does, at x=" + std::to_string(kLaneX + kAcross) +
              "; it is centred at x=" + std::to_string(shifted.centre(0)));
    check(std::abs(shifted.centre(1) - square.centre(1)) < 1.0e-6,
          "and moving it across the lane did not move it along the lane");
    check(std::abs(shifted.centre(2) - square.centre(2)) < 1.0e-6,
          "nor up");
    for (const double across : {0.0, kAcross}) {
        for (const double progress : {0.0, 1.0}) {
            const CadVec3 point = lanePoint(700.0, "spawner", across, progress);
            check(std::abs(point.x - (kLaneX + across)) < 1.0e-9,
                  "the lane runs down the middle of the deck: at progress " +
                      std::to_string(progress) + " it is at x=" + std::to_string(point.x) +
                      " and the deck is centred on x=" + std::to_string(kLaneX + across));
            check(std::abs(point.z - kLaneZ) < 1.0e-9,
                  "and on top of it, at z=" + std::to_string(kLaneZ) + " rather than z=" +
                      std::to_string(point.z));
        }
    }
    const CadVec3 start = lanePoint(700.0, "spawner", 0.0, 0.0);
    const CadVec3 finish = lanePoint(700.0, "spawner", 0.0, 1.0);
    check(std::abs(std::abs(finish.y - start.y) - kLaneLength) < 1.0e-6,
          "the lane is the declared length, not " + std::to_string(std::abs(finish.y - start.y)));
    check(std::abs((start.y + finish.y) * 0.5 - square.centre(1)) < 10.0,
          "and centred where the deck is");
}

void aTurnAngleDrawsACurve() {
    ConveyorSceneryRequest request;
    request.parameters = showcaseParameters(700.0, "normal");
    request.parameters.accessoryTurnAngleDeg = 90.0;
    request.parameters.accessoryCurveRadiusMm = 900.0;
    request.world = standingAt(0.0);
    const ConveyorScenery curved = buildConveyorScenery(request);
    check(curved.triangles > 0, "a curved conveyor is drawn at all");

    const CadVec3 start = conveyorTransformPoint(
        request.world * conveyorAccessoryFrame(), conveyorPathPoseAt(request.parameters, 0.0));
    const CadVec3 finish = conveyorTransformPoint(
        request.world * conveyorAccessoryFrame(), conveyorPathPoseAt(request.parameters, 1.0));
    check(std::abs(std::abs(finish.x - start.x) - 900.0) < 1.0,
          "a quarter turn at 900 mm radius moves the lane 900 mm across; this one moved " +
              std::to_string(std::abs(finish.x - start.x)));
    check(std::abs(finish.z - start.z) < 1.0e-6, "and stays level, because the deck is level");
}

// One triangle's worth of mesh, as a child of `parent`, so a count can be reasoned about exactly.
std::shared_ptr<CadNode> oneTriangleMesh(const char* name, bool visible) {
    auto node = std::make_shared<CadNode>();
    node->name = name;
    node->type = CadNodeType::MeshGeometry;
    node->visible = visible;
    auto mesh = std::make_shared<MeshGeometryData>();
    mesh->vertices = {0.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f, 0.0f, 10.0f, 0.0f};
    mesh->indices = {0, 1, 2};
    node->data = mesh;
    return node;
}

// The hidden prototype is instanced once at each link frame.
void aCarrierIsDrawnOncePerLinkFrame() {
    auto chain = std::make_shared<CadNode>();
    chain->name = "Drag Chain";
    chain->type = CadNodeType::DragChainMechanism;
    auto data = std::make_shared<DragChainMechanismData>();
    chain->data = data;

    std::shared_ptr<CadNode> prototype = oneTriangleMesh("member prototype", /*visible=*/false);
    chain->children.push_back(prototype);
    data->prototypeGeometry = prototype.get();

    constexpr int kMembers = 7;
    for (int member = 0; member < kMembers; ++member) {
        auto frame = std::make_shared<CadNode>();
        frame->name = "member " + std::to_string(member);
        frame->type = CadNodeType::Transform;
        frame->loc.values[3] = 100.0 * member;
        chain->children.push_back(frame);
        data->linkFrames.push_back(frame.get());
    }

    const cadnodedraw::Scenery drawn = cadnodedraw::flatten(chain.get(), CadTransform());
    check(drawn.triangles == kMembers,
          "a carrier of 7 one-triangle members draws 7 triangles; this drew " +
              std::to_string(drawn.triangles));

    Bounds bounds;
    for (const cadnodedraw::DrawGroup& group : drawn.groups) {
        for (size_t at = 0; at + 2 < group.verticesMm.size(); at += 3) {
            bounds.add(group.verticesMm[at], group.verticesMm[at + 1], group.verticesMm[at + 2]);
        }
    }
    // Six 100 mm steps plus the 10 mm member.
    check(std::abs(bounds.size(0) - 610.0) < 1.0e-6,
          "the members stand where their frames put them, spanning 610 mm; this spanned " +
              std::to_string(bounds.size(0)));

    // The prototype must not appear as an additional member.
    auto hiddenOnly = std::make_shared<CadNode>();
    hiddenOnly->name = "holder";
    hiddenOnly->type = CadNodeType::Transform;
    hiddenOnly->children.push_back(oneTriangleMesh("hidden", /*visible=*/false));
    hiddenOnly->children.push_back(oneTriangleMesh("shown", /*visible=*/true));
    const cadnodedraw::Scenery half = cadnodedraw::flatten(hiddenOnly.get(), CadTransform());
    check(half.triangles == 1,
          "an invisible mesh is not in the cell and is not drawn; two children, one hidden, drew " +
              std::to_string(half.triangles));
}

void report() {
    const ConveyorScenery spawner = showcaseLane(700.0, "spawner");
    std::printf("  a 700 mm spawner on the showcase's lane, %d triangles:\n", spawner.triangles);
    for (const ConveyorDrawGroup& group : spawner.groups) {
        Bounds bounds;
        for (size_t at = 0; at + 2 < group.verticesMm.size(); at += 3) {
            bounds.add(group.verticesMm[at], group.verticesMm[at + 1], group.verticesMm[at + 2]);
        }
        std::printf("    %5d tri  x %8.1f..%8.1f  y %8.1f..%8.1f  z %8.1f..%8.1f\n",
                    group.triangles, bounds.low[0], bounds.high[0], bounds.low[1], bounds.high[1],
                    bounds.low[2], bounds.high[2]);
    }
}

} // namespace

int main() {
    theDeckIsCentredOnTheLane();
    theDeckCarriesTheWorkpiece();
    theDeclaredWidthIsTheDrawnWidth();
    aSpawnerDrawsMoreThanAPlainConveyor();
    theLaneIsTheDecksCentreline();
    aTurnAngleDrawsACurve();
    aCarrierIsDrawnOncePerLinkFrame();
    report();

    if (g_failures > 0) {
        std::printf("conveyor scenery smoke FAILED with %d problem(s)\n", g_failures);
        return 1;
    }
    std::printf("conveyor scenery smoke passed\n");
    return 0;
}
