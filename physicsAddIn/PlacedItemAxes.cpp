#include "PlacedItemAxes.h"

#include <cmath>

namespace {

constexpr double kRadToDeg = 57.295779513082320876798154814105;
constexpr double kDegToRad = 0.017453292519943295769236907684886;

} // namespace

PlacedItemAxes::Kind PlacedItemAxes::bind(CadNode* root) {
    m_kind = Kind::None;
    m_chains.clear();
    // Held so the shared schema can be asked what this mechanism's ranges are without the tree being
    // passed back in at every call. It is the tree the controllers already hold raw pointers into, so it
    // outlives this by exactly as much as they do.
    m_root = root;
    if (!root) return m_kind;
    // The arm first, because a rail carrying an arm is both and the arm is what a placement is about.
    // Driving such a package's rail as well needs a second controller per placed item and a verb that
    // says which axis it means; until something asks for that, the honest thing is to move the arm.
    if (m_robot.bind(root)) {
        m_kind = Kind::Robot;
        m_robot.setJoints(m_robot.joints());
    } else if (m_rail.bind(root)) {
        m_kind = Kind::Rail;
        // `bindToGantry` leaves the carriage at zero, which is not the same as the rail's home and for a
        // beam whose travel does not straddle zero is not even a position it has.
        m_rail.resetHome();
    }
    for (CadNode* chainNode : collectDragChainMechanismNodes(root)) {
        DragChainPoseController chain;
        // Silent on refusal. An incomplete carrier - no prototype mesh, no moving frame, one link - is a
        // package that never had one, and a placement is not the place to complain about it: the rest of
        // the mechanism is fine and stands in the cell either way.
        if (!chain.bindToDragChain(chainNode)) continue;
        m_chains.push_back(chain);
    }
    return m_kind;
}

int PlacedItemAxes::axisCount() const {
    switch (m_kind) {
    case Kind::Robot: return 6;
    case Kind::Rail: return 1;
    case Kind::None: break;
    }
    return 0;
}

std::vector<double> PlacedItemAxes::values() const {
    switch (m_kind) {
    case Kind::Robot: {
        const std::array<double, 6>& q = m_robot.joints();
        std::vector<double> degrees(q.size());
        for (size_t axis = 0; axis < q.size(); ++axis) degrees[axis] = q[axis] * kRadToDeg;
        return degrees;
    }
    case Kind::Rail:
        return {m_rail.positionMm()};
    case Kind::None:
        break;
    }
    return {};
}

bool PlacedItemAxes::setValues(const std::vector<double>& values, QString* error) {
    // Validation and editor bounds share PlacedMechanismSchema.
    std::string refusal;
    if (!placedmechanism::axesAreInRange(m_root, values, &refusal)) {
        if (error) *error = QString::fromStdString(refusal);
        return false;
    }

    if (m_kind == Kind::Rail) {
        m_rail.setPositionMm(values[0]);
        if (error) error->clear();
        return true;
    }

    std::array<double, 6> radians{};
    for (size_t axis = 0; axis < radians.size(); ++axis) radians[axis] = values[axis] * kDegToRad;
    m_robot.setJoints(radians);
    if (error) error->clear();
    return true;
}

std::vector<placedmechanism::AxisField> PlacedItemAxes::fields() const {
    return placedmechanism::axisFields(m_root);
}

void PlacedItemAxes::apply() {
    switch (m_kind) {
    case Kind::Robot:
        m_robot.setJoints(m_robot.joints());
        break;
    case Kind::Rail:
        m_rail.setPositionMm(m_rail.positionMm());
        break;
    case Kind::None:
        break;
    }
    for (DragChainPoseController& chain : m_chains) chain.update();
}

CadTransform PlacedItemAxes::flangeInArmBase() const {
    return m_kind == Kind::Robot ? m_robot.toolPose() : CadTransform();
}

CadTransform PlacedItemAxes::armBaseInTree() const {
    // `baseWorldTransform` walks every `loc` from the arm up to whatever has no parent, which for a
    // placed item is the package tree's own root - and `loadLibraryItemTree` has set that root's `loc` to
    // identity, because the item's stored pose is the whole statement of where this stands. So what comes
    // back is in the tree's frame, and the station is one `treeToWorld` further out.
    return m_kind == Kind::Robot ? m_robot.baseWorldTransform() : CadTransform();
}

CadTransform PlacedItemAxes::railBaseInTree() const {
    CadTransform base;
    if (m_kind != Kind::Rail) return base;
    const GantryMechanismData* data = m_rail.gantryData();
    const CadNode* gantry = m_rail.gantryNode();
    if (!data || !gantry) return base;

    // Where the travel runs, in the tree's own frame. `setPositionMm` prepends `transl(axis * p)` onto
    // the moving frame's bind pose, so the axis is stated in the gantry node's frame and has to be
    // rotated up through every parent between there and the tree's root.
    CadTransform toGantry;
    for (const CadNode* node = gantry; node != nullptr; node = node->parent) {
        toGantry = node->loc * toGantry;
    }
    CadVec3 travel = rotate(toGantry, data->axisOfTravel);
    const double length = std::sqrt(travel.x * travel.x + travel.y * travel.y + travel.z * travel.z);
    if (length <= 1.0e-9) return base;
    travel = CadVec3(travel.x / length, travel.y / length, travel.z / length);

    const CadTransform carriage = railCarriageInTree();
    const double at = m_rail.positionMm();

    // A frame whose +Z runs along the travel, because that is the axis RoboDK's own one-axis translation
    // mechanism moves along - `BuildMechanism` takes no parameters for a 1T and translates about Z. So
    // this is the rotation that makes RoboDK's joint and this package's millimetres the same number.
    CadVec3 up(0.0, 0.0, 1.0);
    if (std::abs(travel.z) > 0.9) up = CadVec3(1.0, 0.0, 0.0);
    CadVec3 side(up.y * travel.z - up.z * travel.y, up.z * travel.x - up.x * travel.z,
                 up.x * travel.y - up.y * travel.x);
    const double sideLength = std::sqrt(side.x * side.x + side.y * side.y + side.z * side.z);
    if (sideLength <= 1.0e-9) return base;
    side = CadVec3(side.x / sideLength, side.y / sideLength, side.z / sideLength);
    const CadVec3 other(travel.y * side.z - travel.z * side.y, travel.z * side.x - travel.x * side.z,
                        travel.x * side.y - travel.y * side.x);
    // Columns are the frame's own axes: x = side, y = other, z = travel.
    base.values[0] = side.x;  base.values[1] = other.x;  base.values[2] = travel.x;
    base.values[4] = side.y;  base.values[5] = other.y;  base.values[6] = travel.y;
    base.values[8] = side.z;  base.values[9] = other.z;  base.values[10] = travel.z;
    base.values[3] = carriage.values[3] - travel.x * at;
    base.values[7] = carriage.values[7] - travel.y * at;
    base.values[11] = carriage.values[11] - travel.z * at;
    return base;
}

CadTransform PlacedItemAxes::railCarriageInTree() const {
    CadTransform world;
    if (m_kind != Kind::Rail) return world;
    for (const CadNode* node = m_rail.movingFrame(); node != nullptr; node = node->parent) {
        world = node->loc * world;
    }
    return world;
}
