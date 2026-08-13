#include "PlacedMechanismSchema.h"

#include "OrientationFormat.h"
#include "RobotRuntime.h"

#include <cmath>

namespace placedmechanism {
namespace {

constexpr double kRadToDeg = 57.295779513082320876798154814105;
constexpr double kDegToRad = 0.017453292519943295769236907684886;

// A rail's travel and an arm's joints are typed to a tenth: finer than an operator can mean and
// coarser than the arithmetic, which is where a mechanism's numbers have always been shown.
constexpr int kAxisDecimals = 1;

constexpr orientation::Format kRoboDkPoseFormat{
    "RoboDK XYZRPW", orientation::Kind::EulerIntrinsic, {2, 1, 0}, /*reversedFields=*/true, 3,
    {"Rx", "Ry", "Rz", ""}, "%.3f deg", 1.0, 1.0};

// A metre of travel either way and a full turn. Wide enough that no real cell reaches an edge, and
// bounded so a spin box has something to be, which is the only reason a limit is stated at all.
constexpr double kOffsetLimitMm = 1.0e6;
constexpr double kAngleLimitDeg = 360.0;

const std::array<PlacementField, 6> kPlacementFields{{
    {"x", "X", "mm", kOffsetLimitMm, 10.0, 3, false},
    {"y", "Y", "mm", kOffsetLimitMm, 10.0, 3, false},
    {"z", "Z", "mm", kOffsetLimitMm, 10.0, 3, false},
    {"rx", "Rx", "deg", kAngleLimitDeg, 5.0, 3, true},
    {"ry", "Ry", "deg", kAngleLimitDeg, 5.0, 3, true},
    {"rz", "Rz", "deg", kAngleLimitDeg, 5.0, 3, true},
}};

// A number for a sentence, without dragging a stream or Qt in here. Two decimals, because these appear
// only in refusals an operator reads.
std::string say(double value) {
    const bool negative = value < 0.0;
    double magnitude = negative ? -value : value;
    // Rounded on the way in, so 6399.999 does not print as 6399.99 next to a limit of 6400.
    magnitude = std::floor(magnitude * 100.0 + 0.5) / 100.0;
    const long long whole = static_cast<long long>(magnitude);
    const long long hundredths = static_cast<long long>(
        std::floor((magnitude - static_cast<double>(whole)) * 100.0 + 0.5));
    std::string text = std::to_string(whole) + ".";
    if (hundredths < 10) text += "0";
    text += std::to_string(hundredths);
    return negative ? "-" + text : text;
}

} // namespace

const std::array<PlacementField, 6>& placementFields() { return kPlacementFields; }

std::array<double, 6> placementValues(const CadTransform& pose) {
    const std::array<double, 4> angles = orientation::valuesFromRotation(kRoboDkPoseFormat, pose);
    return {{pose.values[3], pose.values[7], pose.values[11], angles[0], angles[1], angles[2]}};
}

CadTransform placementPose(const std::array<double, 6>& values) {
    CadTransform pose = orientation::rotationFromValues(
        kRoboDkPoseFormat, {{values[3], values[4], values[5], 0.0}});
    pose.values[3] = values[0];
    pose.values[7] = values[1];
    pose.values[11] = values[2];
    return pose;
}

AxisKind axisKindOf(const CadNode* root) {
    if (!root) return AxisKind::None;
    if (findOPW6RobotNode(root)) return AxisKind::Robot;
    if (findGantryMechanismNode(root)) return AxisKind::Rail;
    return AxisKind::None;
}

std::vector<AxisField> axisFields(const CadNode* root) {
    std::vector<AxisField> fields;
    switch (axisKindOf(root)) {
    case AxisKind::None:
        return fields;
    case AxisKind::Rail: {
        const CadNode* node = findGantryMechanismNode(root);
        const GantryMechanismData* data = node ? node->asGantryMechanism() : nullptr;
        AxisField field;
        field.key = "carriage";
        field.label = "Carriage";
        field.unit = "mm";
        field.decimals = kAxisDecimals;
        // A beam whose ends are both zero has stated no travel, and a carriage held to [0, 0] could not
        // be moved at all. The same reading `PlacedItemAxes::setValues` gives an arm's unstated joint.
        field.limited = data && (data->lowerLimitMm != 0.0 || data->upperLimitMm != 0.0);
        field.minimum = field.limited ? data->lowerLimitMm : -kOffsetLimitMm;
        field.maximum = field.limited ? data->upperLimitMm : kOffsetLimitMm;
        // A tenth of the travel per click, so crossing a six-metre beam is not a hundred clicks - and
        // never zero, which would freeze the widget for a rail that stated no travel.
        const double span = field.maximum - field.minimum;
        field.step = span > 0.0 ? span / 100.0 : 10.0;
        fields.push_back(field);
        return fields;
    }
    case AxisKind::Robot: {
        const CadNode* node = findOPW6RobotNode(root);
        const OPW6RobotData* data = node ? node->asOPW6Robot() : nullptr;
        for (int joint = 0; joint < 6; ++joint) {
            AxisField field;
            field.key = "j" + std::to_string(joint + 1);
            field.label = "J" + std::to_string(joint + 1);
            field.unit = "deg";
            field.decimals = kAxisDecimals;
            field.step = 5.0;
            const size_t at = static_cast<size_t>(joint);
            // Both zero means the package stated no limit for this joint, which is not the same as a
            // joint that cannot move. Holding it to [0, 0] would refuse every pose including home.
            field.limited = data && (data->qMin[at] != 0.0 || data->qMax[at] != 0.0);
            field.minimum = field.limited ? data->qMin[at] * kRadToDeg : -kAngleLimitDeg;
            field.maximum = field.limited ? data->qMax[at] * kRadToDeg : kAngleLimitDeg;
            fields.push_back(field);
        }
        return fields;
    }
    }
    return fields;
}

bool axesAreInRange(const CadNode* root, const std::vector<double>& values, std::string* error) {
    const std::vector<AxisField> fields = axisFields(root);
    const auto fail = [&](const std::string& message) {
        if (error) *error = message;
        return false;
    };
    if (fields.empty()) return fail("It has no axes to set.");
    if (values.size() != fields.size()) {
        return fail("It takes " + std::to_string(fields.size()) +
                    (fields.size() == 1 ? " number, not " : " numbers, not ") +
                    std::to_string(values.size()) + ".");
    }
    for (size_t at = 0; at < values.size(); ++at) {
        if (!std::isfinite(values[at])) return fail(say(values[at]) + " is not a number.");
    }
    for (size_t at = 0; at < values.size(); ++at) {
        const AxisField& field = fields[at];
        if (!field.limited) continue;
        // A hair's slack at each end, so a value read out of a widget and typed straight back is not
        // refused for the last bit of its own rounding.
        if (values[at] >= field.minimum - 1.0e-9 && values[at] <= field.maximum + 1.0e-9) continue;
        // Worded per kind, because "travel" is what a rail's range is called and a check greps for it.
        if (field.unit[0] == 'm') {
            return fail(say(values[at]) + " mm is outside this rail's travel, " +
                        say(field.minimum) + " to " + say(field.maximum) + " mm.");
        }
        return fail(field.label + " = " + say(values[at]) + " deg is outside this arm's limits, " +
                    say(field.minimum) + " to " + say(field.maximum) + " deg.");
    }
    if (error) error->clear();
    return true;
}

} // namespace placedmechanism
