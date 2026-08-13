#pragma once

#include "CadNode.h"

#include <array>
#include <string>
#include <vector>

// The editable fields of a placed mechanism, described once: where it stands, and where its axes are.

namespace placedmechanism {

enum class AxisKind { None, Rail, Robot };

// One of the six numbers a placement is typed as.
struct PlacementField {
    const char* key;    // "x".."rz"; the stable id, and what a command line would name
    const char* label;  // what an operator reads
    const char* unit;   // "mm" or "deg"
    double limit;       // the range, symmetric about zero
    double step;
    int decimals;
    bool rotation;      // whether it is one of the three angles rather than one of the three offsets
};

// The six, in the order a panel should show them: three offsets, then three angles.
const std::array<PlacementField, 6>& placementFields();

// A pose as those six numbers, and back.
std::array<double, 6> placementValues(const CadTransform& pose);
CadTransform placementPose(const std::array<double, 6>& values);

// One driven axis: a rail's carriage, or one of an arm's six joints.
struct AxisField {
    std::string key;    // "carriage", or "j1".."j6"
    std::string label;
    const char* unit;   // "mm" or "deg"
    double minimum = 0.0;
    double maximum = 0.0;
    double step = 1.0;
    int decimals = 1;
    // Whether the package stated a range at all. An arm whose limits are both zero for a joint has not
    // stated any, and holding it to [0, 0] would refuse every pose including its own home - so a field
    // that is not `limited` carries a nominal range for a widget and must not be *validated* against it.
    bool limited = true;
};

AxisKind axisKindOf(const CadNode* root);
std::vector<AxisField> axisFields(const CadNode* root);

// Whether `values` is a set this mechanism will take, and why not. The one statement of that question:
// a panel bounds its editors with `axisFields` and a driver validates with this, so the two cannot come
// to disagree about what a rail's travel is.
bool axesAreInRange(const CadNode* root, const std::vector<double>& values, std::string* error);

} // namespace placedmechanism
