
#include "PlacedMechanismSchema.h"

#include "CadNodePackage.h"
#include "LibraryCatalogue.h"

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

const char* kDefaultRoot = "library/packages";

const librarycatalogue::Entry* entryNamed(const std::vector<librarycatalogue::Entry>& entries,
                                          const std::string& name) {
    for (const librarycatalogue::Entry& entry : entries) {
        if (entry.name == name) return &entry;
    }
    return nullptr;
}

double worstElement(const CadTransform& a, const CadTransform& b) {
    double worst = 0.0;
    for (int cell = 0; cell < 12; ++cell) {
        worst = std::max(worst, std::abs(a.values[cell] - b.values[cell]));
    }
    return worst;
}

// Independent transcription of RoboDK's XYZRPW_2_Mat composition.
CadTransform roboDkPose(double x, double y, double z, double r, double p, double w) {
    const double a = r * 3.14159265358979323846 / 180.0;
    const double b = p * 3.14159265358979323846 / 180.0;
    const double c = w * 3.14159265358979323846 / 180.0;
    const double ca = std::cos(a), sa = std::sin(a);
    const double cb = std::cos(b), sb = std::sin(b);
    const double cc = std::cos(c), sc = std::sin(c);
    CadTransform pose;
    pose.values[0] = cb * cc;
    pose.values[1] = cc * sa * sb - ca * sc;
    pose.values[2] = sa * sc + ca * cc * sb;
    pose.values[3] = x;
    pose.values[4] = cb * sc;
    pose.values[5] = ca * cc + sa * sb * sc;
    pose.values[6] = ca * sb * sc - cc * sa;
    pose.values[7] = y;
    pose.values[8] = -sb;
    pose.values[9] = cb * sa;
    pose.values[10] = ca * cb;
    pose.values[11] = z;
    return pose;
}

// Include general rotations and one near an Euler pole.
const double kRotations[][6] = {
    {1200.0, -800.0, 350.0, 0.0, 0.0, 0.0},
    {-4200.0, 1800.0, 300.0, 0.0, 0.0, 37.0},
    {0.0, 0.0, 0.0, 15.0, -25.0, 40.0},
    {900.0, 120.0, -60.0, -170.0, 65.0, 155.0},
    {10.0, 20.0, 30.0, 5.0, 89.7, -12.0},
    {0.0, 0.0, 0.0, 180.0, 0.0, 180.0},
};

void theSixNumbersAreRoboDksSixNumbers() {
    std::printf("--- a placement's six numbers mean what RoboDK means by them\n");
    double worstAgainstRoboDk = 0.0;
    double worstRoundTrip = 0.0;
    for (const auto& row : kRotations) {
        const std::array<double, 6> typed{{row[0], row[1], row[2], row[3], row[4], row[5]}};
        const CadTransform mine = placedmechanism::placementPose(typed);
        const CadTransform theirs = roboDkPose(row[0], row[1], row[2], row[3], row[4], row[5]);
        worstAgainstRoboDk = std::max(worstAgainstRoboDk, worstElement(mine, theirs));

        const std::array<double, 6> read = placedmechanism::placementValues(mine);
        worstRoundTrip = std::max(worstRoundTrip, worstElement(placedmechanism::placementPose(read),
                                                              mine));
    }
    check(worstAgainstRoboDk < 1.0e-9,
          "the schema composes RoboDK's XYZRPW; worst element differs by " +
              std::to_string(worstAgainstRoboDk));
    check(worstRoundTrip < 1.0e-9,
          "a pose read out as six numbers and typed back is the same pose; worst element moved by " +
              std::to_string(worstRoundTrip));
    std::printf("  %zu rotations: %.3e against RoboDK, %.3e round-tripped\n",
                sizeof(kRotations) / sizeof(kRotations[0]), worstAgainstRoboDk, worstRoundTrip);

    const CadTransform pose = placedmechanism::placementPose({{1.0, 2.0, 3.0, 0.0, 0.0, 0.0}});
    check(pose.values[3] == 1.0 && pose.values[7] == 2.0 && pose.values[11] == 3.0,
          "x, y and z are the translation column");

    const std::array<placedmechanism::PlacementField, 6>& fields =
        placedmechanism::placementFields();
    check(!fields[0].rotation && !fields[2].rotation && fields[3].rotation && fields[5].rotation,
          "the first three fields are offsets and the last three are angles");
    for (const placedmechanism::PlacementField& field : fields) {
        check(field.limit > 0.0 && field.step > 0.0,
              std::string(field.key) + " carries a range and a step for a widget to use");
    }
}

void aRailsTravelIsThePackagesTravel(const std::vector<librarycatalogue::Entry>& entries) {
    std::printf("--- a rail's fields are the rail's own numbers\n");
    // Two rails with different travels, so a field that reported a constant would pass on neither.
    const struct { const char* name; double lower; double upper; } kRails[] = {
        {"Gudel TMF-1 6m", 0.0, 6400.0},
        {"WS2000", 0.0, 1107.0},
    };
    for (const auto& wanted : kRails) {
        const librarycatalogue::Entry* entry = entryNamed(entries, wanted.name);
        check(entry != nullptr, std::string(wanted.name) + " is in the catalogue");
        if (!entry || !entry->root) continue;
        check(placedmechanism::axisKindOf(entry->root.get()) == placedmechanism::AxisKind::Rail,
              std::string(wanted.name) + " is driven as a rail");
        const std::vector<placedmechanism::AxisField> fields =
            placedmechanism::axisFields(entry->root.get());
        check(fields.size() == 1, std::string(wanted.name) + " has one axis, not " +
                                      std::to_string(fields.size()));
        if (fields.size() != 1) continue;
        const placedmechanism::AxisField& axis = fields.front();
        check(axis.key == "carriage" && std::string(axis.unit) == "mm",
              std::string(wanted.name) + " drives a carriage in millimetres");
        check(axis.limited, std::string(wanted.name) + " states a travel");
        check(std::abs(axis.minimum - wanted.lower) < 1.0e-6 &&
                  std::abs(axis.maximum - wanted.upper) < 1.0e-6,
              std::string(wanted.name) + " travels " + std::to_string(wanted.lower) + " to " +
                  std::to_string(wanted.upper) + " mm; the field says " +
                  std::to_string(axis.minimum) + " to " + std::to_string(axis.maximum));
        check(axis.step > 0.0, std::string(wanted.name) + "'s carriage has a step");

        // Validate the same bounds exposed by the editor.
        std::string error;
        check(placedmechanism::axesAreInRange(entry->root.get(), {axis.maximum}, &error),
              std::string(wanted.name) + " takes the top of its own travel: " + error);
        check(!placedmechanism::axesAreInRange(entry->root.get(), {axis.maximum + 100.0}, &error),
              std::string(wanted.name) + " refuses 100 mm past the end of its beam");
        check(error.find("travel") != std::string::npos,
              "and names the travel it has: " + error);
        check(!placedmechanism::axesAreInRange(entry->root.get(), {axis.maximum, 0.0}, &error),
              std::string(wanted.name) + " refuses two numbers for one axis");
    }
}

void anArmsLimitsAreTheArmsOwn(const std::vector<librarycatalogue::Entry>& entries) {
    std::printf("--- an arm's fields are the arm's own joint limits\n");
    const librarycatalogue::Entry* entry = entryNamed(entries, "AR4 6DOF Robot");
    check(entry != nullptr, "the AR4 is in the catalogue");
    if (!entry || !entry->root) return;
    check(placedmechanism::axisKindOf(entry->root.get()) == placedmechanism::AxisKind::Robot,
          "the AR4 is driven as an arm");
    const std::vector<placedmechanism::AxisField> fields =
        placedmechanism::axisFields(entry->root.get());
    check(fields.size() == 6, "an arm has six axes, not " + std::to_string(fields.size()));
    if (fields.size() != 6) return;
    for (size_t at = 0; at < fields.size(); ++at) {
        check(fields[at].key == "j" + std::to_string(at + 1),
              "joint " + std::to_string(at + 1) + " is keyed j" + std::to_string(at + 1));
        check(std::string(fields[at].unit) == "deg", "an arm's joints are in degrees");
        check(fields[at].maximum > fields[at].minimum,
              fields[at].label + " has a range that is the right way round");
    }
    check(std::abs(fields[1].maximum - 90.0) < 1.0e-6,
          "the AR4's J2 stops at 90 deg; the field says " + std::to_string(fields[1].maximum));
    check(std::abs(fields[1].minimum - -42.0) < 1.0e-6,
          "and starts at -42 deg; the field says " + std::to_string(fields[1].minimum));

    std::string error;
    check(placedmechanism::axesAreInRange(entry->root.get(),
                                          {10.0, -20.0, 30.0, -40.0, 50.0, -60.0}, &error),
          "the AR4 takes a pose inside its limits: " + error);
    check(!placedmechanism::axesAreInRange(entry->root.get(), {0.0, 120.0, 0.0, 0.0, 0.0, 0.0},
                                           &error),
          "and refuses J2 = 120 deg, which it has not got");
    check(error.find("J2") != std::string::npos && error.find("limits") != std::string::npos,
          "naming the joint and its limits: " + error);
}

void somethingThatIsNotAMechanism(const std::vector<librarycatalogue::Entry>& entries) {
    std::printf("--- a package that is not a mechanism has no axes\n");
    const librarycatalogue::Entry* entry = entryNamed(entries, "Pedestal H12in (Type 1)");
    check(entry != nullptr, "the pedestal is in the catalogue");
    if (!entry || !entry->root) return;
    check(placedmechanism::axisKindOf(entry->root.get()) == placedmechanism::AxisKind::None,
          "a pedestal is driven by nothing");
    check(placedmechanism::axisFields(entry->root.get()).empty(), "so it offers no axis fields");
    std::string error;
    check(!placedmechanism::axesAreInRange(entry->root.get(), {0.0}, &error),
          "and takes no axis values");
    check(placedmechanism::axisKindOf(nullptr) == placedmechanism::AxisKind::None,
          "and neither does nothing at all");
}

} // namespace

int main(int argc, char** argv) {
    const std::string root = argc > 1 ? argv[1] : kDefaultRoot;
    const std::vector<librarycatalogue::Entry> entries = librarycatalogue::scan(root);
    if (entries.empty()) {
        std::printf("placed mechanism schema smoke FAILED: no packages under %s\n", root.c_str());
        return 1;
    }

    theSixNumbersAreRoboDksSixNumbers();
    aRailsTravelIsThePackagesTravel(entries);
    anArmsLimitsAreTheArmsOwn(entries);
    somethingThatIsNotAMechanism(entries);

    if (g_failures > 0) {
        std::printf("placed mechanism schema smoke FAILED with %d problem(s)\n", g_failures);
        return 1;
    }
    std::printf("placed mechanism schema smoke passed\n");
    return 0;
}
