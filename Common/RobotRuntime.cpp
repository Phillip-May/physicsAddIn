#include "RobotRuntime.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <set>

namespace {

using strutil::operator<<;

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

Vec3 makeVec3(const std::array<double, 3>& values) {
    return {values[0], values[1], values[2]};
}

std::array<double, 3> toArray(const Vec3& v) {
    return {{v.x, v.y, v.z}};
}

Vec3 operator+(const Vec3& a, const Vec3& b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

Vec3 operator-(const Vec3& a, const Vec3& b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

Vec3 operator*(double scalar, const Vec3& v) {
    return {scalar * v.x, scalar * v.y, scalar * v.z};
}

double dot(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3 cross(const Vec3& a, const Vec3& b) {
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

double magnitudeSquared(const Vec3& v) {
    return dot(v, v);
}

Vec3 normalized(Vec3 v) {
    const double mag = std::sqrt(magnitudeSquared(v));
    if (mag <= 1.0e-12) return {};
    return (1.0 / mag) * v;
}

Vec3 transformPoint(const CadTransform& transform, const Vec3& point) {
    const CadVec3 out = transform * CadVec3(point.x, point.y, point.z);
    return {out.x, out.y, out.z};
}

Vec3 transformVector(const CadTransform& transform, const Vec3& vector) {
    const CadVec3 out = rotate(transform, CadVec3(vector.x, vector.y, vector.z));
    return {out.x, out.y, out.z};
}

Vec3 translationOf(const CadTransform& transform) {
    return {transform.values[3], transform.values[7], transform.values[11]};
}

double clampValue(double value, double minValue, double maxValue) {
    return std::max(minValue, std::min(maxValue, value));
}

double wrapRadians(double value) {
    constexpr double kPi = 3.14159265358979323846;
    while (value > kPi) value -= 2.0 * kPi;
    while (value < -kPi) value += 2.0 * kPi;
    return value;
}

CadTransform rotationAroundAxis(const Vec3& origin, Vec3 direction, double angle) {
    direction = normalized(direction);
    CadTransform transform;
    if (magnitudeSquared(direction) <= 1.0e-12) return transform;

    const double x = direction.x;
    const double y = direction.y;
    const double z = direction.z;
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    const double oneMinusC = 1.0 - c;

    const double r00 = c + x * x * oneMinusC;
    const double r01 = x * y * oneMinusC - z * s;
    const double r02 = x * z * oneMinusC + y * s;
    const double r10 = y * x * oneMinusC + z * s;
    const double r11 = c + y * y * oneMinusC;
    const double r12 = y * z * oneMinusC - x * s;
    const double r20 = z * x * oneMinusC - y * s;
    const double r21 = z * y * oneMinusC + x * s;
    const double r22 = c + z * z * oneMinusC;

    const Vec3 rotatedOrigin{
        r00 * origin.x + r01 * origin.y + r02 * origin.z,
        r10 * origin.x + r11 * origin.y + r12 * origin.z,
        r20 * origin.x + r21 * origin.y + r22 * origin.z
    };
    const Vec3 translation = origin - rotatedOrigin;

    transform.values = {{r00, r01, r02, translation.x,
                         r10, r11, r12, translation.y,
                         r20, r21, r22, translation.z}};
    return transform;
}

double rotationErrorAngle(const CadTransform& currentPose, const CadTransform& targetPose) {
    const CadTransform error = targetPose * currentPose.rigidInverse();
    const double sinX = 0.5 * (error.values[9] - error.values[6]);
    const double sinY = 0.5 * (error.values[2] - error.values[8]);
    const double sinZ = 0.5 * (error.values[4] - error.values[1]);
    const double sinAngle = std::sqrt(sinX * sinX + sinY * sinY + sinZ * sinZ);
    const double cosAngle = clampValue((error.values[0] + error.values[5] + error.values[10] - 1.0) * 0.5, -1.0, 1.0);
    return std::atan2(sinAngle, cosAngle);
}

double posePositionError(const CadTransform& currentPose, const CadTransform& targetPose) {
    const Vec3 delta = translationOf(targetPose) - translationOf(currentPose);
    return std::sqrt(magnitudeSquared(delta));
}

constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kIkFkPositionToleranceMm = 1.0e-6;
constexpr double kIkFkRotationToleranceRad = 1.0e-6;

CadTransform dhmLinkTransform(double alpha, double a, double theta, double d) {
    const double crx = std::cos(alpha);
    const double srx = std::sin(alpha);
    const double crz = std::cos(theta);
    const double srz = std::sin(theta);
    CadTransform pose;
    pose.values = {{
        crz, -srz, 0.0, a,
        crx * srz, crx * crz, -srx, -d * srx,
        srx * srz, crz * srx, crx, d * crx
    }};
    return pose;
}

CadTransform dhmJointFrameTransform(double alpha, double a) {
    const double crx = std::cos(alpha);
    const double srx = std::sin(alpha);
    CadTransform pose;
    pose.values = {{
        1.0, 0.0, 0.0, a,
        0.0, crx, -srx, 0.0,
        0.0, srx, crx, 0.0
    }};
    return pose;
}

CadTransform ar4ForwardDhm(const RobotDhmData& dhm, const std::array<double, 6>& q) {
    CadTransform pose;
    for (size_t i = 0; i < 6; ++i) {
        pose = pose * dhmLinkTransform(dhm.alpha(i), dhm.a(i), dhm.thetaOffset(i) + q[i], dhm.d(i));
    }
    return pose;
}

CadTransform viewerFromAr4Transform(const CadTransform& ar4) {
    CadTransform s;
    s.values = {{1.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 1.0, 0.0,
                 0.0, -1.0, 0.0, 0.0}};
    return s * ar4 * s.rigidInverse();
}

CadTransform ar4FromViewerTransform(const CadTransform& viewer) {
    CadTransform s;
    s.values = {{1.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 1.0, 0.0,
                 0.0, -1.0, 0.0, 0.0}};
    const CadTransform sInv = s.rigidInverse();
    return sInv * viewer * s;
}

double nearestEquivalentRadians(double value, double reference) {
    while (value - reference > kPi) value -= 2.0 * kPi;
    while (value - reference < -kPi) value += 2.0 * kPi;
    return value;
}

int ar4AnalyticInverseDhm(const RobotDhmData& dhm,
                          const CadTransform& targetAr4Pose,
                          const std::array<double, 6>& referenceQ,
                          std::array<double, 6>* solutions,
                          int solutionCapacity) {
    if (!dhm.isValid() || !solutions || solutionCapacity <= 0) return 0;

    RobotMotionCore::RobotModel model{};
    for (size_t i = 0; i < dhm.values.size(); ++i) model.dhm[i] = dhm.values[i];
    for (double& sign : model.dhmSigns) sign = 1.0;
    model.valid = 1;

    RobotMotionCore::Transform target{};
    for (size_t i = 0; i < targetAr4Pose.values.size(); ++i) {
        target.values[i] = targetAr4Pose.values[i];
    }

    double coreSolutions[12][RobotMotionCore::kAxisCount]{};
    const int capacity = std::min(solutionCapacity, 12);
    const int count = RobotMotionCore::ar4AnalyticInverseDhm(
        model, target, referenceQ.data(), coreSolutions, capacity);
    for (int candidate = 0; candidate < count; ++candidate) {
        for (size_t joint = 0; joint < solutions[candidate].size(); ++joint) {
            solutions[candidate][joint] = coreSolutions[candidate][joint];
        }
    }
    return count;
}

template <typename AxisT>
bool allAxesValid(const std::array<AxisT, 6>& axes) {
    for (const AxisT& axis : axes) {
        if (!axis.valid || magnitudeSquared(makeVec3(axis.direction)) <= 1.0e-12) return false;
    }
    return true;
}

template <typename AxisT>
int validAxisCount(const std::array<AxisT, 6>& axes) {
    int count = 0;
    for (const AxisT& axis : axes) {
        if (axis.valid && magnitudeSquared(makeVec3(axis.direction)) > 1.0e-12) ++count;
    }
    return count;
}

bool isUrStyleDhmData(const RobotDhmData& dhm) {
    if (!dhm.isValid()) return false;
    constexpr double kTolerance = 1.0e-8;
    const auto nearlyEqual = [=](double left, double right) {
        return std::abs(left - right) <= kTolerance;
    };
    return nearlyEqual(dhm.thetaOffset(0), 0.0) &&
           nearlyEqual(dhm.thetaOffset(1), kPi) &&
           nearlyEqual(dhm.thetaOffset(2), 0.0) &&
           nearlyEqual(dhm.thetaOffset(3), 0.0) &&
           nearlyEqual(dhm.thetaOffset(4), 0.0) &&
           nearlyEqual(dhm.thetaOffset(5), kPi) &&
           nearlyEqual(dhm.alpha(0), 0.0) &&
           nearlyEqual(dhm.alpha(1), kPi * 0.5) &&
           nearlyEqual(dhm.alpha(2), 0.0) &&
           nearlyEqual(dhm.alpha(3), 0.0) &&
           nearlyEqual(dhm.alpha(4), -kPi * 0.5) &&
           nearlyEqual(dhm.alpha(5), kPi * 0.5) &&
           nearlyEqual(dhm.a(0), 0.0) &&
           nearlyEqual(dhm.a(1), 0.0) &&
           dhm.a(2) > kTolerance &&
           dhm.a(3) > kTolerance &&
           nearlyEqual(dhm.a(4), 0.0) &&
           nearlyEqual(dhm.a(5), 0.0) &&
           std::abs(dhm.d(4)) > kTolerance &&
           std::abs(dhm.d(5)) > kTolerance;
}

template <typename AxisT>
void loadDhmJointAxes(const OPW6RobotData& robotData, std::array<AxisT, 6>& axes) {
    const bool urStyle = isUrStyleDhmData(robotData.dhm);
    CadTransform ar4Pose;
    for (int i = 0; i < 6; ++i) {
        const size_t index = static_cast<size_t>(i);
        const CadTransform jointFrame = ar4Pose * dhmJointFrameTransform(robotData.dhm.alpha(index), robotData.dhm.a(index));
        const CadTransform viewerJointFrame = viewerFromAr4Transform(jointFrame);
        Vec3 direction = normalized(transformVector(viewerJointFrame, {0.0, 1.0, 0.0}));
        if (urStyle && i == 4) direction = -1.0 * direction;

        AxisT& dst = axes[index];
        dst.origin = toArray(translationOf(viewerJointFrame));
        dst.direction = toArray(direction);
        dst.valid = magnitudeSquared(direction) > 1.0e-12;

        ar4Pose = ar4Pose * dhmLinkTransform(
            robotData.dhm.alpha(index),
            robotData.dhm.a(index),
            robotData.dhm.thetaOffset(index) +
                ((urStyle && i == 4) ? -robotData.qHome[index] : robotData.qHome[index]),
            robotData.dhm.d(index));
    }
}

template <typename AxisT>
std::array<CadTransform, 7> axisLinkDeltas(const std::array<AxisT, 6>& axes,
                                           const std::array<double, 6>& q,
                                           const std::array<double, 6>& qHome) {
    std::array<CadTransform, 7> result{};
    CadTransform cumulative;
    result[0] = cumulative;

    for (int i = 0; i < 6; ++i) {
        const AxisT& homeAxis = axes[static_cast<size_t>(i)];
        const Vec3 currentOrigin = transformPoint(cumulative, makeVec3(homeAxis.origin));
        Vec3 currentDirection = transformVector(cumulative, makeVec3(homeAxis.direction));
        if (magnitudeSquared(currentDirection) <= 1.0e-12) {
            result[static_cast<size_t>(i + 1)] = cumulative;
            continue;
        }
        currentDirection = normalized(currentDirection);

        const CadTransform rotation = rotationAroundAxis(currentOrigin, currentDirection, q[static_cast<size_t>(i)] - qHome[static_cast<size_t>(i)]);
        cumulative = rotation * cumulative;
        result[static_cast<size_t>(i + 1)] = cumulative;
    }
    return result;
}

template <typename AxisT>
std::array<double, 6> dhmSignsForViewerAxes(const RobotDhmData& dhm, const std::array<AxisT, 6>& axes) {
    std::array<double, 6> signs{{1.0, 1.0, 1.0, 1.0, 1.0, 1.0}};
    if (!dhm.isValid()) return signs;
    if (isUrStyleDhmData(dhm)) {
        // The imported UR-style modified-DH convention reverses J5. Keeping this explicit is
        // important: inferring a sign from axes constructed at a non-zero home pose is ambiguous.
        signs[4] = -1.0;
        return signs;
    }

    constexpr double kEpsilon = 1.0e-4;
    const std::array<double, 6> zeroQ{};
    const CadTransform basePose = viewerFromAr4Transform(ar4ForwardDhm(dhm, zeroQ));
    const CadTransform inverseBasePose = basePose.rigidInverse();

    for (int i = 0; i < 6; ++i) {
        std::array<double, 6> perturbedQ{};
        perturbedQ[static_cast<size_t>(i)] = kEpsilon;
        const CadTransform perturbedPose = viewerFromAr4Transform(ar4ForwardDhm(dhm, perturbedQ));
        const CadTransform delta = perturbedPose * inverseBasePose;
        const Vec3 rotationVector{
            0.5 * (delta.values[9] - delta.values[6]) / kEpsilon,
            0.5 * (delta.values[2] - delta.values[8]) / kEpsilon,
            0.5 * (delta.values[4] - delta.values[1]) / kEpsilon
        };
        const double alignment = dot(rotationVector, makeVec3(axes[static_cast<size_t>(i)].direction));
        signs[static_cast<size_t>(i)] = alignment >= 0.0 ? 1.0 : -1.0;
    }
    return signs;
}

bool isDescendantOf(const CadNode* maybeChild, const CadNode* maybeAncestor) {
    const CadNode* current = maybeChild;
    while (current) {
        if (current == maybeAncestor) return true;
        current = current->parent;
    }
    return false;
}

void collectNodesOfType(CadNode* node, CadNodeType type, std::vector<CadNode*>& out) {
    if (!node) return;
    if (node->type == type) out.push_back(node);
    for (auto& child : node->children) collectNodesOfType(child.get(), type, out);
}

// Deliberately not collectNodesOfType: this one stops at a robot instead of descending through it.
void collectRobotNodesInto(CadNode* node, std::vector<CadNode*>& out) {
    if (!node) return;
    if (node->type == CadNodeType::OPW6Robot) {
        out.push_back(node);
        return;
    }
    for (auto& child : node->children) collectRobotNodesInto(child.get(), out);
}

void collectGantryNodesInto(CadNode* node, std::vector<CadNode*>& out) {
    if (!node) return;
    if (node->type == CadNodeType::GantryMechanism) out.push_back(node);
    for (auto& child : node->children) collectGantryNodesInto(child.get(), out);
}

void collectDragChainNodesInto(CadNode* node, std::vector<CadNode*>& out) {
    if (!node) return;
    if (node->type == CadNodeType::DragChainMechanism) out.push_back(node);
    for (auto& child : node->children) collectDragChainNodesInto(child.get(), out);
}

CadTransform nodeWorldTransform(const CadNode* node) {
    CadTransform world;
    for (const CadNode* current = node; current; current = current->parent) {
        world = current->loc * world;
    }
    return world;
}

CadTransform frameFromAxes(const Vec3& x, const Vec3& hinge, const Vec3& position) {
    Vec3 xAxis = normalized(x);
    // The COLLADA converter remaps local (x,y,z) to viewer (x,z,-y). The source member's hinge is
    // local +Y, therefore the converted mesh hinges about local -Z. Keeping that distinction here
    // prevents the 225 mm chain width being rotated into the U-bend plane.
    Vec3 zAxis = normalized((-1.0 * hinge) - dot(-1.0 * hinge, xAxis) * xAxis);
    Vec3 yAxis = normalized(cross(zAxis, xAxis));
    zAxis = normalized(cross(xAxis, yAxis));
    CadTransform result;
    result.values = {{xAxis.x, yAxis.x, zAxis.x, position.x,
                      xAxis.y, yAxis.y, zAxis.y, position.y,
                      xAxis.z, yAxis.z, zAxis.z, position.z}};
    return result;
}

void collectAxisNodes(CadNode* node, std::vector<CadNode*>& out) {
    if (!node) return;
    if (node->type == CadNodeType::MeshGeometry) {
        const MeshGeometryData* mesh = node->asMeshGeometry();
        if (strutil::containsCaseInsensitive(node->name, "axis") ||
            (mesh && strutil::containsCaseInsensitive(mesh->meshSource, "axis"))) {
            out.push_back(node);
        }
    }
    for (auto& child : node->children) collectAxisNodes(child.get(), out);
}

bool parseJointAxisIndex(const CadNode* node, int& outJointIndex) {
    if (!node) return false;
    const MeshGeometryData* mesh = node->asMeshGeometry();
    const std::string source = mesh ? mesh->meshSource : std::string();
    for (int i = 1; i <= 6; ++i) {
        if (strutil::contains(node->name, std::to_string(i)) ||
            strutil::contains(source, "axis_" + std::to_string(i))) {
            outJointIndex = i;
            return true;
        }
    }
    return false;
}

bool transformHullAabb(const ConvexHullData& hull, const CadTransform& transform,
                       double& xmin, double& ymin, double& zmin,
                       double& xmax, double& ymax, double& zmax) {
    if (hull.vertices.empty()) return false;
    xmin = ymin = zmin = std::numeric_limits<double>::max();
    xmax = ymax = zmax = std::numeric_limits<double>::lowest();

    for (const auto& v : hull.vertices) {
        const Vec3 p = transformPoint(transform, {v[0], v[1], v[2]});
        xmin = std::min<double>(xmin, p.x);
        ymin = std::min<double>(ymin, p.y);
        zmin = std::min<double>(zmin, p.z);
        xmax = std::max<double>(xmax, p.x);
        ymax = std::max<double>(ymax, p.y);
        zmax = std::max<double>(zmax, p.z);
    }
    return true;
}

std::vector<Vec3> transformedHullVertices(const ConvexHullData& hull, const CadTransform& transform) {
    std::vector<Vec3> vertices;
    vertices.reserve(hull.vertices.size());
    for (const auto& v : hull.vertices) {
        vertices.push_back(transformPoint(transform, {v[0], v[1], v[2]}));
    }
    return vertices;
}

bool aabbOverlap(double ax0, double ay0, double az0, double ax1, double ay1, double az1,
                 double bx0, double by0, double bz0, double bx1, double by1, double bz1) {
    return ax0 <= bx1 && ax1 >= bx0 &&
           ay0 <= by1 && ay1 >= by0 &&
           az0 <= bz1 && az1 >= bz0;
}

Vec3 negate(const Vec3& v) {
    return {-v.x, -v.y, -v.z};
}

bool sameDirection(const Vec3& direction, const Vec3& toward) {
    return dot(direction, toward) > 0.0;
}

Vec3 perpendicularTowardOrigin(const Vec3& edge, const Vec3& towardOrigin) {
    Vec3 direction = cross(cross(edge, towardOrigin), edge);
    if (magnitudeSquared(direction) <= 1.0e-12) {
        direction = std::abs(edge.x) < std::abs(edge.y)
            ? cross(edge, {1.0, 0.0, 0.0})
            : cross(edge, {0.0, 1.0, 0.0});
    }
    return direction;
}

Vec3 furthestPoint(const std::vector<Vec3>& vertices, const Vec3& direction) {
    Vec3 best{};
    double bestDot = std::numeric_limits<double>::lowest();
    for (const Vec3& point : vertices) {
        const double value = dot(point, direction);
        if (value > bestDot) {
            bestDot = value;
            best = point;
        }
    }
    return best;
}

Vec3 supportMinkowski(const std::vector<Vec3>& aVertices,
                      const std::vector<Vec3>& bVertices,
                      const Vec3& direction) {
    return furthestPoint(aVertices, direction) - furthestPoint(bVertices, negate(direction));
}

bool updateGjkLine(std::vector<Vec3>& simplex, Vec3& direction) {
    const Vec3 a = simplex[1];
    const Vec3 b = simplex[0];
    const Vec3 ab = b - a;
    const Vec3 ao = negate(a);
    if (sameDirection(ab, ao)) {
        direction = perpendicularTowardOrigin(ab, ao);
    } else {
        simplex = {a};
        direction = ao;
    }
    return false;
}

bool updateGjkTriangle(std::vector<Vec3>& simplex, Vec3& direction) {
    const Vec3 a = simplex[2];
    const Vec3 b = simplex[1];
    const Vec3 c = simplex[0];
    const Vec3 ab = b - a;
    const Vec3 ac = c - a;
    const Vec3 ao = negate(a);
    const Vec3 abc = cross(ab, ac);

    const Vec3 acNormal = cross(abc, ac);
    if (sameDirection(acNormal, ao)) {
        if (sameDirection(ac, ao)) {
            simplex = {c, a};
            direction = perpendicularTowardOrigin(ac, ao);
        } else {
            simplex = {b, a};
            return updateGjkLine(simplex, direction);
        }
        return false;
    }

    const Vec3 abNormal = cross(ab, abc);
    if (sameDirection(abNormal, ao)) {
        simplex = {b, a};
        return updateGjkLine(simplex, direction);
    }

    if (sameDirection(abc, ao)) {
        direction = abc;
    } else {
        simplex = {b, c, a};
        direction = negate(abc);
    }
    return false;
}

bool updateGjkTetrahedron(std::vector<Vec3>& simplex, Vec3& direction) {
    const Vec3 a = simplex[3];
    const Vec3 b = simplex[2];
    const Vec3 c = simplex[1];
    const Vec3 d = simplex[0];
    const Vec3 ao = negate(a);

    const Vec3 abc = cross(b - a, c - a);
    if (sameDirection(abc, ao)) {
        simplex = {c, b, a};
        direction = abc;
        return false;
    }

    const Vec3 acd = cross(c - a, d - a);
    if (sameDirection(acd, ao)) {
        simplex = {d, c, a};
        direction = acd;
        return false;
    }

    const Vec3 adb = cross(d - a, b - a);
    if (sameDirection(adb, ao)) {
        simplex = {b, d, a};
        direction = adb;
        return false;
    }

    return true;
}

bool updateGjkSimplex(std::vector<Vec3>& simplex, Vec3& direction) {
    switch (simplex.size()) {
    case 2:
        return updateGjkLine(simplex, direction);
    case 3:
        return updateGjkTriangle(simplex, direction);
    case 4:
        return updateGjkTetrahedron(simplex, direction);
    default:
        break;
    }
    direction = negate(simplex.back());
    return false;
}

bool convexHullsOverlapGJK(const std::vector<Vec3>& aVertices,
                           const std::vector<Vec3>& bVertices) {
    if (aVertices.empty() || bVertices.empty()) return false;

    Vec3 direction = bVertices.front() - aVertices.front();
    if (magnitudeSquared(direction) <= 1.0e-12) direction = {1.0, 0.0, 0.0};

    std::vector<Vec3> simplex;
    simplex.reserve(4);
    simplex.push_back(supportMinkowski(aVertices, bVertices, direction));
    direction = negate(simplex.back());

    constexpr int kMaxIterations = 64;
    constexpr double kProgressTolerance = 1.0e-9;
    for (int iteration = 0; iteration < kMaxIterations; ++iteration) {
        if (magnitudeSquared(direction) <= 1.0e-12) return true;
        const Vec3 point = supportMinkowski(aVertices, bVertices, direction);
        if (dot(point, direction) < -kProgressTolerance) return false;
        simplex.push_back(point);
        if (updateGjkSimplex(simplex, direction)) return true;
    }

    return true;
}

bool convexHullsOverlap(const std::vector<Vec3>& aVertices,
                        const std::vector<Vec3>& bVertices) {
    return convexHullsOverlapGJK(aVertices, bVertices);
}

bool hullIsValid(const ConvexHullData& hull) {
    if (hull.vertices.size() < 4 || hull.indices.empty() || (hull.indices.size() % 1) != 0) return false;
    for (const auto& tri : hull.indices) {
        for (uint32_t index : tri) {
            if (index >= hull.vertices.size()) return false;
        }
    }
    return true;
}

bool boundsFromMeshNode(const CadNode* meshNode, double& xmin, double& ymin, double& zmin,
                        double& xmax, double& ymax, double& zmax) {
    const MeshGeometryData* mesh = meshNode ? meshNode->asMeshGeometry() : nullptr;
    if (!mesh || !mesh->loaded || mesh->vertices.empty()) return false;

    xmin = ymin = zmin = std::numeric_limits<double>::max();
    xmax = ymax = zmax = std::numeric_limits<double>::lowest();
    const CadTransform& transform = meshNode->loc;
    for (size_t i = 0; i + 2 < mesh->vertices.size(); i += 3) {
        const Vec3 p = transformPoint(transform, {mesh->vertices[i], mesh->vertices[i + 1], mesh->vertices[i + 2]});
        xmin = std::min<double>(xmin, p.x);
        ymin = std::min<double>(ymin, p.y);
        zmin = std::min<double>(zmin, p.z);
        xmax = std::max<double>(xmax, p.x);
        ymax = std::max<double>(ymax, p.y);
        zmax = std::max<double>(zmax, p.z);
    }
    return xmin <= xmax && ymin <= ymax && zmin <= zmax;
}

} // namespace

CadNode* findOPW6RobotNode(CadNode* root) {
    if (!root) return nullptr;
    if (root->type == CadNodeType::OPW6Robot) return root;
    for (auto& child : root->children) {
        if (CadNode* found = findOPW6RobotNode(child.get())) return found;
    }
    return nullptr;
}

const CadNode* findOPW6RobotNode(const CadNode* root) {
    return findOPW6RobotNode(const_cast<CadNode*>(root));
}

CadNode* findGantryMechanismNode(CadNode* root) {
    if (!root) return nullptr;
    if (root->type == CadNodeType::GantryMechanism) return root;
    for (auto& child : root->children) {
        if (CadNode* found = findGantryMechanismNode(child.get())) return found;
    }
    return nullptr;
}

const CadNode* findGantryMechanismNode(const CadNode* root) {
    return findGantryMechanismNode(const_cast<CadNode*>(root));
}

std::vector<CadNode*> collectGantryMechanismNodes(CadNode* root) {
    std::vector<CadNode*> gantries;
    collectGantryNodesInto(root, gantries);
    return gantries;
}

std::vector<CadNode*> collectDragChainMechanismNodes(CadNode* root) {
    std::vector<CadNode*> chains;
    collectDragChainNodesInto(root, chains);
    return chains;
}

std::vector<CadNode*> collectRobotNodes(CadNode* root) {
    std::vector<CadNode*> robots;
    collectRobotNodesInto(root, robots);
    return robots;
}

bool GantryPoseController::bind(CadNode* root, std::string* errorMessage) {
    return bindToGantry(findGantryMechanismNode(root), errorMessage);
}

bool GantryPoseController::bindToGantry(CadNode* gantryNode, std::string* errorMessage) {
    m_gantryNode = gantryNode;
    m_data = m_gantryNode ? m_gantryNode->asGantryMechanism() : nullptr;
    m_movingFrame = m_data ? m_data->movingFrame : nullptr;
    m_positionMm = 0.0;
    m_axis = {{1.0, 0.0, 0.0}};

    if (!m_gantryNode || !m_data) {
        if (errorMessage) *errorMessage = "Package does not contain a GantryMechanism node.";
        return false;
    }
    if (!m_movingFrame || m_movingFrame == m_gantryNode) {
        if (errorMessage) *errorMessage = "Gantry mechanism has no valid moving frame.";
        return false;
    }
    if (m_data->baseCollisionHulls.size() > GantryMechanismData::kMaxBaseCollisionHulls) {
        if (errorMessage) *errorMessage = "Gantry base collision exceeds the 128-hull limit.";
        return false;
    }
    for (const ConvexHullData& hull : m_data->baseCollisionHulls) {
        if (!hullIsValid(hull)) {
            if (errorMessage) *errorMessage = "Gantry base contains a malformed collision hull.";
            return false;
        }
    }

    const Vec3 axis = normalized(Vec3{m_data->axisOfTravel.x, m_data->axisOfTravel.y,
                                      m_data->axisOfTravel.z});
    if (magnitudeSquared(axis) <= 1.0e-12) {
        if (errorMessage) *errorMessage = "Gantry mechanism axis is zero length.";
        return false;
    }
    if (!std::isfinite(m_data->positionMm) || !std::isfinite(m_data->homePositionMm) ||
        !std::isfinite(m_data->lowerLimitMm) || !std::isfinite(m_data->upperLimitMm) ||
        m_data->lowerLimitMm > m_data->upperLimitMm) {
        if (errorMessage) *errorMessage = "Gantry mechanism has invalid position limits.";
        return false;
    }

    m_axis = {{axis.x, axis.y, axis.z}};
    if (m_data->positionApplied) {
        CadTransform removePreviousPosition;
        removePreviousPosition.values[3] = -m_axis[0] * m_data->positionMm;
        removePreviousPosition.values[7] = -m_axis[1] * m_data->positionMm;
        removePreviousPosition.values[11] = -m_axis[2] * m_data->positionMm;
        m_movingFrameBindLoc = removePreviousPosition * m_movingFrame->loc;
    } else {
        m_movingFrameBindLoc = m_movingFrame->loc;
    }
    m_data->positionApplied = true;
    setPositionMm(m_data->positionMm);
    if (errorMessage) errorMessage->clear();
    return true;
}

void GantryPoseController::setPositionMm(double positionMm) {
    if (!isBound() || !std::isfinite(positionMm)) return;
    m_positionMm = clampValue(positionMm, m_data->lowerLimitMm, m_data->upperLimitMm);
    m_data->positionMm = m_positionMm;

    CadTransform translation;
    translation.values[3] = m_axis[0] * m_positionMm;
    translation.values[7] = m_axis[1] * m_positionMm;
    translation.values[11] = m_axis[2] * m_positionMm;
    m_movingFrame->loc = translation * m_movingFrameBindLoc;
    m_movingFrame->needsGlobalLocUpdate = true;
}

void GantryPoseController::resetHome() {
    if (m_data) setPositionMm(m_data->homePositionMm);
}

bool DragChainPoseController::bindToDragChain(CadNode* chainNode, std::string* errorMessage) {
    m_chainNode = chainNode;
    m_data = m_chainNode ? m_chainNode->asDragChainMechanism() : nullptr;
    if (!m_data) {
        if (errorMessage) *errorMessage = "Package does not contain a DragChainMechanism node.";
        return false;
    }
    if (!m_data->movingFrame || !m_data->prototypeGeometry ||
        !m_data->prototypeGeometry->asMeshGeometry() || m_data->linkFrames.size() < 2) {
        if (errorMessage) *errorMessage = "Drag chain references are incomplete.";
        return false;
    }
    if (!(m_data->pitchMm > 0.0) || !(m_data->bendRadiusMm > 0.0) ||
        !(m_data->maxJointRotationDeg > 0.0) || !(m_data->maxJointRotationDeg < 180.0)) {
        if (errorMessage) {
            *errorMessage = "Drag chain pitch/bend radius must be positive and max joint rotation must be between 0 and 180 degrees.";
        }
        return false;
    }
    update();
    if (errorMessage) errorMessage->clear();
    return true;
}

void DragChainPoseController::update() {
    if (!isBound()) return;
    constexpr double kPi = 3.14159265358979323846;

    const CadTransform chainWorld = nodeWorldTransform(m_chainNode);
    const CadTransform movingWorld = nodeWorldTransform(m_data->movingFrame);
    const CadTransform movingInChain = chainWorld.rigidInverse() * movingWorld;
    const Vec3 fixed{m_data->fixedAnchorMm.x, m_data->fixedAnchorMm.y, m_data->fixedAnchorMm.z};
    const Vec3 fixedEndMemberOffset{m_data->fixedEndMemberOffsetMm.x,
                                    m_data->fixedEndMemberOffsetMm.y,
                                    m_data->fixedEndMemberOffsetMm.z};
    const Vec3 moving = transformPoint(movingInChain,
        {m_data->movingAnchorMm.x, m_data->movingAnchorMm.y, m_data->movingAnchorMm.z});
    const Vec3 travel = normalized(Vec3{m_data->travelAxis.x, m_data->travelAxis.y, m_data->travelAxis.z});
    const Vec3 hinge = normalized(Vec3{m_data->hingeAxis.x, m_data->hingeAxis.y, m_data->hingeAxis.z});
    const Vec3 delta = fixed - moving;
    Vec3 up = normalized(delta - dot(delta, travel) * travel);
    if (magnitudeSquared(up) <= 1.0e-12 || magnitudeSquared(travel) <= 1.0e-12 ||
        magnitudeSquared(hinge) <= 1.0e-12) return;

    // Each frame owns one complete member from local hinge 0 to local hinge +pitch. N members
    // therefore provide N pitches of contour; treating the frames themselves as joint stations
    // shortened the chain by one member and left its far hinge visibly floating.
    const double contourLength = m_data->linkFrames.size() * m_data->pitchMm;
    const double radiusFromAnchors = 0.5 * std::sqrt(magnitudeSquared(delta - dot(delta, travel) * travel));
    const double radius = std::min(m_data->bendRadiusMm, radiusFromAnchors);
    const double arcLength = kPi * radius;
    const double straightTotal = std::max(0.0, contourLength - arcLength);
    const double axialDelta = dot(delta, travel);
    const double outgoingSign = std::abs(m_data->departureAxisSign) >= 0.5
        ? (m_data->departureAxisSign > 0.0 ? 1.0 : -1.0)
        : (axialDelta >= 0.0 ? 1.0 : -1.0);
    const Vec3 outgoing = outgoingSign * travel;
    const double outgoingLength = clampValue(
        0.5 * (straightTotal + axialDelta * outgoingSign), 0.0, straightTotal);
    const Vec3 bendStart = moving + outgoingLength * outgoing;
    const Vec3 bendCenter = bendStart + radius * up;

    std::vector<Vec3> positions(m_data->linkFrames.size());
    std::vector<Vec3> curveTangents(m_data->linkFrames.size());
    for (size_t index = 0; index < m_data->linkFrames.size(); ++index) {
        const double s = static_cast<double>(index) * m_data->pitchMm;
        Vec3 position;
        Vec3 tangent;
        if (s <= outgoingLength) {
            position = moving + s * outgoing;
            tangent = outgoing;
        } else if (s < outgoingLength + arcLength && radius > 1.0e-9) {
            const double angle = (s - outgoingLength) / radius;
            position = bendCenter - (radius * std::cos(angle)) * up
                       + (radius * std::sin(angle)) * outgoing;
            tangent = std::sin(angle) * up + std::cos(angle) * outgoing;
        } else {
            const double returnDistance = s - outgoingLength - arcLength;
            position = bendCenter + radius * up - returnDistance * outgoing;
            tangent = -1.0 * outgoing;
        }
        positions[index] = position;
        curveTangents[index] = tangent;
    }

    for (size_t index = 0; index < m_data->linkFrames.size(); ++index) {
        CadNode* frame = m_data->linkFrames[index];
        if (!frame) continue;
        Vec3 memberDirection = curveTangents[index];
        if (index + 1 < positions.size()) {
            const Vec3 chord = positions[index + 1] - positions[index];
            if (magnitudeSquared(chord) > 1.0e-12) memberDirection = normalized(chord);
        }
        // Curve tangents are correct only in the infinitesimal limit. A finite rigid member must
        // face the next hinge (the contour chord), otherwise coarse-pitch carriers visibly open at
        // every bend and cannot seed their revolute joints without an impulse.
        frame->loc = frameFromAxes(memberDirection, hinge, positions[index]);
        frame->needsGlobalLocUpdate = true;
    }

    if (m_data->reverseFixedEndMember) {
        // The member mesh has distinct local-zero and +pitch ends. The carriage connector uses
        // local zero, so turn only the terminal member around and put that same end on the fixed
        // connector. Its +pitch hinge then points back to the preceding member.
        CadNode* fixedEnd = m_data->linkFrames.back();
        if (fixedEnd) {
            fixedEnd->loc = frameFromAxes(outgoing, hinge, fixed + fixedEndMemberOffset);
            fixedEnd->needsGlobalLocUpdate = true;
        }
    }
}

const CadNode* robotBaseFrameNode(const CadNode* robotNode) {
    if (!robotNode) return nullptr;
    const CadNode* parent = robotNode->parent;
    if (parent) {
        const TransformNodeData* frame = parent->asTransform();
        if (frame && frame->isRobotBaseFrame) return parent;
    }
    return robotNode;
}

CadNode* robotBaseFrameNode(CadNode* robotNode) {
    return const_cast<CadNode*>(robotBaseFrameNode(static_cast<const CadNode*>(robotNode)));
}

int ensureRobotBaseFrames(CadNode* root) {
    int inserted = 0;
    for (CadNode* robotNode : collectRobotNodes(root)) {
        if (!robotNode) continue;
        CadNode* parent = robotNode->parent;
        if (!parent) continue;
        if (robotBaseFrameNode(robotNode) != robotNode) continue;

        // Find the arm in its parent and swap a frame into that slot, keeping the shared_ptr alive
        // across the swap. Same index, so sibling order - which is the order the tree and the
        // instance list are read in - does not change.
        for (std::shared_ptr<CadNode>& slot : parent->children) {
            if (slot.get() != robotNode) continue;

            auto frame = std::make_shared<CadNode>();
            frame->type = CadNodeType::Transform;
            auto frameData = std::make_shared<TransformNodeData>();
            frameData->isRobotBaseFrame = true;
            frame->data = frameData;
            frame->name = (robotNode->name.empty() ? std::string("Robot") : robotNode->name) + " base";
            frame->parent = parent;
            frame->loc = robotNode->loc;
            frame->needsGlobalLocUpdate = true;

            std::shared_ptr<CadNode> arm = slot;
            arm->loc = CadTransform();
            arm->parent = frame.get();
            arm->needsGlobalLocUpdate = true;
            frame->children.push_back(arm);

            slot = frame;
            ++inserted;
            break;
        }
    }
    return inserted;
}

std::vector<CadNode*> collectRobotLinks(CadNode* robotNode) {
    std::vector<CadNode*> links;
    collectNodesOfType(robotNode, CadNodeType::RobotLink, links);
    return links;
}

std::vector<CadNode*> collectRobotTools(CadNode* robotNode) {
    std::vector<CadNode*> tools;
    collectNodesOfType(robotNode, CadNodeType::RobotTool, tools);
    return tools;
}

ConvexHullData makeBoxHullFromBounds(double xmin, double ymin, double zmin,
                                     double xmax, double ymax, double zmax) {
    ConvexHullData hull;
    hull.vertices = {{
        {xmin, ymin, zmin}, {xmax, ymin, zmin}, {xmax, ymax, zmin}, {xmin, ymax, zmin},
        {xmin, ymin, zmax}, {xmax, ymin, zmax}, {xmax, ymax, zmax}, {xmin, ymax, zmax}
    }};
    hull.indices = {{
        {0, 1, 2}, {0, 2, 3},
        {4, 6, 5}, {4, 7, 6},
        {0, 4, 5}, {0, 5, 1},
        {1, 5, 6}, {1, 6, 2},
        {2, 6, 7}, {2, 7, 3},
        {3, 7, 4}, {3, 4, 0}
    }};
    return hull;
}

bool bakeRobotCollisionHulls(CadNode* root, std::string* errorMessage) {
    CadNode* robotNode = findOPW6RobotNode(root);
    if (!robotNode) {
        if (errorMessage) *errorMessage = "Package does not contain an OPW6Robot node.";
        return false;
    }

    bool bakedAny = false;
    const std::vector<CadNode*> links = collectRobotLinks(robotNode);
    for (CadNode* linkNode : links) {
        RobotLinkData* link = linkNode ? linkNode->asRobotLink() : nullptr;
        if (!link) continue;
        link->collisionHulls.clear();
        for (CadNode* geometryNode : link->geometryNodes) {
            double xmin, ymin, zmin, xmax, ymax, zmax;
            if (!boundsFromMeshNode(geometryNode, xmin, ymin, zmin, xmax, ymax, zmax)) continue;
            link->collisionHulls.push_back(makeBoxHullFromBounds(xmin, ymin, zmin, xmax, ymax, zmax));
        }
        bakedAny = bakedAny || !link->collisionHulls.empty();
    }

    const std::vector<CadNode*> tools = collectRobotTools(robotNode);
    for (CadNode* toolNode : tools) {
        RobotToolData* tool = toolNode ? toolNode->asRobotTool() : nullptr;
        if (!tool) continue;
        tool->collisionHulls.clear();
        for (CadNode* geometryNode : tool->geometryNodes) {
            double xmin, ymin, zmin, xmax, ymax, zmax;
            if (!boundsFromMeshNode(geometryNode, xmin, ymin, zmin, xmax, ymax, zmax)) continue;
            tool->collisionHulls.push_back(makeBoxHullFromBounds(xmin, ymin, zmin, xmax, ymax, zmax));
        }
    }

    if (!bakedAny) {
        if (errorMessage) *errorMessage = "No loaded mesh geometry was available for hull baking.";
        return false;
    }
    return true;
}

bool validateRobotPackage(CadNode* root, std::string* errorMessage) {
    CadNode* robotNode = findOPW6RobotNode(root);
    if (!robotNode || !robotNode->asOPW6Robot()) {
        if (errorMessage) *errorMessage = "Package does not contain an OPW6Robot node.";
        return false;
    }
    const OPW6RobotData* robotData = robotNode->asOPW6Robot();
    if (!robotData->dhm.isValid()) {
        if (errorMessage) *errorMessage = "Robot package must contain a finite 24-value dhm array.";
        return false;
    }

    const std::vector<CadNode*> links = collectRobotLinks(robotNode);
    if (links.size() < 7) {
        if (errorMessage) *errorMessage = strutil::format("Expected at least 7 RobotLink nodes, found %1.").arg(links.size());
        return false;
    }

    for (size_t i = 0; i < links.size(); ++i) {
        const RobotLinkData* link = links[i] ? links[i]->asRobotLink() : nullptr;
        if (!link) continue;
        if (link->geometryNodes.empty()) {
            if (errorMessage) *errorMessage = strutil::format("Robot link %1 has no visual geometry references.").arg(i);
            return false;
        }
        for (const CadNode* geometryNode : link->geometryNodes) {
            const MeshGeometryData* mesh = geometryNode ? geometryNode->asMeshGeometry() : nullptr;
            if (!mesh || !mesh->loaded || mesh->vertices.empty() || mesh->indices.empty()) {
                if (errorMessage) *errorMessage = strutil::format("Robot link %1 references malformed or unloaded mesh geometry.").arg(i);
                return false;
            }
        }
        if (link->collisionHulls.empty()) {
            if (errorMessage) *errorMessage = strutil::format("Robot link %1 has no serialized collision hull.").arg(i);
            return false;
        }
        for (const ConvexHullData& hull : link->collisionHulls) {
            if (!hullIsValid(hull)) {
                if (errorMessage) *errorMessage = strutil::format("Robot link %1 contains a malformed collision hull.").arg(i);
                return false;
            }
        }
    }
    return true;
}

std::vector<std::string> describeRobotJointAxes(CadNode* root) {
    struct ReportAxis {
        std::array<double, 3> origin{{0.0, 0.0, 0.0}};
        std::array<double, 3> direction{{0.0, 0.0, 0.0}};
        bool valid = false;
    };

    std::vector<std::string> lines;
    CadNode* robotNode = findOPW6RobotNode(root);
    if (!robotNode) {
        lines << "Package does not contain an OPW6Robot node.";
        return lines;
    }

    std::array<ReportAxis, 6> axes{};
    if (const OPW6RobotData* robotData = robotNode->asOPW6Robot()) {
        loadDhmJointAxes(*robotData, axes);
    }

    lines << strutil::format("Valid DHM-derived joint axes: %1/6").arg(validAxisCount(axes));
    for (int i = 0; i < 6; ++i) {
        const ReportAxis& axis = axes[static_cast<size_t>(i)];
        if (!axis.valid || magnitudeSquared(makeVec3(axis.direction)) <= 1.0e-12) {
            lines << strutil::format("J%1: invalid").arg(i + 1);
            continue;
        }
        lines << strutil::format("J%1: origin=(%2, %3, %4), direction=(%5, %6, %7)")
            .arg(i + 1)
            .arg(axis.origin[0], 0, 'f', 3)
            .arg(axis.origin[1], 0, 'f', 3)
            .arg(axis.origin[2], 0, 'f', 3)
            .arg(axis.direction[0], 0, 'f', 6)
            .arg(axis.direction[1], 0, 'f', 6)
            .arg(axis.direction[2], 0, 'f', 6);
    }
    lines << strutil::format("Kinematics source: %1").arg(allAxesValid(axes) ? "DHM-derived joint axes" : "invalid package");
    return lines;
}

bool RobotPoseController::bind(CadNode* root, std::string* errorMessage) {
    return bindToRobot(findOPW6RobotNode(root), errorMessage);
}

bool RobotPoseController::bindToRobot(CadNode* robotNode, std::string* errorMessage) {
    m_robotNode = robotNode;
    if (!m_robotNode) {
        if (errorMessage) *errorMessage = "Package does not contain an OPW6Robot node.";
        return false;
    }
    m_robotData = m_robotNode->asOPW6Robot();
    if (!m_robotData) {
        if (errorMessage) *errorMessage = "OPW6Robot node has missing robot data.";
        return false;
    }

    m_q = m_robotData->q;
    m_jointAxes = {};
    loadDhmJointAxes(*m_robotData, m_jointAxes);
    m_linkBindings.clear();
    m_geometryBindings.clear();
    m_axisBindings.clear();
    m_toolBindPose = CadTransform();

    const std::vector<CadNode*> links = collectRobotLinks(m_robotNode);
    for (size_t i = 0; i < links.size(); ++i) {
        CadNode* linkNode = links[i];
        RobotLinkData* linkData = linkNode ? linkNode->asRobotLink() : nullptr;
        if (!linkNode || !linkData) continue;
        const int linkIndex = std::min<int>(static_cast<int>(i), 6);
        m_linkBindings.push_back({linkNode, linkNode->loc, linkIndex});
        for (CadNode* geometryNode : linkData->geometryNodes) {
            if (!geometryNode || isDescendantOf(geometryNode, linkNode)) continue;
            m_geometryBindings.push_back({geometryNode, geometryNode->loc, linkIndex});
        }
    }

    const std::vector<CadNode*> tools = collectRobotTools(m_robotNode);
    for (CadNode* toolNode : tools) {
        RobotToolData* toolData = toolNode ? toolNode->asRobotTool() : nullptr;
        if (!toolNode || !toolData) continue;
        m_linkBindings.push_back({toolNode, toolNode->loc, 6});
        for (CadNode* geometryNode : toolData->geometryNodes) {
            if (!geometryNode || isDescendantOf(geometryNode, toolNode)) continue;
            m_geometryBindings.push_back({geometryNode, geometryNode->loc, 6});
        }
    }

    std::vector<CadNode*> axisNodes;
    collectAxisNodes(m_robotNode, axisNodes);
    for (CadNode* axisNode : axisNodes) {
        int jointIndex = 0;
        if (!parseJointAxisIndex(axisNode, jointIndex)) continue;
        m_axisBindings.push_back({axisNode, axisNode->loc, std::max(0, jointIndex - 1)});
    }
    if (!allAxesValid(m_jointAxes)) {
        if (errorMessage) {
            *errorMessage = strutil::format("Robot package DHM table produced only %1/6 valid joint axes.")
                .arg(validAxisCount(m_jointAxes));
        }
        return false;
    }
    if (!refreshActiveToolBind(errorMessage)) return false;

    applyPose();
    return true;
}

bool RobotPoseController::refreshActiveToolBind(std::string* errorMessage) {
    if (!m_robotData || !m_robotNode) {
        if (errorMessage) *errorMessage = "Robot pose controller is not bound.";
        return false;
    }

    CadNode* selected = m_robotData->activeTool;
    if (!selected) {
        for (const Binding& binding : m_linkBindings) {
            if (binding.node && binding.node->asRobotTool()) {
                selected = binding.node;
                m_robotData->activeTool = selected;
                break;
            }
        }
    }
    if (!selected) {
        if (errorMessage) *errorMessage = "Robot has no tool or flange TCP.";
        return false;
    }

    const Binding* selectedBinding = nullptr;
    for (const Binding& binding : m_linkBindings) {
        if (binding.node == selected) {
            selectedBinding = &binding;
            break;
        }
    }
    RobotToolData* tool = selected->asRobotTool();
    if (!selectedBinding || !tool) {
        if (errorMessage) *errorMessage = "Active tool is not part of this robot's captured bindings.";
        return false;
    }

    CadTransform tcpLoc;
    if (!tool->tcps.empty()) {
        tool->activeTcpIndex = std::max(
            0, std::min(tool->activeTcpIndex, static_cast<int>(tool->tcps.size()) - 1));
        tcpLoc = tool->tcps[static_cast<size_t>(tool->activeTcpIndex)].loc;
    } else {
        tool->activeTcpIndex = 0;
    }
    // bindLoc is the immutable home-pose transform captured once by bindToRobot. toolNode->loc is
    // the currently posed transform and must never be used here: doing so makes every switch apply
    // the current joint delta a second time.
    m_toolBindPose = selectedBinding->bindLoc * tcpLoc;
    if (errorMessage) errorMessage->clear();
    return true;
}

bool RobotPoseController::registerAttachedTool(CadNode* toolNode, std::string* errorMessage) {
    if (!m_robotData || !m_robotNode) {
        if (errorMessage) *errorMessage = "Robot pose controller is not bound.";
        return false;
    }
    RobotToolData* tool = toolNode ? toolNode->asRobotTool() : nullptr;
    if (!tool) {
        if (errorMessage) *errorMessage = "Attached node is not a robot tool.";
        return false;
    }
    for (const Binding& binding : m_linkBindings) {
        if (binding.node == toolNode) {
            if (errorMessage) errorMessage->clear();
            return true;
        }
    }

    const std::array<CadTransform, 7> linkDeltas =
        axisLinkDeltas(m_jointAxes, m_q, m_robotData->qHome);
    const CadTransform bindLoc = linkDeltas[6].rigidInverse() * toolNode->loc;
    m_linkBindings.push_back({toolNode, bindLoc, 6});
    for (CadNode* geometryNode : tool->geometryNodes) {
        if (!geometryNode || isDescendantOf(geometryNode, toolNode)) continue;
        const CadTransform geometryBind =
            linkDeltas[6].rigidInverse() * geometryNode->loc;
        m_geometryBindings.push_back({geometryNode, geometryBind, 6});
    }
    if (errorMessage) errorMessage->clear();
    return true;
}

void RobotPoseController::setJoints(const std::array<double, 6>& q) {
    if (!m_robotData) return;
    m_q = q;
    m_robotData->q = q;
    applyPose();
}

CadTransform RobotPoseController::toolPose() const {
    if (!m_robotData) return CadTransform();
    const std::array<CadTransform, 7> linkDeltas = axisLinkDeltas(m_jointAxes, m_q, m_robotData->qHome);
    return linkDeltas[6] * m_toolBindPose;
}

CadTransform RobotPoseController::baseWorldTransform() const {
    CadTransform world;
    for (const CadNode* node = m_robotNode; node != nullptr; node = node->parent) {
        world = node->loc * world;
    }
    return world;
}

RobotKinematicSnapshot RobotPoseController::kinematicSnapshot() const {
    RobotKinematicSnapshot snapshot;
    if (!m_robotData) return snapshot;
    snapshot.baseWorldTransform = baseWorldTransform();
    for (size_t index = 0; index < snapshot.axisOrigin.size(); ++index) {
        snapshot.axisOrigin[index] = m_jointAxes[index].origin;
        snapshot.axisDirection[index] = m_jointAxes[index].direction;
    }
    snapshot.qHome = m_robotData->qHome;
    snapshot.toolBindPose = m_toolBindPose;
    snapshot.valid = true;
    return snapshot;
}

CadTransform toolPoseFromKinematicSnapshot(const RobotKinematicSnapshot& snapshot,
                                          const std::array<double, 6>& q) {
    if (!snapshot.valid) return CadTransform();
    // Mirrors toolPose() exactly: same axisLinkDeltas walk, same tool bind composition.
    struct SnapshotAxis {
        std::array<double, 3> origin{};
        std::array<double, 3> direction{};
    };
    std::array<SnapshotAxis, 6> axes{};
    for (size_t index = 0; index < axes.size(); ++index) {
        axes[index].origin = snapshot.axisOrigin[index];
        axes[index].direction = snapshot.axisDirection[index];
    }
    const std::array<CadTransform, 7> linkDeltas = axisLinkDeltas(axes, q, snapshot.qHome);
    return linkDeltas[6] * snapshot.toolBindPose;
}

CadTransform worldToolPoseFromKinematicSnapshot(const RobotKinematicSnapshot& snapshot,
                                                const std::array<double, 6>& q) {
    if (!snapshot.valid) return CadTransform();
    return snapshot.baseWorldTransform * toolPoseFromKinematicSnapshot(snapshot, q);
}

RobotMotionCore::RobotModel RobotPoseController::motionModel() const {
    RobotMotionCore::RobotModel model{};
    model.toolBindPose = RobotMotionCore::identityTransform();
    if (!m_robotData || !m_robotData->dhm.isValid() || !allAxesValid(m_jointAxes)) {
        return model;
    }
    for (int i = 0; i < 24; ++i) {
        model.dhm[i] = m_robotData->dhm.values[static_cast<size_t>(i)];
    }
    const std::array<double, 6> signs = dhmSignsForViewerAxes(m_robotData->dhm, m_jointAxes);
    for (int i = 0; i < 6; ++i) {
        const size_t index = static_cast<size_t>(i);
        model.qHome[i] = m_robotData->qHome[index];
        model.qMin[i] = m_robotData->qMin[index];
        model.qMax[i] = m_robotData->qMax[index];
        model.dhmSigns[i] = signs[index];
    }
    for (int i = 0; i < 12; ++i) {
        model.toolBindPose.values[i] = m_toolBindPose.values[static_cast<size_t>(i)];
    }
    model.valid = 1;
    return model;
}

bool RobotPoseController::setToolPose(const CadTransform& targetPose, std::string* errorMessage) {
    return setToolPoseSolution(targetPose, 0, errorMessage);
}

bool RobotPoseController::setToolPoseSolution(const CadTransform& targetPose, int solutionIndex, std::string* errorMessage) {
    if (!m_robotData) {
        if (errorMessage) *errorMessage = "Robot pose controller is not bound.";
        return false;
    }

    std::array<double, 6> solutions[kMaxToolPoseSolutions]{};
    const int solutionCount = toolPoseSolutions(targetPose, solutions, kMaxToolPoseSolutions, errorMessage);
    if (solutionCount <= 0) {
        if (errorMessage && errorMessage->empty()) *errorMessage = "Tool target is outside the reachable workspace.";
        return false;
    }
    if (solutionIndex < 0 || solutionIndex >= solutionCount) {
        if (errorMessage) *errorMessage = strutil::format("Requested IK solution %1, but only %2 solution(s) are available.")
            .arg(solutionIndex + 1)
            .arg(solutionCount);
        return false;
    }

    setJoints(solutions[solutionIndex]);
    return true;
}

void RobotPoseController::resetHome() {
    if (!m_robotData) return;
    setJoints(m_robotData->qHome);
}

void RobotPoseController::setVisualMeshesVisible(bool visible) {
    if (!m_robotNode) return;
    for (CadNode* linkNode : collectRobotLinks(m_robotNode)) {
        RobotLinkData* link = linkNode ? linkNode->asRobotLink() : nullptr;
        if (!link) continue;
        link->visualEnabled = visible;
        for (CadNode* geometryNode : link->geometryNodes) {
            if (geometryNode) geometryNode->visible = visible;
        }
    }
    for (CadNode* toolNode : collectRobotTools(m_robotNode)) {
        RobotToolData* tool = toolNode ? toolNode->asRobotTool() : nullptr;
        if (!tool) continue;
        tool->visualEnabled = visible;
        for (CadNode* geometryNode : tool->geometryNodes) {
            if (geometryNode) geometryNode->visible = visible;
        }
    }
}

void RobotPoseController::setJointAxesVisible(bool visible) {
    for (const Binding& binding : m_axisBindings) {
        if (binding.node) binding.node->visible = visible;
    }
}

void RobotPoseController::setHullOverlaysVisible(bool visible) {
    if (!m_robotNode) return;
    for (CadNode* linkNode : collectRobotLinks(m_robotNode)) {
        RobotLinkData* link = linkNode ? linkNode->asRobotLink() : nullptr;
        if (link) link->collisionHullsVisible = visible;
    }
    for (CadNode* toolNode : collectRobotTools(m_robotNode)) {
        RobotToolData* tool = toolNode ? toolNode->asRobotTool() : nullptr;
        if (tool) tool->collisionHullsVisible = visible;
    }
}

void RobotPoseController::applyPose() {
    if (!m_robotData) return;
    const std::array<CadTransform, 7> linkDeltas = axisLinkDeltas(m_jointAxes, m_q, m_robotData->qHome);

    auto applyBinding = [&](const Binding& binding) {
        if (!binding.node) return;
        const CadTransform& delta = linkDeltas[static_cast<size_t>(std::max(0, std::min(6, binding.linkIndex)))];
        binding.node->loc = delta * binding.bindLoc;
        binding.node->needsGlobalLocUpdate = true;
    };

    for (const Binding& binding : m_linkBindings) applyBinding(binding);
    for (const Binding& binding : m_geometryBindings) applyBinding(binding);
    for (const Binding& binding : m_axisBindings) applyBinding(binding);
}

bool RobotPoseController::solveToolPose(const CadTransform& targetPose, std::array<double, 6>& solvedQ, std::string* errorMessage) const {
    std::array<double, 6> solutions[kMaxToolPoseSolutions]{};
    const int solutionCount = toolPoseSolutions(targetPose, solutions, kMaxToolPoseSolutions, errorMessage);
    if (solutionCount <= 0) return false;
    solvedQ = solutions[0];
    return true;
}

int RobotPoseController::toolPoseSolutions(const CadTransform& targetPose,
                                           std::array<double, 6>* solutions,
                                           int solutionCapacity,
                                           std::string* errorMessage) const {
    if (!solutions || solutionCapacity <= 0) {
        if (errorMessage) *errorMessage = "No IK solution output buffer was provided.";
        return 0;
    }
    if (!m_robotData || !allAxesValid(m_jointAxes)) {
        if (errorMessage) *errorMessage = "Robot pose controller is not bound to valid joint axes.";
        return 0;
    }
    if (!m_robotData->dhm.isValid()) {
        if (errorMessage) *errorMessage = "Robot package has no valid DHM table.";
        return 0;
    }

    auto poseFor = [&](const std::array<double, 6>& q) {
        const std::array<CadTransform, 7> linkDeltas = axisLinkDeltas(m_jointAxes, q, m_robotData->qHome);
        return linkDeltas[6] * m_toolBindPose;
    };

    const std::array<double, 6> dhmSigns = dhmSignsForViewerAxes(m_robotData->dhm, m_jointAxes);
    std::array<double, 6> homeDhmQ{};
    for (int i = 0; i < 6; ++i) {
        homeDhmQ[static_cast<size_t>(i)] = dhmSigns[static_cast<size_t>(i)] * m_robotData->qHome[static_cast<size_t>(i)];
    }

    const CadTransform ar4Home = ar4ForwardDhm(m_robotData->dhm, homeDhmQ);
    const CadTransform ar4HomeInViewerFrame = viewerFromAr4Transform(ar4Home);
    const CadTransform targetAr4ViewerFrame = targetPose * m_toolBindPose.rigidInverse() * ar4HomeInViewerFrame;
    const CadTransform targetAr4Pose = ar4FromViewerTransform(targetAr4ViewerFrame);

    std::array<double, 6> referenceDhmQ{};
    for (int i = 0; i < 6; ++i) {
        referenceDhmQ[static_cast<size_t>(i)] = dhmSigns[static_cast<size_t>(i)] * m_q[static_cast<size_t>(i)];
    }

    std::array<double, 6> candidateDhmQs[12]{};
    int candidateCount = 0;
    const RobotMotionCore::RobotModel motionModelData = motionModel();
    if (RobotMotionCore::isUrStyleDhm(motionModelData)) {
        RobotMotionCore::Transform motionTarget{};
        for (int i = 0; i < 12; ++i) motionTarget.values[i] = targetAr4Pose.values[static_cast<size_t>(i)];
        double urCandidates[12][RobotMotionCore::kAxisCount]{};
        candidateCount = RobotMotionCore::urStyleAnalyticInverseDhm(
            motionModelData, motionTarget, referenceDhmQ.data(), urCandidates, 12);
        for (int candidate = 0; candidate < candidateCount; ++candidate) {
            for (int joint = 0; joint < 6; ++joint) {
                candidateDhmQs[candidate][static_cast<size_t>(joint)] = urCandidates[candidate][joint];
            }
        }
    } else {
        candidateCount = ar4AnalyticInverseDhm(m_robotData->dhm, targetAr4Pose, referenceDhmQ, candidateDhmQs, 12);
    }
    if (candidateCount <= 0) {
        if (errorMessage) *errorMessage = "Closed-form analytic IK found no candidate for the tool target.";
        return 0;
    }

    struct ToolSolutionCandidate {
        std::array<double, 6> q{};
        double jointDistance = 0.0;
    };

    ToolSolutionCandidate validCandidates[kMaxToolPoseSolutions]{};
    int validCount = 0;
    double closestRejectedPositionError = std::numeric_limits<double>::max();
    double closestRejectedRotationError = std::numeric_limits<double>::max();
    std::array<double, 6> closestRejectedQ{};
    int rejectedByLimits = 0;
    int rejectedByMismatch = 0;

    for (int candidateIndex = 0; candidateIndex < candidateCount; ++candidateIndex) {
        std::array<double, 6> q{};
        for (int i = 0; i < 6; ++i) {
            q[static_cast<size_t>(i)] = dhmSigns[static_cast<size_t>(i)] * candidateDhmQs[candidateIndex][static_cast<size_t>(i)];
        }
        bool withinLimits = true;
        double jointDistance = 0.0;
        for (int i = 0; i < 6; ++i) {
            const size_t index = static_cast<size_t>(i);
            q[index] = nearestEquivalentRadians(q[index], m_q[index]);
            const double minValue = m_robotData->qMin[index];
            const double maxValue = m_robotData->qMax[index];
            if (minValue < maxValue && (q[index] < minValue - 1.0e-6 || q[index] > maxValue + 1.0e-6)) {
                withinLimits = false;
                break;
            }
            const double delta = wrapRadians(q[index] - m_q[index]);
            jointDistance += delta * delta;
        }
        if (!withinLimits) {
            ++rejectedByLimits;
            continue;
        }

        const CadTransform candidatePose = poseFor(q);
        const double positionError = posePositionError(candidatePose, targetPose);
        const double rotationError = rotationErrorAngle(candidatePose, targetPose);
        if (positionError > kIkFkPositionToleranceMm || rotationError > kIkFkRotationToleranceRad) {
            ++rejectedByMismatch;
            if (positionError + 1000.0 * rotationError < closestRejectedPositionError + 1000.0 * closestRejectedRotationError) {
                closestRejectedPositionError = positionError;
                closestRejectedRotationError = rotationError;
                closestRejectedQ = q;
            }
            continue;
        }

        if (validCount < kMaxToolPoseSolutions) {
            validCandidates[validCount++] = {q, jointDistance};
        }
    }

    if (validCount <= 0 && errorMessage) {
        if (rejectedByLimits == candidateCount) {
            *errorMessage = strutil::format("Closed-form analytic IK found %1 candidate(s), all outside joint limits.").arg(candidateCount);
        } else if (rejectedByMismatch > 0) {
            *errorMessage = strutil::format("Closed-form analytic IK found %1 candidate(s), but package FK mismatch remained %2 mm / %3 rad at q=[%4,%5,%6,%7,%8,%9] deg.")
                .arg(candidateCount)
                .arg(closestRejectedPositionError, 0, 'f', 6)
                .arg(closestRejectedRotationError, 0, 'f', 6)
                .arg(closestRejectedQ[0] * kRadToDeg, 0, 'f', 3)
                .arg(closestRejectedQ[1] * kRadToDeg, 0, 'f', 3)
                .arg(closestRejectedQ[2] * kRadToDeg, 0, 'f', 3)
                .arg(closestRejectedQ[3] * kRadToDeg, 0, 'f', 3)
                .arg(closestRejectedQ[4] * kRadToDeg, 0, 'f', 3)
                .arg(closestRejectedQ[5] * kRadToDeg, 0, 'f', 3);
        } else {
            *errorMessage = strutil::format("Closed-form analytic IK found %1 candidate(s), but none were usable.").arg(candidateCount);
        }
    }
    if (validCount <= 0) return 0;

    for (int i = 1; i < validCount; ++i) {
        ToolSolutionCandidate item = validCandidates[i];
        int j = i - 1;
        while (j >= 0 && validCandidates[j].jointDistance > item.jointDistance) {
            validCandidates[j + 1] = validCandidates[j];
            --j;
        }
        validCandidates[j + 1] = item;
    }

    const int outputCount = std::min(validCount, solutionCapacity);
    for (int i = 0; i < outputCount; ++i) {
        solutions[i] = validCandidates[i].q;
    }
    if (errorMessage) errorMessage->clear();
    return outputCount;
}

bool RobotCollisionModel::bind(CadNode* root, std::string* errorMessage) {
    return bindToRobot(findOPW6RobotNode(root), errorMessage);
}

bool RobotCollisionModel::bindToRobot(CadNode* robotNode, std::string* errorMessage) {
    if (!robotNode) {
        if (errorMessage) *errorMessage = "Package does not contain an OPW6Robot node.";
        return false;
    }
    m_robotData = robotNode->asOPW6Robot();
    if (!m_robotData) {
        if (errorMessage) *errorMessage = "OPW6Robot node has missing robot data.";
        return false;
    }
    m_jointAxes = {};
    loadDhmJointAxes(*m_robotData, m_jointAxes);
    m_hulls.clear();
    if (!allAxesValid(m_jointAxes)) {
        if (errorMessage) {
            *errorMessage = strutil::format("Robot package DHM table produced only %1/6 valid joint axes.")
                .arg(validAxisCount(m_jointAxes));
        }
        return false;
    }

    const std::vector<CadNode*> links = collectRobotLinks(robotNode);
    for (size_t i = 0; i < links.size(); ++i) {
        const RobotLinkData* link = links[i] ? links[i]->asRobotLink() : nullptr;
        if (!link) continue;
        const int linkIndex = std::min<int>(static_cast<int>(i), 6);
        if (!link->collisionHulls.empty()) {
            for (const ConvexHullData& hull : link->collisionHulls) {
                m_hulls.push_back({linkIndex, hull});
            }
        } else {
            for (const CadNode* geometryNode : link->geometryNodes) {
                double xmin, ymin, zmin, xmax, ymax, zmax;
                if (!boundsFromMeshNode(geometryNode, xmin, ymin, zmin, xmax, ymax, zmax)) continue;
                m_hulls.push_back({linkIndex, makeBoxHullFromBounds(xmin, ymin, zmin, xmax, ymax, zmax)});
            }
        }
    }

    if (m_hulls.empty()) {
        if (errorMessage) *errorMessage = "Robot package has no link collision hulls or loaded mesh geometry.";
        return false;
    }
    return true;
}

std::vector<RobotCollisionPair> RobotCollisionModel::selfCollisions(const std::array<double, 6>& q) const {
    std::vector<RobotCollisionPair> pairs;
    if (!m_robotData) return pairs;

    const std::array<CadTransform, 7> linkDeltas = axisLinkDeltas(m_jointAxes, q, m_robotData->qHome);
    struct WorldHull {
        int linkIndex = -1;
        const ConvexHullData* hull = nullptr;
        std::vector<Vec3> vertices;
        double xmin, ymin, zmin, xmax, ymax, zmax;
    };
    std::vector<WorldHull> worldHulls;
    for (const HullBinding& binding : m_hulls) {
        WorldHull worldHull;
        worldHull.linkIndex = binding.linkIndex;
        worldHull.hull = &binding.hull;
        const CadTransform& delta = linkDeltas[static_cast<size_t>(std::max(0, std::min(6, binding.linkIndex)))];
        if (transformHullAabb(binding.hull, delta,
                              worldHull.xmin, worldHull.ymin, worldHull.zmin, worldHull.xmax, worldHull.ymax, worldHull.zmax)) {
            worldHull.vertices = transformedHullVertices(binding.hull, delta);
            worldHulls.push_back(std::move(worldHull));
        }
    }

    std::set<std::pair<int, int>> uniquePairs;
    for (size_t i = 0; i < worldHulls.size(); ++i) {
        for (size_t j = i + 1; j < worldHulls.size(); ++j) {
            if (worldHulls[i].linkIndex == worldHulls[j].linkIndex) continue;
            const int a = std::min(worldHulls[i].linkIndex, worldHulls[j].linkIndex);
            const int b = std::max(worldHulls[i].linkIndex, worldHulls[j].linkIndex);
            if (isIgnored(a, b)) continue;
            if (!aabbOverlap(worldHulls[i].xmin, worldHulls[i].ymin, worldHulls[i].zmin, worldHulls[i].xmax, worldHulls[i].ymax, worldHulls[i].zmax,
                             worldHulls[j].xmin, worldHulls[j].ymin, worldHulls[j].zmin, worldHulls[j].xmax, worldHulls[j].ymax, worldHulls[j].zmax)) {
                continue;
            }
            if (worldHulls[i].hull && worldHulls[j].hull &&
                convexHullsOverlap(worldHulls[i].vertices, worldHulls[j].vertices)) {
                uniquePairs.insert({a, b});
            }
        }
    }

    for (const auto& pair : uniquePairs) {
        pairs.push_back({pair.first, pair.second});
    }
    return pairs;
}

bool RobotCollisionModel::isIgnored(int a, int b) const {
    if (!m_robotData) return false;
    for (const RobotCollisionIgnore& ignore : m_robotData->collisionIgnores) {
        const int ia = std::min(ignore.a, ignore.b);
        const int ib = std::max(ignore.a, ignore.b);
        if (ia == a && ib == b) return true;
    }
    return false;
}
