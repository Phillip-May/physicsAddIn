#include "ViewRay.h"

#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <utility>

namespace viewray {
namespace {

constexpr double kPi = 3.14159265358979323846;

CadVec3 axisOf(const CadTransform& transform, int column) {
    return CadVec3(transform.values[column], transform.values[4 + column],
                   transform.values[8 + column]);
}

CadVec3 originOf(const CadTransform& transform) {
    return CadVec3(transform.values[3], transform.values[7], transform.values[11]);
}

CadVec3 sum(const CadVec3& a, const CadVec3& b) { return CadVec3(a.x + b.x, a.y + b.y, a.z + b.z); }

CadVec3 normalised(const CadVec3& value) {
    const double length = std::sqrt(dot(value, value));
    return length > 1.0e-12 ? scaled(value, 1.0 / length) : CadVec3(0.0, 0.0, 0.0);
}

// The camera's placement in the world, whichever way round the host reports it. A view pose is rigid, so
// the inverse is the transposed rotation and the negated, rotated translation - no general inverse needed.
struct Placement {
    CadVec3 eye;
    CadVec3 right;
    CadVec3 up;
    CadVec3 forward;
};

Placement placementOf(const Camera& camera) {
    Placement placement;
    const CadVec3 x = axisOf(camera.pose, 0);
    const CadVec3 y = axisOf(camera.pose, 1);
    const CadVec3 z = axisOf(camera.pose, 2);
    const CadVec3 translation = originOf(camera.pose);
    if (camera.poseIsWorldFromCamera) {
        placement.eye = translation;
        placement.right = x;
        placement.up = y;
        placement.forward = camera.looksDownNegativeZ ? scaled(z, -1.0) : z;
    } else {
        // A camera-from-world pose: the rotation is the transpose of the camera's axes in the world, and
        // the eye is -R^T t. Rigid, so nothing here needs a general inverse.
        placement.right = CadVec3(x.x, y.x, z.x);
        placement.up = CadVec3(x.y, y.y, z.y);
        const CadVec3 back(x.z, y.z, z.z);
        placement.forward = camera.looksDownNegativeZ ? scaled(back, -1.0) : back;
        placement.eye = CadVec3(-dot(x, translation), -dot(y, translation), -dot(z, translation));
    }
    placement.right = normalised(placement.right);
    placement.up = normalised(placement.up);
    placement.forward = normalised(placement.forward);
    return placement;
}

} // namespace

double focalLengthPx(const Camera& camera) {
    if (camera.widthPx <= 0 || camera.heightPx <= 0) return 0.0;
    if (camera.viewAngleDeg <= 0.0 || camera.viewAngleDeg >= 180.0) return 0.0;
    // Half the span the angle measures, over the tangent of half the angle. Same relation either way; only
    // which span it is changes.
    const double half = 0.5 * camera.viewAngleDeg * kPi / 180.0;
    const double span = camera.angleIsVertical ? camera.heightPx : camera.widthPx;
    return (0.5 * span) / std::tan(half);
}

CadVec3 cameraEye(const Camera& camera) { return placementOf(camera).eye; }

Ray rayThroughPixel(const Camera& camera, double pixelX, double pixelY) {
    Ray ray;
    const double focal = focalLengthPx(camera);
    if (focal <= 0.0) return ray;
    const Placement placement = placementOf(camera);
    // Pixels come from Qt with y down the screen; the camera's up axis is up, hence the negation.
    const double offsetX = pixelX - 0.5 * camera.widthPx;
    const double offsetY = -(pixelY - 0.5 * camera.heightPx);
    ray.origin = placement.eye;
    ray.direction = normalised(sum(sum(scaled(placement.forward, focal),
                                       scaled(placement.right, offsetX)),
                                   scaled(placement.up, offsetY)));
    return ray;
}

bool projectPoint(const Camera& camera, const CadVec3& world, double* pixelX, double* pixelY) {
    const double focal = focalLengthPx(camera);
    if (focal <= 0.0) return false;
    const Placement placement = placementOf(camera);
    const CadVec3 relative = difference(world, placement.eye);
    const double depth = dot(relative, placement.forward);
    if (depth <= 1.0e-9) return false; // behind the eye, or on it
    if (pixelX) *pixelX = 0.5 * camera.widthPx + focal * dot(relative, placement.right) / depth;
    if (pixelY) *pixelY = 0.5 * camera.heightPx - focal * dot(relative, placement.up) / depth;
    return true;
}

bool rayHitsTriangles(const Ray& ray, const std::vector<float>& verticesMm, int triangles,
                      double* distanceMm) {
    // Moller-Trumbore, nearest hit, both faces. Both faces because a conveyor's deck is a closed shell and
    // an operator clicking it from below means the same thing as clicking it from above.
    double nearest = std::numeric_limits<double>::max();
    bool hit = false;
    for (int triangle = 0; triangle < triangles; ++triangle) {
        const size_t at = static_cast<size_t>(triangle) * 9;
        if (at + 8 >= verticesMm.size()) break;
        const CadVec3 a(verticesMm[at], verticesMm[at + 1], verticesMm[at + 2]);
        const CadVec3 b(verticesMm[at + 3], verticesMm[at + 4], verticesMm[at + 5]);
        const CadVec3 c(verticesMm[at + 6], verticesMm[at + 7], verticesMm[at + 8]);

        const CadVec3 edge1 = difference(b, a);
        const CadVec3 edge2 = difference(c, a);
        const CadVec3 across = cross(ray.direction, edge2);
        const double determinant = dot(edge1, across);
        if (std::fabs(determinant) < 1.0e-12) continue; // parallel to the triangle's plane
        const double inverse = 1.0 / determinant;
        const CadVec3 toA = difference(ray.origin, a);
        const double u = inverse * dot(toA, across);
        if (u < 0.0 || u > 1.0) continue;
        const CadVec3 along = cross(toA, edge1);
        const double v = inverse * dot(ray.direction, along);
        if (v < 0.0 || u + v > 1.0) continue;
        const double distance = inverse * dot(edge2, along);
        if (distance <= 1.0e-6 || distance >= nearest) continue;
        nearest = distance;
        hit = true;
    }
    if (hit && distanceMm) *distanceMm = nearest;
    return hit;
}

MeshEdges buildMeshEdges(const std::vector<float>& verticesMm, int triangles) {
    MeshEdges result;
    // Welded on a hundredth of a millimetre. A triangle soup repeats every shared vertex, so without this
    // no two faces would ever be found to share an edge and every edge would read as a boundary - which
    // would make the "silhouette" the whole wireframe.
    const double weld = 100.0;
    std::map<std::array<long long, 3>, int> byPosition;
    std::vector<CadVec3> points;
    std::vector<int> corner;
    corner.reserve(static_cast<size_t>(triangles) * 3);
    for (int triangle = 0; triangle < triangles; ++triangle) {
        const size_t at = static_cast<size_t>(triangle) * 9;
        if (at + 8 >= verticesMm.size()) break;
        for (int vertex = 0; vertex < 3; ++vertex) {
            const CadVec3 point(verticesMm[at + vertex * 3], verticesMm[at + vertex * 3 + 1],
                                verticesMm[at + vertex * 3 + 2]);
            const std::array<long long, 3> key{static_cast<long long>(std::llround(point.x * weld)),
                                               static_cast<long long>(std::llround(point.y * weld)),
                                               static_cast<long long>(std::llround(point.z * weld))};
            auto found = byPosition.find(key);
            if (found == byPosition.end()) {
                found = byPosition.emplace(key, static_cast<int>(points.size())).first;
                points.push_back(point);
            }
            corner.push_back(found->second);
        }
    }

    // Edge -> the one or two faces along it.
    std::map<std::pair<int, int>, std::pair<CadVec3, int>> along;
    std::map<std::pair<int, int>, CadVec3> second;
    const size_t faces = corner.size() / 3;
    for (size_t face = 0; face < faces; ++face) {
        const CadVec3& a = points[static_cast<size_t>(corner[face * 3])];
        const CadVec3& b = points[static_cast<size_t>(corner[face * 3 + 1])];
        const CadVec3& c = points[static_cast<size_t>(corner[face * 3 + 2])];
        const CadVec3 normal = normalised(cross(difference(b, a), difference(c, a)));
        for (int edge = 0; edge < 3; ++edge) {
            int from = corner[face * 3 + edge];
            int to = corner[face * 3 + (edge + 1) % 3];
            if (from > to) std::swap(from, to);
            if (from == to) continue;
            const std::pair<int, int> key(from, to);
            auto found = along.find(key);
            if (found == along.end()) {
                along.emplace(key, std::make_pair(normal, 1));
            } else if (found->second.second == 1) {
                found->second.second = 2;
                second.emplace(key, normal);
            }
        }
    }

    result.endpointsMm.reserve(along.size() * 6);
    result.faceNormals.reserve(along.size() * 6);
    for (const auto& entry : along) {
        const CadVec3& from = points[static_cast<size_t>(entry.first.first)];
        const CadVec3& to = points[static_cast<size_t>(entry.first.second)];
        const CadVec3& first = entry.second.first;
        // A boundary edge repeats its one normal, so the facing test below always calls it a silhouette -
        // which is right: nothing is on the other side of it.
        auto other = second.find(entry.first);
        const CadVec3 back = other == second.end() ? scaled(first, -1.0) : other->second;
        const float values[6] = {static_cast<float>(from.x), static_cast<float>(from.y),
                                 static_cast<float>(from.z), static_cast<float>(to.x),
                                 static_cast<float>(to.y),   static_cast<float>(to.z)};
        for (const float value : values) result.endpointsMm.push_back(value);
        const float normals[6] = {static_cast<float>(first.x), static_cast<float>(first.y),
                                  static_cast<float>(first.z), static_cast<float>(back.x),
                                  static_cast<float>(back.y),  static_cast<float>(back.z)};
        for (const float value : normals) result.faceNormals.push_back(value);
        ++result.edges;
    }
    return result;
}

void silhouetteLines(const MeshEdges& edges, const CadVec3& eyeMm, std::vector<float>* linesMm) {
    if (!linesMm) return;
    linesMm->clear();
    for (int edge = 0; edge < edges.edges; ++edge) {
        const size_t at = static_cast<size_t>(edge) * 6;
        if (at + 5 >= edges.endpointsMm.size() || at + 5 >= edges.faceNormals.size()) break;
        const CadVec3 from(edges.endpointsMm[at], edges.endpointsMm[at + 1], edges.endpointsMm[at + 2]);
        const CadVec3 to(edges.endpointsMm[at + 3], edges.endpointsMm[at + 4], edges.endpointsMm[at + 5]);
        const CadVec3 first(edges.faceNormals[at], edges.faceNormals[at + 1], edges.faceNormals[at + 2]);
        const CadVec3 back(edges.faceNormals[at + 3], edges.faceNormals[at + 4], edges.faceNormals[at + 5]);
        // From the middle of the edge, because a long edge can face the eye at one end and away at the
        // other and its midpoint is the honest question.
        const CadVec3 middle((from.x + to.x) * 0.5, (from.y + to.y) * 0.5, (from.z + to.z) * 0.5);
        const CadVec3 toEye = difference(eyeMm, middle);
        const bool firstFaces = dot(first, toEye) > 0.0;
        const bool backFaces = dot(back, toEye) > 0.0;
        if (firstFaces == backFaces) continue; // both toward the eye or both away: not an outline
        for (int axis = 0; axis < 6; ++axis) linesMm->push_back(edges.endpointsMm[at + axis]);
    }
}

} // namespace viewray
