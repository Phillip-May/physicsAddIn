#include "AccessoryGeometry.h"

#include "UnitsMath.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

void accessoryAppendBox(MeshGeometryData& mesh, float cx, float cy, float cz,
                        float sx, float sy, float sz) {
    const float hx = sx * 0.5f, hy = sy * 0.5f, hz = sz * 0.5f;
    const uint32_t base = static_cast<uint32_t>(mesh.vertices.size() / 3);
    const float vertices[] = {
        cx - hx, cy - hy, cz - hz, cx + hx, cy - hy, cz - hz,
        cx + hx, cy + hy, cz - hz, cx - hx, cy + hy, cz - hz,
        cx - hx, cy - hy, cz + hz, cx + hx, cy - hy, cz + hz,
        cx + hx, cy + hy, cz + hz, cx - hx, cy + hy, cz + hz,
    };
    mesh.vertices.insert(mesh.vertices.end(), std::begin(vertices), std::end(vertices));
    const uint32_t triangles[] = {
        0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7,
        0, 1, 5, 0, 5, 4, 3, 7, 6, 3, 6, 2,
        0, 4, 7, 0, 7, 3, 1, 2, 6, 1, 6, 5,
    };
    for (uint32_t index : triangles) mesh.indices.push_back(base + index);
}

void accessoryAppendOrientedBox(MeshGeometryData& mesh,
                                const CadVec3& center,
                                const CadVec3& forward,
                                const CadVec3& up,
                                const CadVec3& side,
                                float forwardSize, float upSize, float sideSize) {
    const double halfForward = forwardSize * 0.5;
    const double halfUp = upSize * 0.5;
    const double halfSide = sideSize * 0.5;
    const uint32_t base = static_cast<uint32_t>(mesh.vertices.size() / 3);
    for (int sideSign : {-1, 1}) {
        for (int upSign : {-1, 1}) {
            for (int forwardSign : {-1, 1}) {
                const CadVec3 point(
                    center.x + forward.x * halfForward * forwardSign +
                        up.x * halfUp * upSign + side.x * halfSide * sideSign,
                    center.y + forward.y * halfForward * forwardSign +
                        up.y * halfUp * upSign + side.y * halfSide * sideSign,
                    center.z + forward.z * halfForward * forwardSign +
                        up.z * halfUp * upSign + side.z * halfSide * sideSign);
                mesh.vertices.insert(mesh.vertices.end(), {
                    static_cast<float>(point.x), static_cast<float>(point.y),
                    static_cast<float>(point.z)});
            }
        }
    }
    const uint32_t triangles[] = {
        0, 3, 1, 0, 2, 3, 4, 5, 7, 4, 7, 6,
        0, 1, 5, 0, 5, 4, 2, 6, 7, 2, 7, 3,
        0, 4, 6, 0, 6, 2, 1, 3, 7, 1, 7, 5,
    };
    for (uint32_t index : triangles) mesh.indices.push_back(base + index);
}

void accessoryAppendFaceStroke(MeshGeometryData& mesh,
                               const CadVec3& faceCenter,
                               const CadVec3& faceX,
                               const CadVec3& faceY,
                               const CadVec3& normal,
                               float x0, float y0, float x1, float y1,
                               float thickness, float raisedDepth) {
    CadVec3 direction(faceX.x * (x1 - x0) + faceY.x * (y1 - y0),
                      faceX.y * (x1 - x0) + faceY.y * (y1 - y0),
                      faceX.z * (x1 - x0) + faceY.z * (y1 - y0));
    const double length = std::sqrt(direction.x * direction.x + direction.y * direction.y +
                                    direction.z * direction.z);
    if (length < 1.0e-4) return;
    direction = CadVec3(direction.x / length, direction.y / length, direction.z / length);
    // accessoryAppendOrientedBox expects up == side x forward. direction x normal gives a side
    // vector satisfying that handedness; the reverse cross product culled the raised front face
    // and exposed its coplanar back face, which visibly fought the enclosure.
    CadVec3 across(direction.y * normal.z - direction.z * normal.y,
                   direction.z * normal.x - direction.x * normal.z,
                   direction.x * normal.y - direction.y * normal.x);
    const double acrossLength = std::sqrt(across.x * across.x + across.y * across.y +
                                          across.z * across.z);
    if (acrossLength < 1.0e-4) return;
    across = CadVec3(across.x / acrossLength, across.y / acrossLength, across.z / acrossLength);
    const float midpointX = (x0 + x1) * 0.5f;
    const float midpointY = (y0 + y1) * 0.5f;
    constexpr float faceGap = 1.5f;
    const float outwardOffset = faceGap + raisedDepth * 0.5f;
    const CadVec3 center(faceCenter.x + faceX.x * midpointX + faceY.x * midpointY +
                             normal.x * outwardOffset,
                         faceCenter.y + faceX.y * midpointX + faceY.y * midpointY +
                             normal.y * outwardOffset,
                         faceCenter.z + faceX.z * midpointX + faceY.z * midpointY +
                             normal.z * outwardOffset);
    accessoryAppendOrientedBox(mesh, center, direction, normal, across,
                               static_cast<float>(length), raisedDepth, thickness);
}

void accessoryCollectIconPoints(const CadNode* node, const CadTransform& parent,
                                bool applyNodeTransform,
                                std::vector<AccessoryIconPoint>& points) {
    if (!node) return;
    const CadTransform transform = applyNodeTransform ? parent * node->loc : parent;
    if (const MeshGeometryData* mesh = node->asMeshGeometry(); mesh && mesh->loaded) {
        points.reserve(points.size() + mesh->vertices.size() / 3);
        for (size_t index = 0; index + 2 < mesh->vertices.size(); index += 3) {
            const double x = mesh->vertices[index];
            const double y = mesh->vertices[index + 1];
            const double z = mesh->vertices[index + 2];
            // Front elevation of the actual source object. Depth is intentionally projected away:
            // the role enclosure needs a readable silhouette, not thousands of miniature triangles.
            points.push_back({
                static_cast<float>(transform.values[0] * x + transform.values[1] * y +
                                   transform.values[2] * z + transform.values[3]),
                static_cast<float>(transform.values[4] * x + transform.values[5] * y +
                                   transform.values[6] * z + transform.values[7])});
        }
    }
    for (const std::shared_ptr<CadNode>& child : node->children) {
        accessoryCollectIconPoints(child.get(), transform, true, points);
    }
}

std::vector<AccessoryIconPoint> accessorySpawnerObjectOutline(const CadNode* prototype) {
    std::vector<AccessoryIconPoint> points;
    accessoryCollectIconPoints(prototype, CadTransform(), false, points);
    std::sort(points.begin(), points.end(), [](const AccessoryIconPoint& a,
                                               const AccessoryIconPoint& b) {
        return a.x < b.x || (a.x == b.x && a.y < b.y);
    });
    points.erase(std::unique(points.begin(), points.end(), [](const AccessoryIconPoint& a,
                                                              const AccessoryIconPoint& b) {
        return std::abs(a.x - b.x) < 1.0e-4f && std::abs(a.y - b.y) < 1.0e-4f;
    }), points.end());
    if (points.size() < 3) return points;
    const auto cross = [](const AccessoryIconPoint& a, const AccessoryIconPoint& b,
                          const AccessoryIconPoint& c) {
        return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    };
    std::vector<AccessoryIconPoint> hull(points.size() * 2);
    size_t count = 0;
    for (const AccessoryIconPoint& point : points) {
        while (count >= 2 && cross(hull[count - 2], hull[count - 1], point) <= 0.0f) --count;
        hull[count++] = point;
    }
    const size_t lowerCount = count;
    for (size_t index = points.size() - 1; index-- > 0;) {
        const AccessoryIconPoint& point = points[index];
        while (count > lowerCount && cross(hull[count - 2], hull[count - 1], point) <= 0.0f) {
            --count;
        }
        hull[count++] = point;
    }
    if (count > 1) --count;
    hull.resize(count);
    return hull;
}

void accessoryAppendRoleFaceIcon(MeshGeometryData& mesh,
                                 const CadVec3& faceCenter,
                                 const CadVec3& faceX,
                                 const CadVec3& faceY,
                                 const CadVec3& normal,
                                 float faceWidth, float faceHeight,
                                 bool spawner,
                                 const std::vector<AccessoryIconPoint>& productOutline,

                                 bool rotateIconQuarterTurn) {
    const float unit = std::max(40.0f, std::min(faceWidth, faceHeight) * 0.78f);
    const float thickness = std::max(6.0f, unit * 0.035f);
    constexpr float raisedDepth = 8.0f;
    const auto stroke = [&](float x0, float y0, float x1, float y1) {
        if (rotateIconQuarterTurn) {
            const float rotatedX0 = -y0;
            const float rotatedY0 = x0;
            const float rotatedX1 = -y1;
            const float rotatedY1 = x1;
            x0 = rotatedX0;
            y0 = rotatedY0;
            x1 = rotatedX1;
            y1 = rotatedY1;
        }
        accessoryAppendFaceStroke(mesh, faceCenter, faceX, faceY, normal,
                                  x0, y0, x1, y1, thickness, raisedDepth);
    };
    if (!spawner) {
        stroke(-unit * 0.31f, -unit * 0.31f, unit * 0.31f, unit * 0.31f);
        stroke(-unit * 0.31f, unit * 0.31f, unit * 0.31f, -unit * 0.31f);
        return;
    }

    // Draw a normalized silhouette of the referenced hidden station object above the infinity
    // mark. This is regenerated from that object's loaded mesh, so changing spawnObjectId changes
    // both what is emitted and what the enclosure advertises without a shape-specific code path.
    if (productOutline.size() >= 2) {
        float xmin = productOutline.front().x, xmax = xmin;
        float ymin = productOutline.front().y, ymax = ymin;
        for (const AccessoryIconPoint& point : productOutline) {
            xmin = std::min(xmin, point.x); xmax = std::max(xmax, point.x);
            ymin = std::min(ymin, point.y); ymax = std::max(ymax, point.y);
        }
        const float scale = std::min(unit * 0.42f / std::max(1.0e-3f, xmax - xmin),
                                     unit * 0.25f / std::max(1.0e-3f, ymax - ymin));
        const float cx = (xmin + xmax) * 0.5f;
        const float cy = (ymin + ymax) * 0.5f;
        for (size_t index = 0; index < productOutline.size(); ++index) {
            const AccessoryIconPoint& a = productOutline[index];
            const AccessoryIconPoint& b = productOutline[(index + 1) % productOutline.size()];
            stroke((a.x - cx) * scale, unit * 0.18f + (a.y - cy) * scale,
                   (b.x - cx) * scale, unit * 0.18f + (b.y - cy) * scale);
        }
    }
    constexpr int infinitySegments = 32;
    constexpr float infinityCenterY = -0.23f;
    for (int segment = 0; segment < infinitySegments; ++segment) {
        const float t0 = static_cast<float>(segment) / infinitySegments *
                         6.28318530717958647692f;
        const float t1 = static_cast<float>(segment + 1) / infinitySegments *
                         6.28318530717958647692f;
        stroke(unit * 0.25f * std::sin(t0),
               unit * (infinityCenterY + 0.09f * std::sin(t0 * 2.0f)),
               unit * 0.25f * std::sin(t1),
               unit * (infinityCenterY + 0.09f * std::sin(t1 * 2.0f)));
    }
}

void accessoryAppendSegmentBox(MeshGeometryData& mesh,
                               float ax, float ay, float az,
                               float bx, float by, float bz,
                               float height, float width) {
    float fx = bx - ax, fy = by - ay, fz = bz - az;
    const float length = std::sqrt(fx * fx + fy * fy + fz * fz);
    if (length < 1.0e-4f) return;
    fx /= length; fy /= length; fz /= length;
    const float horizontal = std::max(1.0e-5f, std::sqrt(fx * fx + fz * fz));
    const float sx = -fz / horizontal, sy = 0.0f, sz = fx / horizontal;
    // side x forward gives an upright local Y for a level +X segment.
    const float ux = -sz * fy;
    const float uy = sz * fx - sx * fz;
    const float uz = sx * fy;
    const float cx = (ax + bx) * 0.5f, cy = (ay + by) * 0.5f,
                cz = (az + bz) * 0.5f;
    const float hx = length * 0.5f, hy = height * 0.5f, hz = width * 0.5f;
    const uint32_t base = static_cast<uint32_t>(mesh.vertices.size() / 3);
    for (int zSign : {-1, 1}) {
        for (int ySign : {-1, 1}) {
            for (int xSign : {-1, 1}) {
                const float lx = hx * static_cast<float>(xSign);
                const float ly = hy * static_cast<float>(ySign);
                const float lz = hz * static_cast<float>(zSign);
                mesh.vertices.insert(mesh.vertices.end(), {
                    cx + fx * lx + ux * ly + sx * lz,
                    cy + fy * lx + uy * ly + sy * lz,
                    cz + fz * lx + uz * ly + sz * lz});
            }
        }
    }
    // Vertex order is (z, y, x), producing the same cube topology each call.
    const uint32_t triangles[] = {
        0, 3, 1, 0, 2, 3, 4, 5, 7, 4, 7, 6,
        0, 1, 5, 0, 5, 4, 2, 6, 7, 2, 7, 3,
        0, 4, 6, 0, 6, 2, 1, 3, 7, 1, 7, 5,
    };
    for (uint32_t index : triangles) mesh.indices.push_back(base + index);
}

void accessoryAppendProfiledWall(
    MeshGeometryData& mesh,
    const std::vector<std::array<CadVec3, 4>>& sections) {
    // Each section is {outer-bottom, inner-bottom, outer-top, inner-top}. Connect the complete
    // strip before adding end caps: making every interval a closed box creates overlapping angled
    // caps (visible as teeth) wherever a wall follows a narrowing deck profile.
    if (sections.size() < 2) return;
    const uint32_t base = static_cast<uint32_t>(mesh.vertices.size() / 3);
    for (const auto& section : sections) {
        for (const CadVec3& point : section) {
            mesh.vertices.insert(mesh.vertices.end(), {
                static_cast<float>(point.x),
                static_cast<float>(point.y),
                static_cast<float>(point.z)});
        }
    }
    const auto appendDoubleSidedTriangle = [&](uint32_t a, uint32_t b, uint32_t c) {
        mesh.indices.insert(mesh.indices.end(), {
            base + a, base + b, base + c,
            base + a, base + c, base + b});
    };
    for (uint32_t section = 0; section + 1 < sections.size(); ++section) {
        const uint32_t a = section * 4;
        const uint32_t b = (section + 1) * 4;
        appendDoubleSidedTriangle(a + 0, b + 0, b + 2); // outer face
        appendDoubleSidedTriangle(a + 0, b + 2, a + 2);
        appendDoubleSidedTriangle(a + 1, a + 3, b + 3); // inner face
        appendDoubleSidedTriangle(a + 1, b + 3, b + 1);
        appendDoubleSidedTriangle(a + 0, a + 1, b + 1); // bottom
        appendDoubleSidedTriangle(a + 0, b + 1, b + 0);
        appendDoubleSidedTriangle(a + 2, b + 2, b + 3); // top
        appendDoubleSidedTriangle(a + 2, b + 3, a + 3);
    }
    const uint32_t first = 0;
    const uint32_t last = static_cast<uint32_t>((sections.size() - 1) * 4);
    appendDoubleSidedTriangle(first + 0, first + 2, first + 3);
    appendDoubleSidedTriangle(first + 0, first + 3, first + 1);
    appendDoubleSidedTriangle(last + 0, last + 1, last + 3);
    appendDoubleSidedTriangle(last + 0, last + 3, last + 2);
}

void accessoryAppendHorizontalCylinder(MeshGeometryData& mesh, float centerX, float centerY,
                                       float centerZ, float heading, float halfWidth,
                                       float radius, float rightDropMm,
                                       int segments) {
    constexpr float kTwoPi = 6.28318530717958647692f;
    const float forwardX = std::cos(heading), forwardZ = std::sin(heading);
    const float sideX = -forwardZ, sideZ = forwardX;
    const float sideY = halfWidth > 1.0e-4f ? -rightDropMm / (halfWidth * 2.0f) : 0.0f;
    const float sideLength = std::sqrt(1.0f + sideY * sideY);
    const float axisX = sideX / sideLength;
    const float axisY = sideY / sideLength;
    const float axisZ = sideZ / sideLength;
    // axis x forward is the roller's radial "up" direction, including the bank angle.
    const float upX = axisY * forwardZ;
    const float upY = axisZ * forwardX - axisX * forwardZ;
    const float upZ = -axisY * forwardX;
    const uint32_t base = static_cast<uint32_t>(mesh.vertices.size() / 3);
    for (float axial : {-halfWidth, halfWidth}) {
        for (int segment = 0; segment < segments; ++segment) {
            const float angle = kTwoPi * static_cast<float>(segment) /
                                static_cast<float>(segments);
            const float radialForward = radius * std::cos(angle);
            const float radialUp = radius * std::sin(angle);
            mesh.vertices.insert(mesh.vertices.end(), {
                centerX + sideX * axial + forwardX * radialForward + upX * radialUp,
                centerY + sideY * axial + upY * radialUp,
                centerZ + sideZ * axial + forwardZ * radialForward + upZ * radialUp});
        }
    }
    for (int segment = 0; segment < segments; ++segment) {
        const uint32_t next = static_cast<uint32_t>((segment + 1) % segments);
        const uint32_t a = base + static_cast<uint32_t>(segment);
        const uint32_t b = base + next;
        const uint32_t c = base + static_cast<uint32_t>(segments + segment);
        const uint32_t d = base + static_cast<uint32_t>(segments) + next;
        mesh.indices.insert(mesh.indices.end(), {a, b, d, a, d, c});
    }
    for (int end = 0; end < 2; ++end) {
        const float axial = end == 0 ? -halfWidth : halfWidth;
        const uint32_t center = static_cast<uint32_t>(mesh.vertices.size() / 3);
        mesh.vertices.insert(mesh.vertices.end(), {
            centerX + sideX * axial, centerY + sideY * axial,
            centerZ + sideZ * axial});
        for (int segment = 0; segment < segments; ++segment) {
            const uint32_t next = static_cast<uint32_t>((segment + 1) % segments);
            const uint32_t ring = base + static_cast<uint32_t>(end * segments);
            if (end == 0) {
                mesh.indices.insert(mesh.indices.end(),
                                    {center, ring + next, ring + static_cast<uint32_t>(segment)});
            } else {
                mesh.indices.insert(mesh.indices.end(),
                                    {center, ring + static_cast<uint32_t>(segment), ring + next});
            }
        }
    }
}

void accessoryAppendPerforatedCell(MeshGeometryData& mesh, float centerX, float centerZ,
                                   float yBottom, float yTop, float cellSize,
                                   float holeRadius) {
    const float half = cellSize * 0.5f;
    const std::array<std::array<float, 2>, 8> outer{{
        {{-half, -half}}, {{0.0f, -half}}, {{half, -half}}, {{half, 0.0f}},
        {{half, half}}, {{0.0f, half}}, {{-half, half}}, {{-half, 0.0f}},
    }};
    std::array<std::array<float, 2>, 8> inner{};
    for (size_t i = 0; i < outer.size(); ++i) {
        const float length = std::hypot(outer[i][0], outer[i][1]);
        inner[i] = {{holeRadius * outer[i][0] / length,
                     holeRadius * outer[i][1] / length}};
    }

    const uint32_t base = static_cast<uint32_t>(mesh.vertices.size() / 3);
    for (float y : {yBottom, yTop}) {
        for (const auto& point : outer) {
            mesh.vertices.insert(mesh.vertices.end(),
                                 {centerX + point[0], y, centerZ + point[1]});
        }
        for (const auto& point : inner) {
            mesh.vertices.insert(mesh.vertices.end(),
                                 {centerX + point[0], y, centerZ + point[1]});
        }
    }
    const uint32_t bottomOuter = base, bottomInner = base + 8;
    const uint32_t topOuter = base + 16, topInner = base + 24;
    for (uint32_t i = 0; i < 8; ++i) {
        const uint32_t next = (i + 1) % 8;
        const uint32_t triangles[] = {
            topOuter + i, topOuter + next, topInner + next,
            topOuter + i, topInner + next, topInner + i,
            bottomOuter + i, bottomInner + next, bottomOuter + next,
            bottomOuter + i, bottomInner + i, bottomInner + next,
            bottomOuter + i, bottomOuter + next, topOuter + next,
            bottomOuter + i, topOuter + next, topOuter + i,
            bottomInner + i, topInner + next, bottomInner + next,
            bottomInner + i, topInner + i, topInner + next,
        };
        mesh.indices.insert(mesh.indices.end(), std::begin(triangles), std::end(triangles));
    }
}

void accessoryFinalizeMesh(MeshGeometryData& mesh) {
    if (mesh.vertices.size() < 3) {
        mesh.bounds = {{0, 0, 0, 0, 0, 0}};
        mesh.loaded = true;
        return;
    }
    float xmin = mesh.vertices[0], ymin = mesh.vertices[1], zmin = mesh.vertices[2];
    float xmax = xmin, ymax = ymin, zmax = zmin;
    for (size_t i = 0; i + 2 < mesh.vertices.size(); i += 3) {
        xmin = std::min(xmin, mesh.vertices[i]);
        ymin = std::min(ymin, mesh.vertices[i + 1]);
        zmin = std::min(zmin, mesh.vertices[i + 2]);
        xmax = std::max(xmax, mesh.vertices[i]);
        ymax = std::max(ymax, mesh.vertices[i + 1]);
        zmax = std::max(zmax, mesh.vertices[i + 2]);
    }
    mesh.bounds = {{xmin, ymin, zmin, xmax, ymax, zmax}};
    mesh.loaded = true;
}

int accessoryHoleCount(double dimensionMm, double pitchMm) {
    return std::max(1, static_cast<int>(std::floor(dimensionMm / pitchMm)) - 1);
}

double conveyorRollerRadiusMm(const TransformNodeData& parameters) {
    // Keep a real gap between adjacent rollers at very fine pitches instead of generating
    // intersecting 64 mm rollers. Coarser conveyors retain the established 32 mm maximum radius.
    return std::max(4.0, std::min(kConveyorMaximumRollerRadiusMm,
                                  parameters.accessoryRollerPitchMm * 0.45));
}

double conveyorSurfaceOffsetMm(const TransformNodeData& parameters) {
    return parameters.accessoryRollerCoverEnabled
        ? kConveyorRollerCoverSurfaceOffsetMm : 0.0;
}

double conveyorPathLengthMm(const TransformNodeData& parameters) {
    const double turnRadians = parameters.accessoryTurnAngleDeg * kDegToRad;
    if (std::abs(turnRadians) <= 0.001) return std::max(1.0, parameters.accessoryLengthMm);
    const double radius = std::max(parameters.accessoryCurveRadiusMm,
                                   parameters.accessoryWidthMm * 0.5 + 150.0);
    return std::max(1.0, std::abs(turnRadians) * radius);
}

double conveyorRollerInsetMm(const TransformNodeData& parameters) {
    return std::min(conveyorRollerRadiusMm(parameters),
                    conveyorPathLengthMm(parameters) * 0.10);
}

double conveyorCornerHeightAt(const TransformNodeData& parameters, double t, bool right) {
    const double start = right ? parameters.accessoryStartRightHeightMm
                               : parameters.accessoryStartLeftHeightMm;
    const double end = right ? parameters.accessoryEndRightHeightMm
                             : parameters.accessoryEndLeftHeightMm;
    return start + (end - start) * std::max(0.0, std::min(1.0, t));
}

double conveyorSideHeightOffsetAt(const TransformNodeData& parameters, double t,
                                  double sideSign, double horizontalSideOffsetMm) {
    const double left = conveyorCornerHeightAt(parameters, t, false);
    const double right = conveyorCornerHeightAt(parameters, t, true);
    const double halfWidth = std::max(1.0, parameters.accessoryWidthMm * 0.5);
    return sideSign * (right - left) * 0.5 * horizontalSideOffsetMm / halfWidth;
}

double conveyorRightDropAt(const TransformNodeData& parameters, double t) {
    return conveyorCornerHeightAt(parameters, t, false) -
           conveyorCornerHeightAt(parameters, t, true);
}

ConveyorPathPose conveyorPathPoseAt(const TransformNodeData& parameters, double t) {
    t = std::max(0.0, std::min(1.0, t));
    const float tf = static_cast<float>(t);
    const float length = static_cast<float>(parameters.accessoryLengthMm);
    const float turnRadians = static_cast<float>(parameters.accessoryTurnAngleDeg * kDegToRad);
    const bool curved = std::abs(turnRadians) > 0.001f;
    const float turnSign = turnRadians < 0.0f ? -1.0f : 1.0f;
    const float radius = std::max(static_cast<float>(parameters.accessoryCurveRadiusMm),
                                  static_cast<float>(parameters.accessoryWidthMm * 0.5 + 150.0));
    const float rawEndX = curved ? turnSign * radius * std::sin(turnRadians) : length;
    const float rawEndZ = curved ? turnSign * radius * (1.0f - std::cos(turnRadians)) : 0.0f;
    ConveyorPathPose pose;
    if (curved) {
        const float angle = turnRadians * tf;
        pose.x = turnSign * radius * std::sin(angle) - rawEndX * 0.5f;
        pose.z = turnSign * radius * (1.0f - std::cos(angle)) - rawEndZ * 0.5f;
        pose.heading = angle;
    } else {
        pose.x = (tf - 0.5f) * length;
    }
    const double leftHeight = conveyorCornerHeightAt(parameters, t, false);
    const double rightHeight = conveyorCornerHeightAt(parameters, t, true);
    pose.y = static_cast<float>((leftHeight + rightHeight) * 0.5);
    return pose;
}

CadVec3 conveyorTransformPoint(const CadTransform& world, const ConveyorPathPose& point) {
    return world * CadVec3(point.x, point.y, point.z);
}

double closestConveyorProgress(const TransformNodeData& parameters, const CadTransform& world,
                               const CadVec3& worldPoint) {
    const int segments = std::max(8, std::min(
        4096, static_cast<int>(std::ceil(conveyorPathLengthMm(parameters) / 60.0))));
    double bestProgress = 0.0;
    double bestDistanceSquared = std::numeric_limits<double>::max();
    CadVec3 a = conveyorTransformPoint(world, conveyorPathPoseAt(parameters, 0.0));
    for (int segment = 0; segment < segments; ++segment) {
        const double t1 = static_cast<double>(segment + 1) / segments;
        const CadVec3 b = conveyorTransformPoint(world, conveyorPathPoseAt(parameters, t1));
        const CadVec3 edge(b.x - a.x, b.y - a.y, b.z - a.z);
        const CadVec3 offset(worldPoint.x - a.x, worldPoint.y - a.y, worldPoint.z - a.z);
        const double edgeLengthSquared = edge.x * edge.x + edge.y * edge.y + edge.z * edge.z;
        const double projected = edgeLengthSquared > 1.0e-9
            ? std::max(0.0, std::min(1.0,
                (offset.x * edge.x + offset.y * edge.y + offset.z * edge.z) /
                    edgeLengthSquared))
            : 0.0;
        const CadVec3 nearest(a.x + edge.x * projected,
                              a.y + edge.y * projected,
                              a.z + edge.z * projected);
        const double dx = worldPoint.x - nearest.x;
        const double dy = worldPoint.y - nearest.y;
        const double dz = worldPoint.z - nearest.z;
        const double distanceSquared = dx * dx + dy * dy + dz * dz;
        if (distanceSquared < bestDistanceSquared) {
            bestDistanceSquared = distanceSquared;
            bestProgress = (static_cast<double>(segment) + projected) / segments;
        }
        a = b;
    }
    return bestProgress;
}

CadVec3 conveyorTangentAt(const TransformNodeData& parameters, const CadTransform& world,
                          double progress, bool forward) {
    const double direction = forward ? 1.0 : -1.0;
    double t0 = std::max(0.0, std::min(1.0, progress));
    double t1 = std::max(0.0, std::min(1.0, t0 + direction * 0.002));
    if (t1 == t0) t1 = std::max(0.0, std::min(1.0, t0 - direction * 0.002));
    const CadVec3 a = conveyorTransformPoint(world, conveyorPathPoseAt(parameters, t0));
    const CadVec3 b = conveyorTransformPoint(world, conveyorPathPoseAt(parameters, t1));
    CadVec3 tangent(b.x - a.x, b.y - a.y, b.z - a.z);
    const double length = std::sqrt(tangent.x * tangent.x + tangent.y * tangent.y +
                                    tangent.z * tangent.z);
    tangent = length > 1.0e-9 ? CadVec3(tangent.x / length, tangent.y / length, tangent.z / length)
                              : CadVec3();
    // The step is clamped at the ends, so asking for the tangent at the far end of a path walks
    // backwards; the sign has to come back with it.
    if ((t1 - t0) * direction < 0.0) {
        tangent = CadVec3(-tangent.x, -tangent.y, -tangent.z);
    }
    return tangent;
}

double robotPickFeederSurfaceHalfWidthAt(const TransformNodeData& parameters, double t) {
    (void)parameters;
    t = std::max(0.0, std::min(1.0, t));
    // A narrow intake feeds a gently converging single-file queue. Every cross-section must stay
    // narrower than two nominal 30 mm boxes or pairs enter abreast and lock when the funnel narrows;
    // the monotonic profile keeps visual geometry, collision guides and powered belt patches aligned.
    constexpr double kInletHalfWidthMm = 33.0;
    constexpr double kPocketHalfWidthMm = 30.0;
    constexpr double kTaperStart = 0.0;
    constexpr double kTaperEnd = 0.45;
    const double u = std::max(0.0, std::min(
        1.0, (t - kTaperStart) / (kTaperEnd - kTaperStart)));
    const double smooth = u * u * (3.0 - 2.0 * u);
    return kInletHalfWidthMm * (1.0 - smooth) + kPocketHalfWidthMm * smooth;
}

double robotPickFeederDeckHalfWidthAt(const TransformNodeData& parameters, double t) {
    // In the terminal pocket the 30 mm cube rides on a compact centre platform instead of a
    // full-width belt slab. Its 26 mm half-width is also the outside edge of the 8 mm walls around
    // a 36 mm-clear chute, so the black surface and wall footprint remain exactly coincident.
    constexpr double kTongueHalfWidthMm = 26.0;
    constexpr double kPocketStart = 0.92;
    constexpr double kPocketOpen = 0.96;
    const double fullWidth = robotPickFeederSurfaceHalfWidthAt(parameters, t);
    const double u = std::max(0.0, std::min(
        1.0, (t - kPocketStart) / (kPocketOpen - kPocketStart)));
    const double smooth = u * u * (3.0 - 2.0 * u);
    return fullWidth * (1.0 - smooth) + kTongueHalfWidthMm * smooth;
}

double robotPickFeederWallCenterOffsetAt(const TransformNodeData& parameters, double t) {
    // Follow the visible black deck into the pickup tongue. Preserve a 36 mm clear channel around
    // the nominal 30 mm workpiece after accounting for the 8 mm-thick walls.
    constexpr double kMinimumWallCenterOffsetMm = 22.0;
    constexpr double kWallHalfThicknessMm = 4.0;
    return std::max(kMinimumWallCenterOffsetMm,
                    robotPickFeederDeckHalfWidthAt(parameters, t) - kWallHalfThicknessMm);
}

double robotPickFeederWallHeightAt(double t) {
    // Keep the queue inside full walls until it is single-file. The last short pickup pocket drops
    // to a containment lip so the top-down gripper can descend around the supported workpiece.
    constexpr double kLipHeightMm = 20.0;
    constexpr double kLowerStart = 0.92;
    constexpr double kLowerEnd = 0.96;
    const double u = std::max(0.0, std::min(
        1.0, (t - kLowerStart) / (kLowerEnd - kLowerStart)));
    const double smooth = u * u * (3.0 - 2.0 * u);
    return kRobotPickFeederFullWallHeightMm * (1.0 - smooth) + kLipHeightMm * smooth;
}

