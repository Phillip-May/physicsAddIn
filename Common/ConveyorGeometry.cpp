#include "ConveyorGeometry.h"

#include "AccessoryGeometry.h"
#include "AccessoryPropertySchema.h"
#include "StringUtil.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <vector>

void clampRollerConveyorParameters(TransformNodeData& parametersRef) {
    TransformNodeData* parameters = &parametersRef;
    parameters->accessoryLengthMm =
        clampAccessoryNumber("lengthMm", parameters->accessoryLengthMm);
    parameters->accessoryWidthMm =
        clampAccessoryNumber("widthMm", parameters->accessoryWidthMm);
    if (parameters->accessoryStartHeightMm <= 0.0) {
        parameters->accessoryStartHeightMm = parameters->accessoryHeightMm;
    }
    if (parameters->accessoryEndHeightMm <= 0.0) {
        parameters->accessoryEndHeightMm = parameters->accessoryHeightMm;
    }
    parameters->accessoryStartHeightMm =
        clampAccessoryNumber("startHeightMm", parameters->accessoryStartHeightMm);
    parameters->accessoryEndHeightMm =
        clampAccessoryNumber("endHeightMm", parameters->accessoryEndHeightMm);
    if (parameters->accessoryStartLeftHeightMm <= 0.0) {
        parameters->accessoryStartLeftHeightMm = parameters->accessoryStartHeightMm;
    }
    if (parameters->accessoryStartRightHeightMm <= 0.0) {
        parameters->accessoryStartRightHeightMm = parameters->accessoryStartHeightMm;
    }
    if (parameters->accessoryEndLeftHeightMm <= 0.0) {
        parameters->accessoryEndLeftHeightMm = parameters->accessoryEndHeightMm;
    }
    if (parameters->accessoryEndRightHeightMm <= 0.0) {
        parameters->accessoryEndRightHeightMm = parameters->accessoryEndHeightMm;
    }
    parameters->accessoryStartLeftHeightMm =
        clampAccessoryNumber("startLeftHeightMm", parameters->accessoryStartLeftHeightMm);
    parameters->accessoryStartRightHeightMm =
        clampAccessoryNumber("startRightHeightMm", parameters->accessoryStartRightHeightMm);
    parameters->accessoryEndLeftHeightMm =
        clampAccessoryNumber("endLeftHeightMm", parameters->accessoryEndLeftHeightMm);
    parameters->accessoryEndRightHeightMm =
        clampAccessoryNumber("endRightHeightMm", parameters->accessoryEndRightHeightMm);
    parameters->accessoryStartHeightMm = 0.5 *
        (parameters->accessoryStartLeftHeightMm + parameters->accessoryStartRightHeightMm);
    parameters->accessoryEndHeightMm = 0.5 *
        (parameters->accessoryEndLeftHeightMm + parameters->accessoryEndRightHeightMm);
    parameters->accessoryHeightMm =
        0.5 * (parameters->accessoryStartHeightMm + parameters->accessoryEndHeightMm);
    parameters->accessoryRollerPitchMm =
        clampAccessoryNumber("rollerPitchMm", parameters->accessoryRollerPitchMm);
    parameters->accessoryCurveRadiusMm =
        clampAccessoryNumber("curveRadiusMm", parameters->accessoryCurveRadiusMm);
    parameters->accessoryTurnAngleDeg =
        clampAccessoryNumber("turnAngleDeg", parameters->accessoryTurnAngleDeg);
    parameters->accessoryConveyorSpeedMmS =
        clampAccessoryNumber("speedMmS", parameters->accessoryConveyorSpeedMmS);
    parameters->accessorySpawnIntervalSeconds =
        clampAccessoryNumber("spawnIntervalSeconds", parameters->accessorySpawnIntervalSeconds);
    parameters->accessoryMaxActiveSpawns =
        clampAccessoryInteger("maxActiveSpawns", parameters->accessoryMaxActiveSpawns);
    parameters->accessoryInitialWorkpieceEndInsetMm = clampAccessoryNumber(
        "initialWorkpieceEndInsetMm", parameters->accessoryInitialWorkpieceEndInsetMm);
    parameters->accessoryInitialWorkpieceSpacingMm = clampAccessoryNumber(
        "initialWorkpieceSpacingMm", parameters->accessoryInitialWorkpieceSpacingMm);
    parameters->accessoryInitialWorkpieceCount = clampAccessoryInteger(
        "initialWorkpieceCount", parameters->accessoryInitialWorkpieceCount);
    if (!accessoryChoiceIsValid("simulationMode", parameters->accessoryConveyorMode)) {
        parameters->accessoryConveyorMode = "global";
    }
    if (!accessoryChoiceIsValid("role", parameters->accessoryConveyorRole)) {
        parameters->accessoryConveyorRole = "normal";
    }
}

bool rebuildRollerConveyor(CadNode* root, const CadNode* spawnPrototype) {
    TransformNodeData* parameters = root ? root->asTransform() : nullptr;
    if (!parameters || parameters->accessoryGenerator != "roller_conveyor" ||
        root->children.size() < 5) {
        return false;
    }
    CadNode* frameNode = root->children[0].get();
    MeshGeometryData* frame = frameNode->asMeshGeometry();
    MeshGeometryData* feet = root->children[1]->asMeshGeometry();
    MeshGeometryData* rollers = root->children[2]->asMeshGeometry();
    CadNode* leftConnection = root->children[3].get();
    CadNode* rightConnection = root->children[4].get();
    if (!frame || !feet || !rollers || !leftConnection->asTransform() ||
        !rightConnection->asTransform()) return false;
    CadNode* coverNode = nullptr;
    for (size_t index = 5; index < root->children.size(); ++index) {
        if (root->children[index] && root->children[index]->name == "Black roller cover" &&
            root->children[index]->asMeshGeometry()) {
            coverNode = root->children[index].get();
            break;
        }
    }
    if (!coverNode) {
        auto generatedCover = std::make_shared<CadNode>();
        generatedCover->name = "Black roller cover";
        generatedCover->type = CadNodeType::MeshGeometry;
        generatedCover->color = CADNodeColor::fromSRGB(20, 22, 24);
        generatedCover->data = std::make_shared<MeshGeometryData>();
        generatedCover->parent = root;
        root->children.push_back(generatedCover);
        coverNode = generatedCover.get();
    }
    MeshGeometryData* cover = coverNode->asMeshGeometry();
    if (!cover) return false;

    const auto ensureGeneratedRoleMesh = [&](const char* name, const CADNodeColor& color) {
        for (const std::shared_ptr<CadNode>& child : root->children) {
            if (child && child->name == name && child->asMeshGeometry()) return child.get();
        }
        auto generated = std::make_shared<CadNode>();
        generated->name = name;
        generated->type = CadNodeType::MeshGeometry;
        generated->color = color;
        generated->data = std::make_shared<MeshGeometryData>();
        generated->parent = root;
        root->children.push_back(generated);
        return generated.get();
    };
    CadNode* roleEnclosureNode = ensureGeneratedRoleMesh(
        "Conveyor role enclosure", CADNodeColor::fromSRGB(48, 56, 64));
    CadNode* spawnerMarkingNode = ensureGeneratedRoleMesh(
        "Spawner product and infinity markings", CADNodeColor::fromSRGB(82, 205, 255));
    CadNode* deleterMarkingNode = ensureGeneratedRoleMesh(
        "Deleter X markings", CADNodeColor::fromSRGB(235, 52, 62));
    MeshGeometryData* roleEnclosure = roleEnclosureNode->asMeshGeometry();
    MeshGeometryData* spawnerMarkings = spawnerMarkingNode->asMeshGeometry();
    MeshGeometryData* deleterMarkings = deleterMarkingNode->asMeshGeometry();
    if (!roleEnclosure || !spawnerMarkings || !deleterMarkings) return false;

    clampRollerConveyorParameters(*parameters);

    const float length = static_cast<float>(parameters->accessoryLengthMm);
    const float width = static_cast<float>(parameters->accessoryWidthMm);
    const float startHeight = static_cast<float>(parameters->accessoryStartHeightMm);
    const float endHeight = static_cast<float>(parameters->accessoryEndHeightMm);
    const float rollerPitch = static_cast<float>(parameters->accessoryRollerPitchMm);
    const float turnRadians = static_cast<float>(parameters->accessoryTurnAngleDeg *
                                                  3.14159265358979323846 / 180.0);
    const bool curved = std::abs(turnRadians) > 0.001f;
    constexpr float kTwoPi = 6.28318530717958647692f;
    const bool multiTurn = std::abs(turnRadians) >= kTwoPi - 0.001f;
    const float turnSign = turnRadians < 0.0f ? -1.0f : 1.0f;
    const float radius = std::max(static_cast<float>(parameters->accessoryCurveRadiusMm),
                                  width * 0.5f + 150.0f);
    parameters->accessoryCurveRadiusMm = radius;
    const float pathLength = curved ? std::abs(turnRadians) * radius : length;
    const float rollerRadius = static_cast<float>(conveyorRollerRadiusMm(*parameters));
    const float sideOffset = width * 0.5f - 20.0f;
    const float rollerHalfWidth = width * 0.5f - 45.0f;
    const float legSideOffset = width * 0.5f - 70.0f;
    const bool banked =
        std::abs(parameters->accessoryStartLeftHeightMm -
                 parameters->accessoryStartRightHeightMm) > 0.01 ||
        std::abs(parameters->accessoryEndLeftHeightMm -
                 parameters->accessoryEndRightHeightMm) > 0.01;
    const bool pickFeeder = parameters->accessoryConveyorRole == "pick_feeder";
    const auto feederSurfaceHalfWidthAt = [&](float t) {
        return pickFeeder
            ? static_cast<float>(robotPickFeederSurfaceHalfWidthAt(*parameters, t))
            : rollerHalfWidth + 8.0f;
    };

    const auto pathPose = [&](float t) {
        const ConveyorPathPose shared = conveyorPathPoseAt(*parameters, t);
        return ConveyorPathPose{shared.x, shared.y, shared.z, shared.heading};
    };

    frame->vertices.clear(); frame->indices.clear();
    feet->vertices.clear(); feet->indices.clear();
    rollers->vertices.clear(); rollers->indices.clear();
    cover->vertices.clear(); cover->indices.clear();
    roleEnclosure->vertices.clear(); roleEnclosure->indices.clear();
    spawnerMarkings->vertices.clear(); spawnerMarkings->indices.clear();
    deleterMarkings->vertices.clear(); deleterMarkings->indices.clear();
    // Turn angle is intentionally unbounded: multi-revolution helical conveyors are valid. Cap
    // only render tessellation, not the requested geometry, so an accidental enormous value
    // cannot allocate millions of boxes in one UI frame.
    const float frameSegmentLength = pickFeeder ? 30.0f : 120.0f;
    const int pathSegments = std::max(
        1, static_cast<int>(std::min(
            4096.0f, std::ceil(pathLength / frameSegmentLength))));
    for (int segment = 0; segment < pathSegments; ++segment) {
        const float ta = static_cast<float>(segment) / pathSegments;
        const float tb = static_cast<float>(segment + 1) / pathSegments;
        const ConveyorPathPose a = pathPose(ta);
        const ConveyorPathPose b = pathPose(tb);
        // A curved belt needs a guard above the deck to turn a freely simulated payload, but its
        // interfaces still have to be the same 140 mm rail section as a straight conveyor. Keep
        // the lower edge fixed at -140 mm and smoothly grow only the top over the first/last 8% of
        // the path. This makes both generated mounting seams geometrically continuous.
        const float midpointT = (static_cast<float>(segment) + 0.5f) / pathSegments;
        const float guardBlend = curved
            ? std::max(0.0f, std::min(1.0f, std::min(midpointT, 1.0f - midpointT) / 0.08f))
            : 0.0f;
        const float guardExtension = std::max(100.0f * guardBlend, banked ? 120.0f : 0.0f);
        const float sideRailHeight = 140.0f + guardExtension;
        const float sideRailCenterOffset = -70.0f + guardExtension * 0.5f;
        for (float sideSign : {-1.0f, 1.0f}) {
            const float railSideOffsetA = pickFeeder
                ? static_cast<float>(robotPickFeederWallCenterOffsetAt(*parameters, ta))
                : sideOffset;
            const float railSideOffsetB = pickFeeder
                ? static_cast<float>(robotPickFeederWallCenterOffsetAt(*parameters, tb))
                : sideOffset;
            const float asx = -std::sin(a.heading) * railSideOffsetA * sideSign;
            const float asz = std::cos(a.heading) * railSideOffsetA * sideSign;
            const float bsx = -std::sin(b.heading) * railSideOffsetB * sideSign;
            const float bsz = std::cos(b.heading) * railSideOffsetB * sideSign;
            const float aDeckY = a.y + static_cast<float>(conveyorSideHeightOffsetAt(
                *parameters, ta, sideSign, railSideOffsetA));
            const float bDeckY = b.y + static_cast<float>(conveyorSideHeightOffsetAt(
                *parameters, tb, sideSign, railSideOffsetB));
            const float actualRailHeight = pickFeeder
                ? static_cast<float>(robotPickFeederWallHeightAt(midpointT))
                : sideRailHeight;
            const float actualRailCenterOffset = pickFeeder
                ? actualRailHeight * 0.5f : sideRailCenterOffset;
            if (!pickFeeder && actualRailHeight > 0.5f) {
                accessoryAppendSegmentBox(*frame,
                    a.x + asx, aDeckY + actualRailCenterOffset, a.z + asz,
                    b.x + bsx, bDeckY + actualRailCenterOffset, b.z + bsz,
                    actualRailHeight, pickFeeder ? 8.0f : 40.0f);
            }
            if (parameters->accessorySupportBracesEnabled) {
                // Stay a fixed distance under the deck. Multiplying absolute height by 0.40
                // made an incline brace climb at only 40% of the conveyor grade, so the two
                // visibly diverged and a spiral grew a second, unrelated helix.
                constexpr float kBraceDropMm = 530.0f;
                accessoryAppendSegmentBox(*frame,
                    a.x + asx, std::max(100.0f, aDeckY - kBraceDropMm), a.z + asz,
                    b.x + bsx, std::max(100.0f, bDeckY - kBraceDropMm), b.z + bsz,
                    45.0f, 45.0f);
            }
        }
    }

    if (pickFeeder) {
        constexpr double kWallThicknessMm = 8.0;
        for (double sideSign : {-1.0, 1.0}) {
            std::vector<std::array<CadVec3, 4>> sections;
            sections.reserve(static_cast<size_t>(pathSegments) + 1);
            for (int section = 0; section <= pathSegments; ++section) {
                const double t = static_cast<double>(section) / pathSegments;
                const ConveyorPathPose pose = pathPose(static_cast<float>(t));
                const double outerOffset = robotPickFeederDeckHalfWidthAt(*parameters, t);
                const double innerOffset = outerOffset - kWallThicknessMm;
                const double sideX = -std::sin(pose.heading);
                const double sideZ = std::cos(pose.heading);
                const double outerY = pose.y + conveyorSideHeightOffsetAt(
                    *parameters, t, sideSign, outerOffset);
                const double innerY = pose.y + conveyorSideHeightOffsetAt(
                    *parameters, t, sideSign, innerOffset);
                const double height = robotPickFeederWallHeightAt(t);
                const CadVec3 outerBottom(
                    pose.x + sideX * outerOffset * sideSign,
                    outerY,
                    pose.z + sideZ * outerOffset * sideSign);
                const CadVec3 innerBottom(
                    pose.x + sideX * innerOffset * sideSign,
                    innerY,
                    pose.z + sideZ * innerOffset * sideSign);
                sections.push_back({{
                    outerBottom,
                    innerBottom,
                    CadVec3(outerBottom.x, outerBottom.y + height, outerBottom.z),
                    CadVec3(innerBottom.x, innerBottom.y + height, innerBottom.z)}});
            }
            accessoryAppendProfiledWall(*frame, sections);
        }
    }

    const auto appendEndTie = [&](float t) {
        const ConveyorPathPose pose = pathPose(t);
        const float sideX = -std::sin(pose.heading);
        const float sideZ = std::cos(pose.heading);
        const float tieOffset = width * 0.5f - 40.0f;
        const float leftY = pose.y + static_cast<float>(conveyorSideHeightOffsetAt(
            *parameters, t, -1.0, tieOffset));
        const float rightY = pose.y + static_cast<float>(conveyorSideHeightOffsetAt(
            *parameters, t, 1.0, tieOffset));
        accessoryAppendSegmentBox(*frame,
            pose.x - sideX * tieOffset, leftY - 70.0f,
            pose.z - sideZ * tieOffset,
            pose.x + sideX * tieOffset, rightY - 70.0f,
            pose.z + sideZ * tieOffset, 140.0f, 40.0f);
    };
    appendEndTie(0.0f);
    if (!pickFeeder) appendEndTie(1.0f);
    if (parameters->accessoryEndStopEnabled) {
        const ConveyorPathPose pose = pathPose(1.0f);
        const float sideX = -std::sin(pose.heading);
        const float sideZ = std::cos(pose.heading);
        const float stopOffset = pickFeeder
            ? static_cast<float>(robotPickFeederWallCenterOffsetAt(*parameters, 1.0))
            : width * 0.5f - 30.0f;
        const float leftY = pose.y + static_cast<float>(conveyorSideHeightOffsetAt(
            *parameters, 1.0, -1.0, stopOffset));
        const float rightY = pose.y + static_cast<float>(conveyorSideHeightOffsetAt(
            *parameters, 1.0, 1.0, stopOffset));
        if (pickFeeder) {
    // Close the narrowed chute with a wall matching the side-wall height and deck profile.
            constexpr float kStopHeightMm =
                static_cast<float>(kRobotPickFeederFullWallHeightMm);
            constexpr float kStopThicknessMm = 30.0f;
            accessoryAppendSegmentBox(*frame,
                pose.x - sideX * stopOffset, leftY + kStopHeightMm * 0.5f,
                pose.z - sideZ * stopOffset,
                pose.x + sideX * stopOffset, rightY + kStopHeightMm * 0.5f,
                pose.z + sideZ * stopOffset, kStopHeightMm, kStopThicknessMm);
        } else {
            constexpr float kStopHeightMm = 220.0f;
            accessoryAppendSegmentBox(*frame,
                pose.x - sideX * stopOffset, leftY + kStopHeightMm * 0.5f,
                pose.z - sideZ * stopOffset,
                pose.x + sideX * stopOffset, rightY + kStopHeightMm * 0.5f,
                pose.z + sideZ * stopOffset, kStopHeightMm, 50.0f);
        }
    }

    std::array<CadVec3, 4> footPoints;
    int footIndex = 0;
    const auto appendSupport = [&](float t, float sideSign, float supportSideOffset) {
        const ConveyorPathPose pose = pathPose(t);
        const float sideX = -std::sin(pose.heading);
        const float sideZ = std::cos(pose.heading);
        const float deckY = pose.y + static_cast<float>(conveyorSideHeightOffsetAt(
            *parameters, t, sideSign, supportSideOffset));
        const float legHeight = std::max(120.0f, deckY - 130.0f);
        const float x = pose.x + sideX * supportSideOffset * sideSign;
        const float z = pose.z + sideZ * supportSideOffset * sideSign;
        accessoryAppendBox(*frame, x, 10.0f + legHeight * 0.5f, z,
                           60.0f, legHeight, 60.0f);
        accessoryAppendBox(*feet, x, 5.0f, z, 130.0f, 10.0f, 130.0f);
        if (footIndex < static_cast<int>(footPoints.size())) {
            footPoints[static_cast<size_t>(footIndex++)] = CadVec3(x, 0.0, z);
        }
    };
    if (multiTurn) {
        // Four perimeter towers at the highest occurrence of each quadrant. A multi-turn path
        // revisits the same plan positions at multiple elevations; continuous columns there can
        // carry every pass without the arbitrary pair of end-derived skyscraper legs.
        const float totalAngle = std::abs(turnRadians);
        const float outerSide = -turnSign;
        // Towers must sit outside the swept belt envelope. The previous legSideOffset was 70 mm
        // inside the deck edge, so a tower at a repeated quadrant pierced every revolution.
        const float towerSideOffset = width * 0.5f + 100.0f;
        for (int quadrant = 0; quadrant < 4; ++quadrant) {
            const float phase = static_cast<float>(quadrant) * 0.25f * kTwoPi;
            const float revolutions = std::floor((totalAngle - phase) / kTwoPi);
            const float highestAngle = phase + std::max(0.0f, revolutions) * kTwoPi;
            appendSupport(std::min(1.0f, highestAngle / totalAngle), outerSide,
                          towerSideOffset);
            // These radial arms are support braces too. Respect the same property used by the
            // continuous under-deck rails; otherwise a spiral configured with braces disabled
            // still receives short bars that appear to cut through the carrying path from above.
            if (parameters->accessorySupportBracesEnabled) {
                for (float angle = phase; angle <= totalAngle + 0.001f; angle += kTwoPi) {
                    const float t = std::min(1.0f, angle / totalAngle);
                    const ConveyorPathPose pose = pathPose(t);
                    const float sideX = -std::sin(pose.heading) * outerSide;
                    const float sideZ = std::cos(pose.heading) * outerSide;
                    const float railOffset = width * 0.5f - 20.0f;
                    accessoryAppendSegmentBox(*frame,
                        pose.x + sideX * railOffset, pose.y - 100.0f,
                        pose.z + sideZ * railOffset,
                        pose.x + sideX * towerSideOffset, pose.y - 100.0f,
                        pose.z + sideZ * towerSideOffset,
                        45.0f, 45.0f);
                }
            }
        }
    } else {
        // Role enclosures already designate which half is machinery and which half is the open
        // transfer interface. Keep both support bents under the machinery half so a high source or
        // sink connected to a spiral cannot send a floor-to-deck leg through the curved path.
        float supportT0 = 0.12f;
        float supportT1 = 0.88f;
        if (parameters->accessoryConveyorRole == "spawner") {
            supportT1 = 0.38f;
        } else if (parameters->accessoryConveyorRole == "deleter") {
            supportT0 = 0.62f;
        } else if (pickFeeder) {
            supportT0 = 0.12f;
            supportT1 = 0.56f;
        }
        for (float t : {supportT0, supportT1}) {
            for (float sideSign : {-1.0f, 1.0f}) {
                appendSupport(t, sideSign, legSideOffset);
            }
        }
    }

    if (!parameters->accessoryRollerCoverEnabled) {
        const float rollerInset = static_cast<float>(conveyorRollerInsetMm(*parameters));
        const float rollerRun = std::max(1.0f, pathLength - rollerInset * 2.0f);
        const int rollerCount = std::max(
            2, static_cast<int>(std::min(
                   4096.0f, std::floor(rollerRun / rollerPitch) + 1.0f)));
        for (int index = 0; index < rollerCount; ++index) {
            const float distance = rollerInset +
                rollerRun * static_cast<float>(index) / static_cast<float>(rollerCount - 1);
            const ConveyorPathPose pose = pathPose(distance / pathLength);
            accessoryAppendHorizontalCylinder(*rollers, pose.x, pose.y - rollerRadius,
                                              pose.z, pose.heading, rollerHalfWidth,
                                              rollerRadius,
                                              static_cast<float>(conveyorRightDropAt(
                                                  *parameters, distance / pathLength)));
        }
    }

    if (parameters->accessoryRollerCoverEnabled) {
        // An independent quad per path interval keeps collision extraction simple: each quad is a
        // powered static belt patch whose target velocity follows the local conveyor tangent.
        const int coverSegments = pickFeeder
            ? pathSegments
            : std::max(1, static_cast<int>(std::min(
                  4096.0f, std::ceil(pathLength / 60.0f))));
        const float surfaceOffset = static_cast<float>(kConveyorRollerCoverSurfaceOffsetMm);
        for (int segment = 0; segment < coverSegments; ++segment) {
            const ConveyorPathPose a = pathPose(static_cast<float>(segment) / coverSegments);
            const ConveyorPathPose b = pathPose(static_cast<float>(segment + 1) / coverSegments);
            const float ta = static_cast<float>(segment) / coverSegments;
            const float tb = static_cast<float>(segment + 1) / coverSegments;
            const float coverHalfWidthA = pickFeeder
                ? static_cast<float>(robotPickFeederDeckHalfWidthAt(*parameters, ta))
                : feederSurfaceHalfWidthAt(ta);
            const float coverHalfWidthB = pickFeeder
                ? static_cast<float>(robotPickFeederDeckHalfWidthAt(*parameters, tb))
                : feederSurfaceHalfWidthAt(tb);
            const float asx = -std::sin(a.heading) * coverHalfWidthA;
            const float asz = std::cos(a.heading) * coverHalfWidthA;
            const float bsx = -std::sin(b.heading) * coverHalfWidthB;
            const float bsz = std::cos(b.heading) * coverHalfWidthB;
            const float aLeftY = a.y + surfaceOffset + static_cast<float>(
                conveyorSideHeightOffsetAt(*parameters, ta, -1.0, coverHalfWidthA));
            const float aRightY = a.y + surfaceOffset + static_cast<float>(
                conveyorSideHeightOffsetAt(*parameters, ta, 1.0, coverHalfWidthA));
            const float bLeftY = b.y + surfaceOffset + static_cast<float>(
                conveyorSideHeightOffsetAt(*parameters, tb, -1.0, coverHalfWidthB));
            const float bRightY = b.y + surfaceOffset + static_cast<float>(
                conveyorSideHeightOffsetAt(*parameters, tb, 1.0, coverHalfWidthB));
            const uint32_t base = static_cast<uint32_t>(cover->vertices.size() / 3);
            cover->vertices.insert(cover->vertices.end(), {
                a.x - asx, aLeftY, a.z - asz,
                a.x + asx, aRightY, a.z + asz,
                b.x - bsx, bLeftY, b.z - bsz,
                b.x + bsx, bRightY, b.z + bsz});
            const uint32_t triangles[] = {base, base + 3, base + 2,
                                          base, base + 1, base + 3};
            cover->indices.insert(cover->indices.end(),
                                  std::begin(triangles), std::end(triangles));
        }
    }

    if (parameters->accessoryConveyorRole == "spawner" ||
        parameters->accessoryConveyorRole == "deleter") {
        const bool spawner = parameters->accessoryConveyorRole == "spawner";
        std::vector<AccessoryIconPoint> productOutline;
    // The caller resolves the optional workpiece used for the marking silhouette.
        if (spawner && spawnPrototype) {
            productOutline = accessorySpawnerObjectOutline(spawnPrototype);
        }
        const float t0 = spawner ? 0.0f : 0.5f;
        const float t1 = spawner ? 0.5f : 1.0f;
        ConveyorPathPose a = pathPose(t0);
        ConveyorPathPose b = pathPose(t1);
        CadVec3 forward(b.x - a.x, b.y - a.y, b.z - a.z);
        const double forwardLength = std::sqrt(forward.x * forward.x +
                                               forward.y * forward.y +
                                               forward.z * forward.z);
        if (forwardLength > 1.0e-3) {
            forward = CadVec3(forward.x / forwardLength,
                              forward.y / forwardLength,
                              forward.z / forwardLength);
            const double horizontal = std::max(
                1.0e-5, std::sqrt(forward.x * forward.x + forward.z * forward.z));
            const CadVec3 side(-forward.z / horizontal, 0.0, forward.x / horizontal);
            const CadVec3 up(side.y * forward.z - side.z * forward.y,
                             side.z * forward.x - side.x * forward.z,
                             side.x * forward.y - side.y * forward.x);
            constexpr float enclosureHeight = 360.0f;
            const float enclosureWidth = std::max(180.0f, width - 80.0f);
            const float surfaceOffset = static_cast<float>(conveyorSurfaceOffsetMm(*parameters));
            const CadVec3 center((a.x + b.x) * 0.5 + up.x *
                                     (surfaceOffset + enclosureHeight * 0.5f),
                                 (a.y + b.y) * 0.5 + up.y *
                                     (surfaceOffset + enclosureHeight * 0.5f),
                                 (a.z + b.z) * 0.5 + up.z *
                                     (surfaceOffset + enclosureHeight * 0.5f));
            accessoryAppendOrientedBox(*roleEnclosure, center, forward, up, side,
                                       static_cast<float>(forwardLength), enclosureHeight,
                                       enclosureWidth);

            MeshGeometryData& markingMesh = spawner ? *spawnerMarkings : *deleterMarkings;
            const float halfForward = static_cast<float>(forwardLength) * 0.5f;
            const float halfUp = enclosureHeight * 0.5f;
            const float halfSide = enclosureWidth * 0.5f;
            // Top, both long sides and both end faces: all five exposed faces carry the same role
            // marking. The bottom is deliberately blank against the conveyor surface.
            accessoryAppendRoleFaceIcon(markingMesh,
                CadVec3(center.x + up.x * halfUp, center.y + up.y * halfUp,
                        center.z + up.z * halfUp),
                forward, side, up, static_cast<float>(forwardLength), enclosureWidth, spawner,
                productOutline, true);
            for (double sign : {-1.0, 1.0}) {
                const CadVec3 sideNormal(side.x * sign, side.y * sign, side.z * sign);
                accessoryAppendRoleFaceIcon(markingMesh,
                    CadVec3(center.x + sideNormal.x * halfSide,
                            center.y + sideNormal.y * halfSide,
                            center.z + sideNormal.z * halfSide),
                    forward, up, sideNormal, static_cast<float>(forwardLength),
                    enclosureHeight, spawner, productOutline);
                const CadVec3 endNormal(forward.x * sign, forward.y * sign, forward.z * sign);
                const CadVec3 endFaceX(side.x * sign, side.y * sign, side.z * sign);
                accessoryAppendRoleFaceIcon(markingMesh,
                    CadVec3(center.x + endNormal.x * halfForward,
                            center.y + endNormal.y * halfForward,
                            center.z + endNormal.z * halfForward),
                    endFaceX, up, endNormal, enclosureWidth, enclosureHeight, spawner,
                    productOutline);
            }
        }
    }

    accessoryFinalizeMesh(*frame);
    accessoryFinalizeMesh(*feet);
    accessoryFinalizeMesh(*rollers);
    accessoryFinalizeMesh(*cover);
    accessoryFinalizeMesh(*roleEnclosure);
    accessoryFinalizeMesh(*spawnerMarkings);
    accessoryFinalizeMesh(*deleterMarkings);

    root->mountingHoles.pointsMm.clear();
    root->mountingHoles.grids.clear();
    root->mountingHoles.pointsMm.assign(footPoints.begin(), footPoints.end());

    frameNode->mountingHoles.pointsMm.clear();
    frameNode->mountingHoles.grids.clear();
    const auto updateConnection = [&](CadNode* connection, bool rightEnd) {
        const ConveyorPathPose endpoint = pathPose(rightEnd ? 1.0f : 0.0f);
        const float outwardHeading = endpoint.heading + (rightEnd ? 0.0f : 3.14159265358979323846f);
        const float outwardX = std::cos(outwardHeading);
        const float outwardZ = std::sin(outwardHeading);
        connection->loc = CadTransform();
        connection->loc.values = {{0.0, outwardX, outwardZ, endpoint.x,
                                   1.0, 0.0, 0.0, endpoint.y - 140.0,
                                   0.0, outwardZ, -outwardX, endpoint.z}};
        connection->needsGlobalLocUpdate = true;
        connection->mountingHoles.pointsMm.clear();
        connection->mountingHoles.grids.clear();
        if (rightEnd && parameters->accessoryEndStopEnabled) {
            connection->mountingHoles.placementSource = false;
            connection->mountingHoles.mateOpposite = false;
            connection->mountingHoles.interfaceId.clear();
            connection->mountingHoles.parameterBindings.clear();
            return;
        }
        connection->mountingHoles.placementSource = true;
        connection->mountingHoles.mateOpposite = true;
        connection->mountingHoles.interfaceId = rightEnd ? "end" : "start";
        connection->mountingHoles.parameterBindings.clear();
        connection->mountingHoles.parameterBindings["deckHeight"] =
            rightEnd ? "endHeightMm" : "startHeightMm";
        const float endpointT = rightEnd ? 1.0f : 0.0f;
        const double leftHeightOffset = conveyorSideHeightOffsetAt(
            *parameters, endpointT, -1.0, sideOffset);
        const double rightHeightOffset = conveyorSideHeightOffsetAt(
            *parameters, endpointT, 1.0, sideOffset);
        MountingHoleGridData endGrid;
        // Local X is world up and local Z spans the conveyor.  Encoding the bank in the
        // mounting grid keeps all four interface points on the generated deck plane.
        endGrid.originMm = CadVec3(leftHeightOffset, 0.0, -sideOffset);
        endGrid.uStepMm = CadVec3(140.0, 0.0, 0.0);
        endGrid.vStepMm = CadVec3(rightHeightOffset - leftHeightOffset,
                                  0.0, sideOffset * 2.0);
        endGrid.uCount = 2;
        endGrid.vCount = 2;
        connection->mountingHoles.grids.push_back(endGrid);
    };
    updateConnection(leftConnection, false);
    updateConnection(rightConnection, true);
    const int startRounded = static_cast<int>(std::lround(startHeight));
    const int endRounded = static_cast<int>(std::lround(endHeight));
    const auto metresText = [](double millimetres, bool showSign = false) {
        const double metres = millimetres / 1000.0;
        const std::string number = strutil::format("%1").arg(metres, 0, 'f', 1).str();
        return showSign && metres > 0.0 ? "+" + number : number;
    };
    const int riseMm = endRounded - startRounded;
    if (curved) {
        const double absoluteAngle = std::abs(parameters->accessoryTurnAngleDeg);
        if (absoluteAngle >= 360.0) {
            const double turns = absoluteAngle / 360.0;
            const double roundedTurns = std::round(turns);
            const std::string turnsText = std::abs(turns - roundedTurns) < 0.001
                ? std::to_string(static_cast<int>(roundedTurns))
                : strutil::format("%1").arg(turns, 0, 'f', 1).str();
            root->name = turnsText + "-Turn Spiral Conveyor";
            if (riseMm != 0) root->name += " — " + metresText(riseMm, true) + " m";
        } else {
            const int angleRounded = static_cast<int>(std::lround(absoluteAngle));
            root->name = std::string(parameters->accessoryTurnAngleDeg > 0.0
                                         ? "Left " : "Right ") +
                         std::to_string(angleRounded) + " deg Conveyor";
            if (riseMm == 0) {
                root->name += " — R" + metresText(radius) + " m";
            } else {
                root->name += " — " + metresText(riseMm, true) + " m";
            }
        }
    } else if (riseMm > 0) {
        root->name = "Incline Conveyor — " + metresText(riseMm, true) + " m";
    } else if (riseMm < 0) {
        root->name = "Decline Conveyor — " + metresText(riseMm, true) + " m";
    } else {
        root->name = (length <= 1300.0f ? "Short Conveyor — " : "Straight Conveyor — ") +
                     metresText(length) + " m";
    }
    if (pickFeeder) {
        root->name = "Robot Pick Feeder";
    } else if (parameters->accessoryEndStopEnabled && banked) {
        const bool rightLower = parameters->accessoryEndRightHeightMm <
                                parameters->accessoryEndLeftHeightMm;
        root->name = std::string(rightLower ? "Right" : "Left") +
                     "-Bank Pick Accumulator";
    }
    if (parameters->accessoryConveyorRole == "spawner") {
        root->name = "Object Spawner - " + root->name;
    } else if (parameters->accessoryConveyorRole == "deleter") {
        root->name = "Deleter - " + root->name;
    }
    return true;
}
