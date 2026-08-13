#pragma once

#include <cstdint>
#include <vector>

#include "CadNode.h"

void accessoryAppendBox(MeshGeometryData& mesh, float cx, float cy, float cz,
                        float sx, float sy, float sz);
void accessoryAppendOrientedBox(MeshGeometryData& mesh,
                                const CadVec3& center,
                                const CadVec3& forward,
                                const CadVec3& up,
                                const CadVec3& side,
                                float forwardSize, float upSize, float sideSize);
void accessoryAppendFaceStroke(MeshGeometryData& mesh,
                               const CadVec3& faceCenter,
                               const CadVec3& faceX,
                               const CadVec3& faceY,
                               const CadVec3& normal,
                               float x0, float y0, float x1, float y1,
                               float thickness, float raisedDepth);
struct AccessoryIconPoint {
    float x = 0.0f;
    float y = 0.0f;
};

void accessoryCollectIconPoints(const CadNode* node, const CadTransform& parent,
                                bool applyNodeTransform,
                                std::vector<AccessoryIconPoint>& points);
std::vector<AccessoryIconPoint> accessorySpawnerObjectOutline(const CadNode* prototype);
void accessoryAppendRoleFaceIcon(MeshGeometryData& mesh,
                                 const CadVec3& faceCenter,
                                 const CadVec3& faceX,
                                 const CadVec3& faceY,
                                 const CadVec3& normal,
                                 float faceWidth, float faceHeight,
                                 bool spawner,
                                 const std::vector<AccessoryIconPoint>& productOutline = {},
                                 bool rotateIconQuarterTurn = false);
void accessoryAppendSegmentBox(MeshGeometryData& mesh,
                               float ax, float ay, float az,
                               float bx, float by, float bz,
                               float height, float width);
void accessoryAppendProfiledWall(
    MeshGeometryData& mesh,
    const std::vector<std::array<CadVec3, 4>>& sections);
void accessoryAppendHorizontalCylinder(MeshGeometryData& mesh, float centerX, float centerY,
                                       float centerZ, float heading, float halfWidth,
                                       float radius, float rightDropMm = 0.0f,
                                       int segments = 20);
void accessoryAppendPerforatedCell(MeshGeometryData& mesh, float centerX, float centerZ,
                                   float yBottom, float yTop, float cellSize,
                                   float holeRadius);
void accessoryFinalizeMesh(MeshGeometryData& mesh);
int accessoryHoleCount(double dimensionMm, double pitchMm);
struct ConveyorPathPose {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float heading = 0.0f;
};

constexpr double kConveyorMaximumRollerRadiusMm = 32.0;

constexpr double kConveyorRollerCoverSurfaceOffsetMm = 8.0;

double conveyorRollerRadiusMm(const TransformNodeData& parameters);
double conveyorSurfaceOffsetMm(const TransformNodeData& parameters);
double conveyorPathLengthMm(const TransformNodeData& parameters);
double conveyorRollerInsetMm(const TransformNodeData& parameters);
double conveyorCornerHeightAt(const TransformNodeData& parameters, double t, bool right);
double conveyorSideHeightOffsetAt(const TransformNodeData& parameters, double t,
                                  double sideSign, double horizontalSideOffsetMm);
double conveyorRightDropAt(const TransformNodeData& parameters, double t);
ConveyorPathPose conveyorPathPoseAt(const TransformNodeData& parameters, double t);

CadVec3 conveyorTransformPoint(const CadTransform& world, const ConveyorPathPose& point);

// Inverse of conveyorPathPoseAt for paths that may turn and climb.
double closestConveyorProgress(const TransformNodeData& parameters, const CadTransform& world,
                               const CadVec3& worldPoint);

// The unit direction of travel at `progress`, differenced along the path so a curve is honoured.
CadVec3 conveyorTangentAt(const TransformNodeData& parameters, const CadTransform& world,
                          double progress, bool forward);
double robotPickFeederSurfaceHalfWidthAt(const TransformNodeData& parameters, double t);
double robotPickFeederDeckHalfWidthAt(const TransformNodeData& parameters, double t);
constexpr double kRobotPickFeederFullWallHeightMm = 100.0;

double robotPickFeederWallCenterOffsetAt(const TransformNodeData& parameters, double t);
double robotPickFeederWallHeightAt(double t);
