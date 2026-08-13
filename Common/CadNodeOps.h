#pragma once

#include "CadNode.h"

#include <QVector3D>
#include <QString>
#include <TopLoc_Location.hxx>

#include <functional>
#include <memory>
#include <vector>

class CadOpenGLWidget;

struct FaceWithTransform {
    CadNode* node = nullptr;
    TopLoc_Location accumulatedLoc;
};

void setParentPointersRecursive(CadNode* node, CadNode* parent = nullptr);
std::shared_ptr<CadNode> deepCopyNodeNonExcluded(const CadNode* src);
void adjustSubtreeTransforms(
    const CadNode* src,
    CadNode* copy,
    const TopLoc_Location& baseLoc,
    std::function<const CadNode*(const CadNode*)> getParent);
double computeBoundingBoxLength(const CadNode* node, const QVector3D& axis);
void addRailToPhysicsPreview(CadNode* railNode, std::shared_ptr<CadNode> physicsPreviewRoot);
void collectFaceNodesWithTransform(CadNode* node,
                                   const TopLoc_Location& parentLoc,
                                   std::vector<FaceWithTransform>& out);
void expandRailInPhysicsPreview(CadNode* railNode,
                                std::shared_ptr<CadNode> physicsPreviewRoot,
                                CadOpenGLWidget* oglWidget);
bool bakeRobotPackageHullsFile(const QString& inputPackageFile,
                               const QString& outputPackageFile,
                               QString* errorMessage = nullptr);
