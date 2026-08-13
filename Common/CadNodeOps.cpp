// Operations over an already-built CadNode tree: copying, re-parenting, measuring,
// and tiling a rail. No OpenCascade document is read or written here.
#include "CadNodeOps.h"

#include "CadNodeQtAdapter.h"
#include "CadOpenGLWidget.h"
#include "CadNodePackage.h"
#include "RobotRuntime.h"

#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <QMessageBox>
#include <TopExp_Explorer.hxx>

#include <algorithm>
#include <functional>

void setParentPointersRecursive(CadNode* node, CadNode* parent) {
    if (!node) return;
    node->parent = parent;
    for (auto& child : node->children) {
        setParentPointersRecursive(child.get(), node);
    }
}

// Deep copy of a CadNode, skipping excluded children.
std::shared_ptr<CadNode> deepCopyNodeNonExcluded(const CadNode* src) {
    if (!src || src->excludedFromDecomposition) return nullptr;
    auto copy = std::make_shared<CadNode>(*src);
    // Copy data by value for all known types
    if (src->data) {
        if (src->type == CadNodeType::XCAF) {
            copy->data = std::make_shared<XCAFNodeData>(*static_cast<const XCAFNodeData*>(src->data.get()));
        } else if (src->type == CadNodeType::Rail) {
            copy->data = std::make_shared<RailNodeData>(*static_cast<const RailNodeData*>(src->data.get()));
        } else if (src->type == CadNodeType::GantryMechanism) {
            copy->data = std::make_shared<GantryMechanismData>(*static_cast<const GantryMechanismData*>(src->data.get()));
        } else if (src->type == CadNodeType::DragChainMechanism) {
            copy->data = std::make_shared<DragChainMechanismData>(*static_cast<const DragChainMechanismData*>(src->data.get()));
        } else if (src->type == CadNodeType::Physics) {
            copy->data = std::make_shared<PhysicsNodeData>(*static_cast<const PhysicsNodeData*>(src->data.get()));
        } else if (src->type == CadNodeType::Custom) {
            copy->data = std::make_shared<CustomNodeData>(*static_cast<const CustomNodeData*>(src->data.get()));
        } else if (src->type == CadNodeType::ConnectionPoint) {
            copy->data = std::make_shared<ConnectionPointData>(*static_cast<const ConnectionPointData*>(src->data.get()));
        } else if (src->type == CadNodeType::Transform) {
            copy->data = std::make_shared<TransformNodeData>(*static_cast<const TransformNodeData*>(src->data.get()));
        } else if (src->type == CadNodeType::MutexRoot) {
            const auto* srcData = static_cast<const MutexRootNodeData*>(src->data.get());
            auto copiedData = std::make_shared<MutexRootNodeData>();
            copiedData->groundPlaneVisible = srcData->groundPlaneVisible;
            copiedData->groundPlaneY = srcData->groundPlaneY;
            copiedData->groundPlaneSize = srcData->groundPlaneSize;
            copiedData->groundPlaneThickness = srcData->groundPlaneThickness;
            copiedData->groundPlaneColor = srcData->groundPlaneColor;
            copy->data = copiedData;
        } else if (src->type == CadNodeType::OPW6Robot) {
            copy->data = std::make_shared<OPW6RobotData>(*static_cast<const OPW6RobotData*>(src->data.get()));
        } else if (src->type == CadNodeType::RobotLink) {
            copy->data = std::make_shared<RobotLinkData>(*static_cast<const RobotLinkData*>(src->data.get()));
        } else if (src->type == CadNodeType::RobotTool) {
            copy->data = std::make_shared<RobotToolData>(*static_cast<const RobotToolData*>(src->data.get()));
        } else if (src->type == CadNodeType::MeshGeometry) {
            copy->data = std::make_shared<MeshGeometryData>(*static_cast<const MeshGeometryData*>(src->data.get()));
        } else {
            copy->data = nullptr;
        }
    }
    copy->children.clear();
    for (const auto& child : src->children) {
        auto childCopy = deepCopyNodeNonExcluded(child.get());
        if (childCopy) copy->children.push_back(childCopy);
    }
    return copy;
}

// Adjusts all descendants' transforms so their global transform matches the original.
void adjustSubtreeTransforms(const CadNode* src, CadNode* copy, const TopLoc_Location& baseLoc, std::function<const CadNode*(const CadNode*)> getParent) {
    TopLoc_Location acc;
    std::vector<const CadNode*> ancestry;
    const CadNode* cur = src;
    while (cur) {
        ancestry.push_back(cur);
        cur = getParent(cur);
    }
    for (auto it = ancestry.rbegin(); it != ancestry.rend(); ++it) {
        acc = acc * (*it)->loc;
    }
    copy->loc = baseLoc.Inverted() * acc;
    for (size_t i = 0; i < src->children.size(); ++i) {
        if (copy->children.size() > i) {
            adjustSubtreeTransforms(src->children[i].get(), copy->children[i].get(), baseLoc, getParent);
        }
    }
}

double computeBoundingBoxLength(const CadNode* node, const QVector3D& axis) {
    if (!node) return 0.0;
    Bnd_Box bbox;
    std::function<void(const CadNode*, const TopLoc_Location&)> accumulate;
    accumulate = [&](const CadNode* n, const TopLoc_Location& accLoc) {
        if (!n) return;
        TopLoc_Location newLoc = accLoc * n->loc;
        const XCAFNodeData* xData = n->asXCAF();
        if (xData && xData->type == TopAbs_FACE && xData->hasFace()) {
            TopoDS_Face locatedFace = TopoDS::Face(xData->getFace().Located(newLoc));
            BRepBndLib::Add(locatedFace, bbox);
        }
        for (const auto& child : n->children) {
            accumulate(child.get(), newLoc);
        }
    };
    accumulate(node, TopLoc_Location());
    if (bbox.IsVoid()) return 0.0;
    Standard_Real xmin, ymin, zmin, xmax, ymax, zmax;
    bbox.Get(xmin, ymin, zmin, xmax, ymax, zmax);
    QVector3D min(xmin, ymin, zmin);
    QVector3D max(xmax, ymax, zmax);
    QVector3D delta = max - min;
    return std::abs(QVector3D::dotProduct(delta, axis.normalized()));
}

// Tiles a rail's carriage/start/middle/end children along its axis of travel into a detached
// copy of the rail node. Returns null (after warning) when the rail lacks a Middle Segment.
static std::shared_ptr<CadNode> buildTiledRailCopy(CadNode* railNode, bool includeNonSegmentChildren) {
    RailNodeData* railData = railNode->asRail();
    int numSegments = 1;
    QVector3D axis(1,0,0);
    if (railData) {
        numSegments = railData->numSegments;
        axis = toQVector3D(railData->axisOfTravel);
    }
    CadNode* carriage = nullptr;
    CadNode* startSeg = nullptr;
    CadNode* middleSeg = nullptr;
    CadNode* endSeg = nullptr;
    for (const auto& child : railNode->children) {
        if (!child) continue;
        if (child->name.find("Carriage") != std::string::npos) carriage = child.get();
        else if (child->name.find("Start Segment") != std::string::npos) startSeg = child.get();
        else if (child->name.find("Middle Segment") != std::string::npos) middleSeg = child.get();
        else if (child->name.find("End Segment") != std::string::npos) endSeg = child.get();
    }
    if (!middleSeg) {
        QMessageBox::warning(nullptr, "Rail Error", "Rail must have at least a Middle Segment.");
        return nullptr;
    }
    double segLength = computeBoundingBoxLength(middleSeg, axis);
    if (segLength < 1e-6) segLength = 1000.0;
    auto railCopy = std::make_shared<CadNode>(*railNode);
    railCopy->children.clear();
    QVector3D currentOffset(0,0,0);
    auto appendOffsetCopy = [&](CadNode* src) {
        auto copy = deepCopyNodeNonExcluded(src);
        if (!copy) return false;
        gp_Trsf offsetTrsf;
        offsetTrsf.SetTranslationPart(gp_XYZ(currentOffset.x(), currentOffset.y(), currentOffset.z()));
        copy->loc = TopLoc_Location(src->loc.Transformation() * offsetTrsf);
        railCopy->children.push_back(std::move(copy));
        return true;
    };
    if (carriage) appendOffsetCopy(carriage);
    if (startSeg) appendOffsetCopy(startSeg);
    for (int i = 0; i < numSegments; ++i) {
        if (appendOffsetCopy(middleSeg)) currentOffset += axis * segLength;
    }
    if (endSeg) {
        currentOffset -= axis * segLength;
        appendOffsetCopy(endSeg);
    }
    if (includeNonSegmentChildren) {
        for (const auto& child : railNode->children) {
            if (!child) continue;
            if (child.get() == carriage || child.get() == startSeg || child.get() == middleSeg || child.get() == endSeg)
                continue;
            auto otherCopy = deepCopyNodeNonExcluded(child.get());
            if (otherCopy) railCopy->children.push_back(otherCopy);
        }
    }
    return railCopy;
}

void addRailToPhysicsPreview(CadNode* railNode, std::shared_ptr<CadNode> physicsPreviewRoot) {
    if (!railNode || !physicsPreviewRoot) return;
    auto railCopy = buildTiledRailCopy(railNode, false);
    if (!railCopy) return;
    physicsPreviewRoot->children.clear();
    physicsPreviewRoot->children.push_back(std::move(railCopy));
}

void collectFaceNodesWithTransform(CadNode* node, const TopLoc_Location& parentLoc, std::vector<FaceWithTransform>& out) {
    if (!node) return;
    TopLoc_Location thisLoc = parentLoc * node->loc;
    XCAFNodeData* xData = node->asXCAF();
    if (xData && xData->type == TopAbs_FACE && xData->hasFace()) {
        out.push_back({node, thisLoc});
    }
    for (auto& child : node->children) {
        collectFaceNodesWithTransform(child.get(), thisLoc, out);
    }
}

void expandRailInPhysicsPreview(CadNode* railNode, std::shared_ptr<CadNode> physicsPreviewRoot, CadOpenGLWidget* oglWidget) {
    if (!railNode || !physicsPreviewRoot) return;
    auto railCopy = buildTiledRailCopy(railNode, true);
    if (!railCopy) return;
    railCopy->name = "Expanded Rail Preview";
    physicsPreviewRoot->children.push_back(std::move(railCopy));
    if (oglWidget) oglWidget->markCacheDirty();
}

bool bakeRobotPackageHullsFile(const QString& inputPackageFile, const QString& outputPackageFile, QString* errorMessage)
{
    // Keeps its QString signature for the Qt callers and converts at the boundary; the shared
    // package and runtime API is std::string now.
    std::string localError;
    const auto report = [&]() {
        if (errorMessage) *errorMessage = QString::fromStdString(localError);
    };
    std::shared_ptr<CadNode> root = loadCadNodePackage(inputPackageFile.toStdString(), &localError);
    if (!root) { report(); return false; }
    if (!bakeRobotCollisionHulls(root.get(), &localError)) { report(); return false; }
    if (!saveCadNodePackage(outputPackageFile.toStdString(), *root,
                            inputPackageFile.toStdString(), &localError)) {
        report();
        return false;
    }
    return true;
}
