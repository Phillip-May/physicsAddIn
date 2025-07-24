#include "HelperFunctions.h"
#define ENABLE_VHACD_IMPLEMENTATION 1
#include "../physicsAddIn/v-hacd-4.1.0/include/VHACD.h"
#include "../external/CoACD/public/coacd.h"
#include "CadOpenGLWidget.h"
#include <XCAFDoc_DocumentTool.hxx>
#include <TopExp_Explorer.hxx>
#include <TDataStd_TreeNode.hxx>
#include <STEPCAFControl_Writer.hxx>
#include <STEPCAFControl_ActorWrite.hxx>
#include <TDocStd_Application.hxx>
#include <BinXCAFDrivers.hxx>
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <Geom_Surface.hxx>
#include <BRepTools.hxx>
#include <GeomLProp_SLProps.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <GProp_GProps.hxx>
#include <BRepGProp.hxx>


// Enhanced tree building that follows reference chains for STEP 214 assemblies
std::unique_ptr<XCAFLabelNode> buildLabelTreeWithReferences(const TDF_Label& label, const Handle(XCAFDoc_ShapeTool)& shapeTool) {
    auto node = std::make_unique<XCAFLabelNode>(label);

    if (!shapeTool.IsNull() && !label.IsNull()) {
        // If this is a reference, also add the referred shape as a child
        if (shapeTool->IsReference(label)) {
            TDF_Label refLabel;
            if (shapeTool->GetReferredShape(label, refLabel)) {
                qDebug() << "Following reference from label" << label.Tag() << "to" << refLabel.Tag();
                auto refNode = buildLabelTreeWithReferences(refLabel, shapeTool);
                refNode->label = refLabel; // Ensure the referred label is set correctly
                node->children.push_back(std::move(refNode));
            }
        }

        // Check if this compound has no direct geometry but might be an assembly
        TopoDS_Shape shape = shapeTool->GetShape(label);
        if (!shape.IsNull() && shape.ShapeType() == TopAbs_COMPOUND) {
            bool hasDirectGeometry = false;
            int refChildCount = 0;

            // Check children for direct geometry vs references
            for (TDF_ChildIterator it(label); it.More(); it.Next()) {
                TDF_Label childLabel = it.Value();
                if (!childLabel.IsNull()) {
                    TopoDS_Shape childShape = shapeTool->GetShape(childLabel);
                    if (!childShape.IsNull()) {
                        hasDirectGeometry = true;
                    }
                    if (shapeTool->IsReference(childLabel)) {
                        refChildCount++;
                    }
                }
            }

            // If this is an assembly with references, explore the reference chain
            if (!hasDirectGeometry && refChildCount > 0) {
                qDebug() << "Found assembly with" << refChildCount << "references at label" << label.Tag();
            }
        }
    }

    // Recursively build children
    for (TDF_ChildIterator it(label); it.More(); it.Next()) {
        TDF_Label childLabel = it.Value();
        if (!childLabel.IsNull()) {
            auto childNode = buildLabelTreeWithReferences(childLabel, shapeTool);
            node->children.push_back(std::move(childNode));
        }
    }

    return node;
}

// Helper to set parent pointers recursively in a CadNode tree
void setParentPointersRecursive(CadNode* node, CadNode* parent) {
    if (!node) return;
    node->parent = parent;
    for (auto& child : node->children) {
        setParentPointersRecursive(child.get(), node);
    }
}

// qHash overload for TDF_Label to allow use in QSet/QHash
uint qHash(const TDF_Label& label, uint seed) {
    return qHash(label.Tag(), seed);
}

QString shapeTypeToString(TopAbs_ShapeEnum type) {
    switch (type) {
        case TopAbs_COMPOUND: return "Compound";
        case TopAbs_COMPSOLID: return "CompSolid";
        case TopAbs_SOLID: return "Solid";
        case TopAbs_SHELL: return "Shell";
        case TopAbs_FACE: return "Face";
        case TopAbs_WIRE: return "Wire";
        case TopAbs_EDGE: return "Edge";
        case TopAbs_VERTEX: return "Vertex";
        case TopAbs_SHAPE: return "Shape";
        default: return "Unknown";
    }
}

#include <Quantity_Color.hxx>

CADNodeColor getEffectiveFaceColor(const TopoDS_Face& face, const Handle(XCAFDoc_ShapeTool)& shapeTool, const Handle(XCAFDoc_ColorTool)& colorTool) {
    TDF_Label faceLabel;
    Quantity_Color occColor;
    if (shapeTool->Search(face, faceLabel)) {
        if (colorTool->GetColor(faceLabel, XCAFDoc_ColorSurf, occColor) ||
            colorTool->GetColor(faceLabel, XCAFDoc_ColorCurv, occColor) ||
            colorTool->GetColor(faceLabel, XCAFDoc_ColorGen, occColor)) {
            return CADNodeColor(occColor.Red(), occColor.Green(), occColor.Blue());
        }
        TDF_Label current = faceLabel;
        while (!current.IsNull()) {
            if (colorTool->GetColor(current, XCAFDoc_ColorGen, occColor) ||
                colorTool->GetColor(current, XCAFDoc_ColorSurf, occColor) ||
                colorTool->GetColor(current, XCAFDoc_ColorCurv, occColor)) {
                return CADNodeColor(occColor.Red(), occColor.Green(), occColor.Blue());
            }
            current = current.Father();
        }
    }
    return CADNodeColor::fromSRGB(200, 200, 200);
}

CADNodeColor get_label_color(const TDF_Label& label, const Handle(XCAFDoc_ColorTool)& colorTool, const CADNodeColor& parentColor) {
    Quantity_Color occColor;
    if (colorTool->GetColor(label, XCAFDoc_ColorSurf, occColor) ||
        colorTool->GetColor(label, XCAFDoc_ColorGen, occColor) ||
        colorTool->GetColor(label, XCAFDoc_ColorCurv, occColor)) {
        return CADNodeColor(occColor.Red(), occColor.Green(), occColor.Blue());
    }
    TDF_Label current = label;
    while (!current.IsNull()) {
        current = current.Father();
        if (!current.IsNull()) {
            if (colorTool->GetColor(current, XCAFDoc_ColorGen, occColor) ||
                colorTool->GetColor(current, XCAFDoc_ColorSurf, occColor) ||
                colorTool->GetColor(current, XCAFDoc_ColorCurv, occColor)) {
                return CADNodeColor(occColor.Red(), occColor.Green(), occColor.Blue());
            }
        }
    }
    return parentColor;
}

CADNodeColor get_shape_color(const TopoDS_Shape& shape, const TDF_Label& label, const Handle(XCAFDoc_ShapeTool)& shapeTool, const Handle(XCAFDoc_ColorTool)& colorTool, const CADNodeColor& parentColor) {
    if (shape.IsNull()) {
        return get_label_color(label, colorTool, parentColor);
    }
    if (shape.ShapeType() == TopAbs_FACE) {
        TopoDS_Face face = TopoDS::Face(shape);
        return getEffectiveFaceColor(face, shapeTool, colorTool);
    }
    return get_label_color(label, colorTool, parentColor);
}

bool hasSubAssembliesRecursive(const TDF_Label& label, const Handle(XCAFDoc_ShapeTool)& shapeTool) {
    for (TDF_ChildIterator it(label); it.More(); it.Next()) {
        TDF_Label childLabel = it.Value();
        TopoDS_Shape childShape = shapeTool->GetShape(childLabel);
        if (!childShape.IsNull() && childShape.ShapeType() == TopAbs_COMPOUND) {
            return true;
        }
        if (hasSubAssembliesRecursive(childLabel, shapeTool)) {
            return true;
        }
    }
    return false;
}

QString makeTransformString(const TopLoc_Location& loc) {
    const gp_Trsf& trsf = loc.Transformation();
    const gp_Mat& mat = trsf.VectorialPart();
    const gp_XYZ& trans = trsf.TranslationPart();
    return QString("[%1 %2 %3 %4; %5 %6 %7 %8; %9 %10 %11 %12]")
        .arg(QString::number(mat.Value(1,1), 'f', 2))
        .arg(QString::number(mat.Value(1,2), 'f', 2))
        .arg(QString::number(mat.Value(1,3), 'f', 2))
        .arg(QString::number(trans.X(), 'f', 2))
        .arg(QString::number(mat.Value(2,1), 'f', 2))
        .arg(QString::number(mat.Value(2,2), 'f', 2))
        .arg(QString::number(mat.Value(2,3), 'f', 2))
        .arg(QString::number(trans.Y(), 'f', 2))
        .arg(QString::number(mat.Value(3,1), 'f', 2))
        .arg(QString::number(mat.Value(3,2), 'f', 2))
        .arg(QString::number(mat.Value(3,3), 'f', 2))
        .arg(QString::number(trans.Z(), 'f', 2));
}

bool isAssembly(const TDF_Label& label, const Handle(XCAFDoc_ShapeTool)& shapeTool) {
    TopoDS_Shape shape = shapeTool->GetShape(label);
    if (shape.IsNull() || shape.ShapeType() != TopAbs_COMPOUND) {
        return false;
    }
    int shapeCount = 0;
    for (TDF_ChildIterator it(label); it.More(); it.Next()) {
        TDF_Label childLabel = it.Value();
        TopoDS_Shape childShape = shapeTool->GetShape(childLabel);
        if (!childShape.IsNull()) {
            shapeCount++;
            if (shapeCount > 1) return true;
        }
    }
    return false;
}

std::vector<int> getLabelPath(const TDF_Label& label, const Handle(TDocStd_Document)& doc) {
    std::vector<int> path;
    TDF_Label current = label;
    TDF_Label mainLabel = doc->Main();
    while (!current.IsNull() && current != mainLabel) {
        path.push_back(current.Tag());
        current = current.Father();
    }
    std::reverse(path.begin(), path.end());
    return path;
}

TDF_Label findLabelByPath(const Handle(TDocStd_Document)& doc, const std::vector<int>& path) {
    TDF_Label label = doc->Main();
    QVariantList pathDebug;
    for (int tag : path) pathDebug << tag;
    qDebug() << "[findLabelByPath] doc->Main().Tag():" << label.Tag() << ", path:" << pathDebug;
    for (size_t i = 0; i < path.size(); ++i) {
        bool found = false;
        for (TDF_ChildIterator it(label); it.More(); it.Next()) {
            if (it.Value().Tag() == path[i]) {
                label = it.Value();
                found = true;
                break;
            }
        }
        if (!found) return TDF_Label();
    }
    return label;
}

void debugPrintXCAFRelinkInfo(const CadNode* node, const Handle(TDocStd_Document)& doc, int depth) {
    if (!node) return;
    QString indent(depth * 2, ' ');
    if (node->type == CadNodeType::XCAF) {
        const XCAFNodeData* xData = node->asXCAF();
        qDebug() << indent << "[XCAF] Node name:" << QString::fromStdString(node->name);
        if (xData) {
            QVariantList labelPathList;
            for (int tag : xData->labelPath) labelPathList << tag;
            qDebug() << indent << "  labelPath:" << labelPathList;
            qDebug() << indent << "  shapeIndex:" << xData->shapeIndex;
            qDebug() << indent << "  type:" << shapeTypeToString(xData->type);
            qDebug() << indent << "  shape isNull:" << xData->shape.IsNull();
            if (!xData->labelPath.empty()) {
                TDF_Label label = findLabelByPath(doc, xData->labelPath);
                qDebug() << indent << "  label found:" << (!label.IsNull());
                if (!label.IsNull()) {
                    TopoDS_Shape occShape = XCAFDoc_DocumentTool::ShapeTool(doc->Main())->GetShape(label);
                    qDebug() << indent << "  OCC shape isNull:" << occShape.IsNull();
                    if (xData->shapeIndex >= 0 && (xData->type == TopAbs_FACE || xData->type == TopAbs_EDGE)) {
                        int idx = 0;
                        TopoDS_Shape foundShape;
                        for (TopExp_Explorer exp(occShape, xData->type); exp.More(); exp.Next(), ++idx) {
                            if (idx == xData->shapeIndex) {
                                foundShape = exp.Current();
                                break;
                            }
                        }
                        qDebug() << indent << "  relinked shape isNull:" << foundShape.IsNull();
                    }
                }
            }
        }
    }
    for (const auto& child : node->children) debugPrintXCAFRelinkInfo(child.get(), doc, depth + 1);
}

TopLoc_Location getEffectiveTransform(const TDF_Label& label, const Handle(XCAFDoc_ShapeTool)& shapeTool, const TopLoc_Location& parentLoc) {
    TopLoc_Location effectiveLoc = parentLoc;
    if (shapeTool->IsReference(label)) {
        effectiveLoc = parentLoc * shapeTool->GetLocation(label);
    }
    Handle(TDataStd_TreeNode) treeNode;
    if (label.FindAttribute(TDataStd_TreeNode::GetDefaultTreeID(), treeNode)) {
        // STEP 214 context (not used here)
    }
    return effectiveLoc;
}

// Serialization functions for XCAF data
bool saveXCAFToSTEP(const Handle(TDocStd_Document)& doc, const QString& filename) {
    try {
        STEPCAFControl_Writer writer;
        writer.SetColorMode(true);
        writer.SetNameMode(true);
        writer.SetLayerMode(true);

        if (writer.Transfer(doc, STEPControl_AsIs)) {
            IFSelect_ReturnStatus status = writer.Write(filename.toStdString().c_str());
            if (status == IFSelect_RetDone) {
                qDebug() << "Successfully saved XCAF data to STEP file:" << filename;
                return true;
            } else {
                qDebug() << "Failed to write STEP file. Status:" << status;
                return false;
            }
        } else {
            qDebug() << "Failed to transfer document to STEP writer";
            return false;
        }
    } catch (const Standard_Failure& e) {
        qDebug() << "Exception during STEP save:" << e.GetMessageString();
        return false;
    }
}

bool saveXCAFToBinary(const Handle(TDocStd_Document)& doc, const QString& filename) {
    try {
        Handle(TDocStd_Application) app = Handle(TDocStd_Application)::DownCast(doc->Application());
        if (app.IsNull()) {
            qDebug() << "Failed to get TDocStd_Application from document";
            return false;
        }
        BinXCAFDrivers::DefineFormat(app); // Use XCAF-specific driver
        PCDM_StoreStatus status = app->SaveAs(doc, TCollection_ExtendedString(filename.toStdString().c_str()));
        if (status == PCDM_SS_OK) {
            qDebug() << "Successfully saved XCAF data to binary file:" << filename;
            return true;
        } else {
            qDebug() << "Failed to save binary file. Status:" << status;
            return false;
        }
    } catch (const Standard_Failure& e) {
        qDebug() << "Exception during binary save:" << e.GetMessageString();
        return false;
    }
}

void compareFileSizes(const QString& baseName) {
    QFileInfo stepFile(baseName + ".step");
    QFileInfo binaryFile(baseName + ".bin");
    QFileInfo xmlFile(baseName + ".xml");

    qDebug() << "\n=== FILE SIZE COMPARISON ===";

    if (stepFile.exists()) {
        qDebug() << "STEP file:" << stepFile.fileName() << "-" << stepFile.size() << "bytes";
    }

    if (binaryFile.exists()) {
        qDebug() << "Binary file:" << binaryFile.fileName() << "-" << binaryFile.size() << "bytes";
    }

    if (xmlFile.exists()) {
        qDebug() << "XML file:" << xmlFile.fileName() << "-" << xmlFile.size() << "bytes";
    }

    // Calculate ratios
    if (stepFile.exists() && binaryFile.exists()) {
        double ratio = (double)stepFile.size() / binaryFile.size();
        qDebug() << "STEP/Binary ratio:" << QString::number(ratio, 'f', 2) << "x";
    }

    if (xmlFile.exists() && binaryFile.exists()) {
        double ratio = (double)xmlFile.size() / binaryFile.size();
        qDebug() << "XML/Binary ratio:" << QString::number(ratio, 'f', 2) << "x";
    }

    qDebug() << "=== END COMPARISON ===\n";
}

// Helper: Deep copy a CadNode and all non-excluded children
std::shared_ptr<CadNode> deepCopyNodeNonExcluded(const CadNode* src) {
    if (!src || src->excludedFromDecomposition) return nullptr;
    auto copy = std::make_shared<CadNode>(*src);
    // Copy data by value for all known types
    if (src->data) {
        if (src->type == CadNodeType::XCAF) {
            copy->data = std::make_shared<XCAFNodeData>(*static_cast<const XCAFNodeData*>(src->data.get()));
        } else if (src->type == CadNodeType::Rail) {
            copy->data = std::make_shared<RailNodeData>(*static_cast<const RailNodeData*>(src->data.get()));
        } else if (src->type == CadNodeType::Physics) {
            copy->data = std::make_shared<PhysicsNodeData>(*static_cast<const PhysicsNodeData*>(src->data.get()));
        } else if (src->type == CadNodeType::Custom) {
            copy->data = std::make_shared<CustomNodeData>(*static_cast<const CustomNodeData*>(src->data.get()));
        } else if (src->type == CadNodeType::ConnectionPoint) {
            copy->data = std::make_shared<ConnectionPointData>(*static_cast<const ConnectionPointData*>(src->data.get()));
        } else if (src->type == CadNodeType::Transform) {
            copy->data = std::make_shared<TransformNodeData>(*static_cast<const TransformNodeData*>(src->data.get()));
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

// Helper: Insert a node into the custom model tree at the same position as the selected node(s) in the CAD tree
void insertCustomModelNodeAtCadTreePosition(CadNode* customModelRoot, std::shared_ptr<CadNode> newNode, const std::vector<CadNode*>& selectedCadNodes, CadTreeModel* cadModel) {
    int insertIdx = -1;
    if (!selectedCadNodes.empty()) {
        CadNode* refNode = selectedCadNodes.front();
        const CadNode* parent = cadModel->getParentNode(refNode);
        if (parent) {
            for (size_t i = 0; i < parent->children.size(); ++i) {
                if (parent->children[i].get() == refNode) {
                    insertIdx = static_cast<int>(i);
                    break;
                }
            }
        }
    }
    if (insertIdx >= 0 && insertIdx <= static_cast<int>(customModelRoot->children.size())) {
        customModelRoot->children.insert(customModelRoot->children.begin() + insertIdx, newNode);
    } else {
        customModelRoot->children.push_back(newNode);
    }
}

// Helper: Recursively adjust all descendants' transforms so their global transform matches the original
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

// Helper: Compute the bounding box length along a given axis for a node
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

// Function to add a Rail to the physics simulation with tiling
void addRailToPhysicsPreview(CadNode* railNode, std::shared_ptr<CadNode> physicsPreviewRoot) {
    if (!railNode || !physicsPreviewRoot) return;
    RailNodeData* railData = railNode->asRail();
    int numSegments = 1;
    QVector3D axis(1,0,0);
    if (railData) {
        numSegments = railData->numSegments;
        axis = railData->axisOfTravel;
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
        return;
    }
    double segLength = computeBoundingBoxLength(middleSeg, axis);
    if (segLength < 1e-6) segLength = 1000.0;
    physicsPreviewRoot->children.clear();
    auto railCopy = std::make_shared<CadNode>(*railNode);
    railCopy->children.clear();
    QVector3D currentOffset(0,0,0);
    if (carriage) {
        auto carriageCopy = deepCopyNodeNonExcluded(carriage);
        if (carriageCopy) {
            gp_Trsf offsetTrsf;
            offsetTrsf.SetTranslationPart(gp_XYZ(currentOffset.x(), currentOffset.y(), currentOffset.z()));
            gp_Trsf finalTrsf = carriage->loc.Transformation() * offsetTrsf;
            carriageCopy->loc = TopLoc_Location(finalTrsf);
            railCopy->children.push_back(carriageCopy);
        }
    }
    if (startSeg) {
        auto startCopy = deepCopyNodeNonExcluded(startSeg);
        if (startCopy) {
            gp_Trsf offsetTrsf;
            offsetTrsf.SetTranslationPart(gp_XYZ(currentOffset.x(), currentOffset.y(), currentOffset.z()));
            gp_Trsf finalTrsf = startSeg->loc.Transformation() * offsetTrsf;
            startCopy->loc = TopLoc_Location(finalTrsf);
            railCopy->children.push_back(startCopy);
        }
    }
    for (int i = 0; i < numSegments; ++i) {
        auto midCopy = deepCopyNodeNonExcluded(middleSeg);
        if (midCopy) {
            gp_Trsf offsetTrsf;
            offsetTrsf.SetTranslationPart(gp_XYZ(currentOffset.x(), currentOffset.y(), currentOffset.z()));
            gp_Trsf finalTrsf = middleSeg->loc.Transformation() * offsetTrsf;
            midCopy->loc = TopLoc_Location(finalTrsf);
            railCopy->children.push_back(midCopy);
            currentOffset += axis * segLength;
        }
    }
    if (endSeg) {
        currentOffset -= axis * segLength;
        auto endCopy = deepCopyNodeNonExcluded(endSeg);
        if (endCopy) {
            gp_Trsf offsetTrsf;
            offsetTrsf.SetTranslationPart(gp_XYZ(currentOffset.x(), currentOffset.y(), currentOffset.z()));
            gp_Trsf finalTrsf = endSeg->loc.Transformation() * offsetTrsf;
            endCopy->loc = TopLoc_Location(finalTrsf);
            railCopy->children.push_back(endCopy);
        }
    }
    physicsPreviewRoot->children.clear();
    physicsPreviewRoot->children.push_back(railCopy);
}

// Recursively search for a label whose shape matches the given shape (by TShape pointer)
TDF_Label findLabelForShape(const Handle(XCAFDoc_ShapeTool)& shapeTool, const TDF_Label& label, const TopoDS_Shape& targetShape) {
    TopoDS_Shape shape = shapeTool->GetShape(label);
    if (!shape.IsNull() && !targetShape.IsNull() && shape.TShape() == targetShape.TShape()) {
        return label;
    }
    for (TDF_ChildIterator it(label); it.More(); it.Next()) {
        TDF_Label found = findLabelForShape(shapeTool, it.Value(), targetShape);
        if (!found.IsNull()) return found;
    }
    return TDF_Label();
}

// Recursively search for a label whose shape contains a face/edge IsSame to the given shape
TDF_Label findLabelForFaceOrEdge(const Handle(XCAFDoc_ShapeTool)& shapeTool, const TDF_Label& label, const TopoDS_Shape& targetShape) {
    TopoDS_Shape shape = shapeTool->GetShape(label);
    if (!shape.IsNull()) {
        TopAbs_ShapeEnum targetType = targetShape.ShapeType();
        for (TopExp_Explorer exp(shape, targetType); exp.More(); exp.Next()) {
            if (exp.Current().IsSame(targetShape)) {
                return label;
            }
        }
    }
    for (TDF_ChildIterator it(label); it.More(); it.Next()) {
        TDF_Label found = findLabelForFaceOrEdge(shapeTool, it.Value(), targetShape);
        if (!found.IsNull()) return found;
    }
    return TDF_Label();
}

// Extrude a face by a given distance along its normal
TopoDS_Shape extrudeFace(const TopoDS_Face& face, double distance) {
    Bnd_Box bbox;
    BRepBndLib::Add(face, bbox);
    Standard_Real xmin, ymin, zmin, xmax, ymax, zmax;
    bbox.Get(xmin, ymin, zmin, xmax, ymax, zmax);
    gp_Pnt center((xmin + xmax) / 2, (ymin + ymax) / 2, (zmin + zmax) / 2);
    Handle(Geom_Surface) surf = BRep_Tool::Surface(face);
    Standard_Real umin, umax, vmin, vmax;
    BRepTools::UVBounds(face, umin, umax, vmin, vmax);
    Standard_Real ucenter = (umin + umax) / 2.0;
    Standard_Real vcenter = (vmin + vmax) / 2.0;
    GeomLProp_SLProps props(surf, ucenter, vcenter, 1, 1e-6);
    gp_Dir normal(0, 0, 1);
    if (props.IsNormalDefined()) {
        normal = props.Normal();
    }
    gp_Vec dir(normal);
    dir *= distance;
    TopoDS_Shape prism = BRepPrimAPI_MakePrism(face, dir);
    return prism;
}

// Recursive function to collect faces with accumulated transform
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

// --- BEGIN MOVED FROM main.cpp ---

void generateVHACDStub(const QString& nodeName, int resolution, int maxHulls, double minVolume, CadNode* node) {
    std::vector<FaceWithTransform> faces;
    qDebug() << "Node Name: " << node->name.c_str();
    collectFaceNodesWithTransform(node, TopLoc_Location(), faces);
    // Filter out excluded nodes
    faces.erase(std::remove_if(faces.begin(), faces.end(), [](const FaceWithTransform& f) { return f.node->excludedFromDecomposition; }), faces.end());
    qDebug() << "Face count: " << faces.size();
    int numInputFaces = static_cast<int>(faces.size());
    std::vector<VHACD::Vertex> vertices;
    std::vector<VHACD::Triangle> triangles;
    uint32_t vertOffset = 0;
    // Compute global transform for the Physics node
    TopLoc_Location physicsGlobalLoc = TopLoc_Location();
    {
        CadNode* cur = node;
        std::vector<CadNode*> ancestry;
        while (cur) {
            ancestry.push_back(cur);
            break;
        }
        for (auto it = ancestry.rbegin(); it != ancestry.rend(); ++it) {
            physicsGlobalLoc = physicsGlobalLoc * (*it)->loc;
        }
    }
    // Build mesh from all faces
    for (const auto& faceInfo : faces) {
        CadNode* faceNode = faceInfo.node;
        TopLoc_Location faceGlobalLoc = faceInfo.accumulatedLoc;
        TopLoc_Location relLoc = physicsGlobalLoc.Inverted() * faceGlobalLoc;
        XCAFNodeData* xData = faceNode->asXCAF();
        if (!xData || !xData->hasFace()) continue;
        TopoDS_Face face = xData->getFace();
        BRepMesh_IncrementalMesh mesher(face, 0.5);
        TopLoc_Location locCopy;
        Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, locCopy);
        if (tri.IsNull() || tri->NbTriangles() == 0) continue;
        std::vector<uint32_t> localIndices(tri->NbNodes());
        for (int i = 1; i <= tri->NbNodes(); ++i) {
            gp_Pnt p = tri->Node(i);
            p.Transform(relLoc.Transformation());
            vertices.push_back(VHACD::Vertex(p.X(), p.Y(), p.Z()));
            localIndices[i-1] = vertOffset++;
        }
        for (int i = tri->Triangles().Lower(); i <= tri->Triangles().Upper(); ++i) {
            int n1, n2, n3;
            tri->Triangles()(i).Get(n1, n2, n3);
            triangles.push_back(VHACD::Triangle(localIndices[n1-1], localIndices[n2-1], localIndices[n3-1]));
        }
    }
    // --- Compute original mesh volume (true solid volume) ---
    std::vector<TopoDS_Shape> solids;
    std::function<void(CadNode*)> collectSolids;
    collectSolids = [&](CadNode* node) {
        if (!node) return;
        XCAFNodeData* xData = node->asXCAF();
        if (xData && xData->type == TopAbs_SOLID && !xData->shape.IsNull()) {
            bool alreadyPresent = false;
            for (const auto& s : solids) {
                if (s.TShape().get() == xData->shape.TShape().get()) {
                    alreadyPresent = true;
                    break;
                }
            }
            if (!alreadyPresent) {
                solids.push_back(xData->shape);
            }
        }
        for (const auto& child : node->children) {
            collectSolids(child.get());
        }
    };
    collectSolids(node);
    double originalVolume = 0.0;
    for (const auto& solid : solids) {
        GProp_GProps props;
        BRepGProp::VolumeProperties(solid, props);
        originalVolume += props.Mass();
    }
    if (solids.empty()) {
        qDebug() << "[VHACD] WARNING: No solids found under this Physics node. Cannot compute true volume comparison.";
    } else {
        qDebug() << "[VHACD] Found" << solids.size() << "unique solids. Total original solid volume:" << originalVolume;
    }
    qDebug() << "[VHACD] Approximated original mesh area (sum of face areas):" << originalVolume;
    VHACD::IVHACD::Parameters params;
    params.m_resolution = resolution;
    params.m_maxConvexHulls = maxHulls;
    params.m_minimumVolumePercentErrorAllowed = minVolume;
    VHACD::IVHACD* interfaceVHACD = VHACD::CreateVHACD();
    bool ok = interfaceVHACD->Compute(&vertices[0].mX, (uint32_t)vertices.size(),
                                      (uint32_t*)&triangles[0], (uint32_t)triangles.size(), params);
    PhysicsNodeData* physData = node->asPhysics();
    physData->hulls.clear();
    physData->convexHullGenerated = ok;
    int hullCount = 0;
    double totalVolume = 0.0;
    if (ok) {
        hullCount = interfaceVHACD->GetNConvexHulls();
        for (uint32_t i = 0; i < hullCount; ++i) {
            VHACD::IVHACD::ConvexHull ch;
            interfaceVHACD->GetConvexHull(i, ch);
            ConvexHullData hullData;
            for (const auto& v : ch.m_points) {
                hullData.vertices.push_back({v.mX, v.mY, v.mZ});
            }
            for (const auto& tri : ch.m_triangles) {
                hullData.indices.push_back({tri.mI0, tri.mI1, tri.mI2});
            }
            physData->hulls.push_back(std::move(hullData));
            totalVolume += ch.m_volume;
        }
    }
    if (interfaceVHACD) interfaceVHACD->Release();
    qDebug() << "[VHACD] Node:" << nodeName
             << "Input faces:" << numInputFaces
             << "Input vertices:" << vertices.size()
             << "Input triangles:" << triangles.size()
             << "Generated hulls:" << hullCount
             << "Total hull volume:" << totalVolume
             << "Success:" << ok;
    if (originalVolume > 0.0 && totalVolume > 0.0) {
        double ratio = totalVolume / originalVolume;
        int percent = static_cast<int>(ratio * 100.0 + 0.5);
        QString biggerSmaller = (ratio > 1.0) ? "bigger" : ((ratio < 1.0) ? "smaller" : "equal");
        qDebug() << QString("[VHACD] Decomposition is %1% of original (%2) (hull/original)")
                    .arg(percent)
                    .arg(biggerSmaller);
    }
}

void generateCoACDStub(const QString& nodeName, double concavity, double alpha, double beta, CadNode* node,
    int maxConvexHull, std::string preprocess, int prepRes, int sampleRes, int mctsNodes, int mctsIter, int mctsDepth,
    bool pca, bool merge, bool decimate, int maxChVertex, bool extrude, double extrudeMargin, std::string apxMode, int seed) {
    std::vector<FaceWithTransform> faces;
    qDebug() << "Node Name: " << node->name.c_str();
    collectFaceNodesWithTransform(node, TopLoc_Location(), faces);
    faces.erase(std::remove_if(faces.begin(), faces.end(), [](const FaceWithTransform& f) { return f.node->excludedFromDecomposition; }), faces.end());
    qDebug() << "Face count: " << faces.size();
    int numInputFaces = static_cast<int>(faces.size());
    std::vector<double> vertices;
    std::vector<int> triangles;
    uint32_t vertOffset = 0;
    TopLoc_Location physicsGlobalLoc = TopLoc_Location();
    {
        CadNode* cur = node;
        std::vector<CadNode*> ancestry;
        while (cur) {
            ancestry.push_back(cur);
            break;
        }
        for (auto it = ancestry.rbegin(); it != ancestry.rend(); ++it) {
            physicsGlobalLoc = physicsGlobalLoc * (*it)->loc;
        }
    }
    for (const auto& faceInfo : faces) {
        CadNode* faceNode = faceInfo.node;
        TopLoc_Location faceGlobalLoc = faceInfo.accumulatedLoc;
        TopLoc_Location relLoc = physicsGlobalLoc.Inverted() * faceGlobalLoc;
        XCAFNodeData* xData = faceNode->asXCAF();
        if (!xData || !xData->hasFace()) continue;
        TopoDS_Face face = xData->getFace();
        BRepMesh_IncrementalMesh mesher(face, 0.5);
        TopLoc_Location locCopy;
        Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, locCopy);
        if (tri.IsNull() || tri->NbTriangles() == 0) continue;
        std::vector<uint32_t> localIndices(tri->NbNodes());
        for (int i = 1; i <= tri->NbNodes(); ++i) {
            gp_Pnt p = tri->Node(i);
            p.Transform(relLoc.Transformation());
            vertices.push_back(p.X());
            vertices.push_back(p.Y());
            vertices.push_back(p.Z());
            localIndices[i-1] = vertOffset++;
        }
        for (int i = tri->Triangles().Lower(); i <= tri->Triangles().Upper(); ++i) {
            int n1, n2, n3;
            tri->Triangles()(i).Get(n1, n2, n3);
            triangles.push_back(localIndices[n1-1]);
            triangles.push_back(localIndices[n2-1]);
            triangles.push_back(localIndices[n3-1]);
        }
    }
    std::vector<TopoDS_Shape> solids;
    std::function<void(CadNode*)> collectSolids;
    collectSolids = [&](CadNode* node) {
        if (!node) return;
        XCAFNodeData* xData = node->asXCAF();
        if (xData && xData->type == TopAbs_SOLID && !xData->shape.IsNull()) {
            bool alreadyPresent = false;
            for (const auto& s : solids) {
                if (s.TShape().get() == xData->shape.TShape().get()) {
                    alreadyPresent = true;
                    break;
                }
            }
            if (!alreadyPresent) {
                solids.push_back(xData->shape);
            }
        }
        for (const auto& child : node->children) {
            collectSolids(child.get());
        }
    };
    collectSolids(node);
    double originalVolume = 0.0;
    for (const auto& solid : solids) {
        GProp_GProps props;
        BRepGProp::VolumeProperties(solid, props);
        originalVolume += props.Mass();
    }
    coacd::Mesh inputMesh;
    for (size_t i = 0; i < vertices.size(); i += 3) {
        inputMesh.vertices.push_back({vertices[i], vertices[i+1], vertices[i+2]});
    }
    for (size_t i = 0; i < triangles.size(); i += 3) {
        inputMesh.indices.push_back({triangles[i], triangles[i+1], triangles[i+2]});
    }
    std::vector<coacd::Mesh> resultMeshes = coacd::CoACD(
        inputMesh, concavity, maxConvexHull, preprocess, prepRes, sampleRes, mctsNodes, mctsIter, mctsDepth,
        pca, merge, decimate, maxChVertex, extrude, extrudeMargin, apxMode, static_cast<unsigned int>(seed)
    );
    bool ok = !resultMeshes.empty();
    PhysicsNodeData* physData = node->asPhysics();
    physData->hulls.clear();
    physData->convexHullGenerated = ok;
    int hullCount = 0;
    double totalVolume = 0.0;
    if (ok) {
        hullCount = static_cast<int>(resultMeshes.size());
        for (size_t i = 0; i < resultMeshes.size(); ++i) {
            ConvexHullData hullData;
            const auto& resultMesh = resultMeshes[i];
            for (const auto& vertex : resultMesh.vertices) {
                hullData.vertices.push_back({
                    static_cast<float>(vertex[0]),
                    static_cast<float>(vertex[1]),
                    static_cast<float>(vertex[2])
                });
            }
            for (const auto& triangle : resultMesh.indices) {
                hullData.indices.push_back({
                    static_cast<uint32_t>(triangle[0]),
                    static_cast<uint32_t>(triangle[1]),
                    static_cast<uint32_t>(triangle[2])
                });
            }
            physData->hulls.push_back(std::move(hullData));
            double hullVolume = 0.0;
            for (const auto& triangle : resultMesh.indices) {
                const auto& v1 = resultMesh.vertices[triangle[0]];
                const auto& v2 = resultMesh.vertices[triangle[1]];
                const auto& v3 = resultMesh.vertices[triangle[2]];
                hullVolume += ((v1[0] * v2[1] * v3[2]) + (v1[1] * v2[2] * v3[0]) + (v1[2] * v2[0] * v3[1]) -
                              (v3[0] * v2[1] * v1[2]) - (v3[1] * v2[2] * v1[0]) - (v3[2] * v2[0] * v1[1])) / 6.0;
            }
            totalVolume += std::abs(hullVolume);
        }
    }
    qDebug() << "[CoACD] Node:" << nodeName
             << "Input faces:" << numInputFaces
             << "Input vertices:" << vertices.size() / 3
             << "Input triangles:" << triangles.size() / 3
             << "Generated hulls:" << hullCount
             << "Total hull volume:" << totalVolume
             << "Success:" << ok;
    if (originalVolume > 0.0 && totalVolume > 0.0) {
        double ratio = totalVolume / originalVolume;
        int percent = static_cast<int>(ratio * 100.0 + 0.5);
        QString biggerSmaller = (ratio > 1.0) ? "bigger" : ((ratio < 1.0) ? "smaller" : "equal");
        qDebug() << QString("[CoACD] Decomposition is %1% of original (%2) (hull/original)")
                    .arg(percent)
                    .arg(biggerSmaller);
    }
}

void expandRailInPhysicsPreview(CadNode* railNode, std::shared_ptr<CadNode> physicsPreviewRoot, CadOpenGLWidget* oglWidget) {
    if (!railNode || !physicsPreviewRoot) return;
    RailNodeData* railData = railNode->asRail();
    int numSegments = 1;
    QVector3D axis(1,0,0);
    if (railData) {
        numSegments = railData->numSegments;
        axis = railData->axisOfTravel;
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
        return;
    }
    double segLength = computeBoundingBoxLength(middleSeg, axis);
    if (segLength < 1e-6) segLength = 1000.0;
    // Do NOT clear physicsPreviewRoot->children here (allow multiple rails)
    auto railCopy = std::make_shared<CadNode>(*railNode);
    railCopy->children.clear();
    railCopy->name = "Expanded Rail Preview";
    QVector3D currentOffset(0,0,0);
    if (carriage) {
        auto carriageCopy = deepCopyNodeNonExcluded(carriage);
        if (carriageCopy) {
            gp_Trsf offsetTrsf;
            offsetTrsf.SetTranslationPart(gp_XYZ(currentOffset.x(), currentOffset.y(), currentOffset.z()));
            gp_Trsf finalTrsf = carriage->loc.Transformation() * offsetTrsf;
            carriageCopy->loc = TopLoc_Location(finalTrsf);
            railCopy->children.push_back(carriageCopy);
        }
    }
    if (startSeg) {
        auto startCopy = deepCopyNodeNonExcluded(startSeg);
        if (startCopy) {
            gp_Trsf offsetTrsf;
            offsetTrsf.SetTranslationPart(gp_XYZ(currentOffset.x(), currentOffset.y(), currentOffset.z()));
            gp_Trsf finalTrsf = startSeg->loc.Transformation() * offsetTrsf;
            startCopy->loc = TopLoc_Location(finalTrsf);
            railCopy->children.push_back(startCopy);
        }
    }
    for (int i = 0; i < numSegments; ++i) {
        auto midCopy = deepCopyNodeNonExcluded(middleSeg);
        if (midCopy) {
            gp_Trsf offsetTrsf;
            offsetTrsf.SetTranslationPart(gp_XYZ(currentOffset.x(), currentOffset.y(), currentOffset.z()));
            gp_Trsf finalTrsf = middleSeg->loc.Transformation() * offsetTrsf;
            midCopy->loc = TopLoc_Location(finalTrsf);
            railCopy->children.push_back(midCopy);
            currentOffset += axis * segLength;
        }
    }
    if (endSeg) {
        currentOffset -= axis * segLength;
        auto endCopy = deepCopyNodeNonExcluded(endSeg);
        if (endCopy) {
            gp_Trsf offsetTrsf;
            offsetTrsf.SetTranslationPart(gp_XYZ(currentOffset.x(), currentOffset.y(), currentOffset.z()));
            gp_Trsf finalTrsf = endSeg->loc.Transformation() * offsetTrsf;
            endCopy->loc = TopLoc_Location(finalTrsf);
            railCopy->children.push_back(endCopy);
        }
    }
    // Add any other children that are not segments or carriage
    for (const auto& child : railNode->children) {
        if (!child) continue;
        if (child.get() == carriage || child.get() == startSeg || child.get() == middleSeg || child.get() == endSeg)
            continue;
        auto otherCopy = deepCopyNodeNonExcluded(child.get());
        if (otherCopy) {
            railCopy->children.push_back(otherCopy);
        }
    }
    // Add the expanded rail as a new child (do not clear existing children)
    physicsPreviewRoot->children.push_back(railCopy);
    if (oglWidget) {
        oglWidget->setRootTreeNode(physicsPreviewRoot->children.back().get());
        oglWidget->markCacheDirty();
    }
}
