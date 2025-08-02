#include "HelperFunctions.h"
#include "CustomModelTreeModel.h"
#include "CadOpenGLWidget.h"
#include "CadTreeModel.h"
#include "XCAFLabelTreeModel.h"
#include <QTreeView>
#include <QTabWidget>
#include <QMenu>
#include <QAction>
#include <QFileDialog>
#include <QMessageBox>
#include <QJsonDocument>
#include <QJsonObject>
#include <QVector3D>
#include <QIODevice>
#include <QAbstractItemView>
#include <QModelIndex>
#include <QItemSelectionModel>
#include <QItemSelection>
#include <QSet>
#include <chrono>
#include <functional>
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
#include <BinDrivers.hxx>
#include <XmlDrivers.hxx>
#include <TCollection_ExtendedString.hxx>
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <Geom_Surface.hxx>
#include <BRepTools.hxx>
#include <GeomLProp_SLProps.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <GProp_GProps.hxx>
#include <BRepGProp.hxx>
#include <unordered_map>
#include <XCAFApp_Application.hxx>
#include <STEPCAFControl_Reader.hxx>
#include <TDataStd_Name.hxx>
#include <QLabel>
#include <QFile>
#include <QJsonParseError>
#include <QFileInfo>
#include <QDialog>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QComboBox>
#include <QPushButton>
#include "RailJsonEditorDialog.h"
#include "ConnectionCreationWidget.h"
#include "SimulationManager.h"

// Global vectors for tracking tree views and OpenGL widgets
std::vector<QTreeView*> g_treeViews;
std::vector<CadOpenGLWidget*> g_openGLViews;

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

// Utility: Remove duplicate vertices and remap indices, skipping degenerate triangles
// VertType: array<double, 3> or similar
// TriType: array<int/uint32_t, 3> or similar
template <typename VertType, typename TriType>
void filterAndRemapVerticesAndIndices(
    const std::vector<VertType>& inputVerts,
    const std::vector<TriType>& inputTris,
    std::vector<VertType>& uniqueVerts,
    std::vector<std::array<uint32_t, 3>>& remappedTris,
    double epsilon = 1e-4)
{
    uniqueVerts.clear();
    remappedTris.clear();
    std::vector<int> oldToNew(inputVerts.size(), -1);
    for (size_t vi = 0; vi < inputVerts.size(); ++vi) {
        const auto& p = inputVerts[vi];
        int found = -1;
        for (size_t uj = 0; uj < uniqueVerts.size(); ++uj) {
            double dx = uniqueVerts[uj][0] - p[0], dy = uniqueVerts[uj][1] - p[1], dz = uniqueVerts[uj][2] - p[2];
            if (dx*dx + dy*dy + dz*dz < epsilon*epsilon) { found = (int)uj; break; }
        }
        if (found == -1) {
            oldToNew[vi] = (int)uniqueVerts.size();
            uniqueVerts.push_back(p);
        } else {
            oldToNew[vi] = found;
        }
    }
    for (const auto& tri : inputTris) {
        int i0 = oldToNew[tri[0]];
        int i1 = oldToNew[tri[1]];
        int i2 = oldToNew[tri[2]];
        if (i0 == i1 || i1 == i2 || i2 == i0) continue;
        remappedTris.push_back({(uint32_t)i0, (uint32_t)i1, (uint32_t)i2});
    }
}

// --- BEGIN MOVED FROM main.cpp ---

void generateVHACDStub(const QString& nodeName, const QJsonObject& params, CadNode* node) {
    uint32_t maxConvexHulls = params.value("maxConvexHulls").toInt(8);
    uint32_t resolution = params.value("resolution").toInt(200000);
    double minimumVolumePercentErrorAllowed = params.value("minimumVolumePercentErrorAllowed").toDouble(1.0);
    uint32_t maxRecursionDepth = params.value("maxRecursionDepth").toInt(10);
    bool shrinkWrap = params.value("shrinkWrap").toBool(true);
    int fillMode = params.value("fillMode").toInt(0);
    uint32_t maxNumVerticesPerCH = params.value("maxNumVerticesPerCH").toInt(64);
    bool asyncACD = params.value("asyncACD").toBool(true);
    uint32_t minEdgeLength = params.value("minEdgeLength").toInt(2);
    bool findBestPlane = params.value("findBestPlane").toBool(false);
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
    VHACD::IVHACD::Parameters vhacdParams;
    vhacdParams.m_maxConvexHulls = maxConvexHulls;
    vhacdParams.m_resolution = resolution;
    vhacdParams.m_minimumVolumePercentErrorAllowed = minimumVolumePercentErrorAllowed;
    vhacdParams.m_maxRecursionDepth = maxRecursionDepth;
    vhacdParams.m_shrinkWrap = shrinkWrap;
    vhacdParams.m_fillMode = static_cast<VHACD::FillMode>(fillMode);
    vhacdParams.m_maxNumVerticesPerCH = maxNumVerticesPerCH;
    vhacdParams.m_asyncACD = asyncACD;
    vhacdParams.m_minEdgeLength = minEdgeLength;
    vhacdParams.m_findBestPlane = findBestPlane;
    VHACD::IVHACD* interfaceVHACD = VHACD::CreateVHACD();
    bool ok = interfaceVHACD->Compute(&vertices[0].mX, (uint32_t)vertices.size(),
                                      (uint32_t*)&triangles[0], (uint32_t)triangles.size(), vhacdParams);
    PhysicsNodeData* physData = node->asPhysics();
    physData->hulls.clear();
    physData->convexHullGenerated = ok;
    int hullCount = 0;
    double totalVolume = 0.0;
    if (ok) {
        hullCount = interfaceVHACD->GetNConvexHulls();
        const double epsilon = 1e-4; // 0.1mm for duplicate detection
        for (uint32_t i = 0; i < hullCount; ++i) {
            VHACD::IVHACD::ConvexHull ch;
            interfaceVHACD->GetConvexHull(i, ch);
            ConvexHullData hullData;
            // Prepare input verts and tris
            std::vector<std::array<double, 3>> inputVerts;
            std::vector<std::array<uint32_t, 3>> inputTris;
            for (const auto& v : ch.m_points) {
                inputVerts.push_back({static_cast<double>(v.mX), static_cast<double>(v.mY), static_cast<double>(v.mZ)});
            }
            for (const auto& tri : ch.m_triangles) {
                inputTris.push_back({tri.mI0, tri.mI1, tri.mI2});
            }
            std::vector<std::array<double, 3>> uniqueVerts;
            std::vector<std::array<uint32_t, 3>> remappedTris;
            filterAndRemapVerticesAndIndices(inputVerts, inputTris, uniqueVerts, remappedTris, epsilon);
            if (uniqueVerts.size() < 4 || remappedTris.empty()) {
                qDebug() << "[VHACD] Skipping degenerate hull (" << uniqueVerts.size() << " unique verts, " << remappedTris.size() << " triangles)";
                continue;
            }
            // Warn if hull is very small
            double minX=1e30,maxX=-1e30,minY=1e30,maxY=-1e30,minZ=1e30,maxZ=-1e30;
            for (const auto& p : uniqueVerts) {
                minX = std::min(minX, p[0]); maxX = std::max(maxX, p[0]);
                minY = std::min(minY, p[1]); maxY = std::max(maxY, p[1]);
                minZ = std::min(minZ, p[2]); maxZ = std::max(maxZ, p[2]);
            }
            double extentX = maxX - minX;
            double extentY = maxY - minY;
            double extentZ = maxZ - minZ;
            double diag = std::sqrt(extentX*extentX + extentY*extentY + extentZ*extentZ);
            if (diag < 1e-2) {
                qDebug() << "[VHACD] Skipping very small hull (diag=" << diag << ")";
                continue;
            }
            int smallDims = 0;
            if (extentX < 1e-3) smallDims++;
            if (extentY < 1e-3) smallDims++;
            if (extentZ < 1e-3) smallDims++;
            if (smallDims >= 2) {
                qDebug() << "[VHACD] Skipping nearly flat/coplanar/collinear hull (extents: " << extentX << extentY << extentZ << ")";
                continue;
            }
            hullData.vertices = uniqueVerts;
            hullData.indices = remappedTris;
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
        const double epsilon = 1e-4; // 0.1mm for duplicate detection
        for (size_t i = 0; i < resultMeshes.size(); ++i) {
            ConvexHullData hullData;
            const auto& resultMesh = resultMeshes[i];
            // Prepare input verts and tris
            std::vector<std::array<double, 3>> inputVerts;
            std::vector<std::array<uint32_t, 3>> inputTris;
            for (const auto& v : resultMesh.vertices) {
                inputVerts.push_back({static_cast<double>(v[0]), static_cast<double>(v[1]), static_cast<double>(v[2])});
            }
            for (const auto& tri : resultMesh.indices) {
                inputTris.push_back({(uint32_t)tri[0], (uint32_t)tri[1], (uint32_t)tri[2]});
            }
            std::vector<std::array<double, 3>> uniqueVerts;
            std::vector<std::array<uint32_t, 3>> remappedTris;
            filterAndRemapVerticesAndIndices(inputVerts, inputTris, uniqueVerts, remappedTris, epsilon);
            if (uniqueVerts.size() < 4 || remappedTris.empty()) {
                qDebug() << "[CoACD] Skipping degenerate hull (" << uniqueVerts.size() << " unique verts, " << remappedTris.size() << " triangles)";
                continue;
            }
            // Warn if hull is very small
            double minX=1e30,maxX=-1e30,minY=1e30,maxY=-1e30,minZ=1e30,maxZ=-1e30;
            for (const auto& p : uniqueVerts) {
                minX = std::min(minX, p[0]); maxX = std::max(maxX, p[0]);
                minY = std::min(minY, p[1]); maxY = std::max(maxY, p[1]);
                minZ = std::min(minZ, p[2]); maxZ = std::max(maxZ, p[2]);
            }
            double extentX = maxX - minX;
            double extentY = maxY - minY;
            double extentZ = maxZ - minZ;
            double diag = std::sqrt(extentX*extentX + extentY*extentY + extentZ*extentZ);
            int smallDims = 0;
            if (diag < 1e-2) {
                qDebug() << "[CoACD] Skipping very small hull (diag=" << diag << ")";
                continue;
            }
            if (extentX < 1e-3) smallDims++;
            if (extentY < 1e-3) smallDims++;
            if (extentZ < 1e-3) smallDims++;
            if (smallDims >= 2) {
                qDebug() << "[CoACD] Skipping nearly flat/coplanar/collinear hull (extents: " << extentX << extentY << extentZ << ")";
                continue;
            }
            hullData.vertices = uniqueVerts;
            hullData.indices = remappedTris;
            physData->hulls.push_back(std::move(hullData));
            double hullVolume = 0.0;
            for (const auto& triangle : remappedTris) {
                const auto& v1 = uniqueVerts[triangle[0]];
                const auto& v2 = uniqueVerts[triangle[1]];
                const auto& v3 = uniqueVerts[triangle[2]];
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
        oglWidget->markCacheDirty();
    }
}

// Shared STEP file loading function
bool loadStepFile(const QString& stepFile,
                  Handle(TDocStd_Document)& doc,
                  std::shared_ptr<CadNode>& cadRoot,
                  std::unique_ptr<XCAFLabelNode>& labelRoot,
                  Handle(XCAFDoc_ShapeTool)& shapeTool,
                  Handle(XCAFDoc_ColorTool)& colorTool)
{
    Handle(XCAFApp_Application) appOCC = XCAFApp_Application::GetApplication();
    appOCC->NewDocument("BinXCAF", doc);
    STEPCAFControl_Reader reader;
    reader.SetColorMode(true);
    reader.SetNameMode(true);
    reader.SetLayerMode(true);
    if (reader.ReadFile(stepFile.toStdString().c_str()) != IFSelect_RetDone) {
        QMessageBox::critical(nullptr, "Error", "Failed to read STEP file.");
        return false;
    }
    if (!reader.Transfer(doc)) {
        QMessageBox::critical(nullptr, "Error", "Failed to transfer STEP file.");
        return false;
    }
    // Get shape and color tools from the document
    shapeTool = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
    colorTool = XCAFDoc_DocumentTool::ColorTool(doc->Main());
    // Build main CAD tree from XCAF
    CADNodeColor defaultColor = CADNodeColor::fromSRGB(200, 200, 200);
    TopLoc_Location identityLoc;
    TDF_LabelSequence roots;
    shapeTool->GetFreeShapes(roots);
    cadRoot = std::make_shared<CadNode>();
    cadRoot->name = "Root";
    cadRoot->color = defaultColor;
    cadRoot->loc = identityLoc;
    for (Standard_Integer i = 1; i <= roots.Length(); ++i) {
        TDF_Label rootLabel = roots.Value(i);
        auto child = build_tree_xcaf(rootLabel, shapeTool, colorTool, defaultColor, identityLoc, doc);
        if (child) {
            cadRoot->children.push_back(std::move(child));
            setGlobalLocRecursive(cadRoot->children.back().get(), cadRoot->globalLoc);
        }
    }
    // Build XCAF label tree
    labelRoot = std::make_unique<XCAFLabelNode>(doc->Main());
    labelRoot->label = doc->Main();
    for (TDF_ChildIterator it(doc->Main()); it.More(); it.Next()) {
        TDF_Label childLabel = it.Value();
        labelRoot->children.push_back(buildLabelTreeWithReferences(childLabel, shapeTool));
    }
    TDF_LabelSequence freeShapes;
    shapeTool->GetFreeShapes(freeShapes);
    for (Standard_Integer i = 1; i <= freeShapes.Length(); ++i) {
        TDF_Label freeShapeLabel = freeShapes.Value(i);
        bool alreadyAdded = false;
        for (const auto& existingChild : labelRoot->children) {
            if (existingChild->label == freeShapeLabel) {
                alreadyAdded = true;
                break;
            }
        }
        if (!alreadyAdded) {
            labelRoot->children.push_back(buildLabelTreeWithReferences(freeShapeLabel, shapeTool));
        }
    }
    // Add a special node for all shapes in the document
    auto allShapesNode = std::make_unique<XCAFLabelNode>(TDF_Label());
    allShapesNode->label = TDF_Label();
    std::function<void(const TDF_Label&)> collectShapeLabels = [&](const TDF_Label& label) {
        if (!label.IsNull()) {
            TopoDS_Shape shape = shapeTool->GetShape(label);
            if (!shape.IsNull()) {
                auto shapeNode = std::make_unique<XCAFLabelNode>(label);
                allShapesNode->children.push_back(std::move(shapeNode));
            }
            for (TDF_ChildIterator it(label); it.More(); it.Next()) {
                collectShapeLabels(it.Value());
            }
        }
    };
    collectShapeLabels(doc->Main());
    if (!allShapesNode->children.empty()) {
        labelRoot->children.push_back(std::move(allShapesNode));
    }
    shapeTool = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
    colorTool = XCAFDoc_DocumentTool::ColorTool(doc->Main());
    return true;
}

// Shared tree-viewer connection function
template <typename ModelType>
void connectTreeAndViewer(QTreeView* tree, CadOpenGLWidget* viewer, ModelType* model) {
    // Helper to accumulate transform from root to node
    auto accumulateTransform = [](CadNode* node, std::function<CadNode*(CadNode*)> getParent) -> TopLoc_Location {
        TopLoc_Location acc;
        std::vector<CadNode*> ancestry;
        CadNode* cur = node;
        while (cur) {
            ancestry.push_back(cur);
            cur = getParent(cur);
        }
        for (auto it = ancestry.rbegin(); it != ancestry.rend(); ++it) {
            acc = acc * (*it)->loc;
        }
        return acc;
    };
    // Tree -> Viewer
    QObject::connect(tree->selectionModel(), &QItemSelectionModel::selectionChanged, viewer, [=](const QItemSelection &, const QItemSelection &) {
        viewer->clearSelection();
        QModelIndexList selectedIndexes = tree->selectionModel()->selectedIndexes();
        QSet<CadNode*> allNodesToSelect;
        for (const QModelIndex& index : selectedIndexes) {
            CadNode* node = const_cast<CadNode*>(model->getNode(index));
            if (node) {
                allNodesToSelect.insert(node);
                // Optionally add descendants
                std::function<void(CadNode*)> addDescendants = [&](CadNode* currentNode) {
                    for (const auto& child : currentNode->children) {
                        if (child) {
                            allNodesToSelect.insert(child.get());
                            addDescendants(child.get());
                        }
                    }
                };
                addDescendants(node);
            }
        }
        for (CadNode* node : allNodesToSelect) {
            TopLoc_Location accLoc = node->globalLoc;
            viewer->addToSelection(node, accLoc);
        }
    });

    // Viewer -> Tree
    QObject::connect(viewer, &CadOpenGLWidget::facePicked, tree, [=](CadNode* node) {
        QModelIndex idx = model->indexForNode(node);
        if (idx.isValid()) {
            QModelIndex parentIdx = idx.parent();
            while (parentIdx.isValid()) {
                tree->expand(parentIdx);
                parentIdx = parentIdx.parent();
            }
            // Multi-selection: add to selection instead of replacing
            QItemSelectionModel* selectionModel = tree->selectionModel();
            QItemSelection currentSelection = selectionModel->selection();
            currentSelection.select(idx, idx);
            selectionModel->select(currentSelection, QItemSelectionModel::Select);
        }
    });
}

// Explicit template instantiations
template void connectTreeAndViewer(QTreeView* tree, CadOpenGLWidget* viewer, CustomModelTreeModel* model);
template void connectTreeAndViewer(QTreeView* tree, CadOpenGLWidget* viewer, CadTreeModel* model);

// Shared context menu setup function
void setupContextMenu(QTreeView* treeView, CustomModelTreeModel* model, const Handle(TDocStd_Document)& doc) {
    setupComprehensiveContextMenu(treeView, model, doc, nullptr, nullptr);
}

// Comprehensive context menu setup function that can handle different node types
void setupComprehensiveContextMenu(QTreeView* treeView, 
                                   QAbstractItemModel* model, 
                                   const Handle(TDocStd_Document)& doc,
                                   CadOpenGLWidget* openGLViewer,
                                   SimulationManager* simManager) {
    treeView->setContextMenuPolicy(Qt::CustomContextMenu);
    QObject::connect(treeView, &QTreeView::customContextMenuRequested, treeView, [=](const QPoint& pos) {
        QModelIndex idx = treeView->indexAt(pos);
        if (!idx.isValid()) return;
        
        QMenu menu;
        
        // Handle CadNode type (CustomModelTreeModel)
        CustomModelTreeModel* customModel = qobject_cast<CustomModelTreeModel*>(model);
        if (customModel) {
            CadNode* node = const_cast<CadNode*>(customModel->getNode(idx));
            if (!node) return;
            
            // --- Save/Load Node to/from JSON ---
            QAction* saveNodeAction = menu.addAction("Save Node to JSON...");
            QObject::connect(saveNodeAction, &QAction::triggered, treeView, [=]() {
                QString fileName = QFileDialog::getSaveFileName(nullptr, "Save Node as JSON", "", "JSON Files (*.json)");
                if (fileName.isEmpty()) return;
                if (!fileName.endsWith(".json")) fileName += ".json";
                std::shared_ptr<CadNode> nodeToSave = std::make_shared<CadNode>(*node);
                QJsonObject obj = nodeToSave->toJson();
                QJsonDocument jsondoc(obj);
                QFile file(fileName);
                if (!file.open(QIODevice::WriteOnly)) {
                    QMessageBox::warning(nullptr, "Error", "Failed to open file for writing: " + fileName);
                    return;
                }
                file.write(jsondoc.toJson(QJsonDocument::Indented));
                file.close();
                QMessageBox::information(nullptr, "Success", "Node saved to:\n" + fileName);
            });
            
            QAction* loadNodeAction = menu.addAction("Load Node from JSON...");
            QObject::connect(loadNodeAction, &QAction::triggered, treeView, [=]() {
                QString fileName = QFileDialog::getOpenFileName(nullptr, "Load Node from JSON", "", "JSON Files (*.json)");
                if (fileName.isEmpty()) return;
                QFile file(fileName);
                if (!file.open(QIODevice::ReadOnly)) return;
                QByteArray data = file.readAll();
                file.close();
                QJsonParseError err;
                QJsonDocument docJSON = QJsonDocument::fromJson(data, &err);
                if (docJSON.isNull() || !docJSON.isObject()) return;
                std::shared_ptr<CadNode> loadedNode = CadNode::fromJson(docJSON.object());
                if (!loadedNode) return;
                
                // Update node properties
                node->children.clear();
                for (auto& child : loadedNode->children) {
                    node->children.push_back(child);
                }
                node->name = loadedNode->name;
                node->type = loadedNode->type;
                node->color = loadedNode->color;
                node->loc = loadedNode->loc;
                node->visible = loadedNode->visible;
                node->excludedFromDecomposition = loadedNode->excludedFromDecomposition;
                node->data = loadedNode->data;
                customModel->dataChanged(idx, idx);
                setParentPointersRecursive(node);
                
                // Relink XCAF nodes
                std::function<void(CadNode*)> relinkXCAF = [&](CadNode* n) {
                    if (!n) return;
                    if (n->type == CadNodeType::XCAF) {
                        XCAFNodeData* xData = n->asXCAF();
                        if (xData && !xData->labelPath.empty()) {
                            TDF_Label label = findLabelByPath(doc, xData->labelPath);
                            if (!label.IsNull()) {
                                auto shapeToolLocal = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
                                TopoDS_Shape occShape = shapeToolLocal->GetShape(label);
                                if (xData->type == TopAbs_FACE || xData->type == TopAbs_EDGE) {
                                    if (xData->shapeIndex >= 0) {
                                        int idx = 0;
                                        TopoDS_Shape foundShape;
                                        for (TopExp_Explorer exp(occShape, xData->type); exp.More(); exp.Next(), ++idx) {
                                            if (idx == xData->shapeIndex) {
                                                foundShape = exp.Current();
                                                break;
                                            }
                                        }
                                        xData->shape = foundShape;
                                    } else {
                                        xData->shape = occShape;
                                    }
                                } else {
                                    xData->shape = occShape;
                                }
                            } else {
                                xData->shape.Nullify();
                            }
                        }
                    }
                    for (auto& child : n->children) {
                        relinkXCAF(child.get());
                    }
                };
                relinkXCAF(node);
                if (openGLViewer) {
                    openGLViewer->markCacheDirty();
                    openGLViewer->update();
                }
            });
            
            // --- Edit Node Location/Transform ---
            QAction* editLocationAction = menu.addAction("Edit Location...");
            QObject::connect(editLocationAction, &QAction::triggered, treeView, [=]() {
                QDialog dialog(treeView);
                dialog.setWindowTitle("Edit Node Location - " + QString::fromStdString(node->name));
                dialog.setModal(true);
                dialog.resize(400, 300);
                
                QVBoxLayout* mainLayout = new QVBoxLayout(&dialog);
                
                // Get current transform values
                const gp_Trsf& trsf = node->loc.Transformation();
                const gp_Mat& mat = trsf.VectorialPart();
                const gp_XYZ& trans = trsf.TranslationPart();
                
                // Translation group
                QGroupBox* transGroup = new QGroupBox("Translation");
                QGridLayout* transLayout = new QGridLayout(transGroup);
                
                QDoubleSpinBox* xTrans = new QDoubleSpinBox;
                QDoubleSpinBox* yTrans = new QDoubleSpinBox;
                QDoubleSpinBox* zTrans = new QDoubleSpinBox;
                
                xTrans->setRange(-10000, 10000);
                yTrans->setRange(-10000, 10000);
                zTrans->setRange(-10000, 10000);
                xTrans->setDecimals(3);
                yTrans->setDecimals(3);
                zTrans->setDecimals(3);
                xTrans->setValue(trans.X());
                yTrans->setValue(trans.Y());
                zTrans->setValue(trans.Z());
                
                transLayout->addWidget(new QLabel("X:"), 0, 0);
                transLayout->addWidget(xTrans, 0, 1);
                transLayout->addWidget(new QLabel("Y:"), 1, 0);
                transLayout->addWidget(yTrans, 1, 1);
                transLayout->addWidget(new QLabel("Z:"), 2, 0);
                transLayout->addWidget(zTrans, 2, 1);
                
                // Rotation group (Euler angles)
                QGroupBox* rotGroup = new QGroupBox("Rotation (Euler angles in degrees)");
                QGridLayout* rotLayout = new QGridLayout(rotGroup);
                
                QDoubleSpinBox* xRot = new QDoubleSpinBox;
                QDoubleSpinBox* yRot = new QDoubleSpinBox;
                QDoubleSpinBox* zRot = new QDoubleSpinBox;
                
                xRot->setRange(-360, 360);
                yRot->setRange(-360, 360);
                zRot->setRange(-360, 360);
                xRot->setDecimals(1);
                yRot->setDecimals(1);
                zRot->setDecimals(1);
                
                // Convert rotation matrix to Euler angles (simplified)
                // This is a basic conversion - for more complex cases you might want a quaternion editor
                double rx = atan2(mat.Value(3,2), mat.Value(3,3)) * 180.0 / M_PI;
                double ry = atan2(-mat.Value(3,1), sqrt(mat.Value(3,2)*mat.Value(3,2) + mat.Value(3,3)*mat.Value(3,3))) * 180.0 / M_PI;
                double rz = atan2(mat.Value(2,1), mat.Value(1,1)) * 180.0 / M_PI;
                
                xRot->setValue(rx);
                yRot->setValue(ry);
                zRot->setValue(rz);
                
                rotLayout->addWidget(new QLabel("X (Pitch):"), 0, 0);
                rotLayout->addWidget(xRot, 0, 1);
                rotLayout->addWidget(new QLabel("Y (Yaw):"), 1, 0);
                rotLayout->addWidget(yRot, 1, 1);
                rotLayout->addWidget(new QLabel("Z (Roll):"), 2, 0);
                rotLayout->addWidget(zRot, 2, 1);
                
                // Scale group
                QGroupBox* scaleGroup = new QGroupBox("Scale");
                QGridLayout* scaleLayout = new QGridLayout(scaleGroup);
                
                QDoubleSpinBox* xScale = new QDoubleSpinBox;
                QDoubleSpinBox* yScale = new QDoubleSpinBox;
                QDoubleSpinBox* zScale = new QDoubleSpinBox;
                
                xScale->setRange(0.001, 1000);
                yScale->setRange(0.001, 1000);
                zScale->setRange(0.001, 1000);
                xScale->setDecimals(3);
                yScale->setDecimals(3);
                zScale->setDecimals(3);
                
                // Extract scale from matrix (simplified)
                double sx = sqrt(mat.Value(1,1)*mat.Value(1,1) + mat.Value(2,1)*mat.Value(2,1) + mat.Value(3,1)*mat.Value(3,1));
                double sy = sqrt(mat.Value(1,2)*mat.Value(1,2) + mat.Value(2,2)*mat.Value(2,2) + mat.Value(3,2)*mat.Value(3,2));
                double sz = sqrt(mat.Value(1,3)*mat.Value(1,3) + mat.Value(2,3)*mat.Value(2,3) + mat.Value(3,3)*mat.Value(3,3));
                
                xScale->setValue(sx);
                yScale->setValue(sy);
                zScale->setValue(sz);
                
                scaleLayout->addWidget(new QLabel("X:"), 0, 0);
                scaleLayout->addWidget(xScale, 0, 1);
                scaleLayout->addWidget(new QLabel("Y:"), 1, 0);
                scaleLayout->addWidget(yScale, 1, 1);
                scaleLayout->addWidget(new QLabel("Z:"), 2, 0);
                scaleLayout->addWidget(zScale, 2, 1);
                
                // Buttons
                QHBoxLayout* buttonLayout = new QHBoxLayout;
                QPushButton* okButton = new QPushButton("OK");
                QPushButton* cancelButton = new QPushButton("Cancel");
                QPushButton* resetButton = new QPushButton("Reset to Identity");
                
                buttonLayout->addWidget(resetButton);
                buttonLayout->addStretch();
                buttonLayout->addWidget(cancelButton);
                buttonLayout->addWidget(okButton);
                
                // Add all widgets to main layout
                mainLayout->addWidget(transGroup);
                mainLayout->addWidget(rotGroup);
                mainLayout->addWidget(scaleGroup);
                mainLayout->addLayout(buttonLayout);
                
                // Connect reset button
                QObject::connect(resetButton, &QPushButton::clicked, [&]() {
                    xTrans->setValue(0.0);
                    yTrans->setValue(0.0);
                    zTrans->setValue(0.0);
                    xRot->setValue(0.0);
                    yRot->setValue(0.0);
                    zRot->setValue(0.0);
                    xScale->setValue(1.0);
                    yScale->setValue(1.0);
                    zScale->setValue(1.0);
                });
                
                // Connect OK/Cancel buttons
                QObject::connect(okButton, &QPushButton::clicked, &dialog, &QDialog::accept);
                QObject::connect(cancelButton, &QPushButton::clicked, &dialog, &QDialog::reject);
                
                if (dialog.exec() == QDialog::Accepted) {
                    // Create new transform from the edited values
                    gp_Trsf newTrsf;
                    
                    // Set translation
                    newTrsf.SetTranslationPart(gp_XYZ(xTrans->value(), yTrans->value(), zTrans->value()));
                    
                    // Set rotation (convert Euler angles to rotation matrix)
                    double rxRad = xRot->value() * M_PI / 180.0;
                    double ryRad = yRot->value() * M_PI / 180.0;
                    double rzRad = zRot->value() * M_PI / 180.0;
                    
                    // Create rotation transformations for each axis
                    gp_Trsf rotX, rotY, rotZ;
                    rotX.SetRotation(gp_Ax1(gp_Pnt(0,0,0), gp_Dir(1,0,0)), rxRad);
                    rotY.SetRotation(gp_Ax1(gp_Pnt(0,0,0), gp_Dir(0,1,0)), ryRad);
                    rotZ.SetRotation(gp_Ax1(gp_Pnt(0,0,0), gp_Dir(0,0,1)), rzRad);
                    
                    // Combined rotation
                    gp_Trsf combinedRot = rotZ * rotY * rotX;
                    
                    // Create translation transformation
                    gp_Trsf transTrsf;
                    transTrsf.SetTranslation(gp_Vec(xTrans->value(), yTrans->value(), zTrans->value()));
                    
                    // Combine transformations: translation * rotation
                    newTrsf = transTrsf * combinedRot;
                    
                    // Apply the new transform
                    node->loc = TopLoc_Location(newTrsf);
                    customModel->dataChanged(idx, idx);
                    if (openGLViewer) {
                        openGLViewer->markCacheDirty();
                        openGLViewer->update();
                    }
                }
            });
            
            // --- VHACD and CoACD generation actions ---
            QAction* vhacdAction = menu.addAction("Generate Collision Mesh (VHACD)");
            QObject::connect(vhacdAction, &QAction::triggered, treeView, [=]() {
                // Default parameters as JSON
                QJsonObject defaultParams{
                    {"maxConvexHulls", 256},
                    {"resolution", 500000},
                    {"minimumVolumePercentErrorAllowed", 0.000001},
                    {"maxRecursionDepth", 15},
                    {"shrinkWrap", true},
                    {"fillMode", 0}, // 0 = FLOOD_FILL
                    {"maxNumVerticesPerCH", 256},
                    {"asyncACD", true},
                    {"minEdgeLength", 0.0005},
                    {"findBestPlane", false}
                };
                QJsonDocument doc(defaultParams);
                RailJsonEditorDialog dlg(QString::fromUtf8(doc.toJson(QJsonDocument::Indented)), treeView);
                if (dlg.exec() == QDialog::Accepted) {
                    QJsonDocument userDoc = QJsonDocument::fromJson(dlg.getJsonString().toUtf8());
                    if (!userDoc.isObject()) {
                        QMessageBox::warning(nullptr, "VHACD", "Invalid JSON for VHACD parameters.");
                        return;
                    }
                    QJsonObject obj = userDoc.object();
                    generateVHACDStub(QString::fromStdString(node->name), obj, node);
                    if (openGLViewer) {
                        openGLViewer->markCacheDirty();
                    }
                    QMessageBox::information(nullptr, "VHACD", "VHACD collision mesh generation complete.");
                }
            });
            
            QAction* coacdAction = menu.addAction("Generate Collision Mesh (CoACD)");
            QObject::connect(coacdAction, &QAction::triggered, treeView, [=]() {
                // Default parameters as JSON
                QJsonObject defaultParams{
                    {"concavity", 0.0025},
                    {"alpha", 0.05},
                    {"beta", 0.05},
                    {"maxConvexHull", 16},
                    {"preprocess", "voxel"},
                    {"prepRes", 64},
                    {"sampleRes", 10000},
                    {"mctsNodes", 100},
                    {"mctsIter", 100},
                    {"mctsDepth", 10},
                    {"pca", true},
                    {"merge", true},
                    {"decimate", true},
                    {"maxChVertex", 64},
                    {"extrude", false},
                    {"extrudeMargin", 0.0},
                    {"apxMode", "fast"},
                    {"seed", 42}
                };
                QJsonDocument doc(defaultParams);
                RailJsonEditorDialog dlg(QString::fromUtf8(doc.toJson(QJsonDocument::Indented)), treeView);
                if (dlg.exec() == QDialog::Accepted) {
                    QJsonDocument userDoc = QJsonDocument::fromJson(dlg.getJsonString().toUtf8());
                    if (!userDoc.isObject()) {
                        QMessageBox::warning(nullptr, "CoACD", "Invalid JSON for CoACD parameters.");
                        return;
                    }
                    QJsonObject obj = userDoc.object();
                    double concavity = obj.value("concavity").toDouble(0.0025);
                    double alpha = obj.value("alpha").toDouble(0.05);
                    double beta = obj.value("beta").toDouble(0.05);
                    int maxConvexHull = obj.value("maxConvexHull").toInt(16);
                    std::string preprocess = obj.value("preprocess").toString("voxel").toStdString();
                    int prepRes = obj.value("prepRes").toInt(64);
                    int sampleRes = obj.value("sampleRes").toInt(10000);
                    int mctsNodes = obj.value("mctsNodes").toInt(100);
                    int mctsIter = obj.value("mctsIter").toInt(100);
                    int mctsDepth = obj.value("mctsDepth").toInt(10);
                    bool pca = obj.value("pca").toBool(true);
                    bool merge = obj.value("merge").toBool(true);
                    bool decimate = obj.value("decimate").toBool(true);
                    int maxChVertex = obj.value("maxChVertex").toInt(64);
                    bool extrude = obj.value("extrude").toBool(false);
                    double extrudeMargin = obj.value("extrudeMargin").toDouble(0.0);
                    std::string apxMode = obj.value("apxMode").toString("fast").toStdString();
                    int seed = obj.value("seed").toInt(42);
                    generateCoACDStub(QString::fromStdString(node->name), concavity, alpha, beta, node,
                        maxConvexHull, preprocess, prepRes, sampleRes, mctsNodes, mctsIter, mctsDepth,
                        pca, merge, decimate, maxChVertex, extrude, extrudeMargin, apxMode, seed);
                    if (openGLViewer) {
                        openGLViewer->markCacheDirty();
                    }
                    QMessageBox::information(nullptr, "CoACD", "CoACD collision mesh generation complete.");
                }
            });
            
            // --- Exclude by color submenu ---
            QMenu* excludeByColorMenu = menu.addMenu("Exclude By Color");
            std::set<QString> uniqueColorKeys;
            std::map<QString, QColor> colorKeyToColor;
            std::function<void(const CadNode*)> collectColors;
            collectColors = [&](const CadNode* n) {
                if (!n) return;
                QColor c = n->color.toQColor();
                QString key = QString("%1,%2,%3,%4")
                    .arg(int(c.redF() * 255))
                    .arg(int(c.greenF() * 255))
                    .arg(int(c.blueF() * 255))
                    .arg(int(c.alphaF() * 255));
                uniqueColorKeys.insert(key);
                colorKeyToColor[key] = c;
                for (const auto& child : n->children) collectColors(child.get());
            };
            collectColors(node);
            
            for (const QString& key : uniqueColorKeys) {
                QColor color = colorKeyToColor[key];
                QString colorText = QString("RGB(%1, %2, %3)")
                    .arg(int(color.redF() * 255))
                    .arg(int(color.greenF() * 255))
                    .arg(int(color.blueF() * 255));
                QAction* colorAction = new QAction(colorText, excludeByColorMenu);
                QPixmap pix(16, 16);
                pix.fill(color);
                colorAction->setIcon(QIcon(pix));
                excludeByColorMenu->addAction(colorAction);
                QObject::connect(colorAction, &QAction::triggered, treeView, [=]() {
                    std::vector<QModelIndex> changedIndices;
                    std::function<void(CadNode*)> excludeByColor;
                    excludeByColor = [&](CadNode* n) {
                        if (!n) return;
                        if (n->color.toQColor() == color && !n->excludedFromDecomposition) {
                            n->excludedFromDecomposition = true;
                            QModelIndex changedIdx = customModel->indexForNode(n);
                            if (changedIdx.isValid()) changedIndices.push_back(changedIdx);
                        }
                        for (auto& child : n->children) excludeByColor(child.get());
                    };
                    excludeByColor(const_cast<CadNode*>(node));
                    for (const QModelIndex& changedIdx : changedIndices) {
                        customModel->dataChanged(changedIdx, changedIdx);
                    }
                });
            }
            
            // --- Debug actions ---
            QAction* infoAction = menu.addAction(QString("Node type: %1").arg((int)node->type));
            infoAction->setEnabled(false);
            
            QAction* printPtrAction = menu.addAction("Print Node Pointer (Debug)");
            QObject::connect(printPtrAction, &QAction::triggered, treeView, [=]() {
                qDebug() << "[Debug] Node pointer:" << static_cast<const void*>(node);
                if (auto xData = node->asXCAF()) {
                    QString typeStr = shapeTypeToString(xData->type);
                    bool isNull = xData->shape.IsNull();
                    qDebug() << "[Debug]   XCAF info:";
                    qDebug() << "[Debug]     shapeIndex:" << xData->shapeIndex;
                    qDebug() << "[Debug]     shape type:" << typeStr;
                    qDebug() << "[Debug]     shape isNull:" << isNull;
                }
                if (auto physData = node->asPhysics()) {
                    qDebug() << "[Debug]   PHYSICS info:";
                    qDebug() << "[Debug]     convexHullGenerated:" << physData->convexHullGenerated;
                    qDebug() << "[Debug]     collisionMeshVisible:" << physData->collisionMeshVisible;
                    qDebug() << "[Debug]     hulls.size():" << physData->hulls.size();
                }
            });
            
            // --- Show/Hide node action ---
            QAction* showHideAction = nullptr;
            if (node->visible) {
                showHideAction = menu.addAction("Hide Node");
            } else {
                showHideAction = menu.addAction("Show Node");
            }
            QObject::connect(showHideAction, &QAction::triggered, treeView, [=]() {
                bool newVisibility = !node->visible;
                std::vector<QModelIndex> changedIndices;
                std::function<void(CadNode*)> setVisibility;
                setVisibility = [&](CadNode* n) {
                    if (!n) return;
                    n->visible = newVisibility;
                    QModelIndex changedIdx = customModel->indexForNode(n);
                    if (changedIdx.isValid()) changedIndices.push_back(changedIdx);
                    for (auto& child : n->children) setVisibility(child.get());
                };
                setVisibility(const_cast<CadNode*>(customModel->getNode(idx)));
                for (const QModelIndex& changedIdx : changedIndices) {
                    customModel->dataChanged(changedIdx, changedIdx);
                }
            });
            
            // --- Exclude/Include node action ---
            QAction* excludeIncludeAction = nullptr;
            if (!node->excludedFromDecomposition) {
                excludeIncludeAction = menu.addAction("Exclude Node (and Children)");
            } else {
                excludeIncludeAction = menu.addAction("Include Node (and Children)");
            }
            QObject::connect(excludeIncludeAction, &QAction::triggered, treeView, [=]() {
                bool newExcluded = !node->excludedFromDecomposition;
                std::vector<QModelIndex> changedIndices;
                std::function<void(CadNode*)> setExcluded;
                setExcluded = [&](CadNode* n) {
                    if (!n) return;
                    n->excludedFromDecomposition = newExcluded;
                    QModelIndex changedIdx = customModel->indexForNode(n);
                    if (changedIdx.isValid()) changedIndices.push_back(changedIdx);
                    for (auto& child : n->children) setExcluded(child.get());
                };
                setExcluded(const_cast<CadNode*>(customModel->getNode(idx)));
                for (const QModelIndex& changedIdx : changedIndices) {
                    customModel->dataChanged(changedIdx, changedIdx);
                }
            });
            
            // --- Delete Node action (not for root) ---
            if (node != customModel->getRootNodePointer()) {
                QAction* deleteAction = menu.addAction("Delete Node");
                QObject::connect(deleteAction, &QAction::triggered, treeView, [=]() {
                    customModel->removeNode(node);
                    QModelIndex parentIdx = customModel->indexForNode(customModel->getParentNode(node));
                    if (parentIdx.isValid()) {
                        treeView->setCurrentIndex(parentIdx);
                    } else {
                        treeView->clearSelection();
                    }
                });
            }
            
            // --- Toggle collision mesh visibility for Physics nodes ---
            if (node->type == CadNodeType::Physics && node->asPhysics()) {
                QAction* toggleCollisionAction = menu.addAction("Toggle Collision Mesh Visibility");
                QObject::connect(toggleCollisionAction, &QAction::triggered, treeView, [=]() {
                    PhysicsNodeData* physData = node->asPhysics();
                    physData->collisionMeshVisible = !physData->collisionMeshVisible;
                    customModel->dataChanged(idx, idx);
                    if (openGLViewer) {
                        openGLViewer->markCacheDirty();
                        openGLViewer->update();
                    }
                });
            }
            
            // --- Expand Rail to Other Trees ---
            if (node->type == CadNodeType::Rail && node->asRail()) {
                QAction* expandRailAction = menu.addAction("Expand Rail to Other Tree...");
                QObject::connect(expandRailAction, &QAction::triggered, treeView, [=]() {
                    // Create a dialog to select target tree
                    QDialog dialog(treeView);
                    dialog.setWindowTitle("Select Target Tree for Rail Expansion");
                    dialog.setModal(true);
                    dialog.resize(400, 200);
                    
                    QVBoxLayout* mainLayout = new QVBoxLayout(&dialog);
                    
                    QLabel* label = new QLabel("Select the target tree to expand this rail into:");
                    mainLayout->addWidget(label);
                    
                    QComboBox* treeComboBox = new QComboBox;
                    mainLayout->addWidget(treeComboBox);
                    
                    // Populate combo box with available trees
                    for (size_t i = 0; i < g_treeViews.size(); ++i) {
                        QTreeView* availableTreeView = g_treeViews[i];
                        if (availableTreeView && availableTreeView != treeView) { // Don't include current tree
                            QString treeName = "Tree " + QString::number(i + 1);
                            // Try to get a more descriptive name from the tab widget
                            if (availableTreeView->parent()) {
                                QTabWidget* tabWidget = qobject_cast<QTabWidget*>(availableTreeView->parent()->parent());
                                if (tabWidget) {
                                    QWidget* parentWidget = qobject_cast<QWidget*>(availableTreeView->parent());
                                    if (parentWidget) {
                                        int tabIndex = tabWidget->indexOf(parentWidget);
                                        if (tabIndex >= 0) {
                                            treeName = tabWidget->tabText(tabIndex);
                                        }
                                    }
                                }
                            }
                            treeComboBox->addItem(treeName, QVariant::fromValue(static_cast<void*>(availableTreeView)));
                        }
                    }
                    
                    if (treeComboBox->count() == 0) {
                        QMessageBox::warning(nullptr, "No Target Trees", "No other trees available for rail expansion.");
                        return;
                    }
                    
                    // Buttons
                    QHBoxLayout* buttonLayout = new QHBoxLayout;
                    QPushButton* okButton = new QPushButton("Expand");
                    QPushButton* cancelButton = new QPushButton("Cancel");
                    
                    buttonLayout->addStretch();
                    buttonLayout->addWidget(cancelButton);
                    buttonLayout->addWidget(okButton);
                    mainLayout->addLayout(buttonLayout);
                    
                    // Connect buttons
                    QObject::connect(okButton, &QPushButton::clicked, &dialog, &QDialog::accept);
                    QObject::connect(cancelButton, &QPushButton::clicked, &dialog, &QDialog::reject);
                    
                    if (dialog.exec() == QDialog::Accepted) {
                        QTreeView* targetTreeView = static_cast<QTreeView*>(treeComboBox->currentData().value<void*>());
                        if (targetTreeView) {
                            // Find the corresponding OpenGL viewer
                            CadOpenGLWidget* targetOpenGLViewer = nullptr;
                            for (size_t i = 0; i < g_treeViews.size(); ++i) {
                                if (g_treeViews[i] == targetTreeView && i < g_openGLViews.size()) {
                                    targetOpenGLViewer = g_openGLViews[i];
                                    break;
                                }
                            }
                            
                            // Get the target tree's root node
                            CustomModelTreeModel* targetModel = qobject_cast<CustomModelTreeModel*>(targetTreeView->model());
                            if (targetModel) {
                                std::shared_ptr<const CadNode> targetRootShared = targetModel->getRoot();
                                if (targetRootShared) {
                                    // Create a non-const shared_ptr for the expandRailInPhysicsPreview function
                                    std::shared_ptr<CadNode> targetRootNonConst = std::const_pointer_cast<CadNode>(targetRootShared);
                                    // Expand the rail to the target tree
                                    expandRailInPhysicsPreview(node, targetRootNonConst, targetOpenGLViewer);
                                    
                                    // Update the target tree view
                                    targetModel->dataChanged(targetModel->index(0, 0), targetModel->index(0, 0));
                                    
                                    QMessageBox::information(nullptr, "Success", 
                                        QString("Rail '%1' expanded to target tree successfully.").arg(QString::fromStdString(node->name)));
                                }
                            }
                        }
                    }
                });
            }
            
            // --- Create Connection ---
            QAction* createConnectionAction = menu.addAction("Create Connection...");
            
            // Add test action for drag chain creation
            QAction* testDragChainAction = menu.addAction("Test Drag Chain Creation");
            QObject::connect(testDragChainAction, &QAction::triggered, treeView, [=]() {
                qDebug() << "[ContextMenu] Test drag chain creation triggered from root";
                if (simManager) {
                    simManager->forceDragChainCreation();
                }
            });
            
            QObject::connect(createConnectionAction, &QAction::triggered, treeView, [=]() {
                qDebug() << "[ContextMenu] Create Connection action triggered";
                qDebug() << "[ContextMenu] openGLViewer:" << (openGLViewer ? "valid" : "null");
                qDebug() << "[ContextMenu] openGLViewer address:" << static_cast<const void*>(openGLViewer);
                
                // Create connection widget in a new window
                QWidget* connectionWindow = new QWidget();
                connectionWindow->setWindowTitle("Create Connection");
                connectionWindow->resize(600, 500);
                
                auto layout = new QVBoxLayout(connectionWindow);
                auto connectionWidget = new ConnectionCreationWidget(customModel, connectionWindow);
                layout->addWidget(connectionWidget);
                
                qDebug() << "[ContextMenu] Connection widget created";
                
                // Set the OpenGL widget for visualization
                if (openGLViewer) {
                    qDebug() << "[ContextMenu] Setting OpenGL widget for connection widget";
                    connectionWidget->setOpenGLWidget(openGLViewer);
                } else {
                    qDebug() << "[ContextMenu] No OpenGL viewer available - this is the problem!";
                }
                
                // Connect widget signals
                QObject::connect(connectionWidget, &ConnectionCreationWidget::connectionCreated, 
                    [=](std::shared_ptr<CadNode> connectionNode) {
                        if (connectionNode) {
                            // Get the selected connection points
                            auto point1 = connectionWidget->getPoint1();
                            auto point2 = connectionWidget->getPoint2();
                            
                            if (point1 && point2) {
                                // Find the best placement location for the connection
                                CadNode* placementNode = findBestConnectionPlacement(point1.get(), point2.get());
                                
                                // If no placement found, use the root node
                                if (!placementNode) {
                                    placementNode = const_cast<CadNode*>(customModel->getRootNodePointer());
                                }
                                
                                if (placementNode) {
                                    // Add the connection to the placement node
                                    placementNode->children.push_back(connectionNode);
                                    setParentPointersRecursive(placementNode);
                                    
                                    // Update the model - we need to update the placement node's index
                                    QModelIndex placementIndex = customModel->indexForNode(placementNode);
                                    if (placementIndex.isValid()) {
                                        customModel->dataChanged(placementIndex, placementIndex);
                                    } else {
                                        // Fallback to updating the root if we can't find the placement index
                                        QModelIndex rootIndex = customModel->index(0, 0);
                                        customModel->dataChanged(rootIndex, rootIndex);
                                    }
                                    
                                    // Update OpenGL viewer if available
                                    if (openGLViewer) {
                                        openGLViewer->markCacheDirty();
                                        openGLViewer->update();
                                    }
                                    
                                    QString placementName = QString::fromStdString(placementNode->name);
                                    QMessageBox::information(treeView, "Success", 
                                        QString("Connection '%1' created successfully and added to '%2'.").arg(QString::fromStdString(connectionNode->name), placementName));
                                }
                            }
                        }
                        
                        // Close the window
                        connectionWindow->close();
                    });
                
                QObject::connect(connectionWidget, &ConnectionCreationWidget::connectionCancelled, 
                    [=]() {
                        connectionWindow->close();
                    });
                
                // Show the window
                connectionWindow->show();
            });
            
            // --- Edit Connection (only for connection nodes) ---
            if (node->type == CadNodeType::Connection) {
                QAction* editConnectionAction = menu.addAction("Edit Connection...");
                
                // Add test action for drag chain creation
                QAction* testDragChainAction = menu.addAction("Test Drag Chain Creation");
                QObject::connect(testDragChainAction, &QAction::triggered, treeView, [=]() {
                    qDebug() << "[ContextMenu] Test drag chain creation triggered";
                    if (simManager) {
                        simManager->forceDragChainCreation();
                    }
                });
                QObject::connect(editConnectionAction, &QAction::triggered, treeView, [=]() {
                    qDebug() << "[ContextMenu] Edit Connection action triggered for node:" << QString::fromStdString(node->name);
                    
                    // Create connection widget in a new window
                    QWidget* connectionWindow = new QWidget();
                    connectionWindow->setWindowTitle("Edit Connection - " + QString::fromStdString(node->name));
                    connectionWindow->resize(600, 500);
                    
                    auto layout = new QVBoxLayout(connectionWindow);
                    auto connectionWidget = new ConnectionCreationWidget(customModel, connectionWindow);
                    layout->addWidget(connectionWidget);
                    
                    // Set the OpenGL widget for visualization
                    if (openGLViewer) {
                        connectionWidget->setOpenGLWidget(openGLViewer);
                    }
                    
                    // Load settings from the connection node
                    std::shared_ptr<CadNode> connectionNodeShared = std::make_shared<CadNode>(*node);
                    bool settingsLoaded = connectionWidget->loadSettingsFromConnectionNode(connectionNodeShared);
                    
                    if (settingsLoaded) {
                        qDebug() << "[ContextMenu] Successfully loaded settings from connection node";
                    } else {
                        qDebug() << "[ContextMenu] Failed to load settings from connection node, using defaults";
                    }
                    
                    // Connect widget signals
                    QObject::connect(connectionWidget, &ConnectionCreationWidget::connectionCreated, 
                        [=](std::shared_ptr<CadNode> newConnectionNode) {
                            if (newConnectionNode) {
                                // Replace the old connection node with the new one
                                // Find the parent of the old node
                                CadNode* parent = node->parent;
                                if (parent) {
                                    // Find and replace the old node
                                    for (auto& child : parent->children) {
                                        if (child.get() == node) {
                                            child = newConnectionNode;
                                            newConnectionNode->parent = parent;
                                            break;
                                        }
                                    }
                                    
                                    // Update the model
                                    QModelIndex parentIndex = customModel->indexForNode(parent);
                                    if (parentIndex.isValid()) {
                                        customModel->dataChanged(parentIndex, parentIndex);
                                    }
                                    
                                    // Update OpenGL viewer if available
                                    if (openGLViewer) {
                                        openGLViewer->markCacheDirty();
                                        openGLViewer->update();
                                    }
                                    
                                    QMessageBox::information(treeView, "Success", 
                                        QString("Connection '%1' updated successfully.").arg(QString::fromStdString(newConnectionNode->name)));
                                }
                            }
                            
                            // Close the window
                            connectionWindow->close();
                        });
                    
                    QObject::connect(connectionWidget, &ConnectionCreationWidget::connectionCancelled, 
                        [=]() {
                            connectionWindow->close();
                        });
                    
                    // Show the window
                    connectionWindow->show();
                });
            }
        }
        
        // Handle XCAFLabelNode type (XCAFLabelTreeModel)
        XCAFLabelTreeModel* labelModel = qobject_cast<XCAFLabelTreeModel*>(model);
        if (labelModel) {
            XCAFLabelNode* labelNode = const_cast<XCAFLabelNode*>(labelModel->getNode(idx));
            if (!labelNode) return;
            
            // Add basic info action for XCAF nodes
            QAction* infoAction = menu.addAction(QString("XCAF Label Tag: %1").arg(labelNode->label.Tag()));
            infoAction->setEnabled(false);
            
            // Add debug action
            QAction* debugAction = menu.addAction("Debug XCAF Node");
            QObject::connect(debugAction, &QAction::triggered, treeView, [=]() {
                qDebug() << "[Debug] XCAF Node:";
                qDebug() << "[Debug]   Label Tag:" << labelNode->label.Tag();
                qDebug() << "[Debug]   Label Address:" << static_cast<const void*>(&labelNode->label);
                qDebug() << "[Debug]   Children count:" << labelNode->children.size();
            });
        }
        
        if (!menu.isEmpty()) {
            menu.exec(treeView->viewport()->mapToGlobal(pos));
        }
    });
}

void initTreeAndOpenGLWidget(std::shared_ptr<CadNode>& inputRoot,
                             QTabWidget* treeTabWidget,
                             QTabWidget* openGLTabWidget,
                             const QString& name,
                             const Handle(TDocStd_Document)& doc,
                             SimulationManager* simManager) {
    inputRoot->name = "Tree Root " + name.toStdString();
    CustomModelTreeModel* qtModel = new CustomModelTreeModel(inputRoot);
    QTreeView* treeView = new QTreeView;
    treeView->setModel(qtModel);
    treeView->setHeaderHidden(false);
    treeView->setSelectionMode(QAbstractItemView::ExtendedSelection);
    
    CadOpenGLWidget* openGLViewer = new CadOpenGLWidget(inputRoot.get());
    if (simManager) {
        openGLViewer->setSimulationManager(simManager);
    }
    
    // Setup comprehensive context menu using the new shared function
    setupComprehensiveContextMenu(treeView, qtModel, doc, openGLViewer, simManager);
    
    // Add tabs
    treeTabWidget->addTab(treeView, name + " Tree");
    openGLTabWidget->addTab(openGLViewer, name + " Preview");
    
    // Connect tree and viewer
    connectTreeAndViewer(treeView, openGLViewer, qtModel);
    
    // Store pointers for tab switching
    g_treeViews.push_back(treeView);
    g_openGLViews.push_back(openGLViewer);
}

// Improved tree building function for STEP 214 compatibility
std::shared_ptr<CadNode> build_tree_xcaf(const TDF_Label& label,
                   const Handle(XCAFDoc_ShapeTool)& shapeTool,
                   const Handle(XCAFDoc_ColorTool)& colorTool,
                   const CADNodeColor& parentColor,
                   const TopLoc_Location& parentLoc,
                   const Handle(TDocStd_Document)& doc)
{
    TopoDS_Shape shape = shapeTool->GetShape(label);
    bool isReference = shapeTool->IsReference(label);
    if (isReference && !shape.IsNull()) {
        // 1. Create a new CadNode for the reference node (the parent)
        auto refNode = std::make_shared<CadNode>();
        refNode->type = CadNodeType::XCAF;
        refNode->loc = getEffectiveTransform(label, shapeTool, parentLoc); // The transform for this instance
        refNode->color = get_shape_color(shape, label, shapeTool, colorTool, parentColor);
        // Compose a name for the reference node, including the transform
        QString name;
        Handle(TDataStd_Name) nameAttr;
        if (label.FindAttribute(TDataStd_Name::GetID(), nameAttr) && !nameAttr.IsNull()) {
            name = QString::fromStdWString(nameAttr->Get().ToWideString());
        } else {
            name = QString("Label %1").arg(label.Tag());
        }
        QString transformStr = makeTransformString(refNode->loc);
        refNode->name = QString("Reference Node | Label %1 | Transform: %2 | OCC Name: %3")
            .arg(label.Tag())
            .arg(transformStr)
            .arg(name)
            .toStdString();
        // 2. Get the unique child (the actual geometry/assembly)
        TDF_Label refLabel;
        if (shapeTool->GetReferredShape(label, refLabel)) {
            auto uniqueChild = build_tree_xcaf(refLabel, shapeTool, colorTool, refNode->color, parentLoc, doc);
            if (uniqueChild) {
                refNode->children.push_back(uniqueChild);
            }
        }
        // Always set labelPath for reference node
        auto xData = std::make_shared<XCAFNodeData>();
        xData->shape = shape;
        xData->type = !shape.IsNull() ? shape.ShapeType() : TopAbs_SHAPE;
        xData->labelPath = getLabelPath(label, doc);
        xData->originalXCAFShape = shape;
        refNode->data = xData;
        // Fix debug output for labelPath
        QVariantList labelPathDebug;
        for (int tag : xData->labelPath) labelPathDebug.push_back(QVariant(tag));
        qDebug() << "[build_tree_xcaf] Created reference node labelPath:" << labelPathDebug;
        return refNode;
    }
    CADNodeColor color = get_shape_color(shape, label, shapeTool, colorTool, parentColor);
    if (!shape.IsNull() && (shape.ShapeType() == TopAbs_SOLID || shape.ShapeType() == TopAbs_COMPOUND)) {
        qDebug() << "Label" << label.Tag() << "color: RGB(" << color.r << "," << color.g << "," << color.b << ")";
        if (color.r == parentColor.r && color.g == parentColor.g && color.b == parentColor.b) {
            qDebug() << "  -> Inherited from parent";
        } else {
            qDebug() << "  -> Own color";
        }
    }
    if (!shape.IsNull() && shape.ShapeType() == TopAbs_EDGE) {
        return nullptr;
    }
    QString name;
    Handle(TDataStd_Name) nameAttr;
    if (label.FindAttribute(TDataStd_Name::GetID(), nameAttr) && !nameAttr.IsNull()) {
        name = QString::fromStdWString(nameAttr->Get().ToWideString());
    } else {
        name = QString("Label %1").arg(label.Tag());
    }
    TopLoc_Location nodeLoc = getEffectiveTransform(label, shapeTool, parentLoc);
    auto node = std::make_shared<CadNode>();
    node->color = color;
    node->loc = nodeLoc;
    node->type = CadNodeType::XCAF;
    auto xData = std::make_shared<XCAFNodeData>();
    xData->shape = shape;
    xData->type = !shape.IsNull() ? shape.ShapeType() : TopAbs_SHAPE;
    xData->labelPath = getLabelPath(label, doc); // Always set labelPath for every XCAF node
    xData->originalXCAFShape = shape;
    node->data = xData;
    // Fix debug output for labelPath
    QVariantList labelPathDebug;
    for (int tag : xData->labelPath) labelPathDebug.push_back(QVariant(tag));
    qDebug() << "[build_tree_xcaf] Created node labelPath:" << labelPathDebug;
    QString typeName = shapeTypeToString(xData->type);
    // Enhanced assembly detection and handling
    bool isCompoundAssembly = false;
    if (!shape.IsNull() && shape.ShapeType() == TopAbs_COMPOUND) {
        // Check if this compound has no direct geometry but contains references (assembly pattern)
        bool hasDirectGeometry = false;
        int refChildCount = 0;
        int totalChildCount = 0;
        for (TDF_ChildIterator it(label); it.More(); it.Next()) {
            totalChildCount++;
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
        // If this is an assembly with references but no direct geometry
        if (!hasDirectGeometry && refChildCount > 0) {
            isCompoundAssembly = true;
            qDebug() << "Found assembly compound at label" << label.Tag() << "with" << refChildCount << "references";
        }
    }
    // Enhanced name for STEP 214 debugging
    QString assemblyInfo = "";
    if (isAssembly(label, shapeTool)) {
        assemblyInfo = " | ASSEMBLY";
    }
    if (isCompoundAssembly) {
        assemblyInfo = " | COMPOUND_ASSEMBLY";
    }
    if (shapeTool->IsReference(label)) {
        assemblyInfo += " | REFERENCE";
    }
    // Compose a detailed name string
    node->name = QString("Label %1 | Type: %2 | Transform: %3%4%5")
        .arg(label.Tag())
        .arg(typeName)
        .arg(makeTransformString(nodeLoc))
        .arg(name.isEmpty() ? "" : QString(" | OCC Name: %1").arg(name))
        .arg(assemblyInfo)
        .toStdString();
    // Enhanced reference handling for STEP 214 assemblies
    if (isReference) {
        TDF_Label refLabel;
        if (shapeTool->GetReferredShape(label, refLabel)) {
            qDebug() << "Following reference from label" << label.Tag() << "to" << refLabel.Tag();
            TopLoc_Location refLoc = nodeLoc; // Use the effective transform
            auto child = build_tree_xcaf(refLabel, shapeTool, colorTool, color, parentLoc, doc);
            if (child) {
                // Mark this as a reference node in the name
                child->name = QString("REF->%1 | %2").arg(refLabel.Tag()).arg(QString::fromStdString(child->name)).toStdString();
                node->children.push_back(child);
            }
        }
    }
    // Recurse into children (for all labels)
    for (TDF_ChildIterator it(label); it.More(); it.Next()) {
        auto child = build_tree_xcaf(it.Value(), shapeTool, colorTool, color, nodeLoc, doc);
        if (child) node->children.push_back(child);
    }
    // After node is created and type is set
    if (!shape.IsNull()) {
        xData->shape = shape;
        xData->type = shape.ShapeType();
    }
    // Add face/edge children for shapes that aren't assemblies (SOLID, COMPOUND, etc.)
    if (!shape.IsNull() && !isAssembly(label, shapeTool)) {
        int faceIdx = 0;
        int faceCount = 0;
        for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next(), ++faceIdx) {
            TopoDS_Face face = TopoDS::Face(exp.Current());
            auto faceNode = std::make_shared<CadNode>();
            faceNode->type = CadNodeType::XCAF;
            auto faceData = std::make_shared<XCAFNodeData>();
            faceData->shape = face;
            faceData->type = TopAbs_FACE;
            faceData->labelPath = getLabelPath(label, doc);
            faceData->shapeIndex = faceIdx;
            faceNode->data = faceData;
            faceNode->color = get_shape_color(face, label, shapeTool, colorTool, color);
            faceNode->loc = nodeLoc;
            faceNode->name = QString("Face %1 of %2").arg(faceIdx).arg(QString::fromStdString(node->name)).toStdString();
            node->children.push_back(faceNode);
            faceCount++;
        }
        int edgeIdx = 0;
        int edgeCount = 0;
        for (TopExp_Explorer exp(shape, TopAbs_EDGE); exp.More(); exp.Next(), ++edgeIdx) {
            TopoDS_Shape edge = exp.Current();
            auto edgeNode = std::make_shared<CadNode>();
            edgeNode->type = CadNodeType::XCAF;
            auto edgeData = std::make_shared<XCAFNodeData>();
            edgeData->shape = edge;
            edgeData->type = TopAbs_EDGE;
            edgeData->labelPath = getLabelPath(label, doc);
            edgeData->shapeIndex = edgeIdx;
            edgeNode->data = edgeData;
            edgeNode->color = color;
            edgeNode->loc = nodeLoc;
            edgeNode->name = QString("Edge %1 of %2").arg(edgeIdx).arg(QString::fromStdString(node->name)).toStdString();
            node->children.push_back(edgeNode);
            edgeCount++;
        }
        qDebug() << "Added" << faceCount << "faces and" << edgeCount << "edges to shape at label" << label.Tag();
    }
    return node;
}

bool loadFromJsonAndBin(const QString& railJsonFile,
                        Handle(TDocStd_Document)& doc,
                        std::shared_ptr<CadNode>& cadRoot,
                        std::unique_ptr<XCAFLabelNode>& labelRoot,
                        Handle(XCAFDoc_ShapeTool)& shapeTool,
                        Handle(XCAFDoc_ColorTool)& colorTool,
                        std::shared_ptr<CadNode>& customModelRootContainer,
                        std::shared_ptr<CadNode>& customModelRoot,
                        bool& loadedFromJsonBin)
{
    // Initialize XCAF application
    Handle(XCAFApp_Application) app = XCAFApp_Application::GetApplication();
    BinDrivers::DefineFormat(app);
    XmlDrivers::DefineFormat(app);
    
    // Create new document
    app->NewDocument("BinXCAF", doc);
    
    // Get shape and color tools
    shapeTool = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
    colorTool = XCAFDoc_DocumentTool::ColorTool(doc->Main());
    
    // Set up default color
    CADNodeColor defaultColor(0.8f, 0.8f, 0.8f, 1.0f);
    TopLoc_Location identityLoc;
    
    // Create CAD root node
    cadRoot = std::make_shared<CadNode>();
    cadRoot->name = "CAD Root";
    cadRoot->type = CadNodeType::MutexRoot;
    cadRoot->data = std::make_shared<MutexRootNodeData>();
    cadRoot->visible = true;
    
    // Build XCAF label tree
    labelRoot = std::make_unique<XCAFLabelNode>(doc->Main());
    labelRoot->label = doc->Main();
    for (TDF_ChildIterator it(doc->Main()); it.More(); it.Next()) {
        TDF_Label childLabel = it.Value();
        labelRoot->children.push_back(buildLabelTreeWithReferences(childLabel, shapeTool));
    }
    TDF_LabelSequence freeShapes;
    shapeTool->GetFreeShapes(freeShapes);
    for (Standard_Integer i = 1; i <= freeShapes.Length(); ++i) {
        TDF_Label freeShapeLabel = freeShapes.Value(i);
        bool alreadyAdded = false;
        for (const auto& existingChild : labelRoot->children) {
            if (existingChild->label == freeShapeLabel) {
                alreadyAdded = true;
                break;
            }
        }
        if (!alreadyAdded) {
            labelRoot->children.push_back(buildLabelTreeWithReferences(freeShapeLabel, shapeTool));
        }
    }
    
    // Add a special node for all shapes in the document
    auto allShapesNode = std::make_unique<XCAFLabelNode>(TDF_Label());
    allShapesNode->label = TDF_Label();
    std::function<void(const TDF_Label&)> collectShapeLabels = [&](const TDF_Label& label) {
        if (!label.IsNull()) {
            TopoDS_Shape shape = shapeTool->GetShape(label);
            if (!shape.IsNull()) {
                auto shapeNode = std::make_unique<XCAFLabelNode>(label);
                allShapesNode->children.push_back(std::move(shapeNode));
            }
            for (TDF_ChildIterator it(label); it.More(); it.Next()) {
                collectShapeLabels(it.Value());
            }
        }
    };
    collectShapeLabels(doc->Main());
    if (!allShapesNode->children.empty()) {
        labelRoot->children.push_back(std::move(allShapesNode));
    }
    
    // Load custom model JSON
    QFile file(railJsonFile);
    if (!file.open(QIODevice::ReadOnly)) {
        QMessageBox::critical(nullptr, "Error", "Failed to open file: " + railJsonFile);
        return false;
    }
    QByteArray data = file.readAll();
    file.close();
    QJsonParseError err;
    QJsonDocument jsonDoc = QJsonDocument::fromJson(data, &err);
    if (jsonDoc.isNull() || !jsonDoc.isObject()) {
        QMessageBox::critical(nullptr, "Error", "Invalid JSON file: " + railJsonFile);
        return false;
    }
    customModelRoot = CadNode::fromJson(jsonDoc.object());
    customModelRootContainer = std::make_shared<CadNode>();
    customModelRootContainer->name = "Custom Model Root Container";
    customModelRootContainer->type = CadNodeType::MutexRoot;
    customModelRootContainer->data = std::make_shared<MutexRootNodeData>();
    customModelRootContainer->visible = true;
    
    // Set ground plane properties for Custom model preview
    MutexRootNodeData* mutexData = customModelRootContainer->asMutexRoot();
    
    customModelRootContainer->children.clear();
    customModelRootContainer->children.push_back(customModelRoot);
    loadedFromJsonBin = true;
    return true;
}

bool loadFromStep(const QString& stepFile,
                  Handle(TDocStd_Document)& doc,
                  std::shared_ptr<CadNode>& cadRoot,
                  std::unique_ptr<XCAFLabelNode>& labelRoot,
                  Handle(XCAFDoc_ShapeTool)& shapeTool,
                  Handle(XCAFDoc_ColorTool)& colorTool)
{
    Handle(XCAFApp_Application) appOCC = XCAFApp_Application::GetApplication();
    appOCC->NewDocument("BinXCAF", doc);
    STEPCAFControl_Reader reader;
    reader.SetColorMode(true);
    reader.SetNameMode(true);
    reader.SetLayerMode(true);
    if (reader.ReadFile(stepFile.toStdString().c_str()) != IFSelect_RetDone) {
        QMessageBox::critical(nullptr, "Error", "Failed to read STEP file.");
        return false;
    }
    if (!reader.Transfer(doc)) {
        QMessageBox::critical(nullptr, "Error", "Failed to transfer STEP file.");
        return false;
    }
    // Get shape and color tools from the document
    shapeTool = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
    colorTool = XCAFDoc_DocumentTool::ColorTool(doc->Main());
    // Build main CAD tree from XCAF
    CADNodeColor defaultColor = CADNodeColor::fromSRGB(200, 200, 200);
    TopLoc_Location identityLoc;
    TDF_LabelSequence roots;
    shapeTool->GetFreeShapes(roots);
    cadRoot = std::make_shared<CadNode>();
    cadRoot->name = "Root";
    cadRoot->color = defaultColor;
    cadRoot->loc = identityLoc;
    for (Standard_Integer i = 1; i <= roots.Length(); ++i) {
        TDF_Label rootLabel = roots.Value(i);
        auto child = build_tree_xcaf(rootLabel, shapeTool, colorTool, defaultColor, identityLoc, doc);
        if (child) {
            cadRoot->children.push_back(std::move(child));
            setGlobalLocRecursive(cadRoot->children.back().get(), cadRoot->globalLoc);
        }
    }
    // Build XCAF label tree
    labelRoot = std::make_unique<XCAFLabelNode>(doc->Main());
    labelRoot->label = doc->Main();
    for (TDF_ChildIterator it(doc->Main()); it.More(); it.Next()) {
        TDF_Label childLabel = it.Value();
        labelRoot->children.push_back(buildLabelTreeWithReferences(childLabel, shapeTool));
    }
    TDF_LabelSequence freeShapes;
    shapeTool->GetFreeShapes(freeShapes);
    for (Standard_Integer i = 1; i <= freeShapes.Length(); ++i) {
        TDF_Label freeShapeLabel = freeShapes.Value(i);
        bool alreadyAdded = false;
        for (const auto& existingChild : labelRoot->children) {
            if (existingChild->label == freeShapeLabel) {
                alreadyAdded = true;
                break;
            }
        }
        if (!alreadyAdded) {
            labelRoot->children.push_back(buildLabelTreeWithReferences(freeShapeLabel, shapeTool));
        }
    }
    // Add a special node for all shapes in the document
    auto allShapesNode = std::make_unique<XCAFLabelNode>(TDF_Label());
    allShapesNode->label = TDF_Label();
    std::function<void(const TDF_Label&)> collectShapeLabels = [&](const TDF_Label& label) {
        if (!label.IsNull()) {
            TopoDS_Shape shape = shapeTool->GetShape(label);
            if (!shape.IsNull()) {
                auto shapeNode = std::make_unique<XCAFLabelNode>(label);
                allShapesNode->children.push_back(std::move(shapeNode));
            }
            for (TDF_ChildIterator it(label); it.More(); it.Next()) {
                collectShapeLabels(it.Value());
            }
        }
    };
    collectShapeLabels(doc->Main());
    if (!allShapesNode->children.empty()) {
        labelRoot->children.push_back(std::move(allShapesNode));
    }
    shapeTool = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
    colorTool = XCAFDoc_DocumentTool::ColorTool(doc->Main());
    return true;
}

// Connection management functions
std::shared_ptr<CadNode> createConnectionBetweenPoints(
    const std::string& connectionName,
    CadNode* point1,
    CadNode* point2,
    ConnectionNodeData::ConnectionType connectionType)
{
    if (!point1 || !point2 || point1->type != CadNodeType::ConnectionPoint || point2->type != CadNodeType::ConnectionPoint) {
        return nullptr;
    }
    
    // Check if points can be connected
    if (!canConnectPoints(point1, point2)) {
        return nullptr;
    }
    
    // Debug: Check connection point positions
    qDebug() << "[Connection] Creating connection between points:"
             << QString::fromStdString(point1->name) << "and" << QString::fromStdString(point2->name);
    
    if (point1->loc.IsIdentity()) {
        qDebug() << "[Connection] WARNING: Point1 has identity location!";
    } else {
        gp_Trsf trsf1 = point1->loc.Transformation();
        gp_Pnt pos1 = trsf1.TranslationPart();
        qDebug() << "[Connection] Point1 position:" << pos1.X() << pos1.Y() << pos1.Z();
    }
    
    if (point2->loc.IsIdentity()) {
        qDebug() << "[Connection] WARNING: Point2 has identity location!";
    } else {
        gp_Trsf trsf2 = point2->loc.Transformation();
        gp_Pnt pos2 = trsf2.TranslationPart();
        qDebug() << "[Connection] Point2 position:" << pos2.X() << pos2.Y() << pos2.Z();
    }
    
    // Create the connection node
    auto connectionNode = std::make_shared<CadNode>();
    connectionNode->name = connectionName;
    connectionNode->type = CadNodeType::Connection;
    connectionNode->color = CADNodeColor::fromSRGB(100, 100, 255); // Blue color for connections
    
    // Set the connection node position to the midpoint between the two connection points
    if (!point1->loc.IsIdentity() && !point2->loc.IsIdentity()) {
        gp_Trsf trsf1 = point1->loc.Transformation();
        gp_Trsf trsf2 = point2->loc.Transformation();
        gp_Pnt pos1 = trsf1.TranslationPart();
        gp_Pnt pos2 = trsf2.TranslationPart();
        
        // Calculate midpoint
        gp_Pnt midpoint((pos1.X() + pos2.X()) / 2.0, 
                       (pos1.Y() + pos2.Y()) / 2.0, 
                       (pos1.Z() + pos2.Z()) / 2.0);
        
        // Set connection node position
        gp_Trsf connectionTrsf;
        connectionTrsf.SetTranslation(gp_Vec(midpoint.X(), midpoint.Y(), midpoint.Z()));
        connectionNode->loc = TopLoc_Location(connectionTrsf);
        
        qDebug() << "[Connection] Point1 position:" << pos1.X() << pos1.Y() << pos1.Z();
        qDebug() << "[Connection] Point2 position:" << pos2.X() << pos2.Y() << pos2.Z();
        qDebug() << "[Connection] Set connection node position to midpoint:"
                 << midpoint.X() << midpoint.Y() << midpoint.Z();
    } else {
        qDebug() << "[Connection] WARNING: Connection points have identity locations, using origin";
    }
    
    // Create connection data
    auto connectionData = std::make_shared<ConnectionNodeData>();
    connectionData->connectionType = connectionType;
    connectionData->length = calculateDistanceBetweenPoints(point1, point2);
    connectionData->isFlexible = (connectionType == ConnectionNodeData::ConnectionType::Cable || 
                                 connectionType == ConnectionNodeData::ConnectionType::Hose ||
                                 connectionType == ConnectionNodeData::ConnectionType::Wire);
    
        // Set default values for drag chain parameters
    connectionData->pitchLength = 300.0; // 300mm pitch length by default for drag chains
    connectionData->segmentCount = 3; // 3 segments by default for drag chains
    connectionData->maxBendRadius = 100.0; // Default bend radius for drag chains
    
    // Calculate the effective length using drag chain calculation
    if (connectionType == ConnectionNodeData::ConnectionType::DragChain) {
        // Use attachment-aware calculation if attachments are locked
        if (connectionData->startAttachment.isLocked || connectionData->endAttachment.isLocked) {
            connectionData->length = calculateDragChainLengthWithAttachments(point1, point2,
                                                                          connectionData->maxBendRadius,
                                                                          connectionData->pitchLength,
                                                                          connectionData->segmentCount,
                                                                          connectionData->startAttachment,
                                                                          connectionData->endAttachment);
        } else {
            // Calculate using pitch length and segments
            double straightDistance = calculateDistanceBetweenPoints(point1, point2);
            connectionData->length = connectionData->segmentCount * connectionData->pitchLength;
            
            // Add bend radius contribution
            if (connectionData->maxBendRadius > 0.0) {
                double typicalSegmentLength = connectionData->pitchLength;
                int estimatedBends = std::max(1, static_cast<int>(straightDistance / typicalSegmentLength));
                double bendLength = estimatedBends * (connectionData->maxBendRadius * M_PI / 2.0);
                double transitionLength = estimatedBends * 20.0;
                connectionData->length += bendLength + transitionLength;
            }
        }
    } else {
        // For non-drag chain connections, use standard distance calculation
        connectionData->length = calculateDistanceBetweenPoints(point1, point2);
    }
    
    connectionNode->data = connectionData;
    
    // If connection points have identity locations, try to get position from JSON data
    if (point1->loc.IsIdentity() || point2->loc.IsIdentity()) {
        qDebug() << "[Connection] Connection points have identity locations, trying JSON data...";
        
        // Try to get position from the connection data's JSON settings
        if (connectionData && !connectionData->creationSettingsJson.isEmpty()) {
            QJsonDocument doc = QJsonDocument::fromJson(connectionData->creationSettingsJson.toUtf8());
            if (doc.isObject()) {
                QJsonObject settings = doc.object();
                if (settings.contains("visualization")) {
                    QJsonObject visSettings = settings["visualization"].toObject();
                    QVector3D startPoint(0, 0, 0);
                    QVector3D endPoint(0, 0, 0);
                    
                    if (visSettings.contains("startPoint")) {
                        QJsonObject startObj = visSettings["startPoint"].toObject();
                        startPoint = QVector3D(
                            startObj["x"].toDouble(),
                            startObj["y"].toDouble(),
                            startObj["z"].toDouble()
                        );
                    }
                    if (visSettings.contains("endPoint")) {
                        QJsonObject endObj = visSettings["endPoint"].toObject();
                        endPoint = QVector3D(
                            endObj["x"].toDouble(),
                            endObj["y"].toDouble(),
                            endObj["z"].toDouble()
                        );
                    }
                    
                    // Calculate midpoint from JSON data
                    QVector3D midpoint = (startPoint + endPoint) * 0.5;
                    gp_Trsf connectionTrsf;
                    connectionTrsf.SetTranslation(gp_Vec(midpoint.x(), midpoint.y(), midpoint.z()));
                    connectionNode->loc = TopLoc_Location(connectionTrsf);
                    
                    qDebug() << "[Connection] Set connection node position from JSON data:"
                             << "startPoint:" << startPoint << "endPoint:" << endPoint
                             << "midpoint:" << midpoint;
                } else {
                    qDebug() << "[Connection] No visualization settings in JSON";
                }
            } else {
                qDebug() << "[Connection] Failed to parse JSON settings";
            }
        } else {
            qDebug() << "[Connection] No JSON settings available";
        }
    }
    
    // Add connection points as children with proper world positioning
    auto childPoint1 = std::make_shared<CadNode>(*point1);
    auto childPoint2 = std::make_shared<CadNode>(*point2);
    
    // Give unique names to distinguish the connection points
    childPoint1->name = "Connection_Point_1";
    childPoint2->name = "Connection_Point_2";
    
    // Calculate the world positions of the original connection points
    QVector3D worldPos1(0, 0, 0);
    QVector3D worldPos2(0, 0, 0);
    
    if (!point1->loc.IsIdentity()) {
        gp_Trsf trsf1 = point1->loc.Transformation();
        gp_Pnt pos1 = trsf1.TranslationPart();
        worldPos1 = QVector3D(pos1.X(), pos1.Y(), pos1.Z());
    }
    
    if (!point2->loc.IsIdentity()) {
        gp_Trsf trsf2 = point2->loc.Transformation();
        gp_Pnt pos2 = trsf2.TranslationPart();
        worldPos2 = QVector3D(pos2.X(), pos2.Y(), pos2.Z());
    }
    
    // Calculate the connection node's world position (midpoint)
    QVector3D connectionWorldPos = (worldPos1 + worldPos2) * 0.5;
    
    // Set child point positions relative to the connection node
    QVector3D relativePos1 = worldPos1 - connectionWorldPos;
    QVector3D relativePos2 = worldPos2 - connectionWorldPos;
    
    // Create transformations for the child points
    gp_Trsf childTrsf1;
    childTrsf1.SetTranslation(gp_Vec(relativePos1.x(), relativePos1.y(), relativePos1.z()));
    childPoint1->loc = TopLoc_Location(childTrsf1);
    
    gp_Trsf childTrsf2;
    childTrsf2.SetTranslation(gp_Vec(relativePos2.x(), relativePos2.y(), relativePos2.z()));
    childPoint2->loc = TopLoc_Location(childTrsf2);
    
    qDebug() << "[Connection] Original connection points:";
    qDebug() << "[Connection] Point1 name:" << QString::fromStdString(point1->name);
    qDebug() << "[Connection] Point2 name:" << QString::fromStdString(point2->name);
    qDebug() << "[Connection] Point1 == Point2:" << (point1 == point2);
    qDebug() << "[Connection] Original world positions:";
    qDebug() << "[Connection] Point1 world:" << worldPos1;
    qDebug() << "[Connection] Point2 world:" << worldPos2;
    qDebug() << "[Connection] Connection node world:" << connectionWorldPos;
    qDebug() << "[Connection] Child1 relative:" << relativePos1;
    qDebug() << "[Connection] Child2 relative:" << relativePos2;
    
    connectionNode->children.push_back(childPoint1);
    connectionNode->children.push_back(childPoint2);
    
    // Debug: Check the actual positions of the connection points after adding as children
    qDebug() << "[Connection] After adding children:";
    for (size_t i = 0; i < connectionNode->children.size(); ++i) {
        const auto& child = connectionNode->children[i];
        if (child->type == CadNodeType::ConnectionPoint) {
            if (child->loc.IsIdentity()) {
                qDebug() << "[Connection] Child" << i << "has identity location";
            } else {
                gp_Trsf trsf = child->loc.Transformation();
                gp_Pnt pos = trsf.TranslationPart();
                qDebug() << "[Connection] Child" << i << "position:" << pos.X() << pos.Y() << pos.Z();
            }
        }
    }
    
    // Set parent pointers
    setParentPointersRecursive(connectionNode.get());
    
    return connectionNode;
}

bool canConnectPoints(CadNode* point1, CadNode* point2)
{
    if (!point1 || !point2 || point1->type != CadNodeType::ConnectionPoint || point2->type != CadNodeType::ConnectionPoint) {
        return false;
    }
    
    auto data1 = point1->asConnectionPoint();
    auto data2 = point2->asConnectionPoint();
    
    if (!data1 || !data2) {
        return false;
    }
    
    // Check if both points support the same connection types
    // For now, we'll allow connection if both points support cables
    return data1->canConnectCables() && data2->canConnectCables();
}

double calculateDistanceBetweenPoints(CadNode* point1, CadNode* point2)
{
    if (!point1 || !point2) {
        return 0.0;
    }
    
    // Validate that both nodes are connection points
    if (point1->type != CadNodeType::ConnectionPoint || point2->type != CadNodeType::ConnectionPoint) {
        return 0.0;
    }
    
    try {
        // Find the common parent to use as reference coordinate system
        CadNode* commonParent = findCommonAncestor(point1, point2);
        if (!commonParent) {
            return 0.0;
        }
        
        // Use local positions relative to their immediate parents
        // This is a simplified approach - in a more complex implementation,
        // we'd accumulate transforms up to the common parent
        gp_Pnt pos1 = point1->loc.Transformation().TranslationPart();
        gp_Pnt pos2 = point2->loc.Transformation().TranslationPart();
        
        // Calculate distance using Euclidean distance formula
        double dx = pos2.X() - pos1.X();
        double dy = pos2.Y() - pos1.Y();
        double dz = pos2.Z() - pos1.Z();
        
        double distance = sqrt(dx*dx + dy*dy + dz*dz);
        
        // Validate the result
        if (std::isnan(distance) || std::isinf(distance)) {
            return 0.0;
        }
        
        return distance;
    }
    catch (...) {
        // Handle any exceptions from OpenCascade operations
        return 0.0;
    }
}

double calculateManhattanDistance(CadNode* point1, CadNode* point2)
{
    if (!point1 || !point2) {
        return 0.0;
    }
    
    // Validate that both nodes are connection points
    if (point1->type != CadNodeType::ConnectionPoint || point2->type != CadNodeType::ConnectionPoint) {
        return 0.0;
    }
    
    try {
        // Find the common parent to use as reference coordinate system
        CadNode* commonParent = findCommonAncestor(point1, point2);
        if (!commonParent) {
            return 0.0;
        }
        
        // Use local positions relative to their immediate parents
        gp_Pnt pos1 = point1->loc.Transformation().TranslationPart();
        gp_Pnt pos2 = point2->loc.Transformation().TranslationPart();
        
        // Calculate Manhattan distance (sum of absolute differences)
        double dx = std::abs(pos2.X() - pos1.X());
        double dy = std::abs(pos2.Y() - pos1.Y());
        double dz = std::abs(pos2.Z() - pos1.Z());
        
        double distance = dx + dy + dz;
        
        // Validate the result
        if (std::isnan(distance) || std::isinf(distance)) {
            return 0.0;
        }
        
        return distance;
    }
    catch (...) {
        // Handle any exceptions from OpenCascade operations
        return 0.0;
    }
}

double calculatePathDistance(CadNode* point1, CadNode* point2, const std::vector<gp_Pnt>& waypoints)
{
    if (!point1 || !point2) {
        return 0.0;
    }
    
    // Validate that both nodes are connection points
    if (point1->type != CadNodeType::ConnectionPoint || point2->type != CadNodeType::ConnectionPoint) {
        return 0.0;
    }
    
    try {
        // Find the common parent to use as reference coordinate system
        CadNode* commonParent = findCommonAncestor(point1, point2);
        if (!commonParent) {
            return 0.0;
        }
        
        // Use local positions relative to their immediate parents
        gp_Pnt pos1 = point1->loc.Transformation().TranslationPart();
        gp_Pnt pos2 = point2->loc.Transformation().TranslationPart();
        
        double totalDistance = 0.0;
        gp_Pnt currentPoint = pos1;
        
        // Calculate distance through waypoints
        for (const auto& waypoint : waypoints) {
            double dx = waypoint.X() - currentPoint.X();
            double dy = waypoint.Y() - currentPoint.Y();
            double dz = waypoint.Z() - currentPoint.Z();
            
            double segmentDistance = sqrt(dx*dx + dy*dy + dz*dz);
            
            if (std::isnan(segmentDistance) || std::isinf(segmentDistance)) {
                return 0.0;
            }
            
            totalDistance += segmentDistance;
            currentPoint = waypoint;
        }
        
        // Add final segment to end point
        double dx = pos2.X() - currentPoint.X();
        double dy = pos2.Y() - currentPoint.Y();
        double dz = pos2.Z() - currentPoint.Z();
        
        double finalSegmentDistance = sqrt(dx*dx + dy*dy + dz*dz);
        
        if (std::isnan(finalSegmentDistance) || std::isinf(finalSegmentDistance)) {
            return 0.0;
        }
        
        totalDistance += finalSegmentDistance;
        
        return totalDistance;
    }
    catch (...) {
        // Handle any exceptions from OpenCascade operations
        return 0.0;
    }
}

double calculateDragChainLength(CadNode* point1, CadNode* point2, double bendRadius, double extraLength)
{
    if (!point1 || !point2) {
        return 0.0;
    }
    
    // Get the straight-line distance using the improved calculation
    double straightDistance = calculateDistanceBetweenPoints(point1, point2);
    
    if (straightDistance <= 0.0) {
        return 0.0;
    }
    
    // Calculate the total length for drag chains
    double totalLength = straightDistance;
    
    // Add bend radius contribution for drag chains
    if (bendRadius > 0.0) {
        // Improved bend calculation based on distance and typical drag chain behavior
        // For longer distances, we need more bends
        double typicalSegmentLength = 300.0; // Typical drag chain segment length in mm
        int estimatedBends = std::max(1, static_cast<int>(straightDistance / typicalSegmentLength));
        
        // Each bend adds approximately a quarter circle of length
        double bendLength = estimatedBends * (bendRadius * M_PI / 2.0);
        
        // Add some additional length for the transition between segments
        double transitionLength = estimatedBends * 20.0; // 20mm per transition
        
        totalLength += bendLength + transitionLength;
    }
    
    // Add extra length for slack and manual adjustments
    totalLength += extraLength;
    
    // Ensure we don't return negative or invalid values
    if (std::isnan(totalLength) || std::isinf(totalLength) || totalLength < 0.0) {
        return straightDistance + extraLength; // Fallback to basic calculation
    }
    
    return totalLength;
}

// New function to calculate drag chain length with attachment configurations
double calculateDragChainLengthWithAttachments(CadNode* point1, CadNode* point2, double bendRadius, double pitchLength, int segmentCount,
                                             const ConnectionNodeData::AttachmentConfig& startAttachment,
                                             const ConnectionNodeData::AttachmentConfig& endAttachment)
{
    if (!point1 || !point2) {
        return 0.0;
    }
    
    // Get the straight-line distance
    double straightDistance = calculateDistanceBetweenPoints(point1, point2);
    
    if (straightDistance <= 0.0) {
        return 0.0;
    }
    
    double totalLength = straightDistance;
    
    // Use the specified segment count directly
    int totalSegments = segmentCount;
    
    // Add segments for locked attachments
    if (startAttachment.isLocked) {
        totalSegments += 1; // Additional segment for start attachment
    }
    
    if (endAttachment.isLocked) {
        totalSegments += 1; // Additional segment for end attachment
    }
    
    // Add bend radius contribution
    if (bendRadius > 0.0) {
        // Calculate bends based on attachment configurations
        int estimatedBends = 1; // Base bend
        
        if (startAttachment.isLocked && endAttachment.isLocked) {
            // Two locked attachments require more complex path
            estimatedBends = 3; // Start bend, middle bend, end bend
        } else if (startAttachment.isLocked || endAttachment.isLocked) {
            // One locked attachment
            estimatedBends = 2; // Start/end bend and middle bend
        }
        
        // Each bend adds approximately a quarter circle of length
        double bendLength = estimatedBends * (bendRadius * M_PI / 2.0);
        
        // Add transition length
        double transitionLength = estimatedBends * 20.0; // 20mm per transition
        
        totalLength += bendLength + transitionLength;
    }
    
    // Calculate length based on segment count
    totalLength = totalSegments * pitchLength;
    
    // Ensure we don't return negative or invalid values
    if (std::isnan(totalLength) || std::isinf(totalLength) || totalLength < 0.0) {
        return totalSegments * pitchLength; // Fallback to basic calculation
    }
    
    return totalLength;
}

std::vector<CadNode*> findConnectionPointsInTree(CadNode* root)
{
    std::vector<CadNode*> connectionPoints;
    
    if (!root) return connectionPoints;
    
    std::function<void(CadNode*)> traverse = [&](CadNode* node) {
        if (node->type == CadNodeType::ConnectionPoint) {
            connectionPoints.push_back(node);
        }
        
        for (auto& child : node->children) {
            traverse(child.get());
        }
    };
    
    traverse(root);
    return connectionPoints;
}

// Helper function to find common ancestor of two nodes
CadNode* findCommonAncestor(CadNode* node1, CadNode* node2) {
    if (!node1 || !node2) return nullptr;
    if (node1 == node2) return node1;
    
    // Get paths from root to each node
    std::vector<CadNode*> path1, path2;
    
    // Build path for node1
    CadNode* current = node1;
    while (current) {
        path1.push_back(current);
        current = current->parent;
    }
    
    // Build path for node2
    current = node2;
    while (current) {
        path2.push_back(current);
        current = current->parent;
    }
    
    // Find the first common node from the end of both paths
    int i = path1.size() - 1;
    int j = path2.size() - 1;
    
    while (i >= 0 && j >= 0 && path1[i] == path2[j]) {
        i--;
        j--;
    }
    
    // The common ancestor is the last common node
    if (i + 1 < static_cast<int>(path1.size())) {
        return path1[i + 1];
    }
    
    // If no common ancestor found, return the root (last node in path1)
    if (!path1.empty()) {
        return path1.back();
    }
    
    return nullptr;
}

// Helper function to find the best placement location for a connection
CadNode* findBestConnectionPlacement(CadNode* point1, CadNode* point2) {
    if (!point1 || !point2) return nullptr;
    
    // First, find the common ancestor
    CadNode* commonAncestor = findCommonAncestor(point1, point2);
    if (!commonAncestor) return nullptr;
    
    // If the common ancestor is the root, prefer to place under a physics object
    // or assembly that contains both points
    if (commonAncestor->type == CadNodeType::Custom) {
        // Look for a physics object or assembly that contains both points
        std::vector<CadNode*> path1, path2;
        
        // Build paths from common ancestor to each point
        CadNode* current = point1;
        while (current && current != commonAncestor) {
            path1.push_back(current);
            current = current->parent;
        }
        
        current = point2;
        while (current && current != commonAncestor) {
            path2.push_back(current);
            current = current->parent;
        }
        
        // Look for physics objects or assemblies in the paths
        for (auto node : path1) {
            if (node->type == CadNodeType::Physics || 
                (node->type == CadNodeType::Custom && node->name.find("Assembly") != std::string::npos)) {
                return node;
            }
        }
        
        for (auto node : path2) {
            if (node->type == CadNodeType::Physics || 
                (node->type == CadNodeType::Custom && node->name.find("Assembly") != std::string::npos)) {
                return node;
            }
        }
    }
    
    // Default to common ancestor
    return commonAncestor;
}

// SHARED: Get connection point positions using the same logic as preview
// This ensures both preview and simulation use identical position calculation
std::pair<QVector3D, QVector3D> getConnectionPointPositions(
    CadNode* connectionNode,
    const std::function<bool()>& hasNodeUpdates,
    const std::function<const std::unordered_map<CadNode*, TopLoc_Location>&()>& getLatestNodeLocations)
{
    QVector3D startPoint(0, 0, 0);
    QVector3D endPoint(0, 0, 0);
    
    // Helper function to get node location (same as in preview)
    auto getNodeLocation = [&](const CadNode* node) -> TopLoc_Location {
        if (!node) return TopLoc_Location();
        
        // If we have node updates, check for updated locations
        if (hasNodeUpdates()) {
            const auto& nodeLocations = getLatestNodeLocations();
            auto it = nodeLocations.find(const_cast<CadNode*>(node));
            if (it != nodeLocations.end()) {
                return it->second;
            }
        }
        
        // Fall back to the node's original location
        return node->loc;
    };
    
    // Helper function to find accumulated location for a node (same logic as preview)
    std::function<TopLoc_Location(CadNode*, CadNode*, TopLoc_Location)> findAccumulatedLocation = 
        [&](CadNode* target, CadNode* current, TopLoc_Location parentAccum) -> TopLoc_Location {
            if (!current) return TopLoc_Location();
            
            // Get the node location
            TopLoc_Location nodeLoc = getNodeLocation(current);
            
            // Always accumulate the transform for this node
            TopLoc_Location newAccumulatedLoc = parentAccum * nodeLoc;
            
            // If this is our target, return the accumulated location
            if (current == target) {
                return newAccumulatedLoc;
            }
            
            // Recursively search children
            for (const auto& child : current->children) {
                if (child) {
                    TopLoc_Location childLoc = findAccumulatedLocation(target, child.get(), newAccumulatedLoc);
                    if (!childLoc.IsIdentity()) {
                        // Found the target in this child branch
                        return childLoc;
                    }
                }
            }
            
            return TopLoc_Location();
        };
    
    // Find connection points in the scene graph
    std::vector<CadNode*> connectionPoints;
    for (const auto& child : connectionNode->children) {
        if (child && child->type == CadNodeType::ConnectionPoint) {
            connectionPoints.push_back(child.get());
        }
    }
    
    if (connectionPoints.size() >= 2) {
        // Get the root node for location calculation
        CadNode* rootNode = nullptr;
        // Find root by traversing up from connection node
        CadNode* current = connectionNode;
        while (current && current->parent) {
            current = current->parent;
        }
        rootNode = current;
        
        if (rootNode) {
            // Calculate actual positions using the same logic as preview
            TopLoc_Location startLoc = findAccumulatedLocation(connectionPoints[0], rootNode, TopLoc_Location());
            TopLoc_Location endLoc = findAccumulatedLocation(connectionPoints[1], rootNode, TopLoc_Location());
            
            if (!startLoc.IsIdentity() && !endLoc.IsIdentity()) {
                gp_Trsf startTrsf = startLoc.Transformation();
                gp_Trsf endTrsf = endLoc.Transformation();
                gp_Pnt startPos = startTrsf.TranslationPart();
                gp_Pnt endPos = endTrsf.TranslationPart();
                
                startPoint = QVector3D(startPos.X(), startPos.Y(), startPos.Z());
                endPoint = QVector3D(endPos.X(), endPos.Y(), endPos.Z());
                
                qDebug() << "[Shared] Calculated actual node positions:";
                qDebug() << "[Shared] Start point:" << startPoint;
                qDebug() << "[Shared] End point:" << endPoint;
            } else {
                qDebug() << "[Shared] WARNING: Could not calculate node locations";
            }
        } else {
            qDebug() << "[Shared] WARNING: Could not find root node";
        }
    } else {
        qDebug() << "[Shared] WARNING: Not enough connection points found";
    }
    
    return std::make_pair(startPoint, endPoint);
}

// SHARED: Get connection point positions from direct node references (for preview)
// This ensures both preview and simulation use identical position calculation
std::pair<QVector3D, QVector3D> getConnectionPointPositionsFromNodes(
    CadNode* point1,
    CadNode* point2,
    CadNode* rootNode,
    const std::function<bool()>& hasNodeUpdates,
    const std::function<const std::unordered_map<CadNode*, TopLoc_Location>&()>& getLatestNodeLocations)
{
    QVector3D startPoint(0, 0, 0);
    QVector3D endPoint(0, 0, 0);
    
    // Helper function to get node location (same as in preview)
    auto getNodeLocation = [&](const CadNode* node) -> TopLoc_Location {
        if (!node) return TopLoc_Location();
        
        // If we have node updates, check for updated locations
        if (hasNodeUpdates()) {
            const auto& nodeLocations = getLatestNodeLocations();
            auto it = nodeLocations.find(const_cast<CadNode*>(node));
            if (it != nodeLocations.end()) {
                return it->second;
            }
        }
        
        // Fall back to the node's original location
        return node->loc;
    };
    
    // Helper function to find accumulated location for a node (same logic as preview)
    std::function<TopLoc_Location(CadNode*, CadNode*, TopLoc_Location)> findAccumulatedLocation = 
        [&](CadNode* target, CadNode* current, TopLoc_Location parentAccum) -> TopLoc_Location {
            if (!current) return TopLoc_Location();
            
            // Get the node location
            TopLoc_Location nodeLoc = getNodeLocation(current);
            
            // Always accumulate the transform for this node
            TopLoc_Location newAccumulatedLoc = parentAccum * nodeLoc;
            
            // If this is our target, return the accumulated location
            if (current == target) {
                return newAccumulatedLoc;
            }
            
            // Recursively search children
            for (const auto& child : current->children) {
                if (child) {
                    TopLoc_Location childLoc = findAccumulatedLocation(target, child.get(), newAccumulatedLoc);
                    if (!childLoc.IsIdentity()) {
                        // Found the target in this child branch
                        return childLoc;
                    }
                }
            }
            
            return TopLoc_Location();
        };
    
    if (point1 && point2 && rootNode) {
        // Calculate actual positions using the same logic as preview
        TopLoc_Location startLoc = findAccumulatedLocation(point1, rootNode, TopLoc_Location());
        TopLoc_Location endLoc = findAccumulatedLocation(point2, rootNode, TopLoc_Location());
        
        if (!startLoc.IsIdentity() && !endLoc.IsIdentity()) {
            gp_Trsf startTrsf = startLoc.Transformation();
            gp_Trsf endTrsf = endLoc.Transformation();
            gp_Pnt startPos = startTrsf.TranslationPart();
            gp_Pnt endPos = endTrsf.TranslationPart();
            
            startPoint = QVector3D(startPos.X(), startPos.Y(), startPos.Z());
            endPoint = QVector3D(endPos.X(), endPos.Y(), endPos.Z());
            
            qDebug() << "[Shared] Calculated actual node positions from direct references:";
            qDebug() << "[Shared] Start point:" << startPoint;
            qDebug() << "[Shared] End point:" << endPoint;
        } else {
            qDebug() << "[Shared] WARNING: Could not calculate node locations from direct references";
        }
    } else {
        qDebug() << "[Shared] WARNING: Invalid point or root node references";
    }
    
    return std::make_pair(startPoint, endPoint);
}

// SHARED: Auto-calculate control point using the same logic as preview
QVector3D calculateAutoControlPoint(const QVector3D& startPoint, const QVector3D& endPoint)
{
    QVector3D midPoint = (startPoint + endPoint) * 0.5f;
    QVector3D direction = (endPoint - startPoint).normalized();
    
    // Calculate perpendicular axis in the 2D plane
    QVector3D perpendicular;
    if (std::abs(direction.x()) < 0.9f) {
        perpendicular = QVector3D::crossProduct(QVector3D(1, 0, 0), direction);
    } else {
        perpendicular = QVector3D::crossProduct(QVector3D(0, 1, 0), direction);
    }
    perpendicular.normalize();
    
    double distance = (endPoint - startPoint).length();
    double controlDistance = distance * 0.3; // 30% of total distance
    
    QVector3D autoControlPoint = midPoint + perpendicular * controlDistance;
    return autoControlPoint;
}

// SHARED: Generate waypoint-based segments for both preview and simulation
std::vector<QVector3D> generateWaypointSegments(
    const QVector3D& startPoint, 
    const QVector3D& endPoint, 
    const std::vector<QVector3D>& controlPoints,
    double pitchLength)
{
    std::vector<QVector3D> waypoints;
    waypoints.push_back(startPoint);
    
    if (controlPoints.empty()) {
        // Auto-calculate control point using shared function
        QVector3D autoControlPoint = calculateAutoControlPoint(startPoint, endPoint);
        waypoints.push_back(autoControlPoint);
    } else {
        // Project all control points to the plane defined by start and end points
        for (const auto& controlPoint : controlPoints) {
            // Project control point to plane
            QVector3D direction = (endPoint - startPoint).normalized();
            QVector3D toControl = controlPoint - startPoint;
            double projection = QVector3D::dotProduct(toControl, direction);
            QVector3D projectedControlPoint = startPoint + projection * direction;
            
            // Ensure Z coordinate is consistent
            double targetZ = (startPoint.z() + endPoint.z()) * 0.5;
            projectedControlPoint.setZ(targetZ);
            
            waypoints.push_back(projectedControlPoint);
        }
    }
    
    waypoints.push_back(endPoint);
    return waypoints;
}

// SHARED: Unified connection point position calculation (used by both preview and simulation)
std::pair<QVector3D, QVector3D> getConnectionPointPositionsShared(
    CadNode* point1, 
    CadNode* point2, 
    CadNode* rootNode,
    std::function<bool()> hasNodeUpdates,
    std::function<const std::unordered_map<CadNode*, TopLoc_Location>&()> getLatestNodeLocations) {
    
    // Use the same logic as getConnectionPointPositionsFromNodes
    return getConnectionPointPositionsFromNodes(point1, point2, rootNode, hasNodeUpdates, getLatestNodeLocations);
}

// SHARED: Generate drag chain segments using the same logic for both preview and simulation
std::vector<QVector3D> generateDragChainSegments(
    const QVector3D& startPoint,
    const QVector3D& endPoint,
    const std::vector<QVector3D>& controlPoints,
    double pitchLength,
    double bendRadius)
{
    qDebug() << "[Shared] generateDragChainSegments called with:";
    qDebug() << "[Shared] Start point:" << startPoint;
    qDebug() << "[Shared] End point:" << endPoint;
    qDebug() << "[Shared] Control points count:" << controlPoints.size();
    qDebug() << "[Shared] Pitch length:" << pitchLength;
    qDebug() << "[Shared] Bend radius:" << bendRadius;
    
    // Generate waypoints using shared function
    std::vector<QVector3D> waypoints = generateWaypointSegments(startPoint, endPoint, controlPoints, pitchLength);
    
    qDebug() << "[Shared] Generated" << waypoints.size() << "waypoints";
    for (size_t i = 0; i < waypoints.size(); ++i) {
        qDebug() << "[Shared] Waypoint" << i << ":" << waypoints[i];
    }
    
    // Generate segments between waypoints
    std::vector<QVector3D> segments;
    
    for (size_t i = 0; i < waypoints.size() - 1; ++i) {
        QVector3D segmentStart = waypoints[i];
        QVector3D segmentEnd = waypoints[i + 1];
        
        // Calculate distance between waypoints
        double distance = (segmentEnd - segmentStart).length();
        int segmentCount = static_cast<int>(std::ceil(distance / pitchLength));
        
        if (segmentCount < 1) segmentCount = 1;
        
        qDebug() << "[Shared] Waypoint segment" << i << ":" << segmentStart << "->" << segmentEnd;
        qDebug() << "[Shared] Distance:" << distance << "mm, generating" << segmentCount << "segments";
        
        // Generate segments along the straight line
        for (int j = 0; j < segmentCount; ++j) {
            double t = static_cast<double>(j) / segmentCount;
            QVector3D segmentStartPos = segmentStart + t * (segmentEnd - segmentStart);
            
            double nextT = static_cast<double>(j + 1) / segmentCount;
            if (j == segmentCount - 1) nextT = 1.0;
            QVector3D segmentEndPos = segmentStart + nextT * (segmentEnd - segmentStart);
            
            // Add segment start and end points
            segments.push_back(segmentStartPos);
            segments.push_back(segmentEndPos);
            
            qDebug() << "[Shared] Created segment" << segments.size()/2 - 1 << ":" << segmentStartPos << "->" << segmentEndPos;
        }
    }
    
    qDebug() << "[Shared] Generated" << segments.size()/2 << "segments total";
    qDebug() << "[Shared] Segment points:";
    for (size_t i = 0; i < segments.size(); i += 2) {
        if (i + 1 < segments.size()) {
            qDebug() << "[Shared] Segment" << i/2 << ":" << segments[i] << "->" << segments[i+1];
        }
    }
    return segments;
}

// Helper function to find accumulated location for a target node in a tree
TopLoc_Location findAccumulatedLocation(CadNode* target, CadNode* current, TopLoc_Location parentAccum) {
    if (!current) return TopLoc_Location();
    
    // Get the node location
    TopLoc_Location nodeLoc = current->loc;
    
    // Always accumulate the transform for this node
    TopLoc_Location newAccumulatedLoc = parentAccum * nodeLoc;
    
    // If this is our target, return the accumulated location
    if (current == target) {
        return newAccumulatedLoc;
    }
    
    // Recursively search children
    for (const auto& child : current->children) {
        if (child) {
            TopLoc_Location childLoc = findAccumulatedLocation(target, child.get(), newAccumulatedLoc);
            if (!childLoc.IsIdentity()) {
                // Found the target in this child branch
                return childLoc;
            }
        }
    }
    
    return TopLoc_Location();
}