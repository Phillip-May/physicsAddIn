#include <QApplication>
#include <QMainWindow>
#include <QTreeView>
#include <QFileDialog>
#include <QVBoxLayout>
#include <QWidget>
#include <QMessageBox>
#include <QString>
#include <QHBoxLayout>
#include <QSplitter>
#include "CadOpenGLWidget.h"
#include "CadTreeModel.h"
#include <memory>
#include <QDebug>
#include <BRep_Builder.hxx>
#include <TopoDS_Compound.hxx>
#include <QSet>
#include <QHash>
#include <TDF_ChildIterator.hxx>
#include <TDF_AttributeIterator.hxx>
#include <TDataStd_TreeNode.hxx>
#include <TDF_RelocationTable.hxx>
#include <BRepGProp.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Shape.hxx>
#include <set>
#include <QComboBox>
#include <QCheckBox>
#include <Geom_Plane.hxx>
#include <BRepTools.hxx>
#include <gp_Quaternion.hxx>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonObject>
#include <QJsonDocument>
#include <GeomLProp_SLProps.hxx>

#include "CadNode.h"
#include "XCAFLabelTreeModel.h"
#include "CadOpenGLWidget.h" // For SelectionMode enum
#include "CustomModelTreeModel.h"
#include <QDialog>
#include <QFormLayout>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QPushButton>
#include <QVBoxLayout>
#include <QPoint>
#include <QInputDialog>
#include <QLineEdit>
#include <QTextEdit>
#include <unordered_map>
#include <TDF_Tool.hxx>
#include <vector>
#include "RailJsonEditorDialog.h"
#include <QVariant>
#include <BRepPrimAPI_MakePrism.hxx>
#include "HelperFunctions.h"

// OpenCascade includes for STEP loading and XCAF
#include <STEPCAFControl_Reader.hxx>
#include <TDocStd_Document.hxx>
#include <TDF_LabelSequence.hxx>
#include <TDF_ChildIterator.hxx>
#include <XCAFApp_Application.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <TDataStd_Name.hxx>
#include <Quantity_Color.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Solid.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Edge.hxx>
#include <TopExp_Explorer.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <TopoDS.hxx>
#include <sstream>
#include <QPushButton>
#include <TopLoc_Location.hxx>
#include <QVector3D>
#include <QTabWidget>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <BRep_Tool.hxx>
#include <Geom_Surface.hxx>
#include <STEPCAFControl_Writer.hxx>
#include <IGESCAFControl_Writer.hxx>
#include <BinDrivers.hxx>
#include <XmlDrivers.hxx>
#include <PCDM_StoreStatus.hxx>
#include <QFileInfo>
#include <QDir>
#include <BinXCAFDrivers.hxx>
#include <XmlXCAFDrivers.hxx>
#include <GProp_GProps.hxx>

// qHash overload for TDF_Label to allow use in QSet/QHash
#include <TDF_Label.hxx>

#include "SimulationManager.h"

// Place this above build_tree_xcaf
struct FaceEdgeKeyHash {
    std::size_t operator()(const std::pair<const void*, const void*>& k) const noexcept {
        std::size_t h1 = std::hash<const void*>{}(k.first);
        std::size_t h2 = std::hash<const void*>{}(k.second);
        return h1 ^ (h2 << 1);
    }
};

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

// Qt model for the tree view
#include <QAbstractItemModel>
#include <QMenu>
#include <QAction>

// Generic helper to connect selection between a tree and a viewer
// ModelType must provide getNode(QModelIndex) and indexForNode(CadNode*)
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
        qDebug() << "[PROFILE] selectionChanged: start";
        auto t0 = std::chrono::high_resolution_clock::now();
        viewer->clearSelection();
        QModelIndexList selectedIndexes = tree->selectionModel()->selectedIndexes();
        QSet<CadNode*> allNodesToSelect;
        auto t_descendants_start = std::chrono::high_resolution_clock::now();
        int descendantCalls = 0;
        long long totalDescendantTime = 0;
        long long maxDescendantTime = 0;
        for (const QModelIndex& index : selectedIndexes) {
            CadNode* node = const_cast<CadNode*>(model->getNode(index));
            if (node) {
                allNodesToSelect.insert(node);
                // Optionally add descendants
                std::function<void(CadNode*)> addDescendants = [&](CadNode* currentNode) {
                    auto t1 = std::chrono::high_resolution_clock::now();
                    for (const auto& child : currentNode->children) {
                        if (child) {
                            allNodesToSelect.insert(child.get());
                            addDescendants(child.get());
                        }
                    }
                    auto t2 = std::chrono::high_resolution_clock::now();
                    long long dt = std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count();
                    totalDescendantTime += dt;
                    if (dt > maxDescendantTime) maxDescendantTime = dt;
                    descendantCalls++;
                };
                addDescendants(node);
            }
        }
        auto t_descendants_end = std::chrono::high_resolution_clock::now();
        qDebug() << "[PROFILE] selectionChanged: allNodesToSelect size =" << allNodesToSelect.size();
        qDebug() << "[PROFILE] addDescendants: total time ="
                 << std::chrono::duration_cast<std::chrono::milliseconds>(t_descendants_end-t_descendants_start).count() << "ms,"
                 << "calls:" << descendantCalls << ", total us:" << totalDescendantTime << ", max us:" << maxDescendantTime;
        // Helper to get parent node from model
        auto getParent = [=](CadNode* n) -> CadNode* {
            return const_cast<CadNode*>(model->getParentNode(n));
        };
        int count = 0;
        long long totalAccumulateTime = 0, maxAccumulateTime = 0;
        long long totalAddToSelectionTime = 0, maxAddToSelectionTime = 0;
        for (CadNode* node : allNodesToSelect) {
            auto t_acc_start = std::chrono::high_resolution_clock::now();
            TopLoc_Location accLoc = node->globalLoc;
            auto t_acc_end = std::chrono::high_resolution_clock::now();
            long long accTime = std::chrono::duration_cast<std::chrono::microseconds>(t_acc_end-t_acc_start).count();
            totalAccumulateTime += accTime;
            if (accTime > maxAccumulateTime) maxAccumulateTime = accTime;
            if (count < 5) {
                qDebug() << "[PROFILE] accumulateTransform node" << count << "took" << accTime << "us";
            }
            auto t_sel_start = std::chrono::high_resolution_clock::now();
            viewer->addToSelection(node, accLoc);
            auto t_sel_end = std::chrono::high_resolution_clock::now();
            long long selTime = std::chrono::duration_cast<std::chrono::microseconds>(t_sel_end-t_sel_start).count();
            totalAddToSelectionTime += selTime;
            if (selTime > maxAddToSelectionTime) maxAddToSelectionTime = selTime;
            if (count < 5) {
                qDebug() << "[PROFILE] addToSelection node" << count << "took" << selTime << "us";
            }
            ++count;
        }
        qDebug() << "[PROFILE] accumulateTransform: total us =" << totalAccumulateTime << ", max us =" << maxAccumulateTime;
        qDebug() << "[PROFILE] addToSelection: total us =" << totalAddToSelectionTime << ", max us =" << maxAddToSelectionTime;
        auto t3 = std::chrono::high_resolution_clock::now();
        qDebug() << "[PROFILE] selectionChanged: total time ="
                 << std::chrono::duration_cast<std::chrono::milliseconds>(t3-t0).count() << "ms for"
                 << allNodesToSelect.size() << "nodes.";
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
            QItemSelectionModel* selModel = tree->selectionModel();
            if (selModel) {
                selModel->select(idx, QItemSelectionModel::Select | QItemSelectionModel::Rows);
                tree->scrollTo(idx);
            }
        }
    });
    QObject::connect(viewer, &CadOpenGLWidget::edgePicked, tree, [=](CadNode* node) {
        QModelIndex idx = model->indexForNode(node);
        if (idx.isValid()) {
            QModelIndex parentIdx = idx.parent();
            while (parentIdx.isValid()) {
                tree->expand(parentIdx);
                parentIdx = parentIdx.parent();
            }
            tree->setCurrentIndex(idx);
            tree->scrollTo(idx);
        }
    });
}


// Helper: Compute the bounding box length along a given axis for a node
static double computeBoundingBoxLength(const CadNode* node, const QVector3D& axis) {
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


// --- Add these two functions above main() ---
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
    QString railBinFile = railJsonFile;
    railBinFile.replace(".json", ".bin");
    if (!QFile::exists(railBinFile)) {
        QString altBin = railBinFile + ".xbf";
        if (QFile::exists(altBin)) {
            railBinFile = altBin;
        }
    }
    if (!QFile::exists(railBinFile)) {
        QMessageBox::critical(nullptr, "Error", "Could not find corresponding .bin or .bin.xbf file for the selected project.");
        return false;
    }
    // Load XCAF document
    Handle(XCAFApp_Application) appOCC = XCAFApp_Application::GetApplication();
    appOCC->NewDocument("BinXCAF", doc);
    Handle(TDocStd_Application) occApp = Handle(TDocStd_Application)::DownCast(doc->Application());
    BinXCAFDrivers::DefineFormat(occApp);
    PCDM_ReaderStatus status = occApp->Open(railBinFile.toStdWString().c_str(), doc);
    if (status != PCDM_RS_OK) {
        QMessageBox::critical(nullptr, "Error", "Failed to load XCAF binary file: " + railBinFile);
        return false;
    }
    // Build shape and color tools
    shapeTool = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
    colorTool = XCAFDoc_DocumentTool::ColorTool(doc->Main());
    // Build main CAD tree from XCAF
    CADNodeColor defaultColor = CADNodeColor::fromSRGB(200, 200, 200);
    TopLoc_Location identityLoc;
    TDF_LabelSequence roots;
    shapeTool->GetFreeShapes(roots);
    cadRoot = std::make_unique<CadNode>();
    cadRoot->name = "Root";
    cadRoot->color = defaultColor;
    cadRoot->loc = identityLoc;
    std::unordered_map<const void*, std::shared_ptr<CadNode>> sharedNodeMap;
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
    customModelRootContainer->visible = true;
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
    cadRoot = std::make_unique<CadNode>();
    cadRoot->name = "Root";
    cadRoot->color = defaultColor;
    cadRoot->loc = identityLoc;
    std::unordered_map<const void*, std::shared_ptr<CadNode>> sharedNodeMap;
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

// Add these vectors at file scope, before main()
std::vector<QTreeView*> g_treeViews;
std::vector<CadOpenGLWidget*> g_openGLViews;

SimulationManager simManager;

void initTreeAndOpenGLWidget(std::shared_ptr<CadNode> &inputRoot,
                             QTabWidget* treeTabWidget,
                             QTabWidget* openGLTabWidget,
                             std::string name,
                             const Handle(TDocStd_Document)& doc,
                             CadTreeModel* cadTreeModel = nullptr) {
    inputRoot->name = "Tree Root " + name;
    inputRoot->type = CadNodeType::Custom;
    inputRoot->visible = true;
    CustomModelTreeModel* qtModel = new CustomModelTreeModel(inputRoot);
    QTreeView* treeView = new QTreeView;
    treeView->setModel(qtModel);
    treeView->setHeaderHidden(false);
    treeView->setSelectionMode(QAbstractItemView::ExtendedSelection);
    // --- Context menu support for all custom model/physics trees ---
    treeView->setContextMenuPolicy(Qt::CustomContextMenu);
    CadOpenGLWidget* openGLViewer = new CadOpenGLWidget;
    simManager.registerPhysicsNodeContextMenu(treeView);
    QObject::connect(treeView, &QTreeView::customContextMenuRequested, treeView, [=](const QPoint& pos) {
        QModelIndex idx = treeView->indexAt(pos);
        if (!idx.isValid()) return;
        CadNode* node = const_cast<CadNode*>(qtModel->getNode(idx));
        if (!node) return;
        QMenu menu;
        // --- Save/Load Node to/from JSON ---
        QAction* saveNodeAction = menu.addAction("Save Node to JSON...");
        QObject::connect(saveNodeAction, &QAction::triggered, treeView, [=]() {
            QString fileName = QFileDialog::getSaveFileName(nullptr, "Save Node as JSON", "", "JSON Files (*.json)");
            if (fileName.isEmpty()) return;
            if (!fileName.endsWith(".json")) fileName += ".json";
            std::shared_ptr<CadNode> nodeToSave = std::make_shared<CadNode>(*node);
            // Debug print for XCAF face/edge nodes
            std::function<void(const CadNode*)> printIndex;
            printIndex = [&](const CadNode* n) {
                if (!n) return;
                for (const auto& child : n->children) printIndex(child.get());
            };
            printIndex(nodeToSave.get());
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
            qDebug() << "[LoadNode] Triggered load from JSON";
            QString fileName = QFileDialog::getOpenFileName(nullptr, "Load Node from JSON", "", "JSON Files (*.json)");
            if (fileName.isEmpty()) { qDebug() << "[LoadNode] Cancelled: no file selected"; return; }
            QFile file(fileName);
            if (!file.open(QIODevice::ReadOnly)) { qDebug() << "[LoadNode] Failed to open file"; return; }
            QByteArray data = file.readAll();
            file.close();
            QJsonParseError err;
            QJsonDocument docJSON = QJsonDocument::fromJson(data, &err);
            if (docJSON.isNull() || !docJSON.isObject()) { qDebug() << "[LoadNode] Invalid JSON"; return; }
            std::shared_ptr<CadNode> loadedNode = CadNode::fromJson(docJSON.object());
            if (!loadedNode) { qDebug() << "[LoadNode] Failed to deserialize node"; return; }
            // Clear the selected node's children and update its properties
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
            qtModel->dataChanged(idx, idx);
            setParentPointersRecursive(node); // Ensure parent pointers are set
            qDebug() << "[LoadNode] About to relink XCAF nodes";
            std::function<void(CadNode*)> relinkXCAF = [&](CadNode* n) {
                if (!n) return;
                if (n->type == CadNodeType::XCAF) {
                    XCAFNodeData* xData = n->asXCAF();
                    if (xData && !xData->labelPath.empty()) {
                        TDF_Label label = findLabelByPath(doc, xData->labelPath);
                        QVariantList labelPathDebug;
                        for (int tag : xData->labelPath) labelPathDebug << tag;
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
                                    // shapeIndex == -1: assign the main shape
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
                for (auto& child : n->children) relinkXCAF(child.get());
            };
            relinkXCAF(node);
            // Always update parent pointers and global transforms after loading from JSON
            setParentPointersRecursive(node);
            setGlobalLocRecursive(node, node->parent ? node->parent->globalLoc : TopLoc_Location());
            openGLViewer->clearSelection();
            openGLViewer->markCacheDirty();
            QMessageBox::information(nullptr, "Success", "Node loaded and replaced from:\n" + fileName);
        });
        // Physics node: collision mesh toggle
        if (node->type == CadNodeType::Physics) {
            QAction* toggleCollisionMeshAction = nullptr;
            if (node->asPhysics()->collisionMeshVisible) {
                toggleCollisionMeshAction = menu.addAction("Hide Collision Mesh");
            } else {
                toggleCollisionMeshAction = menu.addAction("Show Collision Mesh");
            }
            QObject::connect(toggleCollisionMeshAction, &QAction::triggered, treeView, [=]() {
                node->asPhysics()->collisionMeshVisible = !node->asPhysics()->collisionMeshVisible;
                qtModel->dataChanged(idx, idx);
                // Optionally update viewer if needed
            });
            // --- Add VHACD and CoACD generation actions ---
            QAction* vhacdAction = menu.addAction("Generate Collision Mesh (VHACD)");
            QObject::connect(vhacdAction, &QAction::triggered, treeView, [=]() {
                // Default parameters as JSON
                QJsonObject defaultParams{
                    {"resolution", 100000},
                    {"maxHulls", 16},
                    {"minVolume", 0.01}
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
                    int resolution = obj.value("resolution").toInt(100000);
                    int maxHulls = obj.value("maxHulls").toInt(16);
                    double minVolume = obj.value("minVolume").toDouble(0.01);
                    generateVHACDStub(QString::fromStdString(node->name), resolution, maxHulls, minVolume, node);
                    openGLViewer->markCacheDirty();
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
                    openGLViewer->markCacheDirty();
                    QMessageBox::information(nullptr, "CoACD", "CoACD collision mesh generation complete.");
                }
            });
        }
        // Exclude by color submenu (unified for all CadNode-based trees)
        QMenu* excludeByColorMenu = menu.addMenu("Exclude By Color");
        // Collect all unique colors in the subtree (including this node)
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
        // For each color, add an action to the submenu
        for (const QString& key : uniqueColorKeys) {
            QColor color = colorKeyToColor[key];
            QString colorText = QString("RGB(%1, %2, %3)")
                .arg(int(color.redF() * 255))
                .arg(int(color.greenF() * 255))
                .arg(int(color.blueF() * 255));
            QAction* colorAction = new QAction(colorText, excludeByColorMenu);
            // Add color swatch
            QPixmap pix(16, 16);
            pix.fill(color);
            colorAction->setIcon(QIcon(pix));
            excludeByColorMenu->addAction(colorAction);
            QObject::connect(colorAction, &QAction::triggered, treeView, [=]() {
                // Helper: recursively exclude nodes by color
                std::vector<QModelIndex> changedIndices;
                std::function<void(CadNode*)> excludeByColor;
                excludeByColor = [&](CadNode* n) {
                    if (!n) return;
                    if (n->color.toQColor() == color && !n->excludedFromDecomposition) {
                        n->excludedFromDecomposition = true;
                        QModelIndex changedIdx = qtModel->indexForNode(n);
                        if (changedIdx.isValid()) changedIndices.push_back(changedIdx);
                    }
                    for (auto& child : n->children) excludeByColor(child.get());
                };
                excludeByColor(const_cast<CadNode*>(node));
                // Emit dataChanged for all affected indices
                for (const QModelIndex& changedIdx : changedIndices) {
                    qtModel->dataChanged(changedIdx, changedIdx);
                }
            });
        }
        // Add generic info action for all nodes
        QAction* infoAction = menu.addAction(QString("Node type: %1").arg((int)node->type));
        infoAction->setEnabled(false);
        // Add debug action to print node pointer
        QAction* printPtrAction = menu.addAction("Print Node Pointer (Debug)");
        QObject::connect(printPtrAction, &QAction::triggered, treeView, [=]() {
            qDebug() << "[Debug] Node pointer:" << static_cast<const void*>(node);
            if (auto xData = node->asXCAF()) {
                QString typeStr = shapeTypeToString(xData->type);
                bool isNull = xData->shape.IsNull();
                qDebug() << "[Debug]   XCAF info:";
                qDebug() << "[Debug]     shapeIndex:" << xData->shapeIndex; // <-- Added line
                qDebug() << "[Debug]     shape type:" << typeStr;
                qDebug() << "[Debug]     shape isNull:" << isNull;
                // Print shape pointer (TShape pointer) and address
                const void* shapePtr = xData->shape.IsNull() ? nullptr : xData->shape.TShape().get();
                qDebug() << "[Debug]     shape TShape pointer:" << shapePtr;
                // Print shape address (handle)
                qDebug() << "[Debug]     shape handle address:" << static_cast<const void*>(&xData->shape);
                // Print number of faces/edges if possible
                int faceCount = 0, edgeCount = 0;
                if (!xData->shape.IsNull()) {
                    for (TopExp_Explorer exp(xData->shape, TopAbs_FACE); exp.More(); exp.Next()) ++faceCount;
                    for (TopExp_Explorer exp(xData->shape, TopAbs_EDGE); exp.More(); exp.Next()) ++edgeCount;
                }
                qDebug() << "[Debug]     shape face count:" << faceCount;
                qDebug() << "[Debug]     shape edge count:" << edgeCount;
            }
            if (auto physData = node->asPhysics()) {
                qDebug() << "[Debug]   PHYSICS info:";
                qDebug() << "[Debug]     convexHullGenerated:" << physData->convexHullGenerated;
                qDebug() << "[Debug]     collisionMeshVisible:" << physData->collisionMeshVisible;
                qDebug() << "[Debug]     hulls.size():" << physData->hulls.size();
                qDebug() << "[Debug]     node name:" << QString::fromStdString(node->name);
                if (!physData->hulls.empty()) {
                    const auto& hull = physData->hulls[0];
                    qDebug() << "[Debug]     First hull vertex count:" << hull.vertices.size();
                    qDebug() << "[Debug]     First hull triangle count:" << hull.indices.size();
                    for (int i = 0; i < std::min<int>(3, hull.vertices.size()); ++i) {
                        const auto& v = hull.vertices[i];
                        qDebug() << "[Debug]       Vertex" << i << ":" << v[0] << v[1] << v[2];
                    }
                }
            }
        });
        // Add relink to XCAF node action (always for XCAF nodes)
        if (node->asXCAF()) {
            QAction* relinkAction = menu.addAction("Relink to XCAF Node (Debug)");
            QObject::connect(relinkAction, &QAction::triggered, treeView, [=]() {
                std::shared_ptr<CadNode> nodePtr = std::make_shared<CadNode>(*node);
                qDebug() << "[Relink] Starting relink for node:" << QString::fromStdString(node->name);
                setParentPointersRecursive(nodePtr.get()); // Ensure parent pointers are set
                qDebug() << "[Relink] Finished relink for node:" << QString::fromStdString(nodePtr->name)
                         << ", new type:" << int(nodePtr->type)
                         << ", new data valid:" << (nodePtr->data != nullptr);
                for (const auto& child : nodePtr->children) {
                    qDebug() << "[Relink] Child node:" << QString::fromStdString(child->name)
                             << ", type:" << int(child->type);
                }
                QMessageBox::information(nullptr, "Relink Complete", "Relinked node to XCAF geometry. See debug output for details.");
            });
        }
        // Show/Hide node action (unified for all CadNode-based trees)
        QAction* showHideAction = nullptr;
        if (node->visible) {
            showHideAction = menu.addAction("Hide Node");
        } else {
            showHideAction = menu.addAction("Show Node");
        }
        QObject::connect(showHideAction, &QAction::triggered, treeView, [=]() {
            bool newVisibility = !node->visible;
            // Recursively set visibility for node and all children
            std::vector<QModelIndex> changedIndices;
            std::function<void(CadNode*)> setVisibility;
            setVisibility = [&](CadNode* n) {
                if (!n) return;
                n->visible = newVisibility;
                QModelIndex changedIdx = qtModel->indexForNode(n);
                if (changedIdx.isValid()) changedIndices.push_back(changedIdx);
                for (auto& child : n->children) setVisibility(child.get());
            };
            setVisibility(const_cast<CadNode*>(qtModel->getNode(idx)));
            // Emit dataChanged for all affected indices
            for (const QModelIndex& changedIdx : changedIndices) {
                qtModel->dataChanged(changedIdx, changedIdx);
            }
        });
        // Exclude/Include node action (unified for all CadNode-based trees)
        QAction* excludeIncludeAction = nullptr;
        if (!node->excludedFromDecomposition) {
            excludeIncludeAction = menu.addAction("Exclude Node (and Children)");
        } else {
            excludeIncludeAction = menu.addAction("Include Node (and Children)");
        }
        QObject::connect(excludeIncludeAction, &QAction::triggered, treeView, [=]() {
            bool newExcluded = !node->excludedFromDecomposition;
            // Recursively set excludedFromDecomposition for node and all children
            std::vector<QModelIndex> changedIndices;
            std::function<void(CadNode*)> setExcluded;
            setExcluded = [&](CadNode* n) {
                if (!n) return;
                n->excludedFromDecomposition = newExcluded;
                QModelIndex changedIdx = qtModel->indexForNode(n);
                if (changedIdx.isValid()) changedIndices.push_back(changedIdx);
                for (auto& child : n->children) setExcluded(child.get());
            };
            setExcluded(const_cast<CadNode*>(qtModel->getNode(idx)));
            // Emit dataChanged for all affected indices
            for (const QModelIndex& changedIdx : changedIndices) {
                qtModel->dataChanged(changedIdx, changedIdx);
            }
        });
        // Add Delete Node action (not for root)
        if (node != qtModel->getRootNodePointer()) {
            QAction* deleteAction = menu.addAction("Delete Node");
            QObject::connect(deleteAction, &QAction::triggered, treeView, [=]() {
                qtModel->removeNode(node);
                // Optionally, clear selection or select parent
                QModelIndex parentIdx = qtModel->indexForNode(qtModel->getParentNode(node));
                if (parentIdx.isValid()) {
                    treeView->setCurrentIndex(parentIdx);
                } else {
                    treeView->clearSelection();
                }
            });
        }
        // --- Align Second Face to First Action ---
        // Only enable if exactly two faces are selected
        QModelIndexList selectedIndexes = treeView->selectionModel()->selectedIndexes();
        std::vector<CadNode*> selectedFaces;
        for (const QModelIndex& selIdx : selectedIndexes) {
            CadNode* selNode = const_cast<CadNode*>(qtModel->getNode(selIdx));
            if (selNode && selNode->asXCAF() && selNode->asXCAF()->type == TopAbs_FACE) {
                selectedFaces.push_back(selNode);
            }
        }
        // --- Add Connection Point Action ---
        // Only show for a single selected face node under a Rail ancestor
        if (selectedFaces.size() == 1) {
            CadNode* faceNode = selectedFaces[0];
            // Find the nearest Rail ancestor
            CadNode* railAncestor = const_cast<CadNode*>(qtModel->getParentNode(faceNode));
            while (railAncestor && railAncestor->type != CadNodeType::Rail) {
                CadNode* next = const_cast<CadNode*>(qtModel->getParentNode(railAncestor));
                if (!next || next == railAncestor) break;
                railAncestor = next;
            }
            if (railAncestor && railAncestor->type == CadNodeType::Rail) {
                QAction* addConnPtAction = menu.addAction("Add Connection Point");
                QObject::connect(addConnPtAction, &QAction::triggered, treeView, [=]() {
                    // Compute global transform of face node
                    auto accumulateTransform = [qtModel](CadNode* node) -> TopLoc_Location {
                        TopLoc_Location acc;
                        std::vector<CadNode*> ancestry;
                        CadNode* cur = node;
                        while (cur) {
                            ancestry.push_back(cur);
                            cur = const_cast<CadNode*>(qtModel->getParentNode(cur));
                        }
                        for (auto it = ancestry.rbegin(); it != ancestry.rend(); ++it) {
                            acc = acc * (*it)->loc;
                        }
                        return acc;
                    };
                    TopLoc_Location faceGlobal = accumulateTransform(faceNode);
                    TopLoc_Location railGlobal = accumulateTransform(railAncestor);
                    // Get face center in world coordinates
                    XCAFNodeData* xData = faceNode->asXCAF();
                    if (!xData || !xData->hasFace()) {
                        QMessageBox::warning(nullptr, "Add Connection Point", "Selected node is not a valid face.");
                        return;
                    }
                    TopoDS_Face face = xData->getFace();
                    TopoDS_Face locatedFace = TopoDS::Face(face.Located(faceGlobal));
                    Bnd_Box bbox;
                    BRepBndLib::Add(locatedFace, bbox);
                    Standard_Real xmin, ymin, zmin, xmax, ymax, zmax;
                    bbox.Get(xmin, ymin, zmin, xmax, ymax, zmax);
                    gp_Pnt center((xmin + xmax) / 2, (ymin + ymax) / 2, (zmin + zmax) / 2);
                    // Compute local transform for the connection point relative to the rail node
                    gp_Trsf connTrsf;
                    gp_Pnt origin(0, 0, 0);
                    gp_Vec offset(origin, center);
                    connTrsf.SetTranslation(offset);
                    // The local transform is from the rail node's global to the face center
                    TopLoc_Location connLoc = railGlobal.Inverted() * TopLoc_Location(connTrsf) * railGlobal;
                    // Create the connection point node
                    auto connNode = std::make_shared<CadNode>();
                    connNode->name = "Connection Point";
                    connNode->type = CadNodeType::ConnectionPoint;
                    connNode->color = CADNodeColor::fromSRGB(255, 0, 255); // Magenta for visibility
                    connNode->loc = railGlobal.Inverted() * TopLoc_Location(connTrsf) * railGlobal;
                    connNode->visible = true;
                    connNode->data = std::make_shared<ConnectionPointData>();
                    // Add as child of the rail node
                    railAncestor->children.push_back(connNode);
                    setParentPointersRecursive(railAncestor);
                    // Refresh the tree model
                    QModelIndex railIdx = qtModel->indexForNode(railAncestor);
                    qtModel->dataChanged(railIdx, railIdx);
                    QMessageBox::information(nullptr, "Add Connection Point", "Connection point added under Rail node.");
                });
            }
        }
        QAction* alignFacesAction = nullptr;
        if (selectedFaces.size() == 2) {
            alignFacesAction = menu.addAction("Align Second Face to First");
            alignFacesAction->setEnabled(true);
            QObject::connect(alignFacesAction, &QAction::triggered, treeView, [=]() {
                // Get the two face nodes
                CadNode* face1 = selectedFaces[0];
                CadNode* face2 = selectedFaces[1];
                // Find the topmost ancestor of type Physics for the second face
                CadNode* physicsAncestor = const_cast<CadNode*>(qtModel->getParentNode(face2));
                while (physicsAncestor && physicsAncestor->type != CadNodeType::Physics) {
                    CadNode* next = const_cast<CadNode*>(qtModel->getParentNode(physicsAncestor));
                    if (!next || next == physicsAncestor) break;
                    physicsAncestor = next;
                }
                if (!face1 || !face2 || !physicsAncestor || physicsAncestor->type != CadNodeType::Physics) {
                    QMessageBox::warning(nullptr, "Align Faces", "Could not find a Physics ancestor for the second face.");
                    return;
                }
                // Get face geometry (center and normal)
                XCAFNodeData* xData1 = face1->asXCAF();
                XCAFNodeData* xData2 = face2->asXCAF();
                if (!xData1 || !xData2 || !xData1->hasFace() || !xData2->hasFace()) {
                    QMessageBox::warning(nullptr, "Align Faces", "Selected nodes are not valid faces.");
                    return;
                }
                TopoDS_Face topoFace1 = xData1->getFace();
                TopoDS_Face topoFace2 = xData2->getFace();
                // Apply accumulated transforms
                auto accumulateTransform = [qtModel](CadNode* node) -> TopLoc_Location {
                    TopLoc_Location acc;
                    std::vector<CadNode*> ancestry;
                    CadNode* cur = node;
                    while (cur) {
                        ancestry.push_back(cur);
                        cur = const_cast<CadNode*>(qtModel->getParentNode(cur));
                    }
                    for (auto it = ancestry.rbegin(); it != ancestry.rend(); ++it) {
                        acc = acc * (*it)->loc;
                    }
                    return acc;
                };
                TopLoc_Location loc1 = accumulateTransform(face1);
                TopLoc_Location loc2 = accumulateTransform(face2);
                TopoDS_Face face1Located = TopoDS::Face(topoFace1.Located(loc1));
                TopoDS_Face face2Located = TopoDS::Face(topoFace2.Located(loc2));
                // Get surface and compute center and normal for each face
                Handle(Geom_Surface) surf1 = BRep_Tool::Surface(face1Located);
                Handle(Geom_Surface) surf2 = BRep_Tool::Surface(face2Located);
                if (surf1.IsNull() || surf2.IsNull()) {
                    QMessageBox::warning(nullptr, "Align Faces", "Could not get face surfaces.");
                    return;
                }
                // Compute center (use face bounding box center)
                Bnd_Box bbox1, bbox2;
                BRepBndLib::Add(face1Located, bbox1);
                BRepBndLib::Add(face2Located, bbox2);
                Standard_Real xmin, ymin, zmin, xmax, ymax, zmax;
                bbox1.Get(xmin, ymin, zmin, xmax, ymax, zmax);
                gp_Pnt center1((xmin + xmax) / 2, (ymin + ymax) / 2, (zmin + zmax) / 2);
                bbox2.Get(xmin, ymin, zmin, xmax, ymax, zmax);
                gp_Pnt center2((xmin + xmax) / 2, (ymin + ymax) / 2, (zmin + zmax) / 2);
                // Get normal at center (project to surface)
                Standard_Real umin1, umax1, vmin1, vmax1;
                BRepTools::UVBounds(face1Located, umin1, umax1, vmin1, vmax1);
                Standard_Real ucenter1 = (umin1 + umax1) / 2.0;
                Standard_Real vcenter1 = (vmin1 + vmax1) / 2.0;
                Standard_Real umin2, umax2, vmin2, vmax2;
                BRepTools::UVBounds(face2Located, umin2, umax2, vmin2, vmax2);
                Standard_Real ucenter2 = (umin2 + umax2) / 2.0;
                Standard_Real vcenter2 = (vmin2 + vmax2) / 2.0;
                GeomLProp_SLProps props1(surf1, ucenter1, vcenter1, 1, 1e-6);
                GeomLProp_SLProps props2(surf2, ucenter2, vcenter2, 1, 1e-6);
                if (!props1.IsNormalDefined() || !props2.IsNormalDefined()) {
                    QMessageBox::warning(nullptr, "Align Faces", "Could not compute face normals.");
                    return;
                }
                gp_Dir normal1 = props1.Normal();
                gp_Dir normal2 = props2.Normal();
                // Compute rotation to align normal2 to normal1
                gp_Vec v1(normal1.X(), normal1.Y(), normal1.Z());
                gp_Vec v2(normal2.X(), normal2.Y(), normal2.Z());
                gp_Vec axis = v2.Crossed(v1);
                Standard_Real angle = v2.Angle(v1);
                gp_Trsf rotTrsf;
                if (axis.Magnitude() > 1e-8 && angle > 1e-8) {
                    gp_Ax1 rotationAxis(center2, gp_Dir(axis));
                    rotTrsf.SetRotation(rotationAxis, angle);
                } else {
                    rotTrsf = gp_Trsf();
                }
                // Apply rotation to center2 to get new center
                gp_Pnt center2Rot = center2;
                center2Rot.Transform(rotTrsf);
                // Compute translation to align centers
                gp_Vec translation(center1, center2Rot);
                gp_Trsf trsf;
                trsf.SetRotation(rotTrsf.GetRotation());
                trsf.SetTranslationPart(translation);
                // Apply transformation to parent2
                physicsAncestor->loc = trsf.Inverted() * physicsAncestor->loc.Transformation();
                qtModel->dataChanged(qtModel->indexForNode(physicsAncestor), qtModel->indexForNode(physicsAncestor));
                openGLViewer->markCacheDirty();
                QMessageBox::information(nullptr, "Align Faces", "Aligned parent of second face to first face.");
                if (physicsAncestor->needsGlobalLocUpdate) {
                    setGlobalLocRecursive(physicsAncestor, physicsAncestor->parent ? physicsAncestor->parent->globalLoc : TopLoc_Location());
                    std::function<void(CadNode*)> clearFlag = [&](CadNode* n) {
                        n->needsGlobalLocUpdate = false;
                        for (auto& child : n->children) clearFlag(child.get());
                    };
                    clearFlag(physicsAncestor);
                }
            });
        }
        // Add to Custom Model Tree submenu (only for CAD tree)
        if (name == "CAD") {
            QMenu* addToCustomMenu = menu.addMenu("Add to Custom Model...");
            // Find all destination trees (CustomModelTreeModel-based)
            struct CustomTreeInfo {
                QTreeView* tree;
                CustomModelTreeModel* model;
                QString tabName;
            };
            std::vector<CustomTreeInfo> customTrees;
            for (int i = 0; i < static_cast<int>(g_treeViews.size()); ++i) {
                QTreeView* tv = g_treeViews[i];
                if (tv->model() && tv->model()->metaObject()->className() == QString("CustomModelTreeModel")) {
                    QString tabName = QString("Custom Model Tree %1").arg(i+1);
                    customTrees.push_back({tv, static_cast<CustomModelTreeModel*>(tv->model()), tabName});
                }
            }
            QStringList segmentTypes = {"Carriage", "Start Segment", "Middle Segment", "End Segment"};
            // For each element type, add a submenu for destination trees
            for (const QString& segType : segmentTypes) {
                QMenu* typeMenu = addToCustomMenu->addMenu(segType);
                for (const auto& treeInfo : customTrees) {
                    QAction* addAction = typeMenu->addAction(treeInfo.tabName);
                    QObject::connect(addAction, &QAction::triggered, treeView, [=]() {
                        qDebug() << "[AddToCustomModel] Action triggered for segment type:" << segType << "to tree:" << treeInfo.tabName;
                        QTreeView* customTree = treeInfo.tree;
                        CustomModelTreeModel* customModel = treeInfo.model;
                        if (!customTree || !customModel) {
                            qDebug() << "[AddToCustomModel] Custom tree or model not found!";
                            return;
                        }
                        // Deep copy the selected node and its children
                        std::shared_ptr<CadNode> newNode = deepCopyNodeNonExcluded(node);
                        if (!newNode) {
                            qDebug() << "[AddToCustomModel] Deep copy failed!";
                            return;
                        }
                        newNode->name = segType.toStdString();
                        // Make the new node a Physics node
                        newNode->type = CadNodeType::Physics;
                        newNode->data = std::make_shared<PhysicsNodeData>();
                        qDebug() << "[AddToCustomModel] Copying node with name:" << QString::fromStdString(node->name) << "as" << segType;
                        // --- Preserve global transform ---
                        // Get parent-getter for source and destination
                        auto srcGetParent = [=](const CadNode* n) { return qtModel->getParentNode(n); };
                        auto dstGetParent = [=](const CadNode* n) { return customModel->getParentNode(n); };
                        // Compute global transform of source node
                        TopLoc_Location srcGlobal;
                        {
                            std::vector<const CadNode*> ancestry;
                            const CadNode* cur = node;
                            while (cur) {
                                ancestry.push_back(cur);
                                cur = srcGetParent(cur);
                            }
                            for (auto it = ancestry.rbegin(); it != ancestry.rend(); ++it) {
                                srcGlobal = srcGlobal * (*it)->loc;
                            }
                        }
                        // Compute global transform of destination parent (custom model root)
                        CadNode* customRoot = const_cast<CadNode*>(customModel->getRootNodePointer());
                        TopLoc_Location dstParentGlobal;
                        {
                            std::vector<const CadNode*> ancestry;
                            const CadNode* cur = customRoot;
                            while (cur) {
                                ancestry.push_back(cur);
                                cur = dstGetParent(cur);
                            }
                            for (auto it = ancestry.rbegin(); it != ancestry.rend(); ++it) {
                                dstParentGlobal = dstParentGlobal * (*it)->loc;
                            }
                        }
                        // Set the local transform of the new node so its global transform matches the original
                        newNode->loc = dstParentGlobal.Inverted() * srcGlobal;
                        // Now recursively fix children
                        for (size_t i = 0; i < node->children.size(); ++i) {
                            if (newNode->children.size() > i) {
                                adjustSubtreeTransforms(node->children[i].get(), newNode->children[i].get(), srcGlobal, srcGetParent);
                            }
                        }
                        // Add to the container's children (not the real root), with model reset for view update
                        customModel->addNodeWithReset(newNode);
                        openGLViewer->markCacheDirty();
                        int afterCount = static_cast<int>(customRoot->children.size());
                        // Select and scroll to the new node at the top level
                        if (customTree) {
                            QModelIndex newIdx = customModel->index(afterCount - 1, 0, QModelIndex());
                            if (newIdx.isValid()) {
                                customTree->setCurrentIndex(newIdx);
                                customTree->scrollTo(newIdx);
                            }
                        }
                        // --- Debug: Print global transforms ---
                        auto getGlobalTransform = [](const CadNode* node, std::function<const CadNode*(const CadNode*)> getParent) {
                            TopLoc_Location acc;
                            std::vector<const CadNode*> ancestry;
                            const CadNode* cur = node;
                            while (cur) {
                                ancestry.push_back(cur);
                                cur = getParent(cur);
                            }
                            for (auto it = ancestry.rbegin(); it != ancestry.rend(); ++it) {
                                acc = acc * (*it)->loc;
                            }
                            return acc;
                        };
                        TopLoc_Location srcGlobalDbg = getGlobalTransform(node, srcGetParent);
                        TopLoc_Location dstGlobalDbg = getGlobalTransform(newNode.get(), dstGetParent);
                        qDebug() << "[DEBUG] Source global transform:" << makeTransformString(srcGlobalDbg);
                        qDebug() << "[DEBUG] Dest global transform:" << makeTransformString(dstGlobalDbg);
                        if (qtModel && customModel) {
                            const CadNode* srcRoot = qtModel->getRootNodePointer();
                            const CadNode* dstRoot = customModel->getRootNodePointer();
                            qDebug() << "[DEBUG] Source tree root loc:" << makeTransformString(srcRoot ? srcRoot->loc : TopLoc_Location());
                            qDebug() << "[DEBUG] Dest tree root loc:" << makeTransformString(dstRoot ? dstRoot->loc : TopLoc_Location());
                        }
                    });
                }
            }
        }
        // Rail node: edit JSON properties
        if (node->type == CadNodeType::Rail) {
            QAction* editRailJsonAction = menu.addAction("Edit Rail Properties...");
            QObject::connect(editRailJsonAction, &QAction::triggered, treeView, [=]() {
                RailNodeData* railData = node->asRail();
                if (!railData) return;
                // Generate JSON string from current fields
                QJsonObject obj = railData->toJson();
                QJsonDocument doc(obj);
                QString jsonStr = QString::fromUtf8(doc.toJson(QJsonDocument::Indented));
                RailJsonEditorDialog dlg(jsonStr, treeView);
                if (dlg.exec() == QDialog::Accepted) {
                    QJsonDocument userDoc = QJsonDocument::fromJson(dlg.getJsonString().toUtf8());
                    if (!userDoc.isObject()) return;
                    QJsonObject newObj = userDoc.object();
                    // Replace the entire RailNodeData object from JSON
                    auto newData = RailNodeData::fromJson(newObj);
                    if (newData) {
                        *railData = *newData;
                        qtModel->dataChanged(idx, idx);
                    }
                }
            });
            // --- Expand Rail in Physics Preview ---
            QMenu* expandRailMenu = menu.addMenu("Expand Rail in Physics Preview...");
            // Build a list of all tree/3D view combos
            struct TreeViewComboInfo {
                QTreeView* tree;
                CustomModelTreeModel* model;
                CadOpenGLWidget* oglWidget;
                QString tabName;
            };
            for (int i = 0; i < static_cast<int>(g_treeViews.size()); ++i) {
                QTreeView* tree = g_treeViews[i];
                CustomModelTreeModel* model = dynamic_cast<CustomModelTreeModel*>(tree->model());
                CadOpenGLWidget* oglWidget = nullptr;
                if (i < static_cast<int>(g_openGLViews.size())) {
                    oglWidget = g_openGLViews[i];
                }
                QString tabName = QString("Tree %1 + 3D View %2").arg(i+1).arg(i+1);
                if (model && oglWidget) {
                    QAction* expandAction = expandRailMenu->addAction(tabName);
                    QObject::connect(expandAction, &QAction::triggered, treeView, [=]() {
                        CadNode* root = const_cast<CadNode*>(model->getRootNodePointer());
                        if (!root) {
                            QMessageBox::warning(nullptr, "Expand Rail", "Target tree root node not found.");
                            return;
                        }
                        expandRailInPhysicsPreview(node, std::shared_ptr<CadNode>(root, [](CadNode*){}), oglWidget);
                        QModelIndex rootIdx = model->indexForNode(root);
                        model->dataChanged(rootIdx, rootIdx);
                        // Do NOT expand the tree view
                    });
                }
            }
        }
        // --- Extrude Face and Add to Convex Hulls (for faces under any Physics ancestor) ---
        if (node->type == CadNodeType::XCAF && node->asXCAF() && node->asXCAF()->type == TopAbs_FACE) {
            // Traverse up to find the nearest Physics ancestor
            CadNode* ancestor = const_cast<CadNode*>(qtModel->getParentNode(node));
            while (ancestor && ancestor->type != CadNodeType::Physics) {
                ancestor = const_cast<CadNode*>(qtModel->getParentNode(ancestor));
            }
            if (ancestor && ancestor->type == CadNodeType::Physics && ancestor->asPhysics()) {
                QAction* extrudeAction = menu.addAction("Extrude Face and Add to Convex Hulls");
                QObject::connect(extrudeAction, &QAction::triggered, treeView, [=]() {
                    bool ok = false;
                    double distance = QInputDialog::getDouble(nullptr, "Extrude Face", "Extrusion distance (mm):", 10.0, -10000, 10000, 2, &ok);
                    if (!ok) return;
                    XCAFNodeData* xData = node->asXCAF();
                    if (!xData || !xData->hasFace()) {
                        QMessageBox::warning(nullptr, "Extrude Face", "Selected node is not a valid face.");
                        return;
                    }
                    TopoDS_Face face = xData->getFace();
                    // Extrude the face
                    TopoDS_Shape extruded = extrudeFace(face, distance);
                    // Mesh the extruded shape
                    BRepMesh_IncrementalMesh mesher(extruded, 0.5);
                    // Collect vertices and triangles
                    std::vector<std::array<double, 3>> vertices;
                    std::vector<std::array<uint32_t, 3>> indices;
                    uint32_t vertOffset = 0;
                    for (TopExp_Explorer exp(extruded, TopAbs_FACE); exp.More(); exp.Next()) {
                        TopoDS_Face f = TopoDS::Face(exp.Current());
                        TopLoc_Location loc;
                        Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(f, loc);
                        if (tri.IsNull() || tri->NbTriangles() == 0) continue;
                        std::vector<uint32_t> localIndices(tri->NbNodes());
                        for (int i = 1; i <= tri->NbNodes(); ++i) {
                            gp_Pnt p = tri->Node(i);
                            p.Transform(loc.Transformation());
                            vertices.push_back({p.X(), p.Y(), p.Z()});
                            localIndices[i-1] = vertOffset++;
                        }
                        for (int i = tri->Triangles().Lower(); i <= tri->Triangles().Upper(); ++i) {
                            int n1, n2, n3;
                            tri->Triangles()(i).Get(n1, n2, n3);
                            indices.push_back({localIndices[n1-1], localIndices[n2-1], localIndices[n3-1]});
                        }
                    }
                    if (vertices.empty() || indices.empty()) {
                        QMessageBox::warning(nullptr, "Extrude Face", "Failed to mesh the extruded geometry.");
                        return;
                    }
                    // Add to Physics ancestor's PhysicsNodeData hulls
                    PhysicsNodeData* physData = ancestor->asPhysics();
                    ConvexHullData hullData;
                    hullData.vertices = std::move(vertices);
                    hullData.indices = std::move(indices);
                    physData->hulls.push_back(std::move(hullData));
                    physData->convexHullGenerated = true;
                    qtModel->dataChanged(qtModel->indexForNode(ancestor), qtModel->indexForNode(ancestor));
                    openGLViewer->markCacheDirty();
                    QMessageBox::information(nullptr, "Extrude Face", "Extruded geometry added to convex hulls.");
                });
            }
        }
        if (!menu.isEmpty()) menu.exec(treeView->viewport()->mapToGlobal(pos));
    });
    // If inputRoot is a container, set OpenGL root to its first child
    CadNode* oglRoot = inputRoot->children.empty() ? nullptr : inputRoot->children[0].get();
    openGLViewer->setRootTreeNode(oglRoot);
    treeTabWidget->addTab(treeView, QString::fromStdString(name) + " Tree");
    openGLTabWidget->addTab(openGLViewer, QString::fromStdString(name) + " Preview");
    connectTreeAndViewer(treeView, openGLViewer, qtModel);
    // Store pointers for tab switching
    g_treeViews.push_back(treeView);
    g_openGLViews.push_back(openGLViewer);
}


int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    QMainWindow mainWindow;
    mainWindow.setWindowTitle("QtCadViewer - OCC C++");

    // Add SimulationManager GUI elements
    simManager.addGuiElements(&mainWindow);

    // Prompt for STEP or Rail Project file on startup
    QString openFile = QFileDialog::getOpenFileName(
        nullptr, "Open STEP or Rail Project File", "", "STEP or Rail Project (*.step *.stp *.json)");
    if (openFile.isEmpty()) {
        QMessageBox::warning(nullptr, "No File", "No file selected. Exiting.");
        return 0;
    }

    //Gui code
    // Central widget and splitter
    // Outer splitter: custom model tree (left) and CAD/XCAF tabs (right)
    QSplitter *outerSplitter = new QSplitter(Qt::Horizontal);
    // Inner splitter: tab widget and OpenGL viewer
    QSplitter *splitter = new QSplitter(Qt::Horizontal);

    // Layout: buttons above splitter
    QWidget *centralWidget = new QWidget;
    QVBoxLayout *vLayout = new QVBoxLayout;
    QHBoxLayout *buttonLayout = new QHBoxLayout;
    buttonLayout->addStretch();
    vLayout->addLayout(buttonLayout);
    vLayout->addWidget(outerSplitter);
    vLayout->setContentsMargins(0,0,0,0);
    centralWidget->setLayout(vLayout);
    mainWindow.setCentralWidget(centralWidget);

    // Add selection mode buttons
    QPushButton* noSelectionBtn = new QPushButton("No Selection");
    QPushButton* faceSelectionBtn = new QPushButton("Face Selection");
    QPushButton* edgeSelectionBtn = new QPushButton("Edge Selection");

    // Style the buttons to show current selection mode
    noSelectionBtn->setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; }");

    buttonLayout->addWidget(noSelectionBtn);
    buttonLayout->addWidget(faceSelectionBtn);
    buttonLayout->addWidget(edgeSelectionBtn);

    // --- CAD/XCAF Tabs and OpenGL Viewer ---
    outerSplitter->addWidget(splitter);

    // Create tab widget for the left side
    QTabWidget *tabWidget = new QTabWidget;
    tabWidget->setMinimumWidth(300);
    splitter->addWidget(tabWidget);


    // --- 3D View Tabs ---
    QTabWidget* viewTabWidget = new QTabWidget;
    splitter->addWidget(viewTabWidget);

    Handle(TDocStd_Document) doc;
    std::shared_ptr<CadNode> cadRootShared;
    std::unique_ptr<XCAFLabelNode> labelRoot;
    Handle(XCAFDoc_ShapeTool) shapeTool;
    Handle(XCAFDoc_ColorTool) colorTool;
    std::shared_ptr<CadNode> customModelRootContainer;
    std::shared_ptr<CadNode> customModelRoot;
    bool loadedFromJsonBin = false;

    if (openFile.endsWith(".json", Qt::CaseInsensitive)) {
        if (!loadFromJsonAndBin(openFile, doc, cadRootShared, labelRoot, shapeTool, colorTool, customModelRootContainer, customModelRoot, loadedFromJsonBin)) {
            return 1;
        }
    } else {
        if (!loadFromStep(openFile, doc, cadRootShared, labelRoot, shapeTool, colorTool)) {
            return 1;
        }
        // --- Custom Model Tree ---
        if (!customModelRootContainer) customModelRootContainer = std::make_shared<CadNode>();
        if (!customModelRoot) customModelRoot = std::make_shared<CadNode>();
        customModelRoot->name = "Custom Rail";
        customModelRoot->visible = true;
        customModelRoot->type = CadNodeType::Rail;
        customModelRoot->data = std::make_shared<RailNodeData>();
        customModelRootContainer->children.clear();
        customModelRootContainer->children.push_back(customModelRoot);
    }
    CadTreeModel* cadTreeModel = nullptr;
    initTreeAndOpenGLWidget(cadRootShared, tabWidget, viewTabWidget, "CAD", doc, cadTreeModel);
    // For the custom model tree, use the container as the root for the tree view, but set the OpenGL widget's root to the real root (first child of the container)
    initTreeAndOpenGLWidget(customModelRootContainer, tabWidget, viewTabWidget, "Custom Model", doc, nullptr);

    CadOpenGLWidget* active3DView = nullptr;
    QTreeView* activeTreeView = nullptr;
    // Set initial active widgets
    if (!g_openGLViews.empty()) active3DView = g_openGLViews[0];
    if (!g_treeViews.empty()) activeTreeView = g_treeViews[0];
    // Connect tab change signals to update active widgets
    QObject::connect(viewTabWidget, &QTabWidget::currentChanged, [&](int idx) {
        if (idx >= 0 && idx < (int)g_openGLViews.size()) {
            active3DView = g_openGLViews[idx];
        }
    });
    QObject::connect(tabWidget, &QTabWidget::currentChanged, [&](int idx) {
        if (idx >= 0 && idx < (int)g_treeViews.size()) {
            activeTreeView = g_treeViews[idx];
        }
    });

    // Connect selection mode buttons
    QObject::connect(noSelectionBtn, &QPushButton::clicked, [&]() {
        if (active3DView) active3DView->setSelectionMode(SelectionMode::None);
        noSelectionBtn->setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; }");
        faceSelectionBtn->setStyleSheet("QPushButton { background-color: #f0f0f0; color: black; }");
        edgeSelectionBtn->setStyleSheet("QPushButton { background-color: #f0f0f0; color: black; }");
    });

    QObject::connect(faceSelectionBtn, &QPushButton::clicked, [&]() {
        if (active3DView) active3DView->setSelectionMode(SelectionMode::Faces);
        noSelectionBtn->setStyleSheet("QPushButton { background-color: #f0f0f0; color: black; }");
        faceSelectionBtn->setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; }");
        edgeSelectionBtn->setStyleSheet("QPushButton { background-color: #f0f0f0; color: black; }");
    });

    QObject::connect(edgeSelectionBtn, &QPushButton::clicked, [&]() {
        if (active3DView) active3DView->setSelectionMode(SelectionMode::Edges);
        noSelectionBtn->setStyleSheet("QPushButton { background-color: #f0f0f0; color: black; }");
        faceSelectionBtn->setStyleSheet("QPushButton { background-color: #f0f0f0; color: black; }");
        edgeSelectionBtn->setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; }");
    });


    // --- Physics Preview Tree and OpenGL Widget ---
    auto physicsPreviewRoot = std::make_shared<CadNode>();
    initTreeAndOpenGLWidget(physicsPreviewRoot,tabWidget,viewTabWidget,"Physics", doc);

    // Add Reframe button
    QPushButton* reframeBtn = new QPushButton("Reframe");
    buttonLayout->addWidget(reframeBtn);
    QObject::connect(reframeBtn, &QPushButton::clicked, [=]() {
        int idx = viewTabWidget->currentIndex();
        if (idx >= 0 && idx < static_cast<int>(g_openGLViews.size())) {
            CadOpenGLWidget* active3DView = g_openGLViews[idx];
            if (active3DView) active3DView->reframeCamera();
        }
    });

    // Add serialization buttons
    QPushButton* saveStepBtn = new QPushButton("Save STEP");
    QPushButton* saveBinaryBtn = new QPushButton("Save Binary");

    buttonLayout->addWidget(saveStepBtn);
    buttonLayout->addWidget(saveBinaryBtn);

    // Connect serialization buttons
    QObject::connect(saveStepBtn, &QPushButton::clicked, [=]() {
        QString baseName = QFileInfo(openFile).baseName();
        QString outputPath = QDir::currentPath() + QDir::separator() + baseName + "_serialized.step";
        if (saveXCAFToSTEP(doc, outputPath)) {
            QMessageBox::information(nullptr, "Success", "XCAF data saved to STEP file:\n" + outputPath);
        } else {
            QMessageBox::warning(nullptr, "Error", "Failed to save XCAF data to STEP file");
        }
    });

    QObject::connect(saveBinaryBtn, &QPushButton::clicked, [=]() {
        QString baseName = QFileInfo(openFile).baseName();
        QString outputPath = QDir::currentPath() + QDir::separator() + baseName + "_serialized.bin";
        if (saveXCAFToBinary(doc, outputPath)) {
            QMessageBox::information(nullptr, "Success", "XCAF data saved to binary file:\n" + outputPath);
        } else {
            QMessageBox::warning(nullptr, "Error", "Failed to save XCAF data to binary file");
        }
    });

    // --- XCAF Label Tree View for second tab ---
    // Build comprehensive label tree showing all XCAF structure
    labelRoot = std::make_unique<XCAFLabelNode>(doc->Main());
    labelRoot->label = doc->Main();

    qDebug() << "Building comprehensive XCAF tree...";

    // Add all direct children of the main document
    for (TDF_ChildIterator it(doc->Main()); it.More(); it.Next()) {
        TDF_Label childLabel = it.Value();
        qDebug() << "Adding main document child with tag:" << childLabel.Tag();
        labelRoot->children.push_back(buildLabelTreeWithReferences(childLabel, shapeTool));
    }

    // Also add all free shapes (important for STEP 214)
    TDF_LabelSequence freeShapes;
    shapeTool->GetFreeShapes(freeShapes);
    qDebug() << "Found" << freeShapes.Length() << "free shapes";

    for (Standard_Integer i = 1; i <= freeShapes.Length(); ++i) {
        TDF_Label freeShapeLabel = freeShapes.Value(i);
        qDebug() << "Adding free shape with tag:" << freeShapeLabel.Tag();

        // Check if this free shape is already in the tree
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

    // --- Removed special node for all shapes ---

    qDebug() << "XCAF tree built with" << labelRoot->children.size() << "root children";

    XCAFLabelTreeModel* labelModel = new XCAFLabelTreeModel(std::move(labelRoot));

    // Pass tools to the model for enhanced information display
    labelModel->setShapeTool(shapeTool);
    labelModel->setColorTool(colorTool);

    QTreeView* labelTreeView = new QTreeView;
    labelTreeView->setModel(labelModel);
    labelTreeView->setHeaderHidden(false);
    labelTreeView->setSelectionMode(QAbstractItemView::ExtendedSelection); // Allow multi-selection and clear selection by clicking below
    // --- Context menu support for XCAF label tree ---
    labelTreeView->setContextMenuPolicy(Qt::CustomContextMenu);
    QObject::connect(labelTreeView, &QTreeView::customContextMenuRequested, labelTreeView, [=](const QPoint& pos) {
        QModelIndex idx = labelTreeView->indexAt(pos);
        if (!idx.isValid()) return;
        XCAFLabelNode* node = labelModel->getNode(idx);
        if (!node) return;
        QMenu menu;
        QAction* infoAction = menu.addAction(QString("Label Tag: %1").arg(node->label.Tag()));
        infoAction->setEnabled(false);
        if (!menu.isEmpty()) menu.exec(labelTreeView->viewport()->mapToGlobal(pos));
    });
    tabWidget->addTab(labelTreeView, "XCAF Tree");

    // Insert the buttons above the custom model tree in the layout
    QVBoxLayout* customModelLayout = new QVBoxLayout;
    QWidget* customModelWidget = new QWidget;
    customModelWidget->setLayout(customModelLayout);
    outerSplitter->insertWidget(0, customModelWidget);

    mainWindow.resize(1200, 800);
    mainWindow.show();

    return app.exec();
}


