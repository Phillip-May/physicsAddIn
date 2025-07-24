#define ENABLE_VHACD_IMPLEMENTATION 1
#include "../physicsAddIn/v-hacd-4.1.0/include/VHACD.h"
#include "../external/CoACD/public/coacd.h"
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

// Helper to recursively build XCAFLabelNode tree with sub-assembly detection
std::unique_ptr<XCAFLabelNode> buildLabelTree(const TDF_Label& label, const Handle(XCAFDoc_ShapeTool)& shapeTool = nullptr) {
    auto node = std::make_unique<XCAFLabelNode>(label);
    
    // Check if this is a compound that might contain sub-assemblies
    bool isCompoundWithSubAssemblies = false;
    if (!shapeTool.IsNull() && !label.IsNull()) {
        TopoDS_Shape shape = shapeTool->GetShape(label);
        if (!shape.IsNull() && shape.ShapeType() == TopAbs_COMPOUND) {
            // Count direct shape children to detect sub-assemblies
            int shapeChildCount = 0;
            for (TDF_ChildIterator it(label); it.More(); it.Next()) {
                TDF_Label childLabel = it.Value();
                if (!childLabel.IsNull()) {
                    TopoDS_Shape childShape = shapeTool->GetShape(childLabel);
                    if (!childShape.IsNull()) {
                        shapeChildCount++;
                        if (shapeChildCount > 1) {
                            isCompoundWithSubAssemblies = true;
                            break;
                        }
                    }
                }
            }
        }
    }
    
    // Recursively build children
    for (TDF_ChildIterator it(label); it.More(); it.Next()) {
        TDF_Label childLabel = it.Value();
        if (!childLabel.IsNull()) {
            auto childNode = buildLabelTree(childLabel, shapeTool);
            node->children.push_back(std::move(childNode));
        }
    }
    
    return node;
}

// Enhanced tree building that follows reference chains for STEP 214 assemblies
std::unique_ptr<XCAFLabelNode> buildLabelTreeWithReferences(const TDF_Label& label, const Handle(XCAFDoc_ShapeTool)& shapeTool = nullptr) {
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
inline uint qHash(const TDF_Label& label, uint seed = 0) {
    return qHash(label.Tag(), seed);
}

// Helper to convert TopAbs_ShapeEnum to string
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

// Helper to extract the most specific color for a face using XCAF color hierarchy
CADNodeColor getEffectiveFaceColor(const TopoDS_Face& face,
                             const Handle(XCAFDoc_ShapeTool)& shapeTool,
                             const Handle(XCAFDoc_ColorTool)& colorTool) {
    // 1. Find the label for this face
    TDF_Label faceLabel;
    Quantity_Color occColor;
    if (shapeTool->Search(face, faceLabel)) {
        // 1a. Face color
        if (colorTool->GetColor(faceLabel, XCAFDoc_ColorSurf, occColor) ||
            colorTool->GetColor(faceLabel, XCAFDoc_ColorCurv, occColor) ||
            colorTool->GetColor(faceLabel, XCAFDoc_ColorGen, occColor)) {
            return CADNodeColor(occColor.Red(), occColor.Green(), occColor.Blue());
        }
        // 1b. Traverse up the label tree for parent/component/assembly color
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
    // 2. Default color
    return CADNodeColor::fromSRGB(200, 200, 200);
}







// Enhanced color extraction that traverses up the label hierarchy
CADNodeColor get_label_color(const TDF_Label& label, const Handle(XCAFDoc_ColorTool)& colorTool, const CADNodeColor& parentColor) {
    Quantity_Color occColor;
    
    // First try to get color from the current label
    if (colorTool->GetColor(label, XCAFDoc_ColorSurf, occColor) ||
        colorTool->GetColor(label, XCAFDoc_ColorGen, occColor) ||
        colorTool->GetColor(label, XCAFDoc_ColorCurv, occColor)) {
        return CADNodeColor(occColor.Red(), occColor.Green(), occColor.Blue());
    }
    
    // If no color found, traverse up the label tree to find inherited color
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
    
    // If still no color found, return the provided parent color
    return parentColor;
}

// Enhanced color extraction for specific shape types
CADNodeColor get_shape_color(const TopoDS_Shape& shape, 
                           const TDF_Label& label,
                           const Handle(XCAFDoc_ShapeTool)& shapeTool,
                           const Handle(XCAFDoc_ColorTool)& colorTool,
                           const CADNodeColor& parentColor) {
    if (shape.IsNull()) {
        return get_label_color(label, colorTool, parentColor);
    }
    
    // For faces, use the sophisticated face color extraction
    if (shape.ShapeType() == TopAbs_FACE) {
        TopoDS_Face face = TopoDS::Face(shape);
        return getEffectiveFaceColor(face, shapeTool, colorTool);
    }
    
    // For other shapes, use the enhanced label color extraction
    return get_label_color(label, colorTool, parentColor);
}

// Helper: recursively check if a label or any of its descendants is a sub-assembly (compound)
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

// Helper to format a transform as a string for node names
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

// Helper to check if a label represents an assembly (compound with multiple children)
bool isAssembly(const TDF_Label& label, const Handle(XCAFDoc_ShapeTool)& shapeTool) {
    TopoDS_Shape shape = shapeTool->GetShape(label);
    if (shape.IsNull() || shape.ShapeType() != TopAbs_COMPOUND) {
        return false;
    }
    
    // Check if this compound has multiple direct shape children
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

// Helper to get the effective transform for a label, considering STEP 214 assembly structure
TopLoc_Location getEffectiveTransform(const TDF_Label& label, 
                                     const Handle(XCAFDoc_ShapeTool)& shapeTool,
                                     const TopLoc_Location& parentLoc) {
    TopLoc_Location effectiveLoc = parentLoc;
    
    // Check if this is a reference
    if (shapeTool->IsReference(label)) {
        effectiveLoc = parentLoc * shapeTool->GetLocation(label);
    }
    
    // For STEP 214, also check if there's a product definition context
    // that might affect the transform
    Handle(TDataStd_TreeNode) treeNode;
    if (label.FindAttribute(TDataStd_TreeNode::GetDefaultTreeID(), treeNode)) {
        // Handle product definition context if present
        // This is specific to STEP 214 assembly structure
    }
    
    return effectiveLoc;
}

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
                   const TopLoc_Location& parentLoc)
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
            auto uniqueChild = build_tree_xcaf(refLabel, shapeTool, colorTool, refNode->color, TopLoc_Location());
            if (uniqueChild) {
                refNode->children.push_back(uniqueChild);
            }
        }
        return refNode;
    }
    CADNodeColor color = get_shape_color(shape, label, shapeTool, colorTool, parentColor);
    // Debug color propagation for important nodes
    if (!shape.IsNull() && (shape.ShapeType() == TopAbs_SOLID || shape.ShapeType() == TopAbs_COMPOUND)) {
        qDebug() << "Label" << label.Tag() << "color: RGB(" << color.r << "," << color.g << "," << color.b << ")";
        if (color.r == parentColor.r && color.g == parentColor.g && color.b == parentColor.b) {
            qDebug() << "  -> Inherited from parent";
        } else {
            qDebug() << "  -> Own color";
        }
    }
    // Skip edge shapes entirely
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
    // Get effective transform for this node
    TopLoc_Location nodeLoc = getEffectiveTransform(label, shapeTool, parentLoc);
    auto node = std::make_shared<CadNode>();
    node->color = color;
    node->loc = nodeLoc;
    node->type = CadNodeType::XCAF;
    auto xData = std::make_shared<XCAFNodeData>();
    xData->shape = shape;
    xData->type = !shape.IsNull() ? shape.ShapeType() : TopAbs_SHAPE;
    xData->xcafLabelTag = label.Tag();
    xData->originalXCAFShape = shape;
    node->data = xData;
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
            auto child = build_tree_xcaf(refLabel, shapeTool, colorTool, color, refLoc);
            if (child) {
                // Mark this as a reference node in the name
                child->name = QString("REF->%1 | %2").arg(refLabel.Tag()).arg(QString::fromStdString(child->name)).toStdString();
                node->children.push_back(child);
            }
        }
    }
    // Recurse into children (for all labels)
    for (TDF_ChildIterator it(label); it.More(); it.Next()) {
        auto child = build_tree_xcaf(it.Value(), shapeTool, colorTool, color, nodeLoc);
        if (child) node->children.push_back(child);
    }
    // After node is created and type is set
    if (!shape.IsNull()) {
        xData->shape = shape;
        xData->type = shape.ShapeType();
    }
    // Add face/edge children for solids that don't have other solid children
    // This allows faces/edges to be shown even when the solid has reference children
    if (!shape.IsNull() && shape.ShapeType() == TopAbs_SOLID && !isAssembly(label, shapeTool)) {
        // Check if any existing children are solids (to avoid intermediate solids)
        bool hasSolidChildren = false;
        for (const auto& child : node->children) {
            auto childXCAF = child->asXCAF();
            if (childXCAF && childXCAF->type == TopAbs_SOLID) {
                hasSolidChildren = true;
                break;
            }
        }
        // Add faces and edges if this solid doesn't have other solid children
        if (!hasSolidChildren) {
            int faceIdx = 0;
            int faceCount = 0;
            for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next(), ++faceIdx) {
                TopoDS_Face face = TopoDS::Face(exp.Current());
                auto faceNode = std::make_shared<CadNode>();
                faceNode->type = CadNodeType::XCAF;
                auto faceData = std::make_shared<XCAFNodeData>();
                faceData->shape = face;
                faceData->type = TopAbs_FACE;
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
                edgeNode->data = edgeData;
                edgeNode->color = color;
                edgeNode->loc = nodeLoc;
                edgeNode->name = QString("Edge %1 of %2").arg(edgeIdx).arg(QString::fromStdString(node->name)).toStdString();
                node->children.push_back(edgeNode);
                edgeCount++;
            }
            qDebug() << "Added" << faceCount << "faces and" << edgeCount << "edges to solid at label" << label.Tag();
        }
    }
    return node;
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
        // Helper to get parent node from model
        auto getParent = [=](CadNode* n) -> CadNode* {
            return const_cast<CadNode*>(model->getParentNode(n));
        };
        for (CadNode* node : allNodesToSelect) {
            TopLoc_Location accLoc = accumulateTransform(node, getParent);
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


// Helper struct for face and accumulated transform
struct FaceWithTransform {
    CadNode* node;
    TopLoc_Location accumulatedLoc;
};

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

static void generateVHACDStub(const QString& nodeName, int resolution, int maxHulls, double minVolume, CadNode* node) {
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
        // Walk up the tree to accumulate the global transform
        CadNode* cur = node;
        std::vector<CadNode*> ancestry;
        while (cur) {
            ancestry.push_back(cur);
            // No parent pointer, so break (assume node is root or passed as such)
            // If you have a getParentNode, use it here
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
        // Compute relative transform from Physics node to face
        TopLoc_Location relLoc = physicsGlobalLoc.Inverted() * faceGlobalLoc;
        XCAFNodeData* xData = faceNode->asXCAF();
        if (!xData || !xData->hasFace()) continue;
        TopoDS_Face face = xData->getFace();
        BRepMesh_IncrementalMesh mesher(face, 0.5);
        TopLoc_Location locCopy;
        Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, locCopy);
        if (tri.IsNull() || tri->NbTriangles() == 0) continue;
        // Add vertices
        std::vector<uint32_t> localIndices(tri->NbNodes());
        for (int i = 1; i <= tri->NbNodes(); ++i) {
            gp_Pnt p = tri->Node(i);
            // Transform to Physics node local space
            p.Transform(relLoc.Transformation());
            vertices.push_back(VHACD::Vertex(p.X(), p.Y(), p.Z()));
            localIndices[i-1] = vertOffset++;
        }
        // Add triangles
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
            // Check for duplicate by TShape pointer
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
    collectSolids(node); // node is the Physics node
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
    // Try to get a better volume estimate if all faces belong to a single solid
    // (Optional: could be improved by traversing up to the solid node and using BRepGProp::VolumeProperties)
    qDebug() << "[VHACD] Approximated original mesh area (sum of face areas):" << originalVolume;
    // Run VHACD
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
            // Use m_points and m_triangles directly
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

static void generateCoACDStub(const QString& nodeName, double concavity, double alpha, double beta, CadNode* node,
    int maxConvexHull, std::string preprocess, int prepRes, int sampleRes, int mctsNodes, int mctsIter, int mctsDepth,
    bool pca, bool merge, bool decimate, int maxChVertex, bool extrude, double extrudeMargin, std::string apxMode, int seed) {
    std::vector<FaceWithTransform> faces;
    qDebug() << "Node Name: " << node->name.c_str();
    collectFaceNodesWithTransform(node, TopLoc_Location(), faces);
    // Filter out excluded nodes
    faces.erase(std::remove_if(faces.begin(), faces.end(), [](const FaceWithTransform& f) { return f.node->excludedFromDecomposition; }), faces.end());
    qDebug() << "Face count: " << faces.size();
    int numInputFaces = static_cast<int>(faces.size());
    std::vector<double> vertices;
    std::vector<int> triangles;
    uint32_t vertOffset = 0;
    // Compute global transform for the Physics node
    TopLoc_Location physicsGlobalLoc = TopLoc_Location();
    {
        CadNode* cur = node;
        std::vector<CadNode*> ancestry;
        while (cur) {
            ancestry.push_back(cur);
            // No parent pointer, so break
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
            // Transform to Physics node local space
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
    // --- Compute original mesh volume (unchanged) ---
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
    // Convert our data to CoACD Mesh format
    coacd::Mesh inputMesh;
    for (size_t i = 0; i < vertices.size(); i += 3) {
        inputMesh.vertices.push_back({vertices[i], vertices[i+1], vertices[i+2]});
    }
    for (size_t i = 0; i < triangles.size(); i += 3) {
        inputMesh.indices.push_back({triangles[i], triangles[i+1], triangles[i+2]});
    }
    // Run CoACD with all parameters
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

// Helper: Deep copy a CadNode and all non-excluded children
std::shared_ptr<CadNode> deepCopyNodeNonExcluded(const CadNode* src) {
    if (!src || src->excludedFromDecomposition) return nullptr;
    auto copy = std::make_shared<CadNode>(*src);
    if (copy->asXCAF() && src->asXCAF()) copy->asXCAF()->xcafLabelTag = src->asXCAF()->xcafLabelTag; // Explicitly copy label tag for XCAF nodes
    copy->children.clear();
    for (const auto& child : src->children) {
        auto childCopy = deepCopyNodeNonExcluded(child.get());
        if (childCopy) copy->children.push_back(childCopy);
    }
    return copy;
}

// Helper: Insert a node into the custom model tree at the same position as the selected node(s) in the CAD tree
void insertCustomModelNodeAtCadTreePosition(CadNode* customModelRoot, std::shared_ptr<CadNode> newNode, const std::vector<CadNode*>& selectedCadNodes, CadTreeModel* cadModel) {
    // Try to find the index of the first selected node in the CAD tree among its siblings
    int insertIdx = -1;
    if (!selectedCadNodes.empty()) {
        CadNode* refNode = selectedCadNodes.front();
        // Find the parent of the reference node in the CAD tree
        const CadNode* parent = cadModel->getParentNode(refNode);
        if (parent) {
            // Find the index of the reference node among its siblings
            for (size_t i = 0; i < parent->children.size(); ++i) {
                if (parent->children[i].get() == refNode) {
                    insertIdx = static_cast<int>(i);
                    break;
                }
            }
        }
    }
    // If we found a valid index, insert at that position; otherwise, append
    if (insertIdx >= 0 && insertIdx <= static_cast<int>(customModelRoot->children.size())) {
        customModelRoot->children.insert(customModelRoot->children.begin() + insertIdx, newNode);
    } else {
        customModelRoot->children.push_back(newNode);
    }
}

// Helper: Recursively adjust all descendants' transforms so their global transform matches the original
void adjustSubtreeTransforms(const CadNode* src, CadNode* copy, const TopLoc_Location& baseLoc, std::function<const CadNode*(const CadNode*)> getParent) {
    // Compute original global transform of src
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
    // Recurse for children
    for (size_t i = 0; i < src->children.size(); ++i) {
        if (copy->children.size() > i) {
            adjustSubtreeTransforms(src->children[i].get(), copy->children[i].get(), baseLoc, getParent);
        }
    }
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

// Function to add a Rail to the physics simulation with tiling
void addRailToPhysicsPreview(CadNode* railNode, std::shared_ptr<CadNode> physicsPreviewRoot) {
    if (!railNode || !physicsPreviewRoot) return;
    // Parse JSON for numSegments and axisOfTravel
    RailNodeData* railData = railNode->asRail();
    int numSegments = 1;
    QVector3D axis(1,0,0);
    if (railData && !railData->jsonString.isEmpty()) {
        QJsonDocument doc = QJsonDocument::fromJson(railData->jsonString.toUtf8());
        if (doc.isObject()) {
            QJsonObject obj = doc.object();
            if (obj.contains("numSegments")) numSegments = obj["numSegments"].toInt(1);
            if (obj.contains("axisOfTravel")) {
                QJsonArray arr = obj["axisOfTravel"].toArray();
                if (arr.size() == 3) axis = QVector3D(arr[0].toDouble(), arr[1].toDouble(), arr[2].toDouble());
            }
        }
    }
    // Find carriage, start, middle, end segments
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
    // Compute the length of the middle segment along the axis
    double segLength = computeBoundingBoxLength(middleSeg, axis);
    if (segLength < 1e-6) segLength = 1000.0; // Fallback if bbox fails
    // Clear previous children
    physicsPreviewRoot->children.clear();
    // Deep copy the rail node as a container
    auto railCopy = std::make_shared<CadNode>(*railNode);
    railCopy->children.clear();
    QVector3D currentOffset(0,0,0);
    // Add carriage if present
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
    // Add start segment if present
    if (startSeg) {
        auto startCopy = deepCopyNodeNonExcluded(startSeg);
        if (startCopy) {
            gp_Trsf offsetTrsf;
            offsetTrsf.SetTranslationPart(gp_XYZ(currentOffset.x(), currentOffset.y(), currentOffset.z()));
            gp_Trsf finalTrsf = startSeg->loc.Transformation() * offsetTrsf;
            startCopy->loc = TopLoc_Location(finalTrsf);
            railCopy->children.push_back(startCopy);
            // Do NOT increment currentOffset here
        }
    }
    // Add tiled middle segments
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
    // Add end segment if present
    if (endSeg) {
        currentOffset -= axis * segLength; // Move back by one segment
        auto endCopy = deepCopyNodeNonExcluded(endSeg);
        if (endCopy) {
            gp_Trsf offsetTrsf;
            offsetTrsf.SetTranslationPart(gp_XYZ(currentOffset.x(), currentOffset.y(), currentOffset.z()));
            gp_Trsf finalTrsf = endSeg->loc.Transformation() * offsetTrsf;
            endCopy->loc = TopLoc_Location(finalTrsf);
            railCopy->children.push_back(endCopy);
        }
    }
    // Add the new rail as the only child of the physics preview root
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

void relinkCadNodeXCAFGeometry(std::shared_ptr<CadNode>& node, const Handle(TDocStd_Document)& doc) {
    Handle(XCAFDoc_ShapeTool) shapeTool = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
    Handle(XCAFDoc_ColorTool) colorTool = XCAFDoc_DocumentTool::ColorTool(doc->Main());
    TDF_Label label;
    bool found = false;
    if ((node->asXCAF() ? node->asXCAF()->xcafLabelTag : -1) >= 0) {
        TDF_Tool::Label(doc->GetData(), node->asXCAF()->xcafLabelTag, label);
        if (!label.IsNull()) {
            TopoDS_Shape shape = shapeTool->GetShape(label);
            if (!shape.IsNull()) {
                auto xData = std::make_shared<XCAFNodeData>();
                xData->shape = shape;
                xData->type = shape.ShapeType();
                xData->xcafLabelTag = label.Tag();
                xData->originalXCAFShape = shape;
                node->data = xData;
                node->type = CadNodeType::XCAF;
                found = true;
                // Set color using get_shape_color
                CADNodeColor parentColor = CADNodeColor::fromSRGB(200, 200, 200); // fallback
                node->color = get_shape_color(shape, label, shapeTool, colorTool, parentColor);
                // Set name as in build_tree_xcaf
                QString typeName = shapeTypeToString(xData->type);
                QString occName;
                Handle(TDataStd_Name) nameAttr;
                if (label.FindAttribute(TDataStd_Name::GetID(), nameAttr) && !nameAttr.IsNull()) {
                    occName = QString::fromStdWString(nameAttr->Get().ToWideString());
                } else {
                    occName = QString("Label %1").arg(label.Tag());
                }
                QString transformStr = makeTransformString(node->loc);
                node->name = QString("Label %1 | Type: %2 | Transform: %3%4%5")
                    .arg(label.Tag())
                    .arg(typeName)
                    .arg(transformStr)
                    .arg(occName.isEmpty() ? "" : QString(" | OCC Name: %1").arg(occName))
                    .arg("")
                    .toStdString();
            }
        }
    }
    // If not found, and node->data is a face/edge, search for the label by shape pointer
    XCAFNodeData* xData = dynamic_cast<XCAFNodeData*>(node->data.get());
    if (!found && xData && (xData->type == TopAbs_FACE || xData->type == TopAbs_EDGE)) {
        TopoDS_Shape targetShape = xData->shape;
        TDF_Label foundLabel = findLabelForShape(shapeTool, doc->Main(), targetShape);
        if (!foundLabel.IsNull()) {
            TopoDS_Shape shape = shapeTool->GetShape(foundLabel);
            if (!shape.IsNull()) {
                auto newXData = std::make_shared<XCAFNodeData>();
                newXData->shape = shape;
                newXData->type = shape.ShapeType();
                newXData->xcafLabelTag = foundLabel.Tag();
                newXData->originalXCAFShape = shape;
                node->data = newXData;
                node->type = CadNodeType::XCAF;
                // Set color using get_shape_color
                CADNodeColor parentColor = CADNodeColor::fromSRGB(200, 200, 200); // fallback
                node->color = get_shape_color(shape, foundLabel, shapeTool, colorTool, parentColor);
                // Set name as in build_tree_xcaf
                QString typeName = shapeTypeToString(newXData->type);
                QString occName;
                Handle(TDataStd_Name) nameAttr;
                if (foundLabel.FindAttribute(TDataStd_Name::GetID(), nameAttr) && !nameAttr.IsNull()) {
                    occName = QString::fromStdWString(nameAttr->Get().ToWideString());
                } else {
                    occName = QString("Label %1").arg(foundLabel.Tag());
                }
                QString transformStr = makeTransformString(node->loc);
                node->name = QString("Label %1 | Type: %2 | Transform: %3%4%5")
                    .arg(foundLabel.Tag())
                    .arg(typeName)
                    .arg(transformStr)
                    .arg(occName.isEmpty() ? "" : QString(" | OCC Name: %1").arg(occName))
                    .arg("")
                    .toStdString();
            }
        }
    }
    for (auto& child : node->children) relinkCadNodeXCAFGeometry(child, doc);
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
        auto child = build_tree_xcaf(rootLabel, shapeTool, colorTool, defaultColor, identityLoc);
        if (child) {
            cadRoot->children.push_back(std::move(child));
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
        auto child = build_tree_xcaf(rootLabel, shapeTool, colorTool, defaultColor, identityLoc);
        if (child) {
            cadRoot->children.push_back(std::move(child));
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
    QObject::connect(treeView, &QTreeView::customContextMenuRequested, treeView, [=](const QPoint& pos) {
        QModelIndex idx = treeView->indexAt(pos);
        if (!idx.isValid()) return;
        CadNode* node = const_cast<CadNode*>(qtModel->getNode(idx));
        if (!node) return;
        QMenu menu;
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
        });
        // Add relink to XCAF node action (always for XCAF nodes)
        if (node->asXCAF()) {
            QAction* relinkAction = menu.addAction("Relink to XCAF Node (Debug)");
            QObject::connect(relinkAction, &QAction::triggered, treeView, [=]() {
                std::shared_ptr<CadNode> nodePtr = std::make_shared<CadNode>(*node);
                qDebug() << "[Relink] Starting relink for node:" << QString::fromStdString(node->name)
                         << ", xcafLabelTag:" << (node->asXCAF() ? node->asXCAF()->xcafLabelTag : -1);
                relinkCadNodeXCAFGeometry(nodePtr, doc);
                qDebug() << "[Relink] Finished relink for node:" << QString::fromStdString(nodePtr->name)
                         << ", new type:" << int(nodePtr->type)
                         << ", new data valid:" << (nodePtr->data != nullptr);
                for (const auto& child : nodePtr->children) {
                    qDebug() << "[Relink] Child node:" << QString::fromStdString(child->name)
                             << ", type:" << int(child->type) << ", xcafLabelTag:" << (child->asXCAF() ? child->asXCAF()->xcafLabelTag : -1);
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
                        // Deep copy the selected node
                        std::shared_ptr<CadNode> newNode = deepCopyNodeNonExcluded(node);
                        if (!newNode) {
                            qDebug() << "[AddToCustomModel] Deep copy failed!";
                            return;
                        }
                        newNode->name = segType.toStdString();
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
                        // Adjust the copied node so its global transform matches the original
                        adjustSubtreeTransforms(
                            node, newNode.get(), dstParentGlobal, srcGetParent
                        );
                        // Always add to the root of the selected custom model tree using the model's reset method
                        int beforeCount = static_cast<int>(customRoot->children.size());
                        qDebug() << "[AddToCustomModel] Children before insertion:" << beforeCount;
                        customModel->resetModelAndAddNode(newNode);
                        int afterCount = static_cast<int>(customRoot->children.size());
                        qDebug() << "[AddToCustomModel] Children after insertion:" << afterCount;
                        // Select and scroll to the new node at the top level
                        if (customTree) {
                            QModelIndex newIdx = customModel->index(afterCount - 1, 0, QModelIndex());
                            qDebug() << "[AddToCustomModel] New node index:" << newIdx.row();
                            if (newIdx.isValid()) {
                                customTree->setCurrentIndex(newIdx);
                                customTree->scrollTo(newIdx);
                            } else {
                                qDebug() << "[AddToCustomModel] New index is not valid!";
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
                        // --- Debug: Print root node transforms ---
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
        if (!menu.isEmpty()) menu.exec(treeView->viewport()->mapToGlobal(pos));
    });
    CadOpenGLWidget* openGLViewer = new CadOpenGLWidget;
    openGLViewer->setRootTreeNode(inputRoot.get());
    treeTabWidget->addTab(treeView, QString::fromStdString(name) + " Tree");
    openGLTabWidget->addTab(openGLViewer, QString::fromStdString(name) + " Preview");
    connectTreeAndViewer(treeView, openGLViewer, qtModel);
    // Store pointers for tab switching
    g_treeViews.push_back(treeView);
    g_openGLViews.push_back(openGLViewer);
}

void saveRailObject(QModelIndexList selected,CustomModelTreeModel* treeModel,
                    const Handle(TDocStd_Document)& doc) {
    QString baseName = QFileDialog::getSaveFileName(nullptr, "Save Rail Base Name", "", "Rail Project (*.json)");
    if (baseName.isEmpty()) return;
    if (!baseName.endsWith(".json")) baseName += ".json";
    if (selected.empty()) return;
    QString binName = baseName;
    binName.replace(".json", ".bin");
    // Save custom model JSON (only the selected Rail node or root)
    std::shared_ptr<CadNode> nodeToSave = nullptr;
    CadNode* n = const_cast<CadNode*>(treeModel->getNode(selected.first()));
    if (n && n->type == CadNodeType::Rail) nodeToSave = std::make_shared<CadNode>(*n);

    if (!nodeToSave) return;
    QJsonObject obj = nodeToSave->toJson();
    QJsonDocument jsondoc(obj);
    QFile file(baseName);
    if (!file.open(QIODevice::WriteOnly)) {
        QMessageBox::warning(nullptr, "Error", "Failed to open file for writing: " + baseName);
        return;
    }
    file.write(jsondoc.toJson(QJsonDocument::Indented));
    file.close();
    // Save XCAF document
    if (!saveXCAFToBinary(doc, binName)) {
        QMessageBox::warning(nullptr, "Error", "Failed to save XCAF binary file: " + binName);
        return;
    }
    QMessageBox::information(nullptr, "Success", "Rail and XCAF saved as:\n" + baseName + "\n" + binName);
}

void loadRailObject() {
    QString baseName = QFileDialog::getOpenFileName(nullptr, "Load Rail Base Name", "", "Rail Project (*.json)");
    if (baseName.isEmpty()) return;
    if (!baseName.endsWith(".json")) baseName += ".json";
    QString binName = baseName;
    binName.replace(".json", ".bin");
    // If .bin does not exist, try .bin.xbf
    if (!QFile::exists(binName)) {
        QString altBinName = binName + ".xbf";
        if (QFile::exists(altBinName)) {
            binName = altBinName;
        }
    }
    // Load XCAF document
    Handle(TDocStd_Document) loadedDoc;
    Handle(XCAFApp_Application) appOCC = XCAFApp_Application::GetApplication();
    appOCC->NewDocument("BinXCAF", loadedDoc);
    Handle(TDocStd_Application) occApp = Handle(TDocStd_Application)::DownCast(loadedDoc->Application());
    BinXCAFDrivers::DefineFormat(occApp);
    PCDM_ReaderStatus status = occApp->Open(binName.toStdWString().c_str(), loadedDoc);
    if (status != PCDM_RS_OK) {
        QMessageBox::warning(nullptr, "Error", "Failed to load XCAF binary file: " + binName);
        return;
    }
    // Load custom model JSON
    QFile file(baseName);
    if (!file.open(QIODevice::ReadOnly)) {
        QMessageBox::warning(nullptr, "Error", "Failed to open file: " + baseName);
        return;
    }
    QByteArray data = file.readAll();
    file.close();
    QJsonParseError err;
    QJsonDocument doc = QJsonDocument::fromJson(data, &err);
    if (doc.isNull() || !doc.isObject()) {
        QMessageBox::warning(nullptr, "Error", "Invalid JSON file: " + baseName);
        return;
    }
    auto loadedNode = CadNode::fromJson(doc.object());
    // Relink: for each node with xcafLabelTag >= 0, find the corresponding XCAF label and update geometry if needed
    relinkCadNodeXCAFGeometry(loadedNode, loadedDoc);
    QMessageBox::information(nullptr, "Success", "Rail and XCAF loaded from:\n" + baseName + "\n" + binName);
}

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    QMainWindow mainWindow;
    mainWindow.setWindowTitle("QtCadViewer - OCC C++");

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
        // The actual rail/root node
        if (!customModelRoot) customModelRoot = std::make_shared<CadNode>();
        customModelRoot->name = "Custom Model Root";
        customModelRoot->visible = true;
        customModelRoot->type = CadNodeType::Rail;
        customModelRoot->data = std::make_shared<RailNodeData>();
        customModelRootContainer->children.clear();
        customModelRootContainer->children.push_back(customModelRoot);
    }
    // Always use the common code for the custom model tree and 3D view
    CadTreeModel* cadTreeModel = nullptr;
    // Pass cadTreeModel to initTreeAndOpenGLWidget for the CAD tree
    initTreeAndOpenGLWidget(cadRootShared, tabWidget, viewTabWidget, "CAD", doc, cadTreeModel);
    // For other trees, pass nullptr
    initTreeAndOpenGLWidget(customModelRootContainer, tabWidget, viewTabWidget, "Custom Model", doc, nullptr);

    // Custom model part OpenGL widget
    relinkCadNodeXCAFGeometry(customModelRoot, doc);


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
    QObject::connect(noSelectionBtn, &QPushButton::clicked, [=]() {
        active3DView->setSelectionMode(SelectionMode::None);
        noSelectionBtn->setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; }");
        faceSelectionBtn->setStyleSheet("QPushButton { background-color: #f0f0f0; color: black; }");
        edgeSelectionBtn->setStyleSheet("QPushButton { background-color: #f0f0f0; color: black; }");
    });
    
    QObject::connect(faceSelectionBtn, &QPushButton::clicked, [=]() {
        active3DView->setSelectionMode(SelectionMode::Faces);
        noSelectionBtn->setStyleSheet("QPushButton { background-color: #f0f0f0; color: black; }");
        faceSelectionBtn->setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; }");
        edgeSelectionBtn->setStyleSheet("QPushButton { background-color: #f0f0f0; color: black; }");
    });
    
    QObject::connect(edgeSelectionBtn, &QPushButton::clicked, [=]() {
        active3DView->setSelectionMode(SelectionMode::Edges);
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
    
    // Add a special node for all shapes in the document
    auto allShapesNode = std::make_unique<XCAFLabelNode>(TDF_Label());
    allShapesNode->label = TDF_Label(); // Empty label to indicate special node
    
    // Find all labels that have shapes
    std::function<void(const TDF_Label&)> collectShapeLabels = [&](const TDF_Label& label) {
        if (!label.IsNull()) {
            TopoDS_Shape shape = shapeTool->GetShape(label);
            if (!shape.IsNull()) {
                auto shapeNode = std::make_unique<XCAFLabelNode>(label);
                allShapesNode->children.push_back(std::move(shapeNode));
            }
            
            // Recurse into children
            for (TDF_ChildIterator it(label); it.More(); it.Next()) {
                collectShapeLabels(it.Value());
            }
        }
    };
    
    collectShapeLabels(doc->Main());
    qDebug() << "Found" << allShapesNode->children.size() << "labels with shapes";
    
    if (!allShapesNode->children.empty()) {
        labelRoot->children.push_back(std::move(allShapesNode));
    }
    
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
    // After creating a new Rail node (e.g., customModelRoot or any new Rail node):
    if (customModelRoot->type == CadNodeType::Rail) {
        RailNodeData* railData = customModelRoot->asRail();
        if (railData && railData->jsonString.isEmpty()) {
            railData->jsonString = R"({"axisOfTravel":[1,0,0],"numSegments":10,"travelLength":10000,"buildJointPosition":0})";
        }
    }
    // If there are other places where Rail nodes are created, apply the same logic after creation.

    return app.exec();
}
