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
                   const TopLoc_Location& parentLoc,
                   std::unordered_map<const void*, std::shared_ptr<CadNode>>& sharedNodeMap)
{
    TopoDS_Shape shape = shapeTool->GetShape(label);
    const void* shapeKey = (!shape.IsNull()) ? shape.TShape().get() : nullptr;
    bool isReference = shapeTool->IsReference(label);
    if (isReference && shapeKey) {
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
        // 2. Get the shared child (the actual geometry/assembly)
        TDF_Label refLabel;
        if (shapeTool->GetReferredShape(label, refLabel)) {
            // Always build the shared child with parentLoc=identity
            auto sharedChild = build_tree_xcaf(refLabel, shapeTool, colorTool, refNode->color, TopLoc_Location(), sharedNodeMap);
            if (sharedChild) {
                refNode->children.push_back(sharedChild);
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
    node->xcafLabelTag = label.Tag(); // <-- Set XCAF label tag for relinking
    auto xData = std::make_shared<XCAFNodeData>();
    xData->shape = shape;
    xData->type = !shape.IsNull() ? shape.ShapeType() : TopAbs_SHAPE;
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
            
            // For references, just point to the shared node for the referred label
            auto child = build_tree_xcaf(refLabel, shapeTool, colorTool, color, refLoc, sharedNodeMap);
            if (child) {
                // Mark this as a reference node in the name
                child->name = QString("REF->%1 | %2").arg(refLabel.Tag()).arg(QString::fromStdString(child->name)).toStdString();
                node->children.push_back(child);
            }
        }
    }

    // Recurse into children (for all labels)
    for (TDF_ChildIterator it(label); it.More(); it.Next()) {
        auto child = build_tree_xcaf(it.Value(), shapeTool, colorTool, color, nodeLoc, sharedNodeMap);
        if (child) node->children.push_back(child);
    }
    
    // After node is created and type is set
    if (!shape.IsNull()) {
        xData->shape = shape;
        xData->type = shape.ShapeType();
    }
    
    // --- SHARING MAP FOR FACE/EDGE NODES ---
    // Static to persist across recursive calls (lifetime: whole build)
    using FaceEdgeKey = std::pair<const void*, const void*>; // (parent shape, face/edge TShape)
    static std::unordered_map<FaceEdgeKey, std::shared_ptr<CadNode>, FaceEdgeKeyHash> faceEdgeNodeMap;
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
                const void* parentKey = shape.TShape().get();
                const void* faceKey = face.TShape().get();
                FaceEdgeKey key = std::make_pair(parentKey, faceKey);
                std::shared_ptr<CadNode> faceNode;
                auto it = faceEdgeNodeMap.find(key);
                if (it != faceEdgeNodeMap.end()) {
                    faceNode = it->second;
                } else {
                    faceNode = std::make_shared<CadNode>();
                    faceNode->type = CadNodeType::XCAF;
                    auto faceData = std::make_shared<XCAFNodeData>();
                    faceData->shape = face;
                    faceData->type = TopAbs_FACE;
                    faceNode->data = faceData;
                    faceNode->color = get_shape_color(face, label, shapeTool, colorTool, color);
                    faceNode->loc = nodeLoc;
                    faceNode->name = QString("Face %1 of %2").arg(faceIdx).arg(QString::fromStdString(node->name)).toStdString();
                    faceEdgeNodeMap[key] = faceNode;
                }
                node->children.push_back(faceNode);
                faceCount++;
            }
            int edgeIdx = 0;
            int edgeCount = 0;
            for (TopExp_Explorer exp(shape, TopAbs_EDGE); exp.More(); exp.Next(), ++edgeIdx) {
                TopoDS_Shape edge = exp.Current();
                const void* parentKey = shape.TShape().get();
                const void* edgeKey = edge.TShape().get();
                FaceEdgeKey key = std::make_pair(parentKey, edgeKey);
                std::shared_ptr<CadNode> edgeNode;
                auto it = faceEdgeNodeMap.find(key);
                if (it != faceEdgeNodeMap.end()) {
                    edgeNode = it->second;
                } else {
                    edgeNode = std::make_shared<CadNode>();
                    edgeNode->type = CadNodeType::XCAF;
                    auto edgeData = std::make_shared<XCAFNodeData>();
                    edgeData->shape = edge;
                    edgeData->type = TopAbs_EDGE;
                    edgeNode->data = edgeData;
                    edgeNode->color = color;
                    edgeNode->loc = nodeLoc;
                    edgeNode->name = QString("Edge %1 of %2").arg(edgeIdx).arg(QString::fromStdString(node->name)).toStdString();
                    faceEdgeNodeMap[key] = edgeNode;
                }
                node->children.push_back(edgeNode);
                edgeCount++;
            }
            qDebug() << "Added" << faceCount << "faces and" << edgeCount << "edges to solid at label" << label.Tag();
        }
    }
    
    // Only store in map for non-reference nodes (for sharing by reference nodes)
    if (shapeKey) {
        sharedNodeMap[shapeKey] = node;
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
            viewer->addToSelection(node);
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
    copy->xcafLabelTag = src->xcafLabelTag; // Explicitly copy label tag
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

// --- Add this at file scope, before main() ---
void relinkCadNodeXCAFGeometry(std::shared_ptr<CadNode>& node, const Handle(TDocStd_Document)& doc) {
    if (node->xcafLabelTag >= 0) {
        Handle(XCAFDoc_ShapeTool) shapeTool = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
        Handle(XCAFDoc_ColorTool) colorTool = XCAFDoc_DocumentTool::ColorTool(doc->Main());
        TDF_Label label;
        TDF_Tool::Label(doc->GetData(), node->xcafLabelTag, label);
        if (!label.IsNull()) {
            // Get the shape for this label
            TopoDS_Shape shape = shapeTool->GetShape(label);
            if (!shape.IsNull()) {
                auto xData = std::make_shared<XCAFNodeData>();
                xData->shape = shape;
                xData->type = shape.ShapeType();
                node->data = xData;
                node->type = CadNodeType::XCAF;
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
    for (auto& child : node->children) relinkCadNodeXCAFGeometry(child, doc);
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

    QString stepFile;
    QString railJsonFile;
    QString railBinFile;
    Handle(TDocStd_Document) doc; // <-- Move doc to outer scope so it's always available
    std::unique_ptr<CadNode> cadRoot; // For main CAD tree
    std::unique_ptr<XCAFLabelNode> labelRoot; // For XCAF label tree
    Handle(XCAFDoc_ShapeTool) shapeTool;
    Handle(XCAFDoc_ColorTool) colorTool;
    std::shared_ptr<CadNode> customModelRootContainer; // For custom model tree
    std::shared_ptr<CadNode> customModelRoot;
    bool loadedFromJsonBin = false;
    if (openFile.endsWith(".json", Qt::CaseInsensitive)) {
        railJsonFile = openFile;
        railBinFile = railJsonFile;
        railBinFile.replace(".json", ".bin");
        if (!QFile::exists(railBinFile)) {
            QString altBin = railBinFile + ".xbf";
            if (QFile::exists(altBin)) {
                railBinFile = altBin;
            }
        }
        if (!QFile::exists(railBinFile)) {
            QMessageBox::critical(nullptr, "Error", "Could not find corresponding .bin or .bin.xbf file for the selected project.");
            return 1;
        }
        // Load XCAF document
        Handle(XCAFApp_Application) appOCC = XCAFApp_Application::GetApplication();
        appOCC->NewDocument("BinXCAF", doc);
        Handle(TDocStd_Application) occApp = Handle(TDocStd_Application)::DownCast(doc->Application());
        BinXCAFDrivers::DefineFormat(occApp);
        PCDM_ReaderStatus status = occApp->Open(railBinFile.toStdWString().c_str(), doc);
        if (status != PCDM_RS_OK) {
            QMessageBox::critical(nullptr, "Error", "Failed to load XCAF binary file: " + railBinFile);
            return 1;
        }
        // Load custom model JSON
        QFile file(railJsonFile);
        if (!file.open(QIODevice::ReadOnly)) {
            QMessageBox::critical(nullptr, "Error", "Failed to open file: " + railJsonFile);
            return 1;
        }
        QByteArray data = file.readAll();
        file.close();
        QJsonParseError err;
        QJsonDocument jsonDoc = QJsonDocument::fromJson(data, &err);
        if (jsonDoc.isNull() || !jsonDoc.isObject()) {
            QMessageBox::critical(nullptr, "Error", "Invalid JSON file: " + railJsonFile);
            return 1;
        }
        customModelRoot = CadNode::fromJson(jsonDoc.object());
        customModelRootContainer = std::make_shared<CadNode>();
        customModelRootContainer->name = "Custom Model Root Container";
        customModelRootContainer->visible = true;
        customModelRootContainer->children.clear();
        customModelRootContainer->children.push_back(customModelRoot);
        loadedFromJsonBin = true;
        stepFile = ""; // skip STEP loading
    } else {
        stepFile = openFile;
    }

    // Always load the XCAF document and build the trees
    if (!stepFile.isEmpty() || loadedFromJsonBin) {
        if (!loadedFromJsonBin) {
            // OCC: Load STEP file into XCAF document
            Handle(XCAFApp_Application) appOCC = XCAFApp_Application::GetApplication();
            appOCC->NewDocument("BinXCAF", doc);
            STEPCAFControl_Reader reader;
            reader.SetColorMode(true);
            reader.SetNameMode(true);
            reader.SetLayerMode(true);
            if (reader.ReadFile(stepFile.toStdString().c_str()) != IFSelect_RetDone) {
                QMessageBox::critical(nullptr, "Error", "Failed to read STEP file.");
                return 1;
            }
            if (!reader.Transfer(doc)) {
                QMessageBox::critical(nullptr, "Error", "Failed to transfer STEP file.");
                return 1;
            }
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
            auto child = build_tree_xcaf(rootLabel, shapeTool, colorTool, defaultColor, identityLoc, sharedNodeMap);
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
    }

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

    // Model/view tree for first tab
    CadTreeModel *model = new CadTreeModel(std::move(cadRoot));
    QTreeView *treeView = new QTreeView;
    treeView->setModel(model);
    treeView->setHeaderHidden(false);
    treeView->setSelectionMode(QAbstractItemView::ExtendedSelection); // Allow multi-selection
    tabWidget->addTab(treeView, "CAD Tree");

    // --- 3D View Tabs ---
    QTabWidget* viewTabWidget = new QTabWidget;
    splitter->addWidget(viewTabWidget);

    // Full model OpenGL widget
    CadOpenGLWidget *viewer = new CadOpenGLWidget;
    viewer->setRootTreeNode(model->getRootNodePointer());
    viewer->update();
    viewTabWidget->addTab(viewer, "Full Model");

    // --- Custom Model Tree ---
    // Only create default root if not loaded from JSON/bin
    if (!loadedFromJsonBin) {
        if (!customModelRootContainer) customModelRootContainer = std::make_shared<CadNode>();
        customModelRootContainer->name = "Custom Model Root Container";
        customModelRootContainer->visible = true;
        // The actual rail/root node
        if (!customModelRoot) customModelRoot = std::make_shared<CadNode>();
        customModelRoot->name = "Custom Model Root";
        customModelRoot->visible = true;
        customModelRoot->type = CadNodeType::Rail;
        customModelRoot->data = std::make_shared<RailNodeData>();
        customModelRootContainer->children.clear();
        customModelRootContainer->children.push_back(customModelRoot);
    }
    CustomModelTreeModel* customModel = new CustomModelTreeModel(customModelRootContainer);
    QTreeView* customModelTreeView = new QTreeView;
    customModelTreeView->setModel(customModel);
    customModelTreeView->setHeaderHidden(false);
    customModelTreeView->setMinimumWidth(220);
    customModelTreeView->setSelectionMode(QAbstractItemView::ExtendedSelection); // Allow multi-selection and clear selection by clicking below

    // Custom model part OpenGL widget
    CadOpenGLWidget *customPartViewer = new CadOpenGLWidget;
    relinkCadNodeXCAFGeometry(customModelRoot, doc);
    customPartViewer->setRootTreeNode(customModelRoot.get());
    viewTabWidget->addTab(customPartViewer, "Custom Model Part Editor");

    // Connect selection for both tree/view pairs
    connectTreeAndViewer(treeView, viewer, model);
    connectTreeAndViewer(customModelTreeView, customPartViewer, customModel);

    // --- Add Load/Save Rail buttons above the custom model tree ---
    QPushButton* loadRailBtn = new QPushButton("Load Rail (JSON + XCAF)");
    QPushButton* saveRailBtn = new QPushButton("Save Rail (JSON + XCAF)");
    QObject::connect(saveRailBtn, &QPushButton::clicked, [=]() {
        QString baseName = QFileDialog::getSaveFileName(nullptr, "Save Rail Base Name", "", "Rail Project (*.json)");
        if (baseName.isEmpty()) return;
        if (!baseName.endsWith(".json")) baseName += ".json";
        QString binName = baseName;
        binName.replace(".json", ".bin");
        // Save custom model JSON (only the selected Rail node or root)
        QModelIndexList selected = customModelTreeView->selectionModel()->selectedIndexes();
        std::shared_ptr<CadNode> nodeToSave = nullptr;
        if (!selected.isEmpty()) {
            CadNode* n = const_cast<CadNode*>(customModel->getNode(selected.first()));
            if (n && n->type == CadNodeType::Rail) nodeToSave = std::make_shared<CadNode>(*n);
        }
        if (!nodeToSave) nodeToSave = customModelRoot;
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
    });
    QObject::connect(loadRailBtn, &QPushButton::clicked, [=]() {
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
        customModelRoot->children.push_back(loadedNode);
        customModel->layoutChanged();
        customPartViewer->setRootTreeNode(customModelRoot.get());
        customPartViewer->markCacheDirty();
        customPartViewer->update();
        QMessageBox::information(nullptr, "Success", "Rail and XCAF loaded from:\n" + baseName + "\n" + binName);
    });




    // Force the viewer to update and render the geometry
    viewer->update();

    // Connect selection mode buttons
    QObject::connect(noSelectionBtn, &QPushButton::clicked, [=]() {
        viewer->setSelectionMode(SelectionMode::None);
        customPartViewer->setSelectionMode(SelectionMode::None);
        noSelectionBtn->setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; }");
        faceSelectionBtn->setStyleSheet("QPushButton { background-color: #f0f0f0; color: black; }");
        edgeSelectionBtn->setStyleSheet("QPushButton { background-color: #f0f0f0; color: black; }");
    });
    
    QObject::connect(faceSelectionBtn, &QPushButton::clicked, [=]() {
        viewer->setSelectionMode(SelectionMode::Faces);
        customPartViewer->setSelectionMode(SelectionMode::Faces);
        noSelectionBtn->setStyleSheet("QPushButton { background-color: #f0f0f0; color: black; }");
        faceSelectionBtn->setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; }");
        edgeSelectionBtn->setStyleSheet("QPushButton { background-color: #f0f0f0; color: black; }");
    });
    
    QObject::connect(edgeSelectionBtn, &QPushButton::clicked, [=]() {
        viewer->setSelectionMode(SelectionMode::Edges);
        customPartViewer->setSelectionMode(SelectionMode::Edges);
        noSelectionBtn->setStyleSheet("QPushButton { background-color: #f0f0f0; color: black; }");
        faceSelectionBtn->setStyleSheet("QPushButton { background-color: #f0f0f0; color: black; }");
        edgeSelectionBtn->setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; }");
    });
    
    // --- Physics Preview Tree and OpenGL Widget ---
    // 1. Create root node and model for physics preview
    auto physicsPreviewRoot = std::make_shared<CadNode>();
    physicsPreviewRoot->name = "Physics Preview Root";
    physicsPreviewRoot->type = CadNodeType::Physics;
    physicsPreviewRoot->visible = true;
    CustomModelTreeModel* physicsPreviewModel = new CustomModelTreeModel(physicsPreviewRoot);
    QTreeView* physicsPreviewTreeView = new QTreeView;
    physicsPreviewTreeView->setModel(physicsPreviewModel);
    physicsPreviewTreeView->setHeaderHidden(false);
    physicsPreviewTreeView->setSelectionMode(QAbstractItemView::ExtendedSelection);

    // 2. Create OpenGL widget for physics preview
    CadOpenGLWidget* physicsPreviewViewer = new CadOpenGLWidget;
    physicsPreviewViewer->setRootTreeNode(physicsPreviewRoot.get());

    // 3. Add to UI: add tree to tabWidget, viewer to viewTabWidget
    int physicsTreeTabIdx = tabWidget->addTab(physicsPreviewTreeView, "Physics Preview Tree");
    int physicsViewTabIdx = viewTabWidget->addTab(physicsPreviewViewer, "Physics Preview");

    // 4. Connect selection sync
    connectTreeAndViewer(physicsPreviewTreeView, physicsPreviewViewer, physicsPreviewModel);


    // Add Reframe button
    QPushButton* reframeBtn = new QPushButton("Reframe");
    buttonLayout->addWidget(reframeBtn);
    QObject::connect(reframeBtn, &QPushButton::clicked, [=]() {
        int idx = viewTabWidget->currentIndex();
        if (idx == 0) {
            qDebug() << "Reframe main";
            viewer->reframeCamera();
        } else if (idx == 1) {
            qDebug() << "Reframe sub";
            customPartViewer->reframeCamera();
        } else if (idx == 2) {
            qDebug() << "Reframe physics preview";
            physicsPreviewViewer->reframeCamera();
        }
    });
    
    // Add serialization buttons
    QPushButton* saveStepBtn = new QPushButton("Save STEP");
    QPushButton* saveBinaryBtn = new QPushButton("Save Binary");
    
    buttonLayout->addWidget(saveStepBtn);
    buttonLayout->addWidget(saveBinaryBtn);
    
    // Connect serialization buttons
    QObject::connect(saveStepBtn, &QPushButton::clicked, [=]() {
        QString baseName = QFileInfo(stepFile).baseName();
        QString outputPath = QDir::currentPath() + QDir::separator() + baseName + "_serialized.step";
        if (saveXCAFToSTEP(doc, outputPath)) {
            QMessageBox::information(nullptr, "Success", "XCAF data saved to STEP file:\n" + outputPath);
        } else {
            QMessageBox::warning(nullptr, "Error", "Failed to save XCAF data to STEP file");
        }
    });
    
    QObject::connect(saveBinaryBtn, &QPushButton::clicked, [=]() {
        QString baseName = QFileInfo(stepFile).baseName();
        QString outputPath = QDir::currentPath() + QDir::separator() + baseName + "_serialized.bin";
        if (saveXCAFToBinary(doc, outputPath)) {
            QMessageBox::information(nullptr, "Success", "XCAF data saved to binary file:\n" + outputPath);
        } else {
            QMessageBox::warning(nullptr, "Error", "Failed to save XCAF data to binary file");
        }
    });
    
    // Connect tree selection to OpenGL widget highlighting
    QObject::connect(treeView->selectionModel(), &QItemSelectionModel::selectionChanged, viewer, [=](const QItemSelection &selected, const QItemSelection &/*deselected*/){
        // Clear current selection first
        viewer->clearSelection();
        
        // Use the full current selection, not just the newly selected indexes
        QModelIndexList selectedIndexes = treeView->selectionModel()->selectedIndexes();
        QSet<CadNode*> allNodesToSelect;
        
        for (const QModelIndex& index : selectedIndexes) {
            CadNode* node = model->getNode(index);
            if (node) {
                // Add the selected node
                allNodesToSelect.insert(node);
                
                // Add all descendants recursively
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
        
        // Add all collected nodes to the selection
        for (CadNode* node : allNodesToSelect) {
            viewer->addToSelection(node);
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
    tabWidget->addTab(labelTreeView, "XCAF Tree");


    // Context menu for show/hide and debug info
    treeView->setContextMenuPolicy(Qt::CustomContextMenu);
    QObject::connect(treeView, &QTreeView::customContextMenuRequested, treeView, [=](const QPoint& pos) {
        QModelIndex idx = treeView->indexAt(pos);
        if (!idx.isValid()) return;
        
        CadNode* node = model->getNode(idx);
        if (!node) return;
        QMenu menu;
        QAction* showAction = menu.addAction("Show");
        QAction* hideAction = menu.addAction("Hide");
        
        // --- Custom Model Part Add Actions ---
        menu.addSeparator();
        QAction* addCarriageAction = menu.addAction("Add as Carriage (Custom Model)");
        QAction* addStartSegAction = menu.addAction("Add as Start Segment (Custom Model)");
        QAction* addEndSegAction = menu.addAction("Add as End Segment (Custom Model)");
        QAction* addMiddleSegAction = menu.addAction("Add as Middle Segment (Custom Model)");
        // Clear actions
        QAction* clearCarriageAction = menu.addAction("Clear Carriage (Custom Model)");
        QAction* clearStartSegAction = menu.addAction("Clear Start Segment (Custom Model)");
        QAction* clearEndSegAction = menu.addAction("Clear End Segment (Custom Model)");
        QAction* clearMiddleSegAction = menu.addAction("Clear Middle Segment (Custom Model)");
        
        // Add debug info option for solids and faces
        QAction* debugAction = nullptr;
        QAction* debugFaceAction = nullptr;
        XCAFNodeData* xData = node->asXCAF();
        if (xData) {
            if (xData->type == TopAbs_SOLID) {
                menu.addSeparator();
                debugAction = menu.addAction("Debug Solid Info");
            } else if (xData->type == TopAbs_FACE) {
                menu.addSeparator();
                debugFaceAction = menu.addAction("Debug Face Info");
            }
        }

        // --- Unselect by Color ---
        QAction* excludeByColorAction = menu.addAction("Exclude All by Color");
        QObject::connect(excludeByColorAction, &QAction::triggered, treeView, [=]() {
            QModelIndexList selectedIndexes = treeView->selectionModel()->selectedIndexes();
            if (selectedIndexes.empty()) return;
            QMap<QString, QList<QModelIndex>> colorToIndexes;
            QMap<QString, QColor> colorKeyToColor;
            // Helper to recursively collect all nodes and their children
            std::function<void(const QModelIndex&)> collectByColor;
            collectByColor = [&](const QModelIndex& idx) {
                if (!idx.isValid()) return;
                CadNode* n = model->getNode(idx);
                if (n) {
                    QColor color = n->color.toQColor();
                    QString colorKey = color.name(QColor::HexArgb);
                    colorToIndexes[colorKey].append(idx);
                    colorKeyToColor[colorKey] = color;
                }
                int rowCount = model->rowCount(idx);
                for (int i = 0; i < rowCount; ++i) {
                    collectByColor(model->index(i, 0, idx));
                }
            };
            for (const QModelIndex& selIdx : selectedIndexes) {
                collectByColor(selIdx);
            }
            if (colorToIndexes.isEmpty()) return;
            if (colorToIndexes.size() == 1) {
                // Only one color, exclude all
                QList<QModelIndex> toExclude = colorToIndexes.first();
                for (const QModelIndex& idx : toExclude) {
                    CadNode* n = model->getNode(idx);
                    if (n) n->excludedFromDecomposition = true;
                }
                viewer->update();
                for (const QModelIndex& idx : toExclude) model->dataChanged(idx, idx);
            } else {
                // Multiple colors, show a submenu to pick
                QMenu colorMenu;
                QMap<QAction*, QString> actionToColorKey;
                for (auto it = colorToIndexes.begin(); it != colorToIndexes.end(); ++it) {
                    QString colorKey = it.key();
                    QColor color = colorKeyToColor[colorKey];
                    QString label = QString("Exclude color: ") + color.name(QColor::HexArgb) + QString(" (%1)").arg(it.value().size());
                    QAction* colorAct = colorMenu.addAction(label);
                    QPixmap pix(16, 16);
                    pix.fill(color);
                    colorAct->setIcon(QIcon(pix));
                    actionToColorKey[colorAct] = colorKey;
                }
                QAction* chosen = colorMenu.exec(QCursor::pos());
                if (chosen && actionToColorKey.contains(chosen)) {
                    QString chosenColorKey = actionToColorKey[chosen];
                    QList<QModelIndex> toExclude = colorToIndexes[chosenColorKey];
                    for (const QModelIndex& idx : toExclude) {
                        CadNode* n = model->getNode(idx);
                        if (n) n->excludedFromDecomposition = true;
                    }
                    viewer->update();
                    for (const QModelIndex& idx : toExclude) model->dataChanged(idx, idx);
                }
            }
        });
        
        // --- Set as Frame Node in Active View ---
        QAction* setFrameNodeAction = menu.addAction("Set as Frame Node in Active View");
        QObject::connect(setFrameNodeAction, &QAction::triggered, treeView, [=]() {
            // Determine which OpenGL view is active
            int activeViewIdx = viewTabWidget->currentIndex();
            CadOpenGLWidget* activeViewer = nullptr;
            if (activeViewIdx == 0) {
                activeViewer = viewer;
            } else if (activeViewIdx == 1) {
                activeViewer = customPartViewer;
            }
            if (activeViewer && node) {
                // Compute accumulated transform from root to node
                TopLoc_Location accumulatedLoc;
                std::function<bool(const CadNode*, TopLoc_Location&)> findAccumulatedLoc;
                findAccumulatedLoc = [&](const CadNode* current, TopLoc_Location& acc) -> bool {
                    if (current == node) {
                        acc = acc * current->loc;
                        return true;
                    }
                    for (const auto& child : current->children) {
                        if (child) {
                            TopLoc_Location childAcc = acc * current->loc;
                            if (findAccumulatedLoc(child.get(), childAcc)) {
                                acc = childAcc;
                                return true;
                            }
                        }
                    }
                    return false;
                };
                const CadNode* root = activeViewer->getRootTreeNode();
                TopLoc_Location accLoc;
                if (root && findAccumulatedLoc(root, accLoc)) {
                    activeViewer->setSelectedFrameNode(node, accLoc);
                } else {
                    activeViewer->setSelectedFrameNode(node, node->loc);
                }
            }
        });
        // --- Custom Model Add Logic ---
        auto getSelectedNodes = [&]() -> std::vector<CadNode*> {
            QModelIndexList selectedIndexes = treeView->selectionModel()->selectedIndexes();
            std::vector<CadNode*> nodes;
            for (const QModelIndex& selIdx : selectedIndexes) {
                CadNode* n = model->getNode(selIdx);
                if (n) nodes.push_back(n);
            }
            return nodes;
        };
        QObject::connect(addCarriageAction, &QAction::triggered, treeView, [=]() {
            auto nodes = getSelectedNodes();
            if (nodes.empty()) return;
            std::shared_ptr<CadNode> newPart = std::make_shared<CadNode>();
            newPart->color = nodes[0]->color;
            newPart->type = CadNodeType::Physics;
            auto physicsData = std::make_shared<PhysicsNodeData>();
            newPart->data = physicsData;
            newPart->visible = false;
            // Compute global transform of the parent of the first selected node
            auto getGlobalTransform = [&](CadNode* node) -> TopLoc_Location {
                TopLoc_Location acc;
                std::vector<CadNode*> ancestry;
                CadNode* cur = node;
                while (cur) {
                    ancestry.push_back(cur);
                    cur = const_cast<CadNode*>(model->getParentNode(cur));
                }
                for (auto it = ancestry.rbegin(); it != ancestry.rend(); ++it) {
                    acc = acc * (*it)->loc;
                }
                return acc;
            };
            // Get parent of first selected node
            CadNode* parentOfFirst = model->getParentNode(nodes[0]);
            TopLoc_Location physicsNodeGlobal = TopLoc_Location();
            if (parentOfFirst) {
                physicsNodeGlobal = getGlobalTransform(parentOfFirst);
            }
            newPart->loc = physicsNodeGlobal;
            newPart->name = QString("Carriage | Type: Physics | Transform: %1")
                .arg(makeTransformString(newPart->loc)).toStdString();
            for (CadNode* n : nodes) {
                auto copy = deepCopyNodeNonExcluded(n);
                if (copy) {
                    TopLoc_Location childGlobal = getGlobalTransform(n);
                    copy->loc = childGlobal * physicsNodeGlobal.Inverted();
                    newPart->children.push_back(copy);
                }
            }
            insertCustomModelNodeAtCadTreePosition(customModelRoot.get(), newPart, nodes, model);
            customModel->layoutChanged();
            customPartViewer->setRootTreeNode(customModelRoot.get());
        });
        QObject::connect(addStartSegAction, &QAction::triggered, treeView, [=]() {
            auto nodes = getSelectedNodes();
            if (nodes.empty()) return;
            std::shared_ptr<CadNode> newPart = std::make_shared<CadNode>();
            newPart->color = nodes[0]->color;
            newPart->type = CadNodeType::Physics;
            auto physicsData = std::make_shared<PhysicsNodeData>();
            newPart->data = physicsData;
            newPart->visible = false;
            auto getGlobalTransform = [&](CadNode* node) -> TopLoc_Location {
                TopLoc_Location acc;
                std::vector<CadNode*> ancestry;
                CadNode* cur = node;
                while (cur) {
                    ancestry.push_back(cur);
                    cur = const_cast<CadNode*>(model->getParentNode(cur));
                }
                for (auto it = ancestry.rbegin(); it != ancestry.rend(); ++it) {
                    acc = acc * (*it)->loc;
                }
                return acc;
            };
            CadNode* parentOfFirst = model->getParentNode(nodes[0]);
            TopLoc_Location physicsNodeGlobal = TopLoc_Location();
            if (parentOfFirst) {
                physicsNodeGlobal = getGlobalTransform(parentOfFirst);
            }
            newPart->loc = physicsNodeGlobal;
            newPart->name = QString("Start Segment | Type: Physics | Transform: %1")
                .arg(makeTransformString(newPart->loc)).toStdString();
            for (CadNode* n : nodes) {
                auto copy = deepCopyNodeNonExcluded(n);
                if (copy) {
                    TopLoc_Location childGlobal = getGlobalTransform(n);
                    copy->loc = childGlobal * physicsNodeGlobal.Inverted();
                    newPart->children.push_back(copy);
                }
            }
            insertCustomModelNodeAtCadTreePosition(customModelRoot.get(), newPart, nodes, model);
            customModel->layoutChanged();
            customPartViewer->setRootTreeNode(customModelRoot.get());
        });
        QObject::connect(addEndSegAction, &QAction::triggered, treeView, [=]() {
            auto nodes = getSelectedNodes();
            if (nodes.empty()) return;
            std::shared_ptr<CadNode> newPart = std::make_shared<CadNode>();
            newPart->color = nodes[0]->color;
            newPart->type = CadNodeType::Physics;
            auto physicsData = std::make_shared<PhysicsNodeData>();
            newPart->data = physicsData;
            newPart->visible = false;
            auto getGlobalTransform = [&](CadNode* node) -> TopLoc_Location {
                TopLoc_Location acc;
                std::vector<CadNode*> ancestry;
                CadNode* cur = node;
                while (cur) {
                    ancestry.push_back(cur);
                    cur = const_cast<CadNode*>(model->getParentNode(cur));
                }
                for (auto it = ancestry.rbegin(); it != ancestry.rend(); ++it) {
                    acc = acc * (*it)->loc;
                }
                return acc;
            };
            CadNode* parentOfFirst = model->getParentNode(nodes[0]);
            TopLoc_Location physicsNodeGlobal = TopLoc_Location();
            if (parentOfFirst) {
                physicsNodeGlobal = getGlobalTransform(parentOfFirst);
            }
            newPart->loc = physicsNodeGlobal;
            newPart->name = QString("End Segment | Type: Physics | Transform: %1")
                .arg(makeTransformString(newPart->loc)).toStdString();
            for (CadNode* n : nodes) {
                auto copy = deepCopyNodeNonExcluded(n);
                if (copy) {
                    TopLoc_Location childGlobal = getGlobalTransform(n);
                    copy->loc = childGlobal * physicsNodeGlobal.Inverted();
                    newPart->children.push_back(copy);
                }
            }
            insertCustomModelNodeAtCadTreePosition(customModelRoot.get(), newPart, nodes, model);
            customModel->layoutChanged();
            customPartViewer->setRootTreeNode(customModelRoot.get());
        });
        QObject::connect(addMiddleSegAction, &QAction::triggered, treeView, [=]() {
            auto nodes = getSelectedNodes();
            if (nodes.empty()) return;
            std::shared_ptr<CadNode> newPart = std::make_shared<CadNode>();
            newPart->color = nodes[0]->color;
            newPart->type = CadNodeType::Physics;
            auto physicsData = std::make_shared<PhysicsNodeData>();
            newPart->data = physicsData;
            newPart->visible = false;
            auto getGlobalTransform = [&](CadNode* node) -> TopLoc_Location {
                TopLoc_Location acc;
                std::vector<CadNode*> ancestry;
                CadNode* cur = node;
                while (cur) {
                    ancestry.push_back(cur);
                    cur = const_cast<CadNode*>(model->getParentNode(cur));
                }
                for (auto it = ancestry.rbegin(); it != ancestry.rend(); ++it) {
                    acc = acc * (*it)->loc;
                }
                return acc;
            };
            CadNode* parentOfFirst = model->getParentNode(nodes[0]);
            TopLoc_Location physicsNodeGlobal = TopLoc_Location();
            if (parentOfFirst) {
                physicsNodeGlobal = getGlobalTransform(parentOfFirst);
            }
            newPart->loc = physicsNodeGlobal;
            newPart->name = QString("Middle Segment | Type: Physics | Transform: %1")
                .arg(makeTransformString(newPart->loc)).toStdString();
            for (CadNode* n : nodes) {
                auto copy = deepCopyNodeNonExcluded(n);
                if (copy) {
                    TopLoc_Location childGlobal = getGlobalTransform(n);
                    copy->loc = childGlobal * physicsNodeGlobal.Inverted();
                    newPart->children.push_back(copy);
                }
            }
            insertCustomModelNodeAtCadTreePosition(customModelRoot.get(), newPart, nodes, model);
            customModel->layoutChanged();
            customPartViewer->setRootTreeNode(customModelRoot.get());
        });
        // --- Custom Model Clear Logic ---
        QObject::connect(clearCarriageAction, &QAction::triggered, treeView, [=]() {
            auto& children = customModelRoot->children;
            children.erase(
                std::remove_if(children.begin(), children.end(),
                               [](const std::shared_ptr<CadNode>& node) {
                                   return node->name == "Carriage";
                               }),
                children.end());
            customModel->layoutChanged(); // Notify model of structure change
            customPartViewer->setRootTreeNode(customModelRoot.get()); // Update viewer's root node
        });
        QObject::connect(clearStartSegAction, &QAction::triggered, treeView, [=]() {
            auto& children = customModelRoot->children;
            children.erase(
                std::remove_if(children.begin(), children.end(),
                               [](const std::shared_ptr<CadNode>& node) {
                                   return node->name == "Start Segment";
                               }),
                children.end());
            customModel->layoutChanged(); // Notify model of structure change
            customPartViewer->setRootTreeNode(customModelRoot.get()); // Update viewer's root node
        });
        QObject::connect(clearEndSegAction, &QAction::triggered, treeView, [=]() {
            auto& children = customModelRoot->children;
            children.erase(
                std::remove_if(children.begin(), children.end(),
                               [](const std::shared_ptr<CadNode>& node) {
                                   return node->name == "End Segment";
                               }),
                children.end());
            customModel->layoutChanged(); // Notify model of structure change
            customPartViewer->setRootTreeNode(customModelRoot.get()); // Update viewer's root node
        });
        QObject::connect(clearMiddleSegAction, &QAction::triggered, treeView, [=]() {
            auto& children = customModelRoot->children;
            children.erase(
                std::remove_if(children.begin(), children.end(),
                               [](const std::shared_ptr<CadNode>& node) {
                                   return node->name == "Middle Segment";
                               }),
                children.end());
            customModel->layoutChanged(); // Notify model of structure change
            customPartViewer->setRootTreeNode(customModelRoot.get()); // Update viewer's root node
        });
        
        QObject::connect(showAction, &QAction::triggered, treeView, [=]() {
            auto selection = treeView->selectionModel()->selectedIndexes();
            QSet<CadNode*> affected;
            for (const QModelIndex& selIdx : selection) {
                CadNode* n = model->getNode(selIdx);
                if (n && !affected.contains(n)) {
                    n->setVisibleRecursive(true);
                    affected.insert(n);
                }
            }
            viewer->update();
            for (const QModelIndex& selIdx : selection) model->dataChanged(selIdx, selIdx);
        });
        
        QObject::connect(hideAction, &QAction::triggered, treeView, [=]() {
            auto selection = treeView->selectionModel()->selectedIndexes();
            QSet<CadNode*> affected;
            for (const QModelIndex& selIdx : selection) {
                CadNode* n = model->getNode(selIdx);
                if (n && !affected.contains(n)) {
                    n->setVisibleRecursive(false);
                    affected.insert(n);
                }
            }
            viewer->update();
            for (const QModelIndex& selIdx : selection) model->dataChanged(selIdx, selIdx);
        });
        
        // Debug info action for solids
        if (debugAction) {
            QObject::connect(debugAction, &QAction::triggered, treeView, [=]() {
                CadNode* solidNode = model->getNode(idx);
                XCAFNodeData* solidNodeXData = solidNode->asXCAF();


                if (!solidNode || xData->type != TopAbs_SOLID) return;
                
                qDebug() << "=== SOLID DEBUG INFO ===";
                qDebug() << "Node name:" << QString::fromStdString(solidNode->name);
                qDebug() << "Node type:" << shapeTypeToString(xData->type);
                qDebug() << "Has shape:" << !xData->shape.IsNull();
                qDebug() << "Visible:" << solidNode->visible;
                qDebug() << "Color:" << solidNode->color.r << solidNode->color.g << solidNode->color.b << solidNode->color.a;
                
                // Check if solid has faces
                if (!solidNodeXData->shape.IsNull()) {
                    TopoDS_Shape shape = solidNodeXData->shape;
                    int faceCount = 0;
                    int edgeCount = 0;
                    for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next()) faceCount++;
                    for (TopExp_Explorer exp(shape, TopAbs_EDGE); exp.More(); exp.Next()) edgeCount++;
                    qDebug() << "Solid has" << faceCount << "faces and" << edgeCount << "edges";
                    
                    // Check bounding box
                    Bnd_Box bbox;
                    BRepBndLib::Add(shape, bbox);
                    Standard_Real xmin, ymin, zmin, xmax, ymax, zmax;
                    bbox.Get(xmin, ymin, zmin, xmax, ymax, zmax);
                    double size = std::max(std::max(xmax-xmin, ymax-ymin), zmax-zmin);
                    qDebug() << "Bounding box:" << xmin << ymin << zmin << "to" << xmax << ymax << zmax;
                    qDebug() << "Size:" << size;
                    
                    // Check if faces were added to tree
                    int treeFaceCount = 0;
                    int treeEdgeCount = 0;
                    for (const auto& child : solidNode->children) {
                        if (xData->type == TopAbs_FACE) treeFaceCount++;
                        if (xData->type == TopAbs_EDGE) treeEdgeCount++;
                    }
                    qDebug() << "Tree has" << treeFaceCount << "faces and" << treeEdgeCount << "edges";
                    
                    if (treeFaceCount == 0 && faceCount > 0) {
                        qDebug() << "WARNING: Solid has faces but no faces in tree!";
                        qDebug() << "This suggests faces were not added during tree building.";
                    }
                }
                
                // Check parent info
                if (idx.parent().isValid()) {
                    CadNode* parentNode = model->getNode(idx.parent());
                    if (parentNode) {
                        qDebug() << "Parent name:" << QString::fromStdString(parentNode->name);
                        qDebug() << "Parent has" << parentNode->children.size() << "children";
                    }
                }
                
                // Check if solid is in viewer selection
                bool isSelected = false;
                // Note: We'd need to access viewer's selection state here
                qDebug() << "Is selected in viewer:" << isSelected;
                
                qDebug() << "=== END SOLID DEBUG INFO ===";
            });
        }
        
        // Debug info action for faces
        if (debugFaceAction) {
            QObject::connect(debugFaceAction, &QAction::triggered, treeView, [=]() {
                CadNode* faceNode = model->getNode(idx);
                XCAFNodeData* faceNodeXData = node->asXCAF();

                if (!faceNode || faceNodeXData->type != TopAbs_FACE) return;
                
                qDebug() << "=== FACE DEBUG INFO ===";
                qDebug() << "Node name:" << QString::fromStdString(faceNode->name);
                qDebug() << "Node type:" << shapeTypeToString(faceNodeXData->type);
                qDebug() << "Has shape:" << !faceNodeXData->shape.IsNull();
                qDebug() << "Visible:" << faceNode->visible;
                qDebug() << "Color:" << faceNode->color.r << faceNode->color.g << faceNode->color.b << faceNode->color.a;
                
                // Check face geometry
                if (!faceNodeXData->shape.IsNull()) {
                    TopoDS_Face face = TopoDS::Face(faceNodeXData->shape);
                    
                    // Check bounding box
                    Bnd_Box bbox;
                    BRepBndLib::Add(face, bbox);
                    Standard_Real xmin, ymin, zmin, xmax, ymax, zmax;
                    bbox.Get(xmin, ymin, zmin, xmax, ymax, zmax);
                    double size = std::max(std::max(xmax-xmin, ymax-ymin), zmax-zmin);
                    qDebug() << "Face bounding box:" << xmin << ymin << zmin << "to" << xmax << ymax << zmax;
                    qDebug() << "Face size:" << size;
                    
                    // Check if face is too small
                    if (size < 0.001) {
                        qDebug() << "WARNING: Face is very small (< 0.001), might be skipped during rendering";
                    }
                    
                    // Check face orientation
                    if (face.Orientation() == TopAbs_FORWARD) {
                        qDebug() << "Face orientation: FORWARD";
                    } else if (face.Orientation() == TopAbs_REVERSED) {
                        qDebug() << "Face orientation: REVERSED";
                    } else {
                        qDebug() << "Face orientation: INTERNAL";
                    }
                    
                    // Check if face is closed
                    TopExp_Explorer edgeExp(face, TopAbs_EDGE);
                    int edgeCount = 0;
                    for (; edgeExp.More(); edgeExp.Next()) edgeCount++;
                    qDebug() << "Face has" << edgeCount << "edges";
                    
                    // Check surface type
                    Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
                    if (!surface.IsNull()) {
                        qDebug() << "Surface type:" << surface->DynamicType()->Name();
                    } else {
                        qDebug() << "WARNING: Face has no surface!";
                    }
                    
                    // Test tessellation
                    qDebug() << "=== TESSELLATION TEST ===";
                    double testPrecision = 0.1; // Start with coarse precision
                    BRepMesh_IncrementalMesh testMesher(face, testPrecision);
                    TopLoc_Location testLoc;
                    Handle(Poly_Triangulation) testTriangulation = BRep_Tool::Triangulation(face, testLoc);
                    
                    if (testTriangulation.IsNull()) {
                        qDebug() << "TESSELLATION FAILED: triangulation is null";
                    } else if (testTriangulation->Triangles().IsEmpty()) {
                        qDebug() << "TESSELLATION FAILED: triangulation has no triangles";
                        qDebug() << "Triangulation has" << testTriangulation->NbNodes() << "nodes";
                    } else {
                        qDebug() << "TESSELLATION SUCCESS: triangulation has" << testTriangulation->Triangles().Size() << "triangles";
                        qDebug() << "Triangulation has" << testTriangulation->NbNodes() << "nodes";
                    }
                    
                    // Try different precision values
                    for (double precision : {0.5, 0.1, 0.05, 0.01}) {
                        BRepMesh_IncrementalMesh testMesher2(face, precision);
                        Handle(Poly_Triangulation) testTri2 = BRep_Tool::Triangulation(face, testLoc);
                        if (!testTri2.IsNull() && !testTri2->Triangles().IsEmpty()) {
                            qDebug() << "SUCCESS with precision" << precision << ":" << testTri2->Triangles().Size() << "triangles";
                            break;
                        } else {
                            qDebug() << "FAILED with precision" << precision;
                        }
                    }
                    qDebug() << "=== END TESSELLATION TEST ===";
                    
                    // Test rendering pipeline
                    qDebug() << "=== RENDERING PIPELINE TEST ===";
                    
                    // Test distance culling (simulate OpenGL widget logic)
                    QVector3D faceCenter((xmin + xmax) * 0.5, (ymin + ymax) * 0.5, (zmin + zmax) * 0.5);
                    qDebug() << "Face center:" << faceCenter;
                    
                    // Simulate camera position (you might need to adjust these values)
                    QVector3D simulatedCameraPos(0, 0, 100); // Typical camera position
                    float simulatedCameraZoom = 100.0f; // Typical zoom
                    
                    QVector3D toCamera = faceCenter - simulatedCameraPos;
                    float distanceSquared = toCamera.x() * toCamera.x() + toCamera.y() * toCamera.y() + toCamera.z() * toCamera.z();
                    float distanceLimit = simulatedCameraZoom * simulatedCameraZoom * 200.0f;
                    
                    qDebug() << "Distance to camera:" << sqrt(distanceSquared);
                    qDebug() << "Distance limit:" << sqrt(distanceLimit);
                    if (distanceSquared > distanceLimit) {
                        qDebug() << "WARNING: Face would be culled due to distance!";
                    } else {
                        qDebug() << "Distance OK for rendering";
                    }
                    
                    // Test size culling
                    if (size < 0.001) {
                        qDebug() << "WARNING: Face would be culled due to small size!";
                    } else {
                        qDebug() << "Size OK for rendering";
                    }
                    
                    // Test if face is a plane (which should render fine)
                    if (surface->DynamicType()->Name() == "Geom_Plane") {
                        qDebug() << "Plane surface - should render easily";
                    }
                    
                    qDebug() << "=== END RENDERING PIPELINE TEST ===";
                }
                
                // Check parent info
                if (idx.parent().isValid()) {
                    CadNode* parentNode = model->getNode(idx.parent());
                    if (parentNode) {
                        qDebug() << "Parent name:" << QString::fromStdString(parentNode->name);
                        qDebug() << "Parent has" << parentNode->children.size() << "children";
                    }
                }
                
                qDebug() << "=== END FACE DEBUG INFO ===";
            });
        }
        
        QAction* excludeAction = nullptr;
        if (node->excludedFromDecomposition) {
            excludeAction = menu.addAction("Include in Decomposition");
        } else {
            excludeAction = menu.addAction("Exclude from Decomposition");
        }
        QObject::connect(excludeAction, &QAction::triggered, treeView, [=]() {
            // Recursively set excludedFromDecomposition for node and all children
            std::function<void(CadNode*, bool)> setExcludedRecursive = [&](CadNode* n, bool value) {
                n->excludedFromDecomposition = value;
                for (auto& child : n->children) {
                    if (child) setExcludedRecursive(child.get(), value);
                }
            };
            bool newValue = !node->excludedFromDecomposition;
            setExcludedRecursive(node, newValue);
            model->dataChanged(idx, idx); // Update tree display
            viewer->update(); // Refresh viewer if needed
        });
        
        // --- Exclude/Include by Color ---
        // (Removed from CAD Tree context menu)
        // (No QAction* excludeByColorAction or includeByColorAction or related logic here)
        
        menu.exec(treeView->viewport()->mapToGlobal(pos));
    });

    customModelTreeView->setContextMenuPolicy(Qt::CustomContextMenu);
    QObject::connect(customModelTreeView, &QTreeView::customContextMenuRequested, customModelTreeView, [=](const QPoint& pos) {
        QModelIndex idx = customModelTreeView->indexAt(pos);
        if (!idx.isValid()) return;
        CadNode* node = const_cast<CadNode*>(customModel->getNode(idx));
        if (!node) return;
        QMenu menu;
        // Exclude/include from decomposition option
        QAction* excludeAction = nullptr;
        if (node->excludedFromDecomposition) {
            excludeAction = menu.addAction("Include in Decomposition");
        } else {
            excludeAction = menu.addAction("Exclude from Decomposition");
        }
        QObject::connect(excludeAction, &QAction::triggered, customModelTreeView, [=]() {
            // Recursively set excludedFromDecomposition for node and all children
            std::function<void(CadNode*, bool)> setExcludedRecursive = [&](CadNode* n, bool value) {
                n->excludedFromDecomposition = value;
                for (auto& child : n->children) {
                    if (child) setExcludedRecursive(child.get(), value);
                }
            };
            bool newValue = !node->excludedFromDecomposition;
            setExcludedRecursive(node, newValue);
            customModel->dataChanged(idx, idx); // Update tree display
            customPartViewer->update(); // Refresh viewer if needed
        });
        // Only show for physics nodes
        if (node->type == CadNodeType::Physics) {
            QAction* vhacdAction = menu.addAction("Generate V-HACD");
            QAction* coacdAction = menu.addAction("Generate CoACD");
            
            QObject::connect(vhacdAction, &QAction::triggered, customModelTreeView, [=]() {
                // V-HACD parameter widget (non-blocking)
                QWidget* vhacdWidget = new QWidget(nullptr);
                vhacdWidget->setAttribute(Qt::WA_DeleteOnClose);
                vhacdWidget->setWindowTitle("V-HACD Parameters");
                QFormLayout* form = new QFormLayout;
                QSpinBox* resolutionSpin = new QSpinBox; 
                resolutionSpin->setRange(1000, 1000000); 
                resolutionSpin->setValue(10000); // Lower default for faster VHACD
                QSpinBox* maxHullSpin = new QSpinBox; 
                maxHullSpin->setRange(1, 1024); 
                maxHullSpin->setValue(8); // Lower default for faster VHACD
                QDoubleSpinBox* minVolumeSpin = new QDoubleSpinBox; 
                minVolumeSpin->setRange(0.0, 0.1); 
                minVolumeSpin->setDecimals(6); 
                minVolumeSpin->setSingleStep(0.001); 
                minVolumeSpin->setValue(0.01); // Less strict for faster VHACD
                form->addRow("Resolution", resolutionSpin);
                form->addRow("Max Hulls", maxHullSpin);
                form->addRow("Min Volume", minVolumeSpin);
                QPushButton* updateBtn = new QPushButton("Update");
                QVBoxLayout* vbox = new QVBoxLayout;
                vbox->addLayout(form);
                vbox->addWidget(updateBtn);
                vhacdWidget->setLayout(vbox);
                QObject::connect(updateBtn, &QPushButton::clicked, vhacdWidget, [=]() {
                    generateVHACDStub(QString::fromStdString(node->name),
                                     resolutionSpin->value(),
                                     maxHullSpin->value(),
                                     minVolumeSpin->value(),
                                     node);
                });
                vhacdWidget->setWindowModality(Qt::NonModal);
                vhacdWidget->setFocusPolicy(Qt::StrongFocus);
                vhacdWidget->resize(350, 200);
                vhacdWidget->show();
                vhacdWidget->activateWindow();
                vhacdWidget->raise();
            });

            QObject::connect(coacdAction, &QAction::triggered, customModelTreeView, [=]() {
                QWidget* coacdWidget = new QWidget(nullptr);
                coacdWidget->setAttribute(Qt::WA_DeleteOnClose);
                coacdWidget->setWindowTitle("CoACD Parameters");
                QFormLayout* form = new QFormLayout;

                QDoubleSpinBox* concavitySpin = new QDoubleSpinBox;
                concavitySpin->setRange(0.0, 1.0);
                concavitySpin->setDecimals(3);
                concavitySpin->setSingleStep(0.01);
                concavitySpin->setValue(0.05);
                concavitySpin->setToolTip("Maximum allowed concavity (lower = more accurate, slower)");
                form->addRow("Concavity", concavitySpin);

                QDoubleSpinBox* alphaSpin = new QDoubleSpinBox;
                alphaSpin->setRange(0.0, 1.0);
                alphaSpin->setDecimals(3);
                alphaSpin->setSingleStep(0.01);
                alphaSpin->setValue(0.05);
                alphaSpin->setToolTip("Alpha: balance between concavity and number of hulls");
                form->addRow("Alpha", alphaSpin);

                QDoubleSpinBox* betaSpin = new QDoubleSpinBox;
                betaSpin->setRange(0.0, 1.0);
                betaSpin->setDecimals(3);
                betaSpin->setSingleStep(0.01);
                betaSpin->setValue(0.05);
                betaSpin->setToolTip("Beta: balance between concavity and volume");
                form->addRow("Beta", betaSpin);

                QSpinBox* maxConvexHullSpin = new QSpinBox;
                maxConvexHullSpin->setRange(-1, 1024);
                maxConvexHullSpin->setValue(-1); // Default: unlimited hulls
                maxConvexHullSpin->setToolTip("Maximum number of convex hulls (-1 = unlimited)");
                form->addRow("Max Convex Hulls", maxConvexHullSpin);

                QComboBox* preprocessCombo = new QComboBox;
                preprocessCombo->addItem("auto");
                preprocessCombo->addItem("on");
                preprocessCombo->addItem("off");
                preprocessCombo->setCurrentText("auto");
                preprocessCombo->setToolTip("Preprocessing: auto/on/off (auto is usually fastest)");
                form->addRow("Preprocess", preprocessCombo);

                QSpinBox* prepResSpin = new QSpinBox;
                prepResSpin->setRange(1, 1000);
                prepResSpin->setValue(20);
                prepResSpin->setToolTip("Voxel resolution for preprocessing (lower = faster, less detail)");
                form->addRow("Prep Resolution", prepResSpin);

                QSpinBox* sampleResSpin = new QSpinBox;
                sampleResSpin->setRange(100, 10000);
                sampleResSpin->setValue(500);
                sampleResSpin->setToolTip("Number of surface samples (lower = faster, less detail)");
                form->addRow("Sample Resolution", sampleResSpin);

                QSpinBox* mctsNodesSpin = new QSpinBox;
                mctsNodesSpin->setRange(1, 100);
                mctsNodesSpin->setValue(10);
                mctsNodesSpin->setToolTip("Number of nodes in MCTS search (lower = faster, less accurate)");
                form->addRow("MCTS Nodes", mctsNodesSpin);

                QSpinBox* mctsIterSpin = new QSpinBox;
                mctsIterSpin->setRange(1, 1000);
                mctsIterSpin->setValue(30);
                mctsIterSpin->setToolTip("Number of MCTS iterations (lower = faster, less accurate)");
                form->addRow("MCTS Iterations", mctsIterSpin);

                QSpinBox* mctsDepthSpin = new QSpinBox;
                mctsDepthSpin->setRange(1, 10);
                mctsDepthSpin->setValue(3);
                mctsDepthSpin->setToolTip("Maximum depth of MCTS search tree");
                form->addRow("MCTS Max Depth", mctsDepthSpin);

                QCheckBox* pcaCheck = new QCheckBox;
                pcaCheck->setChecked(false);
                pcaCheck->setToolTip("Enable PCA alignment for input mesh");
                form->addRow("PCA", pcaCheck);

                QCheckBox* mergeCheck = new QCheckBox;
                mergeCheck->setChecked(true);
                mergeCheck->setToolTip("Merge small hulls together after decomposition");
                form->addRow("Merge", mergeCheck);

                QCheckBox* decimateCheck = new QCheckBox;
                decimateCheck->setChecked(false);
                decimateCheck->setToolTip("Decimate (simplify) hulls after decomposition");
                form->addRow("Decimate", decimateCheck);

                QSpinBox* maxChVertexSpin = new QSpinBox;
                maxChVertexSpin->setRange(4, 4096);
                maxChVertexSpin->setValue(128);
                maxChVertexSpin->setToolTip("Maximum number of vertices per convex hull");
                form->addRow("Max CH Vertex", maxChVertexSpin);

                QCheckBox* extrudeCheck = new QCheckBox;
                extrudeCheck->setChecked(false);
                extrudeCheck->setToolTip("Enable extrusion for thin parts");
                form->addRow("Extrude", extrudeCheck);

                QDoubleSpinBox* extrudeMarginSpin = new QDoubleSpinBox;
                extrudeMarginSpin->setRange(0.0, 1.0);
                extrudeMarginSpin->setDecimals(4);
                extrudeMarginSpin->setSingleStep(0.001);
                extrudeMarginSpin->setValue(0.01);
                extrudeMarginSpin->setToolTip("Margin for extrusion (if enabled)");
                form->addRow("Extrude Margin", extrudeMarginSpin);

                QComboBox* apxModeCombo = new QComboBox;
                apxModeCombo->addItem("auto");
                apxModeCombo->addItem("ch");
                apxModeCombo->addItem("box");
                apxModeCombo->setCurrentText("auto");
                apxModeCombo->setToolTip("Approximation mode: auto, ch = convex hull, box = bounding box");
                form->addRow("Apx Mode", apxModeCombo);

                QSpinBox* seedSpin = new QSpinBox;
                seedSpin->setRange(-1, INT_MAX); // Allow -1 for random
                seedSpin->setValue(-1); // Default: random
                seedSpin->setToolTip("Random seed for reproducibility (-1 = random)");
                form->addRow("Seed", seedSpin);

                QPushButton* updateBtn = new QPushButton("Update");
                QVBoxLayout* vbox = new QVBoxLayout;
                vbox->addLayout(form);
                vbox->addWidget(updateBtn);
                coacdWidget->setLayout(vbox);

                QObject::connect(updateBtn, &QPushButton::clicked, coacdWidget, [=]() {
                    QString preprocessStr = preprocessCombo->currentText();
                    std::string preprocess = preprocessStr.toStdString();
                    std::string apxMode = apxModeCombo->currentText().toStdString();
                    generateCoACDStub(
                        QString::fromStdString(node->name),
                        concavitySpin->value(),
                        alphaSpin->value(),
                        betaSpin->value(),
                        node,
                        maxConvexHullSpin->value(),
                        preprocess,
                        prepResSpin->value(),
                        sampleResSpin->value(),
                        mctsNodesSpin->value(),
                        mctsIterSpin->value(),
                        mctsDepthSpin->value(),
                        pcaCheck->isChecked(),
                        mergeCheck->isChecked(),
                        decimateCheck->isChecked(),
                        maxChVertexSpin->value(),
                        extrudeCheck->isChecked(),
                        extrudeMarginSpin->value(),
                        apxMode,
                        seedSpin->value()
                    );
                });
                coacdWidget->setWindowModality(Qt::NonModal);
                coacdWidget->setFocusPolicy(Qt::StrongFocus);
                coacdWidget->resize(400, 600);
                coacdWidget->show();
                coacdWidget->activateWindow();
                coacdWidget->raise();
            });
        }
        
        // Add connection point configuration for existing connection points
        if (node->type == CadNodeType::ConnectionPoint) {
            menu.addSeparator();
            QAction* configureConnectionAction = menu.addAction("Configure Connection Point");
            QObject::connect(configureConnectionAction, &QAction::triggered, customModelTreeView, [=]() {
                ConnectionPointData* connData = node->asConnectionPoint();
                if (!connData) return;
                
                // Create configuration dialog
                QDialog* configDialog = new QDialog(nullptr);
                configDialog->setWindowTitle("Configure Connection Point");
                configDialog->setAttribute(Qt::WA_DeleteOnClose);
                
                QVBoxLayout* layout = new QVBoxLayout(configDialog);
                
                // Connection type checkboxes
                QCheckBox* cablesCheck = new QCheckBox("Can connect cables");
                QCheckBox* conveyorsCheck = new QCheckBox("Can connect conveyors");
                
                // Set current state
                cablesCheck->setChecked(connData->canConnectCables());
                conveyorsCheck->setChecked(connData->canConnectConveyors());
                
                layout->addWidget(cablesCheck);
                layout->addWidget(conveyorsCheck);
                
                // Description field
                QFormLayout* formLayout = new QFormLayout();
                QLineEdit* descriptionEdit = new QLineEdit(QString::fromStdString(connData->description));
                formLayout->addRow("Description:", descriptionEdit);
                layout->addLayout(formLayout);
                
                // Buttons
                QHBoxLayout* buttonLayout = new QHBoxLayout();
                QPushButton* okButton = new QPushButton("OK");
                QPushButton* cancelButton = new QPushButton("Cancel");
                buttonLayout->addWidget(okButton);
                buttonLayout->addWidget(cancelButton);
                layout->addLayout(buttonLayout);
                
                // Connect buttons
                QObject::connect(okButton, &QPushButton::clicked, configDialog, [=]() {
                    // Update connection flags
                    connData->setCanConnectCables(cablesCheck->isChecked());
                    connData->setCanConnectConveyors(conveyorsCheck->isChecked());
                    connData->description = descriptionEdit->text().toStdString();
                    
                    // Update node name to reflect connection types
                    QString connectionTypes;
                    if (connData->canConnectCables() && connData->canConnectConveyors()) {
                        connectionTypes = "Cables & Conveyors";
                    } else if (connData->canConnectCables()) {
                        connectionTypes = "Cables";
                    } else if (connData->canConnectConveyors()) {
                        connectionTypes = "Conveyors";
                    } else {
                        connectionTypes = "None";
                    }
                    
                    // Extract location from current location
                    gp_Trsf transform = node->loc.Transformation();
                    gp_XYZ translation = transform.TranslationPart();
                    
                    node->name = QString("Connection Point (%1) | Location: (%2, %3, %4)")
                        .arg(connectionTypes)
                        .arg(QString::number(translation.X(), 'f', 2))
                        .arg(QString::number(translation.Y(), 'f', 2))
                        .arg(QString::number(translation.Z(), 'f', 2))
                        .toStdString();
                    
                    // Update tree display
                    customModel->layoutChanged();
                    customPartViewer->update();
                    
                    configDialog->accept();
                });
                
                QObject::connect(cancelButton, &QPushButton::clicked, configDialog, &QDialog::reject);
                
                configDialog->exec();
            });
        }
        
        // In the customModelTreeView context menu, add exclude/include by color options:
        QAction* excludeByColorAction2 = menu.addAction("Exclude All by Color");
        QAction* includeByColorAction2 = menu.addAction("Include All by Color");
        QObject::connect(excludeByColorAction2, &QAction::triggered, customModelTreeView, [=]() {
            QModelIndexList selectedIndexes = customModelTreeView->selectionModel()->selectedIndexes();
            if (selectedIndexes.empty()) return;
            QMap<QString, QColor> colorKeyToColor;
            // Gather all unique colors in the tree
            std::function<void(CadNode*)> collectColors;
            collectColors = [&](CadNode* n) {
                if (n) {
                    QColor color = n->color.toQColor();
                    QString colorKey = color.name(QColor::HexArgb);
                    colorKeyToColor[colorKey] = color;
                    for (auto& child : n->children) collectColors(child.get());
                }
            };
            for (const auto& child : customModel->getRootNodePointer()->children) collectColors(child.get());
            if (colorKeyToColor.isEmpty()) return;
            QMenu colorMenu;
            QMap<QAction*, QString> actionToColorKey;
            for (auto it = colorKeyToColor.begin(); it != colorKeyToColor.end(); ++it) {
                QString colorKey = it.key();
                QColor color = it.value();
                QString label = QString("Exclude color: ") + color.name(QColor::HexArgb);
                QAction* colorAct = colorMenu.addAction(label);
                QPixmap pix(16, 16);
                pix.fill(color);
                colorAct->setIcon(QIcon(pix));
                actionToColorKey[colorAct] = colorKey;
            }
            QAction* chosen = colorMenu.exec(QCursor::pos());
            if (chosen && actionToColorKey.contains(chosen)) {
                QString chosenColorKey = actionToColorKey[chosen];
                // Traverse all nodes and exclude only those with the chosen color (do not recurse from a match)
                std::function<void(CadNode*)> excludeByColor = [&](CadNode* n) {
                    if (!n) return;
                    QColor color = n->color.toQColor();
                    QString colorKey = color.name(QColor::HexArgb);
                    if (colorKey == chosenColorKey) {
                        n->excludedFromDecomposition = true;
                        // Do not recurse into children
                        return;
                    }
                    for (auto& child : n->children) excludeByColor(child.get());
                };
                for (auto& child : customModel->getRootNodePointer()->children) excludeByColor(child.get());
                customPartViewer->update();
                customModel->layoutChanged();
            }
        });
        QObject::connect(includeByColorAction2, &QAction::triggered, customModelTreeView, [=]() {
            QModelIndexList selectedIndexes = customModelTreeView->selectionModel()->selectedIndexes();
            if (selectedIndexes.empty()) return;
            QMap<QString, QColor> colorKeyToColor;
            std::function<void(CadNode*)> collectColors;
            collectColors = [&](CadNode* n) {
                if (n) {
                    QColor color = n->color.toQColor();
                    QString colorKey = color.name(QColor::HexArgb);
                    colorKeyToColor[colorKey] = color;
                    for (auto& child : n->children) collectColors(child.get());
                }
            };
            for (const auto& child : customModel->getRootNodePointer()->children) collectColors(child.get());
            if (colorKeyToColor.isEmpty()) return;
            QMenu colorMenu;
            QMap<QAction*, QString> actionToColorKey;
            for (auto it = colorKeyToColor.begin(); it != colorKeyToColor.end(); ++it) {
                QString colorKey = it.key();
                QColor color = it.value();
                QString label = QString("Include color: ") + color.name(QColor::HexArgb);
                QAction* colorAct = colorMenu.addAction(label);
                QPixmap pix(16, 16);
                pix.fill(color);
                colorAct->setIcon(QIcon(pix));
                actionToColorKey[colorAct] = colorKey;
            }
            QAction* chosen = colorMenu.exec(QCursor::pos());
            if (chosen && actionToColorKey.contains(chosen)) {
                QString chosenColorKey = actionToColorKey[chosen];
                std::function<void(CadNode*)> includeByColor = [&](CadNode* n) {
                    if (!n) return;
                    QColor color = n->color.toQColor();
                    QString colorKey = color.name(QColor::HexArgb);
                    if (colorKey == chosenColorKey) {
                        n->excludedFromDecomposition = false;
                        // Do not recurse into children
                        return;
                    }
                    for (auto& child : n->children) includeByColor(child.get());
                };
                for (auto& child : customModel->getRootNodePointer()->children) includeByColor(child.get());
                customPartViewer->update();
                customModel->layoutChanged();
            }
        });

        // Add connection point action for faces
        QModelIndexList selectedIndexes = customModelTreeView->selectionModel()->selectedIndexes();
        if (selectedIndexes.size() == 1) {
            QModelIndex selIdx = selectedIndexes.first();
            CadNode* n = const_cast<CadNode*>(customModel->getNode(selIdx));
            if (n && n->asXCAF() && n->asXCAF()->type == TopAbs_FACE) {
                QAction* addConnectionPointAction = menu.addAction("Add Connection Point");
                QObject::connect(addConnectionPointAction, &QAction::triggered, customModelTreeView, [=]() {
                    QModelIndex ancestorIdx = selIdx.parent();
                    CadNode* physicsAncestor = nullptr;
                    while (ancestorIdx.isValid()) {
                        CadNode* anc = const_cast<CadNode*>(customModel->getNode(ancestorIdx));
                        if (anc && anc->type == CadNodeType::Physics) {
                            physicsAncestor = anc;
                            break;
                        }
                        ancestorIdx = ancestorIdx.parent();
                    }
                    
                    if (physicsAncestor) {
                        // Calculate face center
                        XCAFNodeData* xData = n->asXCAF();
                        TopoDS_Face face = TopoDS::Face(xData->getFace());
                        Bnd_Box bbox;
                        BRepBndLib::Add(face, bbox);
                        Standard_Real xmin, ymin, zmin, xmax, ymax, zmax;
                        bbox.Get(xmin, ymin, zmin, xmax, ymax, zmax);
                        double centerX = (xmin + xmax) * 0.5;
                        double centerY = (ymin + ymax) * 0.5;
                        double centerZ = (zmin + zmax) * 0.5;

                        // Compute world transform of the face node
                        TopLoc_Location faceWorldLoc;
                        {
                            CadNode* cur = n;
                            std::vector<CadNode*> ancestry;
                            while (cur) {
                                ancestry.push_back(cur);
                                cur = const_cast<CadNode*>(customModel->getParentNode(cur));
                            }
                            for (auto it = ancestry.rbegin(); it != ancestry.rend(); ++it) {
                                faceWorldLoc = faceWorldLoc * (*it)->loc;
                            }
                        }
                        // Compute world transform of the physics node
                        TopLoc_Location physicsWorldLoc;
                        {
                            CadNode* cur = physicsAncestor;
                            std::vector<CadNode*> ancestry;
                            while (cur) {
                                ancestry.push_back(cur);
                                cur = const_cast<CadNode*>(customModel->getParentNode(cur));
                            }
                            for (auto it = ancestry.rbegin(); it != ancestry.rend(); ++it) {
                                physicsWorldLoc = physicsWorldLoc * (*it)->loc;
                            }
                        }
                        // Compute face center in world coordinates
                        gp_Pnt faceCenter(centerX, centerY, centerZ);
                        faceCenter.Transform(faceWorldLoc.Transformation());
                        // Create a transform for the face center in world coordinates
                        gp_Trsf faceCenterTrsf;
                        faceCenterTrsf.SetTranslationPart(faceCenter.XYZ());
                        TopLoc_Location faceCenterLoc(faceCenterTrsf);

                        // Create connection point node
                        auto connectionPointNode = std::make_shared<CadNode>();
                        connectionPointNode->name = QString("Connection Point | Face Center: (%1, %2, %3)")
                            .arg(QString::number(faceCenter.X(), 'f', 2))
                            .arg(QString::number(faceCenter.Y(), 'f', 2))
                            .arg(QString::number(faceCenter.Z(), 'f', 2))
                            .toStdString();
                        connectionPointNode->color = CADNodeColor::fromSRGB(0, 255, 0, 255);
                        connectionPointNode->type = CadNodeType::ConnectionPoint;
                        connectionPointNode->visible = true;
                        auto connectionData = std::make_shared<ConnectionPointData>();
                        connectionData->connectionFlags = ConnectionFlags::Cables;
                        connectionData->description = "Connection point created from face center";
                        connectionPointNode->data = connectionData;

                        // Set connection point loc relative to physics node
                        connectionPointNode->loc = physicsWorldLoc.Inverted() * faceCenterLoc;

                        physicsAncestor->children.push_back(connectionPointNode);
                        customModel->layoutChanged();
                        customPartViewer->setRootTreeNode(customModelRoot.get());
                        customPartViewer->update();
                        qDebug() << "Added connection point at:" << faceCenter.X() << faceCenter.Y() << faceCenter.Z();
                    } else {
                        QMessageBox::warning(nullptr, "No Physics Node", "No Physics node found in the ancestry. Please add a Physics node first.");
                    }
                });
                
                // Traverse up to find the nearest ancestor Physics node for extrusion
                QModelIndex ancestorIdx = selIdx.parent();
                CadNode* physicsAncestor = nullptr;
                while (ancestorIdx.isValid()) {
                    CadNode* anc = const_cast<CadNode*>(customModel->getNode(ancestorIdx));
                    if (anc && anc->type == CadNodeType::Physics) {
                        physicsAncestor = anc;
                        break;
                    }
                    ancestorIdx = ancestorIdx.parent();
                }
                if (physicsAncestor) {
                    QAction* extrudeAction = menu.addAction("Extrude Face to Collision Mesh");
                    QObject::connect(extrudeAction, &QAction::triggered, customModelTreeView, [=]() {
                        bool ok = false;
                        double distance = QInputDialog::getDouble(nullptr, "Extrusion Distance", "Enter extrusion distance:", 10.0, -10000.0, 10000.0, 2, &ok);
                        if (!ok) return;
                        XCAFNodeData* xData = n->asXCAF();
                        TopoDS_Face face = TopoDS::Face(xData->getFace());
                        Handle(Geom_Surface) surf = BRep_Tool::Surface(face);
                        Handle(Geom_Plane) plane = Handle(Geom_Plane)::DownCast(surf);
                        if (plane.IsNull()) {
                            QMessageBox::warning(nullptr, "Not Planar", "Selected face is not planar and cannot be extruded as a convex mesh.");
                            return;
                        }
                        gp_Pln pln = plane->Pln();
                        gp_Dir normal = pln.Axis().Direction();
                        TopoDS_Wire outerWire = BRepTools::OuterWire(face);
                        std::vector<gp_Pnt> baseVerts;
                        for (TopExp_Explorer exp(outerWire, TopAbs_VERTEX); exp.More(); exp.Next()) {
                            TopoDS_Vertex v = TopoDS::Vertex(exp.Current());
                            baseVerts.push_back(BRep_Tool::Pnt(v));
                        }
                        if (baseVerts.size() < 3) {
                            QMessageBox::warning(nullptr, "Invalid Face", "Face does not have enough vertices to extrude.");
                            return;
                        }
                        std::vector<std::array<double, 3>> vertices;
                        for (const auto& p : baseVerts) {
                            vertices.push_back({p.X(), p.Y(), p.Z()});
                        }
                        for (const auto& p : baseVerts) {
                            gp_Pnt p2 = p.Translated(normal.XYZ() * distance);
                            vertices.push_back({p2.X(), p2.Y(), p2.Z()});
                        }
                        std::vector<std::array<uint32_t, 3>> indices;
                        size_t N = baseVerts.size();
                        for (size_t i = 1; i + 1 < N; ++i) {
                            indices.push_back({0, uint32_t(i), uint32_t(i+1)});
                        }
                        for (size_t i = 1; i + 1 < N; ++i) {
                            indices.push_back({uint32_t(N), uint32_t(N+i+1), uint32_t(N+i)});
                        }
                        for (size_t i = 0; i < N; ++i) {
                            size_t next = (i+1)%N;
                            uint32_t a = uint32_t(i);
                            uint32_t b = uint32_t(next);
                            uint32_t c = uint32_t(N+next);
                            uint32_t d = uint32_t(N+i);
                            indices.push_back({a, b, c});
                            indices.push_back({a, c, d});
                        }
                        PhysicsNodeData* physData = physicsAncestor->asPhysics();
                        if (!physData) return;
                        ConvexHullData hull;
                        hull.vertices = vertices;
                        hull.indices = indices;
                        physData->hulls.push_back(std::move(hull));
                        physData->convexHullGenerated = true;
                        customPartViewer->update();
                        QMessageBox::information(nullptr, "Extrusion Complete", "Extruded mesh added to collision data.");
                    });
                }
            }
        }

        // Align two faces by moving the parent solid/compound of the second face
        if (selectedIndexes.size() == 2) {
            CadNode* nodeA = const_cast<CadNode*>(customModel->getNode(selectedIndexes[0]));
            CadNode* nodeB = const_cast<CadNode*>(customModel->getNode(selectedIndexes[1]));
            if (nodeA && nodeB && nodeA->asXCAF() && nodeB->asXCAF() && nodeA->asXCAF()->type == TopAbs_FACE && nodeB->asXCAF()->type == TopAbs_FACE) {
                QAction* alignFacesAction = menu.addAction("Align Selected Faces (Move Second)");
                QObject::connect(alignFacesAction, &QAction::triggered, customModelTreeView, [=]() {
                    // Utility: Compute the global transform (accumulated from root) for a node
                    std::function<TopLoc_Location(CadNode*)> getGlobalTransform = [&](CadNode* node) -> TopLoc_Location {
                        TopLoc_Location acc;
                        std::vector<CadNode*> ancestry;
                        CadNode* cur = node;
                        while (cur) {
                            ancestry.push_back(cur);
                            cur = const_cast<CadNode*>(customModel->getParentNode(cur));
                        }
                        for (auto it = ancestry.rbegin(); it != ancestry.rend(); ++it) {
                            acc = acc * (*it)->loc;
                        }
                        return acc;
                    };
                    // Find parent physics nodes
                    auto findParentPhysicsObject = [&](CadNode* n) -> CadNode* {
                        const CadNode* parent = customModel->getParentNode(n);
                        while (parent) {
                            if (parent->type == CadNodeType::Physics)
                                return const_cast<CadNode*>(parent);
                            parent = customModel->getParentNode(parent);
                        }
                        return nullptr;
                    };
                    CadNode* parentA = findParentPhysicsObject(nodeA);
                    CadNode* parentB = findParentPhysicsObject(nodeB);
                    if (!parentA || !parentB) {
                        QMessageBox::warning(nullptr, "Align Faces", "Could not find parent solid/compound for both faces.");
                        return;
                    }
                    // Get face geometry (center and normal)
                    auto getFaceCenterNormal = [](CadNode* faceNode, gp_Pnt& center, gp_Dir& normal) {
                        XCAFNodeData* xData = faceNode->asXCAF();
                        TopoDS_Face face = TopoDS::Face(xData->getFace());
                        Bnd_Box bbox;
                        BRepBndLib::Add(face, bbox);
                        Standard_Real xmin, ymin, zmin, xmax, ymax, zmax;
                        bbox.Get(xmin, ymin, zmin, xmax, ymax, zmax);
                        center = gp_Pnt((xmin + xmax) * 0.5, (ymin + ymax) * 0.5, (zmin + zmax) * 0.5);
                        TopLoc_Location loc;
                        Handle(Geom_Surface) surf = BRep_Tool::Surface(face, loc);
                        Standard_Real umin, umax, vmin, vmax;
                        surf->Bounds(umin, umax, vmin, vmax);
                        Standard_Real u = (umin + umax) * 0.5;
                        Standard_Real v = (vmin + vmax) * 0.5;
                        gp_Pnt p;
                        gp_Vec d1u, d1v;
                        surf->D1(u, v, p, d1u, d1v);
                        gp_Vec n = d1u.Crossed(d1v);
                        if (n.Magnitude() > 1e-10)
                            normal = gp_Dir(n);
                        else
                            normal = gp_Dir(0, 0, 1); // fallback
                    };
                    gp_Pnt centerA, centerB;
                    gp_Dir normalA, normalB;
                    getFaceCenterNormal(nodeA, centerA, normalA);
                    getFaceCenterNormal(nodeB, centerB, normalB);
                    // Compute global transforms
                    TopLoc_Location faceBGlobalLoc = getGlobalTransform(nodeB);
                    TopLoc_Location faceAGlobalLoc = getGlobalTransform(nodeA);
                    TopLoc_Location parentBGlobal = getGlobalTransform(parentB);
                    CadNode* parentOfB = const_cast<CadNode*>(customModel->getParentNode(parentB));
                    TopLoc_Location parentOfBGlobal = parentOfB ? getGlobalTransform(parentOfB) : TopLoc_Location();
                    // Transform faceB's center and normal to world
                    gp_Pnt centerB_world = centerB.Transformed(faceBGlobalLoc.Transformation());
                    gp_Pnt centerA_world = centerA.Transformed(faceAGlobalLoc.Transformation());
                    gp_Vec normalB_world(normalB.X(), normalB.Y(), normalB.Z());
                    normalB_world.Transform(faceBGlobalLoc.Transformation());
                    gp_Vec normalA_world(normalA.X(), normalA.Y(), normalA.Z());
                    normalA_world.Transform(faceAGlobalLoc.Transformation());
                    // Compute rotation to align normals in world
                    gp_Vec axis = normalB_world.Crossed(normalA_world);
                    Standard_Real angle = normalB_world.Angle(normalA_world);
                    gp_Trsf rotTrsf;
                    if (axis.SquareMagnitude() > 1e-10 && angle > 1e-10) {
                        rotTrsf.SetRotation(gp_Ax1(centerB_world, axis), angle);
                    } else {
                        rotTrsf = gp_Trsf();
                    }
                    // Rotate centerB_world to new position
                    gp_Pnt centerB_rot = centerB_world.Transformed(rotTrsf);
                    gp_Vec translation(centerB_rot, centerA_world);
                    gp_Trsf alignTrsf;
                    alignTrsf.SetRotation(rotTrsf.GetRotation());
                    alignTrsf.SetTranslationPart(translation);
                    // The transform to apply to parentB's global transform
                    gp_Trsf newParentBGlobalTrsf = alignTrsf * parentBGlobal.Transformation();
                    TopLoc_Location newParentBGlobal(newParentBGlobalTrsf);
                    // Set parentB->loc so that parentOfBGlobal * parentB->loc == newParentBGlobal
                    parentB->loc = TopLoc_Location(parentOfBGlobal.Transformation().Inverted() * newParentBGlobal.Transformation());
                    // Debug output
                    qDebug() << "centerA_world:" << centerA_world.X() << centerA_world.Y() << centerA_world.Z();
                    qDebug() << "centerB_world (before):" << centerB_world.X() << centerB_world.Y() << centerB_world.Z();
                    qDebug() << "normalA_world:" << normalA_world.X() << normalA_world.Y() << normalA_world.Z();
                    qDebug() << "normalB_world (before):" << normalB_world.X() << normalB_world.Y() << normalB_world.Z();
                    customPartViewer->markCacheDirty();
                    customModel->layoutChanged();
                    customPartViewer->setRootTreeNode(customModelRoot.get());
                    customPartViewer->update();
                    QMessageBox::information(nullptr, "Align Faces", "Moved second object so faces touch.");
                });
            }
        }
        if (node->type == CadNodeType::Rail) {
            QAction* editRailJsonAction = menu.addAction("Edit Rail JSON...");
            QObject::connect(editRailJsonAction, &QAction::triggered, customModelTreeView, [=]() {
                RailNodeData* railData = node->asRail();
                if (!railData) return;
                // Parse JSON if present, otherwise use defaults
                QJsonObject jsonObj;
                if (!railData->jsonString.isEmpty()) {
                    QJsonDocument doc = QJsonDocument::fromJson(railData->jsonString.toUtf8());
                    if (doc.isObject()) jsonObj = doc.object();
                }
                // Extract or default fields
                QVector3D axis = railData->axisOfTravel;
                double joint = 0.0;
                double travel = railData->travelLength;
                int numSeg = railData->numSegments;
                if (jsonObj.contains("axisOfTravel")) {
                    QJsonArray arr = jsonObj["axisOfTravel"].toArray();
                    if (arr.size() == 3) axis = QVector3D(arr[0].toDouble(), arr[1].toDouble(), arr[2].toDouble());
                }
                if (jsonObj.contains("buildJointPosition")) {
                    joint = jsonObj["buildJointPosition"].toDouble();
                }
                if (jsonObj.contains("travelLength")) travel = jsonObj["travelLength"].toDouble();
                if (jsonObj.contains("numSegments")) numSeg = jsonObj["numSegments"].toInt();
                // Dialog
                QDialog* dialog = new QDialog(nullptr);
                dialog->setWindowTitle("Edit Rail Properties");
                QFormLayout* form = new QFormLayout(dialog);
                // Axis of travel
                QLineEdit* axisEdit = new QLineEdit(QString("%1, %2, %3").arg(axis.x()).arg(axis.y()).arg(axis.z()));
                form->addRow("Axis of Travel (x, y, z):", axisEdit);
                // Build joint position (scalar)
                QDoubleSpinBox* jointSpin = new QDoubleSpinBox();
                jointSpin->setRange(-1e6, 1e6);
                jointSpin->setDecimals(3);
                jointSpin->setValue(joint);
                form->addRow("Build Joint Position (distance along rail):", jointSpin);
                // Travel length
                QDoubleSpinBox* travelSpin = new QDoubleSpinBox();
                travelSpin->setRange(-1e6, 1e6);
                travelSpin->setDecimals(3);
                travelSpin->setValue(travel);
                form->addRow("Travel Length:", travelSpin);
                // Number of segments
                QSpinBox* numSegSpin = new QSpinBox();
                numSegSpin->setRange(1, 1000);
                numSegSpin->setValue(numSeg);
                form->addRow("Number of Segments:", numSegSpin);
                // Raw JSON view
                QTextEdit* textEdit = new QTextEdit(dialog);
                textEdit->setPlainText(railData->jsonString);
                form->addRow("Raw JSON:", textEdit);
                // Buttons
                QHBoxLayout* buttonLayout = new QHBoxLayout();
                QPushButton* okBtn = new QPushButton("OK");
                QPushButton* cancelBtn = new QPushButton("Cancel");
                buttonLayout->addWidget(okBtn);
                buttonLayout->addWidget(cancelBtn);
                form->addRow(buttonLayout);
                dialog->setLayout(form);
                QObject::connect(okBtn, &QPushButton::clicked, dialog, [=]() {
                    // Parse axis
                    QVector3D newAxis = axis;
                    QStringList axisParts = axisEdit->text().split(",");
                    if (axisParts.size() == 3) {
                        bool ok1, ok2, ok3;
                        double x = axisParts[0].toDouble(&ok1);
                        double y = axisParts[1].toDouble(&ok2);
                        double z = axisParts[2].toDouble(&ok3);
                        if (ok1 && ok2 && ok3) newAxis = QVector3D(x, y, z);
                    }
                    double newJoint = jointSpin->value();
                    double newTravel = travelSpin->value();
                    int newNumSeg = numSegSpin->value();
                    // Update RailNodeData fields
                    railData->axisOfTravel = newAxis;
                    railData->buildJointPosition = QVector3D(newJoint, 0, 0); // Store as x only
                    railData->travelLength = newTravel;
                    railData->numSegments = newNumSeg;
                    // Compose JSON
                    QJsonObject newObj;
                    QJsonArray axisArr; axisArr << newAxis.x() << newAxis.y() << newAxis.z();
                    newObj["axisOfTravel"] = axisArr;
                    newObj["buildJointPosition"] = newJoint;
                    newObj["travelLength"] = newTravel;
                    newObj["numSegments"] = newNumSeg;
                    railData->jsonString = QJsonDocument(newObj).toJson(QJsonDocument::Compact);
                    textEdit->setPlainText(railData->jsonString);
                    customModel->layoutChanged();
                    customPartViewer->update();
                    dialog->accept();
                });
                QObject::connect(cancelBtn, &QPushButton::clicked, dialog, &QDialog::reject);
                dialog->exec();
            });

            // --- Add Rail to Physics Preview ---
            QAction* addToPhysicsPreviewAction = menu.addAction("Add Rail to Physics Preview");
            QObject::connect(addToPhysicsPreviewAction, &QAction::triggered, customModelTreeView, [=]() {
                addRailToPhysicsPreview(node, physicsPreviewRoot);
                physicsPreviewModel->layoutChanged();
                physicsPreviewViewer->setRootTreeNode(physicsPreviewRoot.get());
                physicsPreviewViewer->update();
            });
        }
        if (!menu.isEmpty()) menu.exec(customModelTreeView->viewport()->mapToGlobal(pos));
    });

    // Insert the buttons above the custom model tree in the layout
    QVBoxLayout* customModelLayout = new QVBoxLayout;
    customModelLayout->addWidget(saveRailBtn);
    customModelLayout->addWidget(loadRailBtn);
    customModelLayout->addWidget(customModelTreeView);
    QWidget* customModelWidget = new QWidget;
    customModelWidget->setLayout(customModelLayout);
    outerSplitter->insertWidget(0, customModelWidget);

    mainWindow.resize(1200, 800);
    mainWindow.show();

    // Add this after the context menu setup for physicsPreviewTreeView
    physicsPreviewTreeView->setContextMenuPolicy(Qt::CustomContextMenu);
    QObject::connect(physicsPreviewTreeView, &QTreeView::customContextMenuRequested, physicsPreviewTreeView, [=](const QPoint& pos) {
        QModelIndex idx = physicsPreviewTreeView->indexAt(pos);
        if (!idx.isValid()) return;
        CadNode* node = const_cast<CadNode*>(physicsPreviewModel->getNode(idx));
        if (!node) return;
        QMenu menu;
        // Only show for physics nodes
        if (node->type == CadNodeType::Physics) {
            // Add collision mesh visibility toggle
            QAction* toggleCollisionMeshAction = nullptr;
            if (node->asPhysics()->collisionMeshVisible) {
                toggleCollisionMeshAction = menu.addAction("Hide Collision Mesh");
            } else {
                toggleCollisionMeshAction = menu.addAction("Show Collision Mesh");
            }
            QObject::connect(toggleCollisionMeshAction, &QAction::triggered, physicsPreviewTreeView, [=]() {
                node->asPhysics()->collisionMeshVisible = !node->asPhysics()->collisionMeshVisible;
                physicsPreviewModel->dataChanged(idx, idx);
                physicsPreviewViewer->update();
            });
        }
        if (!menu.isEmpty()) menu.exec(physicsPreviewTreeView->viewport()->mapToGlobal(pos));
    });

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
