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

#include "CadNode.h"
#include "XCAFLabelTreeModel.h"
#include "CadOpenGLWidget.h" // For SelectionMode enum
#include "CustomModelTreeModel.h"

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

// Improved tree building function for STEP 214 compatibility
std::shared_ptr<CadNode> build_tree_xcaf(const TDF_Label& label,
                   const Handle(XCAFDoc_ShapeTool)& shapeTool,
                   const Handle(XCAFDoc_ColorTool)& colorTool,
                   const CADNodeColor& parentColor,
                   const TopLoc_Location& parentLoc)
{
    TopoDS_Shape shape = shapeTool->GetShape(label);
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
    
    if (!shape.IsNull()) {
        node->type = shape.ShapeType();
    } else {
        node->type = TopAbs_SHAPE;
    }
    
    QString typeName = shapeTypeToString(node->type);
    
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
    if (shapeTool->IsReference(label)) {
        TDF_Label refLabel;
        if (shapeTool->GetReferredShape(label, refLabel)) {
            qDebug() << "Following reference from label" << label.Tag() << "to" << refLabel.Tag();
            TopLoc_Location refLoc = nodeLoc; // Use the effective transform
            
            // For references, we want to preserve the color from the reference label
            // but also allow the referred shape to have its own colors
            // So we pass the current color as the parent color for inheritance
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
        node->shapeData.shape = shape;
    }
    
    // Add face/edge children for solids that don't have other solid children
    // This allows faces/edges to be shown even when the solid has reference children
    if (!shape.IsNull() && shape.ShapeType() == TopAbs_SOLID && !isAssembly(label, shapeTool)) {
        // Check if any existing children are solids (to avoid intermediate solids)
        bool hasSolidChildren = false;
        for (const auto& child : node->children) {
            if (child->type == TopAbs_SOLID) {
                hasSolidChildren = true;
                break;
            }
        }
        
        // Add faces and edges if this solid doesn't have other solid children
        // Note: We removed the isCompoundAssembly check to allow faces on solids within compounds
        if (!hasSolidChildren) {
            // Add face children with proper color extraction
            int faceIdx = 0;
            int faceCount = 0;
            for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next(), ++faceIdx) {
                TopoDS_Face face = TopoDS::Face(exp.Current());
                auto faceNode = std::make_shared<CadNode>();
                faceNode->type = TopAbs_FACE;
                faceNode->shapeData.shape = face;
                // Use the enhanced color extraction for faces
                faceNode->color = get_shape_color(face, label, shapeTool, colorTool, color);
                faceNode->loc = nodeLoc;
                faceNode->name = QString("Face %1 of %2").arg(faceIdx).arg(QString::fromStdString(node->name)).toStdString();
                node->children.push_back(faceNode);
                faceCount++;
            }
            
            // Add edge children
            int edgeIdx = 0;
            int edgeCount = 0;
            for (TopExp_Explorer exp(shape, TopAbs_EDGE); exp.More(); exp.Next(), ++edgeIdx) {
                TopoDS_Shape edge = exp.Current();
                auto edgeNode = std::make_shared<CadNode>();
                edgeNode->type = TopAbs_EDGE;
                edgeNode->shapeData.shape = edge;
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

// Helper to collect all face nodes under a given node
static void collectFaceNodes(CadNode* node, std::vector<CadNode*>& out) {
    if (!node) return;
    if (node->type == TopAbs_FACE && node->visible) {
        out.push_back(node);
    }
    for (const auto& child : node->children) {
        if (child) collectFaceNodes(child.get(), out);
    }
}

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    QMainWindow mainWindow;
    mainWindow.setWindowTitle("QtCadViewer - OCC C++");

    // Prompt for STEP file on startup
    QString stepFile = QFileDialog::getOpenFileName(
        nullptr, "Open STEP File", "", "STEP Files (*.step *.stp)");
    if (stepFile.isEmpty()) {
        QMessageBox::warning(nullptr, "No File", "No STEP file selected. Exiting.");
        return 0;
    }

    // OCC: Load STEP file into XCAF document
    Handle(XCAFApp_Application) appOCC = XCAFApp_Application::GetApplication();
    Handle(TDocStd_Document) doc;
    appOCC->NewDocument("BinXCAF", doc);
    STEPCAFControl_Reader reader;
    reader.SetColorMode(true);
    reader.SetNameMode(true);
    reader.SetLayerMode(true);
    
    // Enhanced STEP 214 handling - using available methods
    
    if (reader.ReadFile(stepFile.toStdString().c_str()) != IFSelect_RetDone) {
        QMessageBox::critical(nullptr, "Error", "Failed to read STEP file.");
        return 1;
    }
    if (!reader.Transfer(doc)) {
        QMessageBox::critical(nullptr, "Error", "Failed to transfer STEP file.");
        return 1;
    }
    
    // Log STEP file information for debugging
    qDebug() << "STEP file loaded successfully:" << stepFile;
    qDebug() << "Document has" << doc->Main().NbChildren() << "root children";

    // Get shape and color tools from the document
    Handle(XCAFDoc_ShapeTool) shapeTool = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
    Handle(XCAFDoc_ColorTool) colorTool = XCAFDoc_DocumentTool::ColorTool(doc->Main());

    // Build tree and collect faces
    std::vector<ColoredFace> faces;
    CADNodeColor defaultColor = CADNodeColor::fromSRGB(200, 200, 200);
    TopLoc_Location identityLoc;
    TDF_LabelSequence roots;
    shapeTool->GetFreeShapes(roots);
    
    qDebug() << "Found" << roots.Length() << "free shapes in STEP file";
    
    auto root = std::make_unique<CadNode>();
    root->name = "Root";
    root->color = defaultColor;
    root->loc = identityLoc;
    
    for (Standard_Integer i = 1; i <= roots.Length(); ++i) {
        TDF_Label rootLabel = roots.Value(i);
        qDebug() << "Processing root shape" << i << "with tag" << rootLabel.Tag();
        
        auto child = build_tree_xcaf(rootLabel, shapeTool, colorTool, defaultColor, identityLoc);
        if (child) {
            root->children.push_back(child);
            qDebug() << "Successfully added root shape" << i << "to tree";
        } else {
            qDebug() << "Failed to build tree for root shape" << i;
        }
    }
    
    // For STEP 214 files, also explore the main document structure to find assemblies
    // that might not be in the free shapes list
    qDebug() << "Exploring main document structure for STEP 214 assemblies...";
    std::function<void(const TDF_Label&)> exploreDocument = [&](const TDF_Label& label) {
        if (label.IsNull()) return;
        
        // Check if this label has a shape and is not already processed
        TopoDS_Shape shape = shapeTool->GetShape(label);
        if (!shape.IsNull()) {
            // Check if this is an assembly compound
            if (shape.ShapeType() == TopAbs_COMPOUND) {
                bool hasDirectGeometry = false;
                int refChildCount = 0;
                
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
                
                // If this looks like an assembly, add it to the tree
                if (!hasDirectGeometry && refChildCount > 0) {
                    qDebug() << "Found additional assembly at label" << label.Tag() << "with" << refChildCount << "references";
                    auto assemblyChild = build_tree_xcaf(label, shapeTool, colorTool, defaultColor, identityLoc);
                    if (assemblyChild) {
                        root->children.push_back(assemblyChild);
                    }
                }
            }
        }
        
        // Recurse into children
        for (TDF_ChildIterator it(label); it.More(); it.Next()) {
            exploreDocument(it.Value());
        }
    };
    
    exploreDocument(doc->Main());
    
    qDebug() << "Tree building complete. Root has" << root->children.size() << "children";
    qDebug() << "Total faces collected:" << faces.size();

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

    // --- Custom Model Tree ---
    // Create persistent root node for the custom model
    std::shared_ptr<CadNode> customModelRoot = std::make_shared<CadNode>();
    customModelRoot->name = "Custom Model Root";
    CustomModelTreeModel* customModel = new CustomModelTreeModel(customModelRoot);
    QTreeView* customModelTreeView = new QTreeView;
    customModelTreeView->setModel(customModel);
    customModelTreeView->setHeaderHidden(false);
    customModelTreeView->setMinimumWidth(220);
    outerSplitter->addWidget(customModelTreeView);

    // --- CAD/XCAF Tabs and OpenGL Viewer ---
    outerSplitter->addWidget(splitter);

    // Create tab widget for the left side
    QTabWidget *tabWidget = new QTabWidget;
    tabWidget->setMinimumWidth(300);
    splitter->addWidget(tabWidget);

    // Model/view tree for first tab
    CadTreeModel *model = new CadTreeModel(std::move(root));
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

    // Custom model part OpenGL widget
    CadOpenGLWidget *customPartViewer = new CadOpenGLWidget;
    customPartViewer->setRootTreeNode(customModel->getRootNodePointer());
    viewTabWidget->addTab(customPartViewer, "Custom Model Part");

    // Update custom part viewer when selection changes
    auto updateCustomPartViewer = [&]() {
        std::vector<CadNode*> faceNodes;
        // Collect all selected nodes' faces
        QModelIndexList selectedIndexes = customModelTreeView->selectionModel()->selectedIndexes();
        for (const QModelIndex& idx : selectedIndexes) {
            CadNode* node = static_cast<CadNode*>(idx.internalPointer());
            if (node) collectFaceNodes(node, faceNodes);
        }
        //customPartViewer->setCustomNodes(faceNodes); // Uncomment if setCustomNodes is implemented
        customPartViewer->markCacheDirty();
    };

    // Connect custom model tree selection to customPartViewer
    QObject::connect(customModelTreeView->selectionModel(), &QItemSelectionModel::selectionChanged, customPartViewer, [=](const QItemSelection &selected, const QItemSelection &/*deselected*/) {
        updateCustomPartViewer();
    });
    
    // Force the viewer to update and render the geometry
    viewer->update();

    // Add selection mode buttons
    QPushButton* noSelectionBtn = new QPushButton("No Selection");
    QPushButton* faceSelectionBtn = new QPushButton("Face Selection");
    QPushButton* edgeSelectionBtn = new QPushButton("Edge Selection");
    
    // Style the buttons to show current selection mode
    noSelectionBtn->setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; }");
    
    buttonLayout->addWidget(noSelectionBtn);
    buttonLayout->addWidget(faceSelectionBtn);
    buttonLayout->addWidget(edgeSelectionBtn);
    
    // Connect selection mode buttons
    QObject::connect(noSelectionBtn, &QPushButton::clicked, [=]() {
        viewer->setSelectionMode(SelectionMode::None);
        noSelectionBtn->setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; }");
        faceSelectionBtn->setStyleSheet("QPushButton { background-color: #f0f0f0; color: black; }");
        edgeSelectionBtn->setStyleSheet("QPushButton { background-color: #f0f0f0; color: black; }");
    });
    
    QObject::connect(faceSelectionBtn, &QPushButton::clicked, [=]() {
        viewer->setSelectionMode(SelectionMode::Faces);
        noSelectionBtn->setStyleSheet("QPushButton { background-color: #f0f0f0; color: black; }");
        faceSelectionBtn->setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; }");
        edgeSelectionBtn->setStyleSheet("QPushButton { background-color: #f0f0f0; color: black; }");
    });
    
    QObject::connect(edgeSelectionBtn, &QPushButton::clicked, [=]() {
        viewer->setSelectionMode(SelectionMode::Edges);
        noSelectionBtn->setStyleSheet("QPushButton { background-color: #f0f0f0; color: black; }");
        faceSelectionBtn->setStyleSheet("QPushButton { background-color: #f0f0f0; color: black; }");
        edgeSelectionBtn->setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; }");
    });
    
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
    
    // Connect face selection in viewer to tree selection
    QObject::connect(viewer, &CadOpenGLWidget::facePicked, treeView, [=](CadNode* node) {
        QModelIndex idx = model->indexForNode(node);
        if (idx.isValid()) {
            // Expand all parents so the item is visible
            QModelIndex parentIdx = idx.parent();
            while (parentIdx.isValid()) {
                treeView->expand(parentIdx);
                parentIdx = parentIdx.parent();
            }
            treeView->setCurrentIndex(idx);
            treeView->scrollTo(idx);
        }
    });
    
    // Connect edge selection in viewer to tree selection
    QObject::connect(viewer, &CadOpenGLWidget::edgePicked, treeView, [=](CadNode* node) {
        QModelIndex idx = model->indexForNode(node);
        if (idx.isValid()) {
            // Expand all parents so the item is visible
            QModelIndex parentIdx = idx.parent();
            while (parentIdx.isValid()) {
                treeView->expand(parentIdx);
                parentIdx = parentIdx.parent();
            }
            treeView->setCurrentIndex(idx);
            treeView->scrollTo(idx);
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
    auto labelRoot = std::make_unique<XCAFLabelNode>(doc->Main());
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
    tabWidget->addTab(labelTreeView, "XCAF Labels (Enhanced)");
    


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
        if (node->type == TopAbs_SOLID) {
            menu.addSeparator();
            debugAction = menu.addAction("Debug Solid Info");
        } else if (node->type == TopAbs_FACE) {
            menu.addSeparator();
            debugFaceAction = menu.addAction("Debug Face Info");
        }

        // --- Unselect by Color ---
        QAction* unselectByColorAction = menu.addAction("Unselect All by Color");
        QObject::connect(unselectByColorAction, &QAction::triggered, treeView, [=]() {
            // Gather all selected nodes and their colors
            QModelIndexList selectedIndexes = treeView->selectionModel()->selectedIndexes();
            if (selectedIndexes.empty()) return;
            QMap<QString, QList<QModelIndex>> colorToIndexes;
            QMap<QString, QColor> colorKeyToColor;
            for (const QModelIndex& selIdx : selectedIndexes) {
                CadNode* n = model->getNode(selIdx);
                if (n) {
                    QColor color = n->color.toQColor();
                    QString colorKey = color.name(QColor::HexArgb);
                    colorToIndexes[colorKey].append(selIdx);
                    colorKeyToColor[colorKey] = color;
                }
            }
            if (colorToIndexes.isEmpty()) return;
            if (colorToIndexes.size() == 1) {
                // Only one color, unselect all
                QList<QModelIndex> toUnselect = colorToIndexes.first();
                QItemSelectionModel* selModel = treeView->selectionModel();
                for (const QModelIndex& idx : toUnselect) {
                    selModel->select(idx, QItemSelectionModel::Deselect);
                }
            } else {
                // Multiple colors, show a submenu to pick
                QMenu colorMenu;
                QMap<QAction*, QString> actionToColorKey;
                for (auto it = colorToIndexes.begin(); it != colorToIndexes.end(); ++it) {
                    QString colorKey = it.key();
                    QColor color = colorKeyToColor[colorKey];
                    QString label = QString("Unselect color: ") + color.name(QColor::HexArgb) + QString(" (%1)").arg(it.value().size());
                    QAction* colorAct = colorMenu.addAction(label);
                    QPixmap pix(16, 16);
                    pix.fill(color);
                    colorAct->setIcon(QIcon(pix));
                    actionToColorKey[colorAct] = colorKey;
                }
                QAction* chosen = colorMenu.exec(QCursor::pos());
                if (chosen && actionToColorKey.contains(chosen)) {
                    QString chosenColorKey = actionToColorKey[chosen];
                    QList<QModelIndex> toUnselect = colorToIndexes[chosenColorKey];
                    QItemSelectionModel* selModel = treeView->selectionModel();
                    for (const QModelIndex& idx : toUnselect) {
                        selModel->select(idx, QItemSelectionModel::Deselect);
                    }
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
            newPart->name = "Carriage";
            newPart->loc = nodes[0]->loc;
            newPart->color = nodes[0]->color;
            newPart->type = TopAbs_COMPOUND; // Assuming a compound for a part
            newPart->shapeData.shape = TopoDS_Compound(); // Placeholder for compound
            // Add selected nodes as children of the part node
            for (CadNode* n : nodes) {
                if (n) newPart->children.push_back(std::make_shared<CadNode>(*n));
            }
            customModelRoot->children.push_back(newPart);
            customModel->layoutChanged(); // Notify model of structure change
            customPartViewer->setRootTreeNode(customModel->getRootNodePointer()); // Update viewer's root node
            updateCustomPartViewer();
        });
        QObject::connect(addStartSegAction, &QAction::triggered, treeView, [=]() {
            auto nodes = getSelectedNodes();
            if (nodes.empty()) return;
            std::shared_ptr<CadNode> newPart = std::make_shared<CadNode>();
            newPart->name = "Start Segment";
            newPart->loc = nodes[0]->loc;
            newPart->color = nodes[0]->color;
            newPart->type = TopAbs_COMPOUND; // Assuming a compound for a part
            newPart->shapeData.shape = TopoDS_Compound(); // Placeholder for compound
            for (CadNode* n : nodes) {
                if (n) newPart->children.push_back(std::make_shared<CadNode>(*n));
            }
            customModelRoot->children.push_back(newPart);
            customModel->layoutChanged(); // Notify model of structure change
            customPartViewer->setRootTreeNode(customModel->getRootNodePointer()); // Update viewer's root node
            updateCustomPartViewer();
        });
        QObject::connect(addEndSegAction, &QAction::triggered, treeView, [=]() {
            auto nodes = getSelectedNodes();
            if (nodes.empty()) return;
            std::shared_ptr<CadNode> newPart = std::make_shared<CadNode>();
            newPart->name = "End Segment";
            newPart->loc = nodes[0]->loc;
            newPart->color = nodes[0]->color;
            newPart->type = TopAbs_COMPOUND; // Assuming a compound for a part
            newPart->shapeData.shape = TopoDS_Compound(); // Placeholder for compound
            for (CadNode* n : nodes) {
                if (n) newPart->children.push_back(std::make_shared<CadNode>(*n));
            }
            customModelRoot->children.push_back(newPart);
            customModel->layoutChanged(); // Notify model of structure change
            customPartViewer->setRootTreeNode(customModel->getRootNodePointer()); // Update viewer's root node
            updateCustomPartViewer();
        });
        QObject::connect(addMiddleSegAction, &QAction::triggered, treeView, [=]() {
            auto nodes = getSelectedNodes();
            if (nodes.empty()) return;
            std::shared_ptr<CadNode> newPart = std::make_shared<CadNode>();
            newPart->name = "Middle Segment";
            newPart->loc = nodes[0]->loc;
            newPart->color = nodes[0]->color;
            newPart->type = TopAbs_COMPOUND; // Assuming a compound for a part
            newPart->shapeData.shape = TopoDS_Compound(); // Placeholder for compound
            for (CadNode* n : nodes) {
                if (n) newPart->children.push_back(std::make_shared<CadNode>(*n));
            }
            customModelRoot->children.push_back(newPart);
            customModel->layoutChanged(); // Notify model of structure change
            customPartViewer->setRootTreeNode(customModel->getRootNodePointer()); // Update viewer's root node
            updateCustomPartViewer();
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
            customPartViewer->setRootTreeNode(customModel->getRootNodePointer()); // Update viewer's root node
            updateCustomPartViewer();
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
            customPartViewer->setRootTreeNode(customModel->getRootNodePointer()); // Update viewer's root node
            updateCustomPartViewer();
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
            customPartViewer->setRootTreeNode(customModel->getRootNodePointer()); // Update viewer's root node
            updateCustomPartViewer();
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
            customPartViewer->setRootTreeNode(customModel->getRootNodePointer()); // Update viewer's root node
            updateCustomPartViewer();
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
                if (!solidNode || solidNode->type != TopAbs_SOLID) return;
                
                qDebug() << "=== SOLID DEBUG INFO ===";
                qDebug() << "Node name:" << QString::fromStdString(solidNode->name);
                qDebug() << "Node type:" << shapeTypeToString(solidNode->type);
                qDebug() << "Has shape:" << !solidNode->shapeData.shape.IsNull();
                qDebug() << "Visible:" << solidNode->visible;
                qDebug() << "Color:" << solidNode->color.r << solidNode->color.g << solidNode->color.b << solidNode->color.a;
                
                // Check if solid has faces
                if (!solidNode->shapeData.shape.IsNull()) {
                    TopoDS_Shape shape = solidNode->shapeData.shape;
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
                        if (child->type == TopAbs_FACE) treeFaceCount++;
                        if (child->type == TopAbs_EDGE) treeEdgeCount++;
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
                        qDebug() << "Parent type:" << shapeTypeToString(parentNode->type);
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
                if (!faceNode || faceNode->type != TopAbs_FACE) return;
                
                qDebug() << "=== FACE DEBUG INFO ===";
                qDebug() << "Node name:" << QString::fromStdString(faceNode->name);
                qDebug() << "Node type:" << shapeTypeToString(faceNode->type);
                qDebug() << "Has shape:" << !faceNode->shapeData.shape.IsNull();
                qDebug() << "Visible:" << faceNode->visible;
                qDebug() << "Color:" << faceNode->color.r << faceNode->color.g << faceNode->color.b << faceNode->color.a;
                
                // Check face geometry
                if (!faceNode->shapeData.shape.IsNull()) {
                    TopoDS_Face face = TopoDS::Face(faceNode->shapeData.shape);
                    
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
                        qDebug() << "Parent type:" << shapeTypeToString(parentNode->type);
                        qDebug() << "Parent name:" << QString::fromStdString(parentNode->name);
                        qDebug() << "Parent has" << parentNode->children.size() << "children";
                    }
                }
                
                qDebug() << "=== END FACE DEBUG INFO ===";
            });
        }
        
        menu.exec(treeView->viewport()->mapToGlobal(pos));
    });

    mainWindow.resize(1200, 800);
    mainWindow.show();
    return app.exec();
}
