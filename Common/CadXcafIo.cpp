// OpenCascade and XCAF: reading a STEP or JSON/BIN document into a CadNode tree,
// querying labels, colours and transforms, and writing a document back out.
#include "CadXcafIo.h"

#include "CadNodeOps.h"
#include "CadNodePackage.h"

#include <BinDrivers.hxx>
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <BRepGProp.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepTools.hxx>
#include <BinXCAFDrivers.hxx>
#include <GProp_GProps.hxx>
#include <Geom_Surface.hxx>
#include <GeomLProp_SLProps.hxx>
#include <QDataStream>
#include <QDebug>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMessageBox>
#include <STEPCAFControl_Reader.hxx>
#include <STEPCAFControl_Writer.hxx>
#include <TCollection_ExtendedString.hxx>
#include <TDataStd_Name.hxx>
#include <TDataStd_TreeNode.hxx>
#include <TDF_ChildIterator.hxx>
#include <TDocStd_Application.hxx>
#include <TopExp_Explorer.hxx>
#include <XCAFApp_Application.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XmlDrivers.hxx>

#include <functional>
#include <algorithm>
#include <unordered_map>

namespace {

void setGlobalLocations(CadNode* node,
                        const TopLoc_Location& parentGlobalLoc = TopLoc_Location()) {
    if (!node) return;
    node->globalLoc = parentGlobalLoc * node->loc;
    for (auto& child : node->children) {
        setGlobalLocations(child.get(), node->globalLoc);
    }
}

} // namespace

// Follows reference chains, which STEP 214 assemblies use for shared geometry.
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

std::unique_ptr<XCAFLabelNode> buildDocumentLabelTree(
    const Handle(TDocStd_Document)& doc,
    const Handle(XCAFDoc_ShapeTool)& shapeTool) {
    auto root = std::make_unique<XCAFLabelNode>(doc->Main());

    for (TDF_ChildIterator it(doc->Main()); it.More(); it.Next()) {
        root->children.push_back(buildLabelTreeWithReferences(it.Value(), shapeTool));
    }

    TDF_LabelSequence freeShapes;
    shapeTool->GetFreeShapes(freeShapes);
    for (Standard_Integer i = 1; i <= freeShapes.Length(); ++i) {
        const TDF_Label freeShape = freeShapes.Value(i);
        const bool alreadyAdded = std::any_of(
            root->children.begin(), root->children.end(),
            [&](const std::unique_ptr<XCAFLabelNode>& child) {
                return child && child->label == freeShape;
            });
        if (!alreadyAdded) {
            root->children.push_back(buildLabelTreeWithReferences(freeShape, shapeTool));
        }
    }

    auto allShapes = std::make_unique<XCAFLabelNode>(TDF_Label());
    std::function<void(const TDF_Label&)> collectShapes = [&](const TDF_Label& label) {
        if (label.IsNull()) return;
        if (!shapeTool->GetShape(label).IsNull()) {
            allShapes->children.push_back(std::make_unique<XCAFLabelNode>(label));
        }
        for (TDF_ChildIterator it(label); it.More(); it.Next()) {
            collectShapes(it.Value());
        }
    };
    collectShapes(doc->Main());
    if (!allShapes->children.empty()) root->children.push_back(std::move(allShapes));

    return root;
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
    }
    return effectiveLoc;
}

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
            setGlobalLocations(cadRoot->children.back().get(), cadRoot->globalLoc);
        }
    }
    labelRoot = buildDocumentLabelTree(doc, shapeTool);
    shapeTool = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
    colorTool = XCAFDoc_DocumentTool::ColorTool(doc->Main());
    return true;
}

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
                        std::shared_ptr<CadNode>& customModelRoot)
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

    // Create CAD root node
    cadRoot = std::make_shared<CadNode>();
    cadRoot->name = "CAD Root";
    cadRoot->type = CadNodeType::MutexRoot;
    cadRoot->data = std::make_shared<MutexRootNodeData>();
    cadRoot->visible = true;

    labelRoot = buildDocumentLabelTree(doc, shapeTool);

    std::string packageError;
    customModelRoot = loadCadNodePackage(railJsonFile.toStdString(), &packageError);
    if (!customModelRoot) {
        QMessageBox::critical(nullptr, "Error", packageError.empty()
            ? "Failed to load robot package: " + railJsonFile
            : QString::fromStdString(packageError));
        return false;
    }
    customModelRootContainer = std::make_shared<CadNode>();
    customModelRootContainer->name = "Custom Model Root Container";
    customModelRootContainer->type = CadNodeType::MutexRoot;
    customModelRootContainer->data = std::make_shared<MutexRootNodeData>();
    customModelRootContainer->visible = true;

    customModelRootContainer->children.clear();
    customModelRootContainer->children.push_back(customModelRoot);
    setParentPointersRecursive(customModelRootContainer.get());
    CadNode::resolveRobotReferences(customModelRootContainer.get());
    return true;
}
