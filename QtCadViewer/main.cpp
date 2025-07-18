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
// #include "CadNode.h"  // Removed
#include <memory>
#include <QDebug>
#include <BRep_Builder.hxx>
#include <TopoDS_Compound.hxx>
#include <QSet>
#include <QHash>
#include <TDF_ChildIterator.hxx>

#include "CadNode.h"
#include "XCAFLabelTreeModel.h"

// Helper to recursively build XCAFLabelNode tree
std::unique_ptr<XCAFLabelNode> buildLabelTree(const TDF_Label& label) {
    auto node = std::make_unique<XCAFLabelNode>(label);
    for (TDF_ChildIterator it(label); it.More(); it.Next()) {
        node->children.push_back(buildLabelTree(it.Value()));
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







// Helper: Extract color from XCAF label, fallback to parent color
CADNodeColor get_label_color(const TDF_Label& label, const Handle(XCAFDoc_ColorTool)& colorTool, const CADNodeColor& parentColor) {
    Quantity_Color occColor;
    if (colorTool->GetColor(label, XCAFDoc_ColorSurf, occColor) ||
        colorTool->GetColor(label, XCAFDoc_ColorGen, occColor) ||
        colorTool->GetColor(label, XCAFDoc_ColorCurv, occColor)) {
        return CADNodeColor(occColor.Red(), occColor.Green(), occColor.Blue());
    }
    return parentColor;
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

// Recursive function to build tree and collect faces
std::unique_ptr<TreeNode> build_tree_xcaf(const TDF_Label& label,
                   const Handle(XCAFDoc_ShapeTool)& shapeTool,
                   const Handle(XCAFDoc_ColorTool)& colorTool,
                   const CADNodeColor& parentColor,
                   const TopLoc_Location& parentLoc,
                   std::vector<ColoredFace>& all_faces)
{
    CADNodeColor color = get_label_color(label, colorTool, parentColor);
    TopoDS_Shape shape = shapeTool->GetShape(label);
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
    // Determine the correct transform for this node
    TopLoc_Location nodeLoc = parentLoc;
    auto node = std::make_unique<TreeNode>();
    node->color = color;
    node->loc = nodeLoc;
    if (!shape.IsNull()) {
        node->type = shape.ShapeType();
    } else {
        node->type = TopAbs_SHAPE;
    }
    QString typeName = shapeTypeToString(node->type);
    // Compose a detailed name string
    node->name = QString("Label %1 | Type: %2 | Transform: %3%4")
        .arg(label.Tag())
        .arg(typeName)
        .arg(makeTransformString(nodeLoc))
        .arg(name.isEmpty() ? "" : QString(" | OCC Name: %1").arg(name))
        .toStdString();

    // If this label is a reference, add the referred label as a child
    if (shapeTool->IsReference(label)) {
        TDF_Label refLabel;
        if (shapeTool->GetReferredShape(label, refLabel)) {
            TopLoc_Location refLoc = parentLoc * shapeTool->GetLocation(label);
            auto child = build_tree_xcaf(refLabel, shapeTool, colorTool, color, refLoc, all_faces);
            if (child) node->children.push_back(std::move(child));
        }
    }

    // Recurse into children (for all labels)
    for (TDF_ChildIterator it(label); it.More(); it.Next()) {
        auto child = build_tree_xcaf(it.Value(), shapeTool, colorTool, color, nodeLoc, all_faces);
        if (child) node->children.push_back(std::move(child));
    }
    // After node is created and type is set
    if (!shape.IsNull()) {
        node->shapeData.shape = shape;
    }
    // Only add face/edge children for solids that would otherwise be leafs
    if (!shape.IsNull() && shape.ShapeType() == TopAbs_SOLID) {
        // Check if this solid is a leaf node (no child labels)
        bool isLeafNode = true;
        for (TDF_ChildIterator it(label); it.More(); it.Next()) {
            isLeafNode = false;
            break;
        }
        // Also check if any existing children are solids (to avoid intermediate solids)
        bool hasSolidChildren = false;
        for (const auto& child : node->children) {
            if (child->type == TopAbs_SOLID) {
                hasSolidChildren = true;
                break;
            }
        }
        if (isLeafNode && !hasSolidChildren) {
            // Add face children
            int faceIdx = 0;
            for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next(), ++faceIdx) {
                TopoDS_Shape face = exp.Current();
                auto faceNode = std::make_unique<TreeNode>();
                faceNode->type = TopAbs_FACE;
                faceNode->shapeData.shape = face;
                faceNode->color = color;
                faceNode->loc = nodeLoc;
                faceNode->name = QString("Face %1 of %2").arg(faceIdx).arg(QString::fromStdString(node->name)).toStdString();
                node->children.push_back(std::move(faceNode));
            }
            // Add edge children
            int edgeIdx = 0;
            for (TopExp_Explorer exp(shape, TopAbs_EDGE); exp.More(); exp.Next(), ++edgeIdx) {
                TopoDS_Shape edge = exp.Current();
                auto edgeNode = std::make_unique<TreeNode>();
                edgeNode->type = TopAbs_EDGE;
                edgeNode->shapeData.shape = edge;
                edgeNode->color = color;
                edgeNode->loc = nodeLoc;
                edgeNode->name = QString("Edge %1 of %2").arg(edgeIdx).arg(QString::fromStdString(node->name)).toStdString();
                node->children.push_back(std::move(edgeNode));
            }
        }
    }
    return node;
}

// Qt model for the tree view
#include <QAbstractItemModel>
#include <QMenu>
#include <QAction>

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
    appOCC->NewDocument("MDTV-XCAF", doc);
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

    // Get shape and color tools from the document
    Handle(XCAFDoc_ShapeTool) shapeTool = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
    Handle(XCAFDoc_ColorTool) colorTool = XCAFDoc_DocumentTool::ColorTool(doc->Main());

    // Build tree and collect faces
    std::vector<ColoredFace> faces;
    CADNodeColor defaultColor = CADNodeColor::fromSRGB(200, 200, 200);
    TopLoc_Location identityLoc;
    TDF_LabelSequence roots;
    shapeTool->GetFreeShapes(roots);
    auto root = std::make_unique<TreeNode>();
    root->name = "Root";
    root->color = defaultColor;
    root->loc = identityLoc;
    for (Standard_Integer i = 1; i <= roots.Length(); ++i) {
        auto child = build_tree_xcaf(roots.Value(i), shapeTool, colorTool, defaultColor, identityLoc, faces);
        if (child) root->children.push_back(std::move(child));
    }
    qDebug() << "Total faces collected:" << faces.size();

    // Central widget and splitter
    QSplitter *splitter = new QSplitter(Qt::Horizontal);

    // Layout: buttons above splitter
    QWidget *centralWidget = new QWidget;
    QVBoxLayout *vLayout = new QVBoxLayout;
    QHBoxLayout *buttonLayout = new QHBoxLayout;
    buttonLayout->addStretch();
    vLayout->addLayout(buttonLayout);
    vLayout->addWidget(splitter);
    vLayout->setContentsMargins(0,0,0,0);
    centralWidget->setLayout(vLayout);
    mainWindow.setCentralWidget(centralWidget);

    // Model/view tree
    CadTreeModel *model = new CadTreeModel(std::move(root));
    QTreeView *treeView = new QTreeView;
    treeView->setModel(model);
    treeView->setHeaderHidden(false);
    treeView->setSelectionMode(QAbstractItemView::ExtendedSelection); // Allow multi-selection
    splitter->addWidget(treeView);

    // Add OpenGL widget for geometry display
    CadOpenGLWidget *viewer = new CadOpenGLWidget;
    splitter->addWidget(viewer);
    // Pass the root node to the viewer for rendering
    viewer->setRootTreeNode(model->getRootNodePointer());

    // --- XCAF Label Tree View ---
    // Build label tree from XCAF roots
    auto labelRoot = std::make_unique<XCAFLabelNode>(doc->Main());
    for (TDF_ChildIterator it(doc->Main()); it.More(); it.Next()) {
        labelRoot->children.push_back(buildLabelTree(it.Value()));
    }
    XCAFLabelTreeModel* labelModel = new XCAFLabelTreeModel(std::move(labelRoot));
    QTreeView* labelTreeView = new QTreeView;
    labelTreeView->setModel(labelModel);
    labelTreeView->setHeaderHidden(false);
    labelTreeView->setMinimumWidth(250);
    splitter->addWidget(labelTreeView);

    // Context menu for show/hide
    treeView->setContextMenuPolicy(Qt::CustomContextMenu);
    QObject::connect(treeView, &QTreeView::customContextMenuRequested, treeView, [=](const QPoint& pos) {
        QModelIndex idx = treeView->indexAt(pos);
        if (!idx.isValid()) return;
        QMenu menu;
        QAction* showAction = menu.addAction("Show");
        QAction* hideAction = menu.addAction("Hide");
        QObject::connect(showAction, &QAction::triggered, treeView, [=]() {
            auto selection = treeView->selectionModel()->selectedIndexes();
            QSet<TreeNode*> affected;
            for (const QModelIndex& selIdx : selection) {
                TreeNode* n = model->getNode(selIdx);
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
            QSet<TreeNode*> affected;
            for (const QModelIndex& selIdx : selection) {
                TreeNode* n = model->getNode(selIdx);
                if (n && !affected.contains(n)) {
                    n->setVisibleRecursive(false);
                    affected.insert(n);
                }
            }
            viewer->update();
            for (const QModelIndex& selIdx : selection) model->dataChanged(selIdx, selIdx);
        });
        menu.exec(treeView->viewport()->mapToGlobal(pos));
    });

    mainWindow.resize(1200, 800);
    mainWindow.show();
    return app.exec();
}
