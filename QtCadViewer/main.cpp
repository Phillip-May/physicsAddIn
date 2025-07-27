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
#include <QDir>
#include <QFileInfo>
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
#include <gp_Trsf.hxx>
#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>
#include <gp_Vec.hxx>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonObject>
#include <QJsonDocument>
#include <QGroupBox>
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
#include <QGroupBox>
#include <QLabel>
#include <QGridLayout>
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
        if (!customModelRoot) customModelRoot = std::make_shared<CadNode>();

        // Set up Custom model root container as MutexRoot with ground plane
        customModelRootContainer->name = "Custom Model Root Container";
        customModelRootContainer->type = CadNodeType::MutexRoot;
        customModelRootContainer->data = std::make_shared<MutexRootNodeData>();
        customModelRootContainer->visible = true;

        // Set ground plane properties for Custom model preview
        MutexRootNodeData* mutexData = customModelRootContainer->asMutexRoot();
        if (mutexData) {
            mutexData->groundPlaneVisible = true;
            mutexData->groundPlaneY = -50.0;
            mutexData->groundPlaneSize = 10000.0;
            mutexData->groundPlaneThickness = 0.1;
            mutexData->groundPlaneColor = CADNodeColor(0.7f, 0.7f, 0.7f, 0.8f);
        }

        customModelRoot->name = "Custom Rail";
        customModelRoot->visible = true;
        customModelRoot->type = CadNodeType::Rail;
        customModelRoot->data = std::make_shared<RailNodeData>();
        customModelRootContainer->children.clear();
        customModelRootContainer->children.push_back(customModelRoot);
    }
    
    initTreeAndOpenGLWidget(cadRootShared, tabWidget, viewTabWidget, "CAD", doc);
    // For the custom model tree, use the container as the root for the tree view, but set the OpenGL widget's root to the real root (first child of the container)
    initTreeAndOpenGLWidget(customModelRootContainer, tabWidget, viewTabWidget, "Custom Model", doc);

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
    physicsPreviewRoot->type = CadNodeType::MutexRoot;
    physicsPreviewRoot->data = std::make_shared<MutexRootNodeData>();
    // Optionally set a name for debug
    physicsPreviewRoot->name = "Physics MutexRoot";
    std::cout << "[main] physicsPreviewRoot type: " << static_cast<int>(physicsPreviewRoot->type)
              << ", name: " << physicsPreviewRoot->name << std::endl;
    // Print children info for further debugging
    if (!physicsPreviewRoot->children.empty()) {
        std::cout << "[main] physicsPreviewRoot first child type: " << static_cast<int>(physicsPreviewRoot->children[0]->type)
                  << ", name: " << physicsPreviewRoot->children[0]->name << std::endl;
    }
    SimulationManager simManager(physicsPreviewRoot);
    std::cout << "[main] SimulationManager constructed with root node type: " << static_cast<int>(physicsPreviewRoot->type)
              << ", name: " << physicsPreviewRoot->name << std::endl;
    // Add SimulationManager GUI elements
    simManager.addGuiElements(&mainWindow);
    initTreeAndOpenGLWidget(physicsPreviewRoot,tabWidget,viewTabWidget,"Physics", doc,&simManager);

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

    qDebug() << "XCAF tree built with" << labelRoot->children.size() << "root children";

    XCAFLabelTreeModel* labelModel = new XCAFLabelTreeModel(std::move(labelRoot));

    // Pass tools to the model for enhanced information display
    labelModel->setShapeTool(shapeTool);
    labelModel->setColorTool(colorTool);

    QTreeView* labelTreeView = new QTreeView;
    labelTreeView->setModel(labelModel);
    labelTreeView->setHeaderHidden(false);
    labelTreeView->setSelectionMode(QAbstractItemView::ExtendedSelection);
    
    // Setup comprehensive context menu for XCAF tree using shared function
    setupComprehensiveContextMenu(labelTreeView, labelModel, doc, nullptr);
    
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


