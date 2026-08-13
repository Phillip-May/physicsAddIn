#include "CadNode.h"
#include "CadNodeOps.h"
#include "CadNodePackage.h"
#include "CadOpenGLWidget.h"
#include "CadViewerWiring.h"
#include "CadXcafIo.h"
#include "RobotRuntime.h"
#include "SimulationManager.h"
#include "XCAFLabelTreeModel.h"

#include <QAbstractItemView>
#include <QApplication>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QMainWindow>
#include <QMessageBox>
#include <QPushButton>
#include <QSplitter>
#include <QTabWidget>
#include <QTreeView>
#include <QVBoxLayout>
#include <QWidget>

#include <iostream>
#include <memory>

namespace {

int validatePackage(const QString& packageFile) {
    std::string errorMessage;
    const std::shared_ptr<CadNode> root =
        loadCadNodePackage(packageFile.toStdString(), &errorMessage);
    if (!root || !validateRobotPackage(root.get(), &errorMessage)) {
        std::cerr << "Package validation failed: " << errorMessage << '\n';
        return 1;
    }
    std::cout << "Package validation succeeded: " << packageFile.toStdString() << '\n';
    return 0;
}

std::shared_ptr<CadNode> makeCustomModelContainer(std::shared_ptr<CadNode>* modelRoot) {
    auto container = std::make_shared<CadNode>();
    container->name = "Custom Model Root Container";
    container->type = CadNodeType::MutexRoot;
    container->data = std::make_shared<MutexRootNodeData>();
    container->visible = true;

    MutexRootNodeData* ground = container->asMutexRoot();
    ground->groundPlaneVisible = true;
    ground->groundPlaneY = -50.0;
    ground->groundPlaneSize = 10000.0;
    ground->groundPlaneThickness = 0.1;
    ground->groundPlaneColor = CADNodeColor(0.7f, 0.7f, 0.7f, 0.8f);

    auto rail = std::make_shared<CadNode>();
    rail->name = "Custom Rail";
    rail->visible = true;
    rail->type = CadNodeType::Rail;
    rail->data = std::make_shared<RailNodeData>();
    container->children.push_back(rail);
    setParentPointersRecursive(container.get());
    if (modelRoot) *modelRoot = std::move(rail);
    return container;
}

std::shared_ptr<CadNode> makePhysicsPreviewRoot() {
    auto root = std::make_shared<CadNode>();
    root->type = CadNodeType::MutexRoot;
    root->data = std::make_shared<MutexRootNodeData>();
    root->name = "Physics";
    return root;
}

CadOpenGLWidget* activeViewer(QTabWidget* tabs) {
    return tabs ? qobject_cast<CadOpenGLWidget*>(tabs->currentWidget()) : nullptr;
}

} // namespace

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);
    const QStringList args = app.arguments();
    if (args.size() >= 3 && args[1] == "--validate-package") {
        return validatePackage(args[2]);
    }

    const QString openFile = args.size() >= 2
        ? args[1]
        : QFileDialog::getOpenFileName(
              nullptr,
              "Open STEP, JSON, or Robot Package File",
              {},
              "Supported Files (*.step *.stp *.json *.zip);;Robot Packages (*.zip);;"
              "JSON Files (*.json);;STEP Files (*.step *.stp);;All Files (*.*)");
    if (openFile.isEmpty()) return 0;

    Handle(TDocStd_Document) document;
    std::shared_ptr<CadNode> cadRoot;
    std::shared_ptr<CadNode> customModelContainer;
    std::shared_ptr<CadNode> customModelRoot;
    std::unique_ptr<XCAFLabelNode> labelRoot;
    Handle(XCAFDoc_ShapeTool) shapeTool;
    Handle(XCAFDoc_ColorTool) colorTool;

    const bool isPackage = openFile.endsWith(".json", Qt::CaseInsensitive) ||
        openFile.endsWith(".zip", Qt::CaseInsensitive);
    if (isPackage) {
        if (!loadFromJsonAndBin(openFile, document, cadRoot, labelRoot, shapeTool, colorTool,
                                customModelContainer, customModelRoot)) {
            return 1;
        }
    } else {
        if (!loadStepFile(openFile, document, cadRoot, labelRoot, shapeTool, colorTool)) return 1;
        customModelContainer = makeCustomModelContainer(&customModelRoot);
    }

    QMainWindow mainWindow;
    mainWindow.setWindowTitle("QtCadViewer");
    CadViewerWorkspace workspace;

    auto* centralWidget = new QWidget(&mainWindow);
    auto* layout = new QVBoxLayout(centralWidget);
    auto* buttonLayout = new QHBoxLayout;
    auto* outerSplitter = new QSplitter(Qt::Horizontal, centralWidget);
    auto* contentSplitter = new QSplitter(Qt::Horizontal, outerSplitter);
    auto* treeTabs = new QTabWidget(contentSplitter);
    auto* viewTabs = new QTabWidget(contentSplitter);
    treeTabs->setMinimumWidth(300);
    contentSplitter->addWidget(treeTabs);
    contentSplitter->addWidget(viewTabs);
    outerSplitter->addWidget(contentSplitter);
    layout->addLayout(buttonLayout);
    layout->addWidget(outerSplitter);
    layout->setContentsMargins(0, 0, 0, 0);
    mainWindow.setCentralWidget(centralWidget);

    auto* noSelection = new QPushButton("No Selection", centralWidget);
    auto* faceSelection = new QPushButton("Face Selection", centralWidget);
    auto* edgeSelection = new QPushButton("Edge Selection", centralWidget);
    auto* reframe = new QPushButton("Reframe", centralWidget);
    auto* saveStep = new QPushButton("Save STEP", centralWidget);
    auto* saveBinary = new QPushButton("Save Binary", centralWidget);
    buttonLayout->addStretch();
    buttonLayout->addWidget(noSelection);
    buttonLayout->addWidget(faceSelection);
    buttonLayout->addWidget(edgeSelection);
    buttonLayout->addWidget(reframe);
    buttonLayout->addWidget(saveStep);
    buttonLayout->addWidget(saveBinary);

    const QString activeStyle =
        "QPushButton { background-color: #4CAF50; color: white; font-weight: bold; }";
    const QString inactiveStyle =
        "QPushButton { background-color: #f0f0f0; color: black; }";
    const auto applySelectionMode = [=](SelectionMode mode, QPushButton* selected) {
        if (CadOpenGLWidget* viewer = activeViewer(viewTabs)) viewer->setSelectionMode(mode);
        for (QPushButton* button : {noSelection, faceSelection, edgeSelection}) {
            button->setStyleSheet(button == selected ? activeStyle : inactiveStyle);
        }
    };
    applySelectionMode(SelectionMode::None, noSelection);
    QObject::connect(noSelection, &QPushButton::clicked, [=]() {
        applySelectionMode(SelectionMode::None, noSelection);
    });
    QObject::connect(faceSelection, &QPushButton::clicked, [=]() {
        applySelectionMode(SelectionMode::Faces, faceSelection);
    });
    QObject::connect(edgeSelection, &QPushButton::clicked, [=]() {
        applySelectionMode(SelectionMode::Edges, edgeSelection);
    });
    QObject::connect(reframe, &QPushButton::clicked, [=]() {
        if (CadOpenGLWidget* viewer = activeViewer(viewTabs)) viewer->reframeCamera();
    });

    initTreeAndOpenGLWidget(cadRoot, treeTabs, viewTabs, "CAD", document, workspace);
    initTreeAndOpenGLWidget(
        customModelContainer, treeTabs, viewTabs, "Custom Model", document, workspace);

    std::shared_ptr<CadNode> physicsRoot = makePhysicsPreviewRoot();
    SimulationManager simulation(physicsRoot);
    simulation.addGuiElements(&mainWindow);
    initTreeAndOpenGLWidget(
        physicsRoot, treeTabs, viewTabs, "Physics", document, workspace, &simulation);

    QObject::connect(saveStep, &QPushButton::clicked, [=]() {
        const QString output = QDir::currentPath() + QDir::separator() +
            QFileInfo(openFile).baseName() + "_serialized.step";
        if (saveXCAFToSTEP(document, output)) {
            QMessageBox::information(nullptr, "Success", "XCAF data saved to:\n" + output);
        } else {
            QMessageBox::warning(nullptr, "Error", "Failed to save XCAF data to STEP file");
        }
    });
    QObject::connect(saveBinary, &QPushButton::clicked, [=]() {
        const QString output = QDir::currentPath() + QDir::separator() +
            QFileInfo(openFile).baseName() + "_serialized.bin";
        if (saveXCAFToBinary(document, output)) {
            QMessageBox::information(nullptr, "Success", "XCAF data saved to:\n" + output);
        } else {
            QMessageBox::warning(nullptr, "Error", "Failed to save XCAF data to binary file");
        }
    });

    auto* labelTree = new QTreeView(treeTabs);
    auto* labelModel = new XCAFLabelTreeModel(std::move(labelRoot), labelTree);
    labelModel->setShapeTool(shapeTool);
    labelModel->setColorTool(colorTool);
    labelTree->setModel(labelModel);
    labelTree->setHeaderHidden(false);
    labelTree->setSelectionMode(QAbstractItemView::ExtendedSelection);
    setupComprehensiveContextMenu(labelTree, labelModel, document, nullptr, &workspace);
    treeTabs->addTab(labelTree, "XCAF Tree");

    mainWindow.resize(1200, 800);
    mainWindow.show();
    return app.exec();
}
