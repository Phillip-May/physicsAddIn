#include "CadViewerDialog.h"
#include "../Common/CadOpenGLWidget.h"
#include "../Common/CadTreeModel.h"
#include "../Common/CustomModelTreeModel.h"
#include "../Common/XCAFLabelTreeModel.h"
#include "../Common/CadNode.h"
#include "../Common/HelperFunctions.h"
#include <QAction>
#include <QMenu>
#include <QKeySequence>
#include <QIcon>
#include <QFileInfo>
#include <QItemSelectionModel>
#include <QAbstractItemModel>
#include <QLabel>
#include <QTreeWidget>

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

// Helper functions are now defined in Common/HelperFunctions.cpp

CadViewerDialog::CadViewerDialog(QWidget *parent)
    : QDialog(parent)
    , m_menuBar(nullptr)
    , m_toolBar(nullptr)
    , m_statusBar(nullptr)
    , m_treeTabWidget(nullptr)
    , m_viewTabWidget(nullptr)
    , m_mainSplitter(nullptr)
    , m_cadViewer(nullptr)
    , m_cadTreeModel(nullptr)
    , m_customTreeModel(nullptr)
    , m_xcafTreeModel(nullptr)
    , m_simpleTreeWidget(nullptr)
    , m_simulationRootNode(nullptr)
    , m_simulationViewer(nullptr)
    , m_simulationTreeModel(nullptr)
    , m_isModified(false)
    , m_openAction(nullptr)
    , m_saveAction(nullptr)
    , m_saveAsAction(nullptr)
    , m_exportAction(nullptr)
    , m_sampleAction(nullptr)
    , m_exitAction(nullptr)
    , m_aboutAction(nullptr)
    , m_reframeAction(nullptr)
    , m_loadSimulationAction(nullptr)
{
    setWindowTitle("CAD Viewer");
    setMinimumSize(1200, 800);
    
    setupUI();
    setupMenuBar();
    setupToolbar();
    setupStatusBar();
    connectSignals();
    
    // Load simulation tree by default
    loadSimulationTree();
}

CadViewerDialog::~CadViewerDialog()
{
    // Stop simulation if it's running
    if (m_simulationManager) {
        try {
            m_simulationManager->stopSimulation();
        } catch (...) {
            // Ignore exceptions during cleanup
        }
    }
    
    // Cleanup is handled by Qt's parent-child relationship
}

void CadViewerDialog::setupUI()
{
    // Create main layout
    QVBoxLayout* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(0, 0, 0, 0);
    mainLayout->setSpacing(0);
    
    // Create splitter for tree and viewer
    m_mainSplitter = new QSplitter(Qt::Horizontal);
    mainLayout->addWidget(m_mainSplitter);
    
    // Create tree tab widget
    m_treeTabWidget = new QTabWidget();
    m_treeTabWidget->setMaximumWidth(400);
    m_mainSplitter->addWidget(m_treeTabWidget);
    
    // Create view tab widget
    m_viewTabWidget = new QTabWidget();
    m_mainSplitter->addWidget(m_viewTabWidget);
    
    // Set splitter proportions
    m_mainSplitter->setSizes({300, 900});
    
    // Set the layout to stretch the splitter to fill available space
    mainLayout->setStretchFactor(m_mainSplitter, 1);
}

void CadViewerDialog::setupMenuBar()
{
    m_menuBar = new QMenuBar(this);
    
    // File menu
    QMenu* fileMenu = m_menuBar->addMenu("&File");
    
    m_openAction = fileMenu->addAction("&Open...");
    m_openAction->setShortcut(QKeySequence::Open);
    
    fileMenu->addSeparator();
    
    m_saveAction = fileMenu->addAction("&Save");
    m_saveAction->setShortcut(QKeySequence::Save);
    m_saveAction->setEnabled(false);
    
    m_saveAsAction = fileMenu->addAction("Save &As...");
    m_saveAsAction->setShortcut(QKeySequence::SaveAs);
    m_saveAsAction->setEnabled(false);
    
    fileMenu->addSeparator();
    
    m_exportAction = fileMenu->addAction("&Export...");
    m_exportAction->setEnabled(false);
    
    fileMenu->addSeparator();
    
    m_sampleAction = fileMenu->addAction("Load &Sample Model");
    m_sampleAction->setShortcut(QKeySequence("Ctrl+S"));
    
    fileMenu->addSeparator();
    
    m_exitAction = fileMenu->addAction("E&xit");
    m_exitAction->setShortcut(QKeySequence::Quit);
    
    // View menu
    QMenu* viewMenu = m_menuBar->addMenu("&View");
    m_reframeAction = viewMenu->addAction("&Reframe Camera");
    m_reframeAction->setShortcut(QKeySequence("Ctrl+R"));
    m_reframeAction->setToolTip("Reframe camera to fit all objects in view");
    
    viewMenu->addSeparator();
    
    m_loadSimulationAction = viewMenu->addAction("Load &Simulation Tree");
    m_loadSimulationAction->setShortcut(QKeySequence("Ctrl+Shift+S"));
    m_loadSimulationAction->setToolTip("Load simulation tree with sample data");
    
    // Simulation menu
    QMenu* simulationMenu = m_menuBar->addMenu("&Simulation");
    
    m_startPhysicsAction = simulationMenu->addAction("&Start Physics");
    m_startPhysicsAction->setShortcut(QKeySequence("Ctrl+P"));
    m_startPhysicsAction->setToolTip("Start physics simulation");
    m_startPhysicsAction->setEnabled(false); // Initially disabled until simulation is loaded
    
    m_stopPhysicsAction = simulationMenu->addAction("S&top Physics");
    m_stopPhysicsAction->setShortcut(QKeySequence("Ctrl+Shift+P"));
    m_stopPhysicsAction->setToolTip("Stop physics simulation");
    m_stopPhysicsAction->setEnabled(false); // Initially disabled
    
    // Help menu
    QMenu* helpMenu = m_menuBar->addMenu("&Help");
    m_aboutAction = helpMenu->addAction("&About");
    
    // Add menu bar to layout at the top
    QVBoxLayout* layout = qobject_cast<QVBoxLayout*>(this->layout());
    if (layout) {
        layout->insertWidget(0, m_menuBar);
        layout->setStretchFactor(m_menuBar, 0); // Don't stretch the menu bar
    }
}

void CadViewerDialog::setupToolbar()
{
    m_toolBar = new QToolBar("CAD Viewer Toolbar", this);
    m_toolBar->setMovable(true);
    m_toolBar->setFloatable(true);
    
    // Add reframe action (same action as in menu)
    m_reframeAction->setIcon(QIcon(":/icons/reframe.png")); // You can add an icon later
    m_toolBar->addAction(m_reframeAction);
    
    m_toolBar->addSeparator();
    
    // Add load simulation action
    m_loadSimulationAction->setIcon(QIcon(":/icons/simulation.png")); // You can add an icon later
    m_toolBar->addAction(m_loadSimulationAction);
    
    m_toolBar->addSeparator();
    
    // Add physics simulation actions
    m_startPhysicsAction->setIcon(QIcon(":/icons/play.png")); // You can add an icon later
    m_toolBar->addAction(m_startPhysicsAction);
    
    m_stopPhysicsAction->setIcon(QIcon(":/icons/stop.png")); // You can add an icon later
    m_toolBar->addAction(m_stopPhysicsAction);
    
    m_toolBar->addSeparator();
    
    // Add toolbar to layout after menu bar
    QVBoxLayout* layout = qobject_cast<QVBoxLayout*>(this->layout());
    if (layout) {
        layout->insertWidget(1, m_toolBar); // Insert after menu bar (index 0)
        layout->setStretchFactor(m_toolBar, 0); // Don't stretch the toolbar
    }
}

void CadViewerDialog::setupStatusBar()
{
    m_statusBar = new QStatusBar(this);
    
    // Add status bar to layout at the bottom
    QVBoxLayout* layout = qobject_cast<QVBoxLayout*>(this->layout());
    if (layout) {
        layout->addWidget(m_statusBar);
        layout->setStretchFactor(m_statusBar, 0); // Don't stretch the status bar
    }
    
    m_statusBar->showMessage("Ready");
}

void CadViewerDialog::connectSignals()
{
    // Connect menu actions
    connect(m_openAction, &QAction::triggered, this, &CadViewerDialog::openFile);
    connect(m_saveAction, &QAction::triggered, this, &CadViewerDialog::saveFile);
    connect(m_saveAsAction, &QAction::triggered, this, &CadViewerDialog::saveFile);
    connect(m_exportAction, &QAction::triggered, this, &CadViewerDialog::exportModel);
    
    connect(m_sampleAction, &QAction::triggered, this, &CadViewerDialog::loadSampleModel);
    
    connect(m_exitAction, &QAction::triggered, this, &QDialog::close);
    connect(m_aboutAction, &QAction::triggered, this, &CadViewerDialog::about);
    
    // Connect toolbar actions
    connect(m_reframeAction, &QAction::triggered, this, &CadViewerDialog::reframeCamera);
    connect(m_loadSimulationAction, &QAction::triggered, this, &CadViewerDialog::loadSimulationTree);
    connect(m_startPhysicsAction, &QAction::triggered, this, &CadViewerDialog::startPhysics);
    connect(m_stopPhysicsAction, &QAction::triggered, this, &CadViewerDialog::stopPhysics);
}

void CadViewerDialog::openFile()
{
    QString filePath = QFileDialog::getOpenFileName(
        this,
        "Open CAD File",
        QString(),
        "CAD Files (*.step *.stp *.iges *.igs *.stl *.obj);;STEP Files (*.step *.stp);;IGES Files (*.iges *.igs);;STL Files (*.stl);;OBJ Files (*.obj);;All Files (*.*)"
    );
    
    if (!filePath.isEmpty()) {
        if (loadCadModel(filePath)) {
            m_currentFilePath = filePath;
            updateWindowTitle();
            m_statusBar->showMessage(QString("Loaded: %1").arg(filePath), 3000);
        }
    }
}

bool CadViewerDialog::loadCadModel(const QString& filePath)
{
    try {
        // Use shared STEP file loading function
        std::shared_ptr<CadNode> cadRoot;
        std::unique_ptr<XCAFLabelNode> labelRoot;
        Handle(XCAFDoc_ShapeTool) shapeTool;
        Handle(XCAFDoc_ColorTool) colorTool;
        
        if (!loadStepFile(filePath, m_doc, cadRoot, labelRoot, shapeTool, colorTool)) {
            return false;
        }
        
        // Set up the tree and viewer
        setCadNodeTree(cadRoot);
        
        return true;
        
    } catch (const Standard_Failure& e) {
        QMessageBox::critical(this, "Error", QString("OpenCascade error: %1").arg(e.GetMessageString()));
        return false;
    } catch (const std::exception& e) {
        QMessageBox::critical(this, "Error", QString("Exception: %1").arg(e.what()));
        return false;
    }
}

void CadViewerDialog::setCadNodeTree(std::shared_ptr<CadNode> rootNode)
{
    m_rootNode = rootNode;
    
    // Clear existing widgets
    m_treeTabWidget->clear();
    m_viewTabWidget->clear();
    
    if (!m_rootNode) {
        return;
    }
    
    // Use shared tree and OpenGL widget initialization
    initTreeAndOpenGLWidget(m_rootNode, m_treeTabWidget, m_viewTabWidget, "CAD Model", m_doc);
    
    // Re-add simulation tree if it exists
    if (m_simulationRootNode) {
        initTreeAndOpenGLWidget(m_simulationRootNode, m_treeTabWidget, m_viewTabWidget, "Simulation", m_doc, m_simulationManager.get());
    }
    
    // Enable save/export actions
    m_saveAction->setEnabled(true);
    m_saveAsAction->setEnabled(true);
    m_exportAction->setEnabled(true);
    
    updateWindowTitle();
}

std::shared_ptr<CadNode> CadViewerDialog::getRootNode() const
{
    return m_rootNode;
}

void CadViewerDialog::saveFile()
{
    if (m_currentFilePath.isEmpty()) {
        QString filePath = QFileDialog::getSaveFileName(
            this,
            "Save CAD File",
            QString(),
            "STEP Files (*.step *.stp);;IGES Files (*.iges *.igs);;All Files (*.*)"
        );
        
        if (filePath.isEmpty()) {
            return;
        }
        
        m_currentFilePath = filePath;
    }
    
    // TODO: Implement save functionality
    QMessageBox::information(this, "Save", "Save functionality not yet implemented");
}

void CadViewerDialog::exportModel()
{
    QString filePath = QFileDialog::getSaveFileName(
        this,
        "Export CAD Model",
        QString(),
        "STL Files (*.stl);;OBJ Files (*.obj);;All Files (*.*)"
    );
    
    if (!filePath.isEmpty()) {
        // TODO: Implement export functionality
        QMessageBox::information(this, "Export", "Export functionality not yet implemented");
    }
}

void CadViewerDialog::about()
{
    QMessageBox::about(this, "About CAD Viewer",
                      "CAD Viewer\n\n"
                      "A 3D CAD model viewer integrated with the Physics Plugin.\n"
                      "Supports STEP, IGES, STL, and OBJ file formats.\n\n"
                      "Built with OpenCascade and Qt.");
}

void CadViewerDialog::loadSampleModel()
{
    // Create a simple test model
    auto rootNode = std::make_shared<CadNode>();
    rootNode->name = "Sample Model";
    rootNode->type = CadNodeType::Custom;
    rootNode->color = CADNodeColor(0.8f, 0.8f, 0.8f, 1.0f);
    
    // Create a simple box geometry (this would need to be implemented with actual OpenCascade shapes)
    auto boxNode = std::make_shared<CadNode>();
    boxNode->name = "Test Box";
    boxNode->type = CadNodeType::Custom;
    boxNode->color = CADNodeColor(0.2f, 0.6f, 1.0f, 1.0f);
    
    rootNode->children.push_back(boxNode);
    
    setCadNodeTree(rootNode);
    m_statusBar->showMessage("Loaded sample model", 3000);
}

void CadViewerDialog::updateWindowTitle()
{
    QString title = "CAD Viewer";
    if (!m_currentFilePath.isEmpty()) {
        title += QString(" - %1").arg(QFileInfo(m_currentFilePath).fileName());
    }
    if (m_isModified) {
        title += " *";
    }
    setWindowTitle(title);
}

void CadViewerDialog::reframeCamera()
{
    // Get the currently active widget from the view tab widget
    QWidget* currentWidget = m_viewTabWidget->currentWidget();
    if (currentWidget) {
        CadOpenGLWidget* cadWidget = qobject_cast<CadOpenGLWidget*>(currentWidget);
        if (cadWidget) {
            cadWidget->reframeCamera();
            m_statusBar->showMessage("Camera reframed", 2000);
            return;
        }
    }
    
    // If no CadOpenGLWidget found in the current tab, try to find any CadOpenGLWidget
    for (int i = 0; i < m_viewTabWidget->count(); ++i) {
        QWidget* widget = m_viewTabWidget->widget(i);
        CadOpenGLWidget* cadWidget = qobject_cast<CadOpenGLWidget*>(widget);
        if (cadWidget) {
            cadWidget->reframeCamera();
            m_statusBar->showMessage("Camera reframed", 2000);
            return;
        }
    }
    
    // If no CadOpenGLWidget found, show a message
    m_statusBar->showMessage("No 3D view available for reframing", 2000);
}

void CadViewerDialog::loadSimulationTree()
{
    // Create simulation root node as MutexRoot (same as QtCadViewer)
    m_simulationRootNode = std::make_shared<CadNode>();
    m_simulationRootNode->name = "Physics MutexRoot";
    m_simulationRootNode->type = CadNodeType::MutexRoot;
    m_simulationRootNode->data = std::make_shared<MutexRootNodeData>();
    m_simulationRootNode->color = CADNodeColor(0.2f, 0.8f, 0.2f, 1.0f); // Green color for simulation
    
    // Create a physics child node (this will be collected by collectPhysicsNodes)
    auto physicsChild = std::make_shared<CadNode>();
    physicsChild->name = "Custom Simulation Object";
    physicsChild->type = CadNodeType::Physics;
    physicsChild->data = std::make_shared<PhysicsNodeData>();
    physicsChild->color = CADNodeColor(0.8f, 0.4f, 0.2f, 1.0f); // Orange color
    
    // Add the physics child to the simulation root
    m_simulationRootNode->children.push_back(physicsChild);
    
    // Create and initialize the SimulationManager
    m_simulationManager = std::make_unique<SimulationManager>(m_simulationRootNode);
    
    // Debug output to verify the tree structure
    qDebug() << "[CadViewerDialog] Simulation root node type:" << static_cast<int>(m_simulationRootNode->type)
             << ", name:" << QString::fromStdString(m_simulationRootNode->name);
    if (!m_simulationRootNode->children.empty()) {
        qDebug() << "[CadViewerDialog] First child type:" << static_cast<int>(m_simulationRootNode->children[0]->type)
                 << ", name:" << QString::fromStdString(m_simulationRootNode->children[0]->name);
    }
    
    // Add simulation tree and viewer to the existing tab widgets with SimulationManager
    initTreeAndOpenGLWidget(m_simulationRootNode, m_treeTabWidget, m_viewTabWidget, "Simulation", m_doc, m_simulationManager.get());
    
    m_statusBar->showMessage("Loaded simulation tree", 2000);
    
    // Enable physics actions after simulation is loaded
    m_startPhysicsAction->setEnabled(true);
    m_stopPhysicsAction->setEnabled(false); // Stop is disabled until simulation starts
} 

void CadViewerDialog::startPhysics()
{
    if (!m_simulationManager) {
        QMessageBox::warning(this, "Physics Simulation", 
                           "No simulation manager available. Please load a simulation first.");
        return;
    }
    
    try {
        // Start the physics simulation using the SimulationManager
        m_simulationManager->startSimulation();
        
        updateSimulationUI();
        
    } catch (const std::exception& e) {
        QMessageBox::critical(this, "Physics Simulation Error", 
                            QString("Failed to start physics simulation: %1").arg(e.what()));
        m_statusBar->showMessage("Failed to start physics simulation", 3000);
    }
}

void CadViewerDialog::stopPhysics()
{
    if (!m_simulationManager) {
        QMessageBox::warning(this, "Physics Simulation", 
                           "No simulation manager available.");
        return;
    }
    
    try {
        // Stop the physics simulation using the SimulationManager
        m_simulationManager->stopSimulation();
        
        updateSimulationUI();
        
    } catch (const std::exception& e) {
        QMessageBox::critical(this, "Physics Simulation Error", 
                            QString("Failed to stop physics simulation: %1").arg(e.what()));
        m_statusBar->showMessage("Failed to stop physics simulation", 3000);
    }
}

bool CadViewerDialog::isSimulationRunning() const
{
    return m_simulationManager && !m_simulationManager->isSceneBuilding();
}

void CadViewerDialog::updateSimulationUI()
{
    if (!m_simulationManager) {
        m_startPhysicsAction->setEnabled(false);
        m_stopPhysicsAction->setEnabled(false);
        return;
    }
    
    bool isRunning = isSimulationRunning();
    m_startPhysicsAction->setEnabled(!isRunning);
    m_stopPhysicsAction->setEnabled(isRunning);
    
    // Update status bar
    if (isRunning) {
        m_statusBar->showMessage("Physics simulation is running", 0); // Persistent message
    } else {
        m_statusBar->showMessage("Physics simulation stopped", 2000);
    }
}
