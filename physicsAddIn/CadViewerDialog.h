#ifndef CADVIEWERDIALOG_H
#define CADVIEWERDIALOG_H

#include <QDialog>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QSplitter>
#include <QTreeView>
#include <QTabWidget>
#include <QPushButton>
#include <QMenuBar>
#include <QStatusBar>
#include <QToolBar>
#include <QFileDialog>
#include <QMessageBox>
#include <QApplication>
#include <memory>
#include <QTreeWidget>
#include <Standard_Handle.hxx>
#include <TDocStd_Document.hxx>
#include "../Common/SimulationManager.h"

// Forward declarations
struct CadNode;
class CadOpenGLWidget;
class CadTreeModel;
class CustomModelTreeModel;
class XCAFLabelTreeModel;

class CadViewerDialog : public QDialog
{
    Q_OBJECT

public:
    explicit CadViewerDialog(QWidget *parent = nullptr);
    ~CadViewerDialog();

    // Load CAD model from file
    bool loadCadModel(const QString& filePath);
    
    // Set a CadNode tree directly
    void setCadNodeTree(std::shared_ptr<CadNode> rootNode);
    
    // Get the current root node
    std::shared_ptr<CadNode> getRootNode() const;
    
    // Load a sample model for testing
    void loadSampleModel();
    
    // Load simulation tree
    void loadSimulationTree();
    
    // Check if simulation is running
    bool isSimulationRunning() const;
    
    // Update UI state based on simulation status
    void updateSimulationUI();

private slots:
    void openFile();
    void saveFile();
    void exportModel();
    void about();
    void reframeCamera();
    void startPhysics();
    void stopPhysics();

private:
    void setupUI();
    void setupMenuBar();
    void setupToolbar();
    void setupStatusBar();
    void connectSignals();
    void updateWindowTitle();
    
    // UI Components
    QMenuBar* m_menuBar;
    QToolBar* m_toolBar;
    QStatusBar* m_statusBar;
    QTabWidget* m_treeTabWidget;
    QTabWidget* m_viewTabWidget;
    QSplitter* m_mainSplitter;
    
    // CAD Components
    std::shared_ptr<CadNode> m_rootNode;
    CadOpenGLWidget* m_cadViewer;
    CadTreeModel* m_cadTreeModel;
    CustomModelTreeModel* m_customTreeModel;
    XCAFLabelTreeModel* m_xcafTreeModel;
    QTreeWidget* m_simpleTreeWidget;
    Handle(TDocStd_Document) m_doc; // Temporary simple tree widget
    
    // Simulation Components
    std::shared_ptr<CadNode> m_simulationRootNode;
    CadOpenGLWidget* m_simulationViewer;
    CadTreeModel* m_simulationTreeModel;
    std::unique_ptr<SimulationManager> m_simulationManager;
    
    // File handling
    QString m_currentFilePath;
    bool m_isModified;
    
    // Menu actions
    QAction* m_openAction;
    QAction* m_saveAction;
    QAction* m_saveAsAction;
    QAction* m_exportAction;
    QAction* m_sampleAction;
    QAction* m_exitAction;
    QAction* m_aboutAction;
    
    // Toolbar actions
    QAction* m_reframeAction;
    QAction* m_loadSimulationAction;
    QAction* m_startPhysicsAction;
    QAction* m_stopPhysicsAction;
};

#endif // CADVIEWERDIALOG_H 
