#pragma once

#include <QString>
#include <TDocStd_Document.hxx>

#include <memory>
#include <vector>

struct CadNode;
class CadOpenGLWidget;
class QAbstractItemModel;
class QTabWidget;
class QTreeView;
class SimulationManager;

struct CadViewerPane {
    QTreeView* tree = nullptr;
    CadOpenGLWidget* viewer = nullptr;
};

class CadViewerWorkspace {
public:
    void addPane(CadViewerPane pane) { m_panes.push_back(pane); }
    const std::vector<CadViewerPane>& panes() const { return m_panes; }

private:
    std::vector<CadViewerPane> m_panes;
};

void setupComprehensiveContextMenu(QTreeView* treeView,
                                   QAbstractItemModel* model,
                                   const Handle(TDocStd_Document)& doc,
                                   CadOpenGLWidget* openGLViewer = nullptr,
                                   const CadViewerWorkspace* workspace = nullptr);

CadViewerPane initTreeAndOpenGLWidget(std::shared_ptr<CadNode>& inputRoot,
                                      QTabWidget* treeTabWidget,
                                      QTabWidget* openGLTabWidget,
                                      const QString& name,
                                      const Handle(TDocStd_Document)& doc,
                                      CadViewerWorkspace& workspace,
                                      SimulationManager* simManager = nullptr);
