#ifndef QTCADVIEWER_HELPERFUNCTIONS_H
#define QTCADVIEWER_HELPERFUNCTIONS_H

#include "CadNode.h"
#include "XCAFLabelTreeModel.h"
#include "CadTreeModel.h"
#include <memory>
#include <QString>
#include <QTreeView>
#include <QTabWidget>
#include <QSplitter>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFileDialog>
#include <QMessageBox>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonDocument>
#include <QFile>
#include <QJsonParseError>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Edge.hxx>
#include <TopLoc_Location.hxx>
#include <TDF_Label.hxx>
#include <TDF_ChildIterator.hxx>
#include <TDF_LabelSequence.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFApp_Application.hxx>
#include <TDocStd_Document.hxx>
#include <TDocStd_Application.hxx>
#include <BinDrivers.hxx>
#include <XmlDrivers.hxx>
#include <TCollection_ExtendedString.hxx>
#include <vector>
#include <set>
#include <unordered_map>
#include <QVariant>
#include <QVector3D>
#include <QMenu>
#include <QAction>
#include <QInputDialog>
#include <QLineEdit>
#include <QTextEdit>
#include <QCheckBox>
#include <QComboBox>
#include <QFormLayout>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QDialog>
#include <QPoint>
#include <QDebug>
#include <QGroupBox>
#include <QGridLayout>
#include "SimulationManager.h"

// Forward declarations
class CadOpenGLWidget;
class CadTreeModel;
class CustomModelTreeModel;
class RailJsonEditorDialog;

// Global vectors for tracking tree views and OpenGL widgets
extern std::vector<QTreeView*> g_treeViews;
extern std::vector<CadOpenGLWidget*> g_openGLViews;

// All helper function declarations from main.cpp go here
void setParentPointersRecursive(CadNode* node, CadNode* parent = nullptr);
std::unique_ptr<XCAFLabelNode> buildLabelTree(const TDF_Label& label, const Handle(XCAFDoc_ShapeTool)& shapeTool = nullptr);
std::unique_ptr<XCAFLabelNode> buildLabelTreeWithReferences(const TDF_Label& label, const Handle(XCAFDoc_ShapeTool)& shapeTool = nullptr);
// Template implementation must be in the header
// Recursively set globalLoc for each node
// (Moved here to ensure it's declared before use)
template<typename NodeType>
void setGlobalLocRecursive(NodeType* node, const TopLoc_Location& parentGlobalLoc = TopLoc_Location()) {
    if (!node) return;
    node->globalLoc = parentGlobalLoc * node->loc;
    for (auto& child : node->children) {
        setGlobalLocRecursive(child.get(), node->globalLoc);
    }
}
uint qHash(const TDF_Label& label, uint seed = 0);
QString shapeTypeToString(TopAbs_ShapeEnum type);
CADNodeColor getEffectiveFaceColor(const TopoDS_Face& face, const Handle(XCAFDoc_ShapeTool)& shapeTool, const Handle(XCAFDoc_ColorTool)& colorTool);
CADNodeColor get_label_color(const TDF_Label& label, const Handle(XCAFDoc_ColorTool)& colorTool, const CADNodeColor& parentColor);
CADNodeColor get_shape_color(const TopoDS_Shape& shape, const TDF_Label& label, const Handle(XCAFDoc_ShapeTool)& shapeTool, const Handle(XCAFDoc_ColorTool)& colorTool, const CADNodeColor& parentColor);
bool hasSubAssembliesRecursive(const TDF_Label& label, const Handle(XCAFDoc_ShapeTool)& shapeTool);
QString makeTransformString(const TopLoc_Location& loc);
bool isAssembly(const TDF_Label& label, const Handle(XCAFDoc_ShapeTool)& shapeTool);
std::vector<int> getLabelPath(const TDF_Label& label, const Handle(TDocStd_Document)& doc);
TDF_Label findLabelByPath(const Handle(TDocStd_Document)& doc, const std::vector<int>& path);
void debugPrintXCAFRelinkInfo(const CadNode* node, const Handle(TDocStd_Document)& doc, int depth = 0);
TopLoc_Location getEffectiveTransform(const TDF_Label& label, const Handle(XCAFDoc_ShapeTool)& shapeTool, const TopLoc_Location& parentLoc);
struct FaceEdgeKeyHash;
std::shared_ptr<CadNode> build_tree_xcaf(const TDF_Label& label, const Handle(XCAFDoc_ShapeTool)& shapeTool, const Handle(XCAFDoc_ColorTool)& colorTool, const CADNodeColor& parentColor, const TopLoc_Location& parentLoc, const Handle(TDocStd_Document)& doc);
bool saveXCAFToSTEP(const Handle(TDocStd_Document)& doc, const QString& filename);
bool saveXCAFToBinary(const Handle(TDocStd_Document)& doc, const QString& filename);
void compareFileSizes(const QString& baseName);
template <typename ModelType>
void connectTreeAndViewer(QTreeView* tree, CadOpenGLWidget* viewer, ModelType* model);
struct FaceWithTransform {
    CadNode* node;
    TopLoc_Location accumulatedLoc;
};
void collectFaceNodesWithTransform(CadNode* node, const TopLoc_Location& parentLoc, std::vector<FaceWithTransform>& out);
// Advanced VHACD settings exposed for user control
// Refactored: Pass all VHACD parameters as a QJsonObject
void generateVHACDStub(const QString& nodeName, const QJsonObject& params, CadNode* node);
void generateCoACDStub(const QString& nodeName, double concavity, double alpha, double beta, CadNode* node, int maxConvexHull, std::string preprocess, int prepRes, int sampleRes, int mctsNodes, int mctsIter, int mctsDepth, bool pca, bool merge, bool decimate, int maxChVertex, bool extrude, double extrudeMargin, std::string apxMode, int seed);
std::shared_ptr<CadNode> deepCopyNodeNonExcluded(const CadNode* src);
void insertCustomModelNodeAtCadTreePosition(CadNode* customModelRoot, std::shared_ptr<CadNode> newNode, const std::vector<CadNode*>& selectedCadNodes, CadTreeModel* cadModel);
void adjustSubtreeTransforms(const CadNode* src, CadNode* copy, const TopLoc_Location& baseLoc, std::function<const CadNode*(const CadNode*)> getParent);
double computeBoundingBoxLength(const CadNode* node, const QVector3D& axis);
void addRailToPhysicsPreview(CadNode* railNode, std::shared_ptr<CadNode> physicsPreviewRoot);
void expandRailInPhysicsPreview(CadNode* railNode, std::shared_ptr<CadNode> physicsPreviewRoot, CadOpenGLWidget* oglWidget);

// Shared CAD viewer functions
bool loadStepFile(const QString& stepFile,
                  Handle(TDocStd_Document)& doc,
                  std::shared_ptr<CadNode>& cadRoot,
                  std::unique_ptr<XCAFLabelNode>& labelRoot,
                  Handle(XCAFDoc_ShapeTool)& shapeTool,
                  Handle(XCAFDoc_ColorTool)& colorTool);

template <typename ModelType>
void connectTreeAndViewer(QTreeView* tree, CadOpenGLWidget* viewer, ModelType* model);

void setupContextMenu(QTreeView* treeView, CustomModelTreeModel* model, const Handle(TDocStd_Document)& doc);

// New comprehensive context menu setup function that can handle different node types
void setupComprehensiveContextMenu(QTreeView* treeView, 
                                   QAbstractItemModel* model, 
                                   const Handle(TDocStd_Document)& doc,
                                   CadOpenGLWidget* openGLViewer = nullptr);

void initTreeAndOpenGLWidget(std::shared_ptr<CadNode>& inputRoot,
                             QTabWidget* treeTabWidget,
                             QTabWidget* openGLTabWidget,
                             const QString& name,
                             const Handle(TDocStd_Document)& doc,
                             SimulationManager* simManager = nullptr);
TDF_Label findLabelForShape(const Handle(XCAFDoc_ShapeTool)& shapeTool, const TDF_Label& label, const TopoDS_Shape& targetShape);
TDF_Label findLabelForFaceOrEdge(const Handle(XCAFDoc_ShapeTool)& shapeTool, const TDF_Label& label, const TopoDS_Shape& targetShape);
bool loadFromJsonAndBin(const QString& railJsonFile, Handle(TDocStd_Document)& doc, std::shared_ptr<CadNode>& cadRoot, std::unique_ptr<XCAFLabelNode>& labelRoot, Handle(XCAFDoc_ShapeTool)& shapeTool, Handle(XCAFDoc_ColorTool)& colorTool, std::shared_ptr<CadNode>& customModelRootContainer, std::shared_ptr<CadNode>& customModelRoot, bool& loadedFromJsonBin);
TopoDS_Shape extrudeFace(const TopoDS_Face& face, double distance);
bool loadFromStep(const QString& stepFile, Handle(TDocStd_Document)& doc, std::shared_ptr<CadNode>& cadRoot, std::unique_ptr<XCAFLabelNode>& labelRoot, Handle(XCAFDoc_ShapeTool)& shapeTool, Handle(XCAFDoc_ColorTool)& colorTool);

// Connection management functions
std::shared_ptr<CadNode> createConnectionBetweenPoints(
    const std::string& connectionName,
    CadNode* point1,
    CadNode* point2,
    ConnectionNodeData::ConnectionType connectionType = ConnectionNodeData::ConnectionType::Cable);

bool canConnectPoints(CadNode* point1, CadNode* point2);
double calculateDistanceBetweenPoints(CadNode* point1, CadNode* point2);
double calculateManhattanDistance(CadNode* point1, CadNode* point2);
double calculatePathDistance(CadNode* point1, CadNode* point2, const std::vector<gp_Pnt>& waypoints);
double calculateDragChainLength(CadNode* point1, CadNode* point2, double bendRadius, double extraLength);
double calculateDragChainLengthWithAttachments(CadNode* point1, CadNode* point2, double bendRadius, double pitchLength, int extraSegments,
                                             const ConnectionNodeData::AttachmentConfig& startAttachment,
                                             const ConnectionNodeData::AttachmentConfig& endAttachment);
std::vector<CadNode*> findConnectionPointsInTree(CadNode* root);

// Helper function to find common ancestor of two nodes
CadNode* findCommonAncestor(CadNode* node1, CadNode* node2);

// Helper function to find the best placement location for a connection
CadNode* findBestConnectionPlacement(CadNode* point1, CadNode* point2);

#endif // QTCADVIEWER_HELPERFUNCTIONS_H 
