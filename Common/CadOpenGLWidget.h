#pragma once
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <GL/gl.h>
#include <QVector3D>
#include <memory>
#include <vector>
#include <Bnd_Box.hxx>
#include <TopoDS_Face.hxx>
#include <map>
#include <QQuaternion>
#include <QTimer>
#include <vector>

#include "CadNode.h"
#include "SimulationManager.h"

// Selection mode enum
enum class SelectionMode {
    None,
    Faces,
    Edges
};

// Highlight state for faces/edges
enum class HighlightState {
    Normal,
    Hovered,
    Selected,
    Excluded
};

// Geometry cache structure using legacy OpenGL
struct CachedGeometry {
    GLuint displayList = 0;
    int triangleCount = 0;
    bool initialized = false;
    
    CachedGeometry() {}
    
    // Helper to check if this geometry is valid for rendering
    bool isValid() const {
        return initialized && displayList != 0 && triangleCount > 0;
    }
    
    // Helper to mark as invalid
    void markInvalid() {
        initialized = true;
        displayList = 0;
        triangleCount = 0;
    }
};

struct ColorBatch {
    CADNodeColor color;
    std::vector<float> vertices; // All triangles for this color
    int vertexCount = 0;
    ColorBatch(const CADNodeColor& c) : color(c) {}
};

// Camera state struct for sharing logic
struct CameraState {
    QVector3D pos{0,0,0};
    QQuaternion rot;
    float zoom = 1.0f;
    // Pivot interaction
    QPoint pivotScreenPos;
    float pivotDepth = 0.0f;
    bool showPivotSphere = false;
    QVector3D pivotWorldPos;
    // Mouse state for click vs drag detection
    QPoint mousePressPos;
    bool mousePressed = false;
    bool mouseDragged = false;
    // For rotation
    bool rotating = false;
    bool firstRotateMove = false;
    QPoint pivotScreenOnPress;
    QVector3D cameraPosOnPress;
    QQuaternion cameraRotOnPress;
    float cameraZoomOnPress = 0.0f;
};

class CadOpenGLWidget : public QOpenGLWidget, protected QOpenGLFunctions {
    Q_OBJECT
public:
    explicit CadOpenGLWidget(CadNode* root, QWidget *parent = nullptr);
    ~CadOpenGLWidget();
    void setPivotSphere(const QVector3D& worldPos);
    void clearCache(); // Clear geometry cache to free memory
    int getCacheSize() const; // Get number of cached geometries
    // Selection mode control
    void setSelectionMode(SelectionMode mode);
    SelectionMode getSelectionMode() const { return m_selectionMode; }
    // Camera state access
    CameraState getCameraState() const;
    void setCameraState(const CameraState& state);
    // Add this slot for tree selection
    static void collectFaceNodes(CadNode *node, std::vector<CadNode*> &out);
    CadNode* getRootTreeNode() const { return rootNode_; }
    // Add this slot for tree selection
    Q_SLOT void setSelectedFaceNode(CadNode* node, const TopLoc_Location& accLoc);
    struct FaceInstance {
        CadNode* node;
        TopLoc_Location accumulatedLoc;
    };
    struct EdgeInstance {
        CadNode* node;
        TopLoc_Location accumulatedLoc;
    };
    struct SelectedInstance {
        CadNode* node;
        TopLoc_Location accumulatedLoc;
        bool operator==(const SelectedInstance& other) const {
            return node == other.node && accumulatedLoc == other.accumulatedLoc;
        }
    };
public slots:
    void clearSelection();
    void addToSelection(CadNode* node, const TopLoc_Location& accLoc);
    void removeFromSelection(CadNode* node);
    void setCamera(const QVector3D& pos, const QQuaternion& rot, float zoom = 1.0f);
    void reframeCamera();
    void markCacheDirty(); // Force cache to be rebuilt on next paint
    void drawReferenceFrame(const TopLoc_Location& loc, float axisLength);
    CadNode* getSelectedFrameNode() const { return selectedFrameNode_; }
    void setSelectedFrameNode(CadNode* node, const TopLoc_Location& accLoc) { selectedFrameNode_ = node; selectedFrameNodeAccumulatedLoc_ = accLoc; update(); }
    void setSelectedFrameNode(CadNode* node) { selectedFrameNode_ = node; selectedFrameNodeAccumulatedLoc_ = node ? node->loc : TopLoc_Location(); update(); }
    void setSimulationManager(SimulationManager* simManager) { m_simulationManager = simManager; }
protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    void leaveEvent(QEvent* event) override;
    void focusInEvent(QFocusEvent* event) override;
    void focusOutEvent(QFocusEvent* event) override;
private:
    static HighlightState getFaceHighlightState(const CadNode* node, const TopLoc_Location& accumulatedLoc, const CadNode* parentReferenceNode, const std::vector<SelectedInstance>& selectedFaceInstances_, const SelectedInstance& hoveredFaceInstance_);
    static HighlightState getEdgeHighlightState(const CadNode* node, const TopLoc_Location& accumulatedLoc, const std::vector<SelectedInstance>& selectedEdgeInstances_, const SelectedInstance& hoveredEdgeInstance_);
    static void getHighlightColorAndLineWidth(HighlightState state, CADNodeColor baseColor, float& outR, float& outG, float& outB, float& outA, float& outLineWidth);
    void setupOpenGLState(); // Unified OpenGL state setup
    CadNode* const rootNode_;
    CameraState camera_;
    std::vector<ColorBatch> m_colorBatches;
    std::vector<std::pair<QVector3D, QVector3D>> m_wireframeLines; // Tessellated wireframe lines
    
    // Selection state
    TopoDS_Face selectedFace_;
    TopoDS_Edge selectedEdge_;
    TopoDS_Face hoveredFace_;
    TopoDS_Edge hoveredEdge_;
    CadNode* selectedFaceNode_ = nullptr;
    CadNode* selectedEdgeNode_ = nullptr;
    CadNode* selectedFrameNode_ = nullptr;
    TopLoc_Location selectedFrameNodeAccumulatedLoc_;
    SimulationManager* m_simulationManager = nullptr;  // Reference to simulation manager for double-buffered node locations
    
    // Multi-selection support (instance-aware)
    std::vector<SelectedInstance> selectedFaceInstances_;
    std::vector<SelectedInstance> selectedEdgeInstances_;
    SelectedInstance hoveredFaceInstance_;
    SelectedInstance hoveredEdgeInstance_;
    
    // Selection mode
    SelectionMode m_selectionMode = SelectionMode::None;
    
    // Continuous rendering timer for smooth updates
    QTimer m_renderTimer;
    bool m_windowActive = true;
    
    // Flat cache of all visible faces (per instance)
    std::vector<FaceInstance> faceCache_;
    void buildFaceCache();
    
    // Flat cache of all visible edges (per instance)
    std::vector<EdgeInstance> edgeCache_;
    void buildEdgeCache();
    // Update: Add accumulatedLoc parameter for hierarchical transform
    void traverseAndRender(const CadNode *node, CADNodeColor inheritedColor, const TopLoc_Location& accumulatedLoc, bool ancestorsVisible, const CadNode* parentReferenceNode = nullptr);
    void renderEdge(const TopoDS_Edge &edge, const CADNodeColor &color);
    // Overload: pick and output intersection point (returns true if hit)
    bool pickElementAt(const QPoint& pos, QVector3D* outIntersection = nullptr);
    // Edge picking function
    bool pickEdgeAt(const QPoint& pos, QVector3D* outIntersection = nullptr);
    // Picking function for pivot setting that doesn't change selection
    bool pickElementAtForPivot(const QPoint& pos, QVector3D* outIntersection = nullptr);
    // Helper to pick hovered face
    void pickHoveredFaceAt(const QPoint& pos);
    // Helper to pick hovered edge
    void pickHoveredEdgeAt(const QPoint& pos);
    void renderFace(const XCAFNodeData* node, const CADNodeColor& color);
    void drawPivotSphere(); // Draws a sphere at m_center
    QVector3D m_zoomContactPoint; // Last zoom contact point
    bool m_showZoomContactSphere = false;
    QTimer m_zoomContactTimer;
    bool m_rotating = false;
    bool m_firstRotateMove = false;
    QPoint m_pivotScreenOnPress;
    QVector3D m_cameraPosOnPress;
    QQuaternion m_cameraRotOnPress;
    float m_cameraZoomOnPress = 0.0f;
    
    // Optimized rendering functions
    void renderBatchedGeometry();
    void buildColorBatches();
    void renderFaceOptimized(const CadNode* node, const TopLoc_Location& accumulatedLoc, const CadNode* parentReferenceNode = nullptr);
    void renderEdgeOptimized(const CadNode* node, const TopLoc_Location& accumulatedLoc);
    void renderHighlightedEdges();
    CachedGeometry& getOrCreateCachedGeometry(const TopoDS_Face& face);
    
    // Frustum culling helpers
    bool isInFrustum(const TopoDS_Face& face, const TopLoc_Location& loc);
    void updateFrustumPlanes();
    
    // Frustum planes (normal + distance)
    QVector4D m_frustumPlanes[6];
    
    // Cache-related member variables (previously globals)
    std::map<const void*, CachedGeometry> m_geometryCache;
    bool m_cacheDirty = true;
    int m_frameCount = 0; // Frame counter for performance monitoring
    int m_skipFrames = 0; // Frame skipping counter
    int m_totalFaces = 0; // Total faces processed
    int m_skippedFaces = 0; // Faces skipped
    void clearGeometryCache();
    void drawBoundingBox(const QVector3D& min, const QVector3D& max, const QVector4D& color);
    void renderConvexHulls(const PhysicsNodeData* physData);
    void renderConnectionPoint(const CadNode* node, const CADNodeColor& color);
    void renderGroundPlane(const MutexRootNodeData& mutexData);

signals:
    void facePicked(CadNode* node, const TopLoc_Location& accLoc);
    void edgePicked(CadNode* node);
};
