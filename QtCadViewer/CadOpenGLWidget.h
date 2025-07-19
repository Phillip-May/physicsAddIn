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

#include "CadNode.h"

// Selection mode enum
enum class SelectionMode {
    None,
    Faces,
    Edges
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

class CadOpenGLWidget : public QOpenGLWidget, protected QOpenGLFunctions {
    Q_OBJECT
public:
    explicit CadOpenGLWidget(QWidget *parent = nullptr);
    ~CadOpenGLWidget();
    void setRootTreeNode(TreeNode* root);
    void setPivotSphere(const QVector3D& worldPos);
    void clearCache(); // Clear geometry cache to free memory
    int getCacheSize() const; // Get number of cached geometries
    // Selection mode control
    void setSelectionMode(SelectionMode mode);
    SelectionMode getSelectionMode() const { return m_selectionMode; }
    // Add this slot for tree selection
public slots:
    void setSelectedNode(TreeNode* node);
    void clearSelection();
    void addToSelection(TreeNode* node);
    void removeFromSelection(TreeNode* node);
    void setCamera(const QVector3D& pos, const QQuaternion& rot, float zoom = 1.0f);
    void reframeCamera();
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
    TreeNode* rootNode_ = nullptr;
    // New camera state
    QVector3D m_cameraPos;      // Camera position in world space
    QQuaternion m_cameraRot;    // Camera orientation (quaternion)
    float m_cameraZoom = 1.0f;  // Camera zoom (scaling or dolly factor)
    // --- Camera pivot interaction ---
    QPoint m_pivotScreenPos; // Screen position where pivot was set
    float m_pivotDepth = 0.0f; // Depth from camera to pivot
    bool m_showPivotSphere = false; // Show sphere at pivot while rotating
    QVector3D m_pivotWorldPos; // Persistent camera pivot point
    std::vector<ColorBatch> m_colorBatches;
    std::vector<std::pair<QVector3D, QVector3D>> m_wireframeLines; // Tessellated wireframe lines
    
    // Selection state
    TopoDS_Face selectedFace_;
    TopoDS_Edge selectedEdge_;
    TopoDS_Face hoveredFace_;
    TopoDS_Edge hoveredEdge_;
    TreeNode* selectedFaceNode_ = nullptr;
    TreeNode* hoveredFaceNode_ = nullptr;
    TreeNode* selectedEdgeNode_ = nullptr;
    TreeNode* hoveredEdgeNode_ = nullptr;
    
    // Multi-selection support
    std::vector<TreeNode*> selectedFaceNodes_;
    std::vector<TreeNode*> selectedEdgeNodes_;
    
    // Selection mode
    SelectionMode m_selectionMode = SelectionMode::None;
    
    // Continuous rendering timer for smooth updates
    QTimer m_renderTimer;
    bool m_windowActive = true;
    
    // Mouse state for click vs drag detection
    QPoint m_mousePressPos;
    bool m_mousePressed = false;
    bool m_mouseDragged = false;

    // Flat cache of all visible faces (per instance)
    struct FaceInstance {
        TreeNode* node;
    };
    std::vector<FaceInstance> faceCache_;
    void buildFaceCache();
    
    // Flat cache of all visible edges (per instance)
    struct EdgeInstance {
        TreeNode* node;
    };
    std::vector<EdgeInstance> edgeCache_;
    void buildEdgeCache();
    void traverseAndRender(TreeNode *node, CADNodeColor inheritedColor, bool ancestorsVisible);
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
    void renderFace(TreeNode* node, const CADNodeColor& color);
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
    void renderFaceOptimized(TreeNode* node, const CADNodeColor& color);
    void renderEdgeOptimized(TreeNode* node, const CADNodeColor& color);
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
signals:
    void facePicked(TreeNode* node);
    void edgePicked(TreeNode* node);
};
