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

// Geometry cache structure using legacy OpenGL
struct CachedGeometry {
    GLuint displayList = 0;
    int triangleCount = 0;
    bool initialized = false;
    
    CachedGeometry() {}
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
    // Add this slot for tree selection
public slots:
    void setSelectedNode(TreeNode* node);
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
    TopoDS_Face hoveredFace_; // <--- Add this line
    TreeNode* selectedFaceNode_ = nullptr;
    TreeNode* hoveredFaceNode_ = nullptr;

    // Flat cache of all visible faces (per instance)
    struct FaceInstance {
        TreeNode* node;
    };
    std::vector<FaceInstance> faceCache_;
    void buildFaceCache();
    void traverseAndRender(TreeNode *node, CADNodeColor inheritedColor, bool ancestorsVisible);
    void renderEdge(const TopoDS_Edge &edge, const CADNodeColor &color);
    // Overload: pick and output intersection point (returns true if hit)
    bool pickElementAt(const QPoint& pos, QVector3D* outIntersection = nullptr);
    // Helper to pick hovered face
    void pickHoveredFaceAt(const QPoint& pos);
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
    CachedGeometry& getOrCreateCachedGeometry(const TopoDS_Face& face);
    
    // Frustum culling helpers
    bool isInFrustum(const TopoDS_Face& face, const TopLoc_Location& loc);
    void updateFrustumPlanes();
    
    // Frustum planes (normal + distance)
    QVector4D m_frustumPlanes[6];
signals:
    void facePicked(TreeNode* node);
};
