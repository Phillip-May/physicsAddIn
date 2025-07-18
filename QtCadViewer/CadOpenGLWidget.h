#pragma once
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QVector3D>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <memory>
#include <vector>
#include <Bnd_Box.hxx>
#include <TopoDS_Face.hxx>
#include <map>

#include "CadNode.h"

struct ColorBatch {
    CADNodeColor color;
    std::vector<float> vertices; // All triangles for this color
    int vertexCount = 0;
    QOpenGLBuffer vbo;
    std::unique_ptr<QOpenGLVertexArrayObject> vao;
    ColorBatch(const CADNodeColor& c) : color(c), vbo(QOpenGLBuffer::VertexBuffer) {}
};

class CadOpenGLWidget : public QOpenGLWidget, protected QOpenGLFunctions {
    Q_OBJECT
public:
    explicit CadOpenGLWidget(QWidget *parent = nullptr);
    void setRootTreeNode(TreeNode* root);
protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
private:
    TreeNode* rootNode_ = nullptr;
    QVector3D m_center;
    QVector3D m_panOffset; // Camera pan offset
    std::vector<ColorBatch> m_colorBatches;
    std::vector<std::pair<QVector3D, QVector3D>> m_wireframeLines; // Tessellated wireframe lines
    
    void traverseAndRender(TreeNode *node, CADNodeColor inheritedColor, bool ancestorsVisible);
    void renderFace(const TopoDS_Face& face, const CADNodeColor& color);
    void renderEdge(const TopoDS_Edge &edge, const CADNodeColor &color);
};
