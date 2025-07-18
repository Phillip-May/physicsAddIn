#include "CadOpenGLWidget.h"
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Tool.hxx>
#include <Poly_Triangulation.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Pnt.hxx>
#include <QDebug>
#include <QMouseEvent>
#include <QWheelEvent>
#include <GL/glu.h>
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <QVector3D>
#include <cmath>
#include <QOpenGLShaderProgram>
#include <QOpenGLContext>
#include <Geom_Curve.hxx>
#include <Standard_Type.hxx>
#include <TopoDS_Edge.hxx>
#include <map>
#include <QOpenGLFunctions>

#include "CadNode.h"

CadOpenGLWidget::CadOpenGLWidget(QWidget *parent)
    : QOpenGLWidget(parent) {}

// Camera state
static float g_camRotX = 30.0f;
static float g_camRotY = -30.0f;
static float g_camDist = 500.0f;
static QPoint g_lastMousePos;



void CadOpenGLWidget::initializeGL()
{
    initializeOpenGLFunctions();
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glClearColor(0.1f, 0.1f, 0.15f, 1.0f);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    double aspect = double(width()) / double(height() ? height() : 1);
    gluPerspective(45.0, aspect, 1.0, 1e9);
    glMatrixMode(GL_MODELVIEW);
}

void CadOpenGLWidget::resizeGL(int w, int h) {
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    double aspect = double(w) / double(h ? h : 1);
    gluPerspective(45.0, aspect, 1.0, 1e9);
    glMatrixMode(GL_MODELVIEW);
}

// Update traverseAndRender to only check the node's own visible flag
void CadOpenGLWidget::traverseAndRender(TreeNode* node, CADNodeColor inheritedColor, bool /*ancestorsVisible*/) {
    if (!node) return;
    if (node->visible) {
        CADNodeColor nodeColor = (node->color.a >= 0.0f) ? node->color : inheritedColor;
        const TopLoc_Location& fullLoc = node->loc;
        double matrix[16] = {
            1,0,0,0,
            0,1,0,0,
            0,0,1,0,
            0,0,0,1
        };
        if (!fullLoc.IsIdentity()) {
            const gp_Trsf& trsf = fullLoc.Transformation();
            const gp_Mat& mat = trsf.VectorialPart();
            const gp_XYZ& trans = trsf.TranslationPart();
            matrix[0] = mat.Value(1,1); matrix[1] = mat.Value(2,1); matrix[2] = mat.Value(3,1); matrix[3] = 0.0;
            matrix[4] = mat.Value(1,2); matrix[5] = mat.Value(2,2); matrix[6] = mat.Value(3,2); matrix[7] = 0.0;
            matrix[8] = mat.Value(1,3); matrix[9] = mat.Value(2,3); matrix[10] = mat.Value(3,3); matrix[11] = 0.0;
            matrix[12] = trans.X();     matrix[13] = trans.Y();     matrix[14] = trans.Z();     matrix[15] = 1.0;
        }
        // Debug print: node->loc and matrix
        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        glMultMatrixd(matrix);

        switch (node->type) {
            case TopAbs_FACE:
                if (node->hasFace()) {
                    renderFace(node->getFace(), nodeColor);
                }
                break;
            case TopAbs_EDGE:
                if (node->hasEdge()) {
                    renderEdge(node->getEdge(), nodeColor);
                }
                break;
            default:
                break;
        }
        glPopMatrix();
    }
    // Always traverse children, regardless of this node's visibility
    for (const auto& child : node->children) {
        if (child) {
            traverseAndRender(child.get(), node->color, true);
        }
    }
}

void CadOpenGLWidget::paintGL() {
    QOpenGLFunctions* f = QOpenGLContext::currentContext()->functions();
    f->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslatef(0, 0, -g_camDist);
    glRotatef(g_camRotX, 1, 0, 0);
    glRotatef(g_camRotY, 0, 1, 0);
    glTranslatef(m_panOffset.x(), m_panOffset.y(), m_panOffset.z());
    glTranslatef(-m_center.x(), -m_center.y(), -m_center.z());
    // Draw geometry from tree with color inheritance
    if (rootNode_) {
        qDebug() << "Rendering tree with" << rootNode_->children.size() << "root children";
        for (const auto& child : rootNode_->children) {
            traverseAndRender(child.get(), CADNodeColor(1.0f, 1.0f, 1.0f, 1.0f), true);
        }
    } else {
        qDebug() << "No root node to render";
    }
    // Draw XYZ axes
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslatef(0, 0, -g_camDist);
    glRotatef(g_camRotX, 1, 0, 0);
    glRotatef(g_camRotY, 0, 1, 0);
    glTranslatef(m_panOffset.x(), m_panOffset.y(), m_panOffset.z());
    glTranslatef(-m_center.x(), -m_center.y(), -m_center.z());
    glLineWidth(2.0f);
    glBegin(GL_LINES);
    glColor3f(1,0,0); glVertex3f(0,0,0); glVertex3f(100,0,0);
    glColor3f(0,1,0); glVertex3f(0,0,0); glVertex3f(0,100,0);
    glColor3f(0,0,1); glVertex3f(0,0,0); glVertex3f(0,0,100);
    glEnd();
    glLineWidth(1.0f);
}

void CadOpenGLWidget::mousePressEvent(QMouseEvent* event) {
    g_lastMousePos = event->pos();
}
void CadOpenGLWidget::mouseMoveEvent(QMouseEvent* event) {
    int dx = event->x() - g_lastMousePos.x();
    int dy = event->y() - g_lastMousePos.y();
    if (event->buttons() & Qt::LeftButton) {
        float panSpeed = g_camDist * 0.002f;
        float angleY = g_camRotY * M_PI / 180.0f;
        float angleX = g_camRotX * M_PI / 180.0f;
        QVector3D right(cos(angleY), 0, -sin(angleY));
        QVector3D up(
            sin(angleX) * sin(angleY),
            cos(angleX),
            sin(angleX) * cos(angleY)
        );
        m_panOffset += -right * dx * panSpeed + up * dy * panSpeed;
        update();
    } else if (event->buttons() & Qt::RightButton) {
        g_camRotY += dx * 0.5f;
        g_camRotX += dy * 0.5f;
        update();
    }
    g_lastMousePos = event->pos();
}
void CadOpenGLWidget::wheelEvent(QWheelEvent* event) {
    g_camDist -= event->angleDelta().y() * 0.1f;
    update();
}


void CadOpenGLWidget::setRootTreeNode(TreeNode* root) {
    rootNode_ = root;
    update();
}

void CadOpenGLWidget::renderFace(const TopoDS_Face& face, const CADNodeColor& color) {
    if (face.IsNull()) {
        qDebug() << "Null face data provided";
    }
    if (face.IsNull()) return;
    glDisable(GL_LIGHTING); // Ensure flat coloring
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    BRepMesh_IncrementalMesh mesher(face, 0.1); // Ensure the face is meshed
    TopLoc_Location locCopy;
    Handle(Poly_Triangulation) triangulation = BRep_Tool::Triangulation(face, locCopy);
    if (triangulation.IsNull()) {
        qDebug() << "No triangulation available for face";
        return;
    }
    const Poly_Array1OfTriangle& triangles = triangulation->Triangles();
    if (triangles.IsEmpty()) {
        qDebug() << "No triangles in face triangulation";
        return;
    }
    int nodeLower = 1;
    int nodeUpper = triangulation->NbNodes();
    glColor4f(color.r, color.g, color.b, color.a);
    glBegin(GL_TRIANGLES);
    for (int i = triangles.Lower(); i <= triangles.Upper(); ++i) {
        int n1, n2, n3;
        triangles(i).Get(n1, n2, n3);
        if (n1 < nodeLower || n1 > nodeUpper ||
            n2 < nodeLower || n2 > nodeUpper ||
            n3 < nodeLower || n3 > nodeUpper) {
            continue; // Skip invalid triangles
        }
        gp_Pnt p1 = triangulation->Node(n1);
        gp_Pnt p2 = triangulation->Node(n2);
        gp_Pnt p3 = triangulation->Node(n3);
        glVertex3d(p1.X(), p1.Y(), p1.Z());
        glVertex3d(p2.X(), p2.Y(), p2.Z());
        glVertex3d(p3.X(), p3.Y(), p3.Z());
    }
    glEnd();
    glColor4f(0.0f, 0.0f, 0.0f, 0.5f); // Black wireframe
    glLineWidth(1.0f);
    glBegin(GL_LINE_LOOP);
    for (int i = triangles.Lower(); i <= triangles.Upper(); ++i) {
        int n1, n2, n3;
        triangles(i).Get(n1, n2, n3);
        if (n1 < nodeLower || n1 > nodeUpper ||
            n2 < nodeLower || n2 > nodeUpper ||
            n3 < nodeLower || n3 > nodeUpper) {
            continue; // Skip invalid triangles
        }
        gp_Pnt p1 = triangulation->Node(n1);
        gp_Pnt p2 = triangulation->Node(n2);
        gp_Pnt p3 = triangulation->Node(n3);
        // Do NOT apply the triangulation's location here
        // p1.Transform(locCopy.Transformation());
        // p2.Transform(locCopy.Transformation());
        // p3.Transform(locCopy.Transformation());
        glVertex3d(p1.X(), p1.Y(), p1.Z());
        glVertex3d(p2.X(), p2.Y(), p2.Z());
        glVertex3d(p3.X(), p3.Y(), p3.Z());
    }
    glEnd();
}

// Improved renderEdge implementation
void CadOpenGLWidget::renderEdge(const TopoDS_Edge& edge, const CADNodeColor& color) {
    if (edge.IsNull()) return;
    
    glDisable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);
    glColor4f(color.r, color.g, color.b, color.a);
    glLineWidth(2.0f); // Make edges more visible
    
    Standard_Real first, last;
    TopLoc_Location locCopy;
    Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, locCopy, first, last);
    
    if (curve.IsNull()) {
        qDebug() << "No curve available for edge";
        return;
    }
    
    // Check if parameter range is valid
    if (first >= last) {
        qDebug() << "Invalid parameter range for edge curve";
        return;
    }
    
    // Determine number of segments based on curve length
    Standard_Real curveLength = last - first;
    int N = std::max(20, static_cast<int>(curveLength * 10)); // At least 20 segments, more for longer curves
    
    glBegin(GL_LINE_STRIP);
    for (int i = 0; i <= N; ++i) {
        Standard_Real t = first + (last - first) * i / N;
        gp_Pnt p;
        
        try {
            curve->D0(t, p);
            // Do NOT apply locCopy.Transformation() here
            // p.Transform(locCopy.Transformation());
            glVertex3d(p.X(), p.Y(), p.Z());
        } catch (const Standard_Failure&) {
            // Skip invalid parameter values
            continue;
        }
    }
    glEnd();
    
    glLineWidth(1.0f); // Reset line width
} 
