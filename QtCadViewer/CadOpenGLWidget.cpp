#include "CadOpenGLWidget.h"
#include <array>
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
#include <QOpenGLContext>
#include <Geom_Curve.hxx>
#include <Standard_Type.hxx>
#include <TopoDS_Edge.hxx>
#include <map>
#include <QOpenGLFunctions>
#include <gp_Lin.hxx>
#include <BRepIntCurveSurface_Inter.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>
#include <algorithm>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <QMatrix4x4>
#include <QQuaternion>
#include <QTimer>
#include <QVector4D>
#include <QElapsedTimer>
#include <QDebug>

#include "CadNode.h"
#include <QWidget>
#include <QObject>

// Forward declaration for screenToWorldRay
static void screenToWorldRay(const QPoint& pos, int width, int height, const QVector3D& cameraPos, const QQuaternion& cameraRot, float cameraZoom, gp_Pnt& rayOrigin, gp_Dir& rayDir);
QPoint CadOpenGLWidget_projectWorldToScreen(const QVector3D& world, int width, int height, const QVector3D& cameraPos, const QQuaternion& cameraRot, float cameraZoom);
static QVector3D unprojectScreenToWorld(const QPoint& screenPos, float depth, int width, int height, const QVector3D& cameraPos, const QQuaternion& cameraRot, float cameraZoom);
static void accumulateBoundingBox(const CadNode* node, Bnd_Box& bbox);

// Function to clear geometry cache
void CadOpenGLWidget::clearGeometryCache() {
    for (auto& [shapePtr, cache] : m_geometryCache) {
        if (cache.isValid()) {
            glDeleteLists(cache.displayList, 1);
        }
        cache.markInvalid();
    }
    m_geometryCache.clear();
    m_cacheDirty = true;
}

CadOpenGLWidget::CadOpenGLWidget(QWidget *parent)
    : QOpenGLWidget(parent), m_rotating(false), m_firstRotateMove(false) {
    
    // Set up continuous rendering timer for 60fps
    m_renderTimer.setInterval(16); // ~60fps (1000ms / 60fps ≈ 16.67ms)
    connect(&m_renderTimer, &QTimer::timeout, this, [this]() {
        if (m_windowActive) {
            // Update hover state continuously when in selection modes
            QPoint currentPos = mapFromGlobal(QCursor::pos());
            if (rect().contains(currentPos)) {
                if (m_selectionMode == SelectionMode::Faces) {
                    pickHoveredFaceAt(currentPos);
                } else if (m_selectionMode == SelectionMode::Edges) {
                    pickHoveredEdgeAt(currentPos);
                }
            }
            update(); // Trigger a render update
        }
    });
    m_renderTimer.start();
}

CadOpenGLWidget::~CadOpenGLWidget() {
    // Clean up color batches
    m_colorBatches.clear();
    
    // Clear geometry cache
    clearGeometryCache();
}

// Camera state
static QPoint g_lastMousePos;

void CadOpenGLWidget::setupOpenGLState() {
    initializeOpenGLFunctions();
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE); // Disable backface culling for CAD models (often have thin walls)
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_LIGHTING);
    glClearColor(0.1f, 0.1f, 0.15f, 1.0f);
}

void CadOpenGLWidget::initializeGL()
{
    setupOpenGLState();
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    double aspect = double(width()) / double(height() ? height() : 1);
    gluPerspective(45.0, aspect, 1.0, 1e9);
    glMatrixMode(GL_MODELVIEW);
    m_colorBatches.clear();
}

void CadOpenGLWidget::resizeGL(int w, int h) {
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    double aspect = double(w) / double(h ? h : 1);
    gluPerspective(45.0, aspect, 1.0, 1e9);
    glMatrixMode(GL_MODELVIEW);
}

// Optimized geometry caching function with balanced settings
CachedGeometry& CadOpenGLWidget::getOrCreateCachedGeometry(const TopoDS_Face& face) {
    m_totalFaces++;
    const void* shapePtr = face.TShape().get();
    auto it = m_geometryCache.find(shapePtr);
    if (it != m_geometryCache.end() && it->second.initialized) {
        return it->second;
    }
    
    CachedGeometry& cache = m_geometryCache[shapePtr];
    if (cache.initialized) {
        return cache;
    }
    
    // Create mesh for the face with adaptive precision based on face size
    Bnd_Box bbox;
    BRepBndLib::Add(face, bbox);
    Standard_Real xmin, ymin, zmin, xmax, ymax, zmax;
    bbox.Get(xmin, ymin, zmin, xmax, ymax, zmax);
    double faceSize = std::max(std::max(xmax-xmin, ymax-ymin), zmax-zmin);
    
    // Balanced LOD settings
    double basePrecision = std::max<double>(0.05, std::min<double>(1.0, faceSize * 0.2));
    
    // Adjust precision based on camera distance
    QVector3D faceCenter((xmin + xmax) * 0.5, (ymin + ymax) * 0.5, (zmin + zmax) * 0.5);
    QVector3D toCamera = faceCenter - camera_.pos;
    float distanceSquared = toCamera.x() * toCamera.x() + toCamera.y() * toCamera.y() + toCamera.z() * toCamera.z();
    float distanceFactor = std::min<float>(4.0, std::max<float>(0.2, distanceSquared / (camera_.zoom * camera_.zoom)));
    
    double precision = basePrecision * distanceFactor;
    
    // Skip only extremely small faces (be more permissive)
    if (faceSize < 0.0001) {
        m_skippedFaces++;
        qDebug() << "Skipping face due to extremely small size:" << faceSize << "at center" << faceCenter;
        cache.markInvalid();
        return cache;
    }

    // Use moderate precision reduction (less aggressive)
    precision *= 1.2; // 1.2x coarser for performance (was 1.5)
    
    BRepMesh_IncrementalMesh mesher(face, precision);
    TopLoc_Location locCopy;
    Handle(Poly_Triangulation) triangulation = BRep_Tool::Triangulation(face, locCopy);
    
    // Validate that we have a valid OpenGL context before creating display lists
    if (!QOpenGLContext::currentContext()) {
        qDebug() << "No OpenGL context available for face at center" << faceCenter;
        cache.markInvalid();
        return cache;
    }
    
    if (!triangulation.IsNull() && !triangulation->Triangles().IsEmpty()) {
        const Poly_Array1OfTriangle& triangles = triangulation->Triangles();
        
        // Get the number of nodes
        int nbNodes = triangulation->NbNodes();
        
        // Prepare vertex data
        std::vector<float> vertices;
        vertices.reserve(triangles.Size() * 9); // 3 vertices per triangle, 3 floats per vertex
        
        // Reasonable triangle limit
        int maxTriangles = 8000; // Balanced limit
        int triangleCount = 0;
        
        for (int i = triangles.Lower(); i <= triangles.Upper() && triangleCount < maxTriangles; ++i) {
            int n1, n2, n3;
            triangles(i).Get(n1, n2, n3);
            
            if (n1 >= 1 && n1 <= nbNodes &&
                n2 >= 1 && n2 <= nbNodes &&
                n3 >= 1 && n3 <= nbNodes) {
                
                gp_Pnt p1 = triangulation->Node(n1);
                gp_Pnt p2 = triangulation->Node(n2);
                gp_Pnt p3 = triangulation->Node(n3);
                
                // Add vertices
                vertices.push_back(static_cast<float>(p1.X()));
                vertices.push_back(static_cast<float>(p1.Y()));
                vertices.push_back(static_cast<float>(p1.Z()));
                
                vertices.push_back(static_cast<float>(p2.X()));
                vertices.push_back(static_cast<float>(p2.Y()));
                vertices.push_back(static_cast<float>(p2.Z()));
                
                vertices.push_back(static_cast<float>(p3.X()));
                vertices.push_back(static_cast<float>(p3.Y()));
                vertices.push_back(static_cast<float>(p3.Z()));
                
                triangleCount++;
            }
        }
        
        if (!vertices.empty()) {
            if (m_frameCount % 60 == 0) {
                qDebug() << "[Cache] Created geometry for face, vertices:" << vertices.size() << "triangles:" << triangleCount << "at center" << faceCenter;
            }
            // Create display list for legacy OpenGL
            cache.displayList = glGenLists(1);
            if (cache.displayList != 0) { // Check if display list creation succeeded
                glNewList(cache.displayList, GL_COMPILE);
                
                glBegin(GL_TRIANGLES);
                for (size_t i = 0; i < vertices.size(); i += 9) {
                    glVertex3f(vertices[i], vertices[i+1], vertices[i+2]);
                    glVertex3f(vertices[i+3], vertices[i+4], vertices[i+5]);
                    glVertex3f(vertices[i+6], vertices[i+7], vertices[i+8]);
                }
                glEnd();
                
                glEndList();
                cache.triangleCount = vertices.size() / 9;
                cache.initialized = true;
            } else {
                // Display list creation failed
                qDebug() << "Failed to create display list for face at center" << faceCenter;
                cache.markInvalid();
            }
        } else {
            // No vertices generated
            qDebug() << "No vertices generated for face at center" << faceCenter;
            cache.markInvalid();
        }
    } else {
        // Tessellation failed
        m_skippedFaces++;
        qDebug() << "Skipping face due to tessellation failure - triangulation null or empty at center" << faceCenter;
        cache.markInvalid();
    }
    
    return cache;
}

// Optimized batch rendering function
void CadOpenGLWidget::renderBatchedGeometry() {
    if (!rootNode_) {
        if (m_frameCount % 120 == 0) {
            qDebug() << "[Render] No root node available";
        }
        return;
    }
    
    if (m_frameCount % 120 == 0) {
        //qDebug() << "[Render] Rendering geometry, root children:" << rootNode_->children.size();
    }
    
    // Use the original traversal approach but with optimized face rendering
    for (const auto& child : rootNode_->children) {
        traverseAndRender(child.get(), CADNodeColor(1.0f, 1.0f, 1.0f, 1.0f), true);
    }
}

// Build color batches for efficient rendering
void CadOpenGLWidget::buildColorBatches() {
    m_colorBatches.clear();
    
    // With display lists, we don't need complex batching
    // Each face is cached individually and rendered efficiently
}

// Update traverseAndRender to use extremely optimized rendering
void CadOpenGLWidget::traverseAndRender(const CadNode* node, CADNodeColor inheritedColor, bool /*ancestorsVisible*/) {
    if (!node) return;
    if (node->visible) {
        CADNodeColor nodeColor = (node->color.a >= 0.0f) ? node->color : inheritedColor;
        
        // Only apply transformation if needed
        bool needsTransform = !node->loc.IsIdentity();
        if (needsTransform) {
            const TopLoc_Location& fullLoc = node->loc;
            const gp_Trsf& trsf = fullLoc.Transformation();
            const gp_Mat& mat = trsf.VectorialPart();
            const gp_XYZ& trans = trsf.TranslationPart();
            
            glMatrixMode(GL_MODELVIEW);
            glPushMatrix();
            double matrix[16] = {
                mat.Value(1,1), mat.Value(2,1), mat.Value(3,1), 0.0,
                mat.Value(1,2), mat.Value(2,2), mat.Value(3,2), 0.0,
                mat.Value(1,3), mat.Value(2,3), mat.Value(3,3), 0.0,
                trans.X(),     trans.Y(),     trans.Z(),     1.0
            };
            glMultMatrixd(matrix);
        }

        // Only render XCAF nodes as geometry
        if (node->type == CadNodeType::XCAF) {
            const XCAFNodeData* xData = node->asXCAF();
            if (xData) {
                switch (xData->type) {
                    case TopAbs_FACE:
                        if (!xData->shape.IsNull() && xData->type == TopAbs_FACE) {
                            if (isInFrustum(TopoDS::Face(xData->shape), node->loc)) {
                                renderFaceOptimized(node, nodeColor);
                            } else if (m_frameCount % 120 == 0) {
                                qDebug() << "[Render] Face culled by frustum:" << node;
                            }
                        } else if (m_frameCount % 120 == 0) {
                            qDebug() << "[Render] Face node has no face data:" << node;
                        }
                        break;
                    case TopAbs_EDGE:
                        // Skip edge rendering for performance - only render highlighted edges separately
                        break;
                    default:
                        break;
                }
            }
        }
        // Draw reference frame for the selected node during traversal, while transform is on stack
        if (node == selectedFrameNode_) {
            drawReferenceFrame(TopLoc_Location(), 50000.0f);
        }
        if (needsTransform) {
            glPopMatrix();
        }
    }
    // Always traverse children for visibility
    for (const auto& child : node->children) {
        if (child) {
            traverseAndRender(child.get(), node->color, true);
        }
    }
    if (node->type == CadNodeType::Physics) {
        const PhysicsNodeData* physData = node->asPhysics();
        if (physData && physData->convexHullGenerated && !physData->hulls.empty()) {
            renderConvexHulls(physData, node->loc);
        }
    }
}

// Optimized face rendering using cached geometry with wireframe overlay
void CadOpenGLWidget::renderFaceOptimized(const CadNode* node, const CADNodeColor& color) {
    if (!node) return;
    const XCAFNodeData* xData = node->asXCAF();
    if (!xData || !xData->hasFace()) return;
    const TopoDS_Face& face = TopoDS::Face(xData->shape);
    if (face.IsNull()) return;
    CachedGeometry& cache = getOrCreateCachedGeometry(face);
    if (!cache.isValid()) return;
    
    // Check if this face is being hovered over
    bool isHovered = false;
    if (m_selectionMode == SelectionMode::Faces && hoveredFaceNode_) {
        // Compare CadNode pointers to check if this is the hovered face
        if (xData == hoveredFaceNode_->asXCAF()) {
            isHovered = true;
        }
    }
    
    // Check if this face is selected
    bool isSelected = false;
    if (!selectedFaceNodes_.empty()) {
        // Check if this node is in the selected faces list
        auto it = std::find(selectedFaceNodes_.begin(), selectedFaceNodes_.end(), node);
        if (it != selectedFaceNodes_.end()) {
            isSelected = true;
                    if (m_frameCount % 60 == 0) {
            qDebug() << "[MultiSelect] Rendering selected face, total selected:" << selectedFaceNodes_.size() << "node:" << xData;
        }
        }
    }
    
    // Set color based on state
    if (isSelected) {
        // Selected face: bright yellow
        glColor4f(1.0f, 1.0f, 0.0f, 0.8f);
    } else if (isHovered) {
        // Hovered face: bright green
        glColor4f(0.0f, 1.0f, 0.0f, 0.7f);
    } else {
        // Normal face: use original color
        glColor4f(color.r, color.g, color.b, color.a);
    }
    
    // Render using display list (only if valid)
    glCallList(cache.displayList);
    
    // Add thin wireframe overlay for edge effect without dedicated edge rendering
    if (isSelected || isHovered) {
        // Highlighted faces get a more prominent wireframe
        glColor4f(0.0f, 0.0f, 0.0f, 0.8f); // Darker, more opaque wireframe
        glLineWidth(2.0f); // Thicker lines for highlighted faces
    } else {
        glColor4f(0.0f, 0.0f, 0.0f, 0.3f); // Semi-transparent black
        glLineWidth(1.0f);
    }
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glCallList(cache.displayList); // This is safe now since we checked displayList != 0 above
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL); // Reset to fill mode
    glLineWidth(1.0f); // Reset line width
}

void CadOpenGLWidget::paintGL() {
    QElapsedTimer paintTimer;
    paintTimer.start();


    // Frame skipping for very slow rendering
    m_frameCount++;
    if (m_skipFrames > 0) {
        m_skipFrames--;
        //return; // Skip this frame entirely
    }
        
    // Check if we need to skip frames due to slow performance
    static QElapsedTimer lastFrameTimer;
    static int consecutiveSlowFrames = 0;
    
    if (lastFrameTimer.isValid()) {
        qint64 frameTime = lastFrameTimer.nsecsElapsed();
        if (frameTime > 100000000) { // If frame took more than 100ms (10 FPS)
            consecutiveSlowFrames++;
            if (consecutiveSlowFrames > 3) {
                m_skipFrames = 2; // Skip next 2 frames
                consecutiveSlowFrames = 0;
            }
        } else {
            consecutiveSlowFrames = 0;
        }
    }
    lastFrameTimer.start();
    
    //Used for selection, should probably be moved to some kind of scene
    //Related tracking for things moving eventually

    QOpenGLFunctions* f = QOpenGLContext::currentContext()->functions();
    f->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    // New camera transform: apply zoom, then rotation, then translation
    QMatrix4x4 view;
    view.translate(0, 0, -camera_.zoom);
    view.rotate(camera_.rot.conjugated());
    view.translate(-camera_.pos);
    glMultMatrixf(view.constData());

    // Skip frustum updates for maximum performance
    static bool frustumInitialized = false;
    if (!frustumInitialized) {
        updateFrustumPlanes();
        frustumInitialized = true;
    }
    
    // Use optimized batch rendering
    QElapsedTimer renderTimer;
    renderTimer.start();
    
    // Only rebuild cache if dirty
    if (m_cacheDirty) {
        buildColorBatches();
        m_cacheDirty = false;
    }
    
    // Render the main CAD geometry
    renderBatchedGeometry();
        
    // Render highlighted edges only (performance optimized)
    if (m_selectionMode == SelectionMode::Edges && (hoveredEdgeNode_ || !selectedEdgeNodes_.empty())) {
        renderHighlightedEdges();
    }
    
    qint64 renderTime = renderTimer.nsecsElapsed();
    
    // Print render time every frame for real-time monitoring
    //qDebug() << "[Profile] Optimized geometry rendering took:" << renderTime / 1000000.0 << "ms";
    
    // Log cache statistics (very infrequently to minimize overhead)
    if (m_frameCount % 1200 == 0) { // Log every 20 seconds
        qDebug() << "[Cache] Geometry cache size:" << m_geometryCache.size() << "entries";
        qDebug() << "[Camera] Pos:" << camera_.pos << "Zoom:" << camera_.zoom << "Rot:" << camera_.rot;
    }
        
    // Keep camera controls working by not disabling pivot sphere
    // Draw pivot sphere if enabled
    if (camera_.showPivotSphere) {
        drawPivotSphere();
    }

    // Draw zoom contact sphere if enabled
    if (m_showZoomContactSphere) {
        glPushMatrix();
        glTranslatef(m_zoomContactPoint.x(), m_zoomContactPoint.y(), m_zoomContactPoint.z());
        glColor4f(0.2f, 0.8f, 1.0f, 0.7f);
        GLUquadric* quad = gluNewQuadric();
        gluSphere(quad, 150.0, 16, 8);
        gluDeleteQuadric(quad);
        glPopMatrix();
    }
    
    qint64 totalPaintTime = paintTimer.nsecsElapsed();
    //qDebug() << "[Profile] Total optimized paintGL time:" << totalPaintTime / 1000000.0 << "ms";

    // Draw the scene bounding box
    Bnd_Box bbox;
    accumulateBoundingBox(rootNode_, bbox);
    if (!bbox.IsVoid()) {
        Standard_Real xmin, ymin, zmin, xmax, ymax, zmax;
        bbox.Get(xmin, ymin, zmin, xmax, ymax, zmax);
        drawBoundingBox(QVector3D(xmin, ymin, zmin), QVector3D(xmax, ymax, zmax), QVector4D(1.0f, 0.2f, 0.2f, 0.4f));
    }

    // Draw a single bounding box around the entire selection (faces and edges)
    Bnd_Box selectionBox;
    for (CadNode* node : selectedFaceNodes_) {
        if (!node) continue;
        XCAFNodeData* xData = node->asXCAF();
        if (!xData || !xData->hasFace()) continue;
        TopoDS_Face face = TopoDS::Face(xData->getFace().Located(node->loc));
        BRepBndLib::Add(face, selectionBox);
    }
    for (CadNode* node : selectedEdgeNodes_) {
        if (!node) continue;
        XCAFNodeData* xData = node->asXCAF();
        if (!xData || !xData->hasEdge()) continue;
        TopoDS_Edge edge = TopoDS::Edge(xData->getEdge().Located(node->loc));
        BRepBndLib::Add(edge, selectionBox);
    }
    if (!selectionBox.IsVoid()) {
        Standard_Real sxmin, symin, szmin, sxmax, symax, szmax;
        selectionBox.Get(sxmin, symin, szmin, sxmax, symax, szmax);
        // Bright yellow for selection
        drawBoundingBox(QVector3D(sxmin, symin, szmin), QVector3D(sxmax, symax, szmax), QVector4D(1.0f, 1.0f, 0.0f, 0.8f));
    }
}

void CadOpenGLWidget::mousePressEvent(QMouseEvent* event) {
    g_lastMousePos = event->pos();
    
    if (event->button() == Qt::LeftButton) {
        // Store press position for click detection
        camera_.mousePressPos = event->pos();
        camera_.mousePressed = true;
        camera_.mouseDragged = false;
    } else if (event->button() == Qt::MiddleButton) {
        // Middle mouse button for selection based on current mode
        QVector3D hitPoint;
        CadNode* pickedNode = nullptr;
        
        if (m_selectionMode == SelectionMode::Faces) {
            if (pickElementAt(event->pos(), &hitPoint)) {
                pickedNode = selectedFaceNode_;
            }
        } else if (m_selectionMode == SelectionMode::Edges) {
            if (pickEdgeAt(event->pos(), &hitPoint)) {
                pickedNode = selectedEdgeNode_;
            }
        }
        
        // Handle multi-selection with Ctrl+click
        if (pickedNode) {
            if (event->modifiers() & Qt::ControlModifier) {
                // Ctrl+click: add/remove from selection
                if (m_selectionMode == SelectionMode::Faces) {
                    addToSelection(pickedNode);
                    // Don't emit signal for Ctrl+click to avoid tree view interference
                } else if (m_selectionMode == SelectionMode::Edges) {
                    addToSelection(pickedNode);
                    // Don't emit signal for Ctrl+click to avoid tree view interference
                }
            } else {
                // Regular click: clear selection and select single item
                clearSelection();
                XCAFNodeData* pickedData = pickedNode->asXCAF();
                if (pickedData) {
                    if (m_selectionMode == SelectionMode::Faces) {
                        selectedFaceNodes_.push_back(pickedNode);
                        selectedFaceNode_ = pickedNode;
                        selectedFace_ = TopoDS::Face(pickedData->getFace().Located(pickedNode->loc));
                        qDebug() << "[MultiSelect] Regular click selected face, total:" << selectedFaceNodes_.size();
                        emit facePicked(pickedNode); // Emit signal for tree view update
                    } else if (m_selectionMode == SelectionMode::Edges) {
                        selectedEdgeNodes_.push_back(pickedNode);
                        selectedEdgeNode_ = pickedNode;
                        selectedEdge_ = TopoDS::Edge(pickedData->getEdge().Located(pickedNode->loc));
                        qDebug() << "[MultiSelect] Regular click selected edge, total:" << selectedEdgeNodes_.size();
                        emit edgePicked(pickedNode); // Emit signal for tree view update
                    }
                    update();
                }
            }
        }
        // No selection for SelectionMode::None
    } else if (event->button() == Qt::RightButton) {
        QVector3D hitPoint;
        // Use a separate picking function that doesn't change selection
        if (pickElementAtForPivot(event->pos(), &hitPoint)) {
            // Only update the pivot position and show the sphere, do NOT move or recenter the camera
            camera_.pivotWorldPos = hitPoint;
            setPivotSphere(hitPoint);
        } else {
            qDebug() << "No hit for pivot point";
        }
        // Start rotation state
        m_rotating = true;
        m_firstRotateMove = true;
        m_cameraPosOnPress = camera_.pos;
        m_cameraRotOnPress = camera_.rot;
        m_cameraZoomOnPress = camera_.zoom;
        m_pivotScreenOnPress = CadOpenGLWidget_projectWorldToScreen(
            camera_.pivotWorldPos, width(), height(), camera_.pos, camera_.rot, camera_.zoom);
    }
    // Do NOT call pickElementAt(event->pos()) again here, as it may change selection and cause a jump
}

void CadOpenGLWidget::mouseMoveEvent(QMouseEvent* event) {
    int dx = event->x() - g_lastMousePos.x();
    int dy = event->y() - g_lastMousePos.y();
    QVector3D newCameraPos = camera_.pos;
    QQuaternion newCameraRot = camera_.rot;
    float newCameraZoom = camera_.zoom;
    bool cameraChanged = false;
    
    // Check if mouse has moved enough to be considered a drag
    if (camera_.mousePressed && !camera_.mouseDragged) {
        int dragDistance = (event->pos() - camera_.mousePressPos).manhattanLength();
        if (dragDistance > 3) { // 3 pixel threshold for drag detection
            camera_.mouseDragged = true;
        }
    }
    
    if (event->buttons() & Qt::LeftButton) {
        // Pan: move both camera and pivot in local X/Y
        float panSpeed = camera_.zoom * 0.002f;
        QVector3D right = camera_.rot.rotatedVector(QVector3D(1, 0, 0));
        QVector3D up = camera_.rot.rotatedVector(QVector3D(0, 1, 0));
        QVector3D pan = -right * dx * panSpeed + up * dy * panSpeed;
        newCameraPos += pan;
        camera_.pivotWorldPos += pan;
        cameraChanged = true;
    } else if (event->buttons() & Qt::RightButton) {
        // --- Precise fix: Keep pivot at same screen position during rotation using unprojection ---
        QPoint oldScreenPos;
        QVector3D cameraPosForOld;
        QQuaternion cameraRotForOld;
        float cameraZoomForOld;
        if (m_firstRotateMove) {
            // Use the stored values from mouse press for the first move
            oldScreenPos = m_pivotScreenOnPress;
            cameraPosForOld = camera_.pos;
            cameraRotForOld = camera_.rot;
            cameraZoomForOld = camera_.zoom;
            m_firstRotateMove = false;
        } else {
            oldScreenPos = CadOpenGLWidget_projectWorldToScreen(
                camera_.pivotWorldPos, width(), height(), camera_.pos, camera_.rot, camera_.zoom);
            cameraPosForOld = camera_.pos;
            cameraRotForOld = camera_.rot;
            cameraZoomForOld = camera_.zoom;
        }
        // 2. Compute pivot depth in camera space (distance from camera to pivot)
        QMatrix4x4 view;
        view.translate(0, 0, -cameraZoomForOld);
        view.rotate(cameraRotForOld.conjugated());
        view.translate(-cameraPosForOld);
        QVector3D camSpace = view.map(camera_.pivotWorldPos);
        float pivotDepth = -camSpace.z();
        // 3. Apply incremental rotation to camera position and orientation (orbit around pivot)
        float angleX = dy * 0.5f;
        float angleZ = dx * 0.5f;
        QQuaternion rotX = QQuaternion::fromAxisAndAngle(1, 0, 0, angleX);
        QQuaternion rotZ = QQuaternion::fromAxisAndAngle(0, 0, 1, angleZ);
        QQuaternion incrementalRot = rotZ * rotX;
        QVector3D camToPivot = cameraPosForOld - camera_.pivotWorldPos;
        QVector3D newCamToPivot = incrementalRot.rotatedVector(camToPivot);
        newCameraPos = camera_.pivotWorldPos + newCamToPivot;
        newCameraRot = incrementalRot * cameraRotForOld;
        // 4. Project pivot to screen after rotation
        QPoint newScreenPos = CadOpenGLWidget_projectWorldToScreen(
            camera_.pivotWorldPos, width(), height(), newCameraPos, newCameraRot, camera_.zoom);
        // 5. Unproject old and new screen positions at the pivot's depth
        QVector3D worldAtOldScreen = unprojectScreenToWorld(oldScreenPos, pivotDepth, width(), height(), newCameraPos, newCameraRot, camera_.zoom);
        QVector3D worldAtNewScreen = unprojectScreenToWorld(newScreenPos, pivotDepth, width(), height(), newCameraPos, newCameraRot, camera_.zoom);
        // 6. Compute offset
        QVector3D offset = worldAtOldScreen - worldAtNewScreen;
        // 7. Apply offset to camera position and pivot
        newCameraPos += offset;
        camera_.pivotWorldPos += offset;
        cameraChanged = true;
    }
    
    if (cameraChanged) {
        setCamera(newCameraPos, newCameraRot, camera_.zoom);
    }
    
    g_lastMousePos = event->pos();
    
    // Always do hover picking and update rendering on mouse movement
    // This ensures smooth hover highlighting even when no buttons are pressed
    if (m_selectionMode == SelectionMode::Faces) {
        pickHoveredFaceAt(event->pos());
        // Clear edge hover state
        hoveredEdge_.Nullify();
        hoveredEdgeNode_ = nullptr;
    } else if (m_selectionMode == SelectionMode::Edges) {
        pickHoveredEdgeAt(event->pos());
        // Clear face hover state
        hoveredFace_.Nullify();
        hoveredFaceNode_ = nullptr;
    } else {
        // Clear all hover states when not in selection mode
        hoveredFace_.Nullify();
        hoveredFaceNode_ = nullptr;
        hoveredEdge_.Nullify();
        hoveredEdgeNode_ = nullptr;
    }
    
    // Always trigger a render update on mouse movement for smooth hover feedback
    update();
}

// On mouse release, hide the pivot sphere
void CadOpenGLWidget::mouseReleaseEvent(QMouseEvent* event) {
    if (event->button() == Qt::LeftButton) {
        // Check if this was a click (not a drag)
        if (camera_.mousePressed && !camera_.mouseDragged) {
            // This was a click in place - handle selection
            QVector3D hitPoint;
            CadNode* pickedNode = nullptr;
            
            if (m_selectionMode == SelectionMode::Faces) {
                if (pickElementAt(event->pos(), &hitPoint)) {
                    pickedNode = selectedFaceNode_;
                }
            } else if (m_selectionMode == SelectionMode::Edges) {
                if (pickEdgeAt(event->pos(), &hitPoint)) {
                    pickedNode = selectedEdgeNode_;
                }
            }
            
            // Handle multi-selection with Ctrl+click
            bool isValid = pickedNode && pickedNode->asXCAF();
            if (isValid) {
                XCAFNodeData* pickedData = pickedNode->asXCAF();
                if (event->modifiers() & Qt::ControlModifier) {
                    // Ctrl+click: add/remove from selection
                    if (m_selectionMode == SelectionMode::Faces) {
                        addToSelection(pickedNode);
                        // Don't emit signal for Ctrl+click to avoid tree view interference
                    } else if (m_selectionMode == SelectionMode::Edges) {
                        addToSelection(pickedNode);
                        // Don't emit signal for Ctrl+click to avoid tree view interference
                    }
                } else {
                    // Regular click: clear selection and select single item
                    clearSelection();
                    if (m_selectionMode == SelectionMode::Faces) {
                        selectedFaceNodes_.push_back(pickedNode);
                        selectedFaceNode_ = pickedNode;
                        selectedFace_ = TopoDS::Face(pickedData->getFace().Located(pickedNode->loc));
                        qDebug() << "[MultiSelect] Left click selected face, total:" << selectedFaceNodes_.size();
                        emit facePicked(pickedNode); // Only emit signal for single selection
                    } else if (m_selectionMode == SelectionMode::Edges) {
                        selectedEdgeNodes_.push_back(pickedNode);
                        selectedEdgeNode_ = pickedNode;
                        selectedEdge_ = TopoDS::Edge(pickedData->getEdge().Located(pickedNode->loc));
                        qDebug() << "[MultiSelect] Left click selected edge, total:" << selectedEdgeNodes_.size();
                        emit edgePicked(pickedNode); // Only emit signal for single selection
                    }
                    update();
                }
            }
        }
        
        // Reset mouse state
        camera_.mousePressed = false;
        camera_.mouseDragged = false;
    } else if (event->button() == Qt::RightButton) {
        camera_.showPivotSphere = false;
        m_rotating = false;
        m_firstRotateMove = false;
        update();
    }
}

// Clear hover state when mouse leaves the widget
void CadOpenGLWidget::leaveEvent(QEvent* event) {
    hoveredFace_.Nullify();
    hoveredFaceNode_ = nullptr;
    hoveredEdge_.Nullify();
    hoveredEdgeNode_ = nullptr;
    update();
    QOpenGLWidget::leaveEvent(event);
}

// Handle window focus events to control continuous rendering
void CadOpenGLWidget::focusInEvent(QFocusEvent* event) {
    m_windowActive = true;
    QOpenGLWidget::focusInEvent(event);
}

void CadOpenGLWidget::focusOutEvent(QFocusEvent* event) {
    m_windowActive = false;
    QOpenGLWidget::focusOutEvent(event);
}

// Helper: Project a 3D point to screen coordinates using the current camera state
QPoint CadOpenGLWidget_projectWorldToScreen(const QVector3D& world, int width, int height, const QVector3D& cameraPos, const QQuaternion& cameraRot, float cameraZoom) {
    QMatrix4x4 view;
    view.translate(0, 0, -cameraZoom);
    view.rotate(cameraRot.conjugated());
    view.translate(-cameraPos);
    QVector3D camSpace = view.map(world);
    float fovY = 45.0f * M_PI / 180.0f;
    float aspect = float(width) / float(height ? height : 1);
    float nearPlane = 1.0f;
    float tanFovY = tan(fovY / 2.0f);
    float x = camSpace.x() / (-camSpace.z()) / (aspect * tanFovY / nearPlane);
    float y = camSpace.y() / (-camSpace.z()) / (tanFovY / nearPlane);
    int sx = int((x + 1) * 0.5f * width);
    int sy = int((1 - y) * 0.5f * height);
    return QPoint(sx, sy);
}

// Helper: Unproject a screen point at a given depth (in world coordinates)
static QVector3D unprojectScreenToWorld(const QPoint& screenPos, float depth, int width, int height, const QVector3D& cameraPos, const QQuaternion& cameraRot, float cameraZoom) {
    // Build view matrix (world to camera)
    QMatrix4x4 view;
    view.translate(0, 0, -cameraZoom);
    view.rotate(cameraRot.conjugated());
    view.translate(-cameraPos);
    QMatrix4x4 proj;
    float fovY = 45.0f * M_PI / 180.0f;
    float aspect = float(width) / float(height ? height : 1);
    float nearPlane = 1.0f;
    float farPlane = 1e9f;
    proj.perspective(45.0f, aspect, nearPlane, farPlane);
    QMatrix4x4 inv = (proj * view).inverted();
    // Normalized device coordinates
    float ndcX = (2.0f * screenPos.x()) / width - 1.0f;
    float ndcY = 1.0f - (2.0f * screenPos.y()) / height;
    float ndcZ = 2.0f * ((depth - nearPlane) / (farPlane - nearPlane)) - 1.0f; // Linear depth mapping
    QVector4D ndc(ndcX, ndcY, ndcZ, 1.0f);
    QVector4D world = inv * ndc;
    if (world.w() != 0.0f) world /= world.w();
    return world.toVector3D();
}

void CadOpenGLWidget::wheelEvent(QWheelEvent* event) {
    float zoomDelta = event->angleDelta().y() * 4.0f; // Reversed: removed the negative sign
    QPoint mousePos = event->position().toPoint();
    QVector3D intersection;
    
    if (pickElementAtForPivot(mousePos, &intersection)) {
        // Zoom to point: adjust camera position to keep intersection under cursor
        QVector3D viewDir = camera_.rot.rotatedVector(QVector3D(0, 0, -1));
        float newCameraZoom = camera_.zoom - zoomDelta;
        
        // Calculate how much the camera needs to move to keep the point under cursor
        QVector3D camToPoint = intersection - camera_.pos;
        float currentDist = camToPoint.length();
        float zoomRatio = newCameraZoom / camera_.zoom;
        float newDist = currentDist * zoomRatio;
        
        // Move camera position to maintain the point under cursor
        QVector3D newCameraPos = intersection - (camToPoint.normalized() * newDist);
        
        setCamera(newCameraPos, camera_.rot, newCameraZoom);
        
        // Show zoom contact sphere
        m_zoomContactPoint = intersection;
        m_showZoomContactSphere = true;
        m_zoomContactTimer.stop();
        m_zoomContactTimer.setSingleShot(true);
        connect(&m_zoomContactTimer, &QTimer::timeout, this, [this]() {
            m_showZoomContactSphere = false;
            update();
        });
        m_zoomContactTimer.start(1000); // 1 second
    } else {
        // Fallback: simple zoom (dolly)
        float newCameraZoom = camera_.zoom - zoomDelta;
        setCamera(camera_.pos, camera_.rot, newCameraZoom);
    }
}


void CadOpenGLWidget::setRootTreeNode(const CadNode* root) {
    rootNode_ = root;
    markCacheDirty();
    update();
}

void CadOpenGLWidget::clearCache() {
    clearGeometryCache();
    update();
}

int CadOpenGLWidget::getCacheSize() const {
    return static_cast<int>(m_geometryCache.size());
}

// Simple frustum culling implementation with extremely aggressive culling
void CadOpenGLWidget::updateFrustumPlanes() {
    // Get current view matrix
    QMatrix4x4 view;
    view.translate(0, 0, -camera_.zoom);
    view.rotate(camera_.rot.conjugated());
    view.translate(-camera_.pos);
    
    // Get projection matrix
    QMatrix4x4 proj;
    float fovY = 45.0f * M_PI / 180.0f;
    float aspect = float(width()) / float(height() ? height() : 1);
    proj.perspective(45.0f, aspect, 1.0f, 1e9f);
    
    // Combined matrix
    QMatrix4x4 combined = proj * view;
    
    // Extract frustum planes
    for (int i = 0; i < 4; ++i) {
        m_frustumPlanes[0][i] = combined(i,3) + combined(i,0); // Left
        m_frustumPlanes[1][i] = combined(i,3) - combined(i,0); // Right
        m_frustumPlanes[2][i] = combined(i,3) + combined(i,1); // Bottom
        m_frustumPlanes[3][i] = combined(i,3) - combined(i,1); // Top
        m_frustumPlanes[4][i] = combined(i,3) + combined(i,2); // Near
        m_frustumPlanes[5][i] = combined(i,3) - combined(i,2); // Far
    }
    
    // Normalize planes
    for (int i = 0; i < 6; ++i) {
        float length = QVector3D(m_frustumPlanes[i].x(), m_frustumPlanes[i].y(), m_frustumPlanes[i].z()).length();
        if (length > 0) {
            m_frustumPlanes[i] /= length;
        }
    }
}

bool CadOpenGLWidget::isInFrustum(const TopoDS_Face& face, const TopLoc_Location& loc) {
    // Balanced frustum culling - permissive but not overly so
    Bnd_Box bbox;
    if (!loc.IsIdentity()) {
        TopoDS_Face transformedFace = TopoDS::Face(face.Located(loc));
        BRepBndLib::Add(transformedFace, bbox);
    } else {
        BRepBndLib::Add(face, bbox);
    }
    
    if (bbox.IsVoid()) return true; // If we can't determine bounds, render it
    
    Standard_Real xmin, ymin, zmin, xmax, ymax, zmax;
    bbox.Get(xmin, ymin, zmin, xmax, ymax, zmax);
    
    // Moderate size-based culling (more permissive)
    double faceSize = std::max(std::max(xmax-xmin, ymax-ymin), zmax-zmin);
    if (faceSize < 0.00001) return false; // Skip only extremely small faces (was 0.0001)
    
    // Removed distance-based culling logic here
    
    return true;
}

void CadOpenGLWidget::renderFace(const XCAFNodeData* node, const CADNodeColor& color) {
    if (!node || !node->hasFace()) return;
    const TopoDS_Face& face = node->getFace();
    if (face.IsNull()) {
        qDebug() << "Null face data";
    }
    if (face.IsNull()) return;
    // No transform here! It is already applied in traverseAndRender or paintGL.
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
    int N = std::max<int>(20, static_cast<int>(curveLength * 10)); // At least 20 segments, more for longer curves
    
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

// Optimized edge rendering for highlighted edges only
void CadOpenGLWidget::renderEdgeOptimized(const CadNode* node, const CADNodeColor& color) {
    const XCAFNodeData* xData = node->asXCAF();
    if (!xData || !xData->hasEdge()) return;
    const TopoDS_Edge& edge = TopoDS::Edge(xData->shape);
    if (edge.IsNull()) return;
    
    // Apply node transformation if needed
    bool needsTransform = !node->loc.IsIdentity();
    if (needsTransform) {
        const gp_Trsf& trsf = node->loc.Transformation();
        const gp_Mat& mat = trsf.VectorialPart();
        const gp_XYZ& trans = trsf.TranslationPart();
        
        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        double matrix[16] = {
            mat.Value(1,1), mat.Value(2,1), mat.Value(3,1), 0.0,
            mat.Value(1,2), mat.Value(2,2), mat.Value(3,2), 0.0,
            mat.Value(1,3), mat.Value(2,3), mat.Value(3,3), 0.0,
            trans.X(),     trans.Y(),     trans.Z(),     1.0
        };
        glMultMatrixd(matrix);
    }
    
    // Set color and line width based on passed color (already determined by caller)
    glColor4f(color.r, color.g, color.b, color.a);
    if (color.r == 1.0f && color.g == 1.0f && color.b == 0.0f) {
        // Selected edge: bright yellow
        glLineWidth(4.0f);
    } else if (color.r == 0.0f && color.g == 1.0f && color.b == 0.0f) {
        // Hovered edge: bright green
        glLineWidth(3.0f);
    } else {
        // Fallback
        glLineWidth(2.0f);
    }
    
    // Render the edge
    Standard_Real first, last;
    TopLoc_Location locCopy;
    Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, locCopy, first, last);
    
    if (curve.IsNull()) {
        if (needsTransform) {
            glPopMatrix();
        }
        return;
    }
    
    // Determine number of segments based on curve length
    Standard_Real curveLength = last - first;
    int N = std::max<int>(10, static_cast<int>(curveLength * 5)); // Optimized for performance
    
    glBegin(GL_LINE_STRIP);
    for (int i = 0; i <= N; ++i) {
        Standard_Real t = first + (last - first) * i / N;
        gp_Pnt p;
        
        try {
            curve->D0(t, p);
            glVertex3d(p.X(), p.Y(), p.Z());
        } catch (const Standard_Failure&) {
            continue;
        }
    }
    glEnd();
    
    glLineWidth(1.0f); // Reset line width
    
    // Restore transformation if needed
    if (needsTransform) {
        glPopMatrix();
    }
}

// Render only highlighted edges for performance
void CadOpenGLWidget::renderHighlightedEdges() {
    // Render hovered edge
    if (hoveredEdgeNode_) {
        renderEdgeOptimized(hoveredEdgeNode_, CADNodeColor(0.0f, 1.0f, 0.0f, 0.8f));
    }
    
    // Render all selected edges
    for (CadNode* node : selectedEdgeNodes_) {
        if (node != hoveredEdgeNode_) { // Don't render twice if hovered and selected
            renderEdgeOptimized(node, CADNodeColor(1.0f, 1.0f, 0.0f, 0.9f));
        }
    }
}

// Helper: Unproject screen point to world ray, matching new camera system
static void screenToWorldRay(const QPoint& pos, int width, int height, const QVector3D& cameraPos, const QQuaternion& cameraRot, float cameraZoom, gp_Pnt& rayOrigin, gp_Dir& rayDir)
{
    // 1. Build the view matrix (camera-to-world)
    QMatrix4x4 view;
    view.translate(0, 0, -cameraZoom); // Zoom (dolly)
    view.rotate(cameraRot.conjugated());
    view.translate(-cameraPos);
    QMatrix4x4 invView = view.inverted();

    // 2. Compute normalized device coordinates (NDC)
    float ndcX = (2.0f * pos.x()) / width - 1.0f;
    float ndcY = 1.0f - (2.0f * pos.y()) / height;

    // 3. Project to near plane in camera space
    float fovY = 45.0f * M_PI / 180.0f;
    float aspect = float(width) / float(height ? height : 1);
    float nearPlane = 1.0f;
    float tanFovY = tan(fovY / 2.0f);
    float vx = ndcX * aspect * tanFovY * nearPlane;
    float vy = ndcY * tanFovY * nearPlane;
    float vz = -nearPlane;
    QVector3D rayCam(vx, vy, vz);

    // 4. Transform ray origin and direction to world space
    QVector3D camOriginWorld = invView.map(QVector3D(0, 0, 0));
    QVector3D rayWorld = invView.map(rayCam) - camOriginWorld;
    rayWorld.normalize();

    rayOrigin = gp_Pnt(camOriginWorld.x(), camOriginWorld.y(), camOriginWorld.z());
    rayDir = gp_Dir(rayWorld.x(), rayWorld.y(), rayWorld.z());
}

// Overload: pick and output intersection point (returns true if hit)
bool CadOpenGLWidget::pickElementAt(const QPoint& pos, QVector3D* outIntersection) {
    QElapsedTimer pickTimer;
    pickTimer.start();
    
    selectedFace_.Nullify();
    selectedEdge_.Nullify();
    if (!rootNode_) return false;
    
    QElapsedTimer cacheTimer;
    cacheTimer.start();
    qint64 cacheTime = cacheTimer.nsecsElapsed();
    if (m_frameCount % 60 == 0) {
        qDebug() << "[Profile] Face cache build took:" << cacheTime / 1000000.0 << "ms";
    }
    
    QElapsedTimer rayTimer;
    rayTimer.start();
    gp_Pnt rayOrigin;
    gp_Dir rayDir;
    screenToWorldRay(pos, width(), height(), camera_.pos, camera_.rot, camera_.zoom, rayOrigin, rayDir);
    gp_Lin ray(rayOrigin, rayDir);
    qint64 rayTime = rayTimer.nsecsElapsed();
    if (m_frameCount % 60 == 0) {
        qDebug() << "[Profile] Ray setup took:" << rayTime / 1000000.0 << "ms";
    }
    
    QElapsedTimer intersectTimer;
    intersectTimer.start();
    double minDist = 1e9;
    FaceInstance* pickedFace = nullptr;
    gp_Pnt pickedPoint;
    bool found = false;
    for (auto& inst : faceCache_) {
        XCAFNodeData* xData = inst.node->asXCAF();
        CadNode* node = inst.node;
        if (!node || !xData->hasFace()) continue;
        TopoDS_Face face = TopoDS::Face(xData->getFace().Located(node->loc));
        BRepIntCurveSurface_Inter intersector;
        intersector.Init(face, ray, 1e-6);
        while (intersector.More()) {
            gp_Pnt ipt = intersector.Pnt();
            double dist = rayOrigin.Distance(ipt);
            if (dist < minDist) {
                minDist = dist;
                pickedFace = &inst;
                pickedPoint = ipt;
                found = true;
            }
            intersector.Next();
        }
    }
    qint64 intersectTime = intersectTimer.nsecsElapsed();
    if (m_frameCount % 60 == 0) {
        qDebug() << "[Profile] Intersection testing took:" << intersectTime / 1000000.0 << "ms (tested" << faceCache_.size() << "faces)";
    }
    if (pickedFace && pickedFace->node) {
        CadNode* node = pickedFace->node;
        XCAFNodeData* xData = pickedFace->node->asXCAF();
        selectedFace_ = TopoDS::Face(xData->getFace().Located(node->loc));
        selectedFaceNode_ = node;
        if (m_frameCount % 60 == 0) {
            qDebug() << "[Ray] Selected face at TShape address:" << (void*)selectedFace_.TShape().get();
        }
        // Don't emit signal here - let the mouse press handler manage selection
        if (outIntersection && found) {
            *outIntersection = QVector3D(pickedPoint.X(), pickedPoint.Y(), pickedPoint.Z());
        }
    qint64 totalPickTime = pickTimer.nsecsElapsed();
    if (m_frameCount % 60 == 0) {
        qDebug() << "[Profile] Total pickElementAt time:" << totalPickTime / 1000000.0 << "ms";
    }
    update();
    return true;
    } else {
        selectedFaceNode_ = nullptr;
        if (m_frameCount % 60 == 0) {
            qDebug() << "No face selected (ray)";
        }
        qint64 totalPickTime = pickTimer.nsecsElapsed();
        if (m_frameCount % 60 == 0) {
            qDebug() << "[Profile] Total pickElementAt time (no hit):" << totalPickTime / 1000000.0 << "ms";
        }
        update();
        return false;
    }
} 

// Edge picking function
bool CadOpenGLWidget::pickEdgeAt(const QPoint& pos, QVector3D* outIntersection) {
    QElapsedTimer pickTimer;
    pickTimer.start();
    
    selectedEdge_.Nullify();
    selectedEdgeNode_ = nullptr;
    if (!rootNode_) return false;
    
    QElapsedTimer cacheTimer;
    cacheTimer.start();
    buildEdgeCache();
    qint64 cacheTime = cacheTimer.nsecsElapsed();
    if (m_frameCount % 60 == 0) {
        qDebug() << "[Profile] Edge cache build took:" << cacheTime / 1000000.0 << "ms";
    }
    
    QElapsedTimer rayTimer;
    rayTimer.start();
    gp_Pnt rayOrigin;
    gp_Dir rayDir;
    screenToWorldRay(pos, width(), height(), camera_.pos, camera_.rot, camera_.zoom, rayOrigin, rayDir);
    gp_Lin ray(rayOrigin, rayDir);
    qint64 rayTime = rayTimer.nsecsElapsed();
    if (m_frameCount % 60 == 0) {
        qDebug() << "[Profile] Ray setup took:" << rayTime / 1000000.0 << "ms";
    }
    
    QElapsedTimer intersectTimer;
    intersectTimer.start();
    double minDist = 1e9;
    EdgeInstance* pickedEdge = nullptr;
    gp_Pnt pickedPoint;
    bool found = false;
    for (auto& inst : edgeCache_) {
        CadNode* node = inst.node;
        XCAFNodeData* xData = inst.node->asXCAF();

        if (!xData || !xData->hasEdge()) continue;
        TopoDS_Edge edge = TopoDS::Edge(xData->getEdge().Located(node->loc));
        
        // Find closest point on edge to ray
        Standard_Real first, last;
        TopLoc_Location locCopy;
        Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, locCopy, first, last);
        if (curve.IsNull()) continue;
        
        // Sample points along the curve and find closest to ray
        int numSamples = 50;
        for (int i = 0; i <= numSamples; ++i) {
            Standard_Real t = first + (last - first) * i / numSamples;
            gp_Pnt curvePoint;
            try {
                curve->D0(t, curvePoint);
                // Apply the node's transformation to the curve point
                if (!node->loc.IsIdentity()) {
                    curvePoint.Transform(node->loc.Transformation());
                }
                double dist = ray.Distance(curvePoint);
                if (dist < minDist && dist < 10.0) { // Within 10 units of ray
                    minDist = dist;
                    pickedEdge = &inst;
                    pickedPoint = curvePoint;
                    found = true;
                }
            } catch (const Standard_Failure&) {
                continue;
            }
        }
    }
    qint64 intersectTime = intersectTimer.nsecsElapsed();
    if (m_frameCount % 60 == 0) {
        qDebug() << "[Profile] Edge intersection testing took:" << intersectTime / 1000000.0 << "ms (tested" << edgeCache_.size() << "edges)";
    }
    
            if (pickedEdge && pickedEdge->node) {
            CadNode* node = pickedEdge->node;
            XCAFNodeData* xData = node->asXCAF();
            selectedEdge_ = TopoDS::Edge(xData->getEdge().Located(node->loc));
            selectedEdgeNode_ = node;
            if (m_frameCount % 60 == 0) {
                qDebug() << "[Ray] Selected edge at TShape address:" << (void*)selectedEdge_.TShape().get();
            }
            // Don't emit signal here - let the mouse press handler manage selection
            if (outIntersection && found) {
                *outIntersection = QVector3D(pickedPoint.X(), pickedPoint.Y(), pickedPoint.Z());
            }
        qint64 totalPickTime = pickTimer.nsecsElapsed();
        if (m_frameCount % 60 == 0) {
            qDebug() << "[Profile] Total pickEdgeAt time:" << totalPickTime / 1000000.0 << "ms";
        }
        update();
        return true;
    } else {
        selectedEdgeNode_ = nullptr;
        if (m_frameCount % 60 == 0) {
            qDebug() << "No edge selected (ray)";
        }
        qint64 totalPickTime = pickTimer.nsecsElapsed();
        if (m_frameCount % 60 == 0) {
            qDebug() << "[Profile] Total pickEdgeAt time (no hit):" << totalPickTime / 1000000.0 << "ms";
        }
        update();
        return false;
    }
}

// Picking function for pivot setting that doesn't change selection
bool CadOpenGLWidget::pickElementAtForPivot(const QPoint& pos, QVector3D* outIntersection) {
    QElapsedTimer pickTimer;
    pickTimer.start();
    
    if (!rootNode_) return false;
    buildFaceCache();
    
    QElapsedTimer cacheTimer;
    cacheTimer.start();
    qint64 cacheTime = cacheTimer.nsecsElapsed();
    if (m_frameCount % 60 == 0) {
        qDebug() << "[Profile] Face cache build took:" << cacheTime / 1000000.0 << "ms";
    }
    
    QElapsedTimer rayTimer;
    rayTimer.start();
    gp_Pnt rayOrigin;
    gp_Dir rayDir;
    screenToWorldRay(pos, width(), height(), camera_.pos, camera_.rot, camera_.zoom, rayOrigin, rayDir);
    gp_Lin ray(rayOrigin, rayDir);
    qint64 rayTime = rayTimer.nsecsElapsed();
    if (m_frameCount % 60 == 0) {
        qDebug() << "[Profile] Ray setup took:" << rayTime / 1000000.0 << "ms";
    }
    
    QElapsedTimer intersectTimer;
    intersectTimer.start();
    double minDist = 1e9;
    FaceInstance* pickedFace = nullptr;
    gp_Pnt pickedPoint;
    bool found = false;
    for (auto& inst : faceCache_) {
        CadNode* node = inst.node;
        XCAFNodeData* xData = node->asXCAF();
        if (!xData || !xData->hasFace()) continue;
        TopoDS_Face face = TopoDS::Face(xData->getFace().Located(node->loc));
        BRepIntCurveSurface_Inter intersector;
        intersector.Init(face, ray, 1e-6);
        while (intersector.More()) {
            gp_Pnt ipt = intersector.Pnt();
            double dist = rayOrigin.Distance(ipt);
            if (dist < minDist) {
                minDist = dist;
                pickedFace = &inst;
                pickedPoint = ipt;
                found = true;
            }
            intersector.Next();
        }
    }
    qint64 intersectTime = intersectTimer.nsecsElapsed();
    if (m_frameCount % 60 == 0) {
        qDebug() << "[Profile] Intersection testing took:" << intersectTime / 1000000.0 << "ms (tested" << faceCache_.size() << "faces)";
    }
    
    if (found && outIntersection) {
        *outIntersection = QVector3D(pickedPoint.X(), pickedPoint.Y(), pickedPoint.Z());
        qint64 totalPickTime = pickTimer.nsecsElapsed();
        if (m_frameCount % 60 == 0) {
            qDebug() << "[Profile] Total pickElementAtForPivot time:" << totalPickTime / 1000000.0 << "ms";
        }
        return true;
    } else {
        qint64 totalPickTime = pickTimer.nsecsElapsed();
        if (m_frameCount % 60 == 0) {
            qDebug() << "[Profile] Total pickElementAtForPivot time (no hit):" << totalPickTime / 1000000.0 << "ms";
        }
        return false;
    }
}

void CadOpenGLWidget::pickHoveredFaceAt(const QPoint& pos) {
    // Keep hover picking working for camera controls
    QElapsedTimer hoverTimer;
    hoverTimer.start();
    
    hoveredFace_.Nullify();
    if (!rootNode_) return;
    gp_Pnt rayOrigin;
    gp_Dir rayDir;
    screenToWorldRay(pos, width(), height(), camera_.pos, camera_.rot, camera_.zoom, rayOrigin, rayDir);
    gp_Lin ray(rayOrigin, rayDir);
    double minDist = 1e9;
    FaceInstance* pickedFace = nullptr;
    for (auto& inst : faceCache_) {
        CadNode* node = inst.node;
        XCAFNodeData* xData = node->asXCAF();
        if (!node || !xData->hasFace()) continue;
        TopoDS_Face face = TopoDS::Face(xData->getFace().Located(node->loc));
        BRepIntCurveSurface_Inter intersector;
        intersector.Init(face, ray, 1e-6);
        while (intersector.More()) {
            gp_Pnt ipt = intersector.Pnt();
            double dist = rayOrigin.Distance(ipt);
            if (dist < minDist) {
                minDist = dist;
                pickedFace = &inst;
            }
            intersector.Next();
        }
    }
    if (pickedFace && pickedFace->node) {
        CadNode* node = pickedFace->node;
        XCAFNodeData* xData = node->asXCAF();
        hoveredFace_ = TopoDS::Face(xData->getFace().Located(node->loc));
        hoveredFaceNode_ = node; // Store the picked node
    }
    
    qint64 totalHoverTime = hoverTimer.nsecsElapsed();
    if (m_frameCount % 60 == 0) {
        qDebug() << "[Profile] pickHoveredFaceAt time:" << totalHoverTime / 1000000.0 << "ms";
    }
}

void CadOpenGLWidget::pickHoveredEdgeAt(const QPoint& pos) {
    QElapsedTimer hoverTimer;
    hoverTimer.start();
    
    hoveredEdge_.Nullify();
    hoveredEdgeNode_ = nullptr;
    if (!rootNode_) return;
    
    // Only build edge cache if not already built (performance optimization)
    static int lastEdgeCacheFrame = -1;
    if (lastEdgeCacheFrame != m_frameCount) {
        buildEdgeCache();
        lastEdgeCacheFrame = m_frameCount;
    }
    
    gp_Pnt rayOrigin;
    gp_Dir rayDir;
    screenToWorldRay(pos, width(), height(), camera_.pos, camera_.rot, camera_.zoom, rayOrigin, rayDir);
    gp_Lin ray(rayOrigin, rayDir);
    double minDist = 1e9;
    EdgeInstance* pickedEdge = nullptr;
    
    for (auto& inst : edgeCache_) {
        CadNode* node = inst.node;
        XCAFNodeData* xData = node->asXCAF();
        if (!xData || !xData->hasEdge()) continue;
        TopoDS_Edge edge = TopoDS::Edge(xData->getEdge().Located(node->loc));
        
        // Find closest point on edge to ray
        Standard_Real first, last;
        TopLoc_Location locCopy;
        Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, locCopy, first, last);
        if (curve.IsNull()) continue;
        
        // Sample points along the curve and find closest to ray (optimized for hover)
        int numSamples = 20; // Reduced samples for hover performance
        for (int i = 0; i <= numSamples; ++i) {
            Standard_Real t = first + (last - first) * i / numSamples;
            gp_Pnt curvePoint;
            try {
                curve->D0(t, curvePoint);
                // Apply the node's transformation to the curve point
                if (!node->loc.IsIdentity()) {
                    curvePoint.Transform(node->loc.Transformation());
                }
                double dist = ray.Distance(curvePoint);
                if (dist < minDist && dist < 15.0) { // Slightly larger threshold for hover
                    minDist = dist;
                    pickedEdge = &inst;
                }
            } catch (const Standard_Failure&) {
                continue;
            }
        }
    }
    
    if (pickedEdge && pickedEdge->node) {
        CadNode* node = pickedEdge->node;
        XCAFNodeData* xData = node->asXCAF();
        hoveredEdge_ = TopoDS::Edge(xData->getEdge().Located(node->loc));
        hoveredEdgeNode_ = node;
    }
    
    qint64 totalHoverTime = hoverTimer.nsecsElapsed();
    if (m_frameCount % 60 == 0) {
        qDebug() << "[Profile] pickHoveredEdgeAt time:" << totalHoverTime / 1000000.0 << "ms";
    }
} 

void CadOpenGLWidget::collectFaceNodes(CadNode* node, std::vector<CadNode*>& out) {
    if (!node) return;
    XCAFNodeData* xData = node->asXCAF();
    if (xData && xData->type == TopAbs_FACE && xData->hasFace()) {
        out.push_back(node);
    }
    for (auto& child : node->children) {
        collectFaceNodes(child.get(), out);
    }
}

void CadOpenGLWidget::buildFaceCache() {
    QElapsedTimer cacheTimer;
    cacheTimer.start();
    faceCache_.clear();
    if (!rootNode_) return;
    std::vector<CadNode*> faceNodes;
    for (auto& child : rootNode_->children) collectFaceNodes(child.get(), faceNodes);
    for (CadNode* n : faceNodes) faceCache_.push_back(FaceInstance{n});
    qint64 cacheTime = cacheTimer.nsecsElapsed();
    if (m_frameCount % 60 == 0) {
        qDebug() << "[Profile] buildFaceCache took:" << cacheTime / 1000000.0 << "ms (cached" << faceCache_.size() << "faces)";
    }
}

void CadOpenGLWidget::buildEdgeCache() {
    QElapsedTimer cacheTimer;
    cacheTimer.start();
    
    edgeCache_.clear();
    if (!rootNode_) return;
    
    // Optimize edge cache building by pre-allocating and using more efficient traversal
    std::function<void(CadNode*)> traverse;
    traverse = [&](CadNode* node) {
        if (!node || !node->visible) return;
        XCAFNodeData* xData = node->asXCAF();
        if (xData->type == TopAbs_EDGE && xData->hasEdge()) {
            edgeCache_.push_back(EdgeInstance{node});
        } else if (xData->type == TopAbs_SOLID || xData->type == TopAbs_SHELL || xData->type == TopAbs_COMPOUND) {
            // Only add parent nodes for picking if they have significant geometry
            TopoDS_Shape shape = xData->shape;
            if (!shape.IsNull()) {
                // Count edges to determine if this node is worth caching
                int edgeCount = 0;
                for (TopExp_Explorer exp(shape, TopAbs_EDGE); exp.More(); exp.Next()) {
                    edgeCount++;
                    if (edgeCount > 10) break; // Limit edge counting for performance
                }
                if (edgeCount > 0) {
                    edgeCache_.push_back(EdgeInstance{node});
                }
            }
        }
        for (auto& child : node->children) {
            traverse(child.get());
        }
    };
    for (auto& child : rootNode_->children) traverse(child.get());
    
    qint64 cacheTime = cacheTimer.nsecsElapsed();
    if (m_frameCount % 60 == 0) {
        qDebug() << "[Profile] buildEdgeCache took:" << cacheTime / 1000000.0 << "ms (cached" << edgeCache_.size() << "edges)";
    }
} 

// Add this slot for tree selection
void CadOpenGLWidget::setSelectionMode(SelectionMode mode) {
    m_selectionMode = mode;
    // Clear current selection when changing modes
    clearSelection();
    // Clear hover states when changing modes
    hoveredFace_.Nullify();
    hoveredFaceNode_ = nullptr;
    hoveredEdge_.Nullify();
    hoveredEdgeNode_ = nullptr;
    update();
}


void CadOpenGLWidget::clearSelection() {
    selectedFaceNodes_.clear();
    selectedEdgeNodes_.clear();
    selectedFaceNode_ = nullptr;
    selectedEdgeNode_ = nullptr;
    selectedFace_.Nullify();
    selectedEdge_.Nullify();
    selectedFrameNode_ = nullptr;
}

void CadOpenGLWidget::addToSelection(CadNode* node) {
    if (!node) return;

    XCAFNodeData* xData = node->asXCAF();
    if (!xData) return;

    if (xData->type == TopAbs_FACE && xData->hasFace()) {
        // Check if already selected
        auto it = std::find(selectedFaceNodes_.begin(), selectedFaceNodes_.end(), node);
        if (it != selectedFaceNodes_.end()) {
            // Remove from selection
            selectedFaceNodes_.erase(it);
            if (selectedFaceNode_ == node) {
                selectedFaceNode_ = selectedFaceNodes_.empty() ? nullptr : selectedFaceNodes_.back();
                selectedFace_ = selectedFaceNode_ ? TopoDS::Face(selectedFaceNode_->asXCAF()->getFace().Located(selectedFaceNode_->loc)) : TopoDS_Face();
            }
        } else {
            // Add to selection
            selectedFaceNodes_.push_back(node);
            selectedFaceNode_ = node;
            selectedFace_ = TopoDS::Face(xData->getFace().Located(node->loc));
        }
    } else if (xData->type == TopAbs_EDGE && xData->hasEdge()) {
        // Check if already selected
        auto it = std::find(selectedEdgeNodes_.begin(), selectedEdgeNodes_.end(), node);
        if (it != selectedEdgeNodes_.end()) {
            // Remove from selection
            selectedEdgeNodes_.erase(it);
            if (selectedEdgeNode_ == node) {
                selectedEdgeNode_ = selectedEdgeNodes_.empty() ? nullptr : selectedEdgeNodes_.back();
                selectedEdge_ = selectedEdgeNode_ ? TopoDS::Edge(selectedEdgeNode_->asXCAF()->getEdge().Located(selectedEdgeNode_->loc)) : TopoDS_Edge();
            }
        } else {
            // Add to selection
            selectedEdgeNodes_.push_back(node);
            selectedEdgeNode_ = node;
            selectedEdge_ = TopoDS::Edge(xData->getEdge().Located(node->loc));
        }
    }
    update();
}

void CadOpenGLWidget::removeFromSelection(CadNode* node) {
    if (!node) return;
    XCAFNodeData* xData = node->asXCAF();
    if (!xData) return;

    
    if (xData->type == TopAbs_FACE && xData->hasFace()) {
        auto it = std::find(selectedFaceNodes_.begin(), selectedFaceNodes_.end(), node);
        if (it != selectedFaceNodes_.end()) {
            selectedFaceNodes_.erase(it);
            if (selectedFaceNode_ == node) {
                selectedFaceNode_ = selectedFaceNodes_.empty() ? nullptr : selectedFaceNodes_.back();
                selectedFace_ = selectedFaceNode_ ? TopoDS::Face(selectedFaceNode_->asXCAF()->getFace().Located(selectedFaceNode_->loc)) : TopoDS_Face();
            }
        }
    } else if (xData->type == TopAbs_EDGE && xData->hasEdge()) {
        auto it = std::find(selectedEdgeNodes_.begin(), selectedEdgeNodes_.end(), node);
        if (it != selectedEdgeNodes_.end()) {
            selectedEdgeNodes_.erase(it);
            if (selectedEdgeNode_ == node) {
                selectedEdgeNode_ = selectedEdgeNodes_.empty() ? nullptr : selectedEdgeNodes_.back();
                selectedEdge_ = selectedEdgeNode_ ? TopoDS::Edge(selectedEdgeNode_->asXCAF()->getEdge().Located(selectedEdgeNode_->loc)) : TopoDS_Edge();
            }
        }
    }
    update();
} 

// Helper: Project a 3D point to screen coordinates
static QPoint worldToScreen(const QVector3D& world, int width, int height, float camDist, float camRotX, float camRotY, const QVector3D& panOffset, const QVector3D& center) {
    QMatrix4x4 mv;
    mv.translate(0, 0, -camDist);
    mv.rotate(camRotX, 1, 0, 0);
    mv.rotate(camRotY, 0, 1, 0);
    mv.translate(panOffset);
    mv.translate(-center);
    QVector3D camSpace = mv.map(world);
    float fovY = 45.0f * M_PI / 180.0f;
    float aspect = float(width) / float(height ? height : 1);
    float nearPlane = 1.0f;
    float tanFovY = tan(fovY / 2.0f);
    float x = camSpace.x() / (-camSpace.z()) / (aspect * tanFovY / nearPlane);
    float y = camSpace.y() / (-camSpace.z()) / (tanFovY / nearPlane);
    int sx = int((x + 1) * 0.5f * width);
    int sy = int((1 - y) * 0.5f * height);
    return QPoint(sx, sy);
}


void CadOpenGLWidget::drawPivotSphere() {
    glPushMatrix();
    glTranslatef(camera_.pivotWorldPos.x(), camera_.pivotWorldPos.y(), camera_.pivotWorldPos.z());
    glColor4f(1.0f, 0.2f, 0.2f, 0.8f); // Red, semi-transparent
    GLUquadric* quad = gluNewQuadric();
    gluSphere(quad, 150.0, 16, 8); // Increased radius from 5.0 to 15.0
    gluDeleteQuadric(quad);
    glPopMatrix();
} 

void CadOpenGLWidget::setCamera(const QVector3D& pos, const QQuaternion& rot, float zoom) {
    camera_.pos = pos;
    camera_.rot = rot;
    camera_.zoom = zoom;
    update();
} 

void CadOpenGLWidget::setPivotSphere(const QVector3D& worldPos) {
    camera_.pivotWorldPos = worldPos;
    camera_.showPivotSphere = true;
    update();
} 

// Helper to recursively accumulate bounding box for a node and its children
static void accumulateBoundingBox(const CadNode* node, Bnd_Box& bbox) {
    if (!node) return;
    const XCAFNodeData* xData = node->asXCAF();
    if (xData) {
        if (xData->type == TopAbs_FACE && xData->hasFace()) {
            TopoDS_Face locatedFace = TopoDS::Face(xData->getFace().Located(node->loc));
            BRepBndLib::Add(locatedFace, bbox);
        }
    }
    for (const auto& child : node->children) {
        if (child) accumulateBoundingBox(child.get(), bbox);
    }
}

void CadOpenGLWidget::reframeCamera() {
    if (!rootNode_) return;

    // 1. Compute bounding box
    Bnd_Box bbox;
    accumulateBoundingBox(rootNode_, bbox);
    if (bbox.IsVoid()) return;

    Standard_Real xmin, ymin, zmin, xmax, ymax, zmax;
    bbox.Get(xmin, ymin, zmin, xmax, ymax, zmax);
    QVector3D center((xmin + xmax) * 0.5, (ymin + ymax) * 0.5, (zmin + zmax) * 0.5);

    // 2. Get all 8 corners of the bounding box
    QVector<QVector3D> corners = {
        {float(xmin), float(ymin), float(zmin)},
        {float(xmin), float(ymin), float(zmax)},
        {float(xmin), float(ymax), float(zmin)},
        {float(xmin), float(ymax), float(zmax)},
        {float(xmax), float(ymin), float(zmin)},
        {float(xmax), float(ymin), float(zmax)},
        {float(xmax), float(ymax), float(zmin)},
        {float(xmax), float(ymax), float(zmax)}
    };

    // 3. Transform corners to camera local space (view space)
    QMatrix4x4 view;
    view.rotate(camera_.rot.conjugated());
    view.translate(-center); // Center the model

    float maxX = 0, maxY = 0;
    for (const auto& c : corners) {
        QVector3D v = view.map(c);
        maxX = std::max(maxX, std::abs(v.x()));
        maxY = std::max(maxY, std::abs(v.y()));
    }

    // 4. Compute required distance to fit the bounding box in view
    float fovY = 45.0f * M_PI / 180.0f;
    float aspect = float(width()) / float(height() ? height() : 1);
    float halfFovY = fovY / 2.0f;
    float halfFovX = atan(tan(halfFovY) * aspect);

    float distY = maxY / tan(halfFovY);
    float distX = maxX / tan(halfFovX);
    float requiredDist = std::max(distX, distY);
    requiredDist *= 1.1f; // Add a margin

    // 5. Set camera position along current view direction
    QVector3D viewDir = camera_.rot.rotatedVector(QVector3D(0, 0, -1));
    QVector3D newCamPos = center - viewDir * requiredDist;

    camera_.pos = newCamPos;
    camera_.zoom = requiredDist;
    camera_.pivotWorldPos = center;

    update();
}

CameraState CadOpenGLWidget::getCameraState() const {
    return camera_;
}
void CadOpenGLWidget::setCameraState(const CameraState& state) {
    camera_ = state;
    update();
}

void CadOpenGLWidget::markCacheDirty() {
    m_cacheDirty = true;
    update();
}

// Helper: Draw a wireframe bounding box given min/max coordinates
void CadOpenGLWidget::drawBoundingBox(const QVector3D& min, const QVector3D& max, const QVector4D& color) {
    glPushAttrib(GL_ENABLE_BIT | GL_LINE_BIT | GL_COLOR_BUFFER_BIT);
    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glColor4f(color.x(), color.y(), color.z(), color.w());
    glLineWidth(2.0f);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    glBegin(GL_LINES);
    // 8 corners
    QVector3D c[8] = {
        {min.x(), min.y(), min.z()},
        {min.x(), min.y(), max.z()},
        {min.x(), max.y(), min.z()},
        {min.x(), max.y(), max.z()},
        {max.x(), min.y(), min.z()},
        {max.x(), min.y(), max.z()},
        {max.x(), max.y(), min.z()},
        {max.x(), max.y(), max.z()}
    };
    // 12 edges
    int edges[12][2] = {
        {0,1},{0,2},{0,4},
        {1,3},{1,5},
        {2,3},{2,6},
        {3,7},
        {4,5},{4,6},
        {5,7},
        {6,7}
    };
    for (int i = 0; i < 12; ++i) {
        glVertex3f(c[edges[i][0]].x(), c[edges[i][0]].y(), c[edges[i][0]].z());
        glVertex3f(c[edges[i][1]].x(), c[edges[i][1]].y(), c[edges[i][1]].z());
    }
    glEnd();
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glLineWidth(1.0f);
    glPopAttrib();
}

// Helper: Draw a reference frame (axes) at a given transformation
void CadOpenGLWidget::drawReferenceFrame(const TopLoc_Location& loc, float axisLength) {
    const gp_Trsf& trsf = loc.Transformation();
    const gp_Mat& mat = trsf.VectorialPart();
    const gp_XYZ& trans = trsf.TranslationPart();
    glPushMatrix();
    double matrix[16] = {
        mat.Value(1,1), mat.Value(2,1), mat.Value(3,1), 0.0,
        mat.Value(1,2), mat.Value(2,2), mat.Value(3,2), 0.0,
        mat.Value(1,3), mat.Value(2,3), mat.Value(3,3), 0.0,
        trans.X(),     trans.Y(),     trans.Z(),     1.0
    };
    glMultMatrixd(matrix);
    glLineWidth(20.0f); // Thicker than bounding box
    glBegin(GL_LINES);
    // X axis (red)
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(axisLength, 0.0f, 0.0f);
    // Y axis (green)
    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, axisLength, 0.0f);
    // Z axis (blue)
    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, axisLength);
    glEnd();
    glLineWidth(1.0f);
    glPopMatrix();
}

// Add this function near other rendering helpers
void CadOpenGLWidget::renderConvexHulls(const PhysicsNodeData* physData, const TopLoc_Location& loc) {
    if (!physData) return;
    glPushMatrix();
    // Apply transformation if needed
    if (!loc.IsIdentity()) {
        const gp_Trsf& trsf = loc.Transformation();
        const gp_Mat& mat = trsf.VectorialPart();
        const gp_XYZ& trans = trsf.TranslationPart();
        double matrix[16] = {
            mat.Value(1,1), mat.Value(2,1), mat.Value(3,1), 0.0,
            mat.Value(1,2), mat.Value(2,2), mat.Value(3,2), 0.0,
            mat.Value(1,3), mat.Value(2,3), mat.Value(3,3), 0.0,
            trans.X(),     trans.Y(),     trans.Z(),     1.0
        };
        glMultMatrixd(matrix);
    }
    for (const auto& hull : physData->hulls) {
        // Draw filled hull
        glColor4f(1.0f, 0.5f, 0.1f, 0.3f); // Orange, semi-transparent
        glBegin(GL_TRIANGLES);
        for (const auto& tri : hull.indices) {
            for (int i = 0; i < 3; ++i) {
                const auto& v = hull.vertices[tri[i]];
                glVertex3f(v[0], v[1], v[2]);
            }
        }
        glEnd();
        // Draw wireframe
        glColor4f(0.0f, 0.0f, 0.0f, 0.7f);
        glLineWidth(2.0f);
        glBegin(GL_LINES);
        for (const auto& tri : hull.indices) {
            for (int i = 0; i < 3; ++i) {
                const auto& v1 = hull.vertices[tri[i]];
                const auto& v2 = hull.vertices[tri[(i+1)%3]];
                glVertex3f(v1[0], v1[1], v1[2]);
                glVertex3f(v2[0], v2[1], v2[2]);
            }
        }
        glEnd();
    }
    glPopMatrix();
}



