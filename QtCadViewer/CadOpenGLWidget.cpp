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

// Global geometry cache using TShape pointer as key
static std::map<const void*, CachedGeometry> g_geometryCache;
static bool g_cacheDirty = true;
static int g_frameCount = 0; // Add frame counter for performance monitoring
static bool g_performanceMode = false; // Force performance mode immediately
static int g_skipFrames = 0; // Frame skipping counter

// Function to clear geometry cache
void clearGeometryCache() {
    for (auto& [shapePtr, cache] : g_geometryCache) {
        if (cache.initialized && cache.displayList != 0) {
            glDeleteLists(cache.displayList, 1);
            cache.initialized = false;
        }
    }
    g_geometryCache.clear();
    g_cacheDirty = true;
}

CadOpenGLWidget::CadOpenGLWidget(QWidget *parent)
    : QOpenGLWidget(parent), m_rotating(false), m_firstRotateMove(false) {}

CadOpenGLWidget::~CadOpenGLWidget() {
    // Clean up color batches
    m_colorBatches.clear();
    
    // Clear geometry cache
    clearGeometryCache();
}

// Camera state
static QPoint g_lastMousePos;

void CadOpenGLWidget::initializeGL()
{
    initializeOpenGLFunctions();
    
    // Set up OpenGL state once
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE); // Disable backface culling for CAD models (often have thin walls)
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_LIGHTING);
    glClearColor(0.1f, 0.1f, 0.15f, 1.0f);
    
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    double aspect = double(width()) / double(height() ? height() : 1);
    gluPerspective(45.0, aspect, 1.0, 1e9);
    glMatrixMode(GL_MODELVIEW);
    
    // Initialize color batches
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
    const void* shapePtr = face.TShape().get();
    auto it = g_geometryCache.find(shapePtr);
    if (it != g_geometryCache.end() && it->second.initialized) {
        return it->second;
    }
    
    CachedGeometry& cache = g_geometryCache[shapePtr];
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
    QVector3D toCamera = faceCenter - m_cameraPos;
    float distanceSquared = toCamera.x() * toCamera.x() + toCamera.y() * toCamera.y() + toCamera.z() * toCamera.z();
    float distanceFactor = std::min<float>(4.0, std::max<float>(0.2, distanceSquared / (m_cameraZoom * m_cameraZoom)));
    
    double precision = basePrecision * distanceFactor;
    
    // Skip only very small faces
    if (faceSize < 0.001) {
        cache.initialized = true;
        cache.triangleCount = 0;
        return cache;
    }
    
    // Moderate distance culling
    if (distanceSquared > m_cameraZoom * m_cameraZoom * 200.0f) {
        cache.initialized = true;
        cache.triangleCount = 0;
        return cache;
    }
    
    // Use moderate precision reduction
    precision *= 1.5; // 1.5x coarser for performance
    
    BRepMesh_IncrementalMesh mesher(face, precision);
    TopLoc_Location locCopy;
    Handle(Poly_Triangulation) triangulation = BRep_Tool::Triangulation(face, locCopy);
    
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
            // Create display list for legacy OpenGL
            cache.displayList = glGenLists(1);
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
        }
    }
    
    return cache;
}

// Optimized batch rendering function
void CadOpenGLWidget::renderBatchedGeometry() {
    if (!rootNode_) return;
    
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
void CadOpenGLWidget::traverseAndRender(TreeNode* node, CADNodeColor inheritedColor, bool /*ancestorsVisible*/) {
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

        switch (node->type) {
            case TopAbs_FACE:
                if (node->hasFace()) {
                    // Use basic frustum culling for better performance
                    if (isInFrustum(node->getFace(), node->loc)) {
                        renderFaceOptimized(node, nodeColor);
                    }
                }
                break;
            case TopAbs_EDGE:
                // Skip dedicated edge rendering for performance
                break;
            default:
                break;
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
}

// Optimized face rendering using cached geometry with wireframe overlay
void CadOpenGLWidget::renderFaceOptimized(TreeNode* node, const CADNodeColor& color) {
    if (!node || !node->hasFace()) return;
    
    const TopoDS_Face& face = node->getFace();
    if (face.IsNull()) return;
    
    CachedGeometry& cache = getOrCreateCachedGeometry(face);
    if (!cache.initialized) return;
    
    // Set color once
    glColor4f(color.r, color.g, color.b, color.a);
    
    // Render using display list
    glCallList(cache.displayList);
    
    // Add thin wireframe overlay for edge effect without dedicated edge rendering
    glColor4f(0.0f, 0.0f, 0.0f, 0.3f); // Semi-transparent black
    glLineWidth(1.0f);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glCallList(cache.displayList);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL); // Reset to fill mode
}

void CadOpenGLWidget::paintGL() {
    QElapsedTimer paintTimer;
    paintTimer.start();
    
    // Frame skipping for very slow rendering
    g_frameCount++;
    if (g_skipFrames > 0) {
        g_skipFrames--;
        return; // Skip this frame entirely
    }
    
    // Check if we need to skip frames due to slow performance
    static QElapsedTimer lastFrameTimer;
    static int consecutiveSlowFrames = 0;
    
    if (lastFrameTimer.isValid()) {
        qint64 frameTime = lastFrameTimer.nsecsElapsed();
        if (frameTime > 100000000) { // If frame took more than 100ms (10 FPS)
            consecutiveSlowFrames++;
            if (consecutiveSlowFrames > 3) {
                g_skipFrames = 2; // Skip next 2 frames
                consecutiveSlowFrames = 0;
            }
        } else {
            consecutiveSlowFrames = 0;
        }
    }
    lastFrameTimer.start();
    
    QOpenGLFunctions* f = QOpenGLContext::currentContext()->functions();
    f->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    // New camera transform: apply zoom, then rotation, then translation
    QMatrix4x4 view;
    view.translate(0, 0, -m_cameraZoom);
    view.rotate(m_cameraRot.conjugated());
    view.translate(-m_cameraPos);
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
    if (g_cacheDirty) {
        buildColorBatches();
        g_cacheDirty = false;
    }
    
    renderBatchedGeometry();
    
    qint64 renderTime = renderTimer.nsecsElapsed();
    
    // Print render time every frame for real-time monitoring
    qDebug() << "[Profile] Optimized geometry rendering took:" << renderTime / 1000000.0 << "ms";
    
    // Log cache statistics (very infrequently to minimize overhead)
    if (g_frameCount % 1200 == 0) { // Log every 20 seconds
        qDebug() << "[Cache] Geometry cache size:" << g_geometryCache.size() << "entries";
    }
    
    // Keep camera controls working by not disabling pivot sphere
    // Draw pivot sphere if enabled
    if (m_showPivotSphere) {
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
    qDebug() << "[Profile] Total optimized paintGL time:" << totalPaintTime / 1000000.0 << "ms";
}

void CadOpenGLWidget::mousePressEvent(QMouseEvent* event) {
    g_lastMousePos = event->pos();
    if (event->button() == Qt::RightButton) {
        QVector3D hitPoint;
        if (pickElementAt(event->pos(), &hitPoint)) {
            // Only update the pivot position and show the sphere, do NOT move or recenter the camera
            m_pivotWorldPos = hitPoint;
            setPivotSphere(hitPoint);
        }
        // Start rotation state
        m_rotating = true;
        m_firstRotateMove = true;
        m_cameraPosOnPress = m_cameraPos;
        m_cameraRotOnPress = m_cameraRot;
        m_cameraZoomOnPress = m_cameraZoom;
        m_pivotScreenOnPress = CadOpenGLWidget_projectWorldToScreen(
            m_pivotWorldPos, width(), height(), m_cameraPos, m_cameraRot, m_cameraZoom);
    }
    // Do NOT call pickElementAt(event->pos()) again here, as it may change selection and cause a jump
}

void CadOpenGLWidget::mouseMoveEvent(QMouseEvent* event) {
    int dx = event->x() - g_lastMousePos.x();
    int dy = event->y() - g_lastMousePos.y();
    QVector3D newCameraPos = m_cameraPos;
    QQuaternion newCameraRot = m_cameraRot;
    float newCameraZoom = m_cameraZoom;
    if (event->buttons() & Qt::LeftButton) {
        // Pan: move both camera and pivot in local X/Y
        float panSpeed = m_cameraZoom * 0.002f;
        QVector3D right = m_cameraRot.rotatedVector(QVector3D(1, 0, 0));
        QVector3D up = m_cameraRot.rotatedVector(QVector3D(0, 1, 0));
        QVector3D pan = -right * dx * panSpeed + up * dy * panSpeed;
        newCameraPos += pan;
        m_pivotWorldPos += pan;
    } else if (event->buttons() & Qt::RightButton) {
        // --- Precise fix: Keep pivot at same screen position during rotation using unprojection ---
        QPoint oldScreenPos;
        QVector3D cameraPosForOld;
        QQuaternion cameraRotForOld;
        float cameraZoomForOld;
        if (m_firstRotateMove) {
            // Use the stored values from mouse press for the first move
            oldScreenPos = m_pivotScreenOnPress;
            cameraPosForOld = m_cameraPosOnPress;
            cameraRotForOld = m_cameraRotOnPress;
            cameraZoomForOld = m_cameraZoomOnPress;
            m_firstRotateMove = false;
        } else {
            oldScreenPos = CadOpenGLWidget_projectWorldToScreen(
                m_pivotWorldPos, width(), height(), m_cameraPos, m_cameraRot, m_cameraZoom);
            cameraPosForOld = m_cameraPos;
            cameraRotForOld = m_cameraRot;
            cameraZoomForOld = m_cameraZoom;
        }
        // 2. Compute pivot depth in camera space (distance from camera to pivot)
        QMatrix4x4 view;
        view.translate(0, 0, -cameraZoomForOld);
        view.rotate(cameraRotForOld.conjugated());
        view.translate(-cameraPosForOld);
        QVector3D camSpace = view.map(m_pivotWorldPos);
        float pivotDepth = -camSpace.z();
        // 3. Apply incremental rotation to camera position and orientation (orbit around pivot)
        float angleX = dy * 0.5f;
        float angleZ = dx * 0.5f;
        QQuaternion rotX = QQuaternion::fromAxisAndAngle(1, 0, 0, angleX);
        QQuaternion rotZ = QQuaternion::fromAxisAndAngle(0, 0, 1, angleZ);
        QQuaternion incrementalRot = rotZ * rotX;
        QVector3D camToPivot = cameraPosForOld - m_pivotWorldPos;
        QVector3D newCamToPivot = incrementalRot.rotatedVector(camToPivot);
        newCameraPos = m_pivotWorldPos + newCamToPivot;
        newCameraRot = incrementalRot * cameraRotForOld;
        // 4. Project pivot to screen after rotation
        QPoint newScreenPos = CadOpenGLWidget_projectWorldToScreen(
            m_pivotWorldPos, width(), height(), newCameraPos, newCameraRot, m_cameraZoom);
        // 5. Unproject old and new screen positions at the pivot's depth
        QVector3D worldAtOldScreen = unprojectScreenToWorld(oldScreenPos, pivotDepth, width(), height(), newCameraPos, newCameraRot, m_cameraZoom);
        QVector3D worldAtNewScreen = unprojectScreenToWorld(newScreenPos, pivotDepth, width(), height(), newCameraPos, newCameraRot, m_cameraZoom);
        // 6. Compute offset
        QVector3D offset = worldAtOldScreen - worldAtNewScreen;
        // 7. Apply offset to camera position and pivot
        newCameraPos += offset;
        m_pivotWorldPos += offset;
    }
    setCamera(newCameraPos, newCameraRot, m_cameraZoom);
    g_lastMousePos = event->pos();
    pickHoveredFaceAt(event->pos());
    update();
}

// On mouse release, hide the pivot sphere
void CadOpenGLWidget::mouseReleaseEvent(QMouseEvent* event) {
    if (event->button() == Qt::RightButton) {
        m_showPivotSphere = false;
        m_rotating = false;
        m_firstRotateMove = false;
        update();
    }
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
    
    if (pickElementAt(mousePos, &intersection)) {
        // Zoom to point: adjust camera position to keep intersection under cursor
        QVector3D viewDir = m_cameraRot.rotatedVector(QVector3D(0, 0, -1));
        float newCameraZoom = m_cameraZoom - zoomDelta;
        
        // Calculate how much the camera needs to move to keep the point under cursor
        QVector3D camToPoint = intersection - m_cameraPos;
        float currentDist = camToPoint.length();
        float zoomRatio = newCameraZoom / m_cameraZoom;
        float newDist = currentDist * zoomRatio;
        
        // Move camera position to maintain the point under cursor
        QVector3D newCameraPos = intersection - (camToPoint.normalized() * newDist);
        
        setCamera(newCameraPos, m_cameraRot, newCameraZoom);
        
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
        float newCameraZoom = m_cameraZoom - zoomDelta;
        setCamera(m_cameraPos, m_cameraRot, newCameraZoom);
    }
}


void CadOpenGLWidget::setRootTreeNode(TreeNode* root) {
    rootNode_ = root;
    g_cacheDirty = true; // Mark cache as dirty when root changes
    update();
}

void CadOpenGLWidget::clearCache() {
    clearGeometryCache();
    update();
}

int CadOpenGLWidget::getCacheSize() const {
    return static_cast<int>(g_geometryCache.size());
}

// Simple frustum culling implementation with extremely aggressive culling
void CadOpenGLWidget::updateFrustumPlanes() {
    // Get current view matrix
    QMatrix4x4 view;
    view.translate(0, 0, -m_cameraZoom);
    view.rotate(m_cameraRot.conjugated());
    view.translate(-m_cameraPos);
    
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
    
    // Moderate size-based culling
    double faceSize = std::max(std::max(xmax-xmin, ymax-ymin), zmax-zmin);
    if (faceSize < 0.0001) return false; // Skip very small faces
    
    // Moderate distance-based culling
    QVector3D faceCenter((xmin + xmax) * 0.5, (ymin + ymax) * 0.5, (zmin + zmax) * 0.5);
    QVector3D toCamera = faceCenter - m_cameraPos;
    float distanceSquared = toCamera.x() * toCamera.x() + toCamera.y() * toCamera.y() + toCamera.z() * toCamera.z();
    
    // Use reasonable distance threshold
    float maxDistanceSquared = m_cameraZoom * m_cameraZoom * 100.0f; // 100x zoom squared
    
    // For close viewing, be more permissive
    if (m_cameraZoom < 100.0f) {
        maxDistanceSquared = std::max(maxDistanceSquared, 100000.0f); // 100k units squared for close viewing
    }
    
    if (distanceSquared > maxDistanceSquared) {
        return false;
    }
    
    return true;
}

// Overload: Render a face using a TreeNode (applies node's transform)
void CadOpenGLWidget::renderFace(TreeNode* node, const CADNodeColor& color) {
    if (!node || !node->hasFace()) return;
    const TopoDS_Face& face = node->getFace();
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
    buildFaceCache();
    qint64 cacheTime = cacheTimer.nsecsElapsed();
    if (g_frameCount % 60 == 0) {
        qDebug() << "[Profile] Face cache build took:" << cacheTime / 1000000.0 << "ms";
    }
    
    QElapsedTimer rayTimer;
    rayTimer.start();
    gp_Pnt rayOrigin;
    gp_Dir rayDir;
    screenToWorldRay(pos, width(), height(), m_cameraPos, m_cameraRot, m_cameraZoom, rayOrigin, rayDir);
    gp_Lin ray(rayOrigin, rayDir);
    qint64 rayTime = rayTimer.nsecsElapsed();
    if (g_frameCount % 60 == 0) {
        qDebug() << "[Profile] Ray setup took:" << rayTime / 1000000.0 << "ms";
    }
    
    QElapsedTimer intersectTimer;
    intersectTimer.start();
    double minDist = 1e9;
    FaceInstance* pickedFace = nullptr;
    gp_Pnt pickedPoint;
    bool found = false;
    for (auto& inst : faceCache_) {
        TreeNode* node = inst.node;
        if (!node || !node->hasFace()) continue;
        TopoDS_Face face = TopoDS::Face(node->getFace().Located(node->loc));
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
    if (g_frameCount % 60 == 0) {
        qDebug() << "[Profile] Intersection testing took:" << intersectTime / 1000000.0 << "ms (tested" << faceCache_.size() << "faces)";
    }
    if (pickedFace && pickedFace->node) {
        TreeNode* node = pickedFace->node;
        selectedFace_ = TopoDS::Face(node->getFace().Located(node->loc));
        selectedFaceNode_ = node;
        if (g_frameCount % 60 == 0) {
            qDebug() << "[Ray] Selected face at TShape address:" << (void*)selectedFace_.TShape().get();
        }
        emit facePicked(node); // Emit signal on click
        if (outIntersection && found) {
            *outIntersection = QVector3D(pickedPoint.X(), pickedPoint.Y(), pickedPoint.Z());
        }
        qint64 totalPickTime = pickTimer.nsecsElapsed();
        if (g_frameCount % 60 == 0) {
            qDebug() << "[Profile] Total pickElementAt time:" << totalPickTime / 1000000.0 << "ms";
        }
        update();
        return true;
    } else {
        selectedFaceNode_ = nullptr;
        if (g_frameCount % 60 == 0) {
            qDebug() << "No face selected (ray)";
        }
        qint64 totalPickTime = pickTimer.nsecsElapsed();
        if (g_frameCount % 60 == 0) {
            qDebug() << "[Profile] Total pickElementAt time (no hit):" << totalPickTime / 1000000.0 << "ms";
        }
        update();
        return false;
    }
} 

void CadOpenGLWidget::pickHoveredFaceAt(const QPoint& pos) {
    // Keep hover picking working for camera controls
    QElapsedTimer hoverTimer;
    hoverTimer.start();
    
    hoveredFace_.Nullify();
    if (!rootNode_) return;
    buildFaceCache();
    gp_Pnt rayOrigin;
    gp_Dir rayDir;
    screenToWorldRay(pos, width(), height(), m_cameraPos, m_cameraRot, m_cameraZoom, rayOrigin, rayDir);
    gp_Lin ray(rayOrigin, rayDir);
    double minDist = 1e9;
    FaceInstance* pickedFace = nullptr;
    for (auto& inst : faceCache_) {
        TreeNode* node = inst.node;
        if (!node || !node->hasFace()) continue;
        TopoDS_Face face = TopoDS::Face(node->getFace().Located(node->loc));
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
        TreeNode* node = pickedFace->node;
        hoveredFace_ = TopoDS::Face(node->getFace().Located(node->loc));
        hoveredFaceNode_ = node; // Store the picked node
    }
    
    qint64 totalHoverTime = hoverTimer.nsecsElapsed();
    if (g_frameCount % 60 == 0) {
        qDebug() << "[Profile] pickHoveredFaceAt time:" << totalHoverTime / 1000000.0 << "ms";
    }
} 

void CadOpenGLWidget::buildFaceCache() {
    // Keep face cache building working for camera controls
    QElapsedTimer cacheTimer;
    cacheTimer.start();
    
    faceCache_.clear();
    if (!rootNode_) return;
    
    // Optimize face cache building by pre-allocating and using more efficient traversal
    std::function<void(TreeNode*)> traverse;
    traverse = [&](TreeNode* node) {
        if (!node || !node->visible) return;
        if (node->type == TopAbs_FACE && node->hasFace()) {
            faceCache_.push_back(FaceInstance{node});
        } else if (node->type == TopAbs_SOLID || node->type == TopAbs_SHELL || node->type == TopAbs_COMPOUND) {
            // Only add parent nodes for picking if they have significant geometry
            TopoDS_Shape shape = node->shapeData.shape;
            if (!shape.IsNull()) {
                // Count faces to determine if this node is worth caching
                int faceCount = 0;
                for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next()) {
                    faceCount++;
                    if (faceCount > 10) break; // Limit face counting for performance
                }
                if (faceCount > 0) {
                    faceCache_.push_back(FaceInstance{node});
                }
            }
        }
        for (auto& child : node->children) {
            traverse(child.get());
        }
    };
    for (auto& child : rootNode_->children) traverse(child.get());
    
    qint64 cacheTime = cacheTimer.nsecsElapsed();
    if (g_frameCount % 60 == 0) {
        qDebug() << "[Profile] buildFaceCache took:" << cacheTime / 1000000.0 << "ms (cached" << faceCache_.size() << "faces)";
    }
} 

// Add this slot for tree selection
void CadOpenGLWidget::setSelectedNode(TreeNode* node) {
    selectedFaceNode_ = nullptr;
    selectedEdge_ = TopoDS_Edge();
    if (!node) {
        update();
        return;
    }
    if (node->type == TopAbs_FACE && node->hasFace()) {
        selectedFaceNode_ = node;
    } else if (node->type == TopAbs_EDGE && node->hasEdge()) {
        // If you want to support edge highlighting, set selectedEdgeNode_ here (not implemented in this code)
        // selectedEdgeNode_ = node;
        // For now, just clear face selection
        selectedFaceNode_ = nullptr;
    } else {
        selectedFaceNode_ = nullptr;
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
    glTranslatef(m_pivotWorldPos.x(), m_pivotWorldPos.y(), m_pivotWorldPos.z());
    glColor4f(1.0f, 0.2f, 0.2f, 0.8f); // Red, semi-transparent
    GLUquadric* quad = gluNewQuadric();
    gluSphere(quad, 150.0, 16, 8); // Increased radius from 5.0 to 15.0
    gluDeleteQuadric(quad);
    glPopMatrix();
} 

void CadOpenGLWidget::setCamera(const QVector3D& pos, const QQuaternion& rot, float zoom) {
    m_cameraPos = pos;
    m_cameraRot = rot;
    m_cameraZoom = zoom;
    update();
} 

void CadOpenGLWidget::setPivotSphere(const QVector3D& worldPos) {
    m_pivotWorldPos = worldPos;
    m_showPivotSphere = true;
    update();
} 

void CadOpenGLWidget::reframeCamera() {
    if (!rootNode_) return;
    // Compute bounding box of all geometry
    Bnd_Box bbox;
    std::function<void(TreeNode*)> traverse;
    traverse = [&](TreeNode* node) {
        if (!node) return;
        if (node->type == TopAbs_FACE && node->hasFace()) {
            TopoDS_Face face = node->getFace();
            BRepBndLib::Add(face, bbox);
        }
        for (const auto& child : node->children) traverse(child.get());
    };
    for (const auto& child : rootNode_->children) traverse(child.get());
    if (bbox.IsVoid()) return;
    Standard_Real xmin, ymin, zmin, xmax, ymax, zmax;
    bbox.Get(xmin, ymin, zmin, xmax, ymax, zmax);
    QVector3D center((xmin + xmax) * 0.5, (ymin + ymax) * 0.5, (zmin + zmax) * 0.5);
    double diag = std::sqrt((xmax - xmin)*(xmax - xmin) + (ymax - ymin)*(ymax - ymin) + (zmax - zmin)*(zmax - zmin));
    float zoom = float(diag / (2.0 * std::tan(45.0 * M_PI / 360.0))); // fit in 45 deg fov
    QVector3D camPos = center + QVector3D(0, 0, zoom);
    QQuaternion camRot; // identity, looking down -Z
    setCamera(camPos, camRot, zoom);
} 



