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
#include <sstream>

#include "CadNode.h"
#include "SimulationManager.h"
#include <QWidget>
#include <QObject>
#include <functional>
#include <mutex>

namespace {

constexpr float kPi = 3.14159265358979323846f;

template<typename NodeType>
void setGlobalLocRecursive(NodeType* node, const TopLoc_Location& parentGlobalLoc = TopLoc_Location()) {
    if (!node) return;
    node->globalLoc = parentGlobalLoc * node->loc;
    for (auto& child : node->children) {
        setGlobalLocRecursive(child.get(), node->globalLoc);
    }
}
static CadOpenGLWidget::FaceInstance* findFaceInstance(std::vector<CadOpenGLWidget::FaceInstance>& faceCache, CadNode* node) {
    for (auto& inst : faceCache) {
        if (inst.node == node) return &inst;
    }
    return nullptr;
}
static std::pair<const CadNode*, TopLoc_Location> findReferenceParentWithLoc(const CadNode* root, const CadNode* target, const TopLoc_Location& targetLoc) {
    if (!root) return {nullptr, TopLoc_Location()};
    if (root->type == CadNodeType::XCAF && root->children.size() == 1 &&
        root->children[0].get() == target && root->children[0]->type == CadNodeType::XCAF) {
        return {root, targetLoc};
    }
    for (const auto& child : root->children) {
        auto found = findReferenceParentWithLoc(child.get(), target, targetLoc * child->loc);
        if (found.first) return found;
    }
    return {nullptr, TopLoc_Location()};
}
} // end anonymous namespace

static void screenToWorldRay(const QPoint& pos, int width, int height, const QVector3D& cameraPos, const QQuaternion& cameraRot, float cameraZoom, gp_Pnt& rayOrigin, gp_Dir& rayDir);
QPoint CadOpenGLWidget_projectWorldToScreen(const QVector3D& world, int width, int height, const QVector3D& cameraPos, const QQuaternion& cameraRot, float cameraZoom);
static QVector3D unprojectScreenToWorld(const QPoint& screenPos, float depth, int width, int height, const QVector3D& cameraPos, const QQuaternion& cameraRot, float cameraZoom);
static void accumulateBoundingBox(const CadNode* node, const TopLoc_Location& accumulatedLoc, Bnd_Box& bbox);

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

CadOpenGLWidget::CadOpenGLWidget(CadNode* root, QWidget *parent)
    : QOpenGLWidget(parent), rootNode_(root) {
    
    m_renderTimer.setInterval(16); // ~60fps (1000ms / 60fps ≈ 16.67ms)
    connect(&m_renderTimer, &QTimer::timeout, this, [this]() {
        if (m_windowActive) {
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
    m_colorBatches.clear();
    
    clearGeometryCache();
}

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
    
    Bnd_Box bbox;
    BRepBndLib::Add(face, bbox);
    Standard_Real xmin, ymin, zmin, xmax, ymax, zmax;
    bbox.Get(xmin, ymin, zmin, xmax, ymax, zmax);
    double faceSize = std::max(std::max(xmax-xmin, ymax-ymin), zmax-zmin);
    
    double basePrecision = std::max<double>(0.05, std::min<double>(1.0, faceSize * 0.2));
    
    QVector3D faceCenter((xmin + xmax) * 0.5, (ymin + ymax) * 0.5, (zmin + zmax) * 0.5);
    QVector3D toCamera = faceCenter - camera_.pos;
    float distanceSquared = toCamera.x() * toCamera.x() + toCamera.y() * toCamera.y() + toCamera.z() * toCamera.z();
    float distanceFactor = std::min(4.0f, std::max(0.2f, distanceSquared / (camera_.zoom * camera_.zoom)));
    
    double precision = basePrecision * distanceFactor;
    
    if (faceSize < 0.0001) {
        m_skippedFaces++;
        cache.markInvalid();
        return cache;
    }

    precision *= 1.2; // 1.2x coarser for performance (was 1.5)
    
    BRepMesh_IncrementalMesh mesher(face, precision);
    TopLoc_Location locCopy;
    Handle(Poly_Triangulation) triangulation = BRep_Tool::Triangulation(face, locCopy);
    
    if (!QOpenGLContext::currentContext()) {
        qDebug() << "No OpenGL context available for face at center" << faceCenter;
        cache.markInvalid();
        return cache;
    }
    
    if (!triangulation.IsNull() && !triangulation->Triangles().IsEmpty()) {
        const Poly_Array1OfTriangle& triangles = triangulation->Triangles();
        
        int nbNodes = triangulation->NbNodes();
        
        std::vector<float> vertices;
        vertices.reserve(triangles.Size() * 9); // 3 vertices per triangle, 3 floats per vertex
        
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
                cache.triangleCount = static_cast<int>(vertices.size() / 9);
                cache.initialized = true;
            } else {
                qDebug() << "Failed to create display list for face at center" << faceCenter;
                cache.markInvalid();
            }
        } else {
            cache.markInvalid();
        }
    } else {
        m_skippedFaces++;
        cache.markInvalid();
    }
    
    return cache;
}

CachedGeometry& CadOpenGLWidget::getOrCreateCachedMeshGeometry(const MeshGeometryData* meshData) {
    const void* meshPtr = meshData;
    auto it = m_geometryCache.find(meshPtr);
    if (it != m_geometryCache.end() && it->second.initialized) {
        return it->second;
    }

    CachedGeometry& cache = m_geometryCache[meshPtr];
    if (cache.initialized) return cache;

    if (!meshData || !meshData->loaded || meshData->vertices.empty() || meshData->indices.empty()) {
        cache.markInvalid();
        return cache;
    }

    cache.displayList = glGenLists(1);
    if (cache.displayList == 0) {
        cache.markInvalid();
        return cache;
    }

    glNewList(cache.displayList, GL_COMPILE);
    glBegin(GL_TRIANGLES);
    for (size_t i = 0; i + 2 < meshData->indices.size(); i += 3) {
        for (int j = 0; j < 3; ++j) {
            const uint32_t index = meshData->indices[i + static_cast<size_t>(j)];
            const size_t vertexOffset = static_cast<size_t>(index) * 3;
            if (vertexOffset + 2 >= meshData->vertices.size()) continue;
            glVertex3f(meshData->vertices[vertexOffset],
                       meshData->vertices[vertexOffset + 1],
                       meshData->vertices[vertexOffset + 2]);
        }
    }
    glEnd();
    glEndList();

    cache.triangleCount = static_cast<int>(meshData->indices.size() / 3);
    cache.initialized = true;
    return cache;
}

void CadOpenGLWidget::renderBatchedGeometry() {
    if (!rootNode_) {
        if (m_frameCount % 120 == 0) {
            qDebug() << "[Render] No root node available";
        }
        return;
    }
    if (m_frameCount % 120 == 0) {
    }
    // Traverse from the root node itself, not just its children
    traverseAndRender(rootNode_, CADNodeColor(1.0f, 1.0f, 1.0f, 1.0f), TopLoc_Location(), true, nullptr);
}

void CadOpenGLWidget::buildColorBatches() {
    m_colorBatches.clear();
    
}

// Transform, Physics, and reference nodes are all transform-like for accumulation.
static inline bool isTransformLikeNode(const CadNode* node) {
    if (!node) return false;
    if (node->type == CadNodeType::Transform || node->type == CadNodeType::Physics) return true;
    // Reference node: XCAF node with a single child that is also XCAF and is shared (heuristic)
    if (node->type == CadNodeType::XCAF && node->children.size() == 1) {
        if (node->children[0] && node->children[0].get() != node && node->children[0]->type == CadNodeType::XCAF) {
            return true;
        }
    }
    return false;
}

static TopLoc_Location getNodeLocation(const CadNode* node, const SimulationManager* simManager) {
    if (!node) return TopLoc_Location();
    
    if (simManager && simManager->hasNodeUpdates()) {
        const auto& nodeLocations = simManager->getLatestNodeLocations();
        auto it = nodeLocations.find(const_cast<CadNode*>(node));
        if (it != nodeLocations.end()) {
            return it->second;
        }
    }
    
    return node->loc;
}

static TopLoc_Location accumulateNodeTransform(const CadNode* node, const TopLoc_Location& parentAccum, const SimulationManager* simManager = nullptr) {
    if (!node) return parentAccum;
    
    TopLoc_Location nodeLoc = getNodeLocation(node, simManager);
    if (nodeLoc.IsIdentity()) return parentAccum;
    
    if (isTransformLikeNode(node)) {
        return parentAccum * nodeLoc;
    }
    return parentAccum;
}

void CadOpenGLWidget::traverseAndRender(const CadNode* node, CADNodeColor inheritedColor, const TopLoc_Location& accumulatedLoc, bool /*ancestorsVisible*/, const CadNode* parentReferenceNode) {
    if (!node) return;
    
    TopLoc_Location nodeLoc = getNodeLocation(node, m_simulationManager);
    
    TopLoc_Location newAccumulatedLoc = accumulatedLoc * nodeLoc;
    bool needsTransform = !nodeLoc.IsIdentity();
    if (needsTransform) {
        glPushMatrix();
        const gp_Trsf& trsf = nodeLoc.Transformation();
        double mat[16] = {
            trsf.Value(1,1), trsf.Value(2,1), trsf.Value(3,1), 0.0,
            trsf.Value(1,2), trsf.Value(2,2), trsf.Value(3,2), 0.0,
            trsf.Value(1,3), trsf.Value(2,3), trsf.Value(3,3), 0.0,
            trsf.TranslationPart().X(), trsf.TranslationPart().Y(), trsf.TranslationPart().Z(), 1.0
        };
        glMultMatrixd(mat);
    }
    if (node->visible) {
        CADNodeColor nodeColor = (node->color.a >= 0.0f) ? node->color : inheritedColor;
        if (node->type == CadNodeType::XCAF) {
            const XCAFNodeData* xData = node->asXCAF();
            if (xData) {
                switch (xData->type) {
                    case TopAbs_FACE:
                        if (!xData->shape.IsNull() && xData->type == TopAbs_FACE) {
                            if (isInFrustum(TopoDS::Face(xData->shape), newAccumulatedLoc)) {
                                renderFaceOptimized(node, newAccumulatedLoc, parentReferenceNode);
                            }
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
        if (node->type == CadNodeType::MeshGeometry) {
            renderMeshGeometry(node->asMeshGeometry(), nodeColor);
        }
        if (node->type == CadNodeType::ConnectionPoint) {
            renderConnectionPoint(node, nodeColor);
        }
    }
    if (node->type == CadNodeType::Physics) {
        const PhysicsNodeData* physData = node->asPhysics();
        if (physData && physData->convexHullGenerated && !physData->hulls.empty()) {
            renderConvexHulls(physData);
        }
    }
    if (node->type == CadNodeType::RobotLink) {
        const RobotLinkData* linkData = node->asRobotLink();
        if (linkData && linkData->collisionHullsVisible && !linkData->collisionHulls.empty()) {
            renderConvexHullList(linkData->collisionHulls);
        }
    }
    if (node->type == CadNodeType::RobotTool) {
        const RobotToolData* toolData = node->asRobotTool();
        if (toolData && toolData->collisionHullsVisible && !toolData->collisionHulls.empty()) {
            renderConvexHullList(toolData->collisionHulls);
        }
    }
    for (const auto& child : node->children) {
        if (child) {
            traverseAndRender(child.get(), node->color, newAccumulatedLoc, true, parentReferenceNode);
        }
    }
    if (needsTransform) {
        glPopMatrix();
    }
}

void CadOpenGLWidget::renderConnectionPoint(const CadNode*, const CADNodeColor& color) {
    glPushAttrib(GL_ENABLE_BIT | GL_POINT_BIT | GL_COLOR_BUFFER_BIT);
    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glColor4f(color.r, color.g, color.b, color.a > 0.0f ? color.a : 1.0f);
    float radius = 10.0f; // Adjust as needed for visibility
    int slices = 16, stacks = 8;
    for (int i = 0; i < stacks; ++i) {
        float lat0 = kPi * (-0.5f + float(i) / stacks);
        float z0  = sin(lat0);
        float zr0 = cos(lat0);
        float lat1 = kPi * (-0.5f + float(i+1) / stacks);
        float z1 = sin(lat1);
        float zr1 = cos(lat1);
        glBegin(GL_QUAD_STRIP);
        for (int j = 0; j <= slices; ++j) {
            float lng = 2.0f * kPi * float(j) / slices;
            float x = cos(lng);
            float y = sin(lng);
            glVertex3f(radius * x * zr0, radius * y * zr0, radius * z0);
            glVertex3f(radius * x * zr1, radius * y * zr1, radius * z1);
        }
        glEnd();
    }
    glPopAttrib();
}

HighlightState CadOpenGLWidget::getFaceHighlightState(const CadNode* node, const TopLoc_Location& accumulatedLoc, const CadNode* parentReferenceNode, const std::vector<SelectedInstance>& selectedFaceInstances_, const SelectedInstance& hoveredFaceInstance_) {
    if (!node) return HighlightState::Normal;
    if (node->excludedFromDecomposition) return HighlightState::Excluded;
    for (const auto& inst : selectedFaceInstances_) {
        if (inst.node == node && inst.accumulatedLoc == accumulatedLoc) return HighlightState::Selected;
    }
    if (hoveredFaceInstance_.node == node && hoveredFaceInstance_.accumulatedLoc == accumulatedLoc) return HighlightState::Hovered;
    if (parentReferenceNode && hoveredFaceInstance_.node == parentReferenceNode) return HighlightState::Hovered;
    return HighlightState::Normal;
}

HighlightState CadOpenGLWidget::getEdgeHighlightState(const CadNode* node, const TopLoc_Location& accumulatedLoc, const std::vector<SelectedInstance>& selectedEdgeInstances_, const SelectedInstance& hoveredEdgeInstance_) {
    if (!node) return HighlightState::Normal;
    for (const auto& inst : selectedEdgeInstances_) {
        if (inst.node == node && inst.accumulatedLoc == accumulatedLoc) return HighlightState::Selected;
    }
    if (hoveredEdgeInstance_.node == node && hoveredEdgeInstance_.accumulatedLoc == accumulatedLoc) return HighlightState::Hovered;
    return HighlightState::Normal;
}

void CadOpenGLWidget::getHighlightColorAndLineWidth(HighlightState state, CADNodeColor baseColor, float& outR, float& outG, float& outB, float& outA, float& outLineWidth) {
    switch (state) {
        case HighlightState::Selected:
            outR = 1.0f; outG = 1.0f; outB = 0.0f; outA = 0.8f; outLineWidth = 2.5f;
            break;
        case HighlightState::Hovered:
            outR = 0.0f; outG = 1.0f; outB = 0.0f; outA = 0.7f; outLineWidth = 2.0f;
            break;
        case HighlightState::Excluded:
            outR = 0.4f; outG = 0.4f; outB = 0.4f; outA = 0.5f; outLineWidth = 1.0f;
            break;
        case HighlightState::Normal:
        default:
            outR = baseColor.r; outG = baseColor.g; outB = baseColor.b; outA = baseColor.a; outLineWidth = 1.0f;
            break;
    }
}

void CadOpenGLWidget::renderFaceOptimized(const CadNode* node, const TopLoc_Location& accumulatedLoc, const CadNode* parentReferenceNode) {
    if (!node) return;
    const XCAFNodeData* xData = node->asXCAF();
    if (!xData || !xData->hasFace()) return;
    const TopoDS_Face& face = TopoDS::Face(xData->shape);
    if (face.IsNull()) return;
    CachedGeometry& cache = getOrCreateCachedGeometry(face);
    if (!cache.isValid()) return;
    HighlightState state = getFaceHighlightState(node, accumulatedLoc, parentReferenceNode, selectedFaceInstances_, hoveredFaceInstance_);
    float r, g, b, a, lineWidth;
    getHighlightColorAndLineWidth(state, node->color, r, g, b, a, lineWidth);
    glColor4f(r, g, b, a);
    glCallList(cache.displayList);
    if (state == HighlightState::Selected || state == HighlightState::Hovered) {
        glColor4f(0.0f, 0.0f, 0.0f, 0.8f);
        glLineWidth(lineWidth + 1.0f);
    } else {
        glColor4f(0.0f, 0.0f, 0.0f, 0.3f);
        glLineWidth(1.0f);
    }
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glCallList(cache.displayList);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glLineWidth(1.0f);
}

void CadOpenGLWidget::renderMeshGeometry(const MeshGeometryData* meshData, const CADNodeColor& color) {
    if (!meshData || !meshData->loaded) return;
    CachedGeometry& cache = getOrCreateCachedMeshGeometry(meshData);
    if (!cache.isValid()) return;

    glColor4f(color.r, color.g, color.b, color.a > 0.0f ? color.a : 1.0f);
    glCallList(cache.displayList);
}

void CadOpenGLWidget::renderEdgeOptimized(const CadNode* node, const TopLoc_Location& accumulatedLoc) {
    const XCAFNodeData* xData = node->asXCAF();
    if (!xData || !xData->hasEdge()) return;
    const TopoDS_Edge& edge = TopoDS::Edge(xData->shape);
    if (edge.IsNull()) return;
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
    HighlightState state = getEdgeHighlightState(node, accumulatedLoc, selectedEdgeInstances_, hoveredEdgeInstance_);
    float r, g, b, a, lineWidth;
    getHighlightColorAndLineWidth(state, node->color, r, g, b, a, lineWidth);
    glColor4f(r, g, b, a);
    glLineWidth(lineWidth + 1.0f);
    Standard_Real first, last;
    TopLoc_Location locCopy;
    Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, locCopy, first, last);
    if (curve.IsNull()) {
        if (needsTransform) {
            glPopMatrix();
        }
        return;
    }
    Standard_Real curveLength = last - first;
    int N = std::max<int>(10, static_cast<int>(curveLength * 5));
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
    glLineWidth(1.0f);
    if (needsTransform) {
        glPopMatrix();
    }
}

void CadOpenGLWidget::paintGL() {
    if (m_simulationManager && m_simulationManager->isSceneBuilding()) {
        return;
    }
    
    m_frameCount++;
    if (m_skipFrames > 0) {
        m_skipFrames--;
    }

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


    QOpenGLFunctions* f = QOpenGLContext::currentContext()->functions();
    f->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // Camera transform order: zoom, then rotation, then translation.
    QMatrix4x4 view;
    view.translate(0, 0, -camera_.zoom);
    view.rotate(camera_.rot.conjugated());
    view.translate(-camera_.pos);
    glMultMatrixf(view.constData());

    if (rootNode_ && rootNode_->type == CadNodeType::MutexRoot) {
        const MutexRootNodeData* mutexData = rootNode_->asMutexRoot();
        if (mutexData && mutexData->groundPlaneVisible) {
            renderGroundPlane(*mutexData);
        }
    }

    static bool frustumInitialized = false;
    if (!frustumInitialized) {
        updateFrustumPlanes();
        frustumInitialized = true;
    }

    if (m_cacheDirty) {
        buildColorBatches();
        m_cacheDirty = false;
    }

    renderBatchedGeometry();

    if (m_selectionMode == SelectionMode::Edges && (hoveredEdgeInstance_.node != nullptr || !selectedEdgeInstances_.empty())) {
        renderHighlightedEdges();
    }
    
    if (m_simulationManager && m_simulationManager->hasNodeUpdates()) {
        m_simulationManager->markUpdatesProcessed();
    }

    if (camera_.showPivotSphere) {
        drawPivotSphere();
    }

    if (m_showZoomContactSphere) {
        glPushMatrix();
        glTranslatef(m_zoomContactPoint.x(), m_zoomContactPoint.y(), m_zoomContactPoint.z());
        glColor4f(0.2f, 0.8f, 1.0f, 0.7f);
        GLUquadric* quad = gluNewQuadric();
        gluSphere(quad, 150.0, 16, 8);
        gluDeleteQuadric(quad);
        glPopMatrix();
    }

    Bnd_Box bbox;
    accumulateBoundingBox(rootNode_, TopLoc_Location(), bbox);
    if (!bbox.IsVoid()) {
        Standard_Real xmin, ymin, zmin, xmax, ymax, zmax;
        bbox.Get(xmin, ymin, zmin, xmax, ymax, zmax);
        drawBoundingBox(QVector3D(xmin, ymin, zmin), QVector3D(xmax, ymax, zmax), QVector4D(1.0f, 0.2f, 0.2f, 0.4f));
    }

    Bnd_Box selectionBox;
    for (const auto& sel : selectedFaceInstances_) {
        if (!sel.node) continue;
        XCAFNodeData* xData = sel.node->asXCAF();
        if (!xData || !xData->hasFace()) continue;
        TopoDS_Face face = TopoDS::Face(xData->getFace().Located(sel.accumulatedLoc));
        BRepBndLib::Add(face, selectionBox);
    }
    for (const auto& sel : selectedEdgeInstances_) {
        if (!sel.node) continue;
        XCAFNodeData* xData = sel.node->asXCAF();
        if (!xData || !xData->hasEdge()) continue;
        TopoDS_Edge edge = TopoDS::Edge(xData->getEdge().Located(sel.accumulatedLoc));
        BRepBndLib::Add(edge, selectionBox);
    }
    if (!selectionBox.IsVoid()) {
        Standard_Real sxmin, symin, szmin, sxmax, symax, szmax;
        selectionBox.Get(sxmin, symin, szmin, sxmax, symax, szmax);
        drawBoundingBox(QVector3D(sxmin, symin, szmin), QVector3D(sxmax, symax, szmax), QVector4D(1.0f, 1.0f, 0.0f, 0.8f));
    }

    if (selectedFrameNode_) {
        drawReferenceFrame(selectedFrameNodeAccumulatedLoc_, 5000.0f);
    }
    
}

void CadOpenGLWidget::mousePressEvent(QMouseEvent* event) {
    m_lastMousePos = event->pos();
    
    if (event->button() == Qt::LeftButton) {
        camera_.mousePressPos = event->pos();
        camera_.mousePressed = true;
        camera_.mouseDragged = false;
    } else if (event->button() == Qt::MiddleButton) {
        QVector3D hitPoint;
        if (m_selectionMode == SelectionMode::Faces) {
            pickHoveredFaceAt(event->pos());
            if (hoveredFaceInstance_.node) {
                if (event->modifiers() & Qt::ControlModifier) {
                    addToSelection(hoveredFaceInstance_.node, hoveredFaceInstance_.accumulatedLoc);
                    emit facePicked(hoveredFaceInstance_.node, hoveredFaceInstance_.accumulatedLoc);
                } else {
                    clearSelection();
                    selectedFaceInstances_.push_back(hoveredFaceInstance_);
                    selectedFaceNode_ = hoveredFaceInstance_.node;
                    XCAFNodeData* pickedData = hoveredFaceInstance_.node->asXCAF();
                    if (pickedData) {
                        selectedFace_ = TopoDS::Face(pickedData->getFace().Located(hoveredFaceInstance_.accumulatedLoc));
                    }
                    emit facePicked(hoveredFaceInstance_.node, hoveredFaceInstance_.accumulatedLoc);
                    update();
                }
            }
        } else if (m_selectionMode == SelectionMode::Edges) {
            CadNode* pickedNode = nullptr;
            if (pickEdgeAt(event->pos(), &hitPoint)) {
                pickedNode = selectedEdgeNode_;
            }
            if (pickedNode) {
                if (event->modifiers() & Qt::ControlModifier) {
                    addToSelection(pickedNode, TopLoc_Location());
                } else {
                    clearSelection();
                    XCAFNodeData* pickedData = pickedNode->asXCAF();
                    if (pickedData) {
                        selectedEdgeInstances_.push_back(SelectedInstance{pickedNode, TopLoc_Location()});
                        selectedEdgeNode_ = pickedNode;
                        selectedEdge_ = TopoDS::Edge(pickedData->getEdge().Located(pickedNode->loc));
                        emit edgePicked(pickedNode); // Emit signal for tree view update
                    }
                    update();
                }
            }
        }
    } else if (event->button() == Qt::RightButton) {
        QVector3D hitPoint;
        // Use a separate picking function that doesn't change selection
        if (pickElementAtForPivot(event->pos(), &hitPoint)) {
            // Only update the pivot position and show the sphere, do NOT move or recenter the camera
            camera_.pivotWorldPos = hitPoint;
            setPivotSphere(hitPoint);
        }
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
    int dx = event->x() - m_lastMousePos.x();
    int dy = event->y() - m_lastMousePos.y();
    QVector3D newCameraPos = camera_.pos;
    QQuaternion newCameraRot = camera_.rot;
    bool cameraChanged = false;
    
    if (camera_.mousePressed && !camera_.mouseDragged) {
        int dragDistance = (event->pos() - camera_.mousePressPos).manhattanLength();
        if (dragDistance > 3) { // 3 pixel threshold for drag detection
            camera_.mouseDragged = true;
        }
    }
    
    if (event->buttons() & Qt::LeftButton) {
        float panSpeed = camera_.zoom * 0.002f;
        QVector3D right = camera_.rot.rotatedVector(QVector3D(1, 0, 0));
        QVector3D up = camera_.rot.rotatedVector(QVector3D(0, 1, 0));
        QVector3D pan = -right * dx * panSpeed + up * dy * panSpeed;
        newCameraPos += pan;
        camera_.pivotWorldPos += pan;
        cameraChanged = true;
    } else if (event->buttons() & Qt::RightButton) {
        QPoint oldScreenPos;
        QVector3D cameraPosForOld;
        QQuaternion cameraRotForOld;
        float cameraZoomForOld;
        if (m_firstRotateMove) {
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
        QMatrix4x4 view;
        view.translate(0, 0, -cameraZoomForOld);
        view.rotate(cameraRotForOld.conjugated());
        view.translate(-cameraPosForOld);
        QVector3D camSpace = view.map(camera_.pivotWorldPos);
        float pivotDepth = -camSpace.z();
        float angleY = dx * 0.5f;
        float angleX = dy * 0.5f;
        QQuaternion rotY = QQuaternion::fromAxisAndAngle(0, 1, 0, angleY);
        QQuaternion rotX = QQuaternion::fromAxisAndAngle(1, 0, 0, angleX);
        QQuaternion incrementalRot = rotX * rotY;
        QVector3D camToPivot = cameraPosForOld - camera_.pivotWorldPos;
        QVector3D newCamToPivot = incrementalRot.rotatedVector(camToPivot);
        newCameraPos = camera_.pivotWorldPos + newCamToPivot;
        newCameraRot = incrementalRot * cameraRotForOld;
        QPoint newScreenPos = CadOpenGLWidget_projectWorldToScreen(
            camera_.pivotWorldPos, width(), height(), newCameraPos, newCameraRot, camera_.zoom);
        QVector3D worldAtOldScreen = unprojectScreenToWorld(oldScreenPos, pivotDepth, width(), height(), newCameraPos, newCameraRot, camera_.zoom);
        QVector3D worldAtNewScreen = unprojectScreenToWorld(newScreenPos, pivotDepth, width(), height(), newCameraPos, newCameraRot, camera_.zoom);
        QVector3D offset = worldAtOldScreen - worldAtNewScreen;
        newCameraPos += offset;
        camera_.pivotWorldPos += offset;
        cameraChanged = true;
    }
    
    if (cameraChanged) {
        setCamera(newCameraPos, newCameraRot, camera_.zoom);
    }
    
    m_lastMousePos = event->pos();
    
    if (m_selectionMode == SelectionMode::Faces) {
        pickHoveredFaceAt(event->pos());
        hoveredEdge_.Nullify();
        hoveredEdgeInstance_ = SelectedInstance{nullptr, TopLoc_Location()};
    } else if (m_selectionMode == SelectionMode::Edges) {
        pickHoveredEdgeAt(event->pos());
        hoveredFace_.Nullify();
        hoveredFaceInstance_ = SelectedInstance{nullptr, TopLoc_Location()};
    } else {
        hoveredFace_.Nullify();
        hoveredFaceInstance_ = SelectedInstance{nullptr, TopLoc_Location()};
        hoveredEdge_.Nullify();
        hoveredEdgeInstance_ = SelectedInstance{nullptr, TopLoc_Location()};
    }
    
    update();
}

void CadOpenGLWidget::mouseReleaseEvent(QMouseEvent* event) {
    if (event->button() == Qt::LeftButton) {
        if (camera_.mousePressed && !camera_.mouseDragged) {
            if (m_selectionMode == SelectionMode::Faces) {
                if (hoveredFaceInstance_.node) {
                    CadNode* pickedNode = hoveredFaceInstance_.node;
                    TopLoc_Location accLoc = hoveredFaceInstance_.accumulatedLoc;
                    XCAFNodeData* pickedData = pickedNode->asXCAF();
                    if (event->modifiers() & Qt::ControlModifier) {
                        addToSelection(pickedNode, accLoc);
                        emit facePicked(pickedNode, accLoc);
                    } else {
                        clearSelection();
                        selectedFaceInstances_.push_back(SelectedInstance{pickedNode, accLoc});
                        selectedFaceNode_ = pickedNode;
                        if (pickedData) {
                            selectedFace_ = TopoDS::Face(pickedData->getFace().Located(accLoc));
                        }
                        emit facePicked(pickedNode, accLoc);
                    }
                    update();
                }
            } else if (m_selectionMode == SelectionMode::Edges) {
                QVector3D hitPoint;
                if (pickEdgeAt(event->pos(), &hitPoint)) {
                    CadNode* pickedNode = selectedEdgeNode_;
                    XCAFNodeData* pickedData = pickedNode ? pickedNode->asXCAF() : nullptr;
                    if (pickedNode && pickedData) {
                        if (event->modifiers() & Qt::ControlModifier) {
                            addToSelection(pickedNode, TopLoc_Location());
                        } else {
                            clearSelection();
                            selectedEdgeInstances_.push_back(SelectedInstance{pickedNode, TopLoc_Location()});
                            selectedEdgeNode_ = pickedNode;
                            selectedEdge_ = TopoDS::Edge(pickedData->getEdge().Located(pickedNode->loc));
                            emit edgePicked(pickedNode); // Emit signal for tree view update
                        }
                        update();
                    }
                }
            }
        }
        camera_.mousePressed = false;
        camera_.mouseDragged = false;
    } else if (event->button() == Qt::RightButton) {
        camera_.showPivotSphere = false;
        m_firstRotateMove = false;
        update();
    }
}

void CadOpenGLWidget::leaveEvent(QEvent* event) {
    hoveredFace_.Nullify();
    hoveredFaceInstance_ = SelectedInstance{nullptr, TopLoc_Location()};
    hoveredEdge_.Nullify();
    hoveredEdgeInstance_ = SelectedInstance{nullptr, TopLoc_Location()};
    update();
    QOpenGLWidget::leaveEvent(event);
}

void CadOpenGLWidget::focusInEvent(QFocusEvent* event) {
    m_windowActive = true;
    QOpenGLWidget::focusInEvent(event);
}

void CadOpenGLWidget::focusOutEvent(QFocusEvent* event) {
    m_windowActive = false;
    QOpenGLWidget::focusOutEvent(event);
}

QPoint CadOpenGLWidget_projectWorldToScreen(const QVector3D& world, int width, int height, const QVector3D& cameraPos, const QQuaternion& cameraRot, float cameraZoom) {
    QMatrix4x4 view;
    view.translate(0, 0, -cameraZoom);
    view.rotate(cameraRot.conjugated());
    view.translate(-cameraPos);
    QVector3D camSpace = view.map(world);
    float fovY = 45.0f * kPi / 180.0f;
    float aspect = float(width) / float(height ? height : 1);
    float nearPlane = 1.0f;
    float tanFovY = tan(fovY / 2.0f);
    float x = camSpace.x() / (-camSpace.z()) / (aspect * tanFovY / nearPlane);
    float y = camSpace.y() / (-camSpace.z()) / (tanFovY / nearPlane);
    int sx = int((x + 1) * 0.5f * width);
    int sy = int((1 - y) * 0.5f * height);
    return QPoint(sx, sy);
}

static QVector3D unprojectScreenToWorld(const QPoint& screenPos, float depth, int width, int height, const QVector3D& cameraPos, const QQuaternion& cameraRot, float cameraZoom) {
    QMatrix4x4 view;
    view.translate(0, 0, -cameraZoom);
    view.rotate(cameraRot.conjugated());
    view.translate(-cameraPos);
    QMatrix4x4 proj;
    float aspect = float(width) / float(height ? height : 1);
    float nearPlane = 1.0f;
    float farPlane = 1e9f;
    proj.perspective(45.0f, aspect, nearPlane, farPlane);
    QMatrix4x4 inv = (proj * view).inverted();
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
        
        QVector3D camToPoint = intersection - camera_.pos;
        float currentDist = camToPoint.length();
        float zoomRatio = newCameraZoom / camera_.zoom;
        float newDist = currentDist * zoomRatio;
        
        QVector3D newCameraPos = intersection - (camToPoint.normalized() * newDist);
        
        setCamera(newCameraPos, camera_.rot, newCameraZoom);
        
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


void CadOpenGLWidget::clearCache() {
    clearGeometryCache();
    buildFaceCache(); // Rebuild face cache after clearing
    update();
}

int CadOpenGLWidget::getCacheSize() const {
    return static_cast<int>(m_geometryCache.size());
}

void CadOpenGLWidget::updateFrustumPlanes() {
    QMatrix4x4 view;
    view.translate(0, 0, -camera_.zoom);
    view.rotate(camera_.rot.conjugated());
    view.translate(-camera_.pos);
    
    QMatrix4x4 proj;
    float aspect = float(width()) / float(height() ? height() : 1);
    proj.perspective(45.0f, aspect, 1.0f, 1e9f);
    
    QMatrix4x4 combined = proj * view;
    
    for (int i = 0; i < 4; ++i) {
        m_frustumPlanes[0][i] = combined(i,3) + combined(i,0); // Left
        m_frustumPlanes[1][i] = combined(i,3) - combined(i,0); // Right
        m_frustumPlanes[2][i] = combined(i,3) + combined(i,1); // Bottom
        m_frustumPlanes[3][i] = combined(i,3) - combined(i,1); // Top
        m_frustumPlanes[4][i] = combined(i,3) + combined(i,2); // Near
        m_frustumPlanes[5][i] = combined(i,3) - combined(i,2); // Far
    }
    
    for (int i = 0; i < 6; ++i) {
        float length = QVector3D(m_frustumPlanes[i].x(), m_frustumPlanes[i].y(), m_frustumPlanes[i].z()).length();
        if (length > 0) {
            m_frustumPlanes[i] /= length;
        }
    }
}

bool CadOpenGLWidget::isInFrustum(const TopoDS_Face& face, const TopLoc_Location&) {
    Bnd_Box bbox;
    BRepBndLib::Add(face, bbox);
    
    if (bbox.IsVoid()) return true; // If we can't determine bounds, render it
    
    Standard_Real xmin, ymin, zmin, xmax, ymax, zmax;
    bbox.Get(xmin, ymin, zmin, xmax, ymax, zmax);
    
    double faceSize = std::max(std::max(xmax-xmin, ymax-ymin), zmax-zmin);
    if (faceSize < 0.00001) return false; // Skip only extremely small faces (was 0.0001)
    
    
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
    
    if (first >= last) {
        qDebug() << "Invalid parameter range for edge curve";
        return;
    }
    
    Standard_Real curveLength = last - first;
    int N = std::max<int>(20, static_cast<int>(curveLength * 10)); // At least 20 segments, more for longer curves
    
    glBegin(GL_LINE_STRIP);
    for (int i = 0; i <= N; ++i) {
        Standard_Real t = first + (last - first) * i / N;
        gp_Pnt p;
        
        try {
            curve->D0(t, p);
            // Do NOT apply locCopy.Transformation() here
            glVertex3d(p.X(), p.Y(), p.Z());
        } catch (const Standard_Failure&) {
            continue;
        }
    }
    glEnd();
    
    glLineWidth(1.0f); // Reset line width
} 

void CadOpenGLWidget::renderHighlightedEdges() {
    if (hoveredEdgeInstance_.node) {
        renderEdgeOptimized(hoveredEdgeInstance_.node, hoveredEdgeInstance_.accumulatedLoc);
    }
    for (const auto& sel : selectedEdgeInstances_) {
        if (sel.node != hoveredEdgeInstance_.node) { // Don't render twice if hovered and selected
            renderEdgeOptimized(sel.node, sel.accumulatedLoc);
        }
    }
}

static void screenToWorldRay(const QPoint& pos, int width, int height, const QVector3D& cameraPos, const QQuaternion& cameraRot, float cameraZoom, gp_Pnt& rayOrigin, gp_Dir& rayDir)
{
    QMatrix4x4 view;
    view.translate(0, 0, -cameraZoom); // Zoom (dolly)
    view.rotate(cameraRot.conjugated());
    view.translate(-cameraPos);
    QMatrix4x4 invView = view.inverted();

    float ndcX = (2.0f * pos.x()) / width - 1.0f;
    float ndcY = 1.0f - (2.0f * pos.y()) / height;

    float fovY = 45.0f * kPi / 180.0f;
    float aspect = float(width) / float(height ? height : 1);
    float nearPlane = 1.0f;
    float tanFovY = tan(fovY / 2.0f);
    float vx = ndcX * aspect * tanFovY * nearPlane;
    float vy = ndcY * tanFovY * nearPlane;
    float vz = -nearPlane;
    QVector3D rayCam(vx, vy, vz);

    QVector3D camOriginWorld = invView.map(QVector3D(0, 0, 0));
    QVector3D rayWorld = invView.map(rayCam) - camOriginWorld;
    rayWorld.normalize();

    rayOrigin = gp_Pnt(camOriginWorld.x(), camOriginWorld.y(), camOriginWorld.z());
    rayDir = gp_Dir(rayWorld.x(), rayWorld.y(), rayWorld.z());
}

// Shared face-cache ray pick: casts a ray through pos and returns the nearest intersected face
// instance (null on miss), writing the hit point through hitPoint. buildFaceCache is a full tree
// walk, so the hover path reuses the existing cache instead of rebuilding per mouse move.
CadOpenGLWidget::FaceInstance* CadOpenGLWidget::rayPickFace(const QPoint& pos, gp_Pnt* hitPoint, bool rebuildCache) {
    if (rebuildCache) buildFaceCache();
    gp_Pnt rayOrigin;
    gp_Dir rayDir;
    screenToWorldRay(pos, width(), height(), camera_.pos, camera_.rot, camera_.zoom, rayOrigin, rayDir);
    gp_Lin ray(rayOrigin, rayDir);
    double minDist = 1e9;
    FaceInstance* picked = nullptr;
    for (auto& inst : faceCache_) {
        CadNode* node = inst.node;
        XCAFNodeData* xData = node ? node->asXCAF() : nullptr;
        if (!xData || !xData->hasFace()) continue;
        TopoDS_Face face = TopoDS::Face(xData->getFace().Located(inst.accumulatedLoc));
        BRepIntCurveSurface_Inter intersector;
        intersector.Init(face, ray, 1e-6);
        while (intersector.More()) {
            gp_Pnt ipt = intersector.Pnt();
            double dist = rayOrigin.Distance(ipt);
            if (dist < minDist) {
                minDist = dist;
                picked = &inst;
                if (hitPoint) *hitPoint = ipt;
            }
            intersector.Next();
        }
    }
    return picked;
}

bool CadOpenGLWidget::pickElementAt(const QPoint& pos, QVector3D* outIntersection) {
    selectedFace_.Nullify();
    selectedEdge_.Nullify();
    if (!rootNode_) return false;
    gp_Pnt pickedPoint;
    FaceInstance* pickedFace = rayPickFace(pos, &pickedPoint, true);
    if (pickedFace && pickedFace->node) {
        XCAFNodeData* xData = pickedFace->node->asXCAF();
        selectedFace_ = TopoDS::Face(xData->getFace().Located(pickedFace->accumulatedLoc));
        selectedFaceNode_ = pickedFace->node;
        // Don't emit signal here - let the mouse press handler manage selection
        if (outIntersection) {
            *outIntersection = QVector3D(pickedPoint.X(), pickedPoint.Y(), pickedPoint.Z());
        }
        update();
        return true;
    }
    selectedFaceNode_ = nullptr;
    update();
    return false;
}

bool CadOpenGLWidget::pickEdgeAt(const QPoint& pos, QVector3D* outIntersection) {
    selectedEdge_.Nullify();
    selectedEdgeNode_ = nullptr;
    if (!rootNode_) return false;
    buildEdgeCache();
    gp_Pnt rayOrigin;
    gp_Dir rayDir;
    screenToWorldRay(pos, width(), height(), camera_.pos, camera_.rot, camera_.zoom, rayOrigin, rayDir);
    gp_Lin ray(rayOrigin, rayDir);
    double minDist = 1e9;
    EdgeInstance* pickedEdge = nullptr;
    gp_Pnt pickedPoint;
    bool found = false;
    for (auto& inst : edgeCache_) {
        CadNode* node = inst.node;
        XCAFNodeData* xData = inst.node->asXCAF();

        if (!xData || !xData->hasEdge()) continue;
        TopoDS_Edge edge = TopoDS::Edge(xData->getEdge().Located(inst.accumulatedLoc));
        
        Standard_Real first, last;
        TopLoc_Location locCopy;
        Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, locCopy, first, last);
        if (curve.IsNull()) continue;
        
        int numSamples = 50;
        for (int i = 0; i <= numSamples; ++i) {
            Standard_Real t = first + (last - first) * i / numSamples;
            gp_Pnt curvePoint;
            try {
                curve->D0(t, curvePoint);
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
    if (pickedEdge && pickedEdge->node) {
        CadNode* node = pickedEdge->node;
        XCAFNodeData* xData = node->asXCAF();
        selectedEdge_ = TopoDS::Edge(xData->getEdge().Located(pickedEdge->accumulatedLoc));
        selectedEdgeNode_ = node;
        // Don't emit signal here - let the mouse press handler manage selection
        if (outIntersection && found) {
            *outIntersection = QVector3D(pickedPoint.X(), pickedPoint.Y(), pickedPoint.Z());
        }
        update();
        return true;
    } else {
        selectedEdgeNode_ = nullptr;
        update();
        return false;
    }
}

bool CadOpenGLWidget::pickElementAtForPivot(const QPoint& pos, QVector3D* outIntersection) {
    if (!rootNode_) return false;
    gp_Pnt pickedPoint;
    if (!rayPickFace(pos, &pickedPoint, true)) return false;
    if (outIntersection) {
        *outIntersection = QVector3D(pickedPoint.X(), pickedPoint.Y(), pickedPoint.Z());
    }
    return true;
}

static const CadNode* findReferenceParent(const CadNode* root, const CadNode* target) {
    if (!root) return nullptr;
    if (root->type == CadNodeType::XCAF && root->children.size() == 1 &&
        root->children[0].get() == target && root->children[0]->type == CadNodeType::XCAF) {
        return root;
    }
    for (const auto& child : root->children) {
        if (const CadNode* found = findReferenceParent(child.get(), target)) {
            return found;
        }
    }
    return nullptr;
}

void CadOpenGLWidget::pickHoveredFaceAt(const QPoint& pos) {
    hoveredFace_.Nullify();
    if (!rootNode_) return;
    FaceInstance* pickedFace = rayPickFace(pos, nullptr, false);
    if (pickedFace && pickedFace->node) {
        CadNode* node = pickedFace->node;
        XCAFNodeData* xData = node->asXCAF();
        hoveredFace_ = TopoDS::Face(xData->getFace().Located(pickedFace->accumulatedLoc));
        hoveredFaceInstance_ = SelectedInstance{node, pickedFace->accumulatedLoc};
    } else {
        hoveredFaceInstance_ = SelectedInstance{nullptr, TopLoc_Location()};
    }
}

void CadOpenGLWidget::pickHoveredEdgeAt(const QPoint& pos) {
    hoveredEdge_.Nullify();
    hoveredEdgeInstance_ = SelectedInstance{nullptr, TopLoc_Location()};
    if (!rootNode_) return;
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
        TopoDS_Edge edge = TopoDS::Edge(xData->getEdge().Located(inst.accumulatedLoc));
        
        Standard_Real first, last;
        TopLoc_Location locCopy;
        Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, locCopy, first, last);
        if (curve.IsNull()) continue;
        
        int numSamples = 20; // Reduced samples for hover performance
        for (int i = 0; i <= numSamples; ++i) {
            Standard_Real t = first + (last - first) * i / numSamples;
            gp_Pnt curvePoint;
            try {
                curve->D0(t, curvePoint);
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
        hoveredEdge_ = TopoDS::Edge(xData->getEdge().Located(pickedEdge->accumulatedLoc));
        hoveredEdgeInstance_ = SelectedInstance{node, pickedEdge->accumulatedLoc};
    } else {
        hoveredEdgeInstance_ = SelectedInstance{nullptr, TopLoc_Location()};
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
    faceCache_.clear();
    if (!rootNode_) return;
    std::vector<std::pair<const CadNode*, TopLoc_Location>> faceNodes;
    std::function<void(const CadNode*, TopLoc_Location)> collect;
    collect = [&](const CadNode* node, TopLoc_Location accumulatedLoc) {
        if (!node) return;
        TopLoc_Location newAccumulatedLoc = accumulatedLoc * node->loc;
        const XCAFNodeData* xData = node->asXCAF();
        if (xData && xData->type == TopAbs_FACE && xData->hasFace()) {
            faceNodes.emplace_back(node, newAccumulatedLoc);
        }
        for (const auto& child : node->children) {
            collect(child.get(), newAccumulatedLoc);
        }
    };
    // Traverse from the root node itself, not just its children
    collect(rootNode_, TopLoc_Location());
    for (const auto& pair : faceNodes) faceCache_.push_back(FaceInstance{const_cast<CadNode*>(pair.first), pair.second});
}

void CadOpenGLWidget::buildEdgeCache() {
    edgeCache_.clear();
    if (!rootNode_) return;
    
    std::function<void(CadNode*)> traverse;
    traverse = [&](CadNode* node) {
        if (!node || !node->visible) return;
        XCAFNodeData* xData = node->asXCAF();
        if (!xData) {
            for (auto& child : node->children) traverse(child.get());
            return;
        }
        if (xData->type == TopAbs_EDGE && xData->hasEdge()) {
            edgeCache_.push_back(EdgeInstance{node});
        } else if (xData->type == TopAbs_SOLID || xData->type == TopAbs_SHELL || xData->type == TopAbs_COMPOUND) {
            // Only add parent nodes for picking if they have significant geometry
            TopoDS_Shape shape = xData->shape;
            if (!shape.IsNull()) {
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
}

void CadOpenGLWidget::setSelectionMode(SelectionMode mode) {
    m_selectionMode = mode;
    clearSelection();
    hoveredFace_.Nullify();
    hoveredFaceInstance_ = SelectedInstance{nullptr, TopLoc_Location()};
    hoveredEdge_.Nullify();
    hoveredEdgeInstance_ = SelectedInstance{nullptr, TopLoc_Location()};
    update();
}


void CadOpenGLWidget::clearSelection() {
    selectedFaceInstances_.clear();
    selectedEdgeInstances_.clear();
    selectedFaceNode_ = nullptr;
    selectedEdgeNode_ = nullptr;
    selectedFace_.Nullify();
    selectedEdge_.Nullify();
    selectedFrameNode_ = nullptr;
}

void CadOpenGLWidget::addToSelection(CadNode* node, const TopLoc_Location& accLoc) {
    if (!node) return;
    XCAFNodeData* xData = node->asXCAF();
    if (!xData) return;
    if (xData->type == TopAbs_FACE && xData->hasFace()) {
        SelectedInstance instance{node, accLoc};
        auto selIt = std::find(selectedFaceInstances_.begin(), selectedFaceInstances_.end(), instance);
        if (selIt != selectedFaceInstances_.end()) {
            selectedFaceInstances_.erase(selIt);
        } else {
            selectedFaceInstances_.push_back(instance);
        }
    } else if (xData->type == TopAbs_EDGE && xData->hasEdge()) {
        SelectedInstance instance{node, accLoc};
        auto selIt = std::find(selectedEdgeInstances_.begin(), selectedEdgeInstances_.end(), instance);
        if (selIt != selectedEdgeInstances_.end()) {
            selectedEdgeInstances_.erase(selIt);
        } else {
            selectedEdgeInstances_.push_back(instance);
        }
    }
    update();
}

void CadOpenGLWidget::removeFromSelection(CadNode* node) {
    if (!node) return;
    XCAFNodeData* xData = node->asXCAF();
    if (!xData) return;

    
    if (xData->type == TopAbs_FACE && xData->hasFace()) {
        auto it = std::find_if(faceCache_.begin(), faceCache_.end(), [node](const FaceInstance& inst) { return inst.node == node; });
        if (it == faceCache_.end()) return;
        SelectedInstance instance{node, it->accumulatedLoc};
        auto selIt = std::find(selectedFaceInstances_.begin(), selectedFaceInstances_.end(), instance);
        if (selIt != selectedFaceInstances_.end()) {
            selectedFaceInstances_.erase(selIt);
        }
    } else if (xData->type == TopAbs_EDGE && xData->hasEdge()) {
        auto it = std::find_if(edgeCache_.begin(), edgeCache_.end(), [node](const EdgeInstance& inst) { return inst.node == node; });
        if (it == edgeCache_.end()) return;
        SelectedInstance instance{node, it->accumulatedLoc};
        auto selIt = std::find(selectedEdgeInstances_.begin(), selectedEdgeInstances_.end(), instance);
        if (selIt != selectedEdgeInstances_.end()) {
            selectedEdgeInstances_.erase(selIt);
        }
    }
    update();
} 

static QPoint worldToScreen(const QVector3D& world, int width, int height, float camDist, float camRotX, float camRotY, const QVector3D& panOffset, const QVector3D& center) {
    QMatrix4x4 mv;
    mv.translate(0, 0, -camDist);
    mv.rotate(camRotX, 1, 0, 0);
    mv.rotate(camRotY, 0, 1, 0);
    mv.translate(panOffset);
    mv.translate(-center);
    QVector3D camSpace = mv.map(world);
    float fovY = 45.0f * kPi / 180.0f;
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

static void accumulateBoundingBox(const CadNode* node, const TopLoc_Location& accumulatedLoc, Bnd_Box& bbox) {
    if (!node) return;
    TopLoc_Location newAccumulatedLoc = accumulatedLoc * node->loc;
    const XCAFNodeData* xData = node->asXCAF();
    if (xData) {
        if (xData->type == TopAbs_FACE && xData->hasFace()) {
            TopoDS_Face locatedFace = TopoDS::Face(xData->getFace().Located(newAccumulatedLoc));
            BRepBndLib::Add(locatedFace, bbox);
        }
    }
    const MeshGeometryData* meshData = node->asMeshGeometry();
    if (meshData && meshData->loaded && !meshData->vertices.empty()) {
        const gp_Trsf& trsf = newAccumulatedLoc.Transformation();
        for (size_t i = 0; i + 2 < meshData->vertices.size(); i += 3) {
            gp_Pnt p(meshData->vertices[i], meshData->vertices[i + 1], meshData->vertices[i + 2]);
            p.Transform(trsf);
            bbox.Add(p);
        }
    }
    for (const auto& child : node->children) {
        if (child) accumulateBoundingBox(child.get(), newAccumulatedLoc, bbox);
    }
}

void CadOpenGLWidget::reframeCamera() {
    if (!rootNode_) return;
    Bnd_Box bbox;
    accumulateBoundingBox(rootNode_, TopLoc_Location(), bbox);
    if (bbox.IsVoid()) return;

    Standard_Real xmin, ymin, zmin, xmax, ymax, zmax;
    bbox.Get(xmin, ymin, zmin, xmax, ymax, zmax);
    QVector3D center((xmin + xmax) * 0.5, (ymin + ymax) * 0.5, (zmin + zmax) * 0.5);

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

    QMatrix4x4 view;
    view.rotate(camera_.rot.conjugated());
    view.translate(-center); // Center the model

    float maxX = 0, maxY = 0;
    for (const auto& c : corners) {
        QVector3D v = view.map(c);
        maxX = std::max(maxX, std::abs(v.x()));
        maxY = std::max(maxY, std::abs(v.y()));
    }

    float fovY = 45.0f * kPi / 180.0f;
    float aspect = float(width()) / float(height() ? height() : 1);
    float halfFovY = fovY / 2.0f;
    float halfFovX = atan(tan(halfFovY) * aspect);

    float distY = maxY / tan(halfFovY);
    float distX = maxX / tan(halfFovX);
    float requiredDist = std::max(distX, distY);
    requiredDist *= 1.1f; // Add a margin

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
    if (rootNode_) {
        std::function<void(CadNode*, CadNode*)> updateIfNeeded = [&](CadNode* node, CadNode* parent) {
            if (!node) return;
            if (node->needsGlobalLocUpdate) {
                setGlobalLocRecursive(node, parent ? parent->globalLoc : TopLoc_Location());
                std::function<void(CadNode*)> clearFlag = [&](CadNode* n) {
                    n->needsGlobalLocUpdate = false;
                    for (auto& child : n->children) clearFlag(child.get());
                };
                clearFlag(node);
            }
            for (auto& child : node->children) updateIfNeeded(child.get(), node);
        };
        updateIfNeeded(rootNode_, nullptr);
    }
    m_cacheDirty = true;
    update();
}

void CadOpenGLWidget::drawBoundingBox(const QVector3D& min, const QVector3D& max, const QVector4D& color) {
    glPushAttrib(GL_ENABLE_BIT | GL_LINE_BIT | GL_COLOR_BUFFER_BIT);
    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glColor4f(color.x(), color.y(), color.z(), color.w());
    glLineWidth(2.0f);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    glBegin(GL_LINES);
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
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(axisLength, 0.0f, 0.0f);
    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, axisLength, 0.0f);
    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, axisLength);
    glEnd();
    glLineWidth(1.0f);
    glPopMatrix();
}

void CadOpenGLWidget::renderConvexHullList(const std::vector<ConvexHullData>& hulls) {
    // No transform application here; rely on OpenGL stack
    for (const auto& hull : hulls) {
        glColor4f(1.0f, 0.5f, 0.1f, 0.3f); // Orange, semi-transparent
        glBegin(GL_TRIANGLES);
        for (const auto& tri : hull.indices) {
            for (int i = 0; i < 3; ++i) {
                const auto& v = hull.vertices[tri[i]];
                glVertex3f(v[0], v[1], v[2]);
            }
        }
        glEnd();
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
}

void CadOpenGLWidget::renderConvexHulls(const PhysicsNodeData* physData) {
    if (!physData) return;
    if (!physData->collisionMeshVisible) return;
    renderConvexHullList(physData->hulls);
}

static bool findNodeAccumulatedLoc(const CadNode* root, const CadNode* target, TopLoc_Location currentLoc, TopLoc_Location& outLoc) {
    if (!root) return false;
    if (root == target) {
        outLoc = currentLoc;
        return true;
    }
    for (const auto& child : root->children) {
        if (findNodeAccumulatedLoc(child.get(), target, currentLoc * child->loc, outLoc)) {
            return true;
        }
    }
    return false;
}

void CadOpenGLWidget::setSelectedFaceNode(CadNode* node, const TopLoc_Location& accLoc) {
    if (!node) return;
    clearSelection();
    selectedFaceInstances_.push_back(SelectedInstance{node, accLoc});
    selectedFaceNode_ = node;
    XCAFNodeData* pickedData = node->asXCAF();
    if (pickedData) {
        selectedFace_ = TopoDS::Face(pickedData->getFace().Located(accLoc));
    }
    update();
}

void CadOpenGLWidget::renderGroundPlane(const MutexRootNodeData& mutexData) {
    if (!mutexData.groundPlaneVisible) return;
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    glColor4f(mutexData.groundPlaneColor.r, 
              mutexData.groundPlaneColor.g, 
              mutexData.groundPlaneColor.b, 
              mutexData.groundPlaneColor.a);
    
    double size = mutexData.groundPlaneSize;
    double y = mutexData.groundPlaneY;
    double thickness = mutexData.groundPlaneThickness;
    
    glBegin(GL_QUADS);
    glVertex3d(-size, y + thickness/2, -size);
    glVertex3d( size, y + thickness/2, -size);
    glVertex3d( size, y + thickness/2,  size);
    glVertex3d(-size, y + thickness/2,  size);
    glEnd();
    
    glColor4f(0.5f, 0.5f, 0.5f, 0.3f); // Darker grid lines
    glLineWidth(1.0f);
    glBegin(GL_LINES);
    
    double gridSpacing = 100.0;
    for (double x = -size; x <= size; x += gridSpacing) {
        glVertex3d(x, y + thickness/2 + 0.01, -size); // Slightly above surface to avoid z-fighting
        glVertex3d(x, y + thickness/2 + 0.01,  size);
    }
    for (double z = -size; z <= size; z += gridSpacing) {
        glVertex3d(-size, y + thickness/2 + 0.01, z);
        glVertex3d( size, y + thickness/2 + 0.01, z);
    }
    glEnd();
    
    glDisable(GL_BLEND);
}





