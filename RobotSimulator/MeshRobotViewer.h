#pragma once

#include "CadNode.h"
#include "GlLoader.h"

#include "SceneMath.h"

#include <array>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

// Renders the robot scene into an offscreen framebuffer for display as an ImGui image.
class MeshRobotViewer {
public:
    explicit MeshRobotViewer(CadNode* root);
    ~MeshRobotViewer();

    MeshRobotViewer(const MeshRobotViewer&) = delete;
    MeshRobotViewer& operator=(const MeshRobotViewer&) = delete;

    // Compiles the shader and creates the shared dynamic buffer and VAO. Requires a current
    // GL context. Returns false if the shader fails to build.
    bool initializeGraphics();

    unsigned int render(int width, int height);
    // Renders through the same framebuffer as the interactive view and writes a binary PPM.
    // PPM keeps this dependency-free and is intentionally aimed at deterministic CLI visual QA.
    bool renderToPpm(const std::string& path, int width, int height);
    // World-space bounds of one subtree, including the node's ancestry. Kept public so CLI
    // diagnostics can verify full-link clearance using the same transformed CAD as the renderer.
    static bool computeBoundsOf(const CadNode* node, Vec3f& outMin, Vec3f& outMax);

    struct CameraState {
        Vec3f center{0.0f, 0.0f, 0.0f};
        float distance = 1000.0f;
        float yaw = -35.0f;
        float pitch = 20.0f;
        bool framed = false;
    };

    CameraState camera() const;
    void setCamera(const CameraState& camera);

    void reframeCamera();
    void reframeCameraOn(const CadNode* node);
    void markCacheDirty();
    // The indices are link numbers within `robotNode`, which is the space RobotCollisionModel
    // reports in: it walks one arm, so its links are numbered 0..n for that arm alone. Passing the
    // arm is what keeps a two-robot cell from highlighting the wrong one - resolving against the
    // whole tree would number every arm's links into one flat sequence. Null means the whole tree,
    // which is right only while there is exactly one robot in it.
    void setCollidingLinkIndices(const std::vector<int>& linkIndices, const CadNode* robotNode = nullptr);
    void setGimbalPoses(const std::vector<CadTransform>& poses);
    void setGimbalsVisible(bool visible);
    void setGimbalMoveCallback(std::function<void(int index, const CadTransform&)> callback);
    void setPathPreviewPoints(const std::vector<std::vector<Vec3f>>& paths);
    void setPathPreviewVisible(bool visible);
    void setSimPathPreviewPoints(const std::vector<std::vector<Vec3f>>& paths);
    void setSimPathPreviewVisible(bool visible);
    void setDragChainPivotsVisible(bool visible);
    void setGantryHullOverlaysVisible(bool visible);
    void setMountingHolesVisible(bool visible);
    struct ConveyorDirectionGuide {
        std::vector<Vec3f> points;
        bool startConnected = false;
        bool endConnected = false;
    };
    void setConveyorDirectionGuides(const std::vector<ConveyorDirectionGuide>& guides);
    void setConveyorDirectionsVisible(bool visible);
    // Draws a silhouette around one subtree using its current posed visual meshes. Null disables
    // the outline; the caller uses that to make selection a Program-workspace affordance.
    void setSelectionOutlineRoot(const CadNode* root);

    // Pointer input, in pixels relative to the rendered image's top-left corner. The host
    // maps ImGui's screen-space cursor into this space.
    bool onPointerPressed(const PointI& localPos);
    void onPointerMoved(const PointI& localPos, bool pressed);
    void onPointerReleased();
    void onScroll(double steps);

    void onPanPressed(const PointI& localPos);
    void onPanMoved(const PointI& localPos);
    void onPanReleased();

    const CadNode* pickNodeAt(const PointI& localPos, const std::vector<CadNode*>& candidates) const;

    // Placement uses the exact camera matrices the scene was drawn with. Its snap tolerance is in
    // screen space, so projecting through any parallel copy of the camera would let the preview and
    // the cursor disagree by a frame or a resize.
    bool projectWorldPoint(const Vec3f& point, PointF& outScreen) const;
    bool worldRayAt(const PointI& localPos, Vec3f& outOrigin, Vec3f& outDirection) const;

    void setPlacementPreviewRoot(const CadNode* root, bool snapped);
    // During a library drag, show only the useful local patch of target holes around the ghost plus
    // the asset's own placement-source holes. This keeps a large fixture grid readable without
    // requiring the global Mounting holes guide to be enabled.
    void setPlacementMountingGuides(const std::vector<Vec3f>& targetPoints,
                                    const std::vector<Vec3f>& sourcePoints);

    enum class GimbalAxis {
        None,
        X,
        Y,
        Z
    };

    enum class GimbalHandle {
        None,
        TranslateX,
        TranslateY,
        TranslateZ,
        RotateX,
        RotateY,
        RotateZ
    };

private:
    CadNode* m_root = nullptr;
    Vec3f m_center{0.0f, 0.0f, 0.0f};
    float m_distance = 1000.0f;
    float m_yaw = -35.0f;
    float m_pitch = 20.0f;
    PointI m_lastMousePos;
    PointI m_lastPanPos;
    bool m_rotating = false;
    bool m_panning = false;
    bool m_draggingGimbal = false;
    GimbalHandle m_activeGimbalHandle = GimbalHandle::None;
    int m_activeGimbalIndex = -1;
    std::vector<CadTransform> m_gimbalPoses;
    bool m_gimbalsVisible = false;
    bool m_pathPreviewVisible = false;
    bool m_simPathPreviewVisible = false;
    bool m_dragChainPivotsVisible = false;
    bool m_gantryHullOverlaysVisible = false;
    bool m_mountingHolesVisible = false;
    bool m_conveyorDirectionsVisible = false;
    const CadNode* m_selectionOutlineRoot = nullptr;
    const CadNode* m_placementPreviewRoot = nullptr;
    bool m_placementPreviewSnapped = false;
    std::vector<Vec3f> m_placementTargetGuides;
    std::vector<Vec3f> m_placementSourceGuides;
    std::vector<std::vector<Vec3f>> m_pathPreviewPoints;
    std::vector<std::vector<Vec3f>> m_simPathPreviewPoints;
    std::vector<ConveyorDirectionGuide> m_conveyorDirectionGuides;
    std::function<void(int, const CadTransform&)> m_gimbalMoveCallback;
    std::unordered_set<const CadNode*> m_collidingLinkNodes;
    std::unordered_set<const CadNode*> m_collidingGeometryNodes;
    std::unordered_map<const CadNode*, int> m_linkIndices;

    Mat4 m_projection;
    Mat4 m_view;
    Mat4 m_modelMatrix;

    // Raw GL handles. QOpenGLShaderProgram and QOpenGLBuffer both need a QOpenGLContext,
    // which does not exist now that GLFW owns the context.
    unsigned int m_program = 0;
    unsigned int m_vertexBuffer = 0;
    // The core profile rejects glDrawArrays with no vertex array bound, and the previous
    // Qt-hosted code never needed one.
    unsigned int m_vertexArray = 0;
    int m_mvpUniform = -1;
    int m_colorUniform = -1;
    int m_pointSizeUniform = -1;
    int m_roundPointUniform = -1;
    int m_positionAttribute = -1;
    bool m_graphicsReady = false;
    std::vector<float> m_scratchVertices;

    unsigned int m_framebuffer = 0;
    unsigned int m_colorTexture = 0;
    unsigned int m_depthBuffer = 0;
    int m_targetWidth = 0;
    int m_targetHeight = 0;

    // Viewport the current frame is being rendered at, used by projectPoint and the gimbal
    // maths in place of the old QWindow width()/height().
    int m_viewportWidth = 0;
    int m_viewportHeight = 0;

    // Static per-mesh vertex buffers, uploaded once and keyed by mesh identity. The AR4
    // package is 315,888 triangles; de-indexing and re-uploading it every frame was about
    // 11.4 MB of buffer traffic per frame. Positions are uploaded untransformed and the
    // object transform is applied through the model matrix instead.
    struct MeshBuffer {
        unsigned int buffer = 0;
        int vertexCount = 0;
        Vec3f localCenter;
        Vec3f localHalfExtent;
    };
    std::unordered_map<const MeshGeometryData*, std::unique_ptr<MeshBuffer>> m_meshBuffers;

    bool ensureRenderTarget(int width, int height);
    void releaseRenderTarget();
    void renderScene();
    void renderNode(const CadNode* node, const CadTransform& parentTransform, bool placementPreview = false);
    void renderSelectionOutline();
    void renderSelectionOutlineNode(const CadNode* node, const CadTransform& parentTransform,
                                    bool expanded);
    void renderSelectionOutlineMesh(const MeshGeometryData& mesh, const CadTransform& transform,
                                    bool expanded);
    void renderMesh(const MeshGeometryData& mesh, const CadTransform& transform, const CADNodeColor& color);
    void renderHull(const ConvexHullData& hull, const CadTransform& transform, const CADNodeColor& color);
    void renderPathPreview();
    void renderDragChainPivots();
    void renderMountingHoles();
    void renderConveyorDirections();
    void renderPathLine(const std::vector<Vec3f>& points, const CADNodeColor& color, float lineWidth);
    void renderGimbals();
    void renderGimbal(int index);
    // One draw call per batch of CPU-transformed vertices, for geometry that changes every
    // frame. Meshes go through drawMeshBuffer instead.
    void drawVertices(unsigned int mode, const std::vector<float>& vertices, const CADNodeColor& color, float pointSize = 1.0f);
    MeshBuffer* meshBufferFor(const MeshGeometryData& mesh);
    void drawMeshBuffer(const MeshBuffer& meshBuffer, const CADNodeColor& color);
    bool computeBounds(Vec3f& outMin, Vec3f& outMax) const;
    void rebuildCollisionNodeSets(const std::vector<int>& linkIndices, const CadNode* robotNode);
    GimbalHandle hitTestGimbals(const PointI& screenPos, int* outIndex) const;
    void dragGimbal(const PointI& screenPos);
    Vec3f gimbalOrigin(int index) const;
    float gimbalLength() const;
    float gimbalRingRadius() const;
    GimbalAxis gimbalHandleAxis(GimbalHandle handle) const;
    Vec3f gimbalAxisVector(GimbalAxis axis, int index) const;
    bool projectPoint(const Vec3f& point, PointF& outScreen) const;
};
