#include "MeshRobotViewer.h"

#include "SelectionStyle.h"

#include <algorithm>
#include <fstream>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace {
// ImGui redraws every frame; retain update() only for shared setter call sites.
inline void update() {}
} // namespace


namespace {

// Overall screen-space reduction for the complete tool widget, including its centre marker and
// line weights. Keep this separate from the original world-space base scale so those two concerns
// cannot drift apart when the widget is resized again.
constexpr float kGimbalVisualScale = 1.0f / 3.0f;
constexpr float kGimbalScale = (1.0f / 4.0f) * kGimbalVisualScale;

constexpr float kGimbalAxisScale = 5.0f;
constexpr float kGimbalRingScale = 3.0f;

// CadTransform is a row-major 3x4 rigid transform; Mat4 wants 4x4 with the
// translation in the last column.
Mat4 modelMatrixFor(const CadTransform& transform) {
    return Mat4(
        static_cast<float>(transform.values[0]), static_cast<float>(transform.values[1]),
        static_cast<float>(transform.values[2]), static_cast<float>(transform.values[3]),
        static_cast<float>(transform.values[4]), static_cast<float>(transform.values[5]),
        static_cast<float>(transform.values[6]), static_cast<float>(transform.values[7]),
        static_cast<float>(transform.values[8]), static_cast<float>(transform.values[9]),
        static_cast<float>(transform.values[10]), static_cast<float>(transform.values[11]),
        0.0f, 0.0f, 0.0f, 1.0f);
}

Vec3f transformPoint(const CadTransform& transform, float x, float y, float z) {
    const CadVec3 out = transform * CadVec3(x, y, z);
    return Vec3f(static_cast<float>(out.x), static_cast<float>(out.y),
                 static_cast<float>(out.z));
}

void includePoint(Vec3f& minPoint, Vec3f& maxPoint, const Vec3f& point) {
    minPoint.setX(std::min(minPoint.x(), point.x()));
    minPoint.setY(std::min(minPoint.y(), point.y()));
    minPoint.setZ(std::min(minPoint.z(), point.z()));
    maxPoint.setX(std::max(maxPoint.x(), point.x()));
    maxPoint.setY(std::max(maxPoint.y(), point.y()));
    maxPoint.setZ(std::max(maxPoint.z(), point.z()));
}

void collectRobotLinkNodes(const CadNode* node, std::vector<const CadNode*>& out) {
    if (!node) return;
    if (node->type == CadNodeType::RobotLink) out.push_back(node);
    for (const auto& child : node->children) collectRobotLinkNodes(child.get(), out);
}

double pointSegmentDistance(const PointF& point, const PointF& start, const PointF& end) {
    const PointF segment = end - start;
    const double lengthSquared = segment.x() * segment.x() + segment.y() * segment.y();
    if (lengthSquared <= 1.0e-9) {
        const PointF delta = point - start;
        return std::sqrt(delta.x() * delta.x() + delta.y() * delta.y());
    }
    const PointF relative = point - start;
    const double t = std::max(0.0, std::min(1.0, (relative.x() * segment.x() + relative.y() * segment.y()) / lengthSquared));
    const PointF closest = start + segment * t;
    const PointF delta = point - closest;
    return std::sqrt(delta.x() * delta.x() + delta.y() * delta.y());
}

CadTransform localAxisRotation(MeshRobotViewer::GimbalAxis axis, double angleRadians) {
    const double c = std::cos(angleRadians);
    const double s = std::sin(angleRadians);
    CadTransform rotation;
    switch (axis) {
    case MeshRobotViewer::GimbalAxis::X:
        rotation.values = {{1.0, 0.0, 0.0, 0.0,
                            0.0, c, -s, 0.0,
                            0.0, s, c, 0.0}};
        break;
    case MeshRobotViewer::GimbalAxis::Y:
        rotation.values = {{c, 0.0, s, 0.0,
                            0.0, 1.0, 0.0, 0.0,
                            -s, 0.0, c, 0.0}};
        break;
    case MeshRobotViewer::GimbalAxis::Z:
        rotation.values = {{c, -s, 0.0, 0.0,
                            s, c, 0.0, 0.0,
                            0.0, 0.0, 1.0, 0.0}};
        break;
    case MeshRobotViewer::GimbalAxis::None:
        break;
    }
    return rotation;
}

// One hue per link so it is obvious which hulls belong to which joint. Chosen to stay
// distinguishable against the dark viewport and from each other, and to avoid the collision
// red, which has to remain unambiguous. Alpha matches the previous single-colour overlay.
CADNodeColor hullColorForLink(int linkIndex) {
    static const CADNodeColor kLinkColors[] = {
        CADNodeColor(1.00f, 0.82f, 0.18f, 0.85f),  // amber, the original overlay colour
        CADNodeColor(0.36f, 0.78f, 1.00f, 0.85f),  // sky
        CADNodeColor(0.45f, 0.90f, 0.45f, 0.85f),  // green
        CADNodeColor(0.85f, 0.55f, 1.00f, 0.85f),  // violet
        CADNodeColor(1.00f, 0.58f, 0.30f, 0.85f),  // orange
        CADNodeColor(0.30f, 0.90f, 0.85f, 0.85f),  // teal
        CADNodeColor(0.95f, 0.75f, 0.85f, 0.85f)   // pink
    };
    constexpr int kColorCount = static_cast<int>(sizeof(kLinkColors) / sizeof(kLinkColors[0]));
    if (linkIndex < 0) return kLinkColors[0];
    return kLinkColors[linkIndex % kColorCount];
}

CADNodeColor gimbalAxisColor(MeshRobotViewer::GimbalAxis axis) {
    switch (axis) {
    case MeshRobotViewer::GimbalAxis::X:
        return CADNodeColor(1.0f, 0.12f, 0.08f, 1.0f);
    case MeshRobotViewer::GimbalAxis::Y:
        return CADNodeColor(0.1f, 0.85f, 0.2f, 1.0f);
    case MeshRobotViewer::GimbalAxis::Z:
        return CADNodeColor(0.16f, 0.45f, 1.0f, 1.0f);
    case MeshRobotViewer::GimbalAxis::None:
        break;
    }
    return CADNodeColor(1.0f, 1.0f, 1.0f, 1.0f);
}

} // namespace

MeshRobotViewer::MeshRobotViewer(CadNode* root)
    : m_root(root) {
    std::vector<const CadNode*> links;
    collectRobotLinkNodes(m_root, links);
    for (size_t i = 0; i < links.size(); ++i) {
        m_linkIndices.emplace(links[i], static_cast<int>(i));
    }
}

MeshRobotViewer::~MeshRobotViewer() {
    // Only safe while the context that created these is still current, which is the case
    // when the host destroys the renderer from inside the frame loop.
    for (auto& entry : m_meshBuffers) {
        if (entry.second && entry.second->buffer != 0) glDeleteBuffers(1, &entry.second->buffer);
    }
    m_meshBuffers.clear();
    releaseRenderTarget();
    if (m_vertexBuffer != 0) glDeleteBuffers(1, &m_vertexBuffer);
    if (m_vertexArray != 0) glDeleteVertexArrays(1, &m_vertexArray);
    if (m_program != 0) glDeleteProgram(m_program);
}

namespace {

// Compiles one stage and reports the log, which is the only useful diagnostic when a shader
// fails on a driver you cannot attach to.
unsigned int compileShaderStage(unsigned int type, const char* source, const char* label) {
    const unsigned int shader = glCreateShader(type);
    if (shader == 0) return 0;
    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);

    GLint compiled = 0;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
    if (compiled == 0) {
        GLint logLength = 0;
        glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &logLength);
        std::vector<char> log(static_cast<size_t>(logLength > 1 ? logLength : 1), '\0');
        glGetShaderInfoLog(shader, static_cast<GLsizei>(log.size()), nullptr, log.data());
        std::fprintf(stderr, "%s shader failed: %s\n", label, log.data());
        glDeleteShader(shader);
        return 0;
    }
    return shader;
}

// The version header differs per platform, so the shader bodies are shared and prefixed.
// A core profile removed attribute/varying/gl_FragColor, hence in/out and an explicit output.
#ifdef __EMSCRIPTEN__
constexpr const char* kShaderVersion = "#version 300 es\n";
#else
constexpr const char* kShaderVersion = "#version 330 core\n";
#endif

constexpr const char* kVertexBody =
    "in vec3 aPosition;\n"
    "uniform mat4 uMvp;\n"
    "uniform float uPointSize;\n"
    "void main() {\n"
    "    gl_Position = uMvp * vec4(aPosition, 1.0);\n"
    "    gl_PointSize = uPointSize;\n"
    "}\n";

// Flat colour, no lighting: the meshes carry no normals. GLES requires an explicit default
// float precision in fragment shaders.
constexpr const char* kFragmentBody =
    "#ifdef GL_ES\n"
    "precision mediump float;\n"
    "#endif\n"
    "uniform vec4 uColor;\n"
    "uniform bool uRoundPoint;\n"
    "out vec4 fragColor;\n"
    "void main() {\n"
    "    if (uRoundPoint && distance(gl_PointCoord, vec2(0.5)) > 0.5) discard;\n"
    "    fragColor = uColor;\n"
    "}\n";

} // namespace

bool MeshRobotViewer::initializeGraphics() {
    if (m_graphicsReady) return true;

    const std::string vertexSource = std::string(kShaderVersion) + kVertexBody;
    const std::string fragmentSource = std::string(kShaderVersion) + kFragmentBody;

    const unsigned int vertexShader = compileShaderStage(GL_VERTEX_SHADER, vertexSource.c_str(), "Scene vertex");
    if (vertexShader == 0) return false;
    const unsigned int fragmentShader = compileShaderStage(GL_FRAGMENT_SHADER, fragmentSource.c_str(), "Scene fragment");
    if (fragmentShader == 0) {
        glDeleteShader(vertexShader);
        return false;
    }

    m_program = glCreateProgram();
    glAttachShader(m_program, vertexShader);
    glAttachShader(m_program, fragmentShader);
    glLinkProgram(m_program);
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    GLint linked = 0;
    glGetProgramiv(m_program, GL_LINK_STATUS, &linked);
    if (linked == 0) {
        GLint logLength = 0;
        glGetProgramiv(m_program, GL_INFO_LOG_LENGTH, &logLength);
        std::vector<char> log(static_cast<size_t>(logLength > 1 ? logLength : 1), '\0');
        glGetProgramInfoLog(m_program, static_cast<GLsizei>(log.size()), nullptr, log.data());
        std::fprintf(stderr, "Scene shader link failed: %s\n", log.data());
        glDeleteProgram(m_program);
        m_program = 0;
        return false;
    }

    m_positionAttribute = glGetAttribLocation(m_program, "aPosition");
    m_mvpUniform = glGetUniformLocation(m_program, "uMvp");
    m_colorUniform = glGetUniformLocation(m_program, "uColor");
    m_pointSizeUniform = glGetUniformLocation(m_program, "uPointSize");
    m_roundPointUniform = glGetUniformLocation(m_program, "uRoundPoint");

    // A vertex array object is mandatory in a core profile: glDrawArrays with none bound is
    // an error. One shared VAO is enough because every draw uses the same single-attribute
    // layout and only the bound buffer changes.
    glGenVertexArrays(1, &m_vertexArray);
    glGenBuffers(1, &m_vertexBuffer);

    m_graphicsReady = m_positionAttribute >= 0 && m_vertexArray != 0 && m_vertexBuffer != 0;
    return m_graphicsReady;
}

bool MeshRobotViewer::ensureRenderTarget(int width, int height) {
    if (width <= 0 || height <= 0) return false;
    if (m_framebuffer != 0 && width == m_targetWidth && height == m_targetHeight) return true;

    releaseRenderTarget();

    glGenTextures(1, &m_colorTexture);
    glBindTexture(GL_TEXTURE_2D, m_colorTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glBindTexture(GL_TEXTURE_2D, 0);

    glGenRenderbuffers(1, &m_depthBuffer);
    glBindRenderbuffer(GL_RENDERBUFFER, m_depthBuffer);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, width, height);
    glBindRenderbuffer(GL_RENDERBUFFER, 0);

    glGenFramebuffers(1, &m_framebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, m_framebuffer);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_colorTexture, 0);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER,
                              m_depthBuffer);
    const bool complete = glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    if (!complete) {
        std::fprintf(stderr, "Scene framebuffer incomplete at %dx%d\n", width, height);
        releaseRenderTarget();
        return false;
    }

    m_targetWidth = width;
    m_targetHeight = height;
    return true;
}

void MeshRobotViewer::releaseRenderTarget() {
    if (m_framebuffer != 0) glDeleteFramebuffers(1, &m_framebuffer);
    if (m_depthBuffer != 0) glDeleteRenderbuffers(1, &m_depthBuffer);
    if (m_colorTexture != 0) glDeleteTextures(1, &m_colorTexture);
    m_framebuffer = 0;
    m_depthBuffer = 0;
    m_colorTexture = 0;
    m_targetWidth = 0;
    m_targetHeight = 0;
}

unsigned int MeshRobotViewer::render(int width, int height) {
    if (!initializeGraphics()) return 0;
    if (!ensureRenderTarget(width, height)) return 0;

    m_viewportWidth = width;
    m_viewportHeight = height;

    // ImGui leaves its own framebuffer bound, so this must save nothing and simply restore
    // the default binding on the way out.
    glBindFramebuffer(GL_FRAMEBUFFER, m_framebuffer);
    glViewport(0, 0, width, height);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glCullFace(GL_BACK);
    glClearColor(0.08f, 0.09f, 0.10f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

    glBindVertexArray(m_vertexArray);
    renderScene();
    glBindVertexArray(0);

    // ImGui's own draw pass expects these off; leaving them on blanks the UI.
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glDisable(GL_BLEND);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    return m_colorTexture;
}

bool MeshRobotViewer::renderToPpm(const std::string& path, int width, int height) {
    if (render(width, height) == 0 || width <= 0 || height <= 0) return false;
    std::vector<unsigned char> pixels(static_cast<size_t>(width) * static_cast<size_t>(height) * 3);
    glBindFramebuffer(GL_FRAMEBUFFER, m_framebuffer);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    std::ofstream output(path, std::ios::binary);
    if (!output) return false;
    output << "P6\n" << width << " " << height << "\n255\n";
    const size_t rowBytes = static_cast<size_t>(width) * 3;
    for (int row = height - 1; row >= 0; --row) {
        output.write(reinterpret_cast<const char*>(pixels.data() + static_cast<size_t>(row) * rowBytes),
                     static_cast<std::streamsize>(rowBytes));
    }
    return output.good();
}

void MeshRobotViewer::renderScene() {
    // Reproduce the legacy camera using owned matrices.
    const float aspect = m_viewportHeight > 0
        ? static_cast<float>(m_viewportWidth) / static_cast<float>(m_viewportHeight)
        : 1.0f;
    m_projection.setToIdentity();
    m_projection.perspective(45.0f, aspect, 1.0f, 100000.0f);

    m_view.setToIdentity();
    m_view.translate(0.0f, 0.0f, -m_distance);
    m_view.rotate(m_pitch, 1.0f, 0.0f, 0.0f);
    m_view.rotate(m_yaw, 0.0f, 1.0f, 0.0f);
    m_view.translate(-m_center.x(), -m_center.y(), -m_center.z());

    CadTransform identity;
    renderNode(m_root, identity);
    renderSelectionOutline();
    renderPathPreview();
    renderDragChainPivots();
    renderMountingHoles();
    renderConveyorDirections();
    renderGimbals();
}

void MeshRobotViewer::drawVertices(unsigned int mode,
                                   const std::vector<float>& vertices,
                                   const CADNodeColor& color,
                                   float pointSize) {
    if (vertices.empty() || !m_graphicsReady) return;

    glUseProgram(m_program);
    glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer);
    glBufferData(GL_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(vertices.size() * sizeof(float)),
                 vertices.data(),
                 GL_DYNAMIC_DRAW);

    const Mat4 mvp = m_projection * m_view * m_modelMatrix;
    const float colorValues[4] = {color.r, color.g, color.b, color.a};
    glUniformMatrix4fv(m_mvpUniform, 1, GL_FALSE, mvp.constData());
    glUniform4fv(m_colorUniform, 1, colorValues);
    glUniform1f(m_pointSizeUniform, pointSize);
    glUniform1i(m_roundPointUniform, mode == GL_POINTS ? 1 : 0);

    const unsigned int attribute = static_cast<unsigned int>(m_positionAttribute);
    glEnableVertexAttribArray(attribute);
    glVertexAttribPointer(attribute, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
    glDrawArrays(mode, 0, static_cast<GLsizei>(vertices.size() / 3));
    glDisableVertexAttribArray(attribute);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glUseProgram(0);
}

MeshRobotViewer::MeshBuffer* MeshRobotViewer::meshBufferFor(const MeshGeometryData& mesh) {
    const auto existing = m_meshBuffers.find(&mesh);
    if (existing != m_meshBuffers.end()) return existing->second.get();

    // Upload once, untransformed. The meshes are already fully de-indexed on disk
    // (vertexCount == indexCount for every link), but the index list is still walked here so
    // that any indexed mesh keeps working.
    std::vector<float> positions;
    positions.reserve(mesh.indices.size() * 3);
    for (uint32_t index : mesh.indices) {
        const size_t offset = static_cast<size_t>(index) * 3;
        if (offset + 2 >= mesh.vertices.size()) continue;
        positions.push_back(mesh.vertices[offset]);
        positions.push_back(mesh.vertices[offset + 1]);
        positions.push_back(mesh.vertices[offset + 2]);
    }
    if (positions.empty()) return nullptr;

    auto meshBuffer = std::make_unique<MeshBuffer>();
    glGenBuffers(1, &meshBuffer->buffer);
    if (meshBuffer->buffer == 0) return nullptr;
    glBindBuffer(GL_ARRAY_BUFFER, meshBuffer->buffer);
    glBufferData(GL_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(positions.size() * sizeof(float)),
                 positions.data(),
                 GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    meshBuffer->vertexCount = static_cast<int>(positions.size() / 3);

    Vec3f localMin(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
                   std::numeric_limits<float>::max());
    Vec3f localMax(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(),
                   std::numeric_limits<float>::lowest());
    for (size_t i = 0; i + 2 < positions.size(); i += 3) {
        includePoint(localMin, localMax, Vec3f(positions[i], positions[i + 1], positions[i + 2]));
    }
    meshBuffer->localCenter = (localMin + localMax) * 0.5f;
    meshBuffer->localHalfExtent = (localMax - localMin) * 0.5f;

    MeshBuffer* raw = meshBuffer.get();
    m_meshBuffers.emplace(&mesh, std::move(meshBuffer));
    return raw;
}

void MeshRobotViewer::renderSelectionOutlineMesh(const MeshGeometryData& mesh,
                                                 const CadTransform& transform, bool expanded) {
    if (!mesh.loaded || mesh.vertices.empty() || mesh.indices.empty()) return;
    MeshBuffer* meshBuffer = meshBufferFor(mesh);
    if (!meshBuffer) return;

    if (!expanded) {
        m_modelMatrix = modelMatrixFor(transform);
        drawMeshBuffer(*meshBuffer, CADNodeColor(1.0f, 1.0f, 1.0f, 1.0f));
        m_modelMatrix.setToIdentity();
        return;
    }

    // Shared with the RoboDK plugin, which draws the same selection over conveyors it owns: the colour and
    // the standoff are `Common/SelectionStyle`, so a selection reads as the same thing in both cells.
    const float worldThickness =
        static_cast<float>(selectionstyle::outlineThicknessMm(static_cast<double>(m_distance)));
    const auto expandedScale = [worldThickness](float halfExtent) {
        return static_cast<float>(selectionstyle::outlineExpansion(static_cast<double>(halfExtent),
                                                                   static_cast<double>(worldThickness)));
    };
    const float sx = expandedScale(meshBuffer->localHalfExtent.x());
    const float sy = expandedScale(meshBuffer->localHalfExtent.y());
    const float sz = expandedScale(meshBuffer->localHalfExtent.z());
    const Vec3f& center = meshBuffer->localCenter;
    Mat4 expansion;
    expansion.set(0, 0, sx);
    expansion.set(1, 1, sy);
    expansion.set(2, 2, sz);
    expansion.set(0, 3, center.x() * (1.0f - sx));
    expansion.set(1, 3, center.y() * (1.0f - sy));
    expansion.set(2, 3, center.z() * (1.0f - sz));

    m_modelMatrix = modelMatrixFor(transform) * expansion;
    drawMeshBuffer(*meshBuffer, selectionstyle::outlineColor());
    m_modelMatrix.setToIdentity();
}

void MeshRobotViewer::drawMeshBuffer(const MeshBuffer& meshBuffer, const CADNodeColor& color) {
    if (meshBuffer.vertexCount <= 0 || !m_graphicsReady) return;

    glUseProgram(m_program);
    glBindBuffer(GL_ARRAY_BUFFER, meshBuffer.buffer);

    const Mat4 mvp = m_projection * m_view * m_modelMatrix;
    const float colorValues[4] = {color.r, color.g, color.b, color.a};
    glUniformMatrix4fv(m_mvpUniform, 1, GL_FALSE, mvp.constData());
    glUniform4fv(m_colorUniform, 1, colorValues);
    glUniform1f(m_pointSizeUniform, 1.0f);
    glUniform1i(m_roundPointUniform, 0);

    const unsigned int attribute = static_cast<unsigned int>(m_positionAttribute);
    glEnableVertexAttribArray(attribute);
    glVertexAttribPointer(attribute, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
    // Imported CAD commonly contains thin sheet faces and occasionally inconsistent triangle
    // winding. Treat visual meshes as two-sided instead of requiring every importer to duplicate
    // triangles; collision remains driven by its separately baked hulls.
    glDisable(GL_CULL_FACE);
    glDrawArrays(GL_TRIANGLES, 0, meshBuffer.vertexCount);
    glEnable(GL_CULL_FACE);
    glDisableVertexAttribArray(attribute);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glUseProgram(0);
}
bool MeshRobotViewer::onPointerPressed(const PointI& localPos) {
    m_lastMousePos = localPos;
    m_activeGimbalIndex = -1;
    m_activeGimbalHandle = hitTestGimbals(localPos, &m_activeGimbalIndex);
    m_draggingGimbal = m_activeGimbalHandle != GimbalHandle::None && m_activeGimbalIndex >= 0;
    m_rotating = !m_draggingGimbal;
    return m_draggingGimbal;
}

void MeshRobotViewer::onPointerMoved(const PointI& localPos, bool pressed) {
    if (!pressed) return;
    if (m_draggingGimbal) {
        dragGimbal(localPos);
        m_lastMousePos = localPos;
        return;
    }
    if (!m_rotating) return;
    const PointI delta = localPos - m_lastMousePos;
    m_lastMousePos = localPos;
    m_yaw += static_cast<float>(delta.x()) * 0.35f;
    m_pitch += static_cast<float>(delta.y()) * 0.35f;
    m_pitch = std::max(-89.0f, std::min(89.0f, m_pitch));
}

void MeshRobotViewer::onPointerReleased() {
    m_rotating = false;
    m_draggingGimbal = false;
    m_activeGimbalHandle = GimbalHandle::None;
    m_activeGimbalIndex = -1;
}

void MeshRobotViewer::onScroll(double steps) {
    // Preserve the established zoom response per wheel notch.
    m_distance *= static_cast<float>(std::pow(0.88, steps));
    m_distance = std::max(10.0f, m_distance);
}

void MeshRobotViewer::onPanPressed(const PointI& localPos) {
    m_lastPanPos = localPos;
    m_panning = true;
}

void MeshRobotViewer::onPanMoved(const PointI& localPos) {
    if (!m_panning || m_viewportHeight <= 0) return;
    const PointI delta = localPos - m_lastPanPos;
    m_lastPanPos = localPos;

    // World units per pixel at the orbit centre, from the same 45 degree vertical field of view the
    // projection uses. Scaling by distance is what keeps a drag feeling the same close in and far
    // out, instead of crawling when zoomed out over a whole cell.
    constexpr float kPi = 3.14159265358979323846f;
    constexpr float kHalfFovTangent = 0.41421356f;  // tan(45 deg / 2)
    const float worldPerPixel =
        2.0f * m_distance * kHalfFovTangent / static_cast<float>(m_viewportHeight);

    const float yaw = m_yaw * kPi / 180.0f;
    const float pitch = m_pitch * kPi / 180.0f;
    const float cy = std::cos(yaw), sy = std::sin(yaw);
    const float cp = std::cos(pitch), sp = std::sin(pitch);
    const Vec3f right(cy, 0.0f, sy);
    const Vec3f up(sp * sy, cp, -sp * cy);

    m_center = m_center - right * (static_cast<float>(delta.x()) * worldPerPixel) +
               up * (static_cast<float>(delta.y()) * worldPerPixel);
}

void MeshRobotViewer::onPanReleased() {
    m_panning = false;
}

const CadNode* MeshRobotViewer::pickNodeAt(const PointI& localPos,
                                           const std::vector<CadNode*>& candidates) const {
    const CadNode* best = nullptr;
    float bestDepth = std::numeric_limits<float>::max();

    for (CadNode* candidate : candidates) {
        Vec3f minPoint;
        Vec3f maxPoint;
        if (!computeBoundsOf(candidate, minPoint, maxPoint)) continue;

        // The projected screen rectangle of the bounding box, not a ray against the meshes. The
        // question is which arm was clicked, not which triangle, and at three hundred thousand
        // triangles an arm the better answer is not worth what it costs to get.
        double left = std::numeric_limits<double>::max();
        double top = std::numeric_limits<double>::max();
        double right = std::numeric_limits<double>::lowest();
        double bottom = std::numeric_limits<double>::lowest();
        bool projected = false;
        for (int corner = 0; corner < 8; ++corner) {
            const Vec3f point((corner & 1) ? maxPoint.x() : minPoint.x(),
                              (corner & 2) ? maxPoint.y() : minPoint.y(),
                              (corner & 4) ? maxPoint.z() : minPoint.z());
            PointF screen;
            if (!projectPoint(point, screen)) continue;
            left = std::min(left, screen.x());
            right = std::max(right, screen.x());
            top = std::min(top, screen.y());
            bottom = std::max(bottom, screen.y());
            projected = true;
        }
        if (!projected) continue;

        const double x = static_cast<double>(localPos.x());
        const double y = static_cast<double>(localPos.y());
        if (x < left || x > right || y < top || y > bottom) continue;

        const Vec3f centre = (minPoint + maxPoint) * 0.5f;
        const Vec4f viewPoint = m_view * Vec4f(centre, 1.0f);
        const float depth = -viewPoint.z();
        if (depth < bestDepth) {
            bestDepth = depth;
            best = candidate;
        }
    }
    return best;
}
MeshRobotViewer::CameraState MeshRobotViewer::camera() const {
    CameraState state;
    state.center = m_center;
    state.distance = m_distance;
    state.yaw = m_yaw;
    state.pitch = m_pitch;
    state.framed = true;
    return state;
}

void MeshRobotViewer::setCamera(const CameraState& state) {
    m_center = state.center;
    m_distance = std::max(10.0f, state.distance);
    m_yaw = state.yaw;
    m_pitch = std::max(-89.0f, std::min(89.0f, state.pitch));
    update();
}

void MeshRobotViewer::reframeCamera() {
    Vec3f minPoint;
    Vec3f maxPoint;
    if (!computeBounds(minPoint, maxPoint)) return;

    m_center = (minPoint + maxPoint) * 0.5f;
    const float radius = std::max(10.0f, (maxPoint - minPoint).length() * 0.5f);
    m_distance = radius * 2.8f;
    update();
}

void MeshRobotViewer::reframeCameraOn(const CadNode* node) {
    Vec3f minPoint;
    Vec3f maxPoint;
    if (!computeBoundsOf(node, minPoint, maxPoint)) {
        reframeCamera();
        return;
    }

    m_center = (minPoint + maxPoint) * 0.5f;
    const float radius = std::max(10.0f, (maxPoint - minPoint).length() * 0.5f);
    m_distance = radius * 2.8f;
    update();
}

void MeshRobotViewer::markCacheDirty() {
    // A library-placement preview owns a temporary package tree.  Once a drag is cancelled those
    // MeshGeometryData addresses are invalid, and an allocator is free to reuse one for the next
    // preview.  Keeping the old VBO under that pointer would then draw the previous robot.  Drop
    // the small per-package cache whenever the scene graph changes; ordinary animation never calls
    // this path, so it does not turn moving the robot into repeated uploads.
    for (auto& entry : m_meshBuffers) {
        if (entry.second && entry.second->buffer != 0) {
            glDeleteBuffers(1, &entry.second->buffer);
        }
    }
    m_meshBuffers.clear();
    m_linkIndices.clear();
    std::vector<const CadNode*> links;
    collectRobotLinkNodes(m_root, links);
    for (size_t i = 0; i < links.size(); ++i) {
        m_linkIndices.emplace(links[i], static_cast<int>(i));
    }
    update();
}

void MeshRobotViewer::setCollidingLinkIndices(const std::vector<int>& linkIndices, const CadNode* robotNode) {
    rebuildCollisionNodeSets(linkIndices, robotNode);
    update();
}

void MeshRobotViewer::setGimbalPoses(const std::vector<CadTransform>& poses) {
    // Not while a drag is in flight. The host pushes these every frame from the scene, and the scene
    // is being changed by the very drag in progress - taking the new list mid-drag would replace the
    // pose the drag is accumulating into with the one it has already produced, and on a shrinking
    // list could invalidate the index being dragged.
    if (m_draggingGimbal) return;
    m_gimbalPoses = poses;
    update();
}

void MeshRobotViewer::setGimbalsVisible(bool visible) {
    m_gimbalsVisible = visible;
    update();
}

void MeshRobotViewer::setGimbalMoveCallback(std::function<void(int, const CadTransform&)> callback) {
    m_gimbalMoveCallback = std::move(callback);
}

void MeshRobotViewer::setPathPreviewPoints(const std::vector<std::vector<Vec3f>>& paths) {
    m_pathPreviewPoints = paths;
    update();
}

void MeshRobotViewer::setPathPreviewVisible(bool visible) {
    m_pathPreviewVisible = visible;
    update();
}

void MeshRobotViewer::setSimPathPreviewPoints(const std::vector<std::vector<Vec3f>>& paths) {
    m_simPathPreviewPoints = paths;
    update();
}

void MeshRobotViewer::setSimPathPreviewVisible(bool visible) {
    m_simPathPreviewVisible = visible;
    update();
}

void MeshRobotViewer::setDragChainPivotsVisible(bool visible) {
    m_dragChainPivotsVisible = visible;
    update();
}

void MeshRobotViewer::setGantryHullOverlaysVisible(bool visible) {
    m_gantryHullOverlaysVisible = visible;
    update();
}

void MeshRobotViewer::setMountingHolesVisible(bool visible) {
    m_mountingHolesVisible = visible;
    update();
}

void MeshRobotViewer::setConveyorDirectionGuides(
    const std::vector<ConveyorDirectionGuide>& guides) {
    m_conveyorDirectionGuides = guides;
    update();
}

void MeshRobotViewer::setConveyorDirectionsVisible(bool visible) {
    m_conveyorDirectionsVisible = visible;
    update();
}

void MeshRobotViewer::setPlacementMountingGuides(const std::vector<Vec3f>& targetPoints,
                                                 const std::vector<Vec3f>& sourcePoints) {
    m_placementTargetGuides = targetPoints;
    m_placementSourceGuides = sourcePoints;
    update();
}

void MeshRobotViewer::setSelectionOutlineRoot(const CadNode* root) {
    m_selectionOutlineRoot = root;
    update();
}

void MeshRobotViewer::setPlacementPreviewRoot(const CadNode* root, bool snapped) {
    m_placementPreviewRoot = root;
    m_placementPreviewSnapped = snapped;
    if (!root) {
        m_placementTargetGuides.clear();
        m_placementSourceGuides.clear();
    }
    update();
}

void MeshRobotViewer::renderNode(const CadNode* node, const CadTransform& parentTransform,
                                 bool placementPreview) {
    if (!node) return;
    const CadTransform worldTransform = parentTransform * node->loc;
    if (!node->visible) return;

    placementPreview = placementPreview || node == m_placementPreviewRoot;

    const CADNodeColor color = placementPreview
        ? (m_placementPreviewSnapped ? CADNodeColor(0.20f, 1.0f, 0.45f, 0.42f)
                                     : CADNodeColor(1.0f, 0.58f, 0.12f, 0.38f))
        : node->color;
    if (node->type == CadNodeType::MeshGeometry) {
        if (const MeshGeometryData* mesh = node->asMeshGeometry()) {
            const bool colliding = m_collidingGeometryNodes.find(node) != m_collidingGeometryNodes.end();
            renderMesh(*mesh, worldTransform, colliding ? CADNodeColor(1.0f, 0.05f, 0.03f, 1.0f) : color);
        }
    } else if (node->type == CadNodeType::DragChainMechanism) {
        if (const DragChainMechanismData* chain = node->asDragChainMechanism()) {
            const MeshGeometryData* prototype = chain->prototypeGeometry
                ? chain->prototypeGeometry->asMeshGeometry() : nullptr;
            if (prototype) {
                const CADNodeColor instanceColor = chain->prototypeGeometry->color;
                for (const CadNode* frame : chain->linkFrames) {
                    if (frame && frame->visible) renderMesh(*prototype, worldTransform * frame->loc, instanceColor);
                }
            }
        }
    } else if (node->type == CadNodeType::RobotLink) {
        if (const RobotLinkData* link = node->asRobotLink()) {
            if (link->collisionHullsVisible) {
                // Collision still wins over the per-link hue, so a contact stays unmistakable.
                const bool colliding = m_collidingLinkNodes.find(node) != m_collidingLinkNodes.end();
                const auto indexEntry = m_linkIndices.find(node);
                const int linkIndex = indexEntry != m_linkIndices.end() ? indexEntry->second : -1;
                const CADNodeColor hullColor = colliding
                    ? CADNodeColor(1.0f, 0.05f, 0.03f, 1.0f)
                    : hullColorForLink(linkIndex);
                for (const ConvexHullData& hull : link->collisionHulls) renderHull(hull, worldTransform, hullColor);
            }
        }
    } else if (node->type == CadNodeType::RobotTool) {
        if (const RobotToolData* tool = node->asRobotTool()) {
            if (tool->collisionHullsVisible) {
                for (const ConvexHullData& hull : tool->collisionHulls) renderHull(hull, worldTransform, CADNodeColor(1.0f, 0.82f, 0.18f, 0.85f));
            }
        }
    }

    for (const auto& child : node->children) {
        renderNode(child.get(), worldTransform, placementPreview);
    }
    if (node->type == CadNodeType::GantryMechanism && m_gantryHullOverlaysVisible) {
        if (const GantryMechanismData* gantry = node->asGantryMechanism()) {
            // Draw after the visual subtree and through it: this is a diagnostic overlay, and hulls
            // baked close to the source surface would otherwise z-fight or disappear behind it.
            glDisable(GL_DEPTH_TEST);
            const CADNodeColor hullColor(0.12f, 1.0f, 0.38f, 0.95f);
            for (const ConvexHullData& hull : gantry->baseCollisionHulls) {
                renderHull(hull, worldTransform, hullColor);
            }
            glEnable(GL_DEPTH_TEST);
        }
    }
}

void MeshRobotViewer::renderSelectionOutline() {
    if (!m_selectionOutlineRoot) return;

    // The regular scene has already populated depth. First stamp the selected robot's visible
    // pixels into stencil without touching colour or depth. The expanded pass is then allowed only
    // outside that exact mask, producing a silhouette even for open/non-manifold CAD meshes that
    // cannot use the usual back-face-only inverted hull trick.
    glEnable(GL_STENCIL_TEST);
    glClearStencil(0);
    glClear(GL_STENCIL_BUFFER_BIT);
    glStencilMask(0xFF);
    glStencilFunc(GL_ALWAYS, 1, 0xFF);
    glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
    glDepthMask(GL_FALSE);
    glDepthFunc(GL_LEQUAL);

    CadTransform toParent;
    for (const CadNode* ancestor = m_selectionOutlineRoot->parent; ancestor != nullptr;
         ancestor = ancestor->parent) {
        toParent = ancestor->loc * toParent;
    }
    renderSelectionOutlineNode(m_selectionOutlineRoot, toParent, false);

    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glStencilMask(0x00);
    glStencilFunc(GL_NOTEQUAL, 1, 0xFF);
    glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
    glDisable(GL_BLEND);
    renderSelectionOutlineNode(m_selectionOutlineRoot, toParent, true);
    glEnable(GL_BLEND);
    glDepthMask(GL_TRUE);
    glDepthFunc(GL_LESS);
    glStencilMask(0xFF);
    glDisable(GL_STENCIL_TEST);
}

void MeshRobotViewer::renderSelectionOutlineNode(const CadNode* node,
                                                 const CadTransform& parentTransform,
                                                 bool expanded) {
    if (!node || !node->visible) return;
    // A robot mounted to a gantry is a child for motion propagation, but it is not part of the
    // rail the operator selected. Stop at nested robot roots while still traversing the robot when
    // the robot itself is the selected outline root.
    if (node != m_selectionOutlineRoot && node->type == CadNodeType::OPW6Robot) return;
    const CadTransform worldTransform = parentTransform * node->loc;
    if (node->type == CadNodeType::MeshGeometry) {
        if (const MeshGeometryData* mesh = node->asMeshGeometry()) {
            renderSelectionOutlineMesh(*mesh, worldTransform, expanded);
        }
    } else if (node->type == CadNodeType::DragChainMechanism) {
        const DragChainMechanismData* chain = node->asDragChainMechanism();
        const MeshGeometryData* prototype = chain && chain->prototypeGeometry
            ? chain->prototypeGeometry->asMeshGeometry() : nullptr;
        if (prototype) {
            for (const CadNode* frame : chain->linkFrames) {
                if (frame && frame->visible) {
                    renderSelectionOutlineMesh(*prototype, worldTransform * frame->loc, expanded);
                }
            }
        }
    }
    for (const auto& child : node->children) {
        renderSelectionOutlineNode(child.get(), worldTransform, expanded);
    }
}

void MeshRobotViewer::renderPathPreview() {
    const bool hasProgramPath = m_pathPreviewVisible && !m_pathPreviewPoints.empty();
    const bool hasSimPath = m_simPathPreviewVisible && !m_simPathPreviewPoints.empty();
    if (!hasProgramPath && !hasSimPath) return;

    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    if (hasProgramPath) {
        for (const std::vector<Vec3f>& path : m_pathPreviewPoints) {
            renderPathLine(path, CADNodeColor(0.0f, 0.78f, 1.0f, 1.0f), 2.0f);
        }
    }
    if (hasSimPath) {
        for (const std::vector<Vec3f>& path : m_simPathPreviewPoints) {
            renderPathLine(path, CADNodeColor(1.0f, 0.38f, 0.08f, 1.0f), 3.0f);
        }
    }
    glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);
}

void MeshRobotViewer::renderDragChainPivots() {
    if (!m_dragChainPivotsVisible || !m_root) return;

    std::vector<Vec3f> movingPivots;
    std::vector<Vec3f> internalPivots;
    std::vector<Vec3f> fixedPivots;
    std::function<void(const CadNode*, const CadTransform&)> visit =
        [&](const CadNode* node, const CadTransform& parentTransform) {
            if (!node || !node->visible) return;
            const CadTransform worldTransform = parentTransform * node->loc;
            if (const DragChainMechanismData* chain = node->asDragChainMechanism()) {
                if (!chain->linkFrames.empty() && chain->linkFrames.front()) {
                    movingPivots.push_back(transformPoint(
                        worldTransform * chain->linkFrames.front()->loc, 0.0f, 0.0f, 0.0f));
                    for (size_t index = 0; index + 1 < chain->linkFrames.size(); ++index) {
                        if (!chain->linkFrames[index]) continue;
                        internalPivots.push_back(transformPoint(
                            worldTransform * chain->linkFrames[index]->loc,
                            static_cast<float>(chain->pitchMm), 0.0f, 0.0f));
                    }
                }
                fixedPivots.push_back(transformPoint(
                    worldTransform,
                    static_cast<float>(chain->fixedAnchorMm.x),
                    static_cast<float>(chain->fixedAnchorMm.y),
                    static_cast<float>(chain->fixedAnchorMm.z)));
            }
            for (const auto& child : node->children) visit(child.get(), worldTransform);
        };
    CadTransform identity;
    visit(m_root, identity);

    const auto verticesFor = [](const std::vector<Vec3f>& points) {
        std::vector<float> vertices;
        vertices.reserve(points.size() * 3);
        for (const Vec3f& point : points) {
            vertices.push_back(point.x());
            vertices.push_back(point.y());
            vertices.push_back(point.z());
        }
        return vertices;
    };
    const auto crossesFor = [](const std::vector<Vec3f>& points, float radius) {
        std::vector<float> vertices;
        vertices.reserve(points.size() * 18);
        for (const Vec3f& point : points) {
            const float x = point.x();
            const float y = point.y();
            const float z = point.z();
            vertices.insert(vertices.end(), {
                x - radius, y, z, x + radius, y, z,
                x, y - radius, z, x, y + radius, z,
                x, y, z - radius, x, y, z + radius});
        }
        return vertices;
    };
    std::vector<Vec3f> allPivots = internalPivots;
    allPivots.insert(allPivots.end(), movingPivots.begin(), movingPivots.end());
    allPivots.insert(allPivots.end(), fixedPivots.begin(), fixedPivots.end());

    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glLineWidth(6.0f);
    drawVertices(GL_LINES, crossesFor(allPivots, 13.0f),
                 CADNodeColor(0.02f, 0.02f, 0.02f, 1.0f));
    glLineWidth(3.0f);
    drawVertices(GL_LINES, crossesFor(internalPivots, 11.0f),
                 CADNodeColor(1.0f, 0.76f, 0.05f, 1.0f));
    drawVertices(GL_LINES, crossesFor(movingPivots, 11.0f),
                 CADNodeColor(0.0f, 0.85f, 1.0f, 1.0f));
    drawVertices(GL_LINES, crossesFor(fixedPivots, 11.0f),
                 CADNodeColor(1.0f, 0.12f, 0.82f, 1.0f));
    glLineWidth(1.0f);
    drawVertices(GL_POINTS, verticesFor(allPivots), CADNodeColor(0.02f, 0.02f, 0.02f, 1.0f), 15.0f);
    drawVertices(GL_POINTS, verticesFor(internalPivots), CADNodeColor(1.0f, 0.76f, 0.05f, 1.0f), 9.0f);
    drawVertices(GL_POINTS, verticesFor(movingPivots), CADNodeColor(0.0f, 0.85f, 1.0f, 1.0f), 11.0f);
    drawVertices(GL_POINTS, verticesFor(fixedPivots), CADNodeColor(1.0f, 0.12f, 0.82f, 1.0f), 11.0f);
    glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);
}

void MeshRobotViewer::renderMountingHoles() {
    if ((!m_mountingHolesVisible && m_placementTargetGuides.empty() &&
         m_placementSourceGuides.empty()) || !m_root) return;

    // A serialized grid can represent a very large fixture plate. Keep malformed or adversarial
    // data from turning one diagnostic frame into an unbounded allocation while retaining ample
    // room for real industrial tables.
    constexpr size_t kMaximumRenderedCenters = 250000;
    std::vector<Vec3f> explicitCenters;
    std::vector<Vec3f> gridCenters;
    std::function<void(const CadNode*, const CadTransform&)> visit =
        [&](const CadNode* node, const CadTransform& parentTransform) {
            if (!node || !node->visible ||
                explicitCenters.size() + gridCenters.size() >= kMaximumRenderedCenters) {
                return;
            }
            const CadTransform worldTransform = parentTransform * node->loc;
            const CadVec3& markerOffset = node->mountingHoles.markerOffsetMm;
            for (const CadVec3& point : node->mountingHoles.pointsMm) {
                if (explicitCenters.size() + gridCenters.size() >= kMaximumRenderedCenters) break;
                explicitCenters.push_back(transformPoint(
                    worldTransform, static_cast<float>(point.x + markerOffset.x),
                    static_cast<float>(point.y + markerOffset.y),
                    static_cast<float>(point.z + markerOffset.z)));
            }
            for (const MountingHoleGridData& grid : node->mountingHoles.grids) {
                for (uint32_t v = 0; v < grid.vCount; ++v) {
                    for (uint32_t u = 0; u < grid.uCount; ++u) {
                        if (explicitCenters.size() + gridCenters.size() >=
                            kMaximumRenderedCenters) break;
                        const double x = grid.originMm.x + u * grid.uStepMm.x +
                                         v * grid.vStepMm.x;
                        const double y = grid.originMm.y + u * grid.uStepMm.y +
                                         v * grid.vStepMm.y;
                        const double z = grid.originMm.z + u * grid.uStepMm.z +
                                         v * grid.vStepMm.z;
                        gridCenters.push_back(transformPoint(
                            worldTransform, static_cast<float>(x + markerOffset.x),
                            static_cast<float>(y + markerOffset.y),
                            static_cast<float>(z + markerOffset.z)));
                    }
                    if (explicitCenters.size() + gridCenters.size() >=
                        kMaximumRenderedCenters) break;
                }
            }
            for (const auto& child : node->children) visit(child.get(), worldTransform);
        };
    if (m_mountingHolesVisible) {
        CadTransform identity;
        visit(m_root, identity);
    }
    if (explicitCenters.empty() && gridCenters.empty() && m_placementTargetGuides.empty() &&
        m_placementSourceGuides.empty()) return;

    const auto verticesFor = [](const std::vector<Vec3f>& points) {
        std::vector<float> vertices;
        vertices.reserve(points.size() * 3);
        for (const Vec3f& point : points) {
            vertices.insert(vertices.end(), {point.x(), point.y(), point.z()});
        }
        return vertices;
    };
    // Draw hole markers camera-facing at a fixed pixel radius.
    Mat4 inverseViewRotation;
    inverseViewRotation.setToIdentity();
    inverseViewRotation.rotate(-m_yaw, 0.0f, 1.0f, 0.0f);
    inverseViewRotation.rotate(-m_pitch, 1.0f, 0.0f, 0.0f);
    const Vec4f right4 = inverseViewRotation * Vec4f(1.0f, 0.0f, 0.0f, 0.0f);
    const Vec4f up4 = inverseViewRotation * Vec4f(0.0f, 1.0f, 0.0f, 0.0f);
    const Vec3f cameraRight(right4.x(), right4.y(), right4.z());
    const Vec3f cameraUp(up4.x(), up4.y(), up4.z());
    const auto crossesFor = [&](const std::vector<Vec3f>& points, float radiusPixels) {
        std::vector<float> vertices;
        vertices.reserve(points.size() * 12);
        constexpr float kTanHalfFov = 0.4142135623730950f;
        const float viewportHeight = static_cast<float>(std::max(1, m_viewportHeight));
        for (const Vec3f& point : points) {
            const Vec4f viewPoint = m_view * Vec4f(point, 1.0f);
            const float depth = -viewPoint.z();
            if (depth <= 1.0f) continue;
            const float worldRadius = radiusPixels *
                (2.0f * depth * kTanHalfFov / viewportHeight);
            const Vec3f horizontal = cameraRight * worldRadius;
            const Vec3f vertical = cameraUp * worldRadius;
            const Vec3f left = point - horizontal;
            const Vec3f right = point + horizontal;
            const Vec3f down = point - vertical;
            const Vec3f up = point + vertical;
            vertices.insert(vertices.end(), {
                left.x(), left.y(), left.z(), right.x(), right.y(), right.z(),
                down.x(), down.y(), down.z(), up.x(), up.y(), up.z()});
        }
        return vertices;
    };

    std::vector<Vec3f> allCenters = gridCenters;
    allCenters.insert(allCenters.end(), explicitCenters.begin(), explicitCenters.end());
    allCenters.insert(allCenters.end(), m_placementTargetGuides.begin(),
                      m_placementTargetGuides.end());
    allCenters.insert(allCenters.end(), m_placementSourceGuides.begin(),
                      m_placementSourceGuides.end());
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glLineWidth(4.0f);
    drawVertices(GL_LINES, crossesFor(allCenters, 5.5f),
                 CADNodeColor(0.02f, 0.02f, 0.02f, 1.0f));
    glLineWidth(2.0f);
    drawVertices(GL_LINES, crossesFor(gridCenters, 4.0f),
                 CADNodeColor(0.15f, 1.0f, 0.82f, 1.0f));
    drawVertices(GL_LINES, crossesFor(explicitCenters, 4.0f),
                 CADNodeColor(1.0f, 0.35f, 0.88f, 1.0f));
    drawVertices(GL_LINES, crossesFor(m_placementTargetGuides, 4.5f),
                 CADNodeColor(0.15f, 1.0f, 0.82f, 1.0f));
    drawVertices(GL_LINES, crossesFor(m_placementSourceGuides, 4.5f),
                 m_placementPreviewSnapped
                     ? CADNodeColor(0.25f, 1.0f, 0.35f, 1.0f)
                     : CADNodeColor(1.0f, 0.62f, 0.15f, 1.0f));
    glLineWidth(1.0f);
    drawVertices(GL_POINTS, verticesFor(allCenters),
                 CADNodeColor(0.02f, 0.02f, 0.02f, 1.0f), 9.0f);
    drawVertices(GL_POINTS, verticesFor(gridCenters),
                 CADNodeColor(0.15f, 1.0f, 0.82f, 1.0f), 5.0f);
    drawVertices(GL_POINTS, verticesFor(explicitCenters),
                 CADNodeColor(1.0f, 0.35f, 0.88f, 1.0f), 5.0f);
    drawVertices(GL_POINTS, verticesFor(m_placementTargetGuides),
                 CADNodeColor(0.15f, 1.0f, 0.82f, 1.0f), 5.0f);
    drawVertices(GL_POINTS, verticesFor(m_placementSourceGuides),
                 m_placementPreviewSnapped
                     ? CADNodeColor(0.25f, 1.0f, 0.35f, 1.0f)
                     : CADNodeColor(1.0f, 0.62f, 0.15f, 1.0f), 5.0f);
    glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);
}

void MeshRobotViewer::renderConveyorDirections() {
    if (!m_conveyorDirectionsVisible || m_conveyorDirectionGuides.empty()) return;

    const CADNodeColor outline(0.02f, 0.025f, 0.03f, 1.0f);
    const CADNodeColor flow(0.10f, 0.88f, 1.0f, 1.0f);
    const CADNodeColor terminal(1.0f, 0.68f, 0.12f, 1.0f);
    std::vector<float> arrowVertices;
    std::vector<float> terminalVertices;
    for (const ConveyorDirectionGuide& guide : m_conveyorDirectionGuides) {
        if (guide.points.size() < 2) continue;
        const size_t middle = guide.points.size() / 2;
        const size_t before = middle > 0 ? middle - 1 : 0;
        const size_t after = std::min(guide.points.size() - 1, middle + 1);
        Vec3f tangent = guide.points[after] - guide.points[before];
        const float tangentLength = tangent.length();
        if (tangentLength > 1.0e-4f) {
            tangent = tangent / tangentLength;
            Vec3f side(-tangent.z(), 0.0f, tangent.x());
            const float sideLength = side.length();
            if (sideLength > 1.0e-4f) side = side / sideLength;
            else side = Vec3f(1.0f, 0.0f, 0.0f);
            const Vec3f tip = guide.points[middle];
            const Vec3f base = tip - tangent * 95.0f;
            const Vec3f wingA = base + side * 48.0f;
            const Vec3f wingB = base - side * 48.0f;
            arrowVertices.insert(arrowVertices.end(), {
                tip.x(), tip.y(), tip.z(), wingA.x(), wingA.y(), wingA.z(),
                tip.x(), tip.y(), tip.z(), wingB.x(), wingB.y(), wingB.z()});
        }
        const auto appendTerminal = [&](const Vec3f& point) {
            terminalVertices.insert(terminalVertices.end(),
                                    {point.x(), point.y(), point.z()});
        };
        if (!guide.startConnected) appendTerminal(guide.points.front());
        if (!guide.endConnected) appendTerminal(guide.points.back());
    }

    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    for (float lineWidth : {7.0f, 3.0f}) {
        glLineWidth(lineWidth);
        const CADNodeColor& color = lineWidth > 3.0f ? outline : flow;
        for (const ConveyorDirectionGuide& guide : m_conveyorDirectionGuides) {
            if (guide.points.size() < 2) continue;
            m_scratchVertices.clear();
            m_scratchVertices.reserve(guide.points.size() * 3);
            for (const Vec3f& point : guide.points) {
                m_scratchVertices.insert(m_scratchVertices.end(),
                                         {point.x(), point.y(), point.z()});
            }
            drawVertices(GL_LINE_STRIP, m_scratchVertices, color);
        }
        drawVertices(GL_LINES, arrowVertices, color);
    }
    glLineWidth(1.0f);
    drawVertices(GL_POINTS, terminalVertices, outline, 14.0f);
    drawVertices(GL_POINTS, terminalVertices, terminal, 8.0f);
    glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);
}

void MeshRobotViewer::renderPathLine(const std::vector<Vec3f>& points, const CADNodeColor& color, float lineWidth) {
    if (points.size() < 2) return;

    glLineWidth(lineWidth);
    m_scratchVertices.clear();
    m_scratchVertices.reserve(points.size() * 3);
    for (const Vec3f& point : points) {
        m_scratchVertices.push_back(point.x());
        m_scratchVertices.push_back(point.y());
        m_scratchVertices.push_back(point.z());
    }
    drawVertices(GL_LINE_STRIP, m_scratchVertices, color);

    // Endpoint markers, one draw each because they are different colours and a single flat-colour
    // uniform cannot express both.
    const auto drawPoint = [&](const Vec3f& point, const CADNodeColor& pointColor) {
        const std::vector<float> single{point.x(), point.y(), point.z()};
        drawVertices(GL_POINTS, single, pointColor, 5.0f);
    };
    drawPoint(points.front(), CADNodeColor(1.0f, 1.0f, 1.0f, 1.0f));
    drawPoint(points.back(), CADNodeColor(1.0f, 0.72f, 0.12f, 1.0f));

    glLineWidth(1.0f);
}

void MeshRobotViewer::renderGimbals() {
    if (!m_gimbalsVisible) return;
    glDisable(GL_DEPTH_TEST);
    for (int index = 0; index < static_cast<int>(m_gimbalPoses.size()); ++index) renderGimbal(index);
    glLineWidth(1.0f);
    glEnable(GL_DEPTH_TEST);
}

void MeshRobotViewer::renderGimbal(int index) {
    const Vec3f origin = gimbalOrigin(index);
    const float length = gimbalLength();
    const float ringRadius = gimbalRingRadius();
    const struct {
        GimbalAxis axis;
    } axes[] = {
        {GimbalAxis::X},
        {GimbalAxis::Y},
        {GimbalAxis::Z}
    };

    glLineWidth(4.0f * kGimbalVisualScale);
    // One draw per axis: each axis has its own colour, and the shader carries a single flat colour.
    for (const auto& handle : axes) {
        const Vec3f axis = gimbalAxisVector(handle.axis, index);
        const Vec3f end = origin + axis * length;
        const std::vector<float> line{origin.x(), origin.y(), origin.z(), end.x(), end.y(), end.z()};
        drawVertices(GL_LINES, line, gimbalAxisColor(handle.axis));
    }

    glLineWidth(2.5f * kGimbalVisualScale);
    constexpr int kRingSegments = 96;
    for (const auto& handle : axes) {
        const Vec3f normal = gimbalAxisVector(handle.axis, index);
        const Vec3f helper = std::abs(Vec3f::dotProduct(normal, Vec3f(0.0f, 0.0f, 1.0f))) < 0.9f
            ? Vec3f(0.0f, 0.0f, 1.0f)
            : Vec3f(0.0f, 1.0f, 0.0f);
        const Vec3f u = Vec3f::crossProduct(normal, helper).normalized();
        const Vec3f v = Vec3f::crossProduct(normal, u).normalized();
        const CADNodeColor color = gimbalAxisColor(handle.axis);
        m_scratchVertices.clear();
        m_scratchVertices.reserve(static_cast<size_t>(kRingSegments) * 3);
        for (int i = 0; i < kRingSegments; ++i) {
            const double angle = (2.0 * 3.14159265358979323846 * static_cast<double>(i)) / static_cast<double>(kRingSegments);
            const Vec3f point = origin + (u * static_cast<float>(std::cos(angle)) + v * static_cast<float>(std::sin(angle))) * ringRadius;
            m_scratchVertices.push_back(point.x());
            m_scratchVertices.push_back(point.y());
            m_scratchVertices.push_back(point.z());
        }
        drawVertices(GL_LINE_LOOP, m_scratchVertices, color);

        const double arrowAngle = 0.85;
        const Vec3f arrowPoint = origin + (u * static_cast<float>(std::cos(arrowAngle)) + v * static_cast<float>(std::sin(arrowAngle))) * ringRadius;
        const Vec3f tangent = (-u * static_cast<float>(std::sin(arrowAngle)) + v * static_cast<float>(std::cos(arrowAngle))).normalized();
        const Vec3f radial = (arrowPoint - origin).normalized();
        const float arrowSize = std::max(8.0f * kGimbalScale, ringRadius * 0.12f);
        const Vec3f wingA = arrowPoint - tangent * arrowSize + radial * (arrowSize * 0.45f);
        const Vec3f wingB = arrowPoint - tangent * arrowSize - radial * (arrowSize * 0.45f);
        const std::vector<float> arrow{
            arrowPoint.x(), arrowPoint.y(), arrowPoint.z(), wingA.x(), wingA.y(), wingA.z(),
            arrowPoint.x(), arrowPoint.y(), arrowPoint.z(), wingB.x(), wingB.y(), wingB.z()};
        drawVertices(GL_LINES, arrow, color);
    }

    // The centre dot marks the one being dragged, so a cell of overlapping base gimbals says which
    // arm is actually moving.
    const bool dragged = m_draggingGimbal && index == m_activeGimbalIndex;
    const std::vector<float> center{origin.x(), origin.y(), origin.z()};
    drawVertices(GL_POINTS, center,
                 dragged ? CADNodeColor(1.0f, 0.72f, 0.12f, 1.0f) : CADNodeColor(1.0f, 1.0f, 1.0f, 1.0f),
                 (dragged ? 13.0f : 9.0f) * kGimbalVisualScale);
}

void MeshRobotViewer::renderMesh(const MeshGeometryData& mesh, const CadTransform& transform, const CADNodeColor& color) {
    if (!mesh.loaded || mesh.vertices.empty() || mesh.indices.empty()) return;

    MeshBuffer* meshBuffer = meshBufferFor(mesh);
    if (!meshBuffer) return;

    // The vertices live on the GPU untransformed, so the object transform rides in the model
    // matrix. Restored to identity afterwards because the paths, hulls and gimbal still
    // supply world-space vertices.
    m_modelMatrix = modelMatrixFor(transform);
    drawMeshBuffer(*meshBuffer, color);
    m_modelMatrix.setToIdentity();
}

void MeshRobotViewer::renderHull(const ConvexHullData& hull, const CadTransform& transform, const CADNodeColor& color) {
    if (hull.vertices.empty() || hull.indices.empty()) return;

    glDisable(GL_CULL_FACE);
    m_scratchVertices.clear();
    m_scratchVertices.reserve(hull.indices.size() * 18);
    const auto appendVertex = [&](uint32_t index) {
        const auto& v = hull.vertices[index];
        const Vec3f point = transformPoint(transform, static_cast<float>(v[0]), static_cast<float>(v[1]), static_cast<float>(v[2]));
        m_scratchVertices.push_back(point.x());
        m_scratchVertices.push_back(point.y());
        m_scratchVertices.push_back(point.z());
    };
    for (const auto& tri : hull.indices) {
        if (tri.size() < 3) continue;
        bool valid = true;
        for (uint32_t index : tri) {
            if (index >= hull.vertices.size()) valid = false;
        }
        if (!valid) continue;
        for (size_t edge = 0; edge < 3; ++edge) {
            appendVertex(tri[edge]);
            appendVertex(tri[(edge + 1) % 3]);
        }
    }
    drawVertices(GL_LINES, m_scratchVertices, color);
    glEnable(GL_CULL_FACE);
}

bool MeshRobotViewer::computeBoundsOf(const CadNode* node, Vec3f& outMin, Vec3f& outMax) {
    bool found = false;
    outMin = Vec3f(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
    outMax = Vec3f(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest());

    std::function<void(const CadNode*, const CadTransform&)> visit = [&](const CadNode* current, const CadTransform& parentTransform) {
        if (!current || !current->visible) return;
        const CadTransform worldTransform = parentTransform * current->loc;
        if (current->type == CadNodeType::MeshGeometry) {
            const MeshGeometryData* mesh = current->asMeshGeometry();
            if (mesh && mesh->loaded && !mesh->vertices.empty()) {
                for (size_t i = 0; i + 2 < mesh->vertices.size(); i += 3) {
                    includePoint(outMin, outMax, transformPoint(worldTransform, mesh->vertices[i], mesh->vertices[i + 1], mesh->vertices[i + 2]));
                    found = true;
                }
            }
        } else if (current->type == CadNodeType::DragChainMechanism) {
            const DragChainMechanismData* chain = current->asDragChainMechanism();
            const MeshGeometryData* prototype = chain && chain->prototypeGeometry
                ? chain->prototypeGeometry->asMeshGeometry() : nullptr;
            if (prototype && prototype->loaded && !prototype->vertices.empty()) {
                for (const CadNode* frame : chain->linkFrames) {
                    if (!frame || !frame->visible) continue;
                    const CadTransform instanceTransform = worldTransform * frame->loc;
                    for (size_t i = 0; i + 2 < prototype->vertices.size(); i += 3) {
                        includePoint(outMin, outMax, transformPoint(instanceTransform,
                            prototype->vertices[i], prototype->vertices[i + 1], prototype->vertices[i + 2]));
                        found = true;
                    }
                }
            }
        }
        for (const auto& child : current->children) visit(child.get(), worldTransform);
    };

    CadTransform toParent;
    for (const CadNode* ancestor = node ? node->parent : nullptr; ancestor != nullptr;
         ancestor = ancestor->parent) {
        toParent = ancestor->loc * toParent;
    }
    visit(node, toParent);
    return found;
}

bool MeshRobotViewer::computeBounds(Vec3f& outMin, Vec3f& outMax) const {
    return computeBoundsOf(m_root, outMin, outMax);
}

MeshRobotViewer::GimbalHandle MeshRobotViewer::hitTestGimbals(const PointI& screenPos,
                                                             int* outIndex) const {
    if (!m_gimbalsVisible) return GimbalHandle::None;

    const float length = gimbalLength();
    const float ringRadius = gimbalRingRadius();
    const PointF screenPoint(screenPos.x(), screenPos.y());

    GimbalHandle bestHandle = GimbalHandle::None;
    int bestIndex = -1;
    double bestDistance = 14.0;
    // Ties broken by depth: two arms' base gimbals can sit on top of each other on screen, and the
    // nearer one is the one being pointed at. Same rule pickNodeAt uses for whole arms.
    float bestDepth = std::numeric_limits<float>::max();

    for (int index = 0; index < static_cast<int>(m_gimbalPoses.size()); ++index) {
        const Vec3f origin = gimbalOrigin(index);
        PointF originScreen;
        if (!projectPoint(origin, originScreen)) continue;
        const float depth = -(m_view * Vec4f(origin, 1.0f)).z();

        // Equal-distance hits go to the nearer gimbal; a strictly closer hit wins outright. Written
        // as one predicate so both loops below apply the same rule.
        const auto better = [&](double distance) {
            if (distance > bestDistance) return false;
            if (distance < bestDistance) return true;
            return depth < bestDepth;
        };

        for (GimbalHandle handle : {GimbalHandle::TranslateX, GimbalHandle::TranslateY, GimbalHandle::TranslateZ}) {
            const GimbalAxis axis = gimbalHandleAxis(handle);
            PointF endScreen;
            if (!projectPoint(origin + gimbalAxisVector(axis, index) * length, endScreen)) continue;
            const double distance = pointSegmentDistance(screenPoint, originScreen, endScreen);
            if (better(distance)) {
                bestDistance = distance;
                bestDepth = depth;
                bestHandle = handle;
                bestIndex = index;
            }
        }

        constexpr int kRingSegments = 96;
        for (GimbalHandle handle : {GimbalHandle::RotateX, GimbalHandle::RotateY, GimbalHandle::RotateZ}) {
            const GimbalAxis axis = gimbalHandleAxis(handle);
            const Vec3f normal = gimbalAxisVector(axis, index);
            const Vec3f helper = std::abs(Vec3f::dotProduct(normal, Vec3f(0.0f, 0.0f, 1.0f))) < 0.9f
                ? Vec3f(0.0f, 0.0f, 1.0f)
                : Vec3f(0.0f, 1.0f, 0.0f);
            const Vec3f u = Vec3f::crossProduct(normal, helper).normalized();
            const Vec3f v = Vec3f::crossProduct(normal, u).normalized();

            PointF previousScreen;
            bool hasPrevious = false;
            for (int i = 0; i <= kRingSegments; ++i) {
                const double angle = (2.0 * 3.14159265358979323846 * static_cast<double>(i % kRingSegments)) / static_cast<double>(kRingSegments);
                const Vec3f point = origin + (u * static_cast<float>(std::cos(angle)) + v * static_cast<float>(std::sin(angle))) * ringRadius;
                PointF currentScreen;
                if (!projectPoint(point, currentScreen)) {
                    hasPrevious = false;
                    continue;
                }
                if (hasPrevious) {
                    const double distance = pointSegmentDistance(screenPoint, previousScreen, currentScreen);
                    if (better(distance)) {
                        bestDistance = distance;
                        bestDepth = depth;
                        bestHandle = handle;
                        bestIndex = index;
                    }
                }
                previousScreen = currentScreen;
                hasPrevious = true;
            }
        }
    }

    if (outIndex && bestHandle != GimbalHandle::None) *outIndex = bestIndex;
    return bestHandle;
}

void MeshRobotViewer::dragGimbal(const PointI& screenPos) {
    if (m_activeGimbalHandle == GimbalHandle::None) return;
    if (m_activeGimbalIndex < 0 || m_activeGimbalIndex >= static_cast<int>(m_gimbalPoses.size())) return;
    CadTransform& gimbalPose = m_gimbalPoses[static_cast<size_t>(m_activeGimbalIndex)];

    const Vec3f origin = gimbalOrigin(m_activeGimbalIndex);
    const GimbalAxis activeAxis = gimbalHandleAxis(m_activeGimbalHandle);
    const Vec3f axis = gimbalAxisVector(activeAxis, m_activeGimbalIndex);
    const float length = gimbalLength();
    PointF originScreen;
    if (!projectPoint(origin, originScreen)) return;

    if (m_activeGimbalHandle == GimbalHandle::TranslateX ||
        m_activeGimbalHandle == GimbalHandle::TranslateY ||
        m_activeGimbalHandle == GimbalHandle::TranslateZ) {
        PointF endScreen;
        if (!projectPoint(origin + axis * length, endScreen)) return;

        PointF axisScreen = endScreen - originScreen;
        const double screenLength = std::sqrt(axisScreen.x() * axisScreen.x() + axisScreen.y() * axisScreen.y());
        if (screenLength <= 1.0) return;
        axisScreen /= screenLength;

        const PointI delta = screenPos - m_lastMousePos;
        const double pixelsAlongAxis = delta.x() * axisScreen.x() + delta.y() * axisScreen.y();
        const double worldDelta = pixelsAlongAxis * static_cast<double>(length) / screenLength;
        const Vec3f offset = axis * static_cast<float>(worldDelta);

        gimbalPose.values[3] += offset.x();
        gimbalPose.values[7] += offset.y();
        gimbalPose.values[11] += offset.z();
    } else {
        const Vec3f helper = std::abs(Vec3f::dotProduct(axis, Vec3f(0.0f, 0.0f, 1.0f))) < 0.9f
            ? Vec3f(0.0f, 0.0f, 1.0f)
            : Vec3f(0.0f, 1.0f, 0.0f);
        const Vec3f u = Vec3f::crossProduct(axis, helper).normalized();
        const Vec3f v = Vec3f::crossProduct(axis, u).normalized();
        PointF uScreen;
        PointF vScreen;
        double orientationSign = 1.0;
        if (projectPoint(origin + u, uScreen) && projectPoint(origin + v, vScreen)) {
            const PointF us = uScreen - originScreen;
            const PointF vs = vScreen - originScreen;
            const double determinant = us.x() * vs.y() - us.y() * vs.x();
            orientationSign = determinant >= 0.0 ? 1.0 : -1.0;
        }

        const PointF previous(static_cast<double>(m_lastMousePos.x()) - originScreen.x(),
                               static_cast<double>(m_lastMousePos.y()) - originScreen.y());
        const PointF current(static_cast<double>(screenPos.x()) - originScreen.x(),
                              static_cast<double>(screenPos.y()) - originScreen.y());
        const double previousLength = std::sqrt(previous.x() * previous.x() + previous.y() * previous.y());
        const double currentLength = std::sqrt(current.x() * current.x() + current.y() * current.y());
        if (previousLength <= 2.0 || currentLength <= 2.0) return;
        double angleDelta = std::atan2(current.y(), current.x()) - std::atan2(previous.y(), previous.x());
        constexpr double kPi = 3.14159265358979323846;
        if (angleDelta > kPi) angleDelta -= 2.0 * kPi;
        if (angleDelta < -kPi) angleDelta += 2.0 * kPi;
        gimbalPose = gimbalPose * localAxisRotation(activeAxis, angleDelta * orientationSign);
    }

    if (m_gimbalMoveCallback) m_gimbalMoveCallback(m_activeGimbalIndex, gimbalPose);
    update();
}

Vec3f MeshRobotViewer::gimbalOrigin(int index) const {
    if (index < 0 || index >= static_cast<int>(m_gimbalPoses.size())) return Vec3f(0.0f, 0.0f, 0.0f);
    const CadTransform& pose = m_gimbalPoses[static_cast<size_t>(index)];
    return Vec3f(
        static_cast<float>(pose.values[3]),
        static_cast<float>(pose.values[7]),
        static_cast<float>(pose.values[11]));
}

// Shared by rendering, hit testing, and drag calculations.
float MeshRobotViewer::gimbalLength() const {
    // Scale with camera distance to keep a constant apparent size.
    return m_distance * 0.12f * kGimbalScale * kGimbalAxisScale;
}

// Radius of the three rotation rings, sized independently of the axes.
float MeshRobotViewer::gimbalRingRadius() const {
    return m_distance * 0.12f * kGimbalScale * 0.72f * kGimbalRingScale;
}

MeshRobotViewer::GimbalAxis MeshRobotViewer::gimbalHandleAxis(GimbalHandle handle) const {
    switch (handle) {
    case GimbalHandle::TranslateX:
    case GimbalHandle::RotateX:
        return GimbalAxis::X;
    case GimbalHandle::TranslateY:
    case GimbalHandle::RotateY:
        return GimbalAxis::Y;
    case GimbalHandle::TranslateZ:
    case GimbalHandle::RotateZ:
        return GimbalAxis::Z;
    case GimbalHandle::None:
        break;
    }
    return GimbalAxis::None;
}

Vec3f MeshRobotViewer::gimbalAxisVector(GimbalAxis axis, int index) const {
    if (index < 0 || index >= static_cast<int>(m_gimbalPoses.size())) return Vec3f(1.0f, 0.0f, 0.0f);
    const CadTransform& pose = m_gimbalPoses[static_cast<size_t>(index)];
    switch (axis) {
    case GimbalAxis::X:
        return Vec3f(static_cast<float>(pose.values[0]), static_cast<float>(pose.values[4]), static_cast<float>(pose.values[8])).normalized();
    case GimbalAxis::Y:
        return Vec3f(static_cast<float>(pose.values[1]), static_cast<float>(pose.values[5]), static_cast<float>(pose.values[9])).normalized();
    case GimbalAxis::Z:
        return Vec3f(static_cast<float>(pose.values[2]), static_cast<float>(pose.values[6]), static_cast<float>(pose.values[10])).normalized();
    case GimbalAxis::None:
        break;
    }
    return Vec3f(1.0f, 0.0f, 0.0f);
}

bool MeshRobotViewer::projectPoint(const Vec3f& point, PointF& outScreen) const {
    // Replaces gluProject. Uses the same matrices paintGL drew with, so hit testing and rendering
    // cannot drift apart, and rejects points at or behind the eye where the divide is meaningless.
    const Vec4f clip = m_projection * m_view * Vec4f(point, 1.0f);
    if (clip.w() <= 1.0e-6f) return false;
    const Vec3f ndc = Vec3f(clip.x(), clip.y(), clip.z()) / clip.w();
    // Against the framebuffer being rendered this frame, not a window: the panel size is
    // whatever the host passed to render().
    outScreen = PointF((ndc.x() * 0.5f + 0.5f) * static_cast<double>(m_viewportWidth),
                        (1.0 - (ndc.y() * 0.5f + 0.5f)) * static_cast<double>(m_viewportHeight));
    return true;
}

bool MeshRobotViewer::projectWorldPoint(const Vec3f& point, PointF& outScreen) const {
    return projectPoint(point, outScreen);
}

bool MeshRobotViewer::worldRayAt(const PointI& localPos, Vec3f& outOrigin,
                                 Vec3f& outDirection) const {
    if (m_viewportWidth <= 0 || m_viewportHeight <= 0) return false;
    const float ndcX = 2.0f * static_cast<float>(localPos.x()) /
                           static_cast<float>(m_viewportWidth) - 1.0f;
    const float ndcY = 1.0f - 2.0f * static_cast<float>(localPos.y()) /
                           static_cast<float>(m_viewportHeight);
    const float aspect = static_cast<float>(m_viewportWidth) /
                         static_cast<float>(m_viewportHeight);
    constexpr float kTanHalfFov = 0.4142135623730950f; // tan(45 degrees / 2)
    const Vec3f viewDirection(ndcX * aspect * kTanHalfFov, ndcY * kTanHalfFov, -1.0f);

    // Inverse of the view rotation Rx(pitch) * Ry(yaw).
    Mat4 inverseRotation;
    inverseRotation.rotate(-m_yaw, 0.0f, 1.0f, 0.0f);
    inverseRotation.rotate(-m_pitch, 1.0f, 0.0f, 0.0f);
    const Vec4f worldDirection4 = inverseRotation * Vec4f(viewDirection, 0.0f);
    const Vec4f eyeOffset4 = inverseRotation * Vec4f(0.0f, 0.0f, m_distance, 0.0f);
    outOrigin = m_center + Vec3f(eyeOffset4.x(), eyeOffset4.y(), eyeOffset4.z());
    outDirection = Vec3f(worldDirection4.x(), worldDirection4.y(), worldDirection4.z()).normalized();
    return outDirection.length() > 0.5f;
}

void MeshRobotViewer::rebuildCollisionNodeSets(const std::vector<int>& linkIndices, const CadNode* robotNode) {
    m_collidingLinkNodes.clear();
    m_collidingGeometryNodes.clear();

    std::vector<const CadNode*> links;
    collectRobotLinkNodes(robotNode != nullptr ? robotNode : m_root, links);
    for (int linkIndex : linkIndices) {
        if (linkIndex < 0 || linkIndex >= static_cast<int>(links.size())) continue;
        const CadNode* linkNode = links[static_cast<size_t>(linkIndex)];
        m_collidingLinkNodes.insert(linkNode);

        const RobotLinkData* linkData = linkNode ? linkNode->asRobotLink() : nullptr;
        if (!linkData) continue;
        for (const CadNode* geometryNode : linkData->geometryNodes) {
            if (geometryNode) m_collidingGeometryNodes.insert(geometryNode);
        }
    }
}
