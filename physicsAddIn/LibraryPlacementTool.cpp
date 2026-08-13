#include "LibraryPlacementTool.h"

#include "PhysicsWorld.h"
#include "PlacedItemPicker.h"
#include "ViewRay.h"

#include "irobodk.h"

#include <QApplication>
#include <QEvent>
#include <QKeyEvent>
#include <QMainWindow>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QWidget>

namespace {

constexpr double kSnapScreenPercent = 1.5;

// A ghost, not a thing. Green when it is mated and amber when it is not - the same two colours
// RobotSimulator's placement caption uses - and translucent, so what it is being aligned to stays visible
// through it.
constexpr float kSnappedRgba[4] = {0.27f, 0.92f, 0.45f, 0.55f};
constexpr float kFreeRgba[4] = {1.0f, 0.61f, 0.18f, 0.45f};
// The holes themselves. The target's are what the ghost is aiming at, so they are the brighter pair.
constexpr float kTargetGuideRgba[4] = {0.35f, 0.80f, 1.0f, 0.95f};
constexpr float kSourceGuideRgba[4] = {1.0f, 0.95f, 0.35f, 0.95f};
constexpr float kFloorWindowRgba[4] = {0.55f, 0.62f, 0.70f, 0.55f};

struct RayCameraView final : mountingsnap::View {
    viewray::Camera camera;

    bool project(const CadVec3& worldMm, double* pixelX, double* pixelY) const override {
        return viewray::projectPoint(camera, worldMm, pixelX, pixelY);
    }
    bool rayThrough(double pixelX, double pixelY, CadVec3* originMm,
                    CadVec3* direction) const override {
        const viewray::Ray ray = viewray::rayThroughPixel(camera, pixelX, pixelY);
        *originMm = ray.origin;
        *direction = ray.direction;
        return true;
    }
};

void appendPoint(std::vector<float>* buffer, const CadVec3& point) {
    buffer->push_back(static_cast<float>(point.x));
    buffer->push_back(static_cast<float>(point.y));
    buffer->push_back(static_cast<float>(point.z));
}

} // namespace

LibraryPlacementTool::LibraryPlacementTool(QMainWindow* mainWindow, RoboDK* rdk, PhysicsWorld* world,
                                           PlacedItemPicker* picker, QObject* parent)
    : QObject(parent), m_mainWindow(mainWindow), m_rdk(rdk), m_world(world), m_picker(picker) {
    if (qApp) qApp->installEventFilter(this);
}

bool LibraryPlacementTool::arm(const librarycatalogue::Entry& entry, QString* reason) {
    cancel();
    ArmedPackage armed = armLibraryEntry(entry);
    if (!armed.armed()) {
        if (reason) *reason = armed.refusal;
        return false;
    }
    m_armed = std::move(armed);
    m_session.arm(m_armed.sourceInterfaces);
    // Flattened once, here. Everything after this is a transform of these triangles.
    m_ghostLocal = cadnodedraw::flatten(m_armed.tree.get(), CadTransform());
    m_aimed = false;
    if (m_picker) m_picker->setSuspended(true);
    if (reason) reason->clear();
    emit statusChanged(tr("Placing %1 - point at the cell, wheel turns, Esc cancels.")
                           .arg(QString::fromStdString(m_armed.entry.name)));
    return true;
}

void LibraryPlacementTool::cancel() {
    const bool was = m_session.armed();
    m_session.cancel();
    m_armed = ArmedPackage();
    m_ghostLocal = cadnodedraw::Scenery();
    m_aimed = false;
    if (m_picker) m_picker->setSuspended(false);
    if (!was) return;
    emit statusChanged(QString());
    emit disarmed();
    // RenderScreen for the same reason `moved` needs it, in the other direction: the ghost has to stop
    // being drawn, and only a redisplay stops it. An update-only left it on screen until something else
    // happened to repaint.
    if (m_rdk) m_rdk->Render(RoboDK::RenderScreen);
}

void LibraryPlacementTool::moved(const QPoint& pixel) {
    if (!m_session.armed() || !m_world) return;
    RayCameraView view;
    if (!roboDkViewCamera(roboDkViewport(m_mainWindow), m_rdk, &view.camera)) return;

    // Center the finite floor-grid window on the current cursor target before collecting snaps.
    m_world->recentreFloorGrid(view, pixel.x(), pixel.y());

    // Placement targets may move or be deleted between pointer events.
    std::vector<mountingsnap::Interface> targets;
    m_world->collectSnapTargets(&targets);

    m_session.moved(pixel.x(), pixel.y(), targets, view, view.camera.widthPx, view.camera.heightPx,
                    m_world->floorNode(), kSnapScreenPercent);
    m_aimed = true;
    announce();
    if (m_rdk) m_rdk->Render(RoboDK::RenderScreen);
}

void LibraryPlacementTool::announce() {
    const mountingsnap::Result& last = m_session.last();
    emit statusChanged(tr("%1 - %2")
                           .arg(QString::fromStdString(m_armed.entry.name),
                                QString::fromStdString(last.status)));
}

void LibraryPlacementTool::commit() {
    if (!m_session.armed() || !m_world) return;
    if (!m_aimed) {
        // Armed but never aimed: there is no pose to commit, so the click is the one that starts aiming.
        emit statusChanged(tr("Move the pointer over the cell first."));
        return;
    }
    const ArmedPackage armed = m_armed;
    const CadTransform pose = m_session.last().worldPose;
    const bool snapped = m_session.last().snapped;
    const int matched = m_session.last().matchedHoles;
    const QString what = QString::fromStdString(armed.entry.name);
    // Disarmed before delivering, and the ghost with it: a commit is a string of RoboDK calls that come
    // back as an EventChanged, and an EventRender arriving inside that would paint a ghost for something
    // that is now standing in the cell.
    cancel();

    QString name;
    QString error;
    if (!m_world->commitPlacement(armed, &pose, &name, &error)) {
        emit statusChanged(tr("%1 was not placed: %2").arg(what, error));
        return;
    }
    emit statusChanged(snapped ? tr("Placed %1 on %2 mounting holes.").arg(name).arg(matched)
                               : tr("Placed %1.").arg(name));
    emit committed(name);
}

bool LibraryPlacementTool::eventFilter(QObject* watched, QEvent* event) {
    if (!event || !m_session.armed()) return QObject::eventFilter(watched, event);

    // Esc wherever the keyboard focus happens to be. A placement is modal in the operator's head, and one
    // that could only be cancelled by clicking in the 3D view would have to be committed to be escaped.
    if (event->type() == QEvent::KeyPress &&
        static_cast<QKeyEvent*>(event)->key() == Qt::Key_Escape) {
        cancel();
        return true;
    }
    if (!watched || watched->objectName() != QStringLiteral("MainGL")) {
        return QObject::eventFilter(watched, event);
    }

    switch (event->type()) {
    case QEvent::MouseMove:
        moved(static_cast<QMouseEvent*>(event)->pos());
        // Consumed. While a placement is armed the 3D view is aiming it, and letting the move through
        // would orbit the camera the ghost is being aimed through.
        return true;
    case QEvent::MouseButtonPress:
        if (static_cast<QMouseEvent*>(event)->button() == Qt::RightButton) {
            cancel();
            return true;
        }
        if (static_cast<QMouseEvent*>(event)->button() != Qt::LeftButton) break;
        // Aim first if the pointer arrived without a move - a click straight after arming otherwise
        // commits at no pose at all.
        if (!m_aimed) moved(static_cast<QMouseEvent*>(event)->pos());
        commit();
        // Consumed, and this is the one that matters: a press that also reached RoboDK would orbit the
        // view and select whatever stands behind the ghost at the moment of placing it.
        return true;
    case QEvent::MouseButtonRelease:
        // A release that arrives here still armed and already aimed is the end of a *drag* out of the
        // library list: the press happened on a row, so no press over the view ever committed. Dropping is
        // what that gesture means, so it commits.
        if (static_cast<QMouseEvent*>(event)->button() == Qt::LeftButton && m_aimed) commit();
        return true;
    case QEvent::MouseButtonDblClick:
        // The press was consumed, so any double click belongs to the same gesture. Letting it through
        // leaves RoboDK finishing an orbit that never began.
        return true;
    case QEvent::Wheel: {
        const QPoint notches = static_cast<QWheelEvent*>(event)->angleDelta();
        m_session.rotated(static_cast<float>(notches.y()) / 120.0f);
        moved(static_cast<QWheelEvent*>(event)->position().toPoint());
        // Consumed, or the view zooms while the ghost turns.
        return true;
    }
    default:
        break;
    }
    return QObject::eventFilter(watched, event);
}

void LibraryPlacementTool::draw() {
    if (!m_session.armed() || !m_aimed || !m_rdk) return;
    const mountingsnap::Result& last = m_session.last();

    // Transformed per frame, never re-flattened: the triangles are the armed tree's own and the pose is
    // this frame's. Rotation alone for the normals, because a normal is a direction.
    const CadTransform& pose = last.worldPose;
    const float* tint = last.snapped ? kSnappedRgba : kFreeRgba;
    for (const cadnodedraw::DrawGroup& group : m_ghostLocal.groups) {
        if (group.triangles <= 0) continue;
        m_ghostVertices.clear();
        m_ghostNormals.clear();
        m_ghostVertices.reserve(group.verticesMm.size());
        m_ghostNormals.reserve(group.normals.size());
        for (size_t at = 0; at + 2 < group.verticesMm.size(); at += 3) {
            appendPoint(&m_ghostVertices,
                        pose * CadVec3(group.verticesMm[at], group.verticesMm[at + 1],
                                       group.verticesMm[at + 2]));
            appendPoint(&m_ghostNormals,
                        rotate(pose, CadVec3(group.normals[at], group.normals[at + 1],
                                             group.normals[at + 2])));
        }
        float rgba[4] = {tint[0], tint[1], tint[2], tint[3]};
        m_rdk->DrawGeometry(IRoboDK::DrawTriangles, m_ghostVertices.data(), group.triangles, rgba, 2.0f,
                            m_ghostNormals.data());
    }

    const auto points = [&](const std::vector<CadVec3>& worldMm, const float colour[4], float size) {
        if (worldMm.empty()) return;
        m_guides.clear();
        m_guides.reserve(worldMm.size() * 3);
        for (const CadVec3& point : worldMm) appendPoint(&m_guides, point);
        float rgba[4] = {colour[0], colour[1], colour[2], colour[3]};
        m_rdk->DrawGeometry(IRoboDK::DrawPoints, m_guides.data(), static_cast<int>(worldMm.size()), rgba,
                            size);
    };
    m_world->collectFloorPoints(&m_floorWindow);
    points(m_floorWindow, kFloorWindowRgba, 3.0f);
    points(last.targetGuidesMm, kTargetGuideRgba, 6.0f);
    points(last.sourceGuidesMm, kSourceGuideRgba, 9.0f);
}
