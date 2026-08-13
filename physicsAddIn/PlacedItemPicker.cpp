#include "PlacedItemPicker.h"

#include "PhysicsWorld.h"
#include "RoboDkBridge.h"
#include "SelectionStyle.h"
#include "ViewRay.h"

#include "iitem.h"
#include "irobodk.h"

#include <QApplication>
#include <QDateTime>
#include <QDir>
#include <QEvent>
#include <QFile>
#include <QMainWindow>
#include <QMouseEvent>
#include <QTextStream>
#include <QWidget>

#include <iterator>
#include <limits>

namespace {

// Every reading of the two undocumented conventions, so one run of clicks settles them.
struct Convention {
    const char* name;
    bool angleIsVertical;
    bool poseIsWorldFromCamera;
    bool looksDownNegativeZ;
};

const Convention kConventions[] = {
    {"vertical, pose=world<-camera, -Z", true, true, true},
    {"vertical, pose=world<-camera, +Z", true, true, false},
    {"vertical, pose=camera<-world, -Z", true, false, true},
    {"horizontal, pose=world<-camera, -Z", false, true, true},
    {"horizontal, pose=camera<-world, -Z", false, false, true},
};

// **Measured, not assumed.** Twelve clicks on the showcase's conveyor from several angles and zooms:
// `vertical, pose=camera<-world, -Z` hit it twelve times out of twelve, `horizontal` hit nine - failing on
// exactly the three clicks furthest from the centre of the view - and neither `world<-camera` reading ever
// hit at all. So `ViewPose` is the camera-from-world transform, and "View angle" is measured vertically.
constexpr int kAssumed = 2;

double g_viewAngleDeg = 30.0;

constexpr int kMeasuredClicks = 12;

CadVec3 positionOf(const CadTransform& transform) {
    return CadVec3(transform.values[3], transform.values[7], transform.values[11]);
}

bool sameTransform(const CadTransform& a, const CadTransform& b) {
    for (int cell = 0; cell < 12; ++cell) {
        if (a.values[cell] != b.values[cell]) return false;
    }
    return true;
}

} // namespace

QWidget* roboDkViewport(QMainWindow* mainWindow) {
    if (!mainWindow) return nullptr;
    // Not cached: it does not exist at PluginLoad in a hidden session, and a station change can replace it.
    return mainWindow->findChild<QWidget*>(QStringLiteral("MainGL"));
}

double roboDkViewAngleDeg() { return g_viewAngleDeg; }

void setRoboDkViewAngleDeg(double degrees) {
    if (degrees > 1.0 && degrees < 179.0) g_viewAngleDeg = degrees;
}

bool roboDkViewCamera(QWidget* viewport, RoboDK* rdk, viewray::Camera* camera, int convention) {
    if (!viewport || !rdk || !camera) return false;
    const Convention& reading =
        kConventions[convention < 0 || convention >= static_cast<int>(std::size(kConventions))
                         ? kAssumed : convention];
    camera->pose = rdkbridge::toCadTransform(rdk->ViewPose());
    camera->poseIsWorldFromCamera = reading.poseIsWorldFromCamera;
    camera->looksDownNegativeZ = reading.looksDownNegativeZ;
    camera->widthPx = viewport->width();
    camera->heightPx = viewport->height();
    camera->viewAngleDeg = g_viewAngleDeg;
    camera->angleIsVertical = reading.angleIsVertical;
    return camera->widthPx > 0 && camera->heightPx > 0;
}

int roboDkViewConventionCount() { return static_cast<int>(std::size(kConventions)); }

const char* roboDkViewConventionName(int convention) {
    if (convention < 0 || convention >= roboDkViewConventionCount()) return "(none)";
    return kConventions[convention].name;
}

PlacedItemPicker::PlacedItemPicker(QMainWindow* mw, RoboDK* rdk, PhysicsWorld* world, QObject* parent)
    : QObject(parent), m_mainWindow(mw), m_rdk(rdk), m_world(world) {
    if (qApp) qApp->installEventFilter(this);
}

QString PlacedItemPicker::report() {
    return QDir(QDir::tempPath()).filePath(QStringLiteral("physics_conveyor_pick.txt"));
}

void PlacedItemPicker::note(const QString& line) {
    QFile out(report());
    if (!out.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text)) return;
    QTextStream stream(&out);
    stream << line << '\n';
}

QString PlacedItemPicker::itemUnder(double pixelX, double pixelY, int convention,
                                    double* distanceMm) {
    if (!m_world) return QString();
    viewray::Camera camera;
    if (!roboDkViewCamera(roboDkViewport(m_mainWindow), m_rdk, &camera, convention)) return QString();

    const viewray::Ray ray = viewray::rayThroughPixel(camera, pixelX, pixelY);

    QString nearestName;
    double nearest = std::numeric_limits<double>::max();
    for (const PlacedItem* placed : m_world->placedItems()) {
        if (placed->name.isEmpty() || placed->scenery.groups.empty()) continue;
        for (const ConveyorDrawGroup& group : placed->scenery.groups) {
            if (group.triangles <= 0) continue;
            double distance = 0.0;
            if (!viewray::rayHitsTriangles(ray, group.verticesMm, group.triangles, &distance)) continue;
            if (distance >= nearest) continue;
            nearest = distance;
            nearestName = placed->name;
        }
    }
    if (!nearestName.isEmpty() && distanceMm) *distanceMm = nearest;
    return nearestName;
}

bool PlacedItemPicker::eventFilter(QObject* watched, QEvent* event) {
    if (!event || event->type() != QEvent::MouseButtonPress) {
        return QObject::eventFilter(watched, event);
    }
    if (!watched || watched->objectName() != QStringLiteral("MainGL")) {
        return QObject::eventFilter(watched, event);
    }
    auto* mouse = static_cast<QMouseEvent*>(event);
    if (mouse->button() != Qt::LeftButton) return QObject::eventFilter(watched, event);
    // An armed placement owns the view. Without this, the click that commits a ghost also picks whatever
    // stands behind it, and the operator is left looking at the wrong item's outline.
    if (m_suspended) return QObject::eventFilter(watched, event);

    if (m_measured < kMeasuredClicks) {
        ++m_measured;
        QWidget* view = roboDkViewport(m_mainWindow);
        note(QStringLiteral("--- click %1 at (%2, %3) in a %4x%5 view, %6 deg, %7")
                 .arg(m_measured)
                 .arg(mouse->pos().x())
                 .arg(mouse->pos().y())
                 .arg(view ? view->width() : -1)
                 .arg(view ? view->height() : -1)
                 .arg(roboDkViewAngleDeg())
                 .arg(QDateTime::currentDateTime().toString(Qt::ISODate)));
        for (int convention = 0; convention < roboDkViewConventionCount(); ++convention) {
            double distance = 0.0;
            const QString hit = itemUnder(mouse->pos().x(), mouse->pos().y(), convention, &distance);
            note(QStringLiteral("    %1 -> %2")
                     .arg(QString::fromLatin1(roboDkViewConventionName(convention)),
                          hit.isEmpty() ? QStringLiteral("(nothing)")
                                        : QStringLiteral("%1 at %2 mm").arg(hit).arg(distance, 0, 'f', 1)));
        }
    }

    const QString hit = itemUnder(mouse->pos().x(), mouse->pos().y(), -1, nullptr);
    if (hit.isEmpty()) {
        // Not consumed, and the highlight is cleared: a click on empty space is RoboDK's to handle, and
        // leaving a conveyor lit would say it was still selected.
        m_picked.clear();
        return QObject::eventFilter(watched, event);
    }
    m_picked = hit;
    emit itemPicked(hit);
    // Still not consumed. While RoboDK's own picking is also live this must not fight it, and an operator
    // who meant to rotate the view would otherwise find a click did nothing.
    return QObject::eventFilter(watched, event);
}

void PlacedItemPicker::drawHighlight() {
    if (m_picked.isEmpty() || !m_rdk || !m_world) return;
    const PlacedItem* placed = m_world->placedItemNamed(m_picked);
    if (!placed || placed->scenery.groups.empty()) return;

    // The edge set is rebuilt when the item or its placement changes, never per frame: welding a soup
    // of six thousand triangles into edges is the expensive half, and asking those edges which of them
    // face the eye is the cheap one. A render is the cheap half. An item that has moved is re-baked at
    // its new placement, so `sceneryAt` is what says these edges are stale.
    if (m_edgesFor != m_picked || !sameTransform(m_edgesAt, placed->sceneryAt)) {
        std::vector<float> soup;
        for (const ConveyorDrawGroup& group : placed->scenery.groups) {
            soup.insert(soup.end(), group.verticesMm.begin(), group.verticesMm.end());
        }
        m_edges = viewray::buildMeshEdges(soup, static_cast<int>(soup.size() / 9));
        m_edgesFor = m_picked;
        m_edgesAt = placed->sceneryAt;
    }
    if (m_edges.edges <= 0) return;

    viewray::Camera camera;
    if (!roboDkViewCamera(roboDkViewport(m_mainWindow), m_rdk, &camera)) return;
    const CadVec3 eye = viewray::cameraEye(camera);

    std::vector<float> lines;
    viewray::silhouetteLines(m_edges, eye, &lines);
    if (lines.empty()) return;

    const CADNodeColor colour = selectionstyle::outlineColor();
    float rgba[4] = {colour.r, colour.g, colour.b, colour.a};
    const CadVec3 stands = difference(eye, positionOf(placed->sceneryAt));
    const double away = std::sqrt(dot(stands, stands));
    const double thickness = selectionstyle::outlineThicknessMm(away);
    const float width = static_cast<float>(std::max(2.0, std::min(6.0, thickness * 0.01)));
    m_rdk->DrawGeometry(IRoboDK::DrawLines, lines.data(), static_cast<int>(lines.size() / 3), rgba, width);
}
