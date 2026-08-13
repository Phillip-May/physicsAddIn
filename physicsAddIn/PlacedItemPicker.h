#ifndef PLACEDITEMPICKER_H
#define PLACEDITEMPICKER_H

#include <QHash>
#include <QObject>
#include <QString>

#include "ViewRay.h"
#include "robodktypes.h"

class QMainWindow;
class QWidget;
class PhysicsWorld;

// Looks up MainGL lazily; hidden RoboDK sessions have no viewport.
QWidget* roboDkViewport(QMainWindow* mainWindow);

double roboDkViewAngleDeg();
void setRoboDkViewAngleDeg(double degrees);

bool roboDkViewCamera(QWidget* viewport, RoboDK* rdk, viewray::Camera* camera, int convention = -1);

int roboDkViewConventionCount();
const char* roboDkViewConventionName(int convention);

class PlacedItemPicker : public QObject {
    Q_OBJECT

public:
    PlacedItemPicker(QMainWindow* mw, RoboDK* rdk, PhysicsWorld* world, QObject* parent = nullptr);

    static QString report();

    // Held by name because RoboDK may invalidate an Item asynchronously.
    QString picked() const { return m_picked; }
    void forget() { m_picked.clear(); }

    void drawHighlight();

    void setSuspended(bool suspended) { m_suspended = suspended; }

signals:
    void itemPicked(const QString& conveyor);

protected:
    bool eventFilter(QObject* watched, QEvent* event) override;

private:
    QString itemUnder(double pixelX, double pixelY, int convention, double* distanceMm);
    void note(const QString& line);

    QMainWindow* m_mainWindow = nullptr;
    RoboDK* m_rdk = nullptr;
    PhysicsWorld* m_world = nullptr;
    QString m_picked;
    bool m_suspended = false;
    int m_measured = 0;
    // Cached until the selected conveyor or its placement changes.
    viewray::MeshEdges m_edges;
    QString m_edgesFor;
    CadTransform m_edgesAt;
};

#endif // PLACEDITEMPICKER_H
