#ifndef LIBRARYPLACEMENTTOOL_H
#define LIBRARYPLACEMENTTOOL_H

#include "LibraryPlacer.h"
#include "PlacementSession.h"

#include <QObject>
#include <QString>

#include <vector>

class QMainWindow;
class QWidget;
class PhysicsWorld;
class PlacedItemPicker;

class LibraryPlacementTool : public QObject {
    Q_OBJECT

public:
    LibraryPlacementTool(QMainWindow* mainWindow, RoboDK* rdk, PhysicsWorld* world,
                         PlacedItemPicker* picker, QObject* parent = nullptr);

    // Arming creates no station item until commit.
    bool arm(const librarycatalogue::Entry& entry, QString* reason);
    bool armed() const { return m_session.armed(); }
    void cancel();

    void draw();

signals:
    void statusChanged(const QString& text);
    void committed(const QString& name);
    void disarmed();

protected:
    bool eventFilter(QObject* watched, QEvent* event) override;

private:
    void moved(const QPoint& pixel);
    void commit();
    void announce();

    QMainWindow* m_mainWindow = nullptr;
    RoboDK* m_rdk = nullptr;
    PhysicsWorld* m_world = nullptr;
    PlacedItemPicker* m_picker = nullptr;

    ArmedPackage m_armed;
    placementsession::Session m_session;
    // Ghost geometry remains item-local and is transformed per render.
    cadnodedraw::Scenery m_ghostLocal;
    // Scratch buffers reused each frame.
    std::vector<float> m_ghostVertices;
    std::vector<float> m_ghostNormals;
    std::vector<float> m_guides;
    std::vector<CadVec3> m_floorWindow;
    bool m_aimed = false;
};

#endif // LIBRARYPLACEMENTTOOL_H
