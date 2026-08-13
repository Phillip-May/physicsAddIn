#ifndef PHYSICSPANEL_H
#define PHYSICSPANEL_H

#include "PhysicsWorld.h"

#include <QDockWidget>

class QDoubleSpinBox;
class QLabel;
class QPushButton;
class QTreeWidget;

// The plugin's whole interface, as a dock RoboDK arranges alongside its own panels.
class PhysicsPanel : public QDockWidget {
    Q_OBJECT

public:
    PhysicsPanel(PhysicsWorld* world, QWidget* parent = nullptr);

    // Re-reads the world. Called after any change made from outside the panel - a context-menu
    // role, a station change - so the panel never holds its own copy of what the world knows.
    void refresh();

    void refreshStatus();

    // The item tree brought up to date *without* rebuilding it: rows added for participants that
    // are new, dropped for participants that have gone, and role and status written in place.
    void syncItems();

signals:
    // The run state changed in a way the host cares about: it drives whether frames are stepped at
    // all, and so whether RoboDK is asked to render.
    void runStateChanged();

private:
    void buildUi();
    void applyGravity();
    void removeSelected();

    PhysicsWorld* m_world = nullptr;
    QTreeWidget* m_items = nullptr;
    QPushButton* m_start = nullptr;
    QPushButton* m_pause = nullptr;
    QPushButton* m_stop = nullptr;
    QPushButton* m_remove = nullptr;
    QDoubleSpinBox* m_gravity = nullptr;
    QLabel* m_status = nullptr;
};

#endif // PHYSICSPANEL_H
