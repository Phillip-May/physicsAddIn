#ifndef PLUGINPHYSICS_H
#define PLUGINPHYSICS_H

#include <QObject>
#include <QtPlugin>

#include <map>
#include <memory>

#include "iapprobodk.h"
#include "robodktypes.h"

#include <QString>
#include <QStringList>

// moc requires PhysicsWorld to be complete when instantiating the plugin.
#include "PhysicsWorld.h"

class QAction;
class QMenu;
class QTimer;
class QToolBar;
class IRoboDK;
class IItem;

class PhysicsPanel;
class ConveyorPropertiesPanel;
class PlacedItemPropertiesPanel;
class LibraryDock;
class LibraryPlacementTool;
class PlacedItemPicker;
class QDockWidget;

class PluginPhysics : public QObject, IAppRoboDK {
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "RoboDK.IAppRoboDK")
    Q_INTERFACES(IAppRoboDK)

public:
    ~PluginPhysics() override;

    static QString getPluginName();
    QString PluginName() override;
    QString PluginLoad(QMainWindow* mw, QMenuBar* menubar, QStatusBar* statusbar, RoboDK* rdk,
                       const QString& settings = "") override;
    void PluginUnload() override;
    void PluginLoadToolbar(QMainWindow* mw, int icon_size) override;
    bool PluginItemClick(Item item, QMenu* menu, TypeClick click_type) override;
    QString PluginCommand(const QString& command, const QString& value) override;
    void PluginEvent(TypeEvent event_type) override;

public:
    QMainWindow* MainWindow = nullptr;
    QStatusBar* StatusBar = nullptr;
    RoboDK* RDK = nullptr;

private:
    void stepSimulation();
    void setSimulationTimerActive(bool active);
    void loadStationConfiguration();
    bool anyProgramRunning();
    void pollStationRunControl();
    // RoboDK has no safe item-edited or item-deleted event, so panels are polled.
    void syncConveyorPanels();
    void openConveyorPanel(Item item);
    void syncPlacedItemPanels();
    void openPlacedItemPanel(Item item);
    // Deferred because EventChanged may arrive re-entrantly during a simulation step.
    void adoptConveyorsIfRequested();
    void addConveyor();
    QStringList workpieceCandidates();

    std::unique_ptr<PhysicsWorld> m_world;
    PhysicsPanel* m_panel = nullptr;
    // Item identity survives renames; hidden panels are reused.
    std::map<Item, ConveyorPropertiesPanel*> m_conveyorPanels;
    std::map<Item, PlacedItemPropertiesPanel*> m_placedItemPanels;
    PlacedItemPicker* m_itemPicker = nullptr;
    LibraryDock* m_libraryDock = nullptr;
    LibraryPlacementTool* m_libraryPlacer = nullptr;
    QAction* m_actionAddConveyor = nullptr;
    bool m_conveyorAdoptionRequested = false;
    QStringList m_workpieceCandidates;
    QTimer* m_stepTimer = nullptr;
    QTimer* m_runControlTimer = nullptr;
    QTimer* m_panelTimer = nullptr;
    QAction* m_actionShowPanel = nullptr;
    qint64 m_lastStepMs = 0;
    // RoboDK does not emit a program-start event; run control detects this edge.
    bool m_programWasRunning = false;
};

#endif // PLUGINPHYSICS_H
