#include "pluginPhysics.h"

#include "PlacedItemPicker.h"
#include "ConveyorPropertiesPanel.h"
#include "PhysicsIcons.h"
#include "PlacedItemPropertiesPanel.h"
#include "LibraryDock.h"
#include "LibraryPlacementTool.h"
#include "PhysicsPanel.h"
#include "PhysicsWorld.h"
#include "RoboDkBridge.h"

#include "iitem.h"
#include "irobodk.h"
#include "robodktools.h"

#include <QAction>
#include <QDateTime>
#include <QDebug>
#include <QDockWidget>
#include <QMainWindow>
#include <QMenu>
#include <QMenuBar>
#include <QStatusBar>
#include <QStringList>
#include <QTimer>
#include <QToolBar>

namespace {

constexpr int kStepIntervalMs = 16;

// How often the station's programs are checked for having been played. RoboDK has no event for a
// program starting, so this is the only way to notice one, and it runs whether or not a simulation
// is going - a run that an operator stopped is exactly the one a play has to bring back.
constexpr int kRunControlIntervalMs = 25;

constexpr int kPanelSyncIntervalMs = 100;

// Which items can take part. RoboDK's frames, targets and programs carry no geometry, so offering
// them a role would only produce a failed export later.
bool itemCanBeSimulated(Item item) {
    if (!item) return false;
    switch (item->Type()) {
    case IItem::ITEM_TYPE_OBJECT:
    case IItem::ITEM_TYPE_TOOL:
        return true;
    default:
        return false;
    }
}

bool itemIsStationDriven(Item item) {
    return item && item->Type() == IItem::ITEM_TYPE_TOOL;
}

} // namespace

PluginPhysics::~PluginPhysics() = default;

QString PluginPhysics::getPluginName() {
    return "Physics Simulation";
}

QString PluginPhysics::PluginName() {
    return getPluginName();
}

QString PluginPhysics::PluginLoad(QMainWindow* mw, QMenuBar* menubar, QStatusBar* statusbar,
                                  RoboDK* rdk, const QString& settings) {
    RDK = rdk;
    MainWindow = mw;
    StatusBar = statusbar;

    qDebug() << "Loading plugin" << PluginName() << "with settings:" << settings;

    m_world.reset(new PhysicsWorld(rdk));
    m_panel = new PhysicsPanel(m_world.get(), mw);
    mw->addDockWidget(Qt::RightDockWidgetArea, m_panel);
    connect(m_panel, &PhysicsPanel::runStateChanged, this,
            [this]() { setSimulationTimerActive(m_world->isRunning()); });

    m_stepTimer = new QTimer(this);
    m_stepTimer->setInterval(kStepIntervalMs);
    connect(m_stepTimer, &QTimer::timeout, this, &PluginPhysics::stepSimulation);

    m_runControlTimer = new QTimer(this);
    m_runControlTimer->setInterval(kRunControlIntervalMs);
    connect(m_runControlTimer, &QTimer::timeout, this, &PluginPhysics::pollStationRunControl);
    m_runControlTimer->start();

    m_panelTimer = new QTimer(this);
    m_panelTimer->setInterval(kPanelSyncIntervalMs);
    connect(m_panelTimer, &QTimer::timeout, this, [this]() {
        if (m_panel && m_world) m_panel->syncItems();
        adoptConveyorsIfRequested();
        // A conveyor is moved by moving the RoboDK parent it hangs under, and RoboDK reports that to
        // nobody - not as an EventChanged on the conveyor, not at all. So the placement is compared here
        // and the drawing re-baked when it has drifted, which is the only way a dragged frame reaches
        // the triangles the plugin paints.
        bool painted = m_world && m_world->syncRailMechanisms();
        if (m_world && m_world->remeasureMovedPlacedItems()) painted = true;
        if (painted && RDK) {
            RDK->Render(RoboDK::RenderScreen);
        }
        syncConveyorPanels();
        // On the same tick and for the same reason: a placed item's pose can change without this plugin
        // being told - the frame it hangs under is dragged, or its axes are driven by a command - and a
        // window showing stale numbers is worse than one showing none.
        syncPlacedItemPanels();
    });
    m_panelTimer->start();

    // Clicking anything the plugin painted. RoboDK hit-tests only geometry it owns, and none of this is
    // added to the station, so this is the only way one of these can be clicked where it stands.
    m_itemPicker = new PlacedItemPicker(mw, rdk, m_world.get(), this);
    connect(m_itemPicker, &PlacedItemPicker::itemPicked, this, [this](const QString& name) {
        const PlacedItem* placed = m_world ? m_world->placedItemNamed(name) : nullptr;
        if (!placed || !placed->item || !RDK) return;
        RDK->setSelection({placed->item});
        RDK->Render(RoboDK::RenderScreen);
        if (StatusBar) StatusBar->showMessage(tr("'%1'").arg(name), 3000);
    });

    m_libraryDock = new LibraryDock(m_world.get(), mw);
    mw->addDockWidget(Qt::LeftDockWidgetArea, m_libraryDock);
    m_libraryDock->hide();
    m_libraryPlacer = new LibraryPlacementTool(mw, rdk, m_world.get(), m_itemPicker, this);
    connect(m_libraryDock, &LibraryDock::armRequested, this,
            [this](const librarycatalogue::Entry& entry) {
                QString reason;
                if (!m_libraryPlacer->arm(entry, &reason) && m_libraryDock) {
                    m_libraryDock->setStatus(tr("%1 cannot be armed: %2")
                                                 .arg(QString::fromStdString(entry.name), reason));
                }
            });
    connect(m_libraryPlacer, &LibraryPlacementTool::statusChanged, this, [this](const QString& text) {
        if (m_libraryDock) m_libraryDock->setStatus(text);
    });
    connect(m_libraryPlacer, &LibraryPlacementTool::committed, this, [this](const QString& name) {
        Q_UNUSED(name)
        if (RDK) RDK->Render(RoboDK::RenderComplete);
        if (m_panel) m_panel->refresh();
        if (m_libraryDock) m_libraryDock->clearArmedRow();
    });
    connect(m_libraryPlacer, &LibraryPlacementTool::disarmed, this, [this]() {
        if (m_libraryDock) m_libraryDock->clearArmedRow();
    });
    connect(m_libraryDock->toggleViewAction(), &QAction::toggled, this, [this](bool shown) {
        if (shown && m_libraryDock) m_libraryDock->reload();
    });

    m_actionShowPanel = m_panel->toggleViewAction();
    m_actionShowPanel->setText(tr("Physics Panel"));

    // Adding one costs an object and a parameter block. There is no drop hook in IAppRoboDK and no way
    // into RoboDK's own library panel, so this is the route in - and RoboDK's own copy, paste and merge
    // of a .rdk are the other, because the description travels with the item.
    m_actionAddConveyor = new QAction(conveyorIcon(), tr("Add Conveyor"), this);
    connect(m_actionAddConveyor, &QAction::triggered, this, &PluginPhysics::addConveyor);

    QMenu* menu = mw->findChild<QMenu*>("menu-Program");
    if (menu == nullptr) menu = menubar->addMenu(tr("Physics Simulation"));
    menu->addAction(m_actionAddConveyor);
    menu->addAction(m_actionShowPanel);
    QAction* showLibrary = m_libraryDock->toggleViewAction();
    showLibrary->setText(tr("Physics Library"));
    menu->addAction(showLibrary);

    if (StatusBar) {
        StatusBar->showMessage(tr("%1 loaded. Right-click an object to give it a physics role.")
                                   .arg(PluginName()), 5000);
    }
    return "";
}

void PluginPhysics::PluginUnload() {
    qDebug() << "Unloading plugin" << PluginName();

    setSimulationTimerActive(false);
    if (m_runControlTimer) m_runControlTimer->stop();
    if (m_panelTimer) m_panelTimer->stop();
    // Before the world goes: stopping restores every item RoboDK owns, and a world destroyed
    // mid-run would leave the station holding simulated poses.
    if (m_world) m_world->stop();

    // Before the docks go: it holds an application event filter and a ghost, and both are about a station
    // that is being let go of.
    if (m_libraryPlacer) {
        m_libraryPlacer->cancel();
        m_libraryPlacer->deleteLater();
        m_libraryPlacer = nullptr;
    }
    if (m_libraryDock) {
        if (MainWindow) MainWindow->removeDockWidget(m_libraryDock);
        m_libraryDock->deleteLater();
        m_libraryDock = nullptr;
    }
    if (m_panel) {
        if (MainWindow) MainWindow->removeDockWidget(m_panel);
        m_panel->deleteLater();
        m_panel = nullptr;
    }
    for (const auto& entry : m_conveyorPanels) {
        if (entry.second) entry.second->deleteLater();
    }
    m_conveyorPanels.clear();
    for (const auto& entry : m_placedItemPanels) {
        if (entry.second) entry.second->deleteLater();
    }
    m_placedItemPanels.clear();
    m_actionShowPanel = nullptr;
    m_world.reset();
}

void PluginPhysics::PluginLoadToolbar(QMainWindow* mw, int icon_size) {
    if (!mw || !m_actionAddConveyor) return;
    QToolBar* toolbar = mw->addToolBar(tr("Physics Simulation"));
    toolbar->setObjectName(QStringLiteral("physics-toolbar"));
    if (icon_size > 0) toolbar->setIconSize(QSize(icon_size, icon_size));
    toolbar->addAction(m_actionAddConveyor);
}

void PluginPhysics::addConveyor() {
    if (!m_world) return;
    QString name;
    QString error;
    if (!m_world->addConveyor(&name, &error)) {
        if (StatusBar) StatusBar->showMessage(tr("Add conveyor: %1").arg(error), 8000);
        return;
    }
    Item added = nullptr;
    for (const RoboDkConveyorHost::Segment& segment : m_world->conveyors()) {
        if (segment.name == name) { added = segment.item; break; }
    }
    if (RDK) {
        if (added) RDK->setSelection({added});
        RDK->Render(RoboDK::RenderComplete);
    }
    if (added) openConveyorPanel(added);
    if (m_panel) m_panel->refresh();
    if (StatusBar) StatusBar->showMessage(tr("Added conveyor '%1'.").arg(name), 6000);
}

bool PluginPhysics::PluginItemClick(Item item, QMenu* menu, TypeClick click_type) {
    if (!m_world) return false;
    const RoboDkConveyorHost::Segment* conveyor = nullptr;
    if (item) {
        for (const RoboDkConveyorHost::Segment& segment : m_world->conveyors()) {
            if (segment.item == item) { conveyor = &segment; break; }
        }
    }
    Item node = conveyor ? conveyor->item : nullptr;
    const PlacedItem* placed = item && !conveyor ? m_world->placedItemFor(item) : nullptr;
    if (click_type == ClickDouble) {
        // Double-clicking a machine to open it is the gesture an operator brings with them, and it is the
        // only one RoboDK reports here that is not a menu. Whether returning true also stops RoboDK
        // opening its own object dialog is not something iapprobodk.h states, so it is claimed and has to
        // be looked at.
        if (conveyor) {
            openConveyorPanel(node);
            return true;
        }
        if (placed) {
            openPlacedItemPanel(item);
            return true;
        }
        return false;
    }
    if (click_type != ClickRight || !menu) return false;
    if (placed) {
        const placedmechanism::AxisKind kind =
            placedmechanism::axisKindOf(placed->scenery.root.get());
        const QString label = kind == placedmechanism::AxisKind::Rail
            ? tr("Rail properties...")
            : (kind == placedmechanism::AxisKind::Robot ? tr("Arm properties...")
                                                       : tr("Placed item properties..."));
        QAction* properties = menu->addAction(categoryIcon(kind), label);
        connect(properties, &QAction::triggered, this, [this, item]() { openPlacedItemPanel(item); },
                Qt::QueuedConnection);
        const QString name = placed->name;
        QAction* remove = menu->addAction(deleteIcon(), tr("Delete %1").arg(name));
        connect(remove, &QAction::triggered, this, [this, name]() {
            QString error;
            if (m_world && !m_world->deleteLibraryItem(name, &error) && StatusBar) {
                StatusBar->showMessage(tr("'%1': %2").arg(name, error), 8000);
            }
            if (m_panel) m_panel->refresh();
            if (RDK) RDK->Render(RoboDK::RenderComplete);
        }, Qt::QueuedConnection);
        return true;
    }
    if (conveyor) {
        const QString name = conveyor->name;
        connect(menu->addAction(conveyorIcon(), tr("Conveyor properties...")), &QAction::triggered,
                this, [this, node]() { openConveyorPanel(node); }, Qt::QueuedConnection);
        QAction* remove = menu->addAction(deleteIcon(), tr("Delete conveyor"));
        remove->setEnabled(!m_world->isRunning());
        connect(remove, &QAction::triggered, this, [this, name]() {
            QString error;
            if (m_world && !m_world->deleteConveyor(name, &error) && StatusBar) {
                StatusBar->showMessage(tr("Conveyor '%1': %2").arg(name, error), 8000);
            }
            if (m_panel) m_panel->refresh();
            if (RDK) RDK->Render(RoboDK::RenderComplete);
        }, Qt::QueuedConnection);
        return true;
    }
    if (item && item->Type() == IItem::ITEM_TYPE_ROBOT) {
        QAction* explanation = menu->addAction(tr("Physics: mark this robot's tool instead"));
        explanation->setEnabled(false);
        return true;
    }
    if (!itemCanBeSimulated(item)) return false;

    QMenu* roleMenu = menu->addMenu(tr("Physics Role"));
    const PhysicsWorld::Role current = m_world->roleOf(item);

    const auto addRole = [&](const QString& label, PhysicsWorld::Role role) {
        QAction* action = roleMenu->addAction(label);
        action->setCheckable(true);
        action->setChecked(current == role);
        action->setEnabled(!m_world->isRunning());
        connect(action, &QAction::triggered, this, [this, item, role]() {
            m_world->setRole(item, role);
            if (m_panel) m_panel->refresh();
        }, Qt::QueuedConnection);
    };

    addRole(tr("None"), PhysicsWorld::Role::None);
    if (itemIsStationDriven(item)) {
        // A robot or tool is moved by RoboDK's own motion; letting the solver own its pose would
        // put the two in a fight the station always loses.
        addRole(tr("Kinematic (driven by RoboDK)"), PhysicsWorld::Role::Kinematic);
    } else {
        addRole(tr("Static (immovable)"), PhysicsWorld::Role::Static);
        addRole(tr("Dynamic (simulated)"), PhysicsWorld::Role::Dynamic);
        addRole(tr("Kinematic (driven by RoboDK)"), PhysicsWorld::Role::Kinematic);
    }
    return true;
}

QString PluginPhysics::PluginCommand(const QString& command, const QString& value) {
    if (!m_world) return "Physics plugin is not initialised.";
    const QString verb = command.trimmed().toLower();
    const QString argument = value.trimmed();

    if (verb == QLatin1String("start")) {
        QString error;
        const bool started = m_world->start(&error);
        if (m_panel) m_panel->refresh();
        setSimulationTimerActive(m_world->isRunning());
        return started ? "OK" : error;
    }
    if (verb == QLatin1String("stop")) {
        m_world->stop();
        if (m_panel) m_panel->refresh();
        setSimulationTimerActive(false);
        return "OK";
    }
    if (verb == QLatin1String("pause")) {
        m_world->pause();
        if (m_panel) m_panel->refresh();
        setSimulationTimerActive(false);
        return "OK";
    }
    if (verb == QLatin1String("status")) return m_world->summary();
    if (verb == QLatin1String("items")) return m_world->itemReport();
    if (verb == QLatin1String("conveyors")) return m_world->conveyorReport();
    if (verb == QLatin1String("library")) return m_world->libraryReport();
    if (verb == QLatin1String("librarylist")) return m_world->catalogueReport();
    if (verb == QLatin1String("saveconfig")) {
        if (RDK) RDK->setParam(kConfigParam, m_world->configString());
        return QStringLiteral("OK %1").arg(m_world->configString());
    }
    if (verb == QLatin1String("loadconfig")) {
        loadStationConfiguration();
        return QStringLiteral("OK %1").arg(m_world->summary());
    }

    const auto itemNamed = [this](const QString& name) -> Item {
        return (name.isEmpty() || !RDK) ? nullptr : RDK->getItem(name);
    };

    if (verb == QLatin1String("spawn")) {
        // "<prototype>" or "<prototype>|<parent>". The clone lands on the prototype's own pose,
        // which for a spawn template is exactly where boxes should appear.
        const QStringList parts = argument.split(QLatin1Char('|'));
        Item prototype = itemNamed(parts.value(0).trimmed());
        if (!prototype) return QStringLiteral("No item named '%1'.").arg(parts.value(0).trimmed());
        Item parent = parts.size() > 1 ? itemNamed(parts.value(1).trimmed()) : nullptr;
        QString name;
        QString error;
        const CadTransform pose = rdkbridge::toCadTransform(prototype->PoseAbs());
        if (!m_world->spawn(prototype, parent, pose, &name, &error)) return error;
        if (m_panel) m_panel->refresh();
        return QStringLiteral("OK %1").arg(name);
    }
    if (verb == QLatin1String("attach")) {
        Item item = itemNamed(argument);
        if (!item) return QStringLiteral("No item named '%1'.").arg(argument);
        QString error;
        const bool done = m_world->attach(item, &error);
        if (m_panel) m_panel->refresh();
        return done ? "OK" : error;
    }
    if (verb == QLatin1String("release")) {
        Item item = itemNamed(argument);
        if (!item) return QStringLiteral("No item named '%1'.").arg(argument);
        QString error;
        const bool done = m_world->release(item, &error);
        if (m_panel) m_panel->refresh();
        return done ? "OK" : error;
    }
    if (verb == QLatin1String("spawner")) {
        const QStringList parts = argument.split(QLatin1Char('|'));
        Item prototype = itemNamed(parts.value(0).trimmed());
        if (!prototype) return QStringLiteral("No item named '%1'.").arg(parts.value(0).trimmed());
        Item parent = parts.size() > 1 ? itemNamed(parts.value(1).trimmed()) : nullptr;
        bool ok = false;
        const double interval = parts.value(2).trimmed().toDouble(&ok);
        const int cap = parts.value(3).trimmed().toInt();
        m_world->configureSpawner(prototype, parent, ok ? interval : 1.0, cap);
        if (m_panel) m_panel->refresh();
        return "OK";
    }
    if (verb == QLatin1String("conveyor")) {
        // Fields describe one conveyor item and are persisted on that item.
        RoboDkConveyorHost::Segment segment;
        QString error;
        if (!parseConveyorSegment(RDK, argument.split(QLatin1Char('|')), &segment, &error)) {
            return error;
        }
        if (!m_world->configureConveyor(segment, &error)) return error;
        if (m_panel) m_panel->refresh();
        return "OK";
    }
    if (verb == QLatin1String("addconveyor")) {
        // Replies with RoboDK's unique item name.
        QString name;
        QString error;
        if (!m_world->addConveyor(&name, &error)) return error;
        if (m_panel) m_panel->refresh();
        return QStringLiteral("OK %1").arg(name);
    }
    if (verb == QLatin1String("moveconveyor")) {
        // Parent-relative XYZRPW pose in millimetres and degrees.
        const QStringList fields = argument.split(QLatin1Char('|'));
        if (fields.size() < 7) {
            return QStringLiteral("moveconveyor needs <conveyor>|<x>|<y>|<z>|<rx>|<ry>|<rz>.");
        }
        tXYZWPR values;
        for (int at = 0; at < 6; ++at) {
            bool number = false;
            values[at] = fields.value(at + 1).trimmed().toDouble(&number);
            if (!number) {
                return QStringLiteral("'%1' is not a number.").arg(fields.value(at + 1).trimmed());
            }
        }
        QString error;
        if (!m_world->setConveyorPlacement(fields.value(0).trimmed(),
                                           rdkbridge::toCadTransform(Mat::XYZRPW_2_Mat(values)),
                                           &error)) {
            return error;
        }
        if (RDK) RDK->Render(RoboDK::RenderScreen);
        return "OK";
    }
    if (verb == QLatin1String("libraryroot")) {
        m_world->setLibraryRoot(argument.trimmed());
        return QStringLiteral("OK %1").arg(m_world->libraryRoot());
    }
    if (verb == QLatin1String("libraryplace")) {
        // Package, optional variant, and parent-relative XYZRPW pose.
        const QStringList fields = argument.split(QLatin1Char('|'));
        if (fields.size() < 8) {
            return QStringLiteral(
                "libraryplace needs <package>|<variant>|<x>|<y>|<z>|<rx>|<ry>|<rz>.");
        }
        tXYZWPR values;
        for (int at = 0; at < 6; ++at) {
            bool number = false;
            values[at] = fields.value(at + 2).trimmed().toDouble(&number);
            if (!number) {
                return QStringLiteral("'%1' is not a number.").arg(fields.value(at + 2).trimmed());
            }
        }
        LibraryItemSpec spec;
        spec.packageRef = fields.value(0).trimmed();
        spec.variantId = fields.value(1).trimmed();
        spec.poseLocal = rdkbridge::toCadTransform(Mat::XYZRPW_2_Mat(values));
        spec.hasPose = true;
        QString name;
        QString error;
        if (!m_world->placeLibraryItem(spec, &name, &error)) return error;
        if (RDK) RDK->Render(RoboDK::RenderScreen);
        return QStringLiteral("OK %1").arg(name);
    }
    if (verb == QLatin1String("librarysnap")) {
        // Headless snap route; the final three fields are optional filters.
        const QStringList fields = argument.split(QLatin1Char('|'));
        if (fields.value(0).trimmed().isEmpty()) {
            return QStringLiteral("librarysnap needs <package>|<variant>|<near>, and optionally "
                                  "|<source interface>|<target interface>.");
        }
        QString report;
        m_world->snapLibraryItem(fields.value(0).trimmed(), fields.value(1).trimmed(),
                                 fields.value(2).trimmed(), fields.value(3).trimmed(),
                                 fields.value(4).trimmed(), &report);
        if (RDK) RDK->Render(RoboDK::RenderComplete);
        if (m_panel) m_panel->refresh();
        return report;
    }
    if (verb == QLatin1String("conveyorworkpiece")) {
        const QStringList fields = argument.split(QLatin1Char('|'));
        if (fields.value(0).trimmed().isEmpty()) {
            return QStringLiteral("conveyorworkpiece needs <conveyor>|<object>.");
        }
        QString error;
        if (!m_world->setConveyorWorkpiece(fields.value(0).trimmed(), fields.value(1).trimmed(),
                                           &error)) {
            return error;
        }
        if (m_panel) m_panel->refresh();
        if (RDK) RDK->Render(RoboDK::RenderScreen);
        return "OK";
    }
    if (verb == QLatin1String("placedjoints")) {
        const QStringList fields = argument.split(QLatin1Char('|'));
        if (fields.size() < 2) {
            return QStringLiteral("placedjoints needs <name>|j1|...|j6 for an arm, or <name>|<mm> for "
                                  "a rail.");
        }
        std::vector<double> values;
        for (int at = 1; at < fields.size(); ++at) {
            bool number = false;
            values.push_back(fields.value(at).trimmed().toDouble(&number));
            if (!number) {
                return QStringLiteral("'%1' is not a number.").arg(fields.value(at).trimmed());
            }
        }
        QString report;
        QString error;
        if (!m_world->setPlacedItemAxes(fields.value(0).trimmed(), values, &report, &error)) {
            return error;
        }
        if (RDK) RDK->Render(RoboDK::RenderScreen);
        return report;
    }
    if (verb == QLatin1String("librarydelete")) {
        QString error;
        if (!m_world->deleteLibraryItem(argument.trimmed(), &error)) return error;
        if (RDK) RDK->Render(RoboDK::RenderScreen);
        return "OK";
    }
    if (verb == QLatin1String("libraryrename")) {
        const QStringList fields = argument.split(QLatin1Char('|'));
        if (fields.size() < 2 || fields.value(0).trimmed().isEmpty()) {
            return QStringLiteral("libraryrename needs <name>|<wanted>.");
        }
        const PlacedItem* placed = m_world->placedItemNamed(fields.value(0).trimmed());
        if (!placed) {
            return QStringLiteral("There is no placed item called '%1'.").arg(fields.value(0).trimmed());
        }
        QString error;
        if (!m_world->renameLibraryItem(placed->item, fields.value(1).trimmed(), &error)) return error;
        const PlacedItem* renamed = m_world->placedItemFor(placed->item);
        if (m_panel) m_panel->refresh();
        if (RDK) RDK->Render(RoboDK::RenderScreen);
        return QStringLiteral("OK %1").arg(renamed ? renamed->name : QString());
    }
    if (verb == QLatin1String("deleteconveyor")) {
        // Zeroes its three IO variables, forgets it, then deletes its frame - and its products go with
        // the frame, because they are its children.
        QString error;
        if (!m_world->deleteConveyor(argument, &error)) return error;
        if (m_panel) m_panel->refresh();
        return "OK";
    }
    if (verb == QLatin1String("clearspawners") || verb == QLatin1String("clearconveyors")) {
        m_world->clearConveyors();
        if (m_panel) m_panel->refresh();
        return "OK";
    }
    if (verb == QLatin1String("clearspawned")) {
        m_world->clearSpawned();
        if (m_panel) m_panel->refresh();
        return "OK";
    }
    if (verb == QLatin1String("role")) {
        const QStringList parts = argument.split(QLatin1Char('|'));
        Item item = itemNamed(parts.value(0).trimmed());
        if (!item) return QStringLiteral("No item named '%1'.").arg(parts.value(0).trimmed());
        const QString wanted = parts.value(1).trimmed().toLower();
        PhysicsWorld::Role role = PhysicsWorld::Role::None;
        if (wanted == QLatin1String("static")) role = PhysicsWorld::Role::Static;
        else if (wanted == QLatin1String("dynamic")) role = PhysicsWorld::Role::Dynamic;
        else if (wanted == QLatin1String("kinematic")) role = PhysicsWorld::Role::Kinematic;
        else if (wanted != QLatin1String("none")) return QStringLiteral("Unknown role '%1'.").arg(wanted);
        m_world->setRole(item, role);
        if (m_panel) m_panel->refresh();
        return "OK";
    }
    return QStringLiteral("Unknown command '%1'.").arg(command);
}

void PluginPhysics::loadStationConfiguration() {
    if (!m_world || !RDK) return;
    m_programWasRunning = false;
    const QString text = RDK->getParam(kConfigParam);

    QStringList warnings;
    m_world->applyConfigString(text, &warnings);
    for (const QString& warning : warnings) {
        qDebug() << "Physics configuration:" << warning;
    }
    if (StatusBar && !warnings.isEmpty()) {
        StatusBar->showMessage(tr("Physics configuration: %1").arg(warnings.join(QStringLiteral("; "))),
                               8000);
    }
    if (m_world->autoStart()) {
        QString error;
        m_world->start(&error);
        setSimulationTimerActive(m_world->isRunning());
    }
    if (m_panel) m_panel->refresh();
}

bool PluginPhysics::anyProgramRunning() {
    if (!RDK) return false;
    for (const int filter : {IItem::ITEM_TYPE_PROGRAM, IItem::ITEM_TYPE_PROGRAM_PYTHON}) {
        for (Item program : RDK->getItemList(filter)) {
            if (program && RDK->Valid(program) && program->Busy()) return true;
        }
    }
    return false;
}

void PluginPhysics::pollStationRunControl() {
    if (!m_world) return;
    const bool running = anyProgramRunning();
    const bool played = running && !m_programWasRunning;
    m_programWasRunning = running;
    if (!played || !m_world->autoStart()) return;

    // Each play starts from a clean physics run, including Reset programs.
    m_world->stop();
    QString error;
    if (!m_world->start(&error)) qDebug() << "Physics run control:" << error;
    setSimulationTimerActive(m_world->isRunning());
    if (m_panel) m_panel->refresh();
}

QStringList PluginPhysics::workpieceCandidates() {
    QStringList names;
    if (!RDK) return names;
    // Objects only: a frame, a target or a program carries no geometry, so offering one as a workpiece
    // would produce a failed export later. Held as names because that is how the parameter block names
    // one - RoboDK resolves an object by name and a station parameter outlives the session.
    for (Item item : RDK->getItemList(IItem::ITEM_TYPE_OBJECT)) {
        if (item && RDK->Valid(item)) names << item->Name();
    }
    names.sort();
    names.removeDuplicates();
    return names;
}

void PluginPhysics::adoptConveyorsIfRequested() {
    if (!m_conveyorAdoptionRequested || !m_world) return;
    m_conveyorAdoptionRequested = false;
    QStringList warnings;
    m_world->adoptConveyorItems(&warnings);
    // Both kinds of placed item stand up on the same request, because both arrive the same way: an
    // opened station, a paste, or a merged .rdk. Requested from EventChanged and done here, because
    // that event comes synchronously from inside the RoboDK calls a commit makes.
    m_world->adoptLibraryItems(&warnings);
    for (const QString& warning : warnings) qDebug() << "Physics:" << warning;
    if (StatusBar && !warnings.isEmpty()) {
        StatusBar->showMessage(tr("Physics: %1").arg(warnings.join(QStringLiteral("; "))), 8000);
    }
}

void PluginPhysics::openConveyorPanel(Item item) {
    if (!item || !m_world || !MainWindow) return;
    auto existing = m_conveyorPanels.find(item);
    if (existing == m_conveyorPanels.end()) {
        auto* panel = new ConveyorPropertiesPanel(MainWindow);
        panel->setWindowFlags(panel->windowFlags() | Qt::Window);
        connect(panel, &ConveyorPropertiesPanel::parametersEdited, this,
                [this](const QString& conveyor, const TransformNodeData& parameters) {
                    QString error;
                    if (m_world && !m_world->applyConveyorParameters(conveyor, parameters, &error) &&
                        StatusBar) {
                        StatusBar->showMessage(tr("Conveyor '%1': %2").arg(conveyor, error), 8000);
                    }
                    if (RDK) RDK->Render(RoboDK::RenderComplete);
                });
        connect(panel, &ConveyorPropertiesPanel::placementEdited, this,
                [this](const QString& conveyor, const CadTransform& poseLocal) {
                    QString error;
                    if (m_world && !m_world->setConveyorPlacement(conveyor, poseLocal, &error) &&
                        StatusBar) {
                        StatusBar->showMessage(tr("Conveyor '%1': %2").arg(conveyor, error), 8000);
                    }
                    // RenderScreen: what moved is where the plugin *paints* the conveyor, so a redisplay
                    // is exactly what is needed - and it is what provokes the EventRender that painting
                    // happens in. RenderUpdateOnly updates the positions of modified items and explicitly
                    // does not redisplay (irobodk.h:359-363), so it left the conveyor drawn where it used
                    // to be until something else repainted.
                    if (RDK) RDK->Render(RoboDK::RenderScreen);
                });
        connect(panel, &ConveyorPropertiesPanel::renameRequested, this,
                [this](const QString& conveyor, const QString& wanted) {
                    QString error;
                    if (m_world && !m_world->renameConveyor(conveyor, wanted, &error) && StatusBar) {
                        StatusBar->showMessage(tr("Conveyor '%1': %2").arg(conveyor, error), 8000);
                    }
                });
        existing = m_conveyorPanels.emplace(item, panel).first;
    }
    // Shown before syncing, because syncing skips hidden panels - otherwise a newly opened window would
    // read empty until the next timer tick.
    existing->second->show();
    m_workpieceCandidates = workpieceCandidates();
    syncConveyorPanels();
    existing->second->raise();
    existing->second->activateWindow();
}

void PluginPhysics::syncConveyorPanels() {
    if (m_conveyorPanels.empty() || !m_world) return;
    const bool stopped = m_world->state() == PhysicsWorld::State::Stopped;
    for (auto entry = m_conveyorPanels.begin(); entry != m_conveyorPanels.end();) {
        const RoboDkConveyorHost::Segment* shown = nullptr;
        for (const RoboDkConveyorHost::Segment& segment : m_world->conveyors()) {
            if (segment.item == entry->first) { shown = &segment; break; }
        }
        if (!shown) {
            // The panel key is invalid after RoboDK deletes the item.
            if (entry->second) entry->second->deleteLater();
            entry = m_conveyorPanels.erase(entry);
            continue;
        }
        // Hidden panels are kept, so reopening one is instant - but syncing one costs widget writes for
        // rows nobody is looking at.
        if (entry->second->isVisible()) {
            entry->second->setWindowTitle(tr("Conveyor - %1").arg(shown->name));
            entry->second->showConveyor(shown, stopped, m_workpieceCandidates);
        }
        ++entry;
    }
}

void PluginPhysics::openPlacedItemPanel(Item item) {
    if (!item || !m_world || !MainWindow) return;
    auto existing = m_placedItemPanels.find(item);
    if (existing == m_placedItemPanels.end()) {
        auto* panel = new PlacedItemPropertiesPanel(MainWindow);
        panel->setWindowFlags(panel->windowFlags() | Qt::Window);
        connect(panel, &PlacedItemPropertiesPanel::placementEdited, this,
                [this](Item edited, const CadTransform& poseLocal) {
                    QString error;
                    if (m_world && !m_world->setLibraryItemPlacementFor(edited, poseLocal, &error) &&
                        StatusBar) {
                        StatusBar->showMessage(error, 8000);
                    }
                    // RenderScreen, for the reason every render here is: what moved is where the plugin
                    // *paints* this, and only a redisplay provokes the EventRender that painting happens
                    // in. RenderUpdateOnly does not redisplay (irobodk.h:359-363).
                    if (RDK) RDK->Render(RoboDK::RenderScreen);
                });
        connect(panel, &PlacedItemPropertiesPanel::axesEdited, this,
                [this](Item edited, const std::vector<double>& values) {
                    QString error;
                    QString report;
                    if (m_world &&
                        !m_world->setPlacedItemAxesFor(edited, values, &report, &error) && StatusBar) {
                        StatusBar->showMessage(error, 8000);
                    }
                    if (RDK) RDK->Render(RoboDK::RenderScreen);
                });
        connect(panel, &PlacedItemPropertiesPanel::renameRequested, this,
                [this](Item edited, const QString& wanted) {
                    QString error;
                    if (m_world && !m_world->renameLibraryItem(edited, wanted, &error) && StatusBar) {
                        StatusBar->showMessage(error, 8000);
                    }
                    if (m_panel) m_panel->refresh();
                    if (RDK) RDK->Render(RoboDK::RenderScreen);
                });
        existing = m_placedItemPanels.emplace(item, panel).first;
    }
    // Shown before syncing, because syncing skips hidden windows - otherwise a newly opened one would
    // read empty until the next timer tick.
    existing->second->show();
    syncPlacedItemPanels();
    existing->second->raise();
    existing->second->activateWindow();
}

void PluginPhysics::syncPlacedItemPanels() {
    if (m_placedItemPanels.empty() || !m_world) return;
    const bool stopped = m_world->state() == PhysicsWorld::State::Stopped;
    for (auto entry = m_placedItemPanels.begin(); entry != m_placedItemPanels.end();) {
        const PlacedItem* shown = m_world->placedItemFor(entry->first);
        if (!shown) {
            // The panel key is invalid after RoboDK deletes the item.
            if (entry->second) entry->second->deleteLater();
            entry = m_placedItemPanels.erase(entry);
            continue;
        }
        if (entry->second->isVisible()) {
            entry->second->setWindowTitle(shown->name);
            entry->second->showPlacedItem(shown, stopped);
        }
        ++entry;
    }
}

void PluginPhysics::PluginEvent(TypeEvent event_type) {
    if (event_type == EventChangedStation) {
        if (m_world) m_world->stop();
        setSimulationTimerActive(false);
        if (m_libraryPlacer) m_libraryPlacer->cancel();
        loadStationConfiguration();
        if (m_libraryDock && m_libraryDock->isVisible()) m_libraryDock->reload();
        return;
    }
    switch (event_type) {
    case EventRender:
        if (m_world) m_world->drawPlacedItems();
        if (m_itemPicker) m_itemPicker->drawHighlight();
        if (m_libraryPlacer) m_libraryPlacer->draw();
        break;
    case EventChanged:
        // Simulation updates also emit EventChanged, so this event must not stop the run.
        if (m_world) m_world->pruneInvalidItems();
        // Defer adoption because EventChanged may arrive during a simulation iteration.
        m_conveyorAdoptionRequested = true;
        // refresh() rebuilds the item tree, which at this event's rate would discard the operator's
        // selection continuously. Roles cannot change mid-run, so the status line is the only part
        // that has anything new to say.
        if (m_panel) {
            if (m_world && m_world->state() != PhysicsWorld::State::Stopped) m_panel->refreshStatus();
            else m_panel->refresh();
        }
        break;
    case EventAbout2Save:
        // RoboDK saves per-item data with the station, so a conveyor is saved by RoboDK saving - and
        // this is the last moment an edit made through the panel can still reach the file. Nothing is
        // written for a conveyor whose bytes are unchanged, so an ordinary save costs nothing.
        if (m_world) m_world->flushConveyorParameters();
        break;
    case EventAbout2ChangeStation:
    case EventAbout2CloseStation:
        // The station itself is going away. A run cannot outlive the station it was built from, and
        // stopping restores the poses before RoboDK stops being able to accept them.
        if (m_world) {
            if (m_world->state() != PhysicsWorld::State::Stopped) m_world->stop();
            m_world->pruneInvalidItems();
        }
        if (m_libraryPlacer) m_libraryPlacer->cancel();
        setSimulationTimerActive(false);
        if (m_panel) m_panel->refresh();
        break;
    default:
        break;
    }
}

void PluginPhysics::setSimulationTimerActive(bool active) {
    if (!m_stepTimer) return;
    if (active && !m_stepTimer->isActive()) {
        m_lastStepMs = QDateTime::currentMSecsSinceEpoch();
        m_stepTimer->start();
    } else if (!active && m_stepTimer->isActive()) {
        m_stepTimer->stop();
    }
}

void PluginPhysics::stepSimulation() {
    if (!m_world || !m_world->isRunning()) {
        setSimulationTimerActive(false);
        return;
    }
    const qint64 now = QDateTime::currentMSecsSinceEpoch();
    const double elapsed = qBound(0.0, static_cast<double>(now - m_lastStepMs) / 1000.0, 0.1);
    m_lastStepMs = now;
    if (m_world->step(elapsed) && RDK) RDK->Render(RoboDK::RenderComplete);
    if (m_panel) m_panel->refreshStatus();
}
