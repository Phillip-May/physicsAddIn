#ifndef PHYSICSWORLD_H
#define PHYSICSWORLD_H

#include "ConveyorCore.h"
#include "ConveyorPhysics.h"
#include "RoboDkBridge.h"
#include "LibraryItems.h"
#include "LibraryPlacer.h"
#include "RoboDkConveyorHost.h"
#include "robodktypes.h"

#include <QString>

#include <vector>

class IRoboDK;

// Station parameter holding the cell's physics setup, saved with the .rdk. Conveyors are not in it;
// they are items carrying their own description. See docs/conveyors.md.
extern const char* const kConfigParam;

// RoboDK owns item poses until start() and gets them back on stop().
class PhysicsWorld {
public:
    enum class Role {
        None,
        Static,     // Immovable collision geometry cooked from the item's mesh.
        Dynamic,    // PhysX owns its pose while running.
        Kinematic   // Driven by RoboDK. Moves dynamic bodies and is never moved by them.
    };

    struct Participant {
        Item item = nullptr;
        QString name;
        Role role = Role::None;
        ConveyorPhysics::BodyHandle body = 0;
        CadTransform restorePose;
        bool restoreValid = false;
        size_t triangles = 0;
        QString status;

        // The station owns a carried pose, so there is no body. A box riding a conveyor is carried
        // and held by nothing; the handoff triggers on the change, not the state.
        bool carried = false;
        bool heldByTool = false;
        bool spawned = false;
        std::vector<CadVec3> hullPoints;    // Rebuilds a body on release without re-exporting.
        bool hullValid = false;
        Item originSpawner = nullptr;       // For the cap. Unlike the parent, never reassigned.
    };

    // Every declared conveyor publishes three station variables, which RoboDK's native
    // "Wait for I/O" instruction can read:
    //
    //     <name> ready   1 while a workpiece rests at the end stop, ungrasped
    //     <name> full    1 while there is no room at the entry
    //     <name> count   how many workpieces the conveyor carries
    struct ConveyorIo {
        QString conveyor;   // Also the variables' prefix.
        int ready = -1;
        int full = -1;
        int count = -1;
    };

    enum class State { Stopped, Running, Paused };

    explicit PhysicsWorld(RoboDK* rdk);
    ~PhysicsWorld();

    static QString roleName(Role role);

    PhysicsWorld(const PhysicsWorld&) = delete;
    PhysicsWorld& operator=(const PhysicsWorld&) = delete;

    void setRole(Item item, Role role);
    Role roleOf(Item item) const;
    void forget(Item item);
    void pruneInvalidItems();

    const std::vector<Participant>& participants() const { return m_participants; }
    State state() const { return m_state; }
    bool isRunning() const { return m_state == State::Running; }

    void setGravity(const CadVec3& gravityMps2) { m_gravityMps2 = gravityMps2; }
    CadVec3 gravity() const { return m_gravityMps2; }

    // Per-item problems go to that item's status, not to `errorMessage`.
    bool start(QString* errorMessage);
    void pause();
    void resume();
    void stop();
    bool step(double seconds);          // Whether anything moved, so a render can be skipped.

    // ---- Logical/physical handoff, driven by the station's own programs ----------------------

    Item spawn(Item prototype, Item parent, const CadTransform& pose, QString* outName,
               QString* errorMessage);
    bool attach(Item item, QString* errorMessage);
    // Re-parented to the station root first, keeping the absolute pose.
    bool release(Item item, QString* errorMessage);
    void clearSpawned();

    // ---- Conveyors, over Common/ConveyorCore's rules. See docs/conveyors.md. -----------------

    // Identity is the item. False when a second item claims a name a conveyor already has.
    bool configureConveyor(const RoboDkConveyorHost::Segment& segment, QString* error = nullptr);
    // The one route in. Unchanged items are left alone: re-declaring re-exports the mesh.
    void adoptConveyorItems(QStringList* warnings = nullptr);
    Item standUpConveyor(Item was, CadTransform* outPoseLocal, QString* error);
    // RoboDK saves per-item data with the station, so this must happen before it does.
    void flushConveyorParameters();
    bool addConveyor(QString* outName = nullptr, QString* error = nullptr);
    // IO dark, then forgotten, then the item goes, taking its products with it as its children.
    bool deleteConveyor(const QString& conveyor, QString* error = nullptr);
    const RoboDkConveyorHost::Segment* conveyorNamed(const QString& name) const;
    // Relative to the parent the item hangs under. RoboDK's own move tools cannot reach it.
    bool setConveyorPlacement(const QString& conveyor, const CadTransform& poseLocal,
                              QString* error = nullptr);
    bool applyConveyorParameters(const QString& conveyor, const TransformNodeData& parameters,
                                 QString* error = nullptr);
    bool setConveyorWorkpiece(const QString& conveyor, const QString& object, QString* error = nullptr);
    bool renameConveyor(const QString& conveyor, const QString& wanted, QString* error = nullptr);
    // A spawner with no path, whose products the station carries.
    void configureSpawner(Item prototype, Item parent, double intervalSeconds, int maxActiveSpawns);
    void clearConveyors();
    const std::vector<RoboDkConveyorHost::Segment>& conveyors() const {
        return m_conveyorHost.segments();
    }

    const std::vector<ConveyorIo>& conveyorIo() const { return m_conveyorIo; }


    // Pointers into storage rebuilt on the panel timer. Hold for one render or one click.
    std::vector<const PlacedItem*> placedItems() const;
    const PlacedItem* placedItemNamed(const QString& name) const;
    const PlacedItem* placedItemFor(Item item) const;
    std::vector<PlacedLibraryItem>& libraryItems() { return m_libraryItems; }
    bool placeLibraryItem(const LibraryItemSpec& spec, QString* outName, QString* error = nullptr,
                          Item* outItem = nullptr);
    bool deleteLibraryItem(const QString& name, QString* error = nullptr);
    bool setLibraryItemPlacement(const QString& name, const CadTransform& poseLocal,
                                 QString* error = nullptr);
    bool setLibraryItemPlacementFor(Item item, const CadTransform& poseLocal,
                                    QString* error = nullptr);
    // Degrees per joint for an arm, millimetres for a rail. `report` states the flange in the arm's
    // own base and that base in station millimetres, the only frames two binaries share.
    bool setPlacedItemAxes(const QString& name, const std::vector<double>& values, QString* report,
                           QString* error = nullptr);
    bool setPlacedItemAxesFor(Item item, const std::vector<double>& values, QString* report,
                              QString* error = nullptr);
    bool renameLibraryItem(Item item, const QString& wanted, QString* error = nullptr);
    // On the panel timer: neither host is told about the other.
    bool syncRailMechanisms();
    bool commitPlacement(const ArmedPackage& armed, const CadTransform* treeToStation,
                         QString* outName, QString* error = nullptr);
    // Each entry carries a loaded tree, so hold for one dock list or one command.
    std::vector<librarycatalogue::Entry> libraryCatalogue() const;
    // RoboDK's own objects carry no mounting metadata and cannot be among these.
    void collectSnapTargets(std::vector<mountingsnap::Interface>* targets) const;
    // The one target a single declared point may satisfy. Runtime-only, in no station.
    const CadNode* floorNode() const { return m_floor.get(); }
    void recentreFloorGrid(const mountingsnap::View& view, double cursorX, double cursorY);
    void collectFloorPoints(std::vector<CadVec3>* pointsMm) const;
    bool snapLibraryItem(const QString& package, const QString& variant, const QString& mateTo,
                         const QString& sourceId, const QString& targetId, QString* report);
    void adoptLibraryItems(QStringList* warnings = nullptr);
    QString libraryRoot() const;
    static QString builtinLibraryRoot();
    void setLibraryRoot(const QString& root) { m_libraryRoot = root; }
    // On placement and on move, never per frame.
    void bakePlacedItem(PlacedItem& placed, const CadTransform& stand);
    // Only from a PluginEvent of type EventRender, where the main OpenGL context is current.
    void drawPlacedItems();
    // Nothing reports a moved parent, so this runs on the panel timer.
    bool remeasureMovedPlacedItems();

    bool hasBody(Item item) const;
    bool bodyPoseOf(Item item, CadTransform* world) const;
    void removeItem(Item item);

    // ---- Configuration, which outlives the session that wrote it -----------------------------

    QString configString() const;
    void applyConfigString(const QString& text, QStringList* warnings = nullptr);
    bool autoStart() const { return m_autoStart; }
    void setAutoStart(bool autoStart) { m_autoStart = autoStart; }

    const QString& lastError() const { return m_lastError; }
    QString summary() const;

    // No check can open a window. One line each, " || " separated records, because RoboDK reads a
    // PluginCommand reply up to the first newline.
    QString itemReport() const;
    QString conveyorReport() const;
    QString libraryReport() const;
    QString catalogueReport() const;

private:
    Participant* find(Item item);
    void releaseScene();
    std::vector<conveyorcore::ConveyorId> spawningConveyors() const;
    bool hasSpawningConveyor() const { return !spawningConveyors().empty(); }
    void syncSpawners();
    bool createDynamicBody(Participant& participant, QString* errorMessage);
    const Participant* findConst(Item item) const;
    // At any depth. How a native "Attach to <tool>" instruction is noticed.
    bool itemIsHeldByTool(Item item) const;
    // Without this the handoff is closed to anything the plugin did not spawn.
    void adoptToolChildren();
    void updateCarryStates();
    QString nextSpawnName(const QString& prototypeName);
    bool migrateConveyorEntry(const QString& value, QStringList* warnings);
    void pushRailMechanismPose(PlacedLibraryItem& placed);
    void pushRailMechanismJoint(PlacedLibraryItem& placed);
    void standLibraryItem(PlacedLibraryItem& placed, const CadTransform& poseLocal);
    bool drivePlacedItem(PlacedLibraryItem& placed, const std::vector<double>& values, QString* report,
                         QString* error);
    // `base`, then `base 2`. RoboDK does not uniquify a generic item's name on AddItem or setName.
    QString freeItemName(const QString& base) const;
    QString freeConveyorName() const;
    QString cappedSpawnerReport(const RoboDkConveyorHost::Segment& segment) const;
    // Only changed values. Forced to 0 on stop, or a station waits on one nothing writes again.
    void publishConveyorIo();
    void darkenConveyorIo(const QString& conveyor);
    void forgetConveyor(conveyorcore::ConveyorId id);

    RoboDK* m_rdk = nullptr;
    ConveyorPhysics m_physics;
    std::vector<Participant> m_participants;
    RoboDkConveyorHost m_conveyorHost;      // Declared before the runtime that points at it.
    conveyorcore::Runtime m_conveyors;
    std::vector<ConveyorIo> m_conveyorIo;
    std::vector<PlacedLibraryItem> m_libraryItems;
    QString m_libraryRoot;
    std::shared_ptr<CadNode> m_floor;
    State m_state = State::Stopped;
    CadVec3 m_gravityMps2{0.0, 0.0, -9.81};
    QString m_lastError;
    size_t m_staticCount = 0;
    size_t m_dynamicCount = 0;
    size_t m_kinematicCount = 0;
    size_t m_carriedCount = 0;
    unsigned m_spawnSerial = 0;             // Monotonic per session, not per run: names collide.
    uint64_t m_stepCount = 0;               // Per run, zeroed by start().
    double m_steppedSeconds = 0.0;
    // A step re-enters through RoboDK's synchronous events, so both are deferred.
    bool m_stepping = false;
    bool m_pruneRequested = false;
    bool m_autoStart = false;
};

#endif // PHYSICSWORLD_H
