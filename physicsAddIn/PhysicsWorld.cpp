#include "PhysicsWorld.h"

// A new conveyor's defaults are CadNode's own, brought up to the schema's floors by the geometry's
// own clamp - so adding one invents no numbers.
#include "ConveyorGeometry.h"

#include "iitem.h"
#include "irobodk.h"

#include "PlacedItemAxes.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QStandardPaths>
#include <QStringList>

#include <algorithm>

const char* const kConfigParam = "PhysicsAddIn";

namespace {

// A body cooked from a mesh smaller than this is more likely to be an artefact of a failed export
// than a real part, and PhysX refuses to cook a degenerate hull anyway.
constexpr size_t kMinimumHullPoints = 4;

// The floor a free placement stands on: a window onto a 100 mm lattice, moved by
// recentreFloorGrid. Declared in the accessory convention and turned into station axes by
// conveyorAccessoryFrame(), which is what lands a y-up package upright. See docs/architecture.md.
std::shared_ptr<CadNode> buildFloorNode() {
    auto floor = std::make_shared<CadNode>();
    floor->name = "Station floor";
    floor->type = CadNodeType::Transform;
    floor->data = std::make_shared<TransformNodeData>();
    floor->loc = conveyorAccessoryFrame();
    // Around the origin until a cursor says otherwise, which is what the cursorless `librarysnap` route
    // gets: it has no viewport to cast a ray through, so the window stays where this puts it.
    floor->mountingHoles.grids.push_back(mountingsnap::floorGridWindow(CadVec3()));
    return floor;
}

CadVec3 positionOfItem(Item item) {
    const CadTransform pose = rdkbridge::toCadTransform(item->PoseAbs());
    return CadVec3{pose.values[3], pose.values[7], pose.values[11]};
}

QString conveyorRoleName(conveyorcore::Role role) {
    switch (role) {
    case conveyorcore::Role::Spawner: return QStringLiteral("spawner");
    case conveyorcore::Role::Deleter: return QStringLiteral("deleter");
    case conveyorcore::Role::PickFeeder: return QStringLiteral("pick_feeder");
    case conveyorcore::Role::Normal: break;
    }
    return QStringLiteral("normal");
}

conveyorcore::Role conveyorRoleFrom(const QString& text) {
    const QString wanted = text.trimmed().toLower();
    if (wanted == QLatin1String("spawner")) return conveyorcore::Role::Spawner;
    if (wanted == QLatin1String("deleter")) return conveyorcore::Role::Deleter;
    if (wanted == QLatin1String("pick_feeder")) return conveyorcore::Role::PickFeeder;
    return conveyorcore::Role::Normal;
}

} // namespace

QString PhysicsWorld::roleName(Role role) {
    switch (role) {
    case Role::Static: return QStringLiteral("Static");
    case Role::Dynamic: return QStringLiteral("Dynamic");
    case Role::Kinematic: return QStringLiteral("Kinematic");
    case Role::None: break;
    }
    return QStringLiteral("None");
}

PhysicsWorld::PhysicsWorld(RoboDK* rdk) : m_rdk(rdk), m_conveyorHost(rdk, this) {
    m_conveyors.setHost(&m_conveyorHost);
    m_floor = buildFloorNode();
}

PhysicsWorld::~PhysicsWorld() {
    // Without this, closing RoboDK with a run in progress leaves the station holding whatever pose
    // the last step produced.
    if (m_state != State::Stopped) stop();
}

PhysicsWorld::Participant* PhysicsWorld::find(Item item) {
    const auto hit = std::find_if(m_participants.begin(), m_participants.end(),
                                  [item](const Participant& p) { return p.item == item; });
    return hit == m_participants.end() ? nullptr : &*hit;
}

const PhysicsWorld::Participant* PhysicsWorld::findConst(Item item) const {
    const auto hit = std::find_if(m_participants.begin(), m_participants.end(),
                                  [item](const Participant& p) { return p.item == item; });
    return hit == m_participants.end() ? nullptr : &*hit;
}

bool PhysicsWorld::hasBody(Item item) const {
    const Participant* participant = findConst(item);
    return participant && participant->body != 0;
}

bool PhysicsWorld::bodyPoseOf(Item item, CadTransform* world) const {
    const Participant* participant = findConst(item);
    return participant && participant->body != 0 &&
           m_physics.bodyPose(participant->body, world);
}

void PhysicsWorld::removeItem(Item item) {
    const auto hit = std::find_if(m_participants.begin(), m_participants.end(),
                                  [item](const Participant& p) { return p.item == item; });
    if (hit != m_participants.end()) {
        if (hit->body != 0) {
            m_physics.removeBody(hit->body);
            if (m_dynamicCount > 0) --m_dynamicCount;
        }
        if (hit->carried && m_carriedCount > 0) --m_carriedCount;
        m_participants.erase(hit);
    }
    if (m_rdk && m_rdk->Valid(item)) item->Delete();
}

void PhysicsWorld::setRole(Item item, Role role) {
    if (!item) return;
    if (role == Role::None) {
        forget(item);
        return;
    }
    if (Participant* existing = find(item)) {
        existing->role = role;
        existing->status = QStringLiteral("Marked %1.").arg(roleName(role));
        return;
    }
    Participant participant;
    participant.item = item;
    participant.name = item->Name();
    participant.role = role;
    participant.status = QStringLiteral("Marked %1.").arg(roleName(role));
    m_participants.push_back(participant);
}

PhysicsWorld::Role PhysicsWorld::roleOf(Item item) const {
    const auto hit = std::find_if(m_participants.begin(), m_participants.end(),
                                  [item](const Participant& p) { return p.item == item; });
    return hit == m_participants.end() ? Role::None : hit->role;
}

void PhysicsWorld::forget(Item item) {
    m_participants.erase(
        std::remove_if(m_participants.begin(), m_participants.end(),
                       [item](const Participant& p) { return p.item == item; }),
        m_participants.end());
}

void PhysicsWorld::pruneInvalidItems() {
    if (!m_rdk) return;
    if (m_stepping) {
        // EventChanged arrives synchronously from inside the API calls a step makes, so this can
        // land while the rules are iterating their own product list. Deferred to the next step.
        m_pruneRequested = true;
        return;
    }
    m_pruneRequested = false;
    m_participants.erase(
        std::remove_if(m_participants.begin(), m_participants.end(),
                       [this](Participant& p) {
                           if (m_rdk->Valid(p.item)) return false;
                           if (p.body != 0) {
                               m_physics.removeBody(p.body);
                               p.body = 0;
                               size_t& count = p.role == Role::Kinematic ? m_kinematicCount
                                                                        : m_dynamicCount;
                               if (count > 0) --count;
                           }
                           if (p.carried && m_carriedCount > 0) --m_carriedCount;
                           return true;
                       }),
        m_participants.end());
    // A product whose item RoboDK has deleted is a dangling pointer the rules would go on
    // advancing. Dropped from the runtime and from the host's item map together.
    for (const conveyorcore::ProductId id : m_conveyorHost.invalidProducts()) {
        m_conveyors.removeProduct(id);
        m_conveyorHost.forgetProduct(id);
    }
    // A conveyor whose item has gone was deleted. Its IO goes dark before it is forgotten; its
    // products were its children, so RoboDK already took them.
    for (const conveyorcore::ConveyorId id : m_conveyorHost.invalidConveyors()) forgetConveyor(id);
}

void PhysicsWorld::releaseScene() {
    m_physics.stopAsync();
    std::string ignored;
    m_physics.reset({}, &ignored);
    for (Participant& participant : m_participants) participant.body = 0;
    m_staticCount = 0;
    m_dynamicCount = 0;
    m_kinematicCount = 0;
    m_carriedCount = 0;
}

std::vector<conveyorcore::ConveyorId> PhysicsWorld::spawningConveyors() const {
    std::vector<conveyorcore::ConveyorId> feeding;
    for (size_t index = 0; index < m_conveyorHost.segments().size(); ++index) {
        const RoboDkConveyorHost::Segment& segment = m_conveyorHost.segments()[index];
        if (segment.name.isEmpty()) continue;
        if (segment.spec.role != conveyorcore::Role::Spawner) continue;
        if (!segment.prototype || !m_rdk || !m_rdk->Valid(segment.prototype)) continue;
        feeding.push_back(static_cast<conveyorcore::ConveyorId>(index + 1));
    }
    return feeding;
}

void PhysicsWorld::syncSpawners() {
    m_conveyors.setSpawners(spawningConveyors());
}

bool PhysicsWorld::start(QString* errorMessage) {
    if (m_state == State::Paused) {
        resume();
        return true;
    }
    if (m_state == State::Running) return true;

    m_lastError.clear();
    pruneInvalidItems();
    const bool spawning = hasSpawningConveyor();
    if (m_participants.empty() && !spawning) {
        m_lastError = QStringLiteral(
            "Nothing is marked for simulation and no conveyor is set to spawn. Either right-click an "
            "object in the station tree and give it a physics role, or set a conveyor's role to Object "
            "spawner and choose the workpiece it feeds.");
        if (errorMessage) *errorMessage = m_lastError;
        return false;
    }
    if (!ConveyorPhysics::backendAvailable()) {
        m_lastError = QStringLiteral("This build has no PhysX backend.");
        if (errorMessage) *errorMessage = m_lastError;
        return false;
    }

    releaseScene();
    m_physics.setGravity(m_gravityMps2);
    syncSpawners();
    m_conveyors.restartSpawners();
    m_stepCount = 0;
    m_steppedSeconds = 0.0;

    struct Pending {
        Participant* participant = nullptr;
        rdkbridge::MeshData mesh;
        CadTransform pose;
    };
    std::vector<Pending> movable;
    std::vector<ConveyorPhysics::CollisionMesh> statics;

    for (Participant& participant : m_participants) {
        participant.body = 0;
        participant.restoreValid = false;
        participant.name = participant.item->Name();

        rdkbridge::MeshData mesh;
        QString meshError;
        if (!rdkbridge::exportItemMesh(m_rdk, participant.item, &mesh, &meshError)) {
            participant.status = meshError;
            continue;
        }
        participant.triangles = mesh.triangleCount();

        const CadTransform pose = rdkbridge::toCadTransform(participant.item->PoseAbs());
        participant.restorePose = pose;
        participant.restoreValid = true;

        if (participant.role == Role::Static) {
            ConveyorPhysics::CollisionMesh collision;
            collision.pose = pose;
            collision.verticesMm.reserve(mesh.verticesMm.size() * 3);
            for (const CadVec3& vertex : mesh.verticesMm) {
                collision.verticesMm.push_back(static_cast<float>(vertex.x));
                collision.verticesMm.push_back(static_cast<float>(vertex.y));
                collision.verticesMm.push_back(static_cast<float>(vertex.z));
            }
            collision.indices = mesh.indices;
            statics.push_back(std::move(collision));
            participant.status = QStringLiteral("Static, %1 triangles.").arg(mesh.triangleCount());
            ++m_staticCount;
            continue;
        }
        // A carried item is the station's while it is carried. It keeps its export - so releasing it
        // mid-run costs no round trip - but contributes no body and no collision.
        if (participant.carried) {
            participant.hullPoints = rdkbridge::reducedHullPoints(mesh.verticesMm);
            participant.hullValid = participant.hullPoints.size() >= kMinimumHullPoints;
            participant.status = QStringLiteral("Carried by the station.");
            ++m_carriedCount;
            continue;
        }
        movable.push_back({&participant, std::move(mesh), pose});
    }

    if (statics.empty() && movable.empty() && !spawning) {
        m_lastError = QStringLiteral(
            "None of the marked items produced geometry. See each item's status for why.");
        if (errorMessage) *errorMessage = m_lastError;
        return false;
    }

    std::string physicsError;
    if (!m_physics.reset(statics, &physicsError)) {
        m_lastError = QString::fromStdString(physicsError);
        if (errorMessage) *errorMessage = m_lastError;
        return false;
    }

    for (Pending& pending : movable) {
        Participant& participant = *pending.participant;
        participant.hullPoints = rdkbridge::reducedHullPoints(pending.mesh.verticesMm);
        participant.hullValid = participant.hullPoints.size() >= kMinimumHullPoints;
        if (!participant.hullValid) {
            participant.status = QStringLiteral("Too few vertices to form a body.");
            continue;
        }
        std::string bodyError;
        if (participant.role == Role::Dynamic) {
            participant.body =
                m_physics.addConvex(pending.pose, participant.hullPoints, 1200.0, &bodyError);
            if (participant.body) ++m_dynamicCount;
        } else {
            participant.body =
                m_physics.addKinematicConvex(pending.pose, participant.hullPoints, &bodyError);
            if (participant.body) ++m_kinematicCount;
        }
        participant.status = participant.body
            ? QStringLiteral("%1, %2 triangles, hull of %3 points.")
                  .arg(roleName(participant.role)).arg(participant.triangles)
                  .arg(participant.hullPoints.size())
            : QString::fromStdString(bodyError);
    }

    m_state = State::Running;
    return true;
}

QString PhysicsWorld::nextSpawnName(const QString& prototypeName) {
    return QStringLiteral("%1 #%2").arg(prototypeName).arg(++m_spawnSerial);
}

bool PhysicsWorld::createDynamicBody(Participant& participant, QString* errorMessage) {
    if (!participant.hullValid) {
        rdkbridge::MeshData mesh;
        QString meshError;
        if (!rdkbridge::exportItemMesh(m_rdk, participant.item, &mesh, &meshError)) {
            if (errorMessage) *errorMessage = meshError;
            return false;
        }
        participant.triangles = mesh.triangleCount();
        participant.hullPoints = rdkbridge::reducedHullPoints(mesh.verticesMm);
        participant.hullValid = participant.hullPoints.size() >= kMinimumHullPoints;
    }
    if (!participant.hullValid) {
        if (errorMessage) *errorMessage = QStringLiteral("Too few vertices to form a body.");
        return false;
    }
    const CadTransform pose = rdkbridge::toCadTransform(participant.item->PoseAbs());
    std::string bodyError;
    participant.body = m_physics.addConvex(pose, participant.hullPoints, 1200.0, &bodyError);
    if (!participant.body) {
        if (errorMessage) *errorMessage = QString::fromStdString(bodyError);
        return false;
    }
    return true;
}

Item PhysicsWorld::spawn(Item prototype, Item parent, const CadTransform& pose, QString* outName,
                         QString* errorMessage) {
    if (!m_rdk || !prototype || !m_rdk->Valid(prototype)) {
        if (errorMessage) *errorMessage = QStringLiteral("No prototype to spawn from.");
        return nullptr;
    }
    Item destination = parent && m_rdk->Valid(parent) ? parent : prototype->Parent();
    if (!destination) {
        if (errorMessage) *errorMessage = QStringLiteral("The prototype has no parent to spawn into.");
        return nullptr;
    }
    prototype->Copy();
    Item clone = destination->Paste();
    if (!clone) {
        if (errorMessage) *errorMessage = QStringLiteral("RoboDK would not copy the prototype.");
        return nullptr;
    }
    const QString name = nextSpawnName(prototype->Name());
    clone->setName(name);
    clone->setPoseAbs(rdkbridge::toRoboDkPose(pose));
    // Visible, whatever the prototype was: a hidden template is the usual way to keep a spawn
    // source out of the cell, and a clone that inherited that would simulate invisibly.
    clone->setVisible(true);

    Participant participant;
    participant.item = clone;
    participant.name = name;
    participant.role = Role::Dynamic;
    participant.carried = true;
    participant.spawned = true;
    participant.status = QStringLiteral("Spawned, carried by the station.");
    m_participants.push_back(participant);
    if (m_state != State::Stopped) ++m_carriedCount;

    if (outName) *outName = name;
    if (errorMessage) errorMessage->clear();
    return clone;
}

bool PhysicsWorld::attach(Item item, QString* errorMessage) {
    Participant* participant = find(item);
    if (!participant) {
        if (errorMessage) *errorMessage = QStringLiteral("That item is not in the physics scene.");
        return false;
    }
    if (participant->body != 0) {
        m_physics.removeBody(participant->body);
        participant->body = 0;
        if (m_dynamicCount > 0) --m_dynamicCount;
    }
    if (!participant->carried) {
        participant->carried = true;
        if (m_state != State::Stopped) ++m_carriedCount;
    }
    participant->status = QStringLiteral("Carried by the station.");
    if (errorMessage) errorMessage->clear();
    return true;
}

bool PhysicsWorld::release(Item item, QString* errorMessage) {
    if (!item) {
        if (errorMessage) *errorMessage = QStringLiteral("No item to release.");
        return false;
    }
    Participant* participant = find(item);
    if (!participant) {
        Participant fresh;
        fresh.item = item;
        fresh.name = item->Name();
        fresh.role = Role::Dynamic;
        fresh.carried = true;
        fresh.restorePose = rdkbridge::toCadTransform(item->PoseAbs());
        fresh.restoreValid = true;
        fresh.status = QStringLiteral("Handed over by the station.");
        m_participants.push_back(fresh);
        participant = &m_participants.back();
        if (m_state != State::Stopped) ++m_carriedCount;
    }
    if (m_state != State::Running) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("The simulation is not running, so there is nothing to "
                                           "release into.");
        }
        return false;
    }
    // Out of the gripper's hierarchy before it becomes a body, keeping the pose it is at. Left
    // parented to the tool, RoboDK would keep carrying it while PhysX also placed it.
    if (Item station = m_rdk ? m_rdk->getActiveStation() : nullptr) {
        participant->item->setParentStatic(station);
    }
    QString bodyError;
    if (!createDynamicBody(*participant, &bodyError)) {
        participant->status = bodyError;
        if (errorMessage) *errorMessage = bodyError;
        return false;
    }
    if (participant->carried) {
        participant->carried = false;
        if (m_carriedCount > 0) --m_carriedCount;
    }
    participant->role = Role::Dynamic;
    ++m_dynamicCount;
    participant->status = QStringLiteral("Released - simulated, hull of %1 points.")
                              .arg(participant->hullPoints.size());
    if (errorMessage) errorMessage->clear();
    return true;
}

bool PhysicsWorld::configureConveyor(const RoboDkConveyorHost::Segment& incoming, QString* error) {
    if (error) error->clear();
    RoboDkConveyorHost::Segment segment = incoming;
    if (segment.name.isEmpty()) {
        if (error) *error = QStringLiteral("A conveyor needs a name.");
        return false;
    }
    if (segment.item && segment.item->Type() != IItem::ITEM_TYPE_GENERIC) {
        const bool parentsItsOwn = segment.parent == segment.item;
        // Preserve the declared item's placement during conversion.
        Item node = standUpConveyor(segment.item, &segment.poseLocal, error);
        if (!node) return false;
        segment.item = node;
        segment.hasPose = true;
        if (parentsItsOwn) segment.parent = node;
    }
    const conveyorcore::ConveyorId byName = m_conveyorHost.idNamed(segment.name);
    const conveyorcore::ConveyorId existing =
        segment.item ? m_conveyorHost.idOfItem(segment.item) : byName;
    if (byName != 0 && byName != existing) {
        if (error) {
            *error = QStringLiteral("two conveyors are called '%1'; the second was refused, because "
                                    "both would write the same three IO variables")
                         .arg(segment.name);
        }
        return false;
    }
    if (existing != 0) {
        RoboDkConveyorHost::Segment* found = m_conveyorHost.mutableSegment(existing);
        const QString was = found->name;
        if (!segment.hasPose && found->hasPose) {
            segment.poseLocal = found->poseLocal;
            segment.hasPose = true;
        }
        const TransformNodeData& had = found->parameters;
        if (!segment.stated.endInset) {
            segment.parameters.accessoryInitialWorkpieceEndInsetMm =
                had.accessoryInitialWorkpieceEndInsetMm;
        }
        if (!segment.stated.pitch) {
            segment.parameters.accessoryInitialWorkpieceSpacingMm =
                had.accessoryInitialWorkpieceSpacingMm;
        }
        if (!segment.stated.deckWidth) segment.parameters.accessoryWidthMm = had.accessoryWidthMm;
        if (!segment.stated.deckHeight) {
            segment.parameters.accessoryHeightMm = had.accessoryHeightMm;
            segment.parameters.accessoryStartHeightMm = had.accessoryStartHeightMm;
            segment.parameters.accessoryEndHeightMm = had.accessoryEndHeightMm;
            segment.parameters.accessoryStartLeftHeightMm = had.accessoryStartLeftHeightMm;
            segment.parameters.accessoryStartRightHeightMm = had.accessoryStartRightHeightMm;
            segment.parameters.accessoryEndLeftHeightMm = had.accessoryEndLeftHeightMm;
            segment.parameters.accessoryEndRightHeightMm = had.accessoryEndRightHeightMm;
        }
        *found = segment;
        // Clear IO published under a conveyor's previous name.
        if (was != found->name) darkenConveyorIo(was);
        m_conveyorHost.measureSegment(*found);
        writeConveyorParameters(m_rdk, *found);
        syncSpawners();
        return true;
    }
    const conveyorcore::ConveyorId id = m_conveyorHost.add(segment);
    if (RoboDkConveyorHost::Segment* added = m_conveyorHost.mutableSegment(id)) {
        m_conveyorHost.measureSegment(*added);
        writeConveyorParameters(m_rdk, *added);
    }
    syncSpawners();
    return true;
}

void PhysicsWorld::adoptConveyorItems(QStringList* warnings) {
    if (!m_rdk) return;
    // Accept generic items and older frame/object conveyor representations.
    for (int kind : {IItem::ITEM_TYPE_GENERIC, IItem::ITEM_TYPE_FRAME, IItem::ITEM_TYPE_OBJECT}) {
        for (Item item : m_rdk->getItemList(kind)) {
            if (!item || !m_rdk->Valid(item)) continue;
            QByteArray blob;
            if (!item->getParam(kConveyorItemParam, blob) || blob.trimmed().isEmpty()) continue;
            if (const conveyorcore::ConveyorId known = m_conveyorHost.idOfItem(item)) {
                const RoboDkConveyorHost::Segment* segment = m_conveyorHost.mutableSegment(known);
                // Unchanged bytes and name mean an unchanged conveyor; re-declaring exports the prototype
                // mesh to measure it. The name is checked separately because it is not in the bytes. This
                // is also what stops measureSegment's shape push feeding itself through EventChanged.
                if (segment && segment->parametersBlob == blob &&
                    segment->name == item->Name()) continue;
            }
            RoboDkConveyorHost::Segment segment;
            QString error;
            if (!conveyorSegmentFromItem(m_rdk, item, &segment, &error) ||
                !configureConveyor(segment, &error)) {
                if (warnings) *warnings << error;
            }
        }
    }
}

Item PhysicsWorld::standUpConveyor(Item was, CadTransform* outPoseLocal, QString* error) {
    if (!m_rdk || !was) return nullptr;
    // Reuse the name only after deleting the previous item to avoid RoboDK uniquifying it.
    const QString wanted = was->Name();
    // Stored placement is local to the item's current parent.
    if (outPoseLocal) *outPoseLocal = rdkbridge::toCadTransform(was->Pose());
    Item node = createConveyorItem(m_rdk, wanted + QStringLiteral(" (new)"), error);
    if (!node) return nullptr;

    QByteArray blob;
    if (was->getParam(kConveyorItemParam, blob) && !blob.trimmed().isEmpty()) {
        node->setParam(kConveyorItemParam, blob);
    }
    if (Item parent = was->Parent()) node->setParent(parent);
    for (Item child : was->Childs()) {
        if (child && m_rdk->Valid(child) && child != node) child->setParentStatic(node);
    }
    was->Delete();
    node->setName(wanted);
    if (error) error->clear();
    return node;
}

std::vector<const PlacedItem*> PhysicsWorld::placedItems() const {
    std::vector<const PlacedItem*> painted;
    painted.reserve(m_conveyorHost.segments().size() + m_libraryItems.size());
    for (const RoboDkConveyorHost::Segment& segment : m_conveyorHost.segments()) {
        painted.push_back(&segment);
    }
    for (const PlacedLibraryItem& placed : m_libraryItems) painted.push_back(&placed);
    return painted;
}

const PlacedItem* PhysicsWorld::placedItemNamed(const QString& name) const {
    if (name.isEmpty()) return nullptr;
    for (const PlacedItem* placed : placedItems()) {
        if (placed->name == name) return placed;
    }
    return nullptr;
}

const PlacedItem* PhysicsWorld::placedItemFor(Item item) const {
    if (!item) return nullptr;
    for (const PlacedItem* placed : placedItems()) {
        if (placed->item == item) return placed;
    }
    return nullptr;
}

void PhysicsWorld::drawPlacedItems() {
    if (!m_rdk) return;
    // Only from an EventRender, which is DrawGeometry's contract. Nothing here touches the
    // station. The triangles are already in station coordinates, so there is no pose to apply.
    for (const PlacedItem* placed : placedItems()) {
        for (const ConveyorDrawGroup& group : placed->scenery.groups) {
            if (group.triangles <= 0) continue;
            float rgba[4] = {group.rgba[0], group.rgba[1], group.rgba[2], group.rgba[3]};
            m_rdk->DrawGeometry(IRoboDK::DrawTriangles,
                                const_cast<float*>(group.verticesMm.data()), group.triangles, rgba,
                                2.0f, const_cast<float*>(group.normals.data()));
        }
    }
}

void PhysicsWorld::bakePlacedItem(PlacedItem& placed, const CadTransform& stand) {
    if (!placed.scenery.root) return;
    if (placed.axes) placed.axes->apply();
    static_cast<cadnodedraw::Scenery&>(placed.scenery) =
        cadnodedraw::flatten(placed.scenery.root.get(), stand);
    placed.sceneryAt = stand;
    bool any = false;
    float lo[3] = {0.0f, 0.0f, 0.0f};
    float hi[3] = {0.0f, 0.0f, 0.0f};
    for (const ConveyorDrawGroup& group : placed.scenery.groups) {
        for (size_t at = 0; at + 2 < group.verticesMm.size(); at += 3) {
            for (int axis = 0; axis < 3; ++axis) {
                const float value = group.verticesMm[at + static_cast<size_t>(axis)];
                if (!any || value < lo[axis]) lo[axis] = value;
                if (!any || value > hi[axis]) hi[axis] = value;
            }
            any = true;
        }
    }
    placed.sceneryMinMm = any ? CadVec3(lo[0], lo[1], lo[2]) : CadVec3();
    placed.sceneryMaxMm = any ? CadVec3(hi[0], hi[1], hi[2]) : CadVec3();
    // A package's tree stands in the frame it was placed in; only a generated accessory has a frame of
    // its own between the two.
    placed.treeToWorld = stand;
}

QString PhysicsWorld::libraryReport() const {
    QStringList records;
    for (const PlacedLibraryItem& placed : m_libraryItems) {
        const bool alive = placed.item && m_rdk && m_rdk->Valid(placed.item);
        QString record = QStringLiteral("%1 | %2 | %3 | %4 tri | at %5,%6,%7 | %8")
                             .arg(placed.name,
                                  placed.spec.packageRef,
                                  placed.spec.variantId.isEmpty() ? QStringLiteral("-")
                                                                  : placed.spec.variantId)
                             .arg(placed.scenery.triangles)
                             .arg(placed.sceneryAt.values[3], 0, 'f', 1)
                             .arg(placed.sceneryAt.values[7], 0, 'f', 1)
                             .arg(placed.sceneryAt.values[11], 0, 'f', 1)
                             .arg(alive ? QStringLiteral("generic node")
                                        : QStringLiteral("item gone"));
        record += QStringLiteral(" | box %1,%2,%3 to %4,%5,%6")
                      .arg(placed.sceneryMinMm.x, 0, 'f', 3)
                      .arg(placed.sceneryMinMm.y, 0, 'f', 3)
                      .arg(placed.sceneryMinMm.z, 0, 'f', 3)
                      .arg(placed.sceneryMaxMm.x, 0, 'f', 3)
                      .arg(placed.sceneryMaxMm.y, 0, 'f', 3)
                      .arg(placed.sceneryMaxMm.z, 0, 'f', 3);
        if (placed.axes && placed.axes->axisCount() > 0) {
            QStringList axes;
            for (double value : placed.axes->values()) axes << QString::number(value, 'f', 6);
            record += QStringLiteral(" | axes %1 %2")
                          .arg(axes.join(QLatin1Char(',')),
                               placed.axes->kind() == PlacedItemAxes::Kind::Rail
                                   ? QStringLiteral("mm")
                                   : QStringLiteral("deg"));
        }
        records << record;
    }
    if (records.isEmpty()) return QStringLiteral("no placed library items");
    return records.join(QStringLiteral(" || "));
}

QString PhysicsWorld::catalogueReport() const {
    QStringList records;
    for (const librarycatalogue::Entry& entry : libraryCatalogue()) {
        const QString refusal = dockRefusal(entry);
        records << QStringLiteral("%1 | %2 | %3 | %4")
                       .arg(QString::fromLatin1(librarycatalogue::categoryLabel(entry.category)),
                            QString::fromStdString(entry.name),
                            QString::fromStdString(entry.variantId),
                            refusal.isEmpty() ? QStringLiteral("armable") : refusal);
    }
    if (records.isEmpty()) {
        return QStringLiteral("no library at '%1'").arg(libraryRoot());
    }
    return records.join(QStringLiteral(" || "));
}

QString PhysicsWorld::libraryRoot() const {
    if (!m_libraryRoot.isEmpty()) return m_libraryRoot;
    if (m_rdk) {
        const QString declared = m_rdk->getParam(QStringLiteral("PhysicsLibraryRoot"));
        if (!declared.isEmpty()) return declared;
    }
    return builtinLibraryRoot();
}

QString PhysicsWorld::builtinLibraryRoot() {
    static const QString root = [] {
        const QString cache =
            QDir(QStandardPaths::writableLocation(QStandardPaths::CacheLocation))
                .filePath(QStringLiteral("PhysicsAddIn/packages"));
        QDir directory(cache);
        if (!directory.mkpath(QStringLiteral("."))) return QString();
        const QDir embedded(QStringLiteral(":/packages"));
        const QStringList names = embedded.entryList(QStringList{QStringLiteral("*.zip")}, QDir::Files);
        if (names.isEmpty()) return QString(); // built without the resource; the station parameter is it
        for (const QString& name : names) {
            QFile source(embedded.filePath(name));
            const QString destination = directory.filePath(name);
            if (QFileInfo(destination).size() == source.size()) continue;
            if (!source.open(QIODevice::ReadOnly)) continue;
            QFile out(destination);
            if (out.open(QIODevice::WriteOnly | QIODevice::Truncate)) out.write(source.readAll());
        }
        return cache;
    }();
    return root;
}

bool PhysicsWorld::placeLibraryItem(const LibraryItemSpec& spec, QString* outName, QString* error,
                                    Item* outItem) {
    if (outItem) *outItem = nullptr;
    if (!m_rdk) {
        if (error) *error = QStringLiteral("No RoboDK.");
        return false;
    }
    std::shared_ptr<CadNode> root = loadLibraryItemTree(spec, libraryRoot(), error);
    if (!root) return false;
    if (const TransformNodeData* transform = root->asTransform()) {
        if (transform->hasParametricAccessory()) {
            if (error) {
                *error = QStringLiteral("'%1' is a generated accessory; add it as a conveyor instead.")
                             .arg(spec.packageRef);
            }
            return false;
        }
    }
    // Kinematics bound and the asked-for axes checked against the package's own limits before
    // anything is created, so a placement naming a pose this arm has not got is a refusal rather
    // than an item standing at some other pose.
    auto axes = std::make_shared<PlacedItemAxes>();
    axes->bind(root.get());
    if (!axes->drives()) {
        axes.reset();
    } else if (!spec.axes.empty()) {
        QString refusal;
        if (!axes->setValues(spec.axes, &refusal)) {
            if (error) {
                *error = QStringLiteral("'%1': %2").arg(spec.packageRef, refusal);
            }
            return false;
        }
    }

    // Made free before it is asked for: RoboDK hands the same name to both copies without
    // complaint, and two items sharing one are one item to every verb that takes a name.
    const QString wanted = freeItemName(root->name.empty()
        ? QFileInfo(spec.packageRef).completeBaseName()
        : QString::fromStdString(root->name));
    // A rail stands on a real RoboDK mechanism; everything else on a generic node. The distinction is the
    // package's, not this function's: a rail is the one kind whose one number RoboDK already has a
    // mechanism for, so it is the one kind that gains anything by being one.
    Item item = nullptr;
    if (axes && axes->kind() == PlacedItemAxes::Kind::Rail) {
        const std::vector<placedmechanism::AxisField> fields = axes->fields();
        const placedmechanism::AxisField& travel = fields.front();
        item = createRailMechanismItem(m_rdk, wanted, travel.minimum, travel.maximum,
                                       axes->values().front(), error);
    } else {
        item = createLibraryItem(m_rdk, wanted, error);
    }
    if (!item) return false;

    PlacedLibraryItem placed;
    placed.item = item;
    placed.name = item->Name();
    placed.spec = spec;
    placed.poseLocal = spec.poseLocal;
    placed.hasPose = spec.hasPose;
    placed.scenery.root = std::move(root);
    placed.axes = std::move(axes);
    writeLibraryItemSpec(m_rdk, item, placed.spec, &placed.blob);

    // The mechanism posed and its joint set before anything reads it back. syncRailMechanisms
    // treats the base frame as the authority on placement, so a rail created without its frame
    // posed would have its placement overwritten on the next timer tick.
    pushRailMechanismPose(placed);
    pushRailMechanismJoint(placed);
    CadTransform stand;
    if (placedItemStand(m_rdk, placed, &stand)) bakePlacedItem(placed, stand);
    if (outName) *outName = placed.name;
    if (outItem) *outItem = placed.item;
    m_libraryItems.push_back(std::move(placed));
    if (error) error->clear();
    return true;
}

bool PhysicsWorld::deleteLibraryItem(const QString& name, QString* error) {
    for (size_t index = 0; index < m_libraryItems.size(); ++index) {
        if (m_libraryItems[index].name != name) continue;
        Item item = m_libraryItems[index].item;
        // A rail's mechanism stands on a frame `BuildMechanism` made, and deleting the mechanism alone
        // would leave that frame behind as litter nothing owns. Found before the item goes, because it is
        // reached *through* the item.
        Item frame = railMechanismBaseFrame(m_rdk, item);
        // Forgotten before the item goes, so nothing draws or picks a pointer RoboDK has freed.
        m_libraryItems.erase(m_libraryItems.begin() + static_cast<ptrdiff_t>(index));
        if (item && m_rdk && m_rdk->Valid(item)) item->Delete();
        // After the mechanism, because the mechanism hangs under it and RoboDK would take the mechanism
        // with the frame - which is the same end state by a route that deletes a live item twice.
        if (frame && m_rdk && m_rdk->Valid(frame)) frame->Delete();
        if (error) error->clear();
        return true;
    }
    if (error) *error = QStringLiteral("There is no placed item called '%1'.").arg(name);
    return false;
}

void PhysicsWorld::standLibraryItem(PlacedLibraryItem& placed, const CadTransform& poseLocal) {
    placed.poseLocal = poseLocal;
    placed.hasPose = true;
    placed.spec.poseLocal = poseLocal;
    placed.spec.hasPose = true;
    writeLibraryItemSpec(m_rdk, placed.item, placed.spec, &placed.blob);
    pushRailMechanismPose(placed);
    CadTransform stand;
    if (placedItemStand(m_rdk, placed, &stand)) bakePlacedItem(placed, stand);
}

bool PhysicsWorld::setLibraryItemPlacement(const QString& name, const CadTransform& poseLocal,
                                           QString* error) {
    for (PlacedLibraryItem& placed : m_libraryItems) {
        if (placed.name != name) continue;
        standLibraryItem(placed, poseLocal);
        if (error) error->clear();
        return true;
    }
    if (error) *error = QStringLiteral("There is no placed item called '%1' to move.").arg(name);
    return false;
}

bool PhysicsWorld::setLibraryItemPlacementFor(Item item, const CadTransform& poseLocal,
                                              QString* error) {
    for (PlacedLibraryItem& placed : m_libraryItems) {
        if (placed.item != item) continue;
        standLibraryItem(placed, poseLocal);
        if (error) error->clear();
        return true;
    }
    if (error) *error = QStringLiteral("That item is not one of this plugin's placed packages.");
    return false;
}

namespace {

QString describeTransform(const CadTransform& transform) {
    QStringList cells;
    for (int cell = 0; cell < 12; ++cell) {
        cells << QString::number(transform.values[cell], 'f', 9);
    }
    return cells.join(QLatin1Char(','));
}

} // namespace

bool PhysicsWorld::drivePlacedItem(PlacedLibraryItem& placed, const std::vector<double>& values,
                                   QString* report, QString* error) {
    const QString name = placed.name;
    if (!placed.axes) {
        if (error) {
            *error = QStringLiteral("'%1' is not a mechanism; it has no axes to drive.").arg(name);
        }
        return false;
    }
    if (!placed.axes->setValues(values, error)) return false;
    // Onto RoboDK's own mechanism as well, for a rail. Without this the plugin and RoboDK would hold two
    // different positions for one carriage until the next sync tick noticed and - because RoboDK is the
    // authority there - undid the edit that had just been accepted.
    pushRailMechanismJoint(placed);
    placed.spec.axes = placed.axes->values();
    writeLibraryItemSpec(m_rdk, placed.item, placed.spec, &placed.blob);
    // Re-baked because the joints moved the tree the triangles are flattened out of. Without this the
    // arm would be posed in the model and unmoved on screen, which is the same silent disagreement
    // between what is computed and what is drawn that `RenderUpdateOnly` once hid.
    CadTransform stand;
    if (placedItemStand(m_rdk, placed, &stand)) bakePlacedItem(placed, stand);
    if (report) {
        QStringList axes;
        for (double value : placed.spec.axes) axes << QString::number(value, 'f', 6);
        const bool rail = placed.axes->kind() == PlacedItemAxes::Kind::Rail;
        *report = QStringLiteral("OK %1 | %2 %3 | %4")
                      .arg(name,
                           axes.join(QLatin1Char(',')),
                           rail ? QStringLiteral("mm") : QStringLiteral("deg"),
                           // A rail's one driven frame in station millimetres; an arm's two, stated
                           // apart because the arm's own base is a frame the station cannot reach and
                           // pre-composing them would hide which of the three was got wrong.
                           rail ? QStringLiteral("carriage %1").arg(describeTransform(
                                      placed.treeToWorld * placed.axes->railCarriageInTree()))
                                : QStringLiteral("tcp %1 | base %2")
                                      .arg(describeTransform(placed.axes->flangeInArmBase()),
                                           describeTransform(placed.treeToWorld *
                                                             placed.axes->armBaseInTree())));
    }
    if (error) error->clear();
    return true;
}

bool PhysicsWorld::setPlacedItemAxes(const QString& name, const std::vector<double>& values,
                                     QString* report, QString* error) {
    for (PlacedLibraryItem& placed : m_libraryItems) {
        if (placed.name != name) continue;
        return drivePlacedItem(placed, values, report, error);
    }
    if (error) {
        *error = QStringLiteral("There is no placed item called '%1' to drive.").arg(name);
    }
    return false;
}

bool PhysicsWorld::setPlacedItemAxesFor(Item item, const std::vector<double>& values, QString* report,
                                        QString* error) {
    for (PlacedLibraryItem& placed : m_libraryItems) {
        if (placed.item != item) continue;
        return drivePlacedItem(placed, values, report, error);
    }
    if (error) *error = QStringLiteral("That item is not one of this plugin's placed packages.");
    return false;
}

bool PhysicsWorld::renameLibraryItem(Item item, const QString& wanted, QString* error) {
    for (PlacedLibraryItem& placed : m_libraryItems) {
        if (placed.item != item) continue;
        if (!item || (m_rdk && !m_rdk->Valid(item))) {
            if (error) *error = QStringLiteral("Its item is gone.");
            return false;
        }
        const QString trimmed = wanted.trimmed();
        if (trimmed.isEmpty()) {
            if (error) *error = QStringLiteral("A placed item needs a name.");
            return false;
        }
        if (trimmed == placed.name) {
            if (error) error->clear();
            return true;
        }
        // RoboDK grants a taken name in silence, so the operator's name is honoured if free and
        // suffixed if not.
        const QString given = freeItemName(trimmed);
        item->setName(given);
        placed.name = item->Name();
        if (error) error->clear();
        return true;
    }
    if (error) *error = QStringLiteral("That item is not one of this plugin's placed packages.");
    return false;
}

std::vector<librarycatalogue::Entry> PhysicsWorld::libraryCatalogue() const {
    const QString root = libraryRoot();
    if (root.isEmpty()) return {};
    return librarycatalogue::scan(root.toStdString());
}

void PhysicsWorld::recentreFloorGrid(const mountingsnap::View& view, double cursorX, double cursorY) {
    if (!m_floor) return;
    mountingsnap::recentreFloorGrid(m_floor.get(), m_floor->loc, view, cursorX, cursorY);
}

void PhysicsWorld::collectFloorPoints(std::vector<CadVec3>* pointsMm) const {
    if (!pointsMm) return;
    pointsMm->clear();
    std::vector<mountingsnap::Interface> interfaces;
    collectNodeTargets(m_floor.get(), CadTransform(), &interfaces);
    for (const mountingsnap::Interface& interface : interfaces) {
        pointsMm->insert(pointsMm->end(), interface.pointsMm.begin(), interface.pointsMm.end());
    }
}

void PhysicsWorld::collectSnapTargets(std::vector<mountingsnap::Interface>* targets) const {
    if (!targets) return;
    for (const PlacedItem* placed : placedItems()) collectPlacedItemTargets(*placed, targets);
    collectNodeTargets(m_floor.get(), CadTransform(), targets);
}

bool PhysicsWorld::commitPlacement(const ArmedPackage& armed, const CadTransform* treeToStation,
                                   QString* outName, QString* error) {
    if (!armed.armed()) {
        if (error) {
            *error = armed.refusal.isEmpty() ? QStringLiteral("Nothing is armed to place.")
                                             : armed.refusal;
        }
        return false;
    }
    QString name;
    Item made = nullptr;
    if (armed.asConveyor) {
        if (!addConveyor(&name, error)) return false;
        TransformNodeData parameters = armed.conveyorParameters;
        parameters.accessoryConveyorMode = "logical";
        QString refusal;
        if (!applyConveyorParameters(name, parameters, &refusal)) {
            deleteConveyor(name);
            if (error) *error = refusal;
            return false;
        }
    } else {
        LibraryItemSpec spec;
        spec.packageRef = QString::fromStdString(armed.entry.path);
        spec.variantId = QString::fromStdString(armed.entry.variantId);
        spec.parameters = armed.entry.parameters;
        // No pose yet. It is set below, once there is an item whose parent can be read.
        if (!placeLibraryItem(spec, &name, error, &made)) return false;
    }

    // The item it just made, not the first thing wearing the same name: a lookup by name here
    // aimed the pose at whichever copy was placed first.
    const PlacedItem* placed = made ? placedItemFor(made) : placedItemNamed(name);
    if (!placed) {
        if (error) *error = QStringLiteral("'%1' was created but is not a placed item.").arg(name);
        return false;
    }
    CadTransform poseLocal;
    if (treeToStation) {
        poseLocal = placedItemParentWorld(m_rdk, *placed).rigidInverse() * *treeToStation *
                    armed.treeFrame().rigidInverse();
    }
    const bool moved = armed.asConveyor ? setConveyorPlacement(name, poseLocal, error)
                                        : setLibraryItemPlacementFor(made, poseLocal, error);
    if (!moved) return false;
    if (outName) *outName = name;
    if (error) error->clear();
    return true;
}

bool PhysicsWorld::snapLibraryItem(const QString& package, const QString& variant,
                                   const QString& mateTo, const QString& sourceId,
                                   const QString& targetId, QString* report) {
    const auto fail = [&](const QString& why) {
        if (report) *report = why;
        return false;
    };
    const std::vector<librarycatalogue::Entry> entries = libraryCatalogue();
    if (entries.empty()) {
        return fail(QStringLiteral("There is no library at '%1'.").arg(libraryRoot()));
    }
    const librarycatalogue::Entry* entry = findLibraryEntry(entries, package, variant);
    if (!entry) {
        return fail(QStringLiteral("The library at '%1' has no '%2' preset '%3'.")
                        .arg(libraryRoot(), package,
                             variant.isEmpty() ? QStringLiteral("(default)") : variant));
    }
    const ArmedPackage armed = armLibraryEntry(*entry);
    if (!armed.armed()) return fail(armed.refusal);

    QString name;
    QString error;
    if (mateTo.isEmpty()) {
        // Unmated, at its parent's origin. Nothing was solved, so there is no pose to convert - which is
        // also how the first of a pair gets into a station that had nothing to mate to.
        if (!commitPlacement(armed, nullptr, &name, &error)) return fail(error);
        if (report) {
            *report = QStringLiteral("OK %1 | unmated | at its parent's origin").arg(name);
        }
        return true;
    }

    const PlacedItem* target = placedItemNamed(mateTo);
    if (!target) {
        return fail(QStringLiteral("There is nothing called '%1' to mate to. Only what this plugin "
                                   "paints - its conveyors and its placed packages - carries mounting "
                                   "holes; RoboDK's own objects declare none.").arg(mateTo));
    }
    std::vector<mountingsnap::Interface> targets;
    collectPlacedItemTargets(*target, &targets);
    if (targets.empty()) {
        return fail(QStringLiteral("'%1' declares no mounting interfaces to mate to.").arg(mateTo));
    }
    const CursorlessMate mate = mateWithoutACursor(armed.sourceInterfaces, targets, sourceId, targetId);
    if (!mate.found) {
        return fail(QStringLiteral("No interface of '%1' mates '%2' completely. A partial match leaves "
                                   "corners floating, so it is refused rather than placed.")
                        .arg(QString::fromStdString(entry->name), mateTo));
    }
    if (!commitPlacement(armed, &mate.worldPose, &name, &error)) return fail(error);
    if (report) {
        *report = QStringLiteral("OK %1 | %2 -> %3 | %4/%5 holes | turn %6 | worst %7 mm | apart %8 mm")
                      .arg(name,
                           mate.sourceId.isEmpty() ? QStringLiteral("(placement)") : mate.sourceId,
                           mate.targetId.isEmpty() ? QStringLiteral("(placement)") : mate.targetId)
                      .arg(mate.matchedHoles)
                      .arg(mate.requiredHoles)
                      .arg(mate.quarterTurn * 90)
                      .arg(mate.worstErrorMm, 0, 'f', 6)
                      .arg(mate.apartMm, 0, 'f', 3);
    }
    return true;
}

void PhysicsWorld::adoptLibraryItems(QStringList* warnings) {
    if (!m_rdk) return;
    // The ones whose items RoboDK no longer has - deleted from the tree, or with the Delete key.
    m_libraryItems.erase(
        std::remove_if(m_libraryItems.begin(), m_libraryItems.end(),
                       [this](const PlacedLibraryItem& placed) {
                           return !placed.item || !m_rdk->Valid(placed.item);
                       }),
        m_libraryItems.end());

    // Items, never names. Two lists, because a placed package is a generic node unless it is a
    // rail, and RoboDK files a rail's mechanism under robots.
    QList<Item> candidates = m_rdk->getItemList(IItem::ITEM_TYPE_GENERIC);
    candidates.append(m_rdk->getItemList(IItem::ITEM_TYPE_ROBOT));
    for (Item item : candidates) {
        if (!item || !m_rdk->Valid(item)) continue;
        QByteArray blob;
        if (!item->getParam(kLibraryItemParam, blob) || blob.trimmed().isEmpty()) continue;
        PlacedLibraryItem* known = nullptr;
        for (PlacedLibraryItem& placed : m_libraryItems) {
            if (placed.item == item) { known = &placed; break; }
        }
        // Unchanged bytes and an unchanged name mean an unchanged item. Re-standing one reloads its
        // package, which is an adoption-time cost and not an event-rate one.
        if (known && known->blob == blob && known->name == item->Name()) continue;

        LibraryItemSpec spec;
        QString error;
        if (!libraryItemSpecFromItem(m_rdk, item, &spec, &error)) {
            if (warnings) *warnings << error;
            continue;
        }
        std::shared_ptr<CadNode> root = loadLibraryItemTree(spec, libraryRoot(), &error);
        if (!root) {
            if (warnings) *warnings << error;
            continue;
        }
        PlacedLibraryItem placed;
        placed.item = item;
        placed.name = item->Name();
        placed.spec = spec;
        placed.poseLocal = spec.poseLocal;
        placed.hasPose = spec.hasPose;
        placed.blob = blob;
        placed.scenery.root = std::move(root);
        QString axesWarning;
        bindPlacedItemAxes(placed, &axesWarning);
        if (!axesWarning.isEmpty() && warnings) *warnings << axesWarning;
        // A reopened rail takes its joint from RoboDK's mechanism, not this plugin's JSON: RoboDK
        // saved it, and its jog panel may have moved the carriage since.
        if (placed.axes && placed.axes->kind() == PlacedItemAxes::Kind::Rail) {
            const tJoints held = item->Joints();
            if (held.Length() >= 1) {
                QString ignored;
                if (placed.axes->setValues({held.Values()[0]}, &ignored)) {
                    placed.spec.axes = placed.axes->values();
                }
            }
        }
        CadTransform stand;
        if (placedItemStand(m_rdk, placed, &stand)) bakePlacedItem(placed, stand);
        if (known) {
            *known = std::move(placed);
        } else {
            m_libraryItems.push_back(std::move(placed));
        }
    }
}

namespace {

bool sameStanding(const CadTransform& a, const CadTransform& b) {
    for (int cell = 0; cell < 12; ++cell) {
        if (std::abs(a.values[cell] - b.values[cell]) > 1.0e-6) return false;
    }
    return true;
}

} // namespace

void PhysicsWorld::pushRailMechanismPose(PlacedLibraryItem& placed) {
    if (!placed.axes || placed.axes->kind() != PlacedItemAxes::Kind::Rail) return;
    Item frame = railMechanismBaseFrame(m_rdk, placed.item);
    if (!frame) return;
    // The frame's pose is relative to whatever it hangs under, exactly as the stored placement is - so the
    // two are the same statement and no parent transform enters here.
    const CadTransform base = placed.poseLocal * placed.axes->railBaseInTree();
    frame->setPose(rdkbridge::toRoboDkPose(base));
}

bool PhysicsWorld::syncRailMechanisms() {
    if (!m_rdk) return false;
    bool moved = false;
    for (PlacedLibraryItem& placed : m_libraryItems) {
        if (!placed.axes || placed.axes->kind() != PlacedItemAxes::Kind::Rail) continue;
        if (!placed.item || !m_rdk->Valid(placed.item)) continue;

        const tJoints held = placed.item->Joints();
        if (held.Length() >= 1) {
            const double roboDk = held.Values()[0];
            const double ours = placed.axes->values().front();
            if (std::abs(roboDk - ours) > 1.0e-6) {
                QString ignored;
                if (placed.axes->setValues({roboDk}, &ignored)) {
                    placed.spec.axes = placed.axes->values();
                    writeLibraryItemSpec(m_rdk, placed.item, placed.spec, &placed.blob);
                    CadTransform stand;
                    if (placedItemStand(m_rdk, placed, &stand)) bakePlacedItem(placed, stand);
                    moved = true;
                } else {
                    pushRailMechanismJoint(placed);
                }
            }
        }

        // The placement goes the other way: the base frame is a real RoboDK frame, so RoboDK's own
        // move tools reach it. A generic item answers PoseAbs as identity however its pose is set.
        Item frame = railMechanismBaseFrame(m_rdk, placed.item);
        if (!frame) continue;
        const CadTransform standing =
            rdkbridge::toCadTransform(frame->Pose()) * placed.axes->railBaseInTree().rigidInverse();
        if (!sameStanding(standing, placed.poseLocal)) {
            placed.poseLocal = standing;
            placed.hasPose = true;
            placed.spec.poseLocal = standing;
            placed.spec.hasPose = true;
            writeLibraryItemSpec(m_rdk, placed.item, placed.spec, &placed.blob);
            CadTransform stand;
            if (placedItemStand(m_rdk, placed, &stand)) bakePlacedItem(placed, stand);
            moved = true;
        }
    }
    return moved;
}

void PhysicsWorld::pushRailMechanismJoint(PlacedLibraryItem& placed) {
    if (!placed.axes || placed.axes->kind() != PlacedItemAxes::Kind::Rail) return;
    if (!placed.item || (m_rdk && !m_rdk->Valid(placed.item))) return;
    const double value = placed.axes->values().front();
    const tJoints joints(&value, 1);
    placed.item->setJoints(joints);
}

bool PhysicsWorld::remeasureMovedPlacedItems() {
    bool moved = m_conveyorHost.remeasureMoved();
    if (moved) flushConveyorParameters();
    for (PlacedItem& placed : m_libraryItems) {
        CadTransform stand;
        if (!placedItemStand(m_rdk, placed, &stand)) continue;
        if (!placedItemHasDrifted(placed, stand)) continue;
        bakePlacedItem(placed, stand);
        moved = true;
    }
    return moved;
}

void PhysicsWorld::flushConveyorParameters() {
    for (size_t index = 0; index < m_conveyorHost.segments().size(); ++index) {
        const conveyorcore::ConveyorId id = static_cast<conveyorcore::ConveyorId>(index + 1);
        if (RoboDkConveyorHost::Segment* segment = m_conveyorHost.mutableSegment(id)) {
            writeConveyorParameters(m_rdk, *segment);
        }
    }
}

void PhysicsWorld::forgetConveyor(conveyorcore::ConveyorId id) {
    const RoboDkConveyorHost::Segment* segment = m_conveyorHost.mutableSegment(id);
    if (!segment || segment->name.isEmpty()) return;
    darkenConveyorIo(segment->name);
    m_conveyorHost.forget(id);
    syncSpawners();
}

bool PhysicsWorld::deleteConveyor(const QString& conveyor, QString* error) {
    const conveyorcore::ConveyorId id = m_conveyorHost.idNamed(conveyor);
    RoboDkConveyorHost::Segment* segment = m_conveyorHost.mutableSegment(id);
    if (!segment) {
        if (error) *error = QStringLiteral("There is no conveyor called '%1'.").arg(conveyor);
        return false;
    }
    Item doomed = segment->item;
    forgetConveyor(id);
    // The item last, and only after the conveyor has been forgotten. Its products are its children, so
    // RoboDK takes them with it - and a Delete comes straight back as an EventChanged, which is where
    // the participants holding those now-dangling items are pruned.
    if (doomed && m_rdk && m_rdk->Valid(doomed)) doomed->Delete();
    if (error) error->clear();
    return true;
}

QString PhysicsWorld::freeItemName(const QString& base) const {
    if (base.isEmpty()) return base;
    // RoboDK does not uniquify a colliding generic item name on AddItem or setName.
    for (int suffix = 1; suffix < 1000; ++suffix) {
        const QString candidate = suffix == 1 ? base
                                             : QStringLiteral("%1 %2").arg(base).arg(suffix);
        if (m_conveyorHost.idNamed(candidate) != 0) continue;
        bool taken = false;
        for (const PlacedLibraryItem& placed : m_libraryItems) {
            if (placed.name == candidate) { taken = true; break; }
        }
        if (taken) continue;
        if (m_rdk && m_rdk->getItem(candidate)) continue;
        return candidate;
    }
    return base;
}

QString PhysicsWorld::freeConveyorName() const {
    return freeItemName(QStringLiteral("Conveyor"));
}

bool PhysicsWorld::addConveyor(QString* outName, QString* error) {
    if (!m_rdk) {
        if (error) *error = QStringLiteral("There is no station to add a conveyor to.");
        return false;
    }
    RoboDkConveyorHost::Segment segment;
    segment.parameters.accessoryGenerator = "roller_conveyor";
    segment.parameters.accessoryConveyorMode = "logical";
    // CadNode's own accessory defaults, brought up to the schema's floors by the geometry's own clamp.
    // Nothing here invents a number: a new conveyor is the smallest valid one, and the properties panel
    // is where it becomes the one a cell wants.
    clampRollerConveyorParameters(segment.parameters);

    Item item = createConveyorItem(m_rdk, freeConveyorName(), error);
    if (!item) return false;
    segment.item = item;
    segment.parent = item;
    segment.name = item->Name();
    segment.hasPose = true;
    QString refusal;
    if (!configureConveyor(segment, &refusal)) {
        item->Delete();
        if (error) *error = refusal;
        return false;
    }
    if (outName) *outName = segment.name;
    if (error) error->clear();
    return true;
}

const RoboDkConveyorHost::Segment* PhysicsWorld::conveyorNamed(const QString& name) const {
    const conveyorcore::ConveyorId id = m_conveyorHost.idNamed(name);
    if (id == 0) return nullptr;
    return &m_conveyorHost.segments()[static_cast<size_t>(id - 1)];
}

bool PhysicsWorld::setConveyorPlacement(const QString& conveyor, const CadTransform& poseLocal,
                                        QString* error) {
    const conveyorcore::ConveyorId id = m_conveyorHost.idNamed(conveyor);
    RoboDkConveyorHost::Segment* segment = m_conveyorHost.mutableSegment(id);
    if (!segment || !segment->item) {
        if (error) *error = QStringLiteral("There is no conveyor called '%1' to move.").arg(conveyor);
        return false;
    }
    segment->poseLocal = poseLocal;
    segment->hasPose = true;
    m_conveyorHost.measureSegment(*segment);
    writeConveyorParameters(m_rdk, *segment);
    if (error) error->clear();
    return true;
}

bool PhysicsWorld::applyConveyorParameters(const QString& conveyor,
                                           const TransformNodeData& parameters, QString* error) {
    const conveyorcore::ConveyorId id = m_conveyorHost.idNamed(conveyor);
    if (id == 0) {
        if (error) *error = QStringLiteral("There is no conveyor called '%1'.").arg(conveyor);
        return false;
    }
    if (!m_conveyorHost.reconfigure(id, parameters, error)) return false;
    syncSpawners();
    if (RoboDkConveyorHost::Segment* segment = m_conveyorHost.mutableSegment(id)) {
        writeConveyorParameters(m_rdk, *segment);
    }
    return true;
}

bool PhysicsWorld::setConveyorWorkpiece(const QString& conveyor, const QString& object,
                                        QString* error) {
    const RoboDkConveyorHost::Segment* segment =
        m_conveyorHost.mutableSegment(m_conveyorHost.idNamed(conveyor));
    if (!segment) {
        if (error) *error = QStringLiteral("There is no conveyor called '%1'.").arg(conveyor);
        return false;
    }
    TransformNodeData edited = segment->parameters;
    edited.accessorySpawnObjectId = object.toStdString();
    return applyConveyorParameters(conveyor, edited, error);
}

bool PhysicsWorld::renameConveyor(const QString& conveyor, const QString& wanted, QString* error) {
    const conveyorcore::ConveyorId id = m_conveyorHost.idNamed(conveyor);
    RoboDkConveyorHost::Segment* segment = m_conveyorHost.mutableSegment(id);
    if (!segment || !segment->item || (m_rdk && !m_rdk->Valid(segment->item))) {
        if (error) *error = QStringLiteral("There is no conveyor called '%1'.").arg(conveyor);
        return false;
    }
    if (m_conveyorHost.idNamed(wanted) != 0) {
        if (error) {
            *error = QStringLiteral("A conveyor is already called '%1'; two would write the same "
                                    "three IO variables.").arg(wanted);
        }
        return false;
    }
    segment->item->setName(wanted);
    const QString was = segment->name;
    segment->name = segment->item->Name();
    if (was != segment->name) darkenConveyorIo(was);
    if (error) error->clear();
    return true;
}

void PhysicsWorld::darkenConveyorIo(const QString& conveyor) {
    if (!m_rdk || conveyor.isEmpty()) return;
    for (const char* suffix : {" ready", " full", " count"}) {
        m_rdk->setParam(conveyor + QLatin1String(suffix), QStringLiteral("0"));
    }
    // publishConveyorIo realigns the parallel IO slots after the segment is erased.
}

void PhysicsWorld::configureSpawner(Item prototype, Item parent, double intervalSeconds,
                                    int maxActiveSpawns) {
    if (!prototype) return;
    RoboDkConveyorHost::Segment segment;
    segment.name = prototype->Name();
    segment.prototype = prototype;
    segment.parent = parent ? parent : prototype->Parent();
    segment.parameters.accessoryGenerator = "roller_conveyor";
    segment.parameters.accessoryConveyorMode = "logical";
    segment.parameters.accessoryConveyorRole = "spawner";
    // No transport: the station carries these. Zero speed keeps their progress at the start of a
    // path that does not exist, which is the honest description of a box riding RoboDK's own
    // Conveyor Belt mechanism - and the length is the rules' own default, because there is no
    // geometry to take one from and the queue arithmetic still has to mean something.
    segment.parameters.accessoryConveyorSpeedMmS = 0.0;
    segment.parameters.accessoryLengthMm = conveyorcore::ConveyorSpec().pathLengthMm;
    segment.parameters.accessorySpawnIntervalSeconds = intervalSeconds;
    segment.parameters.accessoryMaxActiveSpawns = std::max(0, maxActiveSpawns);
    segment.parameters.accessorySpawnObjectId = prototype->Name().toStdString();
    segment.spec = conveyorcore::conveyorSpecFrom(segment.parameters, conveyorcore::Mode::Logical);
    configureConveyor(segment);
}

void PhysicsWorld::clearConveyors() {
    for (ConveyorIo& io : m_conveyorIo) {
        if (!m_rdk) break;
        if (io.ready != 0) m_rdk->setParam(io.conveyor + QStringLiteral(" ready"), QStringLiteral("0"));
        if (io.full != 0) m_rdk->setParam(io.conveyor + QStringLiteral(" full"), QStringLiteral("0"));
        if (io.count != 0) m_rdk->setParam(io.conveyor + QStringLiteral(" count"), QStringLiteral("0"));
    }
    m_conveyorIo.clear();
    m_conveyors.reset();
    m_conveyorHost.clear();
}


void PhysicsWorld::publishConveyorIo() {
    if (!m_rdk) return;
    // setParam is the live publish. IItem::setDO / waitDI / setAO look like the right names and are
    // not: they *emit instructions into a program*. setParam is what releases a native Wait for I/O
    // and is the only way a plugin writes a value a running station can read.
    const bool running = m_state == State::Running;
    const std::vector<RoboDkConveyorHost::Segment>& segments = m_conveyorHost.segments();
    m_conveyorIo.resize(segments.size());
    for (size_t index = 0; index < segments.size(); ++index) {
        ConveyorIo& io = m_conveyorIo[index];
        if (io.conveyor != segments[index].name) {
            io = ConveyorIo();
            io.conveyor = segments[index].name;
        }
        const conveyorcore::ConveyorId id = static_cast<conveyorcore::ConveyorId>(index + 1);
        const int ready = running && m_conveyors.endStopOccupied(id) ? 1 : 0;
        const int full = running && m_conveyors.entryBlocked(id) ? 1 : 0;
        const int count = running ? static_cast<int>(m_conveyors.productsOn(id)) : 0;
        if (ready != io.ready) {
            io.ready = ready;
            m_rdk->setParam(io.conveyor + QStringLiteral(" ready"), QString::number(ready));
        }
        if (full != io.full) {
            io.full = full;
            m_rdk->setParam(io.conveyor + QStringLiteral(" full"), QString::number(full));
        }
        if (count != io.count) {
            io.count = count;
            m_rdk->setParam(io.conveyor + QStringLiteral(" count"), QString::number(count));
        }
    }
}

QString PhysicsWorld::configString() const {
    QStringList parts;
    parts << QStringLiteral("autostart=%1").arg(m_autoStart ? 1 : 0);
    parts << QStringLiteral("gravity=%1").arg(m_gravityMps2.z);
    for (const Participant& participant : m_participants) {
        if (participant.spawned || participant.role == Role::None) continue;
        parts << QStringLiteral("role=%1:%2").arg(participant.name, roleName(participant.role).toLower());
    }
    for (const RoboDkConveyorHost::Segment& segment : m_conveyorHost.segments()) {
        // Only the pathless ones. A conveyor is an item now, and its description is saved with that
        // item by RoboDK itself - writing it here as well would be two descriptions of one thing,
        // which is the whole failure this moved away from. `conveyor=` is still *read*, once, to
        // migrate a station written before that.
        if (segment.item || !segment.prototype) continue;
        parts << QStringLiteral("spawner=%1:%2:%3:%4")
                     .arg(segment.prototype->Name(),
                          segment.parent ? segment.parent->Name() : QString(),
                          QString::number(segment.spec.spawnIntervalSeconds),
                          QString::number(segment.spec.maxActiveSpawns));
    }
    return parts.join(QLatin1Char(';'));
}

void PhysicsWorld::applyConfigString(const QString& text, QStringList* warnings) {
    if (!m_rdk) return;
    if (m_state != State::Stopped) stop();
    m_participants.clear();
    clearConveyors();
    m_autoStart = false;

    bool migrated = false;
    const QStringList entries = text.split(QLatin1Char(';'), Qt::SkipEmptyParts);
    for (const QString& entry : entries) {
        const int equals = entry.indexOf(QLatin1Char('='));
        if (equals <= 0) continue;
        const QString key = entry.left(equals).trimmed().toLower();
        const QString value = entry.mid(equals + 1).trimmed();

        if (key == QLatin1String("autostart")) {
            m_autoStart = value.toInt() != 0;
        } else if (key == QLatin1String("gravity")) {
            bool ok = false;
            const double z = value.toDouble(&ok);
            if (ok) m_gravityMps2 = CadVec3{0.0, 0.0, z};
        } else if (key == QLatin1String("role")) {
            const int colon = value.lastIndexOf(QLatin1Char(':'));
            if (colon <= 0) continue;
            const QString name = value.left(colon);
            const QString wanted = value.mid(colon + 1).toLower();
            Item item = m_rdk->getItem(name);
            if (!item) {
                if (warnings) *warnings << QStringLiteral("no item named '%1'").arg(name);
                continue;
            }
            Role role = Role::None;
            if (wanted == QLatin1String("static")) role = Role::Static;
            else if (wanted == QLatin1String("dynamic")) role = Role::Dynamic;
            else if (wanted == QLatin1String("kinematic")) role = Role::Kinematic;
            setRole(item, role);
        } else if (key == QLatin1String("spawner")) {
            const QStringList fields = value.split(QLatin1Char(':'));
            if (fields.isEmpty()) continue;
            Item prototype = m_rdk->getItem(fields.value(0));
            if (!prototype) {
                if (warnings) *warnings << QStringLiteral("no spawner prototype '%1'").arg(fields.value(0));
                continue;
            }
            Item parent = fields.size() > 1 && !fields.value(1).isEmpty()
                ? m_rdk->getItem(fields.value(1)) : nullptr;
            const double interval = fields.value(2).toDouble();
            const int cap = fields.value(3).toInt();
            configureSpawner(prototype, parent, interval > 0.0 ? interval : 1.0, cap);
        } else if (key == QLatin1String("conveyor")) {
            migrated = migrateConveyorEntry(value, warnings) || migrated;
        }
    }
    adoptConveyorItems(warnings);
    if (migrated && m_rdk) m_rdk->setParam(kConfigParam, configString());
}

bool PhysicsWorld::migrateConveyorEntry(const QString& value, QStringList* warnings) {
    const QStringList legacy = value.split(QLatin1Char(':'));
    const QString wanted = legacy.value(0).trimmed();
    if (wanted.isEmpty()) {
        if (warnings) *warnings << QStringLiteral("a conveyor= entry names no conveyor");
        return false;
    }
    // Silently, when the frame it named is not there. That is what a station looks like once this has
    // already run and been saved - the frame has been renamed to the conveyor's name - and a warning
    // on every open would be noise about work already done. A station where the frame genuinely never
    // existed simply declares no conveyor, and `conveyors` is where that shows.
    Item start = m_rdk->getItem(legacy.value(1).trimmed());
    if (!start) return false;
    QByteArray existing;
    if (start->getParam(kConveyorItemParam, existing) && !existing.trimmed().isEmpty()) {
        return false;
    }

    RoboDkConveyorHost::Segment segment;
    QString error;
    // The name and the start frame have become one field, so the parser is handed the rest.
    if (!parseConveyorSegment(m_rdk, legacy.mid(1), &segment, &error)) {
        if (warnings) *warnings << error;
        return false;
    }
    if (segment.item->Name() != wanted) {
        segment.item->setName(wanted);
        segment.name = segment.item->Name();
    }
    return writeConveyorParameters(m_rdk, segment);
}

void PhysicsWorld::clearSpawned() {
    // Remove bookkeeping before Delete(), whose synchronous EventChanged callback prunes these lists.
    std::vector<Item> doomed;
    for (Participant& participant : m_participants) {
        if (!participant.spawned) continue;
        if (participant.body != 0) {
            m_physics.removeBody(participant.body);
            participant.body = 0;
        }
        doomed.push_back(participant.item);
    }
    m_participants.erase(
        std::remove_if(m_participants.begin(), m_participants.end(),
                       [](const Participant& p) { return p.spawned; }),
        m_participants.end());
    for (const conveyorcore::Product& product : m_conveyors.products()) {
        m_conveyorHost.forgetProduct(product.id);
    }
    m_conveyors.setProducts({});
    for (Item item : doomed) {
        if (m_rdk && m_rdk->Valid(item)) item->Delete();
    }
}

void PhysicsWorld::pause() {
    if (m_state == State::Running) m_state = State::Paused;
}

void PhysicsWorld::resume() {
    if (m_state == State::Paused) m_state = State::Running;
}

void PhysicsWorld::stop() {
    if (m_state == State::Stopped) return;
    m_state = State::Stopped;
    publishConveyorIo();
    clearSpawned();
    // Poses go back before the scene does, because restoring reads the participant list that
    // releaseScene leaves intact but empty of bodies.
    for (Participant& participant : m_participants) {
        participant.carried = false;
        participant.heldByTool = false;
        if (!participant.restoreValid) continue;
        if (m_rdk && !m_rdk->Valid(participant.item)) continue;
        participant.item->setPoseAbs(rdkbridge::toRoboDkPose(participant.restorePose));
        participant.status = QStringLiteral("Restored.");
    }
    releaseScene();
}

bool PhysicsWorld::itemIsHeldByTool(Item item) const {
    if (!item || !m_rdk) return false;
    Item cursor = item;
    for (int depth = 0; depth < 32; ++depth) {
        Item parent = cursor->Parent();
        if (!parent || !m_rdk->Valid(parent) || parent == cursor) return false;
        if (parent->Type() == IItem::ITEM_TYPE_TOOL) return true;
        if (parent->Type() == IItem::ITEM_TYPE_ROBOT ||
            parent->Type() == IItem::ITEM_TYPE_STATION) {
            return false;
        }
        cursor = parent;
    }
    return false;
}

void PhysicsWorld::adoptToolChildren() {
    if (!m_rdk) return;
    std::vector<Item> tools;
    for (const Participant& participant : m_participants) {
        if (participant.role != Role::Kinematic) continue;
        if (!m_rdk->Valid(participant.item)) continue;
        if (participant.item->Type() == IItem::ITEM_TYPE_TOOL) tools.push_back(participant.item);
    }
    for (Item tool : tools) {
        // Direct children only. RoboDK's own Attach re-parents an object straight onto the tool, and
        // reaching deeper would start claiming whatever a station has mounted on its gripper.
        for (Item child : tool->Childs()) {
            if (!child || !m_rdk->Valid(child)) continue;
            if (child->Type() != IItem::ITEM_TYPE_OBJECT || find(child)) continue;
            Participant adopted;
            adopted.item = child;
            adopted.name = child->Name();
            adopted.role = Role::Dynamic;
            adopted.carried = true;
            adopted.heldByTool = true;
            adopted.restorePose = rdkbridge::toCadTransform(child->PoseAbs());
            adopted.restoreValid = true;
            adopted.status = QStringLiteral("Adopted from %1.").arg(tool->Name());
            m_participants.push_back(adopted);
            ++m_carriedCount;
        }
    }
}

void PhysicsWorld::updateCarryStates() {
    adoptToolChildren();
    for (size_t index = 0; index < m_participants.size(); ++index) {
        Participant& participant = m_participants[index];
        if (participant.role != Role::Dynamic) continue;
        if (m_rdk && !m_rdk->Valid(participant.item)) continue;

        const bool held = itemIsHeldByTool(participant.item);
        if (held == participant.heldByTool) continue;
        participant.heldByTool = held;

        const conveyorcore::ProductId product = m_conveyorHost.productFor(participant.item);
        QString error;
        if (held) {
            if (product != 0) m_conveyors.setGrasped(product, true);
            attach(participant.item, &error);
        } else if (participant.carried) {
            if (product != 0) {
                m_conveyors.removeProduct(product);
                m_conveyorHost.forgetProduct(product);
            }
            // It was the station's and has just left a tool: put down. release() re-parents it out
            // of the tool first, so RoboDK stops carrying something PhysX is also placing.
            release(participant.item, &error);
        }
        if (!error.isEmpty()) m_lastError = error;
    }
}

bool PhysicsWorld::step(double seconds) {
    if (m_state != State::Running || seconds <= 0.0) return false;
    // A step makes RoboDK API calls and RoboDK answers some of them with a synchronous event, so
    // this can be re-entered. Nothing about advancing a simulation is re-entrant.
    if (m_stepping) return false;
    if (m_pruneRequested) pruneInvalidItems();
    m_stepping = true;
    struct Guard {
        bool* flag;
        ~Guard() { *flag = false; }
    } guard{&m_stepping};
    ++m_stepCount;
    m_steppedSeconds += seconds;

    m_conveyors.step(seconds);
    updateCarryStates();

    for (Participant& participant : m_participants) {
        if (participant.role != Role::Kinematic || participant.body == 0) continue;
        if (m_rdk && !m_rdk->Valid(participant.item)) continue;
        m_physics.setKinematicPose(participant.body,
                                   rdkbridge::toCadTransform(participant.item->PoseAbs()));
    }

    m_physics.step(seconds);

    bool moved = m_conveyorHost.consumeStationChanged();
    for (Participant& participant : m_participants) {
        if (participant.role != Role::Dynamic || participant.body == 0) continue;
        if (m_rdk && !m_rdk->Valid(participant.item)) continue;
        CadTransform pose;
        if (!m_physics.bodyPose(participant.body, &pose)) continue;
        participant.item->setPoseAbs(rdkbridge::toRoboDkPose(pose));
        moved = true;
    }

    publishConveyorIo();
    return moved;
}

QString PhysicsWorld::itemReport() const {
    QStringList records;
    for (const Participant& participant : m_participants) {
        records << QStringLiteral("%1 | %2 | %3 | %4 | %5 | %6 tri | %7")
                       .arg(participant.name,
                            roleName(participant.role),
                            participant.carried ? QStringLiteral("carried") : QStringLiteral("free"),
                            participant.heldByTool ? QStringLiteral("held") : QStringLiteral("loose"),
                            participant.body != 0 ? QStringLiteral("body") : QStringLiteral("nobody"),
                            QString::number(participant.triangles),
                            participant.status);
    }
    return records.join(QStringLiteral(" || "));
}

QString PhysicsWorld::cappedSpawnerReport(const RoboDkConveyorHost::Segment& segment) const {
    if (segment.spec.role != conveyorcore::Role::Spawner) return QStringLiteral("cap n/a");
    const conveyorcore::ConveyorId id = m_conveyorHost.idNamed(segment.name);
    if (segment.spec.maxActiveSpawns <= 0) {
        return QStringLiteral("cap unlimited, %1 live").arg(m_conveyors.activeSpawnCount(id));
    }
    // Said out loud, because a cap doing exactly what it was asked to and a line that has broken read
    // identically otherwise - and a capped spawner whose products can never leave the flow is a
    // deadlock. The product records below give the other half: a progress of 1.000000 that never
    // changes is one of those products.
    return QStringLiteral("cap %1/%2%3")
        .arg(m_conveyors.activeSpawnCount(id))
        .arg(segment.spec.maxActiveSpawns)
        .arg(m_conveyors.spawnerAtCap(id) ? QStringLiteral(" - AT CAP, not feeding") : QString());
}

QString PhysicsWorld::conveyorReport() const {
    QStringList records;
    for (size_t index = 0; index < m_conveyorHost.segments().size(); ++index) {
        const RoboDkConveyorHost::Segment& segment = m_conveyorHost.segments()[index];
        CadTransform stand;
        const bool path = m_conveyorHost.placement(segment, &stand);
        const CadVec3 startMm = m_conveyorHost.pathPoint(segment, 0.0);
        const CadVec3 endMm = m_conveyorHost.pathPoint(segment, 1.0);
        records << QStringLiteral("c %1 | %2 | speed %3 | length %4 | next '%5' | %6 | %7 | %8")
                       .arg(segment.name, conveyorRoleName(segment.spec.role),
                            QString::number(segment.spec.speedMmS),
                            QString::number(segment.spec.pathLengthMm), segment.nextName,
                            cappedSpawnerReport(segment),
                            path ? QStringLiteral("path %1,%2,%3 -> %4,%5,%6")
                                       .arg(QString::number(startMm.x), QString::number(startMm.y),
                                            QString::number(startMm.z), QString::number(endMm.x),
                                            QString::number(endMm.y), QString::number(endMm.z))
                                 : QStringLiteral("no path - the station carries these"),
                            segment.scenery.triangles > 0
                                ? QStringLiteral("drawn %1 tri")
                                      .arg(QString::number(segment.scenery.triangles))
                                : QStringLiteral("not drawn"));
    }
    for (size_t index = 0; index < m_conveyorHost.segments().size(); ++index) {
        const QString& name = m_conveyorHost.segments()[index].name;
        const ConveyorIo* io = index < m_conveyorIo.size() && m_conveyorIo[index].conveyor == name
            ? &m_conveyorIo[index] : nullptr;
        const auto value = [io](int ConveyorIo::*field) {
            return io ? QString::number(io->*field) : QStringLiteral("-");
        };
        records << QStringLiteral("io %1 ready=%2 | %1 full=%3 | %1 count=%4")
                       .arg(name, value(&ConveyorIo::ready), value(&ConveyorIo::full),
                            value(&ConveyorIo::count));
    }
    for (const conveyorcore::Product& product : m_conveyors.products()) {
        const RoboDkConveyorHost::Segment* segment = nullptr;
        if (product.conveyor != 0 && product.conveyor <= m_conveyorHost.segments().size()) {
            segment = &m_conveyorHost.segments()[static_cast<size_t>(product.conveyor - 1)];
        }
        records << QStringLiteral("p %1 | on %2 | progress %3 | %4 | %5")
                       .arg(QString::number(product.id),
                            segment ? segment->name : QStringLiteral("?"),
                            QString::number(product.progress, 'f', 6),
                            product.forward ? QStringLiteral("forward") : QStringLiteral("reverse"),
                            product.grasped ? QStringLiteral("grasped") : QStringLiteral("free"));
    }
    return records.join(QStringLiteral(" || "));
}

QString PhysicsWorld::summary() const {
    switch (m_state) {
    case State::Running:
    case State::Paused: {
        const QString label = m_state == State::Running ? QStringLiteral("Running")
                                                        : QStringLiteral("Paused");
        size_t atCap = 0;
        for (const RoboDkConveyorHost::Segment& segment : m_conveyorHost.segments()) {
            if (m_conveyors.spawnerAtCap(m_conveyorHost.idNamed(segment.name))) ++atCap;
        }
        return QStringLiteral("%1 - %2 static, %3 dynamic, %4 kinematic, %5 carried, "
                              "%6 conveyor(s), %7 product(s), %8 step(s) over %9 s%10.")
            .arg(label).arg(m_staticCount).arg(m_dynamicCount).arg(m_kinematicCount)
            .arg(m_carriedCount).arg(m_conveyorHost.segments().size())
            .arg(m_conveyors.products().size()).arg(m_stepCount)
            .arg(m_steppedSeconds, 0, 'f', 2)
            .arg(atCap == 0 ? QString()
                            : QStringLiteral(", %1 spawner(s) at cap and not feeding").arg(atCap));
    }
    case State::Stopped:
        break;
    }
    if (!m_lastError.isEmpty()) return m_lastError;
    return QStringLiteral("Stopped - %1 item(s) marked.").arg(m_participants.size());
}
