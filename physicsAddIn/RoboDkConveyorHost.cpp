#include "RoboDkConveyorHost.h"

#include "PhysicsWorld.h"
#include "RoboDkBridge.h"

#include "AccessoryGeometry.h"
#include "ConveyorGeometry.h"

#include "iitem.h"
#include "irobodk.h"

#include <QDir>
#include <QFile>

#include <algorithm>
#include <cmath>

const char* const kConveyorItemParam = "PhysicsConveyor";

namespace {

using conveyorcore::ConveyorId;
using conveyorcore::Product;
using conveyorcore::ProductId;

constexpr double kDefaultDeckWidthMm = 700.0;
constexpr double kDefaultDeckHeightMm = 890.0;

CadVec3 positionOf(const CadTransform& transform) {
    return CadVec3{transform.values[3], transform.values[7], transform.values[11]};
}

// Blank means "no opinion, take the default"; a stated 0 has to survive as a zero. The difference
// matters for a lane whose far end is already the delivery point: giving it the 70 mm inset a roller
// deck has would move its stop, and with it the pick pose.
bool applyIfStated(const QString& field, double* into) {
    const QString text = field.trimmed();
    if (text.isEmpty()) return false;
    *into = std::max(0.0, text.toDouble());
    return true;
}

// One column of a pose, unit length: the conveyor's own axes in station coordinates. Column 0 runs
// along the lane, 1 across it and 2 up.
CadVec3 axisOf(const CadTransform& pose, int column) {
    const CadVec3 axis(pose.values[column], pose.values[4 + column], pose.values[8 + column]);
    const double length = lengthOf(axis);
    return length > 1.0e-9 ? CadVec3(axis.x / length, axis.y / length, axis.z / length) : CadVec3();
}

void deriveSegment(RoboDkConveyorHost::Segment* segment) {
    clampRollerConveyorParameters(segment->parameters);
    segment->spec = conveyorcore::conveyorSpecFrom(segment->parameters,
                                                   conveyorcore::Mode::Logical);
}

} // namespace

RoboDkConveyorHost::RoboDkConveyorHost(RoboDK* rdk, PhysicsWorld* world)
    : m_rdk(rdk), m_world(world) {}

void RoboDkConveyorHost::clear() {
    m_segments.clear();
    m_items.clear();
}

ConveyorId RoboDkConveyorHost::add(const Segment& segment) {
    for (size_t index = 0; index < m_segments.size(); ++index) {
        if (!m_segments[index].name.isEmpty()) continue;
        m_segments[index] = segment;
        return static_cast<ConveyorId>(index + 1);
    }
    m_segments.push_back(segment);
    return static_cast<ConveyorId>(m_segments.size());
}

void RoboDkConveyorHost::forget(ConveyorId id) {
    Segment* segment = mutableSegment(id);
    if (!segment) return;
    *segment = Segment();
}

ConveyorId RoboDkConveyorHost::idNamed(const QString& name) const {
    if (name.isEmpty()) return 0;
    for (size_t index = 0; index < m_segments.size(); ++index) {
        if (m_segments[index].name == name) return static_cast<ConveyorId>(index + 1);
    }
    return 0;
}

std::vector<ConveyorId> RoboDkConveyorHost::invalidConveyors() const {
    std::vector<ConveyorId> gone;
    if (!m_rdk) return gone;
    for (size_t index = 0; index < m_segments.size(); ++index) {
        const Segment& segment = m_segments[index];
        if (segment.name.isEmpty() || !segment.item) continue;
        if (!m_rdk->Valid(segment.item)) gone.push_back(static_cast<ConveyorId>(index + 1));
    }
    return gone;
}

ConveyorId RoboDkConveyorHost::idOfItem(Item item) const {
    if (!item) return 0;
    for (size_t index = 0; index < m_segments.size(); ++index) {
        if (m_segments[index].item == item) return static_cast<ConveyorId>(index + 1);
    }
    return 0;
}

RoboDkConveyorHost::Segment* RoboDkConveyorHost::mutableSegment(ConveyorId id) {
    if (id == 0 || id > m_segments.size()) return nullptr;
    return &m_segments[static_cast<size_t>(id - 1)];
}

const RoboDkConveyorHost::Segment* RoboDkConveyorHost::segmentOf(ConveyorId id) const {
    if (id == 0 || id > m_segments.size()) return nullptr;
    return &m_segments[static_cast<size_t>(id - 1)];
}

bool RoboDkConveyorHost::placement(const Segment& segment, CadTransform* world) const {
    return placedItemStand(m_rdk, segment, world);
}

bool RoboDkConveyorHost::lane(const Segment& segment, CadTransform* world) const {
    if (!placement(segment, world)) return false;
    // The accessory's own frame, which is what conveyorPathPoseAt answers in. Folding the change of
    // basis in here is what lets every shared path routine be asked directly, instead of the plugin
    // carrying a second path model to answer the same questions in RoboDK's axes.
    *world = *world * conveyorAccessoryFrame();
    return true;
}

CadVec3 RoboDkConveyorHost::pathPoint(const Segment& segment, double progress) const {
    CadTransform world;
    if (!lane(segment, &world)) return CadVec3();
    return conveyorTransformPoint(world, conveyorPathPoseAt(segment.parameters, progress));
}

void RoboDkConveyorHost::measureSegment(Segment& segment) {
    segment.scenery = ConveyorScenery();
    ConveyorSceneryRequest request;
    request.parameters = segment.parameters;
    CadTransform stand;
    if (!placement(segment, &stand)) return; // no machine in the cell, nothing to draw
    // Plugin-drawn scenery is baked in station coordinates and remeasured after moves.
    request.world = stand;
    segment.sceneryAt = stand;
    // Compose the accessory-frame basis before exposing mounting interfaces.
    segment.treeToWorld = stand * conveyorAccessoryFrame();

    std::shared_ptr<CadNode> prototype;
    if (segment.prototype && m_rdk && m_rdk->Valid(segment.prototype)) {
        rdkbridge::MeshData mesh;
        QString error;
        if (rdkbridge::exportItemMesh(m_rdk, segment.prototype, &mesh, &error) && mesh.valid()) {
            if (segment.spec.role == conveyorcore::Role::Spawner) {
                prototype = conveyorPrototypeNode(mesh.verticesMm, mesh.indices);
            }
            // Where the workpiece's body sits relative to the point that rides the lane. Exported
            // in the prototype's own frame, so its pose turns it into station axes - and the
            // translation is subtracted back out because this is an offset, not a position.
            CadVec3 low = mesh.verticesMm.front();
            CadVec3 high = low;
            for (const CadVec3& vertex : mesh.verticesMm) {
                low = CadVec3(std::min(low.x, vertex.x), std::min(low.y, vertex.y),
                              std::min(low.z, vertex.z));
                high = CadVec3(std::max(high.x, vertex.x), std::max(high.y, vertex.y),
                               std::max(high.z, vertex.z));
            }
            const CadTransform pose = rdkbridge::toCadTransform(segment.prototype->PoseAbs());
            segment.workpieceMeasuredAt = pose;
            const CadVec3 centre = pose * CadVec3((low.x + high.x) * 0.5, (low.y + high.y) * 0.5,
                                                  (low.z + high.z) * 0.5);
            segment.workpieceCentreOffsetMm = difference(centre, positionOf(pose));

            // Offset the spawn so the complete workpiece begins on the deck.
            const CadVec3 along = axisOf(stand, 0);
            const double laneLength = conveyorPathLengthMm(segment.parameters);
            double behind = 0.0;
            double ahead = 0.0;
            for (int corner = 0; corner < 8; ++corner) {
                const CadVec3 local((corner & 1) ? high.x : low.x, (corner & 2) ? high.y : low.y,
                                    (corner & 4) ? high.z : low.z);
                const CadVec3 offset = difference(pose * local, positionOf(pose));
                behind = std::min(behind, dot(offset, along));
                ahead = std::max(ahead, dot(offset, along));
            }
            segment.spawnProgress = std::min(1.0, -behind / laneLength);
            segment.workpieceLengthAlongLaneMm = ahead - behind;
            if (conveyorcore::raisePitchToFitWorkpiece(segment.parameters,
                                                       segment.workpieceLengthAlongLaneMm)) {
                deriveSegment(&segment);
            }
        }
    }
    request.spawnPrototype = prototype.get();
    segment.scenery = buildConveyorScenery(request);
}

bool RoboDkConveyorHost::workpieceHasTurned(const Segment& segment) const {
    if (!segment.prototype || !m_rdk || !m_rdk->Valid(segment.prototype)) return false;
    const CadTransform now = rdkbridge::toCadTransform(segment.prototype->PoseAbs());
    for (int cell : {0, 1, 2, 4, 5, 6, 8, 9, 10}) {
        if (now.values[cell] != segment.workpieceMeasuredAt.values[cell]) return true;
    }
    return false;
}

bool RoboDkConveyorHost::remeasureMoved() {
    bool moved = false;
    for (Segment& segment : m_segments) {
        if (segment.name.isEmpty() || segment.scenery.groups.empty()) continue;
        CadTransform stand;
        if (!placement(segment, &stand)) continue;
        if (!placedItemHasDrifted(segment, stand) && !workpieceHasTurned(segment)) continue;
        measureSegment(segment);
        moved = true;
    }
    return moved;
}

namespace {

QString conveyorIconPath() {
    static const QString path = [] {
        static const char* const kSvg = R"svg(<?xml version="1.0" encoding="UTF-8"?>
<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 256 256" width="256" height="256">
  <rect x="86" y="60" width="84" height="56" rx="6" fill="#d9932b" stroke="#8a5c14" stroke-width="6"/>
  <path d="M104 88h48M128 72v32" stroke="#8a5c14" stroke-width="6" stroke-linecap="round"/>
  <rect x="18" y="124" width="220" height="16" rx="8" fill="#3f4a55"/>
  <g fill="#9aa8b5" stroke="#3f4a55" stroke-width="6">
    <circle cx="42" cy="164" r="15"/><circle cx="85" cy="164" r="15"/>
    <circle cx="128" cy="164" r="15"/><circle cx="171" cy="164" r="15"/>
    <circle cx="214" cy="164" r="15"/>
  </g>
  <rect x="18" y="188" width="220" height="14" rx="7" fill="#3f4a55"/>
  <path d="M40 202v34M216 202v34" stroke="#3f4a55" stroke-width="14" stroke-linecap="round"/>
</svg>
)svg";
        const QString file = QDir(QDir::tempPath()).filePath(QStringLiteral("physics_conveyor.svg"));
        QFile out(file);
        if (!out.open(QIODevice::WriteOnly | QIODevice::Truncate)) return QString();
        out.write(kSvg);
        out.close();
        return file;
    }();
    return path;
}

} // namespace

Item createConveyorItem(RoboDK* rdk, const QString& name, QString* error) {
    if (!rdk) return nullptr;
    const QList<Item> before = rdk->getItemList(IItem::ITEM_TYPE_GENERIC);
    rdk->Command(QStringLiteral("AddItem"), name);
    Item node = nullptr;
    for (Item candidate : rdk->getItemList(IItem::ITEM_TYPE_GENERIC)) {
        if (candidate && !before.contains(candidate)) { node = candidate; break; }
    }
    if (!node) {
        if (error) {
            *error = QStringLiteral("RoboDK would not add a generic item for conveyor '%1'.").arg(name);
        }
        return nullptr;
    }
    // IconSet requires a generic item and the QString overload; failure is non-fatal.
    const QString icon = conveyorIconPath();
    if (!icon.isEmpty()) node->setParam(QStringLiteral("IconSet"), icon);
    node->setName(name);
    if (error) error->clear();
    return node;
}

bool RoboDkConveyorHost::reconfigure(ConveyorId id, const TransformNodeData& parameters,
                                     QString* error) {
    Segment* segment = mutableSegment(id);
    if (!segment) {
        if (error) *error = QStringLiteral("There is no such conveyor.");
        return false;
    }
    Item prototype = nullptr;
    const QString wanted = QString::fromStdString(parameters.accessorySpawnObjectId);
    if (!wanted.isEmpty() && m_rdk) {
        prototype = m_rdk->getItem(wanted);
        if (!prototype) {
            if (error) {
                *error = QStringLiteral("There is no item called '%1' to spawn.").arg(wanted);
            }
            return false;
        }
    }
    const bool workpieceChanged = prototype != segment->prototype;
    segment->parameters = parameters;
    segment->prototype = prototype;
    deriveSegment(segment);
    measureSegment(*segment);
    if (workpieceChanged && segment->workpieceLengthAlongLaneMm > 0.0) {
        segment->parameters.accessoryInitialWorkpieceSpacingMm = segment->workpieceLengthAlongLaneMm;
        deriveSegment(segment);
    }
    if (error) error->clear();
    return true;
}

ProductId RoboDkConveyorHost::productFor(Item item) const {
    for (const auto& entry : m_items) {
        if (entry.second == item) return entry.first;
    }
    return 0;
}

Item RoboDkConveyorHost::itemFor(ProductId id) const {
    const auto hit = m_items.find(id);
    return hit == m_items.end() ? nullptr : hit->second;
}

void RoboDkConveyorHost::trackProduct(ProductId id, Item item) {
    if (id != 0 && item) m_items[id] = item;
}

void RoboDkConveyorHost::forgetProduct(ProductId id) {
    m_items.erase(id);
}

std::vector<ProductId> RoboDkConveyorHost::invalidProducts() const {
    std::vector<ProductId> gone;
    if (!m_rdk) return gone;
    for (const auto& entry : m_items) {
        if (!m_rdk->Valid(entry.second)) gone.push_back(entry.first);
    }
    return gone;
}

bool RoboDkConveyorHost::conveyorSpec(ConveyorId id, conveyorcore::ConveyorSpec* out) const {
    const Segment* segment = segmentOf(id);
    if (!segment || segment->name.isEmpty()) return false;
    *out = segment->spec;
    return true;
}

bool RoboDkConveyorHost::productHasBody(const Product& product) const {
    Item item = itemFor(product.id);
    return item && m_world && m_world->hasBody(item);
}

bool RoboDkConveyorHost::productBodyPose(const Product& product, CadTransform* world) const {
    Item item = itemFor(product.id);
    if (!item || !m_world || !m_world->bodyPoseOf(item, world)) return false;
    return true;
}

double RoboDkConveyorHost::closestProgress(ConveyorId id, const CadVec3& world) const {
    const Segment* segment = segmentOf(id);
    CadTransform lanePose;
    if (!segment || !lane(*segment, &lanePose)) return 0.0;
    return closestConveyorProgress(segment->parameters, lanePose, world);
}

bool RoboDkConveyorHost::inDeleterVolume(const Product& product, const CadVec3& world) const {
    const Segment* segment = segmentOf(product.conveyor);
    if (!segment || segment->spec.role != conveyorcore::Role::Deleter) return false;
    CadTransform lanePose;
    if (!lane(*segment, &lanePose)) return false;
    const double along = closestConveyorProgress(segment->parameters, lanePose, world);
    if (along < 0.5) return false;
    const CadVec3 nearest =
        conveyorTransformPoint(lanePose, conveyorPathPoseAt(segment->parameters, along));
    return lengthOf(difference(world, nearest)) <= segment->spec.pathLengthMm * 0.25;
}

bool RoboDkConveyorHost::reachedEnd(const Product& product, const CadVec3& world,
                                    double allowanceMm) const {
    const Segment* segment = segmentOf(product.conveyor);
    CadTransform lanePose;
    if (!segment || !lane(*segment, &lanePose)) return false;
    const double endpoint = product.forward ? 1.0 : 0.0;
    const CadVec3 exit =
        conveyorTransformPoint(lanePose, conveyorPathPoseAt(segment->parameters, endpoint));
    const CadVec3 outward =
        conveyorTangentAt(segment->parameters, lanePose, endpoint, product.forward);
    return dot(difference(world, exit), outward) >= -allowanceMm;
}

void RoboDkConveyorHost::notePhysicalCenter(const Product&, const CadVec3&) {
}

conveyorcore::TransferTarget RoboDkConveyorHost::nextConveyor(ConveyorId id,
                                                              bool leavingForward) const {
    conveyorcore::TransferTarget target;
    const Segment* segment = segmentOf(id);
    if (!segment || segment->name.isEmpty()) return target;

    if (leavingForward && !segment->nextName.isEmpty()) {
        const ConveyorId destination = idNamed(segment->nextName);
        if (destination != 0) {
            target.conveyor = destination;
            target.forward = true;
            target.valid = true;
        }
        return target;
    }

    // Without an explicit link, resolve the next lane geometrically.
    std::vector<conveyorcore::Lane> lanes;
    lanes.reserve(m_segments.size());
    for (const Segment& candidate : m_segments) {
        conveyorcore::Lane entry;
        CadTransform lanePose;
        // nextLane expects path poses with the accessory-frame basis already applied.
        if (!candidate.name.isEmpty() && lane(candidate, &lanePose)) {
            entry.parameters = &candidate.parameters;
            entry.world = lanePose;
            entry.mode = candidate.spec.mode;
        }
        lanes.push_back(entry);
    }
    const conveyorcore::Handover found =
        conveyorcore::nextLane(lanes, static_cast<size_t>(id - 1), leavingForward);
    if (!found.valid()) return target;
    target.conveyor = static_cast<ConveyorId>(found.lane + 1);
    target.forward = found.forward;
    target.valid = true;
    return target;
}

void RoboDkConveyorHost::reparentProduct(const Product& product, ConveyorId destination) {
    Item item = itemFor(product.id);
    const Segment* segment = segmentOf(destination);
    if (!item || !segment || !segment->parent) return;
    if (m_rdk && (!m_rdk->Valid(item) || !m_rdk->Valid(segment->parent))) return;
    // Static, so the product keeps the pose it transferred at. placeProduct puts it on the new
    // path in the same tick.
    item->setParentStatic(segment->parent);
    m_stationChanged = true;
}

const RoboDkConveyorHost::Segment* RoboDkConveyorHost::originOf(const Product& product) const {
    if (const Segment* spawner = segmentOf(product.originSpawner)) return spawner;
    return segmentOf(product.conveyor);
}

void RoboDkConveyorHost::placeProduct(const Product& product) {
    const Segment* segment = segmentOf(product.conveyor);
    CadTransform stand;
    if (!segment || !placement(*segment, &stand)) return; // the station carries it
    Item item = itemFor(product.id);
    if (!item || (m_rdk && !m_rdk->Valid(item))) return;

    const Segment* origin = originOf(product);
    const CadVec3 across = axisOf(stand, 1);
    const CadVec3 point = conveyorTransformPoint(
        stand * conveyorAccessoryFrame(), conveyorPathPoseAt(segment->parameters, product.progress));
    const double sideways = origin ? dot(origin->workpieceCentreOffsetMm, across) : 0.0;
    const CadVec3 up = axisOf(stand, 2);
    const double lift = conveyorSurfaceOffsetMm(segment->parameters);

    // Preserve the prototype attitude; clone and conveyor poses use different frames.
    Item attitude = origin && origin->prototype ? origin->prototype : item;
    if (m_rdk && !m_rdk->Valid(attitude)) attitude = item;
    CadTransform pose = rdkbridge::toCadTransform(attitude->PoseAbs());
    pose.values[3] = point.x - across.x * sideways + up.x * lift;
    pose.values[7] = point.y - across.y * sideways + up.y * lift;
    pose.values[11] = point.z - across.z * sideways + up.z * lift;
    item->setPoseAbs(rdkbridge::toRoboDkPose(pose));
    m_stationChanged = true;
}

bool RoboDkConveyorHost::consumeStationChanged() {
    const bool changed = m_stationChanged;
    m_stationChanged = false;
    return changed;
}

bool RoboDkConveyorHost::spawnProduct(ConveyorId id, Product* out) {
    const Segment* segment = segmentOf(id);
    if (!segment || !segment->prototype || !m_world) return false;
    QString name;
    QString error;
    const CadTransform pose = rdkbridge::toCadTransform(segment->prototype->PoseAbs());
    Item created = m_world->spawn(segment->prototype, segment->parent, pose, &name, &error);
    if (!created) return false;

    out->id = m_nextProductId++;
    out->conveyor = id;
    out->originSpawner = id;
    out->progress = segment->spawnProgress;
    out->forward = true;
    trackProduct(out->id, created);
    m_stationChanged = true;
    return true;
}

void RoboDkConveyorHost::destroyProduct(const Product& product) {
    Item item = itemFor(product.id);
    forgetProduct(product.id);
    if (item && m_world) m_world->removeItem(item);
    m_stationChanged = true;
}

bool conveyorSegmentFromItem(RoboDK* rdk, Item item, RoboDkConveyorHost::Segment* out,
                             QString* error) {
    const auto fail = [error](const QString& message) {
        if (error) *error = message;
        return false;
    };
    if (!rdk || !item || !out) return fail(QStringLiteral("No item to read a conveyor from."));
    QByteArray blob;
    if (!item->getParam(kConveyorItemParam, blob) || blob.trimmed().isEmpty()) {
        return fail(QStringLiteral("'%1' carries no conveyor parameters.").arg(item->Name()));
    }
    const Json parameters = Json::parse(blob.begin(), blob.end(), nullptr, false);
    if (!parameters.is_object()) {
        return fail(QStringLiteral("'%1' has conveyor parameters that are not a JSON object.")
                        .arg(item->Name()));
    }

    RoboDkConveyorHost::Segment segment;
    segment.item = item;
    // Clones ride under the conveyor's own node, so deleting it takes its load with it and re-parenting
    // it onto a frame carries its load along. A generic node contributes identity to the pose chain, so a
    // product's local pose under one is its absolute pose - which costs nothing, because a product on a
    // conveyor is placed by `placeProduct` every step and never left to be carried by arithmetic.
    segment.parent = item;
    segment.name = item->Name();
    segment.parametersBlob = blob;
    segment.parameters.applyAccessoryParametersJson(parameters);
    if (segment.parameters.accessoryGenerator != "roller_conveyor") {
        return fail(QStringLiteral("'%1' describes a '%2', which is not a conveyor.")
                        .arg(item->Name(),
                             QString::fromStdString(segment.parameters.accessoryGenerator)));
    }
    segment.nextName = QString::fromStdString(jsoncompat::fieldString(parameters, "next"));
    // The placement, and the only statement of it there is: the item's own pose is identity and there is
    // no second item standing in the cell. A conveyor whose JSON has no `pose` therefore stands at its
    // parent's origin until something places it, which is what a conveyor with no placement means.
    segment.hasPose = false;
    if (jsoncompat::contains(parameters, "pose")) {
        const Json& pose = parameters["pose"];
        if (pose.is_array() && pose.size() >= 12) {
            for (int cell = 0; cell < 12; ++cell) {
                segment.poseLocal.values[cell] = pose[static_cast<size_t>(cell)].get<double>();
            }
            segment.hasPose = true;
        }
    }

    if (jsoncompat::contains(parameters, "endFrame")) {
        return fail(QStringLiteral("Conveyor '%1' declares an end frame, which conveyors stopped "
                                   "having: its pose is now the conveyor's own placement. Regenerate "
                                   "the station.").arg(segment.name));
    }
    // Named, never inferred from whatever the products land on. RoboDK identifies objects by name,
    // so that is what the accessory block's own spawn-object id holds here.
    const QString prototype =
        QString::fromStdString(segment.parameters.accessorySpawnObjectId);
    if (!prototype.isEmpty()) {
        segment.prototype = rdk->getItem(prototype);
        if (!segment.prototype) {
            return fail(QStringLiteral("Conveyor '%1' has no prototype '%2'.")
                            .arg(segment.name, prototype));
        }
    }

    deriveSegment(&segment);
    *out = segment;
    if (error) error->clear();
    return true;
}

bool writeConveyorParameters(RoboDK* rdk, RoboDkConveyorHost::Segment& segment) {
    if (!segment.item || (rdk && !rdk->Valid(segment.item))) return false;
    Json parameters = segment.parameters.accessoryParametersJson();
    if (!parameters.is_object()) return false;
    // The one key the accessory block cannot express, because a CadNode station finds what a conveyor
    // feeds geometrically and a RoboDK station's frames are placed by hand - a lane that silently
    // stopped feeding because someone nudged a frame 20 mm is worse than saying what feeds what.
    parameters["next"] = segment.nextName.toStdString();
    // Where the conveyor stands, relative to its item's RoboDK parent - the way a CadNode's transform is
    // relative to its parent. The other key the accessory block cannot express, and for a reason worth
    // stating: in a CadNode station the *node* carries the placement and the accessory block is only the
    // recipe, so putting a pose in that block would be a second answer for RobotSimulator. It stays the
    // plugin's, beside `next`.
    Json pose = Json::array();
    for (int cell = 0; cell < 12; ++cell) pose.push_back(segment.poseLocal.values[cell]);
    parameters["pose"] = pose;
    const std::string text = parameters.dump();
    const QByteArray blob(text.data(), static_cast<int>(text.size()));
    // A setParam is a station edit. Rewriting identical bytes on every declaration would mark a
    // station dirty for having been opened.
    if (blob == segment.parametersBlob) return false;
    segment.item->setParam(kConveyorItemParam, blob);
    segment.parametersBlob = blob;
    return true;
}

bool parseConveyorSegment(RoboDK* rdk, const QStringList& fields,
                          RoboDkConveyorHost::Segment* out, QString* error) {
    const auto fail = [error](const QString& message) {
        if (error) *error = message;
        return false;
    };
    if (!rdk || !out) return fail(QStringLiteral("No station to declare a conveyor in."));

    const QString itemName = fields.value(0).trimmed();
    if (itemName.isEmpty()) return fail(QStringLiteral("A conveyor needs an item."));

    RoboDkConveyorHost::Segment segment;
    segment.item = rdk->getItem(itemName);
    if (!segment.item) {
        return fail(QStringLiteral("There is no item called '%1' to be a conveyor.").arg(itemName));
    }
    segment.parent = segment.item;
    segment.name = segment.item->Name();
    segment.nextName = fields.value(4).trimmed();

    TransformNodeData& parameters = segment.parameters;
    parameters.accessoryGenerator = "roller_conveyor";
    // Logical, because a RoboDK station carries its products: they are placed by item pose and have
    // no body while they are on the belt.
    parameters.accessoryConveyorMode = "logical";
    parameters.accessoryLengthMm = fields.value(1).toDouble();
    parameters.accessoryConveyorSpeedMmS = fields.value(2).toDouble();
    parameters.accessoryConveyorRole = fields.value(3).trimmed().toLower().toStdString();

    const QString prototype = fields.value(5).trimmed();
    if (!prototype.isEmpty()) {
        segment.prototype = rdk->getItem(prototype);
        if (!segment.prototype) {
            return fail(QStringLiteral("Conveyor '%1' has no prototype '%2'.")
                            .arg(segment.name, prototype));
        }
        parameters.accessorySpawnObjectId = prototype.toStdString();
    }
    const double interval = fields.value(6).toDouble();
    parameters.accessorySpawnIntervalSeconds = interval > 0.0 ? interval : 1.0;
    parameters.accessoryMaxActiveSpawns = std::max(0, fields.value(7).toInt());
    // The queue this conveyor accumulates into, and how it is drawn. All four optional, all four
    // ending up in the same parameter block every other route reads.
    segment.stated.endInset =
        applyIfStated(fields.value(8), &parameters.accessoryInitialWorkpieceEndInsetMm);
    segment.stated.pitch =
        applyIfStated(fields.value(9), &parameters.accessoryInitialWorkpieceSpacingMm);
    parameters.accessoryWidthMm = kDefaultDeckWidthMm;
    parameters.accessoryHeightMm = kDefaultDeckHeightMm;
    segment.stated.deckWidth = applyIfStated(fields.value(10), &parameters.accessoryWidthMm);
    segment.stated.deckHeight = applyIfStated(fields.value(11), &parameters.accessoryHeightMm);
    parameters.accessoryStartHeightMm = parameters.accessoryHeightMm;
    parameters.accessoryEndHeightMm = parameters.accessoryHeightMm;
    parameters.accessoryStartLeftHeightMm = parameters.accessoryHeightMm;
    parameters.accessoryStartRightHeightMm = parameters.accessoryHeightMm;
    parameters.accessoryEndLeftHeightMm = parameters.accessoryHeightMm;
    parameters.accessoryEndRightHeightMm = parameters.accessoryHeightMm;

    deriveSegment(&segment);
    *out = segment;
    if (error) error->clear();
    return true;
}
