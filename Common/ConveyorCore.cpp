#include "ConveyorCore.h"

#include "AccessoryGeometry.h"

#include <algorithm>
#include <cmath>

namespace conveyorcore {

ConveyorSpec conveyorSpecFrom(const TransformNodeData& parameters, Mode mode) {
    ConveyorSpec spec;
    spec.role = parameters.accessoryConveyorRole == "spawner" ? Role::Spawner
        : parameters.accessoryConveyorRole == "deleter" ? Role::Deleter
        : parameters.accessoryConveyorRole == "pick_feeder" ? Role::PickFeeder
        : Role::Normal;
    spec.mode = mode;
    spec.speedMmS = parameters.accessoryConveyorSpeedMmS;
    spec.pathLengthMm = conveyorPathLengthMm(parameters);
    spec.spawnIntervalSeconds = parameters.accessorySpawnIntervalSeconds;
    spec.maxActiveSpawns = parameters.accessoryMaxActiveSpawns;
    // The queue an accumulating conveyor forms is the layout it already lays its *initial*
    // workpieces out in. One pair of numbers, so a station cannot describe the two differently.
    spec.endInsetMm = parameters.accessoryInitialWorkpieceEndInsetMm;
    spec.productPitchMm = parameters.accessoryInitialWorkpieceSpacingMm;
    return spec;
}

bool raisePitchToFitWorkpiece(TransformNodeData& parameters, double extentAlongLaneMm) {
    if (!(extentAlongLaneMm > 0.0)) return false;
    if (parameters.accessoryInitialWorkpieceSpacingMm >= extentAlongLaneMm) return false;
    parameters.accessoryInitialWorkpieceSpacingMm = extentAlongLaneMm;
    return true;
}

namespace {

double laneEndDistance(const CadVec3& a, const CadVec3& b) {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

CadVec3 laneEndPoint(const Lane& lane, double t) {
    return conveyorTransformPoint(lane.world, conveyorPathPoseAt(*lane.parameters, t));
}

} // namespace

Handover nextLane(const std::vector<Lane>& lanes, size_t from, bool leavingForward) {
    Handover best;
    if (from >= lanes.size() || !lanes[from].parameters) return best;
    const Lane& current = lanes[from];
    const CadVec3 exit = laneEndPoint(current, leavingForward ? 1.0 : 0.0);
    // Initialised *at* the tolerance and compared strictly, so a lane exactly that far away is not a join.
    // The behaviour RobotSimulator's own traces are goldened against, kept to the letter.
    double nearest = kLaneJoinToleranceMm;
    for (size_t index = 0; index < lanes.size(); ++index) {
        const Lane& candidate = lanes[index];
        if (index == from || !candidate.parameters || candidate.mode != current.mode) continue;
        for (int endpoint = 0; endpoint < 2; ++endpoint) {
            const double distance =
                laneEndDistance(exit, laneEndPoint(candidate, endpoint == 0 ? 0.0 : 1.0));
            if (distance >= nearest) continue;
            nearest = distance;
            best.lane = static_cast<int>(index);
            // Entering at the start runs forward; entering at the end runs in reverse.
            best.forward = endpoint == 0;
        }
    }
    return best;
}

void Runtime::reset() {
    m_products.clear();
    m_spawners.clear();
}

void Runtime::addSpawner(ConveyorId conveyor) {
    m_spawners.push_back(Spawner{conveyor, 0.0});
}

void Runtime::setSpawners(const std::vector<ConveyorId>& conveyors) {
    std::vector<Spawner> kept;
    kept.reserve(conveyors.size());
    for (ConveyorId conveyor : conveyors) {
        const auto existing = std::find_if(m_spawners.begin(), m_spawners.end(),
                                           [conveyor](const Spawner& spawner) {
                                               return spawner.conveyor == conveyor;
                                           });
        kept.push_back(existing == m_spawners.end() ? Spawner{conveyor, 0.0} : *existing);
    }
    m_spawners = std::move(kept);
}

void Runtime::restartSpawners() {
    for (Spawner& spawner : m_spawners) spawner.secondsUntilSpawn = 0.0;
}

void Runtime::addProduct(const Product& product) {
    m_products.push_back(product);
}

void Runtime::setProducts(std::vector<Product> products) {
    m_products = std::move(products);
}

void Runtime::removeProduct(ProductId id) {
    m_products.erase(std::remove_if(m_products.begin(), m_products.end(),
                                    [id](const Product& p) { return p.id == id; }),
                     m_products.end());
}

void Runtime::setGrasped(ProductId id, bool grasped) {
    if (Product* found = product(id)) found->grasped = grasped;
}

Product* Runtime::product(ProductId id) {
    const auto hit = std::find_if(m_products.begin(), m_products.end(),
                                  [id](const Product& p) { return p.id == id; });
    return hit == m_products.end() ? nullptr : &*hit;
}

size_t Runtime::activeSpawnCount(ConveyorId spawner) const {
    // By originSpawner, never by current conveyor. A product that has transferred downstream or
    // been picked up still occupies the flow its spawner is capped on; counting by parent would let
    // a capped spawner produce without limit the moment its first product moved on.
    return static_cast<size_t>(std::count_if(
        m_products.begin(), m_products.end(),
        [spawner](const Product& p) { return p.originSpawner == spawner; }));
}

bool Runtime::spawnerAtCap(ConveyorId spawner) const {
    ConveyorSpec spec;
    if (!m_host || !m_host->conveyorSpec(spawner, &spec)) return false;
    return spec.role == Role::Spawner && spec.maxActiveSpawns > 0 &&
        activeSpawnCount(spawner) >= static_cast<size_t>(spec.maxActiveSpawns);
}

namespace {

double pitchProgress(const ConveyorSpec& spec) {
    return spec.pathLengthMm > 0.0 ? std::max(0.0, spec.productPitchMm) / spec.pathLengthMm : 0.0;
}

double restProgress(const ConveyorSpec& spec) {
    if (spec.pathLengthMm <= 0.0) return 1.0;
    return std::max(0.0, 1.0 - std::max(0.0, spec.endInsetMm) / spec.pathLengthMm);
}

// Who takes part in a queue. A grasped product is the tool's - it is being lifted away, not
// occupying the belt - and a physical one is queued by contact, where two rigid bodies do for free
// what these rules do by arithmetic. Overwriting a solved progress would fight the solver.
bool queues(const Product& product) {
    return !product.grasped && !product.physical;
}

} // namespace

bool Runtime::entryBlocked(ConveyorId conveyor) const {
    ConveyorSpec spec;
    if (!m_host || !m_host->conveyorSpec(conveyor, &spec)) return false;
    const double pitch = pitchProgress(spec);
    const double region = spec.role == Role::Spawner ? kSpawnerRegionProgress : 0.0;
    for (const Product& product : m_products) {
        // Physical products count here even though they do not queue: this is a question about
        // whether the space is occupied, and a body sitting in it occupies it.
        if (product.conveyor != conveyor || product.grasped) continue;
        // The region plus the pitch a new workpiece takes up: for a plain conveyor that is the one
        // pitch at the entry, unchanged, and for a spawner it is everything up to a clear workpiece
        // length past its own enclosure.
        if (product.progress < region + pitch) return true;
    }
    return false;
}

bool Runtime::endStopOccupied(ConveyorId conveyor) const {
    ConveyorSpec spec;
    if (!m_host || !m_host->conveyorSpec(conveyor, &spec)) return false;
    // An end stop is what a *disconnected* end has. A conveyor that feeds another has no part for a
    // product to come to rest against, so nothing is ever waiting to be picked off it - whatever is
    // down there is passing through, and a cell that waited on it would grab at a moving box.
    if (m_host->nextConveyor(conveyor, true).valid) return false;
    const double rest = restProgress(spec) - 1.0e-9;
    for (const Product& product : m_products) {
        if (product.conveyor != conveyor || product.grasped) continue;
        if (product.progress >= rest) return true;
    }
    return false;
}

size_t Runtime::productsOn(ConveyorId conveyor) const {
    return static_cast<size_t>(std::count_if(
        m_products.begin(), m_products.end(), [conveyor](const Product& p) {
            return p.conveyor == conveyor && !p.grasped;
        }));
}

void Runtime::resolveAccumulation() {
    // Rule 16, the leader stops at the end inset, and rule 17, nobody passes the product ahead.
    std::vector<ConveyorId> seen;
    std::vector<size_t> queue;
    for (size_t index = 0; index < m_products.size(); ++index) {
        const ConveyorId conveyor = m_products[index].conveyor;
        if (std::find(seen.begin(), seen.end(), conveyor) != seen.end()) continue;
        seen.push_back(conveyor);

        ConveyorSpec spec;
        if (!m_host->conveyorSpec(conveyor, &spec)) continue;
        const double pitch = pitchProgress(spec);

        for (int pass = 0; pass < 2; ++pass) {
            const bool forward = pass == 0;
            queue.clear();
            for (size_t other = index; other < m_products.size(); ++other) {
                const Product& product = m_products[other];
                if (product.conveyor != conveyor || product.forward != forward) continue;
                if (queues(product)) queue.push_back(other);
            }
            if (queue.empty()) continue;
            // Front first: furthest along for a product running forward, least far for one running
            // back towards the start.
            std::sort(queue.begin(), queue.end(), [&](size_t a, size_t b) {
                return forward ? m_products[a].progress > m_products[b].progress
                               : m_products[a].progress < m_products[b].progress;
            });

            // Rule 16 applies at the end alone, because that is where the end stop is; a product
            // running backwards off a start has no part to rest against and still stops at 0, which
            // is rule 12 unchanged. The transfer is only asked about when the leader is inside the
            // inset - a conveyor that feeds another does not hold its products back, and asking
            // every tick would cost a geometric search per conveyor for an answer that almost never
            // matters.
            double limit = forward ? 1.0 : 0.0;
            if (forward) {
                const double rest = restProgress(spec);
                if (m_products[queue.front()].progress > rest &&
                    !m_host->nextConveyor(conveyor, true).valid) {
                    limit = rest;
                }
            }
            for (const size_t position : queue) {
                Product& product = m_products[position];
                if (forward ? product.progress > limit : product.progress < limit) {
                    product.progress = limit;
                }
                limit = forward ? product.progress - pitch : product.progress + pitch;
            }
        }
    }
}

void Runtime::step(double seconds) {
    if (!m_host) return;

    for (Spawner& spawner : m_spawners) {
        ConveyorSpec spec;
        if (!m_host->conveyorSpec(spawner.conveyor, &spec)) continue;
        spawner.secondsUntilSpawn -= seconds;
        if (spawner.secondsUntilSpawn > 0.0) continue;
        // Rule 18: a full conveyor refuses the spawn. Two different limits, and a line needs both -
        // the cap is a budget across the whole cell that follows a product wherever it goes, and
        // fullness is a length that only asks whether this conveyor has room for one more.
        if (!spawnerAtCap(spawner.conveyor) && !entryBlocked(spawner.conveyor)) {
            Product spawned;
            if (m_host->spawnProduct(spawner.conveyor, &spawned)) m_products.push_back(spawned);
        }
        // Added, never assigned, and outside both tests: fractional tick time is preserved, so the
        // cadence does not drift when the operator changes simulation speed, and a refusal does not
        // also shift every later spawn or queue itself up to fire as a burst once there is room.
        spawner.secondsUntilSpawn += std::max(kMinimumSpawnIntervalSeconds, spec.spawnIntervalSeconds);
    }

    bool sceneGraphChanged = false;
    for (size_t index = 0; index < m_products.size();) {
        Product& product = m_products[index];
        // A grasped product is the tool's. Not advanced, not transferred, not deleted.
        if (product.grasped) {
            ++index;
            continue;
        }
        const auto retire = [&]() {
            m_host->destroyProduct(product);
            m_products.erase(m_products.begin() + static_cast<ptrdiff_t>(index));
            sceneGraphChanged = true;
        };

        ConveyorSpec spec;
        if (!m_host->conveyorSpec(product.conveyor, &spec)) {
            // Its conveyor is gone. Nothing carries it and nothing will delete it later.
            retire();
            continue;
        }

        CadTransform bodyPose;
        CadVec3 center;
        bool havePhysicalPose = false;
        bool leftPath = false;
        bool enteredDeleter = false;
        bool leavingForward = product.forward;

        product.physical = spec.mode == Mode::Physx && m_host->productHasBody(product);
        if (product.physical && m_host->productBodyPose(product, &bodyPose)) {
            havePhysicalPose = true;
            center = CadVec3(bodyPose.values[3], bodyPose.values[7], bodyPose.values[11]);
            // Derived, not integrated. Adding a step as well would double-count everything contact
            // and gravity already did to the body.
            product.progress = m_host->closestProgress(product.conveyor, center);
            enteredDeleter = m_host->inDeleterVolume(product, center);
            leftPath = m_host->reachedEnd(product, center,
                                          std::max(0.0, spec.speedMmS) * seconds);
        } else {
            // A path of no length cannot be advanced along, and dividing by it would put this product
            // at inf or nan for the rest of the run - and, through the transfer rules, whatever it
            // touches. No shipped station declares one; a host that measures its length from geometry
            // can produce one from two frames left on top of each other.
            const double delta = spec.pathLengthMm > 0.0
                ? std::max(0.0, spec.speedMmS) * seconds / spec.pathLengthMm *
                      (product.forward ? 1.0 : -1.0)
                : 0.0;
            product.progress += delta;
            enteredDeleter = spec.role == Role::Deleter &&
                product.progress >= 0.5 && product.progress <= 1.0;
            leftPath = product.progress > 1.0 || product.progress < 0.0;
            leavingForward = product.progress > 1.0;
        }

        if (enteredDeleter) {
            retire();
            continue;
        }
        // After the deleter test, because that test sweeps from the previous centre to this one.
        if (havePhysicalPose) m_host->notePhysicalCenter(product, center);

        if (leftPath) {
            if (spec.role == Role::Deleter) {
                retire();
                continue;
            }
            const TransferTarget target = m_host->nextConveyor(product.conveyor, leavingForward);
            if (target.valid) {
                m_host->reparentProduct(product, target.conveyor);
                product.conveyor = target.conveyor;
                product.forward = target.forward;
                // Entering at the start runs forward from 0; entering at the end runs back from 1.
                product.progress = target.forward ? 0.0 : 1.0;
                sceneGraphChanged = true;
            } else {
                // A disconnected endpoint is a stop, not a chute into nowhere.
                product.progress = leavingForward ? 1.0 : 0.0;
            }
        }

        // Re-read: a transfer may have put the product on a conveyor in the other mode, and it is
        // the destination's mode that decides who places it from here on.
        ConveyorSpec destination;
        product.physical = m_host->conveyorSpec(product.conveyor, &destination) &&
            m_host->productHasBody(product) && destination.mode == Mode::Physx;
        ++index;
    }

    resolveAccumulation();

    for (const Product& product : m_products) {
        if (!product.grasped && !product.physical) m_host->placeProduct(product);
    }

    if (sceneGraphChanged) m_host->sceneChanged();
}

} // namespace conveyorcore
