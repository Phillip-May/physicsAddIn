// The conveyor transport rules, against a host made of arithmetic.

#include "ConveyorCore.cpp"
#include "AccessoryGeometry.cpp"

#include <cmath>
#include <cstdio>
#include <functional>
#include <map>
#include <set>
#include <string>
#include <vector>

using namespace conveyorcore;

namespace {

int g_failures = 0;

void check(bool condition, const std::string& what) {
    if (condition) return;
    ++g_failures;
    std::printf("FAIL: %s\n", what.c_str());
}

void checkClose(double actual, double expected, const std::string& what) {
    check(std::abs(actual - expected) < 1.0e-9,
          what + " (expected " + std::to_string(expected) + ", got " + std::to_string(actual) + ")");
}

struct LineHost : Host {
    struct Line {
        ConveyorSpec spec;
        double startMm = 0.0;
    };

    std::map<ConveyorId, Line> lines;
    std::map<ConveyorId, ConveyorId> downstream;
    std::map<ProductId, CadVec3> bodies;      // only products the test gave a body
    std::map<ProductId, CadVec3> lastCenters;
    std::set<ConveyorId> refuseSpawn;

    ProductId nextProductId = 1;
    std::vector<ProductId> destroyed;
    std::vector<std::pair<ProductId, ConveyorId>> reparented;
    std::vector<std::pair<ProductId, double>> placed;
    int sceneChanges = 0;

    void addLine(ConveyorId id, Role role, Mode mode, double speed, double length,
                 double interval = 1.0, int cap = 0) {
        Line line;
        line.spec.role = role;
        line.spec.mode = mode;
        line.spec.speedMmS = speed;
        line.spec.pathLengthMm = length;
        line.spec.spawnIntervalSeconds = interval;
        line.spec.maxActiveSpawns = cap;
        line.startMm = static_cast<double>(id - 1) * length;
        lines[id] = line;
    }

    bool conveyorSpec(ConveyorId id, ConveyorSpec* out) const override {
        const auto hit = lines.find(id);
        if (hit == lines.end()) return false;
        *out = hit->second.spec;
        return true;
    }
    bool productHasBody(const Product& product) const override {
        return bodies.count(product.id) != 0;
    }
    bool productBodyPose(const Product& product, CadTransform* world) const override {
        const auto hit = bodies.find(product.id);
        if (hit == bodies.end()) return false;
        *world = CadTransform();
        world->values[3] = hit->second.x;
        world->values[7] = hit->second.y;
        world->values[11] = hit->second.z;
        return true;
    }
    double closestProgress(ConveyorId id, const CadVec3& world) const override {
        const Line& line = lines.at(id);
        const double along = (world.x - line.startMm) / line.spec.pathLengthMm;
        return std::max(0.0, std::min(1.0, along));
    }
    bool inDeleterVolume(const Product& product, const CadVec3& world) const override {
        const Line& line = lines.at(product.conveyor);
        if (line.spec.role != Role::Deleter) return false;
        const double along = (world.x - line.startMm) / line.spec.pathLengthMm;
        return along >= 0.5 && along <= 1.0;
    }
    bool reachedEnd(const Product& product, const CadVec3& world,
                    double allowanceMm) const override {
        const Line& line = lines.at(product.conveyor);
        const double endMm = line.startMm + (product.forward ? line.spec.pathLengthMm : 0.0);
        const double past = product.forward ? world.x - endMm : endMm - world.x;
        return past >= -allowanceMm;
    }
    void notePhysicalCenter(const Product& product, const CadVec3& world) override {
        lastCenters[product.id] = world;
    }
    TransferTarget nextConveyor(ConveyorId id, bool leavingForward) const override {
        if (!leavingForward) return TransferTarget();
        const auto hit = downstream.find(id);
        if (hit == downstream.end()) return TransferTarget();
        TransferTarget target;
        target.conveyor = hit->second;
        target.forward = true;
        target.valid = true;
        return target;
    }
    void reparentProduct(const Product& product, ConveyorId destination) override {
        reparented.emplace_back(product.id, destination);
    }
    void placeProduct(const Product& product) override {
        placed.emplace_back(product.id, product.progress);
    }
    bool spawnProduct(ConveyorId conveyor, Product* out) override {
        if (refuseSpawn.count(conveyor)) return false;
        out->id = nextProductId++;
        out->conveyor = conveyor;
        out->originSpawner = conveyor;
        out->progress = 0.0;
        out->forward = true;
        return true;
    }
    void destroyProduct(const Product& product) override {
        destroyed.push_back(product.id);
    }
    void sceneChanged() override { ++sceneChanges; }
};

constexpr double kTick = 1.0 / 60.0;

void addFreeRunningSpawner(LineHost& host, double interval, double speed) {
    host.addLine(1, Role::Spawner, Mode::Logical, speed, 200.0, interval);
    host.addLine(2, Role::Deleter, Mode::Logical, speed, 200.0);
    host.downstream[1] = 2;
}

size_t everProduced(const LineHost& host) { return host.nextProductId - 1; }

// Rule 1: the interval is added, never assigned, so fractional tick time survives.
void cadenceDoesNotDrift() {
    LineHost host;
    addFreeRunningSpawner(host, 0.26, 4000.0);
    Runtime runtime;
    runtime.setHost(&host);
    runtime.addSpawner(1);
    for (int tick = 0; tick < 600; ++tick) runtime.step(kTick);

    // 10 s at 0.26 s: one at t=0 and one per interval after it.
    const size_t expected = 1 + static_cast<size_t>(10.0 / 0.26);
    check(everProduced(host) == expected,
          "cadence produced " + std::to_string(everProduced(host)) + ", expected " +
              std::to_string(expected));
}

// Rule 1 again: the floor stops a zero interval becoming a spawn per tick.
void cadenceHasAFloor() {
    LineHost host;
    addFreeRunningSpawner(host, 0.0, 8000.0);
    Runtime runtime;
    runtime.setHost(&host);
    runtime.addSpawner(1);
    for (int tick = 0; tick < 62; ++tick) runtime.step(kTick);
    check(everProduced(host) == 21,
          "a zero interval should floor at 0.05 s and yield 21 in 1.03 s, not one per tick; "
          "produced " + std::to_string(everProduced(host)));
}

// Rule 2: the cap counts by originating spawner, not by current parent.
void capCountsByOrigin() {
    LineHost host;
    host.addLine(1, Role::Spawner, Mode::Logical, 1000.0, 1000.0, 0.25, 2);
    host.addLine(2, Role::Normal, Mode::Logical, 1000.0, 1000.0);
    host.downstream[1] = 2;
    Runtime runtime;
    runtime.setHost(&host);
    runtime.addSpawner(1);
    for (int tick = 0; tick < 600; ++tick) runtime.step(kTick);

    check(runtime.products().size() == 2,
          "a cap of two produced " + std::to_string(runtime.products().size()));
    // Both have long since left conveyor 1. Counting by parent would have let it spawn forever.
    for (const Product& product : runtime.products()) {
        check(product.conveyor == 2, "capped products should have transferred downstream");
        check(product.originSpawner == 1, "originSpawner must survive the transfer");
    }
}

void capIsReportedWhileItRefuses() {
    LineHost host;
    host.addLine(1, Role::Spawner, Mode::Logical, 1000.0, 1000.0, 0.25, 2);
    Runtime runtime;
    runtime.setHost(&host);
    runtime.addSpawner(1);

    check(!runtime.spawnerAtCap(1), "an empty spawner is not at cap");
    runtime.step(kTick);
    check(runtime.products().size() == 1 && !runtime.spawnerAtCap(1),
          "one product against a cap of two leaves room");
    for (int tick = 0; tick < 600; ++tick) runtime.step(kTick);

    check(runtime.products().size() == 2, "the cap held at two");
    check(runtime.spawnerAtCap(1), "a spawner that has stopped producing must say it is at cap");
    runtime.removeProduct(runtime.products().front().id);
    check(!runtime.spawnerAtCap(1), "removing a product frees the cap and the report follows");

    // Zero is unlimited, and unlimited is never at cap however many products it has made.
    LineHost uncapped;
    uncapped.addLine(1, Role::Spawner, Mode::Logical, 1000.0, 1000.0, 0.25, 0);
    Runtime unlimited;
    unlimited.setHost(&uncapped);
    unlimited.addSpawner(1);
    for (int tick = 0; tick < 600; ++tick) unlimited.step(kTick);
    check(unlimited.products().size() > 2, "an uncapped spawner keeps producing");
    check(!unlimited.spawnerAtCap(1), "zero means unlimited, which is never at cap");
    LineHost plain;
    plain.addLine(1, Role::Normal, Mode::Logical, 1000.0, 1000.0, 0.25, 1);
    Runtime normal;
    normal.setHost(&plain);
    check(!normal.spawnerAtCap(1), "a normal conveyor is never at cap");
    check(!normal.spawnerAtCap(7), "and neither is an id that names no conveyor");
}

void zeroLengthPathDoesNotPoisonProgress() {
    LineHost host;
    host.addLine(1, Role::Normal, Mode::Logical, 1000.0, 0.0);
    Runtime runtime;
    runtime.setHost(&host);
    Product product;
    product.id = 1;
    product.conveyor = 1;
    product.progress = 0.25;
    runtime.addProduct(product);
    for (int tick = 0; tick < 10; ++tick) runtime.step(kTick);

    check(runtime.products().size() == 1, "a product on a zero-length path is not destroyed");
    const double progress = runtime.products().front().progress;
    check(progress == progress, "its progress must not be nan");
    checkClose(progress, 0.25, "and it cannot advance along a path with no length");
    check(host.destroyed.empty(), "nothing was deleted");
    check(host.reparented.empty(), "and nothing transferred off a path it never reached the end of");
}

// Rule 3: a grasped product is skipped entirely - not advanced, not deleted, not placed.
void graspedProductIsUntouched() {
    LineHost host;
    host.addLine(1, Role::Deleter, Mode::Logical, 1000.0, 1000.0);
    Runtime runtime;
    runtime.setHost(&host);
    Product product;
    product.id = 7;
    product.conveyor = 1;
    product.progress = 0.75; // inside the deleter's half of the deck
    product.grasped = true;
    runtime.addProduct(product);
    for (int tick = 0; tick < 60; ++tick) runtime.step(kTick);

    check(runtime.products().size() == 1, "a grasped product must not be deleted");
    checkClose(runtime.products().front().progress, 0.75, "a grasped product must not advance");
    check(host.placed.empty(), "a grasped product must not be placed");
}

// Rule 4: a product whose conveyor no longer resolves is destroyed.
void orphanedProductIsDestroyed() {
    LineHost host;
    host.addLine(1, Role::Normal, Mode::Logical, 100.0, 1000.0);
    Runtime runtime;
    runtime.setHost(&host);
    Product product;
    product.id = 3;
    product.conveyor = 9; // never added
    runtime.addProduct(product);
    runtime.step(kTick);

    check(runtime.products().empty(), "a product on a missing conveyor must be destroyed");
    check(host.destroyed.size() == 1 && host.destroyed.front() == 3, "and reported to the host");
    check(host.sceneChanges == 1, "destroying it changes the graph, once");
}

// Rule 5 and 13: logical advance is a signed step, and a logical product is placed.
void logicalAdvanceIsSigned() {
    LineHost host;
    host.addLine(1, Role::Normal, Mode::Logical, 600.0, 1200.0);
    Runtime runtime;
    runtime.setHost(&host);
    Product forward;
    forward.id = 1;
    forward.conveyor = 1;
    forward.progress = 0.5;
    Product backward = forward;
    backward.id = 2;
    backward.forward = false;
    runtime.addProduct(forward);
    runtime.addProduct(backward);
    runtime.step(kTick);

    const double step = 600.0 * kTick / 1200.0;
    checkClose(runtime.products()[0].progress, 0.5 + step, "forward advance");
    checkClose(runtime.products()[1].progress, 0.5 - step, "reverse advance");
    check(host.placed.size() == 2, "logical products are placed by the host");
}

// Rule 6: a physical product's progress is derived from the body, not integrated.
void physicalProgressIsDerived() {
    LineHost host;
    host.addLine(1, Role::Normal, Mode::Physx, 600.0, 1000.0);
    Runtime runtime;
    runtime.setHost(&host);
    Product product;
    product.id = 1;
    product.conveyor = 1;
    product.progress = 0.0;
    runtime.addProduct(product);
    host.bodies[1] = CadVec3(300.0, 0.0, 0.0);
    runtime.step(kTick);

    // 0.3 exactly. Integrating as well would have added another 0.01.
    checkClose(runtime.products().front().progress, 0.3, "physical progress comes from the body");
    check(host.placed.empty(), "a physical product is not placed by the host");
    check(host.lastCenters.count(1) == 1, "its centre is remembered for the next sweep");
}

// Rules 7 and 8: the deleter, logically by progress and physically by volume.
void deleterRetiresBothWays() {
    LineHost logicalHost;
    logicalHost.addLine(1, Role::Deleter, Mode::Logical, 600.0, 1000.0);
    Runtime logical;
    logical.setHost(&logicalHost);
    Product product;
    product.id = 1;
    product.conveyor = 1;
    product.progress = 0.49;
    logical.addProduct(product);
    logical.step(kTick);
    check(logical.products().empty(), "a logical product crossing 0.5 on a deleter is retired");

    LineHost physicalHost;
    physicalHost.addLine(1, Role::Deleter, Mode::Physx, 600.0, 1000.0);
    Runtime physical;
    physical.setHost(&physicalHost);
    physical.addProduct(product);
    physicalHost.bodies[1] = CadVec3(400.0, 0.0, 0.0); // short of the volume
    physical.step(kTick);
    check(physical.products().size() == 1, "a body short of the deleter volume survives");
    physicalHost.bodies[1] = CadVec3(600.0, 0.0, 0.0);
    physical.step(kTick);
    check(physical.products().empty(), "a body inside the deleter volume is retired");
}

// Rules 9, 10 and 11: leaving the path either way, and the transfer that follows.
void transferResetsProgressByDirection() {
    LineHost host;
    host.addLine(1, Role::Normal, Mode::Logical, 60000.0, 1000.0);
    host.addLine(2, Role::Normal, Mode::Logical, 600.0, 1000.0);
    host.downstream[1] = 2;
    Runtime runtime;
    runtime.setHost(&host);
    Product product;
    product.id = 1;
    product.conveyor = 1;
    product.progress = 0.99;
    runtime.addProduct(product);
    runtime.step(kTick);

    check(runtime.products().front().conveyor == 2, "it transferred");
    check(runtime.products().front().forward, "entering at the start runs forward");
    checkClose(runtime.products().front().progress, 0.0, "and starts from zero");
    check(host.reparented.size() == 1, "the host was asked to re-parent it");
    check(host.sceneChanges == 1, "a transfer changes the graph, once");
}

void disconnectedEndpointStops() {
    LineHost host;
    host.addLine(1, Role::Normal, Mode::Logical, 60000.0, 1000.0);
    // No downstream entry at all.
    Runtime runtime;
    runtime.setHost(&host);
    Product product;
    product.id = 1;
    product.conveyor = 1;
    product.progress = 0.99;
    runtime.addProduct(product);
    for (int tick = 0; tick < 10; ++tick) runtime.step(kTick);

    check(runtime.products().size() == 1, "a disconnected end is a stop, not a deletion");
    checkClose(runtime.products().front().progress, 0.93, "rested at the 70 mm end inset");
    check(host.destroyed.empty(), "and nothing was destroyed");
    check(host.sceneChanges == 0, "stopping is not a graph change");
    check(host.placed.back().second == runtime.products().front().progress,
          "and it was placed where it rests, not where it advanced to before being held back");

    LineHost reverseHost;
    reverseHost.addLine(1, Role::Normal, Mode::Logical, 60000.0, 1000.0);
    Runtime reverse;
    reverse.setHost(&reverseHost);
    Product backward;
    backward.id = 2;
    backward.conveyor = 1;
    backward.progress = 0.01;
    backward.forward = false;
    reverse.addProduct(backward);
    reverse.step(kTick);
    check(reverse.products().size() == 1, "a reverse disconnected end is also a stop");
    checkClose(reverse.products().front().progress, 0.0, "clamped to zero");
}

void modeOfTheDestinationDecidesPlacement() {
    LineHost host;
    host.addLine(1, Role::Normal, Mode::Physx, 600.0, 1000.0);
    host.addLine(2, Role::Normal, Mode::Logical, 600.0, 1000.0);
    host.downstream[1] = 2;
    Runtime runtime;
    runtime.setHost(&host);
    Product product;
    product.id = 1;
    product.conveyor = 1;
    product.progress = 0.95;
    runtime.addProduct(product);
    host.bodies[1] = CadVec3(1005.0, 0.0, 0.0); // past the end of conveyor 1
    runtime.step(kTick);

    check(runtime.products().front().conveyor == 2, "it transferred onto the logical conveyor");
    check(!runtime.products().front().physical, "and stopped being physical");
    check(host.placed.size() == 1, "so the host places it, body or no body");
}

// Rule 15: raised once per tick, and only when the graph actually changed.
void sceneDirtyIsRaisedOncePerTick() {
    LineHost host;
    host.addLine(1, Role::Deleter, Mode::Logical, 600.0, 1000.0);
    Runtime runtime;
    runtime.setHost(&host);
    for (ProductId id = 1; id <= 3; ++id) {
        Product product;
        product.id = id;
        product.conveyor = 1;
        product.progress = 0.49;
        runtime.addProduct(product);
    }
    runtime.step(kTick);
    check(host.sceneChanges == 1, "three deletions in one tick raise it once, got " +
                                      std::to_string(host.sceneChanges));
    runtime.step(kTick);
    check(host.sceneChanges == 1, "a tick that changes nothing does not raise it");
}

void refusedSpawnAddsNothing() {
    LineHost host;
    addFreeRunningSpawner(host, 0.22, 4000.0);
    host.refuseSpawn.insert(1);
    Runtime runtime;
    runtime.setHost(&host);
    runtime.addSpawner(1);
    for (int tick = 0; tick < 120; ++tick) runtime.step(kTick);
    check(everProduced(host) == 0, "a refused spawn creates no product");

    host.refuseSpawn.clear();
    for (int tick = 0; tick < 32; ++tick) runtime.step(kTick);
    check(everProduced(host) == 2,
          "the cadence should not queue up refusals; produced " +
              std::to_string(everProduced(host)));
}

// Rules 16 and 17: queue at the end inset and product pitch.
void queueFormsAtPitch() {
    LineHost host;
    host.addLine(1, Role::Normal, Mode::Logical, 1000.0, 1000.0);
    Runtime runtime;
    runtime.setHost(&host);
    // floor((1000 - 70)/90) + 1: slots at 930, 840 ... 30 mm.
    const size_t capacity = static_cast<size_t>((1000.0 - 70.0) / 90.0) + 1;
    check(capacity == 11, "capacity arithmetic");
    for (ProductId id = 1; id <= capacity; ++id) {
        Product product;
        product.id = id;
        product.conveyor = 1;
        runtime.addProduct(product);
    }
    for (int tick = 0; tick < 1800; ++tick) runtime.step(kTick);

    check(runtime.products().size() == capacity,
          "nothing was retired filling the line; it holds " +
              std::to_string(runtime.products().size()));

    std::vector<double> progresses;
    for (const Product& product : runtime.products()) progresses.push_back(product.progress);
    std::sort(progresses.begin(), progresses.end(), std::greater<double>());
    checkClose(progresses.front(), 0.93, "the leader rests at the end inset");
    for (size_t index = 1; index < progresses.size(); ++index) {
        checkClose(progresses[index - 1] - progresses[index], 0.09,
                   "product " + std::to_string(index) + " sits one pitch behind the one ahead");
    }
    check(progresses.back() >= 0.0,
          "the last of floor((length - endInset)/pitch) + 1 products still fits on the line, at " +
              std::to_string(progresses.back()));
    check(progresses.back() - 0.09 < 0.0, "and one more would not");
}

void aSpawnerRefusesWhileItsOwnRegionIsOccupied() {
    LineHost host;
    host.addLine(1, Role::Spawner, Mode::Logical, 700.0, 1400.0, 1.0);
    host.lines[1].spec.endInsetMm = 0.0;
    host.lines[1].spec.productPitchMm = 610.0;
    Runtime runtime;
    runtime.setHost(&host);
    runtime.addSpawner(1);
    for (int tick = 0; tick < 3600; ++tick) runtime.step(kTick);

    check(runtime.products().size() == 2,
          "two workpieces cover this spawner's enclosure, so it holds two; it holds " +
              std::to_string(runtime.products().size()));
    std::vector<double> progresses;
    for (const Product& product : runtime.products()) progresses.push_back(product.progress);
    std::sort(progresses.begin(), progresses.end(), std::greater<double>());
    checkClose(progresses.front(), 1.0, "the leader rests at the end, which has no inset");
    checkClose(progresses[1], 1.0 - 610.0 / 1400.0, "the second sits one workpiece behind it");
    check(runtime.entryBlocked(1), "and the spawner says it has no room");
    check(!runtime.spawnerAtCap(1), "which is fullness, not a cap - there is no cap here");

    LineHost plainHost;
    plainHost.addLine(1, Role::Normal, Mode::Logical, 700.0, 1400.0);
    plainHost.lines[1].spec.endInsetMm = 0.0;
    plainHost.lines[1].spec.productPitchMm = 610.0;
    Runtime plain;
    plain.setHost(&plainHost);
    for (ProductId id = 1; id <= 3; ++id) {
        Product product;
        product.id = id;
        product.conveyor = 1;
        plain.addProduct(product);
    }
    for (int tick = 0; tick < 600; ++tick) plain.step(kTick);
    check(plain.products().size() == 3, "a plain conveyor holds what is put on it");
    check(plain.entryBlocked(1), "and reports its entry taken by the third");
}

void fullLineRefusesWithoutDriftingTheCadence() {
    LineHost host;
    host.addLine(1, Role::Spawner, Mode::Logical, 1000.0, 1400.0, 0.25);
    host.lines[1].spec.endInsetMm = 0.0;
    host.lines[1].spec.productPitchMm = 610.0;
    Runtime runtime;
    runtime.setHost(&host);
    runtime.addSpawner(1);
    for (int tick = 0; tick < 1800; ++tick) runtime.step(kTick);
    const size_t full = runtime.products().size();
    check(full == 2, "the line filled to " + std::to_string(full));

    double leadProgress = 0.0;
    ProductId leader = 0;
    for (const Product& product : runtime.products()) {
        if (product.progress > leadProgress) {
            leadProgress = product.progress;
            leader = product.id;
        }
    }
    runtime.removeProduct(leader);
    check(runtime.entryBlocked(1),
          "retiring the workpiece furthest from the spawner does not free its region");
    for (int tick = 0; tick < 300; ++tick) runtime.step(kTick);
    check(runtime.products().size() == full,
          "one place freed should produce one workpiece, not a burst; the line holds " +
              std::to_string(runtime.products().size()) + " of " + std::to_string(full));
}

// Retiring the leader lets the queue advance one complete slot.
void retiringTheLeaderAdvancesTheQueue() {
    LineHost host;
    host.addLine(1, Role::Normal, Mode::Logical, 1000.0, 1000.0);
    Runtime runtime;
    runtime.setHost(&host);
    for (ProductId id = 1; id <= 3; ++id) {
        Product product;
        product.id = id;
        product.conveyor = 1;
        product.progress = 0.5;
        runtime.addProduct(product);
    }
    for (int tick = 0; tick < 120; ++tick) runtime.step(kTick);
    checkClose(runtime.products()[0].progress, 0.93, "the first-created product led");
    checkClose(runtime.products()[1].progress, 0.84, "the second sits one pitch back");
    checkClose(runtime.products()[2].progress, 0.75, "and the third two");

    runtime.removeProduct(1);
    for (int tick = 0; tick < 120; ++tick) runtime.step(kTick);
    checkClose(runtime.products()[0].progress, 0.93, "the queue advanced one whole slot");
    checkClose(runtime.products()[1].progress, 0.84, "all of it, not just the new leader");
}

void aFedConveyorDoesNotAccumulateAtItsEnd() {
    LineHost host;
    host.addLine(1, Role::Normal, Mode::Logical, 1000.0, 1000.0);
    host.addLine(2, Role::Normal, Mode::Logical, 1000.0, 1000.0);
    host.downstream[1] = 2;
    Runtime runtime;
    runtime.setHost(&host);
    Product product;
    product.id = 1;
    product.conveyor = 1;
    product.progress = 0.95;
    runtime.addProduct(product);
    for (int tick = 0; tick < 10; ++tick) runtime.step(kTick);
    check(runtime.products().front().conveyor == 2,
          "it transferred rather than resting 70 mm short of an interface it was meant to cross");
}

void physicalProductsAreNotHeldBackByTheQueue() {
    LineHost host;
    host.addLine(1, Role::Normal, Mode::Physx, 600.0, 1000.0);
    Runtime runtime;
    runtime.setHost(&host);
    for (ProductId id = 1; id <= 2; ++id) {
        Product product;
        product.id = id;
        product.conveyor = 1;
        runtime.addProduct(product);
    }
    // Both bodies at the same point, which contact would never allow and the rules must not correct.
    host.bodies[1] = CadVec3(500.0, 0.0, 0.0);
    host.bodies[2] = CadVec3(500.0, 0.0, 0.0);
    runtime.step(kTick);
    checkClose(runtime.products()[0].progress, 0.5, "the first keeps the progress its body has");
    checkClose(runtime.products()[1].progress, 0.5, "and so does the second");
    check(host.placed.empty(), "and neither is placed by the host");
}

void endStopAndEntryAreTheRulesOwnTests() {
    LineHost host;
    host.addLine(1, Role::Normal, Mode::Logical, 1000.0, 1000.0);
    Runtime runtime;
    runtime.setHost(&host);

    check(!runtime.endStopOccupied(1), "an empty conveyor has nothing at its end stop");
    check(!runtime.entryBlocked(1), "and room at its entry");
    check(runtime.productsOn(1) == 0, "and no products");
    check(!runtime.endStopOccupied(7), "an id that names no conveyor is not ready");

    for (ProductId id = 1; id <= 11; ++id) {
        Product product;
        product.id = id;
        product.conveyor = 1;
        runtime.addProduct(product);
    }
    // Part way along: on the conveyor, but not yet resting at the stop.
    for (int tick = 0; tick < 20; ++tick) runtime.step(kTick);
    check(runtime.productsOn(1) == 11, "the conveyor is carrying them");
    check(!runtime.endStopOccupied(1), "a product still travelling is not ready to pick");

    for (int tick = 0; tick < 1800; ++tick) runtime.step(kTick);
    check(runtime.endStopOccupied(1), "a product resting at the end stop is ready to pick");
    check(runtime.entryBlocked(1), "and a full line says so");
    check(runtime.productsOn(1) == 11, "count is what the conveyor holds, " +
                                           std::to_string(runtime.productsOn(1)));

    // ready clears as soon as a tool grasps the product.
    ProductId leader = 0;
    double best = -1.0;
    for (const Product& product : runtime.products()) {
        if (product.progress > best) {
            best = product.progress;
            leader = product.id;
        }
    }
    runtime.setGrasped(leader, true);
    check(!runtime.endStopOccupied(1), "ready drops the instant the leader is grasped");
    check(runtime.productsOn(1) == 10, "and the count drops with it - it is the tool's now");
    runtime.setGrasped(leader, false);
    check(runtime.endStopOccupied(1), "and comes back if the tool puts it down again");
}

void aFedConveyorIsNeverReady() {
    LineHost host;
    host.addLine(1, Role::Normal, Mode::Logical, 1000.0, 1000.0);
    host.addLine(2, Role::Normal, Mode::Logical, 1000.0, 1000.0);
    host.downstream[1] = 2;
    Runtime runtime;
    runtime.setHost(&host);
    Product product;
    product.id = 1;
    product.conveyor = 1;
    product.progress = 0.999;
    runtime.addProduct(product);
    check(!runtime.endStopOccupied(1), "a conveyor whose end feeds another is never ready");
    for (int tick = 0; tick < 120; ++tick) runtime.step(kTick);
    check(runtime.products().front().conveyor == 2, "it transferred");
    check(runtime.endStopOccupied(2), "and is ready at the end of the line, which is a dead end");
}

} // namespace

int main() {
    cadenceDoesNotDrift();
    cadenceHasAFloor();
    capCountsByOrigin();
    capIsReportedWhileItRefuses();
    zeroLengthPathDoesNotPoisonProgress();
    graspedProductIsUntouched();
    orphanedProductIsDestroyed();
    logicalAdvanceIsSigned();
    physicalProgressIsDerived();
    deleterRetiresBothWays();
    transferResetsProgressByDirection();
    disconnectedEndpointStops();
    modeOfTheDestinationDecidesPlacement();
    sceneDirtyIsRaisedOncePerTick();
    refusedSpawnAddsNothing();
    queueFormsAtPitch();
    aSpawnerRefusesWhileItsOwnRegionIsOccupied();
    fullLineRefusesWithoutDriftingTheCadence();
    retiringTheLeaderAdvancesTheQueue();
    aFedConveyorDoesNotAccumulateAtItsEnd();
    physicalProductsAreNotHeldBackByTheQueue();
    endStopAndEntryAreTheRulesOwnTests();
    aFedConveyorIsNeverReady();

    if (g_failures > 0) {
        std::printf("conveyor core smoke FAILED with %d problem(s)\n", g_failures);
        return 1;
    }
    std::printf("conveyor core smoke passed\n");
    return 0;
}
