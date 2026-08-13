#pragma once

#include "CadNode.h"

#include <cstdint>
#include <vector>

// Host-independent transport rules. See docs/conveyors.md.
namespace conveyorcore {

using ConveyorId = uint64_t;
using ProductId = uint64_t;

enum class Mode { Logical, Physx };
enum class Role { Normal, Spawner, Deleter, PickFeeder };

constexpr double kMinimumSpawnIntervalSeconds = 0.05;

// Fraction of a spawner deck reserved for newly created products.
constexpr double kSpawnerRegionProgress = 0.5;

struct ConveyorSpec {
    Role role = Role::Normal;
    Mode mode = Mode::Logical;
    double speedMmS = 250.0;
    double pathLengthMm = 1000.0;
    double spawnIntervalSeconds = 1.0;
    int maxActiveSpawns = 0;    // Zero is unlimited.
    double endInsetMm = 70.0;
    double productPitchMm = 90.0;
};

ConveyorSpec conveyorSpecFrom(const TransformNodeData& parameters, Mode mode);

// Raises an origin-to-origin queue pitch to fit one workpiece.
bool raisePitchToFitWorkpiece(TransformNodeData& parameters, double extentAlongLaneMm);

struct Lane {
    const TransformNodeData* parameters = nullptr;
    CadTransform world;
    Mode mode = Mode::Logical;
};

struct Handover {
    int lane = -1;
    bool forward = true;
    bool valid() const { return lane >= 0; }
};

constexpr double kLaneJoinToleranceMm = 15.0;

// Finds the nearest compatible lane endpoint within kLaneJoinToleranceMm.
Handover nextLane(const std::vector<Lane>& lanes, size_t from, bool leavingForward);

struct Product {
    ProductId id = 0;
    ConveyorId conveyor = 0;
    double progress = 0.0;
    bool forward = true;
    bool grasped = false;
    bool physical = false;
    // Stable across transfers and used for spawner caps.
    ConveyorId originSpawner = 0;
};

struct TransferTarget {
    ConveyorId conveyor = 0;
    bool forward = true;
    bool valid = false;
};

class Host {
public:
    virtual ~Host() = default;

    virtual bool conveyorSpec(ConveyorId, ConveyorSpec* out) const = 0;

    virtual bool productHasBody(const Product&) const = 0;
    virtual bool productBodyPose(const Product&, CadTransform* world) const = 0;

    virtual double closestProgress(ConveyorId, const CadVec3& world) const = 0;

    virtual bool inDeleterVolume(const Product&, const CadVec3& world) const = 0;
    virtual bool reachedEnd(const Product&, const CadVec3& world, double allowanceMm) const = 0;
    virtual void notePhysicalCenter(const Product&, const CadVec3& world) = 0;

    virtual TransferTarget nextConveyor(ConveyorId, bool leavingForward) const = 0;
    virtual void reparentProduct(const Product&, ConveyorId destination) = 0;

    virtual void placeProduct(const Product&) = 0;

    virtual bool spawnProduct(ConveyorId, Product* out) = 0;
    virtual void destroyProduct(const Product&) = 0;

    virtual void sceneChanged() = 0;
};

class Runtime {
public:
    void setHost(Host* host) { m_host = host; }
    void reset();

    void addSpawner(ConveyorId);
    // Existing spawners retain their countdown.
    void setSpawners(const std::vector<ConveyorId>& conveyors);
    void restartSpawners();
    void addProduct(const Product&);
    void removeProduct(ProductId);
    void setProducts(std::vector<Product> products);
    void setGrasped(ProductId, bool);

    void step(double seconds);

    const std::vector<Product>& products() const { return m_products; }
    Product* product(ProductId);
    size_t activeSpawnCount(ConveyorId spawner) const;

    bool spawnerAtCap(ConveyorId spawner) const;
    bool entryBlocked(ConveyorId) const;
    bool endStopOccupied(ConveyorId) const;
    size_t productsOn(ConveyorId) const;

private:
    struct Spawner {
        ConveyorId conveyor = 0;
        double secondsUntilSpawn = 0.0;
    };

    void resolveAccumulation();

    Host* m_host = nullptr;
    std::vector<Product> m_products;
    std::vector<Spawner> m_spawners;
};

} // namespace conveyorcore
