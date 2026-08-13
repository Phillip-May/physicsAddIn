#include "SceneConveyorHost.h"

#include "AccessoryGeometry.h"
#include "AppState.h"
#include "ConveyorCore.h"
#include "ConveyorRuntime.h"

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

namespace {

using conveyorcore::ConveyorId;
using conveyorcore::ConveyorSpec;
using conveyorcore::Mode;
using conveyorcore::Product;
using conveyorcore::ProductId;
using conveyorcore::TransferTarget;

// A conveyor is its CadNode, and the core only ever hands the id back. Nothing is dereferenced
// without conveyorParameters agreeing it is still a conveyor, which is also how a product whose
// conveyor has been deleted is noticed.
ConveyorId conveyorIdOf(const CadNode* node) {
    return reinterpret_cast<ConveyorId>(node);
}

CadNode* conveyorNodeOf(ConveyorId id) {
    return reinterpret_cast<CadNode*>(id);
}

Product productFrom(const ConveyorWorkpiece& piece) {
    Product product;
    product.id = piece.id;
    product.conveyor = conveyorIdOf(piece.conveyor);
    product.progress = piece.progress;
    product.forward = piece.forward;
    product.grasped = piece.graspTool != nullptr;
    product.originSpawner = conveyorIdOf(piece.originSpawner);
    return product;
}

ConveyorWorkpiece* sceneWorkpiece(ProductId id) {
    const auto hit = std::find_if(g_scene.conveyorWorkpieces.begin(),
                                  g_scene.conveyorWorkpieces.end(),
                                  [id](const ConveyorWorkpiece& piece) { return piece.id == id; });
    return hit == g_scene.conveyorWorkpieces.end() ? nullptr : &*hit;
}

// The scene's own record, brought up to date with what the rules have just decided.
ConveyorWorkpiece* syncedWorkpiece(const Product& product) {
    ConveyorWorkpiece* piece = sceneWorkpiece(product.id);
    if (!piece) return nullptr;
    piece->conveyor = conveyorNodeOf(product.conveyor);
    piece->progress = product.progress;
    piece->forward = product.forward;
    return piece;
}

class SceneConveyorHost final : public conveyorcore::Host {
public:
    bool conveyorSpec(ConveyorId id, ConveyorSpec* out) const override {
        const TransformNodeData* parameters = conveyorParameters(conveyorNodeOf(id));
        if (!parameters) return false;
        // The mapping itself is `conveyorSpecFrom`, shared with the RoboDK host, which now carries
        // the same accessory block on its conveyor's item. Only the mode is this host's: "global"
        // resolves against a station default the core knows nothing about.
        *out = conveyorcore::conveyorSpecFrom(
            *parameters, resolvedConveyorMode(*parameters) == ConveyorSimulationMode::PhysX
                ? Mode::Physx : Mode::Logical);
        return true;
    }

    bool productHasBody(const Product& product) const override {
        const ConveyorWorkpiece* piece = sceneWorkpiece(product.id);
        return piece && piece->physicsBody != 0;
    }

    bool productBodyPose(const Product& product, CadTransform* world) const override {
        const ConveyorWorkpiece* piece = sceneWorkpiece(product.id);
        return piece && piece->physicsBody &&
               g_scene.conveyorPhysics.bodyPose(piece->physicsBody, world);
    }

    double closestProgress(ConveyorId id, const CadVec3& world) const override {
        return closestConveyorProgress(conveyorNodeOf(id), world);
    }

    bool inDeleterVolume(const Product& product, const CadVec3& world) const override {
        const ConveyorWorkpiece* piece = syncedWorkpiece(product);
        return piece && deleterVolumeIntersects(*piece, world);
    }

    bool reachedEnd(const Product& product, const CadVec3& world,
                    double allowanceMm) const override {
        const ConveyorWorkpiece* piece = syncedWorkpiece(product);
        return piece && physicalBodyReachedEnd(*piece, world, allowanceMm);
    }

    void notePhysicalCenter(const Product& product, const CadVec3& world) override {
        if (ConveyorWorkpiece* piece = sceneWorkpiece(product.id)) {
            piece->previousPhysicalCenter = world;
            piece->hasPreviousPhysicalCenter = true;
        }
    }

    TransferTarget nextConveyor(ConveyorId id, bool leavingForward) const override {
        const ConveyorTransferTarget found = ::nextConveyor(conveyorNodeOf(id), leavingForward);
        TransferTarget target;
        target.conveyor = conveyorIdOf(found.conveyor);
        target.forward = found.forward;
        target.valid = found.conveyor != nullptr;
        return target;
    }

    void reparentProduct(const Product& product, ConveyorId destination) override {
        if (ConveyorWorkpiece* piece = syncedWorkpiece(product)) {
            reparentConveyorWorkpiece(*piece, conveyorNodeOf(destination));
        }
    }

    void placeProduct(const Product& product) override {
        if (ConveyorWorkpiece* piece = syncedWorkpiece(product)) placeConveyorWorkpiece(*piece);
    }

    bool spawnProduct(ConveyorId id, Product* out) override {
        const size_t before = g_scene.conveyorWorkpieces.size();
        spawnConveyorWorkpiece(conveyorNodeOf(id));
        if (g_scene.conveyorWorkpieces.size() == before) return false;
        *out = productFrom(g_scene.conveyorWorkpieces.back());
        return true;
    }

    void destroyProduct(const Product& product) override {
        const auto hit = std::find_if(
            g_scene.conveyorWorkpieces.begin(), g_scene.conveyorWorkpieces.end(),
            [&](const ConveyorWorkpiece& piece) { return piece.id == product.id; });
        if (hit == g_scene.conveyorWorkpieces.end()) return;
        if (hit->physicsBody) g_scene.conveyorPhysics.removeBody(hit->physicsBody);
        detachConveyorWorkpiece(*hit);
        g_scene.conveyorWorkpieces.erase(hit);
    }

    void sceneChanged() override { conveyorMarkSceneDirty(); }
};

SceneConveyorHost g_host;
conveyorcore::Runtime g_core;

} // namespace

void resetSceneConveyorCore() {
    g_core.reset();
}

void addSceneConveyorSpawner(CadNode* conveyor) {
    g_core.addSpawner(conveyorIdOf(conveyor));
}

void stepSceneConveyorRules(double seconds) {
    g_core.setHost(&g_host);
    std::vector<Product> products;
    products.reserve(g_scene.conveyorWorkpieces.size());
    for (const ConveyorWorkpiece& piece : g_scene.conveyorWorkpieces) {
        products.push_back(productFrom(piece));
    }
    g_core.setProducts(std::move(products));

    g_core.step(seconds);

    for (const Product& product : g_core.products()) syncedWorkpiece(product);
}
