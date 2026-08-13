#include "ConveyorPhysics.h"
#include "RollingRateWindow.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <mutex>
#include <thread>
#include <vector>

#ifdef ROBOTSIM_WITH_PHYSX
#include "PxCadTransform.h"

#include <PxPhysicsAPI.h>

using namespace physx;

namespace {
constexpr float kGripperDynamicFriction = 2.5f;
// A 30 mm, 0.1 kg box has very little rotational inertia; the angular pad response must therefore
// be much softer than the translational response to remain compliant at the 120 Hz fixed step.
constexpr float kGripAngularStiffnessNmRad = 0.02f;
constexpr float kGripAngularDampingNmsRad = 0.001f;
constexpr float kGripPadLeverM = 0.03f;
constexpr PxU32 kSurfaceFilter = 1u << 0;
constexpr PxU32 kProductFilter = 1u << 1;

struct ConveyorSurfaceData {
    PxVec3 linearVelocityMps{0.0f};
    PxVec3 centerM{0.0f};
    PxVec3 angularVelocityRadS{0.0f};
};

PxFilterFlags conveyorFilterShader(PxFilterObjectAttributes attributes0,
                                   PxFilterData filterData0,
                                   PxFilterObjectAttributes attributes1,
                                   PxFilterData filterData1,
                                   PxPairFlags& pairFlags,
                                   const void*, PxU32) {
    if (PxFilterObjectIsTrigger(attributes0) || PxFilterObjectIsTrigger(attributes1)) {
        pairFlags = PxPairFlag::eTRIGGER_DEFAULT;
        return PxFilterFlag::eDEFAULT;
    }
    pairFlags = PxPairFlag::eCONTACT_DEFAULT | PxPairFlag::eDETECT_CCD_CONTACT;
    const bool surfaceProduct =
        ((filterData0.word0 & kSurfaceFilter) && (filterData1.word0 & kProductFilter)) ||
        ((filterData1.word0 & kSurfaceFilter) && (filterData0.word0 & kProductFilter));
    if (surfaceProduct) pairFlags |= PxPairFlag::eMODIFY_CONTACTS;
    return PxFilterFlag::eDEFAULT;
}

class ConveyorContactModifyCallback final : public PxContactModifyCallback {
public:
    void onContactModify(PxContactModifyPair* const pairs, PxU32 count) override {
        for (PxU32 pairIndex = 0; pairIndex < count; ++pairIndex) {
            PxContactModifyPair& pair = pairs[pairIndex];
            int surfaceIndex = -1;
            if (pair.shape[0] && pair.shape[0]->userData) surfaceIndex = 0;
            else if (pair.shape[1] && pair.shape[1]->userData) surfaceIndex = 1;
            if (surfaceIndex < 0) continue;
            const auto* surface = static_cast<const ConveyorSurfaceData*>(
                pair.shape[surfaceIndex]->userData);
            // PxContactSet target velocity is actor[0] relative to actor[1]. To make a dynamic
            // product move with the belt, request the opposite relative velocity when the static
            // conveyor surface is actor[0]. PhysX may reverse pair order between frames.
            for (PxU32 contact = 0; contact < pair.contacts.size(); ++contact) {
                const PxVec3 radius = pair.contacts.getPoint(contact) - surface->centerM;
                const PxVec3 surfaceVelocity = surface->linearVelocityMps +
                    surface->angularVelocityRadS.cross(radius);
                const PxVec3 target = surfaceIndex == 0
                    ? -surfaceVelocity : surfaceVelocity;
                pair.contacts.setTargetVelocity(contact, target);
            }
        }
    }
};

}

struct ConveyorPhysics::Impl {
    struct Body {
        BodyHandle handle = 0;
        PxRigidDynamic* actor = nullptr;
        CadTransform cachedPose;
    };

    struct CylinderMesh {
        double radiusMm = 0.0;
        double heightMm = 0.0;
        PxConvexMesh* mesh = nullptr;
    };

    struct GripDrive {
        BodyHandle body = 0;
        PxVec3 targetM;
        PxVec3 targetVelocityMps;
        PxQuat targetOrientation{PxIdentity};
        float stiffnessNPerM = 0.0f;
        float dampingNsPerM = 0.0f;
        float maximumForceN = 0.0f;
    };

    CadVec3 gravityMps2{0.0, -9.81, 0.0};
    PxDefaultAllocator allocator;
    PxDefaultErrorCallback errors;
    PxFoundation* foundation = nullptr;
    PxPhysics* physics = nullptr;
    PxDefaultCpuDispatcher* dispatcher = nullptr;
    PxScene* scene = nullptr;
    PxMaterial* deckMaterial = nullptr;
    PxMaterial* rollerMaterial = nullptr;
    PxMaterial* productMaterial = nullptr;
    PxMaterial* gripperMaterial = nullptr;
    ConveyorContactModifyCallback contactModifyCallback;
    std::vector<std::unique_ptr<ConveyorSurfaceData>> surfaces;
    std::vector<Body> bodies;
    std::vector<CylinderMesh> cylinderMeshes;
    std::vector<GripDrive> gripDrives;
    BodyHandle nextHandle = 1;
    double accumulator = 0.0;
    mutable std::mutex sceneMutex;
    mutable std::mutex snapshotMutex;
#ifndef __EMSCRIPTEN__
    std::thread worker;
#endif
    std::atomic<bool> workerStop{false};
    std::atomic<bool> workerActive{false};
    std::atomic<bool> workerPaused{false};
    std::atomic<double> workerSpeedFactor{1.0};
    std::atomic<uint64_t> completedSteps{0};
    std::atomic<double> droppedSimSeconds{0.0};
    std::atomic<double> achievedRate{0.0};
    std::atomic<bool> achievedRateValid{false};
    std::atomic<int> sceneMutationWaiters{0};
    std::atomic<uint32_t> staticActorCount{0};
    std::atomic<uint32_t> activeDynamicBodyCount{0};
    std::atomic<uint32_t> contactPairCount{0};
    std::atomic<uint32_t> convexBoxPairCount{0};
    std::atomic<uint32_t> convexTrianglePairCount{0};
    std::atomic<uint32_t> convexConvexPairCount{0};
    std::atomic<uint32_t> dispatcherThreadCount{0};
    std::atomic<uint32_t> solverBatchSize{0};
    std::atomic<double> averageWakeMs{0.0};
    std::atomic<double> averageSimulateMs{0.0};
    std::atomic<double> averageSnapshotMs{0.0};
    std::atomic<double> minimumBodyYmm{0.0};

    ~Impl() { stopWorker(); release(); }

    void publishBodyPosesLocked() {
        std::lock_guard<std::mutex> snapshotGuard(snapshotMutex);
        double minimumY = 0.0;
        bool foundBody = false;
        for (Body& body : bodies) {
            if (!body.actor) continue;
            body.cachedPose = fromPxTransform(body.actor->getGlobalPose());
            minimumY = foundBody ? std::min(minimumY, body.cachedPose.values[7])
                                 : body.cachedPose.values[7];
            foundBody = true;
        }
        minimumBodyYmm.store(foundBody ? minimumY : 0.0);
    }

    void simulateFixedStepLocked(double seconds) {
        if (!scene) return;
        for (const GripDrive& drive : gripDrives) {
            Body* body = find(drive.body);
            if (!body || !body->actor ||
                (body->actor->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC)) continue;
            const PxVec3 displacement = drive.targetM - body->actor->getGlobalPose().p;
            PxVec3 force = displacement * drive.stiffnessNPerM +
                (drive.targetVelocityMps - body->actor->getLinearVelocity()) *
                    drive.dampingNsPerM;
            const float magnitude = force.magnitude();
            if (magnitude > drive.maximumForceN && magnitude > 1.0e-6f) {
                force *= drive.maximumForceN / magnitude;
            }
            body->actor->addForce(force, PxForceMode::eFORCE, true);

            // Two separated friction pads also resist relative rotation. This remains a bounded
            // torque on a free rigid body, so an overloaded part can still rotate or slip.
            PxQuat angularError = drive.targetOrientation *
                body->actor->getGlobalPose().q.getConjugate();
            if (angularError.w < 0.0f) {
                angularError = PxQuat(-angularError.x, -angularError.y, -angularError.z,
                                      -angularError.w);
            }
            float errorRadians = 0.0f;
            PxVec3 errorAxis(0.0f);
            angularError.toRadiansAndUnitAxis(errorRadians, errorAxis);
            PxVec3 torque = errorAxis * (errorRadians * kGripAngularStiffnessNmRad) -
                body->actor->getAngularVelocity() * kGripAngularDampingNmsRad;
            const float maximumTorqueNm = drive.maximumForceN * kGripPadLeverM;
            const float torqueMagnitude = torque.magnitude();
            if (torqueMagnitude > maximumTorqueNm && torqueMagnitude > 1.0e-6f) {
                torque *= maximumTorqueNm / torqueMagnitude;
            }
            body->actor->addTorque(torque, PxForceMode::eFORCE, true);
        }
        using Clock = std::chrono::steady_clock;
        const auto wakeStart = Clock::now();
        const auto simulateStart = Clock::now();
        scene->simulate(static_cast<float>(seconds));
        scene->fetchResults(true);
        const auto snapshotStart = Clock::now();
        PxSimulationStatistics statistics;
        scene->getSimulationStatistics(statistics);
        activeDynamicBodyCount.store(statistics.nbActiveDynamicBodies);
        contactPairCount.store(statistics.nbDiscreteContactPairsTotal);
        convexBoxPairCount.store(statistics.nbDiscreteContactPairs
            [PxGeometryType::eBOX][PxGeometryType::eCONVEXMESH]);
        convexTrianglePairCount.store(statistics.nbDiscreteContactPairs
            [PxGeometryType::eCONVEXMESH][PxGeometryType::eTRIANGLEMESH]);
        convexConvexPairCount.store(statistics.nbDiscreteContactPairs
            [PxGeometryType::eCONVEXMESH][PxGeometryType::eCONVEXMESH]);
        publishBodyPosesLocked();
        const auto finished = Clock::now();
        const auto updateAverage = [](std::atomic<double>& average, double sample) {
            const double previous = average.load();
            average.store(previous <= 0.0 ? sample : previous * 0.95 + sample * 0.05);
        };
        updateAverage(averageWakeMs,
                      std::chrono::duration<double, std::milli>(simulateStart - wakeStart).count());
        updateAverage(averageSimulateMs,
                      std::chrono::duration<double, std::milli>(snapshotStart - simulateStart).count());
        updateAverage(averageSnapshotMs,
                      std::chrono::duration<double, std::milli>(finished - snapshotStart).count());
    }

    void startWorker(double speedFactor) {
        stopWorker();
        workerSpeedFactor.store(std::max(0.01, speedFactor));
        completedSteps.store(0);
        droppedSimSeconds.store(0.0);
        achievedRate.store(0.0);
        achievedRateValid.store(false);
#ifndef __EMSCRIPTEN__
        if (!scene) return;
        workerStop.store(false);
        workerActive.store(true);
        worker = std::thread([this]() {
            using Clock = std::chrono::steady_clock;
            constexpr double fixedStep = ConveyorPhysics::fixedStepSeconds();
            // Bound simulation debt. At very high requested rates PhysX is intentionally allowed
            // to fall behind station time instead of monopolizing a core trying to catch up.
            constexpr double maxWallDebtSeconds = 0.10;
            auto previous = Clock::now();
            double pendingSimSeconds = 0.0;
            RollingRateWindow rateWindow;
            double previousSpeedFactor = workerSpeedFactor.load();
            while (!workerStop.load()) {
                const auto now = Clock::now();
                const double wallSeconds = std::chrono::duration<double>(now - previous).count();
                previous = now;
                const double speedFactor = std::max(0.01, workerSpeedFactor.load());
                if (std::abs(speedFactor - previousSpeedFactor) > 1.0e-9) {
                    pendingSimSeconds = 0.0;
                    rateWindow.reset();
                    achievedRateValid.store(false);
                    previousSpeedFactor = speedFactor;
                }
                if (workerPaused.load()) {
                    pendingSimSeconds = 0.0;
                    rateWindow.reset();
                    achievedRateValid.store(false);
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    continue;
                }

                pendingSimSeconds += std::max(0.0, std::min(0.1, wallSeconds)) * speedFactor;
                const double maxSimDebt = std::max(fixedStep, maxWallDebtSeconds * speedFactor);
                if (pendingSimSeconds > maxSimDebt) {
                    droppedSimSeconds.store(
                        droppedSimSeconds.load() + pendingSimSeconds - maxSimDebt);
                    pendingSimSeconds = maxSimDebt;
                }

                double deliveredSimSeconds = 0.0;
                if (pendingSimSeconds >= fixedStep) {
                    {
                        std::lock_guard<std::mutex> sceneGuard(sceneMutex);
                        simulateFixedStepLocked(fixedStep);
                    }
                    pendingSimSeconds -= fixedStep;
                    deliveredSimSeconds = fixedStep;
                    completedSteps.fetch_add(1);
                    // addCylinder/removeBody are rare UI-side mutations. Give a waiting mutation
                    // the next lock acquisition instead of allowing this tight loop to win the
                    // non-fair std::mutex repeatedly at high requested rates.
                    while (sceneMutationWaiters.load() > 0 && !workerStop.load()) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    }
                } else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }

                double rollingRate = 0.0;
                if (rateWindow.addSample(wallSeconds, deliveredSimSeconds, &rollingRate)) {
                    achievedRate.store(rollingRate);
                    achievedRateValid.store(true);
                }
            }
            workerActive.store(false);
        });
#endif
    }

    void stopWorker() {
        workerActive.store(false);
        workerStop.store(true);
#ifndef __EMSCRIPTEN__
        if (worker.joinable()) worker.join();
#endif
        workerStop.store(false);
        workerPaused.store(false);
        completedSteps.store(0);
        achievedRateValid.store(false);
    }

    void release() {
        stopWorker();
        std::lock_guard<std::mutex> sceneGuard(sceneMutex);
        std::lock_guard<std::mutex> snapshotGuard(snapshotMutex);
        bodies.clear();
        gripDrives.clear();
        surfaces.clear();
        if (scene) scene->release();
        scene = nullptr;
        for (CylinderMesh& entry : cylinderMeshes) {
            if (entry.mesh) entry.mesh->release();
        }
        cylinderMeshes.clear();
        if (dispatcher) dispatcher->release();
        dispatcher = nullptr;
        if (productMaterial) productMaterial->release();
        productMaterial = nullptr;
        if (gripperMaterial) gripperMaterial->release();
        gripperMaterial = nullptr;
        if (rollerMaterial) rollerMaterial->release();
        rollerMaterial = nullptr;
        if (deckMaterial) deckMaterial->release();
        deckMaterial = nullptr;
        if (physics) physics->release();
        physics = nullptr;
        if (foundation) foundation->release();
        foundation = nullptr;
        accumulator = 0.0;
        staticActorCount.store(0);
        activeDynamicBodyCount.store(0);
        contactPairCount.store(0);
        convexBoxPairCount.store(0);
        convexTrianglePairCount.store(0);
        convexConvexPairCount.store(0);
        dispatcherThreadCount.store(0);
        solverBatchSize.store(0);
        averageWakeMs.store(0.0);
        averageSimulateMs.store(0.0);
        averageSnapshotMs.store(0.0);
        minimumBodyYmm.store(0.0);
    }

    bool fail(const char* message, std::string* errorMessage) {
        if (errorMessage) *errorMessage = message;
        return false;
    }

    Body* find(BodyHandle handle) {
        const auto found = std::find_if(bodies.begin(), bodies.end(),
                                        [handle](const Body& body) {
                                            return body.handle == handle;
                                        });
        return found == bodies.end() ? nullptr : &*found;
    }

    const Body* find(BodyHandle handle) const {
        const auto found = std::find_if(bodies.begin(), bodies.end(),
                                        [handle](const Body& body) {
                                            return body.handle == handle;
                                        });
        return found == bodies.end() ? nullptr : &*found;
    }
};

#else
struct ConveyorPhysics::Impl {
    CadVec3 gravityMps2{0.0, -9.81, 0.0};
    BodyHandle nextHandle = 1;
};
#endif

void ConveyorPhysics::setGravity(const CadVec3& metresPerSecondSquared) {
    m_impl->gravityMps2 = metresPerSecondSquared;
}

ConveyorPhysics::ConveyorPhysics() : m_impl(std::make_unique<Impl>()) {}
ConveyorPhysics::~ConveyorPhysics() = default;
ConveyorPhysics::ConveyorPhysics(ConveyorPhysics&&) noexcept = default;
ConveyorPhysics& ConveyorPhysics::operator=(ConveyorPhysics&&) noexcept = default;

bool ConveyorPhysics::backendAvailable() {
#ifdef ROBOTSIM_WITH_PHYSX
    return true;
#else
    return false;
#endif
}

bool ConveyorPhysics::reset(const std::vector<CollisionMesh>& meshes, std::string* errorMessage) {
#ifndef ROBOTSIM_WITH_PHYSX
    (void)meshes;
    if (errorMessage) errorMessage->clear();
    return true;
#else
    m_impl->release();
    if (meshes.empty()) {
        if (errorMessage) errorMessage->clear();
        return true;
    }
    m_impl->foundation = PxCreateFoundation(PX_PHYSICS_VERSION, m_impl->allocator, m_impl->errors);
    if (!m_impl->foundation) return m_impl->fail("PhysX conveyor foundation creation failed.", errorMessage);
    PxTolerancesScale scale;
    scale.length = 1.0f;
    scale.speed = 10.0f;
    m_impl->physics = PxCreatePhysics(PX_PHYSICS_VERSION, *m_impl->foundation, scale, false, nullptr);
    if (!m_impl->physics) return m_impl->fail("PhysX conveyor core creation failed.", errorMessage);
    PxSceneDesc desc(scale);
    desc.gravity = PxVec3(static_cast<float>(m_impl->gravityMps2.x),
                          static_cast<float>(m_impl->gravityMps2.y),
                          static_cast<float>(m_impl->gravityMps2.z));
    desc.solverType = PxSolverType::eTGS;
    desc.flags |= PxSceneFlag::eENABLE_CCD;
    uint32_t solverBatchSize = 128;
    if (const char* configured = std::getenv("ROBOTSIM_CONVEYOR_SOLVER_BATCH")) {
        const long parsed = std::strtol(configured, nullptr, 10);
        if (parsed > 0) solverBatchSize = static_cast<uint32_t>(std::min<long>(parsed, 1024));
    }
    desc.solverBatchSize = solverBatchSize;
    m_impl->solverBatchSize.store(solverBatchSize);
    // The browser build deliberately does not require SharedArrayBuffer/COOP/COEP. PhysX supports
    // a zero-worker dispatcher: submitted tasks execute on the calling thread, which gives WASM
    // the complete CPU solver without trying to create unavailable pthread workers.
#ifdef __EMSCRIPTEN__
    uint32_t dispatcherThreads = 0;
#else
    uint32_t dispatcherThreads = 1;
#endif
    if (const char* configured = std::getenv("ROBOTSIM_CONVEYOR_PHYSX_THREADS")) {
        const long parsed = std::strtol(configured, nullptr, 10);
#ifdef __EMSCRIPTEN__
        (void)parsed;
#else
        if (parsed > 0) dispatcherThreads = static_cast<uint32_t>(std::min<long>(parsed, 32));
#endif
    }
    m_impl->dispatcher = PxDefaultCpuDispatcherCreate(dispatcherThreads);
    m_impl->dispatcherThreadCount.store(dispatcherThreads);
    desc.cpuDispatcher = m_impl->dispatcher;
    desc.filterShader = conveyorFilterShader;
    desc.contactModifyCallback = &m_impl->contactModifyCallback;
    m_impl->scene = m_impl->physics->createScene(desc);
    if (!m_impl->scene) return m_impl->fail("PhysX conveyor scene creation failed.", errorMessage);
    // Frame members and transfer ties should not brake a product being pushed by adjacent
    // rollers. Powered roller shapes use their own high-friction contact material below.
    m_impl->deckMaterial = m_impl->physics->createMaterial(0.15f, 0.10f, 0.02f);
    m_impl->rollerMaterial = m_impl->physics->createMaterial(0.95f, 0.85f, 0.02f);
    m_impl->productMaterial = m_impl->physics->createMaterial(0.6f, 0.5f, 0.05f);
    // A closed industrial gripper uses compliant high-friction pads. Values above one are valid
    // Coulomb coefficients in PhysX and intentionally model the pad, not bare aluminium jaws.
    m_impl->gripperMaterial = m_impl->physics->createMaterial(
        3.0f, kGripperDynamicFriction, 0.0f);
    if (!m_impl->deckMaterial || !m_impl->rollerMaterial || !m_impl->productMaterial ||
        !m_impl->gripperMaterial) {
        return m_impl->fail("PhysX conveyor material creation failed.", errorMessage);
    }

    const PxCookingParams cookingParams(m_impl->physics->getTolerancesScale());
    // Every generated conveyor member is stationary. Keeping them as shapes on one static actor
    // preserves independent geometry, materials and per-shape belt target velocities while
    // avoiding a separate actor/island-graph node for every short spiral chord and frame mesh.
    PxRigidStatic* staticActor = m_impl->physics->createRigidStatic(PxTransform(PxIdentity));
    if (!staticActor) {
        return m_impl->fail("PhysX conveyor static actor creation failed.", errorMessage);
    }
    for (const CollisionMesh& mesh : meshes) {
        const bool nativeBox = mesh.boxHalfExtentsMm.x > 0.0 &&
                               mesh.boxHalfExtentsMm.y > 0.0 &&
                               mesh.boxHalfExtentsMm.z > 0.0;
        if (!nativeBox &&
            (mesh.verticesMm.size() < 9 || mesh.verticesMm.size() % 3 != 0 ||
             mesh.indices.size() < 3 || mesh.indices.size() % 3 != 0)) {
            staticActor->release();
            return m_impl->fail("PhysX conveyor received malformed generated collision geometry.",
                                errorMessage);
        }
        const PxVec3 surfaceAngularVelocity(
            static_cast<float>(mesh.surfaceAngularVelocityRadS.x),
            static_cast<float>(mesh.surfaceAngularVelocityRadS.y),
            static_cast<float>(mesh.surfaceAngularVelocityRadS.z));
        const PxVec3 surfaceLinearVelocity(
            static_cast<float>(mesh.surfaceLinearVelocityMmS.x) * kMmToM,
            static_cast<float>(mesh.surfaceLinearVelocityMmS.y) * kMmToM,
            static_cast<float>(mesh.surfaceLinearVelocityMmS.z) * kMmToM);
        const bool poweredSurface = surfaceAngularVelocity.magnitudeSquared() > 1.0e-12f ||
                                    surfaceLinearVelocity.magnitudeSquared() > 1.0e-12f;
        PxShape* shape = nullptr;
        if (nativeBox) {
            const PxBoxGeometry geometry(
                static_cast<float>(mesh.boxHalfExtentsMm.x) * kMmToM,
                static_cast<float>(mesh.boxHalfExtentsMm.y) * kMmToM,
                static_cast<float>(mesh.boxHalfExtentsMm.z) * kMmToM);
            shape = m_impl->physics->createShape(
                geometry, poweredSurface ? *m_impl->rollerMaterial : *m_impl->deckMaterial, true);
        } else {
            std::vector<PxVec3> vertices;
            vertices.reserve(mesh.verticesMm.size() / 3);
            for (size_t index = 0; index < mesh.verticesMm.size(); index += 3) {
                vertices.emplace_back(mesh.verticesMm[index] * kMmToM,
                                      mesh.verticesMm[index + 1] * kMmToM,
                                      mesh.verticesMm[index + 2] * kMmToM);
            }
            PxTriangleMeshDesc descriptor;
            descriptor.points.count = static_cast<PxU32>(vertices.size());
            descriptor.points.stride = sizeof(PxVec3);
            descriptor.points.data = vertices.data();
            descriptor.triangles.count = static_cast<PxU32>(mesh.indices.size() / 3);
            descriptor.triangles.stride = 3 * sizeof(uint32_t);
            descriptor.triangles.data = mesh.indices.data();
            PxDefaultMemoryOutputStream cooked;
            if (!PxCookTriangleMesh(cookingParams, descriptor, cooked)) {
                staticActor->release();
                return m_impl->fail("PhysX could not cook generated conveyor collision geometry.",
                                    errorMessage);
            }
            PxDefaultMemoryInputData input(cooked.getData(), cooked.getSize());
            PxTriangleMesh* triangleMesh = m_impl->physics->createTriangleMesh(input);
            if (!triangleMesh) {
                staticActor->release();
                return m_impl->fail("PhysX could not load generated conveyor collision geometry.",
                                    errorMessage);
            }
            const PxTriangleMeshGeometry geometry(
                triangleMesh, PxMeshScale(), PxMeshGeometryFlag::eDOUBLE_SIDED);
            shape = m_impl->physics->createShape(
                geometry, poweredSurface ? *m_impl->rollerMaterial : *m_impl->deckMaterial, true);
            triangleMesh->release();
        }
        if (!shape) {
            staticActor->release();
            return m_impl->fail("PhysX conveyor mesh shape creation failed.", errorMessage);
        }
        shape->setLocalPose(toPxTransform(mesh.pose));
        shape->setContactOffset(0.003f);
        if (poweredSurface) {
            auto surface = std::make_unique<ConveyorSurfaceData>();
            surface->linearVelocityMps = surfaceLinearVelocity;
            surface->centerM = PxVec3(
                static_cast<float>(mesh.surfaceCenterMm.x) * kMmToM,
                static_cast<float>(mesh.surfaceCenterMm.y) * kMmToM,
                static_cast<float>(mesh.surfaceCenterMm.z) * kMmToM);
            surface->angularVelocityRadS = surfaceAngularVelocity;
            shape->userData = surface.get();
            m_impl->surfaces.push_back(std::move(surface));
            PxFilterData filter;
            filter.word0 = kSurfaceFilter;
            shape->setSimulationFilterData(filter);
        }
        staticActor->attachShape(*shape);
        shape->release();
    }
    m_impl->scene->addActor(*staticActor);
    m_impl->staticActorCount.store(1);
    if (errorMessage) errorMessage->clear();
    return true;
#endif
}

ConveyorPhysics::BodyHandle ConveyorPhysics::addCylinder(
    const CadTransform& worldPose, double radiusMm, double heightMm, std::string* errorMessage) {
#ifndef ROBOTSIM_WITH_PHYSX
    (void)worldPose; (void)radiusMm; (void)heightMm;
    if (errorMessage) *errorMessage = "PhysX conveyor products are unavailable in this build.";
    return 0;
#else
    m_impl->sceneMutationWaiters.fetch_add(1);
    std::unique_lock<std::mutex> sceneGuard(m_impl->sceneMutex);
    m_impl->sceneMutationWaiters.fetch_sub(1);
    if (!m_impl->scene || !m_impl->physics || !m_impl->productMaterial) {
        if (errorMessage) *errorMessage = "PhysX conveyor scene is not initialized.";
        return 0;
    }
    PxRigidDynamic* actor = m_impl->physics->createRigidDynamic(toPxTransform(worldPose));
    if (!actor) return 0;
    PxConvexMesh* convexMesh = nullptr;
    for (const Impl::CylinderMesh& cached : m_impl->cylinderMeshes) {
        if (std::abs(cached.radiusMm - radiusMm) < 1.0e-6 &&
            std::abs(cached.heightMm - heightMm) < 1.0e-6) {
            convexMesh = cached.mesh;
            break;
        }
    }
    if (!convexMesh) {
        // PhysX has no native cylinder shape. A 16-sided convex hull is visually indistinguishable
        // at product scale while reducing GJK/manifold work versus mirroring every one of the 24
        // render sides. Spawners reuse this immutable cooked mesh once per configured product size.
        constexpr int kSegments = 16;
        constexpr float kTwoPi = 6.28318530717958647692f;
        std::vector<PxVec3> vertices;
        vertices.reserve(kSegments * 2);
        for (int end = 0; end < 2; ++end) {
            const float y = static_cast<float>((end == 0 ? -0.5 : 0.5) * heightMm) * kMmToM;
            for (int segment = 0; segment < kSegments; ++segment) {
                const float angle = kTwoPi * static_cast<float>(segment) / kSegments;
                vertices.emplace_back(static_cast<float>(radiusMm) * std::cos(angle) * kMmToM,
                                      y,
                                      static_cast<float>(radiusMm) * std::sin(angle) * kMmToM);
            }
        }
        PxConvexMeshDesc descriptor;
        descriptor.points.count = static_cast<PxU32>(vertices.size());
        descriptor.points.stride = sizeof(PxVec3);
        descriptor.points.data = vertices.data();
        descriptor.flags = PxConvexFlag::eCOMPUTE_CONVEX;
        PxDefaultMemoryOutputStream cooked;
        const PxCookingParams cookingParams(m_impl->physics->getTolerancesScale());
        if (!PxCookConvexMesh(cookingParams, descriptor, cooked)) {
            actor->release();
            if (errorMessage) {
                *errorMessage = "PhysX could not cook the spawned cylinder collision mesh.";
            }
            return 0;
        }
        PxDefaultMemoryInputData input(cooked.getData(), cooked.getSize());
        convexMesh = m_impl->physics->createConvexMesh(input);
        if (!convexMesh) {
            actor->release();
            if (errorMessage) {
                *errorMessage = "PhysX could not load the spawned cylinder collision mesh.";
            }
            return 0;
        }
        m_impl->cylinderMeshes.push_back({radiusMm, heightMm, convexMesh});
    }
    PxShape* shape = m_impl->physics->createShape(
        PxConvexMeshGeometry(convexMesh), *m_impl->productMaterial, true);
    if (!shape) {
        actor->release();
        return 0;
    }
    shape->setContactOffset(0.002f);
    PxFilterData productFilter;
    productFilter.word0 = kProductFilter;
    shape->setSimulationFilterData(productFilter);
    actor->attachShape(*shape);
    shape->release();
    const float volumeM3 = std::max(1.0e-5f,
        static_cast<float>(3.14159265358979323846 * radiusMm * radiusMm * heightMm) * 1.0e-9f);
    constexpr float kDefaultProductDensityKgM3 = 1200.0f;
    PxRigidBodyExt::setMassAndUpdateInertia(
        *actor, std::max(0.1f, volumeM3 * kDefaultProductDensityKgM3));
    actor->setLinearDamping(0.15f);
    actor->setAngularDamping(0.8f);
    actor->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, true);
    // At 60 Hz the normal PhysX 4/1 iteration budget is sufficient for these compact products.
    // The previous 16/4 setting multiplied every belt contact solve by roughly four; the showcase
    // reaches dozens of simultaneous products, so solver work, not the simulation clock, capped it.
    actor->setSolverIterationCounts(4, 1);
    m_impl->scene->addActor(*actor);
    const BodyHandle handle = m_impl->nextHandle++;
    {
        std::lock_guard<std::mutex> snapshotGuard(m_impl->snapshotMutex);
        m_impl->bodies.push_back({handle, actor, worldPose});
    }
    if (errorMessage) errorMessage->clear();
    return handle;
#endif
}

ConveyorPhysics::BodyHandle ConveyorPhysics::addBox(
    const CadTransform& worldPose, const CadVec3& halfExtentsMm, std::string* errorMessage) {
#ifndef ROBOTSIM_WITH_PHYSX
    (void)worldPose; (void)halfExtentsMm;
    if (errorMessage) *errorMessage = "PhysX conveyor products are unavailable in this build.";
    return 0;
#else
    m_impl->sceneMutationWaiters.fetch_add(1);
    std::unique_lock<std::mutex> sceneGuard(m_impl->sceneMutex);
    m_impl->sceneMutationWaiters.fetch_sub(1);
    if (!m_impl->scene || !m_impl->physics || !m_impl->productMaterial) {
        if (errorMessage) *errorMessage = "PhysX conveyor scene is not initialized.";
        return 0;
    }
    const PxVec3 halfExtents(
        std::max(1.0, halfExtentsMm.x) * kMmToM,
        std::max(1.0, halfExtentsMm.y) * kMmToM,
        std::max(1.0, halfExtentsMm.z) * kMmToM);
    PxRigidDynamic* actor = m_impl->physics->createRigidDynamic(toPxTransform(worldPose));
    if (!actor) return 0;
    PxShape* shape = m_impl->physics->createShape(
        PxBoxGeometry(halfExtents), *m_impl->productMaterial, true);
    if (!shape) {
        actor->release();
        return 0;
    }
    shape->setContactOffset(0.002f);
    PxFilterData productFilter;
    productFilter.word0 = kProductFilter;
    shape->setSimulationFilterData(productFilter);
    actor->attachShape(*shape);
    shape->release();
    const float volumeM3 = std::max(1.0e-5f,
        8.0f * halfExtents.x * halfExtents.y * halfExtents.z);
    constexpr float kDefaultProductDensityKgM3 = 1200.0f;
    PxRigidBodyExt::setMassAndUpdateInertia(
        *actor, std::max(0.1f, volumeM3 * kDefaultProductDensityKgM3));
    actor->setLinearDamping(0.15f);
    actor->setAngularDamping(0.8f);
    actor->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, true);
    actor->setSolverIterationCounts(4, 1);
    m_impl->scene->addActor(*actor);
    const BodyHandle handle = m_impl->nextHandle++;
    {
        std::lock_guard<std::mutex> snapshotGuard(m_impl->snapshotMutex);
        m_impl->bodies.push_back({handle, actor, worldPose});
    }
    if (errorMessage) errorMessage->clear();
    return handle;
#endif
}

ConveyorPhysics::BodyHandle ConveyorPhysics::addKinematicBox(
    const CadTransform& worldPose, const CadVec3& halfExtentsMm, std::string* errorMessage) {
#ifndef ROBOTSIM_WITH_PHYSX
    (void)worldPose; (void)halfExtentsMm;
    if (errorMessage) *errorMessage = "PhysX mechanism contacts are unavailable in this build.";
    return 0;
#else
    m_impl->sceneMutationWaiters.fetch_add(1);
    std::unique_lock<std::mutex> sceneGuard(m_impl->sceneMutex);
    m_impl->sceneMutationWaiters.fetch_sub(1);
    if (!m_impl->scene || !m_impl->physics || !m_impl->gripperMaterial) {
        if (errorMessage) *errorMessage = "PhysX conveyor scene is not initialized.";
        return 0;
    }
    const PxVec3 halfExtents(
        std::max(1.0, halfExtentsMm.x) * kMmToM,
        std::max(1.0, halfExtentsMm.y) * kMmToM,
        std::max(1.0, halfExtentsMm.z) * kMmToM);
    PxRigidDynamic* actor = m_impl->physics->createRigidDynamic(toPxTransform(worldPose));
    if (!actor) return 0;
    PxShape* shape = m_impl->physics->createShape(
        PxBoxGeometry(halfExtents), *m_impl->gripperMaterial, true);
    if (!shape) {
        actor->release();
        return 0;
    }
    shape->setContactOffset(0.001f);
    actor->attachShape(*shape);
    shape->release();
    actor->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
    actor->setSolverIterationCounts(8, 2);
    m_impl->scene->addActor(*actor);
    const BodyHandle handle = m_impl->nextHandle++;
    {
        std::lock_guard<std::mutex> snapshotGuard(m_impl->snapshotMutex);
        m_impl->bodies.push_back({handle, actor, worldPose});
    }
    if (errorMessage) errorMessage->clear();
    return handle;
#endif
}

ConveyorPhysics::BodyHandle ConveyorPhysics::addConvex(
    const CadTransform& worldPose, const std::vector<CadVec3>& verticesMm, double densityKgM3,
    std::string* errorMessage) {
#ifndef ROBOTSIM_WITH_PHYSX
    (void)worldPose; (void)verticesMm; (void)densityKgM3;
    if (errorMessage) *errorMessage = "PhysX convex bodies are unavailable in this build.";
    return 0;
#else
    m_impl->sceneMutationWaiters.fetch_add(1);
    std::unique_lock<std::mutex> sceneGuard(m_impl->sceneMutex);
    m_impl->sceneMutationWaiters.fetch_sub(1);
    if (!m_impl->scene || !m_impl->physics || !m_impl->productMaterial) {
        if (errorMessage) *errorMessage = "PhysX conveyor scene is not initialized.";
        return 0;
    }
    if (verticesMm.size() < 4) {
        if (errorMessage) *errorMessage = "A convex body requires at least four vertices.";
        return 0;
    }
    std::vector<PxVec3> vertices;
    vertices.reserve(verticesMm.size());
    for (const CadVec3& vertex : verticesMm) {
        vertices.emplace_back(static_cast<float>(vertex.x) * kMmToM,
                              static_cast<float>(vertex.y) * kMmToM,
                              static_cast<float>(vertex.z) * kMmToM);
    }
    PxConvexMeshDesc descriptor;
    descriptor.points.count = static_cast<PxU32>(vertices.size());
    descriptor.points.stride = sizeof(PxVec3);
    descriptor.points.data = vertices.data();
    descriptor.flags = PxConvexFlag::eCOMPUTE_CONVEX;
    PxDefaultMemoryOutputStream cooked;
    const PxCookingParams cookingParams(m_impl->physics->getTolerancesScale());
    if (!PxCookConvexMesh(cookingParams, descriptor, cooked)) {
        if (errorMessage) *errorMessage = "PhysX could not cook the convex body.";
        return 0;
    }
    PxDefaultMemoryInputData input(cooked.getData(), cooked.getSize());
    PxConvexMesh* convex = m_impl->physics->createConvexMesh(input);
    if (!convex) {
        if (errorMessage) *errorMessage = "PhysX could not load the convex body.";
        return 0;
    }
    PxRigidDynamic* actor = m_impl->physics->createRigidDynamic(toPxTransform(worldPose));
    PxShape* shape = actor
        ? m_impl->physics->createShape(PxConvexMeshGeometry(convex), *m_impl->productMaterial, true)
        : nullptr;
    convex->release();
    if (!actor || !shape) {
        if (actor) actor->release();
        if (errorMessage) *errorMessage = "PhysX could not create the convex body.";
        return 0;
    }
    shape->setContactOffset(0.002f);
    PxFilterData productFilter;
    productFilter.word0 = kProductFilter;
    shape->setSimulationFilterData(productFilter);
    actor->attachShape(*shape);
    shape->release();
    PxRigidBodyExt::updateMassAndInertia(*actor, static_cast<float>(std::max(1.0, densityKgM3)));
    actor->setLinearDamping(0.15f);
    actor->setAngularDamping(0.8f);
    actor->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, true);
    actor->setSolverIterationCounts(4, 1);
    m_impl->scene->addActor(*actor);
    const BodyHandle handle = m_impl->nextHandle++;
    {
        std::lock_guard<std::mutex> snapshotGuard(m_impl->snapshotMutex);
        m_impl->bodies.push_back({handle, actor, worldPose});
    }
    if (errorMessage) errorMessage->clear();
    return handle;
#endif
}

ConveyorPhysics::BodyHandle ConveyorPhysics::addKinematicConvex(
    const CadTransform& worldPose, const std::vector<CadVec3>& verticesMm,
    std::string* errorMessage) {
#ifndef ROBOTSIM_WITH_PHYSX
    (void)worldPose; (void)verticesMm;
    if (errorMessage) *errorMessage = "PhysX mechanism contacts are unavailable in this build.";
    return 0;
#else
    m_impl->sceneMutationWaiters.fetch_add(1);
    std::unique_lock<std::mutex> sceneGuard(m_impl->sceneMutex);
    m_impl->sceneMutationWaiters.fetch_sub(1);
    if (!m_impl->scene || !m_impl->physics || !m_impl->gripperMaterial) {
        if (errorMessage) *errorMessage = "PhysX conveyor scene is not initialized.";
        return 0;
    }
    if (verticesMm.size() < 4) {
        if (errorMessage) *errorMessage = "Mechanism convex contact requires at least four vertices.";
        return 0;
    }
    std::vector<PxVec3> vertices;
    vertices.reserve(verticesMm.size());
    for (const CadVec3& vertex : verticesMm) {
        vertices.emplace_back(static_cast<float>(vertex.x) * kMmToM,
                              static_cast<float>(vertex.y) * kMmToM,
                              static_cast<float>(vertex.z) * kMmToM);
    }
    PxConvexMeshDesc descriptor;
    descriptor.points.count = static_cast<PxU32>(vertices.size());
    descriptor.points.stride = sizeof(PxVec3);
    descriptor.points.data = vertices.data();
    descriptor.flags = PxConvexFlag::eCOMPUTE_CONVEX;
    PxDefaultMemoryOutputStream cooked;
    const PxCookingParams cookingParams(m_impl->physics->getTolerancesScale());
    if (!PxCookConvexMesh(cookingParams, descriptor, cooked)) {
        if (errorMessage) *errorMessage = "PhysX could not cook the mechanism contact convex.";
        return 0;
    }
    PxDefaultMemoryInputData input(cooked.getData(), cooked.getSize());
    PxConvexMesh* convex = m_impl->physics->createConvexMesh(input);
    if (!convex) {
        if (errorMessage) *errorMessage = "PhysX could not load the mechanism contact convex.";
        return 0;
    }
    PxRigidDynamic* actor = m_impl->physics->createRigidDynamic(toPxTransform(worldPose));
    PxShape* shape = actor
        ? m_impl->physics->createShape(PxConvexMeshGeometry(convex), *m_impl->gripperMaterial, true)
        : nullptr;
    convex->release();
    if (!actor || !shape) {
        if (actor) actor->release();
        if (errorMessage) *errorMessage = "PhysX could not create the mechanism contact body.";
        return 0;
    }
    shape->setContactOffset(0.001f);
    actor->attachShape(*shape);
    shape->release();
    actor->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
    actor->setSolverIterationCounts(8, 2);
    m_impl->scene->addActor(*actor);
    const BodyHandle handle = m_impl->nextHandle++;
    {
        std::lock_guard<std::mutex> snapshotGuard(m_impl->snapshotMutex);
        m_impl->bodies.push_back({handle, actor, worldPose});
    }
    if (errorMessage) errorMessage->clear();
    return handle;
#endif
}

bool ConveyorPhysics::setKinematicPose(BodyHandle body, const CadTransform& worldPose) {
#ifdef ROBOTSIM_WITH_PHYSX
    m_impl->sceneMutationWaiters.fetch_add(1);
    std::unique_lock<std::mutex> sceneGuard(m_impl->sceneMutex);
    m_impl->sceneMutationWaiters.fetch_sub(1);
    std::lock_guard<std::mutex> snapshotGuard(m_impl->snapshotMutex);
    Impl::Body* found = m_impl->find(body);
    if (!found || !found->actor ||
        !(found->actor->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC)) return false;
    found->actor->setKinematicTarget(toPxTransform(worldPose));
    found->cachedPose = worldPose;
    return true;
#else
    (void)body; (void)worldPose;
    return false;
#endif
}

bool ConveyorPhysics::applyGripForce(BodyHandle body, const CadTransform& targetWorldPose,
                                     const CadVec3& targetLinearVelocityMmS,
                                     double stiffnessNPerM, double dampingNsPerM,
                                     double normalEffortN) {
#ifdef ROBOTSIM_WITH_PHYSX
    std::lock_guard<std::mutex> sceneGuard(m_impl->sceneMutex);
    Impl::Body* found = m_impl->find(body);
    if (!found || !found->actor ||
        (found->actor->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC)) return false;
    const auto existing = std::find_if(m_impl->gripDrives.begin(), m_impl->gripDrives.end(),
                                       [body](const Impl::GripDrive& drive) {
                                           return drive.body == body;
                                       });
    Impl::GripDrive drive;
    drive.body = body;
    drive.targetM = PxVec3(static_cast<float>(targetWorldPose.values[3]) * kMmToM,
                           static_cast<float>(targetWorldPose.values[7]) * kMmToM,
                           static_cast<float>(targetWorldPose.values[11]) * kMmToM);
    drive.targetVelocityMps = PxVec3(
        static_cast<float>(targetLinearVelocityMmS.x) * kMmToM,
        static_cast<float>(targetLinearVelocityMmS.y) * kMmToM,
        static_cast<float>(targetLinearVelocityMmS.z) * kMmToM);
    drive.targetOrientation = toPxTransform(targetWorldPose).q;
    drive.stiffnessNPerM = static_cast<float>(std::max(0.0, stiffnessNPerM));
    drive.dampingNsPerM = static_cast<float>(std::max(0.0, dampingNsPerM));
    // The actuator stores one normal-effort limit. The carrying-force limit follows directly from
    // the same pad material used by the jaw contacts, so the coefficient is not duplicated in tool
    // assets or mechanism runtime code.
    drive.maximumForceN = static_cast<float>(std::max(0.0, normalEffortN)) *
        kGripperDynamicFriction;
    if (existing == m_impl->gripDrives.end()) m_impl->gripDrives.push_back(drive);
    else *existing = drive;
    return true;
#else
    (void)body; (void)targetWorldPose; (void)targetLinearVelocityMmS;
    (void)stiffnessNPerM; (void)dampingNsPerM;
    (void)normalEffortN;
    return false;
#endif
}

void ConveyorPhysics::clearGripForce(BodyHandle body) {
#ifdef ROBOTSIM_WITH_PHYSX
    std::lock_guard<std::mutex> sceneGuard(m_impl->sceneMutex);
    m_impl->gripDrives.erase(
        std::remove_if(m_impl->gripDrives.begin(), m_impl->gripDrives.end(),
                       [body](const Impl::GripDrive& drive) { return drive.body == body; }),
        m_impl->gripDrives.end());
#else
    (void)body;
#endif
}

bool ConveyorPhysics::bodyPose(BodyHandle body, CadTransform* worldPose) const {
#ifdef ROBOTSIM_WITH_PHYSX
    if (!worldPose) return false;
    std::lock_guard<std::mutex> snapshotGuard(m_impl->snapshotMutex);
    const Impl::Body* found = m_impl->find(body);
    if (!found || !found->actor) return false;
    *worldPose = found->cachedPose;
    return true;
#else
    (void)body; (void)worldPose;
    return false;
#endif
}

void ConveyorPhysics::removeBody(BodyHandle body) {
#ifdef ROBOTSIM_WITH_PHYSX
    m_impl->sceneMutationWaiters.fetch_add(1);
    std::unique_lock<std::mutex> sceneGuard(m_impl->sceneMutex);
    m_impl->sceneMutationWaiters.fetch_sub(1);
    m_impl->gripDrives.erase(
        std::remove_if(m_impl->gripDrives.begin(), m_impl->gripDrives.end(),
                       [body](const Impl::GripDrive& drive) { return drive.body == body; }),
        m_impl->gripDrives.end());
    std::lock_guard<std::mutex> snapshotGuard(m_impl->snapshotMutex);
    const auto found = std::find_if(m_impl->bodies.begin(), m_impl->bodies.end(),
                                    [body](const Impl::Body& entry) {
                                        return entry.handle == body;
                                    });
    if (found == m_impl->bodies.end()) return;
    if (found->actor) found->actor->release();
    m_impl->bodies.erase(found);
#else
    (void)body;
#endif
}

void ConveyorPhysics::step(double elapsedSeconds) {
#ifdef ROBOTSIM_WITH_PHYSX
    if (!m_impl->scene || m_impl->workerActive.load()) return;
    std::lock_guard<std::mutex> sceneGuard(m_impl->sceneMutex);
    m_impl->accumulator += std::max(0.0, std::min(0.05, elapsedSeconds));
    constexpr double fixedStep = ConveyorPhysics::fixedStepSeconds();
    int steps = 0;
    while (m_impl->accumulator >= fixedStep && steps++ < 6) {
        m_impl->simulateFixedStepLocked(fixedStep);
        m_impl->accumulator -= fixedStep;
    }
#else
    (void)elapsedSeconds;
#endif
}

void ConveyorPhysics::startAsync(double speedFactor) {
#ifdef ROBOTSIM_WITH_PHYSX
    m_impl->startWorker(speedFactor);
#else
    (void)speedFactor;
#endif
}

void ConveyorPhysics::stopAsync() {
#ifdef ROBOTSIM_WITH_PHYSX
    m_impl->stopWorker();
#endif
}

void ConveyorPhysics::setAsyncPaused(bool paused) {
#ifdef ROBOTSIM_WITH_PHYSX
    m_impl->workerPaused.store(paused);
#else
    (void)paused;
#endif
}

void ConveyorPhysics::setAsyncSpeedFactor(double speedFactor) {
#ifdef ROBOTSIM_WITH_PHYSX
    m_impl->workerSpeedFactor.store(std::max(0.01, speedFactor));
#else
    (void)speedFactor;
#endif
}

uint64_t ConveyorPhysics::consumeCompletedAsyncSteps() {
#ifdef ROBOTSIM_WITH_PHYSX
    return m_impl->completedSteps.exchange(0);
#else
    return 0;
#endif
}

bool ConveyorPhysics::asyncActive() const {
#ifdef ROBOTSIM_WITH_PHYSX
    return m_impl->workerActive.load();
#else
    return false;
#endif
}

bool ConveyorPhysics::asyncAchievedRate(double* outRate) const {
#ifdef ROBOTSIM_WITH_PHYSX
    if (!outRate || !m_impl->achievedRateValid.load()) return false;
    *outRate = m_impl->achievedRate.load();
    return true;
#else
    (void)outRate;
    return false;
#endif
}

double ConveyorPhysics::asyncDroppedSimSeconds() const {
#ifdef ROBOTSIM_WITH_PHYSX
    return m_impl->droppedSimSeconds.load();
#else
    return 0.0;
#endif
}

ConveyorPhysics::RuntimeStats ConveyorPhysics::runtimeStats() const {
    RuntimeStats result;
#ifdef ROBOTSIM_WITH_PHYSX
    result.staticActors = m_impl->staticActorCount.load();
    result.activeDynamicBodies = m_impl->activeDynamicBodyCount.load();
    result.contactPairs = m_impl->contactPairCount.load();
    result.convexBoxPairs = m_impl->convexBoxPairCount.load();
    result.convexTrianglePairs = m_impl->convexTrianglePairCount.load();
    result.convexConvexPairs = m_impl->convexConvexPairCount.load();
    result.dispatcherThreads = m_impl->dispatcherThreadCount.load();
    result.solverBatchSize = m_impl->solverBatchSize.load();
    result.averageWakeMs = m_impl->averageWakeMs.load();
    result.averageSimulateMs = m_impl->averageSimulateMs.load();
    result.averageSnapshotMs = m_impl->averageSnapshotMs.load();
    result.minimumBodyYmm = m_impl->minimumBodyYmm.load();
    std::lock_guard<std::mutex> snapshotGuard(m_impl->snapshotMutex);
    result.dynamicBodies = static_cast<uint32_t>(m_impl->bodies.size());
#endif
    return result;
}

bool ConveyorPhysics::isActive() const {
#ifdef ROBOTSIM_WITH_PHYSX
    return m_impl && m_impl->scene;
#else
    return false;
#endif
}
