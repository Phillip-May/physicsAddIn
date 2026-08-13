#include "DragChainPhysics.h"

#include "../Common/CadNode.h"

#include <algorithm>
#include <cmath>
#include <vector>

#ifdef ROBOTSIM_WITH_PHYSX
#include "PxCadTransform.h"

#include <PxPhysicsAPI.h>

using namespace physx;

namespace {

PxFilterFlags chainFilterShader(PxFilterObjectAttributes, PxFilterData data0,
                                PxFilterObjectAttributes, PxFilterData data1,
                                PxPairFlags& pairFlags, const void*, PxU32) {
    constexpr PxU32 kBaseCollisionCategory = 1u;
    constexpr PxU32 kIgnoreBaseCollision = 1u;
    // word0 is a one-based link index. Neighbours share a pin and must not collide; all other
    // members retain self-collision so the returning run cannot pass through the outgoing run.
    if (data0.word0 && data1.word0 &&
        std::abs(static_cast<int>(data0.word0) - static_cast<int>(data1.word0)) <= 1) {
        return PxFilterFlag::eSUPPRESS;
    }
    // The three members entering the fixed connector occupy the bracket represented by the base
    // decomposition, so their box proxies overlap that static hull by design. Suppress only those
    // connector-region pairs; every free member continues to collide with the base.
    if (((data0.word1 & kBaseCollisionCategory) && (data1.word2 & kIgnoreBaseCollision)) ||
        ((data1.word1 & kBaseCollisionCategory) && (data0.word2 & kIgnoreBaseCollision))) {
        return PxFilterFlag::eSUPPRESS;
    }
    pairFlags = PxPairFlag::eCONTACT_DEFAULT;
    return PxFilterFlag::eDEFAULT;
}
}

struct DragChainPhysics::Impl {
    PxDefaultAllocator allocator;
    PxDefaultErrorCallback errors;
    PxFoundation* foundation = nullptr;
    PxPhysics* physics = nullptr;
    PxDefaultCpuDispatcher* dispatcher = nullptr;
    PxScene* scene = nullptr;
    PxMaterial* material = nullptr;
    PxRigidStatic* baseCollision = nullptr;
    PxRigidStatic* fixedAnchor = nullptr;
    PxRigidDynamic* movingAnchor = nullptr;
    std::vector<PxRigidDynamic*> bodies;
    std::vector<PxRevoluteJoint*> joints;
    CadNode* chainNode = nullptr;
    DragChainMechanismData* data = nullptr;
    double accumulator = 0.0;
    bool active = false;
    bool simulated = false;
    bool extensionsOpen = false;
    size_t baseCollisionHulls = 0;

    ~Impl() { release(); }

    void release() {
        for (PxRevoluteJoint* joint : joints) if (joint) joint->release();
        joints.clear();
        if (scene) scene->release();
        scene = nullptr;
        if (dispatcher) dispatcher->release();
        dispatcher = nullptr;
        if (material) material->release();
        material = nullptr;
        if (extensionsOpen) PxCloseExtensions();
        extensionsOpen = false;
        if (physics) physics->release();
        physics = nullptr;
        if (foundation) foundation->release();
        foundation = nullptr;
        bodies.clear();
        baseCollision = nullptr;
        baseCollisionHulls = 0;
        fixedAnchor = nullptr;
        movingAnchor = nullptr;
        active = false;
        simulated = false;
    }

    bool create(std::string* errorMessage) {
        foundation = PxCreateFoundation(PX_PHYSICS_VERSION, allocator, errors);
        if (!foundation) return fail("PhysX foundation creation failed.", errorMessage);
        PxTolerancesScale scale;
        scale.length = 1.0f;
        scale.speed = 10.0f;
        physics = PxCreatePhysics(PX_PHYSICS_VERSION, *foundation, scale, false, nullptr);
        if (!physics) return fail("PhysX core creation failed.", errorMessage);
        extensionsOpen = PxInitExtensions(*physics, nullptr);
        if (!extensionsOpen) return fail("PhysX extensions initialization failed.", errorMessage);

        PxSceneDesc desc(scale);
        desc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
        desc.solverType = PxSolverType::eTGS;
        dispatcher = PxDefaultCpuDispatcherCreate(2);
        desc.cpuDispatcher = dispatcher;
        desc.filterShader = chainFilterShader;
        scene = physics->createScene(desc);
        if (!scene) return fail("PhysX scene creation failed.", errorMessage);
        material = physics->createMaterial(0.55f, 0.45f, 0.05f);
        return material != nullptr || fail("PhysX drag-chain material creation failed.", errorMessage);
    }

    bool fail(const char* message, std::string* errorMessage) {
        if (errorMessage) *errorMessage = message;
        return false;
    }

    bool feasible() const {
        if (!data || data->linkFrames.size() < 2 || !data->linkFrames.front()) return false;
        const CadTransform& moving = data->linkFrames.front()->loc;
        const double dx = data->fixedAnchorMm.x - moving.values[3];
        const double dy = data->fixedAnchorMm.y - moving.values[7];
        const double dz = data->fixedAnchorMm.z - moving.values[11];
        const double reach = data->linkFrames.size() * data->pitchMm;
        return dx * dx + dy * dy + dz * dz <= reach * reach * 1.0001;
    }

    void reseed() {
        for (size_t i = 0; i < bodies.size(); ++i) {
            bodies[i]->setGlobalPose(toPxTransform(data->linkFrames[i]->loc));
            bodies[i]->setLinearVelocity(PxVec3(0.0f));
            bodies[i]->setAngularVelocity(PxVec3(0.0f));
            bodies[i]->wakeUp();
        }
        movingAnchor->setGlobalPose(toPxTransform(data->linkFrames.front()->loc));
        accumulator = 0.0;
    }
};

#else
struct DragChainPhysics::Impl {
    bool active = false;
    bool simulated = false;
};
#endif

DragChainPhysics::DragChainPhysics() : m_impl(std::make_unique<Impl>()) {}
DragChainPhysics::~DragChainPhysics() = default;
DragChainPhysics::DragChainPhysics(DragChainPhysics&&) noexcept = default;
DragChainPhysics& DragChainPhysics::operator=(DragChainPhysics&&) noexcept = default;

bool DragChainPhysics::bind(CadNode* chainNode, std::string* errorMessage) {
#ifndef ROBOTSIM_WITH_PHYSX
    (void)chainNode;
    if (errorMessage) errorMessage->clear();
    return true;
#else
    m_impl->release();
    m_impl->chainNode = chainNode;
    m_impl->data = chainNode ? chainNode->asDragChainMechanism() : nullptr;
    if (!m_impl->data || !m_impl->data->prototypeGeometry ||
        m_impl->data->linkFrames.size() < 2) {
        return m_impl->fail("Drag chain cannot bind PhysX: mechanism references are incomplete.", errorMessage);
    }
    const MeshGeometryData* mesh = m_impl->data->prototypeGeometry->asMeshGeometry();
    if (!mesh || !mesh->loaded) {
        return m_impl->fail("Drag chain cannot bind PhysX: prototype mesh is not loaded.", errorMessage);
    }
    GantryMechanismData* gantryData = nullptr;
    for (CadNode* parent = chainNode ? chainNode->parent : nullptr;
         parent && !gantryData; parent = parent->parent) {
        gantryData = parent->asGantryMechanism();
    }
    if (!m_impl->data->physicsEnabled) {
        m_impl->baseCollisionHulls = gantryData ? gantryData->baseCollisionHulls.size() : 0;
        m_impl->active = true;
        m_impl->simulated = false;
        if (errorMessage) errorMessage->clear();
        return true;
    }
    if (!m_impl->create(errorMessage)) return false;
    m_impl->simulated = true;
    if (gantryData &&
        gantryData->baseCollisionHulls.size() > GantryMechanismData::kMaxBaseCollisionHulls) {
        return m_impl->fail("Drag chain base collision exceeds the 128-hull limit.", errorMessage);
    }
    if (gantryData && !gantryData->baseCollisionHulls.empty()) {
        m_impl->baseCollision = m_impl->physics->createRigidStatic(PxTransform(PxIdentity));
        if (!m_impl->baseCollision) {
            return m_impl->fail("Drag chain could not create the static base collision actor.",
                                errorMessage);
        }
        const PxCookingParams cookingParams(m_impl->physics->getTolerancesScale());
        for (const ConvexHullData& hull : gantryData->baseCollisionHulls) {
            if (hull.vertices.size() < 4 || hull.vertices.size() > 255 || hull.indices.empty()) {
                return m_impl->fail("Drag chain base contains a malformed convex hull.",
                                    errorMessage);
            }
            std::vector<PxVec3> vertices;
            vertices.reserve(hull.vertices.size());
            for (const auto& vertex : hull.vertices) {
                vertices.emplace_back(static_cast<float>(vertex[0]) * kMmToM,
                                      static_cast<float>(vertex[1]) * kMmToM,
                                      static_cast<float>(vertex[2]) * kMmToM);
            }
            PxConvexMeshDesc descriptor;
            descriptor.points.count = static_cast<PxU32>(vertices.size());
            descriptor.points.stride = sizeof(PxVec3);
            descriptor.points.data = vertices.data();
            descriptor.flags = PxConvexFlag::eCOMPUTE_CONVEX;
            PxDefaultMemoryOutputStream cooked;
            if (!PxCookConvexMesh(cookingParams, descriptor, cooked)) {
                return m_impl->fail("PhysX could not cook a serialized base collision hull.",
                                    errorMessage);
            }
            PxDefaultMemoryInputData input(cooked.getData(), cooked.getSize());
            PxConvexMesh* convexMesh = m_impl->physics->createConvexMesh(input);
            if (!convexMesh) {
                return m_impl->fail("PhysX could not load a cooked base collision hull.",
                                    errorMessage);
            }
            PxShape* shape = m_impl->physics->createShape(
                PxConvexMeshGeometry(convexMesh), *m_impl->material, true);
            convexMesh->release();
            if (!shape) {
                return m_impl->fail("PhysX could not create a base collision shape.", errorMessage);
            }
            shape->setContactOffset(0.002f);
            shape->setRestOffset(0.0f);
            PxFilterData baseFilter;
            baseFilter.word1 = 1u;
            shape->setSimulationFilterData(baseFilter);
            m_impl->baseCollision->attachShape(*shape);
            shape->release();
            ++m_impl->baseCollisionHulls;
        }
        m_impl->scene->addActor(*m_impl->baseCollision);
    }

    const float pitch = static_cast<float>(m_impl->data->pitchMm) * kMmToM;
    const float xmin = mesh->bounds[0] * kMmToM;
    const float ymin = mesh->bounds[1] * kMmToM;
    const float zmin = mesh->bounds[2] * kMmToM;
    const float xmax = mesh->bounds[3] * kMmToM;
    const float ymax = mesh->bounds[4] * kMmToM;
    const float zmax = mesh->bounds[5] * kMmToM;
    const PxVec3 center(0.5f * (xmin + xmax), 0.5f * (ymin + ymax), 0.5f * (zmin + zmax));
    const PxVec3 halfExtents(std::max(0.002f, std::min(0.44f * pitch, 0.48f * (xmax - xmin))),
                             std::max(0.002f, 0.45f * (ymax - ymin)),
                             std::max(0.002f, 0.45f * (zmax - zmin)));

    for (size_t index = 0; index < m_impl->data->linkFrames.size(); ++index) {
        PxRigidDynamic* body = m_impl->physics->createRigidDynamic(
            toPxTransform(m_impl->data->linkFrames[index]->loc));
        PxShape* shape = m_impl->physics->createShape(PxBoxGeometry(halfExtents), *m_impl->material, true);
        shape->setLocalPose(PxTransform(center));
        shape->setContactOffset(0.002f);
        shape->setRestOffset(0.0f);
        PxFilterData filter;
        filter.word0 = static_cast<PxU32>(index + 1);
        filter.word2 = index + 3 >= m_impl->data->linkFrames.size() ? 1u : 0u;
        shape->setSimulationFilterData(filter);
        body->attachShape(*shape);
        shape->release();
        PxRigidBodyExt::setMassAndUpdateInertia(*body, static_cast<float>(m_impl->data->linkMassKg));
        body->setLinearDamping(0.8f);
        body->setAngularDamping(1.2f);
        // A cable carrier has a long, highly coupled constraint chain. Higher iteration counts keep
        // the terminal load from accumulating angular error across the members and pushing an early
        // hinge past its configured hard stop.
        body->setSolverIterationCounts(255, 64);
        m_impl->scene->addActor(*body);
        m_impl->bodies.push_back(body);
    }

    m_impl->movingAnchor = m_impl->physics->createRigidDynamic(
        toPxTransform(m_impl->data->linkFrames.front()->loc));
    m_impl->movingAnchor->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
    m_impl->scene->addActor(*m_impl->movingAnchor);
    const PxTransform fixedEndPose = toPxTransform(m_impl->data->linkFrames.back()->loc);
    const PxVec3 fixedEndOffsetWorld(
        static_cast<float>(m_impl->data->fixedEndMemberOffsetMm.x) * kMmToM,
        static_cast<float>(m_impl->data->fixedEndMemberOffsetMm.y) * kMmToM,
        static_cast<float>(m_impl->data->fixedEndMemberOffsetMm.z) * kMmToM);
    const PxVec3 fixedEndOffsetLocal = fixedEndPose.q.rotateInv(fixedEndOffsetWorld);
    PxVec3 fixedBodyHinge(m_impl->data->reverseFixedEndMember ? 0.0f : pitch, 0.0f, 0.0f);
    fixedBodyHinge -= fixedEndOffsetLocal;
    const PxTransform fixedAnchorPose = fixedEndPose * PxTransform(fixedBodyHinge);
    m_impl->fixedAnchor = m_impl->physics->createRigidStatic(fixedAnchorPose);
    m_impl->scene->addActor(*m_impl->fixedAnchor);

    // Converted member geometry hinges about local -Z. A +90 degree rotation about local Y maps
    // the PhysX revolute joint's local +X axis onto that direction.
    const PxQuat hingeFrame(PxHalfPi, PxVec3(0.0f, 1.0f, 0.0f));
    // The fixed-end member is stored facing back into the chain so its local-zero mesh end can sit
    // on the fixed connector. Rotate that member's joint frame half a turn around the hinge axis;
    // otherwise PhysX calls a folded-back terminal pair "zero" and limits the wrong angle.
    const PxQuat reversedMemberHingeFrame =
        hingeFrame * PxQuat(PxPi, PxVec3(1.0f, 0.0f, 0.0f));
    const float configuredMaxJointRotation = static_cast<float>(
        m_impl->data->maxJointRotationDeg * 3.14159265358979323846 / 180.0);
    const float solverGuard = std::min(
        configuredMaxJointRotation * 0.2f,
        static_cast<float>(3.0 * 3.14159265358979323846 / 180.0));
    const float maxJointRotation = std::max(1.0e-4f,
                                            configuredMaxJointRotation - solverGuard);
    const auto addJoint = [&](PxRigidActor* a, const PxTransform& aFrame,
                              PxRigidActor* b, const PxTransform& bFrame) {
        PxRevoluteJoint* joint = PxRevoluteJointCreate(*m_impl->physics, a, aFrame, b, bFrame);
        joint->setLimit(PxJointAngularLimitPair(-maxJointRotation, maxJointRotation));
        joint->setRevoluteJointFlag(PxRevoluteJointFlag::eLIMIT_ENABLED, true);
        m_impl->joints.push_back(joint);
    };
    addJoint(m_impl->movingAnchor, PxTransform(PxVec3(0.0f), hingeFrame),
             m_impl->bodies.front(), PxTransform(PxVec3(0.0f), hingeFrame));
    for (size_t index = 1; index < m_impl->bodies.size(); ++index) {
        const bool reversedTerminal = m_impl->data->reverseFixedEndMember &&
                                      index + 1 == m_impl->bodies.size();
        const bool terminal = index + 1 == m_impl->bodies.size();
        PxVec3 currentHinge(reversedTerminal ? pitch : 0.0f, 0.0f, 0.0f);
        if (terminal) currentHinge -= fixedEndOffsetLocal;
        addJoint(m_impl->bodies[index - 1], PxTransform(PxVec3(pitch, 0.0f, 0.0f), hingeFrame),
                 m_impl->bodies[index],
                 PxTransform(currentHinge,
                             reversedTerminal ? reversedMemberHingeFrame : hingeFrame));
    }
    addJoint(m_impl->bodies.back(), PxTransform(fixedBodyHinge, hingeFrame),
             m_impl->fixedAnchor, PxTransform(PxVec3(0.0f), hingeFrame));

    m_impl->active = m_impl->feasible();
    if (errorMessage) errorMessage->clear();
    return true;
#endif
}

void DragChainPhysics::step(double elapsedSeconds) {
#ifdef ROBOTSIM_WITH_PHYSX
    if (!m_impl->scene || !m_impl->data || m_impl->bodies.empty()) return;
    const bool canReach = m_impl->feasible();
    if (!canReach) {
        m_impl->active = false;
        return;
    }
    if (!m_impl->active) {
        m_impl->reseed();
        m_impl->active = true;
    }
    const PxTransform movingTarget = toPxTransform(m_impl->data->linkFrames.front()->loc);
    const PxTransform movingStart = m_impl->movingAnchor->getGlobalPose();
    const float movingDistance = (movingTarget.p - movingStart.p).magnitude();
    // A gantry slider can move hundreds of millimetres in one UI frame. Sending that as one
    // kinematic teleport lets the position constraints win before the angular limits propagate
    // through the carrier, briefly folding a member past its stop. Advance large edits through
    // small internal targets and expose only the final constrained result to the renderer.
    const float maxTravelPerStep = std::max(
        0.002f,
        std::min(0.010f, static_cast<float>(m_impl->data->pitchMm) * kMmToM * 0.05f));
    const int travelSteps = std::max(
        1, std::min(512, static_cast<int>(std::ceil(movingDistance / maxTravelPerStep))));
    if (travelSteps > 1) {
        for (int travelStep = 1; travelStep <= travelSteps; ++travelStep) {
            const float fraction = static_cast<float>(travelStep) /
                                   static_cast<float>(travelSteps);
            const PxVec3 position = movingStart.p +
                                    (movingTarget.p - movingStart.p) * fraction;
            m_impl->movingAnchor->setKinematicTarget(PxTransform(position, movingTarget.q));
            m_impl->scene->simulate(1.0f / 120.0f);
            m_impl->scene->fetchResults(true);
        }
        m_impl->accumulator = 0.0;
    } else {
        m_impl->movingAnchor->setKinematicTarget(movingTarget);
    }
    m_impl->accumulator += std::max(0.0, std::min(elapsedSeconds, 0.05));
    constexpr double fixedStep = 1.0 / 120.0;
    int steps = 0;
    while (m_impl->accumulator >= fixedStep && steps++ < 6) {
        m_impl->scene->simulate(static_cast<float>(fixedStep));
        m_impl->scene->fetchResults(true);
        m_impl->accumulator -= fixedStep;
    }
    for (size_t i = 0; i < m_impl->bodies.size(); ++i) {
        m_impl->data->linkFrames[i]->loc = fromPxTransform(m_impl->bodies[i]->getGlobalPose());
        m_impl->data->linkFrames[i]->needsGlobalLocUpdate = true;
    }
#else
    (void)elapsedSeconds;
#endif
}

bool DragChainPhysics::isActive() const { return m_impl && m_impl->active; }

bool DragChainPhysics::isSimulated() const { return m_impl && m_impl->simulated; }

double DragChainPhysics::maxAbsJointRotationDeg(size_t* jointIndex) const {
    if (jointIndex) *jointIndex = 0;
#ifdef ROBOTSIM_WITH_PHYSX
    if (!m_impl) return 0.0;
    double maximum = 0.0;
    for (size_t index = 0; index < m_impl->joints.size(); ++index) {
        const PxRevoluteJoint* joint = m_impl->joints[index];
        if (!joint) continue;
        const double angleRadians = std::remainder(
            static_cast<double>(joint->getAngle()), 2.0 * 3.14159265358979323846);
        const double angle = std::abs(angleRadians) * 180.0 / 3.14159265358979323846;
        if (angle > maximum) {
            maximum = angle;
            if (jointIndex) *jointIndex = index;
        }
    }
    return maximum;
#else
    return 0.0;
#endif
}

size_t DragChainPhysics::baseCollisionHullCount() const {
#ifdef ROBOTSIM_WITH_PHYSX
    return m_impl ? m_impl->baseCollisionHulls : 0;
#else
    return 0;
#endif
}
