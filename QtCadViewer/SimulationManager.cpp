#include "SimulationManager.h"
#include <QPointer>
#include "MaterialEditorDialog.h"
#include <QToolBar>
#include <QComboBox>
#include <QAction>
#include <QMainWindow>
#include <QToolButton>
#include <QMenu>
#include <QElapsedTimer>
#include "ObjectPropertiesDialog.h"
#include <QTreeView>
#include <QMenu>
#include <QMessageBox>
#include <QModelIndex>
#include <QMetaMethod>
#include "CadNode.h"
#include "CustomModelTreeModel.h"
#include "CadTreeModel.h"

// OpenCASCADE includes for coordinate conversion
#include <gp_Trsf.hxx>
#include <gp_Pnt.hxx>
#include <gp_Mat.hxx>
#include <TopLoc_Location.hxx>

#define PVD_HOST "127.0.0.1"
#include "PxPhysicsAPI.h"
#include "extensions/PxDeformableVolumeExt.h"
#include "PxDeformableVolume.h"
#include "extensions/PxTetMakerExt.h"
#include "foundation/PxArray.h"
#include "geometry/PxSimpleTriangleMesh.h"
#include "extensions/PxTetMakerExt.h"
#include "extensions/PxRemeshingExt.h"
#include "extensions/PxCudaHelpersExt.h"
#include "extensions/PxTetrahedronMeshExt.h"


using namespace physx;

// PhysXEngine destructor
PhysXEngine::~PhysXEngine() {
    // Clean up PVD transport
    if (m_pvdTransport) {
        m_pvdTransport->release();
        m_pvdTransport = nullptr;
    }
    
    // Clean up other PhysX resources
    if (m_scene) {
        m_scene->release();
        m_scene = nullptr;
    }
    if (m_dispatcher) {
        //m_dispatcher->release();
        m_dispatcher = nullptr;
    }
    if (m_physics) {
        m_physics->release();
        m_physics = nullptr;
    }
    if (m_pvd) {
        m_pvd->release();
        m_pvd = nullptr;
    }
    if (m_foundation) {
        m_foundation->release();
        m_foundation = nullptr;
    }
}

// Coordinate system conversion functions
// PhysX uses Y-up, CAD viewer likely uses Z-up
PxVec3 convertCADToPhysX(const PxVec3& cadVertex)
{
    // Convert CAD coordinate system to PhysX coordinate system
    // CAD: (X,Y,Z) -> PhysX: (X,Z,Y)
    PxVec3 physx = PxVec3(cadVertex.x, cadVertex.z, cadVertex.y);
    return physx;
}

PxVec3 convertPhysXToCAD(const PxVec3& physxVertex)
{
    // Convert PhysX coordinate system to CAD coordinate system
    // PhysX: (X,Z,Y) -> CAD: (X,Y,Z)
    PxVec3 cad = PxVec3(physxVertex.x, physxVertex.z, physxVertex.y);
    return cad;
}

PxTransform convertCADPoseToPhysX(const PxTransform& cadPose)
{
    PxTransform physxPose;
    // Position: CAD (X,Y,Z) -> PhysX (X,Z,Y)
    physxPose.p = convertCADToPhysX(cadPose.p);

    // Convert rotation (quaternion to rotation matrix, apply coordinate transformation)
    PxMat33 cadRotMat(cadPose.q);
    
    // Create coordinate system transformation matrix: CAD (X,Y,Z) -> PhysX (X,Z,Y)
    PxMat33 coordTransform(
        PxVec3(1, 0, 0),  // X axis stays the same
        PxVec3(0, 0, 1),  // Y axis becomes Z axis
        PxVec3(0, 1, 0)   // Z axis becomes Y axis
    );

    // Apply transformation: physxRot = coordTransform * cadRot * coordTransform^T
    PxMat33 physxRotMat = coordTransform * cadRotMat * coordTransform.getTranspose();

    // Convert back to quaternion
    physxPose.q = PxQuat(physxRotMat);

    return physxPose;
}

PxTransform convertPhysXPoseToCAD(const PxTransform& physxPose)
{
    PxTransform cadPose;
    // Position: PhysX (X,Z,Y) -> CAD (X,Y,Z)
    cadPose.p = convertPhysXToCAD(physxPose.p);

    // Convert rotation (quaternion to rotation matrix, apply inverse coordinate transformation)
    PxMat33 physxRotMat(physxPose.q);
    
    // Create inverse coordinate system transformation matrix: PhysX (X,Z,Y) -> CAD (X,Y,Z)
    PxMat33 coordTransformInv(
        PxVec3(1, 0, 0),  // X axis stays the same
        PxVec3(0, 0, 1),  // Y axis becomes Z axis
        PxVec3(0, 1, 0)   // Z axis becomes Y axis
    );

    // Apply inverse transformation: cadRot = coordTransformInv * physxRot * coordTransformInv^T
    PxMat33 cadRotMat = coordTransformInv * physxRotMat * coordTransformInv.getTranspose();

    // Convert back to quaternion
    cadPose.q = PxQuat(cadRotMat);

    return cadPose;
}

PxFilterFlags PhysXEngine::simulationFilterShader (
    PxFilterObjectAttributes attributes0, PxFilterData filterData0,
    PxFilterObjectAttributes attributes1, PxFilterData filterData1,
    PxPairFlags& pairFlags, const void* constantBlock, PxU32 constantBlockSize)
{
    // Extract filter data components
    // word0: object type (0=regular, 1=robot, 2=soft body)
    // word1: attachment group (0=none, 1=group1, 2=group2, etc.)
    // word2: collision group (0=default, 1=group1, 2=group2, etc.)
    // word3: attachment surface ID (0=none, 1=surface1, 2=surface2, etc.)

    PxU32 objectType0 = filterData0.word0;
    PxU32 attachmentGroup0 = filterData0.word1;
    PxU32 collisionGroup0 = filterData0.word2;
    PxU32 attachmentSurface0 = filterData0.word3;

    PxU32 objectType1 = filterData1.word0;
    PxU32 attachmentGroup1 = filterData1.word1;
    PxU32 collisionGroup1 = filterData1.word2;
    PxU32 attachmentSurface1 = filterData1.word3;

    // Default behavior: allow all contacts
    pairFlags = PxPairFlag::eCONTACT_DEFAULT;

    // Special case: If both objects are in the same attachment group but different surfaces,
    // disable collision between them (soft body and its attachment surface)
    if (attachmentGroup0 != 0 && attachmentGroup0 == attachmentGroup1) {
        if (attachmentSurface0 != attachmentSurface1) {
            // Same attachment group but different surfaces - disable collision
            return PxFilterFlag::eKILL;
        }
    }

    // Special case: Soft body self-collision (already handled by flag, but double-check)
    if (objectType0 == 2 && objectType1 == 2) {
        // Soft body to soft body - use default behavior (self-collision disabled by flag)
        return PxFilterFlag::eDEFAULT;
    }

    // Special case: Soft body with its attachment surface
    if ((objectType0 == 2 && attachmentSurface1 != 0) ||
        (objectType1 == 2 && attachmentSurface0 != 0)) {
        // Check if this is the soft body's own attachment surface
        if (attachmentGroup0 != 0 && attachmentGroup0 == attachmentGroup1) {
            // This is the soft body's attachment surface - disable collision
            return PxFilterFlag::eKILL;
        }
    }

    return PxFilterFlag::eDEFAULT;
}

bool PhysXEngine::initializePhysX(const std::shared_ptr<CadNode>& rootNode)
{
    // Store reference to root node for ground plane configuration
    m_rootNode = rootNode;
    
    // Initialize PhysX engine with proper coordinate system
    m_foundation = PxCreateFoundation(PX_PHYSICS_VERSION, m_allocator, m_errorCallback);
    if (!m_foundation) {
        qDebug() << "Failed to create PhysX foundation";
        return false;
    }

    // Create PVD instance
    m_pvd = PxCreatePvd(*m_foundation);
    if (!m_pvd) {
        qDebug() << "Failed to create PVD instance";
        return false;
    }

    // Create PVD transport (socket-based) - keep it alive for PhysX 5.x
    m_pvdTransport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
    if (!m_pvdTransport) {
        qDebug() << "Failed to create PVD transport";
        return false;
    }

    // Connect PVD with full instrumentation
    bool pvdConnected = m_pvd->connect(*m_pvdTransport, PxPvdInstrumentationFlag::eALL);
    if (pvdConnected) {
        qDebug() << "PVD connected successfully to" << PVD_HOST << ":5425";
        qDebug() << "PVD will automatically transmit scene data during simulation";
    } else {
        qDebug() << "PVD connection failed - PVD may not be running";
        qDebug() << "To use PVD:";
        qDebug() << "1. Download PhysX Visual Debugger from NVIDIA";
        qDebug() << "2. Start PVD and listen on port 5425";
        qDebug() << "3. Use 'Connect PVD' menu option to reconnect";
        qDebug() << "Note: PVD will attempt auto-reconnection during simulation";
    }

    PxTolerancesScale scale;
    
    // Create physics with PVD enabled (works in both debug and release)
    m_physics = PxCreatePhysics(PX_PHYSICS_VERSION, *m_foundation, scale, true, m_pvd);
    if (!m_physics) {
        qDebug() << "Failed to create PhysX physics";
        return false;
    }
    
    qDebug() << "PhysX physics created successfully with PVD support";

    // Initialize gravity for PhysX (Y-up coordinate system) - using value from PhysXEngineOld.cpp
    // Gravity points down in Y direction in PhysX
    // This corresponds to gravity pointing down in Z direction in CAD coordinates
    m_gravity = PxVec3(0.0f, -9806.65f, 0.0f);  // Use mm/s² units like in PhysXEngineOld.cpp
    
    // Initialize other PhysX parameters - using values from PhysXEngineOld.cpp for better stability
    m_solverPositionIterations = 32;  // Increased from 4 to 32 for better contact resolution
    m_solverVelocityIterations = 32;  // Increased from 1 to 32 for better stability
    m_globalStaticFriction = 0.95f;   // Increased from 0.5f for better ground contact
    m_globalDynamicFriction = 0.90f;  // Increased from 0.3f for better ground contact
    m_globalRestitution = 0.1f;       // Kept the same
    m_pcmEnabled = true;              // Kept the same
    m_stabilizationEnabled = true;    // Kept the same
    m_contactOffset = 5.0f;           // Increased from 0.02f to 5.0f for better collision detection
    m_restOffset = 1.0f;              // Increased from 0.01f to 1.0f for better collision detection
    m_sleepThreshold = 10.0f;         // Increased from 0.01f to 10.0f for better stability
    m_stabilizationThreshold = 10.0f; // Increased from 0.01f to 10.0f for better stability
    m_ccdEnabled = true;              // Enable CCD for better collision detection and prevent tunneling
    m_wakeDistance = 1000.0f;         // Increased from 0.1f to 1000.0f for better wake behavior

    try {
        if (!PxInitExtensions(*m_physics, m_pvd)) {
            qDebug() << "Failed to initialize PhysX extensions";
            return false;
        }
    } catch (const std::exception& e) {
        qDebug() << "Exception during PhysX extensions initialization:" << e.what();
        return false;
    }

    PxCudaContextManagerDesc cudaContextManagerDesc;
    m_cudaContextManager = PxCreateCudaContextManager(*m_foundation, cudaContextManagerDesc, PxGetProfilerCallback());
    if (m_cudaContextManager && !m_cudaContextManager->contextIsValid()) {
        PX_RELEASE(m_cudaContextManager);
        qDebug() << "Failed to initialize cuda context.";
        return false;
    }

    // Use cooking parameters from PhysXEngineOld.cpp for better collision detection
    m_globalCookingParams = PxCookingParams(m_physics->getTolerancesScale());
    m_globalCookingParams.meshWeldTolerance = 0.001f;  // Much smaller tolerance for better precision
    m_globalCookingParams.meshPreprocessParams = PxMeshPreprocessingFlags(PxMeshPreprocessingFlag::eWELD_VERTICES);
    m_globalCookingParams.buildTriangleAdjacencies = false;
    m_globalCookingParams.buildGPUData = true;

    PxSceneDesc sceneDesc(m_physics->getTolerancesScale());
    sceneDesc.gravity = m_gravity;

    if (!sceneDesc.cudaContextManager)
        sceneDesc.cudaContextManager = m_cudaContextManager;

    sceneDesc.flags |= PxSceneFlag::eENABLE_GPU_DYNAMICS;
    sceneDesc.flags |= PxSceneFlag::eENABLE_PCM;

    if (m_stabilizationEnabled) {
        sceneDesc.flags |= PxSceneFlag::eENABLE_STABILIZATION;
    }

    // Enable CCD for better collision detection, but keep it disabled for individual actors
    // This allows the scene to handle CCD but we control it per-actor
    sceneDesc.flags |= PxSceneFlag::eENABLE_CCD;
    sceneDesc.broadPhaseType = PxBroadPhaseType::eGPU;
    sceneDesc.gpuMaxNumPartitions = 8;
    sceneDesc.solverType = PxSolverType::eTGS;

    PxU32 numCores = 2;
    m_dispatcher = PxDefaultCpuDispatcherCreate(numCores);
    sceneDesc.cpuDispatcher = m_dispatcher;
    sceneDesc.filterShader = simulationFilterShader;

    m_scene = m_physics->createScene(sceneDesc);
    if (!m_scene) {
        qDebug() << "Failed to create PhysX scene";
        return false;
    }

    // PVD scene client configuration for PhysX 5.x
    // In PhysX 5.x, PVD instrumentation is handled via PxPvdInstrumentationFlag during connection
    // and the scene automatically transmits data when PVD is connected
    qDebug() << "PVD scene client ready for PhysX 5.x";

    m_material = m_physics->createMaterial(m_globalStaticFriction, m_globalDynamicFriction, m_globalRestitution);
    if (!m_material) {
        qDebug() << "Failed to create PhysX material";
        return false;
    }

    // Create low-friction material for kinematic actors (robots)
    if (!createKinematicMaterial()) {
        qDebug() << "Failed to create kinematic material";
        return false;
    }

    if (!createGroundPlane()) {
        qDebug() << "Failed to create ground plane";
        return false;
    }

    return true;
}

bool PhysXEngine::createKinematicMaterial()
{
    // Create low-friction material for kinematic actors (robots)
    // Using sheet metal-like properties: very low friction for smooth surfaces
    m_kinematicMaterial = m_physics->createMaterial(0.15f, 0.12f, 0.05f);
    if (!m_kinematicMaterial) {
        qDebug() << "Failed to create kinematic material";
        return false;
    }

    qDebug() << "Created kinematic material with low friction (static: 0.15, dynamic: 0.12, restitution: 0.05)";
    return true;
}


bool PhysXEngine::createGroundPlane()
{
    // Get ground plane configuration from root node
    float groundPlaneY = -50.0f;  // Default value
    float groundPlaneSize = 10000.0f;  // Default value (same as PhysXEngineOld.cpp)
    float groundPlaneThickness = 0.1f;  // Default value (same as PhysXEngineOld.cpp)
    
    if (m_rootNode && m_rootNode->type == CadNodeType::MutexRoot) {
        MutexRootNodeData* mutexData = m_rootNode->asMutexRoot();
        if (mutexData) {
            groundPlaneY = mutexData->groundPlaneY;
            groundPlaneSize = mutexData->groundPlaneSize;
            groundPlaneThickness = mutexData->groundPlaneThickness;
            qDebug() << "Using ground plane config from root node - Y:" << groundPlaneY 
                     << "Size:" << groundPlaneSize << "Thickness:" << groundPlaneThickness;
        }
    } else {
        qDebug() << "Using default ground plane config - Y:" << groundPlaneY 
                 << "Size:" << groundPlaneSize << "Thickness:" << groundPlaneThickness;
    }
    
    // Create ground plane in CAD coordinates (Z-up) and convert to PhysX coordinates
    // Ground plane Y coordinate in CAD system corresponds to Z coordinate
    PxVec3 cadGroundPos(0, 0, groundPlaneY);  // CAD coordinates: (X, Y, Z)
    PxVec3 physxGroundPos = convertCADToPhysX(cadGroundPos);  // Convert to PhysX coordinates: (X, Z, Y)
    
    m_groundPlane = m_physics->createRigidStatic(PxTransform(physxGroundPos));
    PxShape* groundShape = m_physics->createShape(PxBoxGeometry(groundPlaneSize, groundPlaneThickness, groundPlaneSize), *m_material);

    // Use enhanced contact offset values from PhysXEngineOld.cpp for better collision detection
    float enhancedContactOffset = m_contactOffset * 2.0f;  // 5.0f * 2.0f = 10.0f
    float enhancedRestOffset = m_restOffset * 1.5f;        // 1.0f * 1.5f = 1.5f
    groundShape->setContactOffset(enhancedContactOffset);
    groundShape->setRestOffset(enhancedRestOffset);
    groundShape->setSimulationFilterData(PxFilterData(0, 0, 1, 0));

    m_groundPlane->attachShape(*groundShape);
    m_scene->addActor(*m_groundPlane);
    groundShape->release();

    qDebug() << "Created ground plane at CAD pos:" << cadGroundPos.x << cadGroundPos.y << cadGroundPos.z
             << "PhysX pos:" << physxGroundPos.x << physxGroundPos.y << physxGroundPos.z
             << "Size:" << groundPlaneSize << "Thickness:" << groundPlaneThickness;
    qDebug() << "Ground plane contact offset:" << enhancedContactOffset << "rest offset:" << enhancedRestOffset;
    qDebug() << "Ground plane material - static friction:" << m_material->getStaticFriction() 
             << "dynamic friction:" << m_material->getDynamicFriction() 
             << "restitution:" << m_material->getRestitution();
    return true;
}

void PhysXEngine::configureDynamicActor(PxRigidDynamic* actor)
{
    // Use mass and damping values from PhysXEngineOld.cpp for better stability
    PxRigidBodyExt::setMassAndUpdateInertia(*actor, 10000.0f);  // Keep the same mass
    actor->setAngularDamping(0.5f);   // Keep the same angular damping
    actor->setLinearDamping(0.2f);    // Keep the same linear damping
    actor->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, false);
    actor->setSleepThreshold(m_sleepThreshold);
    actor->setStabilizationThreshold(m_stabilizationThreshold);
    actor->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, false);
    actor->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, m_ccdEnabled);
    actor->setSolverIterationCounts(m_solverPositionIterations, m_solverVelocityIterations);
}


void PhysXEngine::stepSimulationExtended(float deltaTime)
{
    if (!m_scene) {
        return;
    }

    const float fixedTimeStep = 1.0f / 60.0f;
    stepSimulation(fixedTimeStep);

    PxActorTypeFlags actorTypes = PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC;
    PxU32 nbActors = m_scene->getNbActors(actorTypes);
    std::vector<PxActor*> actors(nbActors);
    m_scene->getActors(actorTypes, actors.data(), nbActors);

    for (PxActor* physxActor : actors) {
        PxRigidDynamic* dynamic = physxActor->is<PxRigidDynamic>();
        if (dynamic) {
            bool isKinematic = dynamic->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC;

            if (isKinematic) {
                dynamic->setActorFlag(PxActorFlag::eDISABLE_SIMULATION, false);
            } else {
                PxVec3 linearVel = dynamic->getLinearVelocity();
                PxVec3 angularVel = dynamic->getAngularVelocity();

                float linearThreshold = 0.1f;
                float angularThreshold = 0.01f;

                if (linearVel.magnitudeSquared() < linearThreshold * linearThreshold &&
                    angularVel.magnitudeSquared() < angularThreshold * angularThreshold) {

                    if (!dynamic->isSleeping()) {
                        dynamic->putToSleep();
                    }
                } else {
                    if (dynamic->isSleeping()) {
                        dynamic->wakeUp();
                    }
                }
            }
        }
    }

    bool actorsChanged = false;

    for (PxActor* physxActor : actors) {
        PxRigidDynamic* dynamic = physxActor->is<PxRigidDynamic>();
        if (dynamic && (dynamic->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC)) {
            dynamic->setActorFlag(PxActorFlag::eDISABLE_SIMULATION, false);
        }
    }

    //Insert code to update objects in tree

}

void PhysXEngine::stepSimulation(float deltaTime)
{
    if (!m_scene) return;

    const float fixedTimeStep = 1.0f / 60.0f;
    const float numberOfSubIterations = 1.0f;

    for (int i = 0; i < numberOfSubIterations; i++) {
        bool isLastSubstep = (i == numberOfSubIterations - 1);

        if (!isLastSubstep) {
            m_scene->setSceneQueryUpdateMode(PxSceneQueryUpdateMode::eBUILD_ENABLED_COMMIT_DISABLED);
        } else {
            m_scene->setSceneQueryUpdateMode(PxSceneQueryUpdateMode::eBUILD_ENABLED_COMMIT_ENABLED);
        }

        m_scene->simulate(fixedTimeStep/numberOfSubIterations);
        m_scene->fetchResults(true);
    }
    
    // Debug: Log simulation step
    static int stepCount = 0;
    stepCount++;
    if (stepCount % 60 == 0) { // Log every 60 steps (about once per second)
        qDebug() << "[PhysX] Simulation step" << stepCount << "completed, actors in scene:" 
                 << m_scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC);
        
        // Debug: Check positions of dynamic actors
        PxU32 nbActors = m_scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC);
        std::vector<PxActor*> actors(nbActors);
        m_scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC, actors.data(), nbActors);
        
        for (PxActor* actor : actors) {
            PxRigidDynamic* dynamic = actor->is<PxRigidDynamic>();
            if (dynamic) {
                PxTransform pose = dynamic->getGlobalPose();
                qDebug() << "[PhysX] Actor at step" << stepCount << "position:" << pose.p.x << pose.p.y << pose.p.z;
            }
        }
    }
}

void PhysXEngine::buildSceneFromNodes(const std::vector<CadNode*>& physicsNodes,
                                     std::unordered_map<CadNode*, PxRigidDynamic*>& nodeToActor,
                                     std::unordered_map<PxRigidDynamic*, CadNode*>& actorToNode)
{
    for (CadNode* node : physicsNodes) {
        // 1. Extract geometry (convex hulls)
        PhysicsNodeData* physData = node->asPhysics();
        if (!physData || physData->hulls.empty()) continue;

        // For each hull, create a PhysX convex mesh and shape
        std::vector<PxShape*> shapes;
        for (size_t hullIdx = 0; hullIdx < physData->hulls.size(); ++hullIdx) {
            const auto& hull = physData->hulls[hullIdx];
            // Print debug info
            double minX=1e30,maxX=-1e30,minY=1e30,maxY=-1e30,minZ=1e30,maxZ=-1e30;
            for (const auto& v : hull.vertices) {
                minX = std::min(minX, (double)v[0]); maxX = std::max(maxX, (double)v[0]);
                minY = std::min(minY, (double)v[1]); maxY = std::max(maxY, (double)v[1]);
                minZ = std::min(minZ, (double)v[2]); maxZ = std::max(maxZ, (double)v[2]);
            }
            double extentX = maxX - minX;
            double extentY = maxY - minY;
            double extentZ = maxZ - minZ;
            double diag = std::sqrt(extentX*extentX + extentY*extentY + extentZ*extentZ);
            // Compute volume (signed, using triangles)
            double hullVolume = 0.0;
            for (const auto& tri : hull.indices) {
                const auto& v1 = hull.vertices[tri[0]];
                const auto& v2 = hull.vertices[tri[1]];
                const auto& v3 = hull.vertices[tri[2]];
                hullVolume += ((v1[0] * v2[1] * v3[2]) + (v1[1] * v2[2] * v3[0]) + (v1[2] * v2[0] * v3[1]) -
                              (v3[0] * v2[1] * v1[2]) - (v3[1] * v2[2] * v1[0]) - (v3[2] * v2[0] * v1[1])) / 6.0;
            }
            hullVolume = std::abs(hullVolume);
            qDebug() << "[PhysX] Cooking hull" << hullIdx
                     << "verts:" << hull.vertices.size()
                     << "tris:" << hull.indices.size()
                     << "extents:" << extentX << extentY << extentZ
                     << "diag:" << diag
                     << "volume:" << hullVolume;
            // Minimum volume check
            if (hullVolume < 1e-6) {
                qDebug() << "[PhysX] Skipping hull" << hullIdx << "due to very small volume:" << hullVolume;
                continue;
            }
            // Prepare vertices - convert from CAD to PhysX coordinates
            std::vector<PxVec3> pxVertices(hull.vertices.size());
            for (size_t i = 0; i < hull.vertices.size(); ++i) {
                PxVec3 cadVertex((float)hull.vertices[i][0], (float)hull.vertices[i][1], (float)hull.vertices[i][2]);
                pxVertices[i] = convertCADToPhysX(cadVertex);
            }
            // Create convex mesh descriptor
            PxConvexMeshDesc convexDesc;
            convexDesc.points.count = static_cast<uint32_t>(pxVertices.size());
            convexDesc.points.stride = sizeof(PxVec3);
            convexDesc.points.data = pxVertices.data();
            convexDesc.flags = PxConvexFlag::eCOMPUTE_CONVEX;
            if (pxVertices.size() < 4) {
                qDebug() << "[PhysX] Hull" << hullIdx << "has fewer than 4 vertices, skipping";
                continue;
            }
            // Cook the mesh
            PxDefaultMemoryOutputStream buf;
            if (!PxCookConvexMesh(m_globalCookingParams, convexDesc, buf)) {
                qDebug() << "[PhysX] Failed to cook convex mesh for hull" << hullIdx
                         << "verts:" << hull.vertices.size()
                         << "tris:" << hull.indices.size()
                         << "extents:" << extentX << extentY << extentZ
                         << "diag:" << diag
                         << "volume:" << hullVolume;
                continue;
            }
            PxDefaultMemoryInputData input(buf.getData(), buf.getSize());
            PxConvexMesh* convexMesh = m_physics->createConvexMesh(input);
            // Create shape
            PxShape* shape = m_physics->createShape(
                PxConvexMeshGeometry(convexMesh),
                *m_material
            );
            // Set friction, restitution, etc.
            shape->setMaterials(&m_material, 1);
            shape->setContactOffset(m_contactOffset);
            shape->setRestOffset(m_restOffset);
            shapes.push_back(shape);
        }

        // 2. Create rigid body with proper coordinate conversion
        PxTransform pose;
        
        // Extract transform from node->loc and convert from CAD to PhysX coordinates
        if (node->loc.IsIdentity()) {
            pose = PxTransform(PxVec3(0,0,0), PxQuat(0,0,0,1));
        } else {
            // Convert OpenCASCADE transform to PhysX transform
            gp_Trsf trsf = node->loc.Transformation();
            
            // Extract position
            gp_Pnt origin = trsf.TranslationPart();
            PxVec3 cadPos((float)origin.X(), (float)origin.Y(), (float)origin.Z());
            PxVec3 physxPos = convertCADToPhysX(cadPos);
            
            // Extract rotation matrix
            gp_Mat rotMat = trsf.VectorialPart();
            PxMat33 cadRotMat(
                PxVec3((float)rotMat.Value(1,1), (float)rotMat.Value(1,2), (float)rotMat.Value(1,3)),
                PxVec3((float)rotMat.Value(2,1), (float)rotMat.Value(2,2), (float)rotMat.Value(2,3)),
                PxVec3((float)rotMat.Value(3,1), (float)rotMat.Value(3,2), (float)rotMat.Value(3,3))
            );
            
            // Create coordinate system transformation matrix: CAD (X,Y,Z) -> PhysX (X,Z,Y)
            PxMat33 coordTransform(
                PxVec3(1, 0, 0),  // X axis stays the same
                PxVec3(0, 0, 1),  // Y axis becomes Z axis
                PxVec3(0, 1, 0)   // Z axis becomes Y axis
            );
            
            // Apply transformation: physxRot = coordTransform * cadRot * coordTransform^T
            PxMat33 physxRotMat = coordTransform * cadRotMat * coordTransform.getTranspose();
            
            pose = PxTransform(physxPos, PxQuat(physxRotMat));
        }
        
        PxRigidDynamic* actor = m_physics->createRigidDynamic(pose);
        
        qDebug() << "[PhysX] Created actor for node" << node->name.c_str() 
                 << "at PhysX pos:" << pose.p.x << pose.p.y << pose.p.z;

        // Attach all shapes
        for (PxShape* shape : shapes) {
            actor->attachShape(*shape);
            shape->release(); // actor now owns the shape
        }

        // 3. Set properties
        actor->setMass(physData->mass);
        qDebug() << "[PhysX] Set mass for" << node->name.c_str() << "to" << physData->mass;
        // Configure the actor with proper solver iterations and other settings
        configureDynamicActor(actor);

        // 4. Add to scene
        m_scene->addActor(*actor);
        node->asPhysics()->isPhysicsActive = true;

        // 5. Store mappings
        nodeToActor[node] = actor;
        actorToNode[actor] = node;
    }
}

SimulationManager::SimulationManager(const std::shared_ptr<CadNode>& m_CadNodeRootIn) :
    materialManager(new MaterialManager(nullptr)),
    m_CadNodeRoot(m_CadNodeRootIn) {
    std::cout << "[SimulationManager::ctor] Received root node type: " << static_cast<int>(m_CadNodeRootIn->type)
              << ", name: " << m_CadNodeRootIn->name << std::endl;
    // Initialize state
    m_currentState.time = 0.0f;
    m_currentState.isPaused = true;
    m_currentState.stepCount = 0;
}

void SimulationManager::buildPhysXSceneFromNodes() {
    qDebug() << "Build physics scene called";
    
    // Set scene building flag to prevent GUI rendering during this process
    m_sceneBuilding = true;
    
    m_nodeToActor.clear();
    m_actorToNode.clear();
    
    if (!m_physXEngine) {
        m_physXEngine = std::make_unique<PhysXEngine>();
        if (!m_physXEngine->initializePhysX(m_CadNodeRoot)) {
            std::cout << "Failed to initialize PhysX engine" << std::endl;
            m_sceneBuilding = false;  // Reset flag on failure
            return;
        }
        else {
            std::cout << "PhysXEngine Initialized";
        }
    }

    if (!m_physXEngine || !m_CadNodeRoot) {
        m_sceneBuilding = false;  // Reset flag on failure
        return;
    }

    std::vector<CadNode*> physicsNodes;
    collectPhysicsNodes(m_CadNodeRoot, physicsNodes);

    qDebug() << "Starting simulation with n actors: " << physicsNodes.size();

    // Delegate scene building to PhysXEngine
    m_physXEngine->buildSceneFromNodes(physicsNodes, m_nodeToActor, m_actorToNode);

    qDebug() << "Started simulation with n actors: " << physicsNodes.size();
    
    // Scene building complete, but keep flag true until first simulation step
    // The flag will be reset in updateSimulation() after the first step
}

void SimulationManager::startSimulation() {
    if (m_running) {
        std::cout << "Simulation is already running" << std::endl;
        return;
    }

    // Print the main thread id
    std::cout << "Main thread id (startSimulation): " << std::this_thread::get_id() << std::endl;

    m_running = true;
    m_paused = false;
    m_currentState.isPaused = false;

    // Start simulation thread - scene building will happen on the simulation thread
    m_simulationThread = std::thread(&SimulationManager::simulationLoop, this);
    std::cout << "Simulation started" << std::endl;
}

void SimulationManager::pauseSimulation() {
    if (!m_running) {
        std::cout << "Simulation is not running" << std::endl;
        return;
    }

    m_paused = true;
    {
        std::lock_guard<std::mutex> lock(m_stateMutex);
        m_currentState.isPaused = true;
    }

    std::cout << "Simulation paused" << std::endl;
}

void SimulationManager::resumeSimulation() {
    if (!m_running) {
        std::cout << "Simulation is not running" << std::endl;
        return;
    }

    m_paused = false;
    {
        std::lock_guard<std::mutex> lock(m_stateMutex);
        m_currentState.isPaused = false;
    }

    std::cout << "Simulation resumed" << std::endl;
}

void SimulationManager::stopSimulation() {
    if (!m_running) {
        return;
    }

    // Send quit command
    sendCommand(SimulationCommand::QUIT);

    // Wait for thread to finish
    if (m_simulationThread.joinable()) {
        m_simulationThread.join();
    }

    m_running = false;
    m_paused = false;
    m_currentState.isPaused = false;
}


void SimulationManager::addGuiElements(QMainWindow* mainWindow)
{
    if (!mainWindow) return;
    // Create a toolbar
    QToolBar* toolbar = new QToolBar("Simulation Toolbar", mainWindow);
    mainWindow->addToolBar(toolbar);

    // Create a QToolButton with a dropdown menu
    QToolButton* toolButton = new QToolButton(toolbar);
    toolButton->setText("Test");
    toolButton->setPopupMode(QToolButton::InstantPopup);

    // Create the menu
    QMenu* menu = new QMenu(toolButton);
    QAction* showMaterialManagerAction = menu->addAction("Show Material Manager");
    QAction* startSimulationAction = menu->addAction("Start Simulation");
    QAction* stopSimulationAction = menu->addAction("Stop Simulation");
    menu->addSeparator();
    QAction* connectPvdAction = menu->addAction("Connect PVD");
    QAction* pvdStatusAction = menu->addAction("PVD Status");
    toolButton->setMenu(menu);

    toolbar->addWidget(toolButton);

    QObject::connect(showMaterialManagerAction, &QAction::triggered, mainWindow, [mainWindow, this]() {
        if (materialManager) {
            materialManager->showMaterialManagerDialog(mainWindow);
        }
    });
    QObject::connect(startSimulationAction, &QAction::triggered, mainWindow, [this]() {
        this->startSimulation();
    });
    QObject::connect(stopSimulationAction, &QAction::triggered, mainWindow, [this]() {
        this->stopSimulation();
    });
    QObject::connect(connectPvdAction, &QAction::triggered, mainWindow, [this]() {
        this->connectPvd();
    });
    QObject::connect(pvdStatusAction, &QAction::triggered, mainWindow, [this]() {
        bool connected = this->isPvdConnected();
        qDebug() << "PVD Status:" << (connected ? "Connected" : "Disconnected");
    });
}

void SimulationManager::registerPhysicsNodeContextMenu(QMenu* menu, CadNode* node)
{
    if (!menu || !node) return;
    if (node->type != CadNodeType::Physics || !node->asPhysics()) return;
    QAction* editPropsAction = menu->addAction("Edit Object Properties...");
    QObject::connect(editPropsAction, &QAction::triggered, nullptr, [this, node]() {
        // You may want to pass a parent QWidget if needed
        ObjectPropertiesDialog dlg(node, materialManager, nullptr);
        dlg.exec();
    });
}


void SimulationManager::simulationLoop() {
    std::cout << "Simulation thread started " <<  std::this_thread::get_id() << std::endl;
    
    // Build PhysX scene on the simulation thread
    buildPhysXSceneFromNodes();

    while (m_running) {
        // At the top of the loop, mark frame start
        auto frameStart = std::chrono::steady_clock::now();

        // Process any pending commands
        processCommands();

        // Check if we should quit
        if (!m_running) {
            break;
        }

        // If paused, wait for resume or step command
        if (m_paused && !m_stepRequested) {
            std::unique_lock<std::mutex> lock(m_commandMutex);
            m_commandCV.wait(lock, [this] {
                return !m_running || !m_paused || m_stepRequested;
            });
            continue;
        }

        // Perform simulation step
        updateSimulation();

        // If this was a single step request, mark it complete
        if (m_stepRequested) {
            m_stepRequested = false;
            m_stepComplete = true;
            m_stepCompleteCV.notify_one();
        }

        // At the end of the loop, measure elapsed and sleep if needed
        auto frameEnd = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(frameEnd - frameStart);
        auto targetDuration = std::chrono::milliseconds(static_cast<int>(m_timeStepMS));

        if (elapsed < targetDuration) {
            std::this_thread::sleep_for(targetDuration - elapsed);
        }
    }

    std::cout << "Simulation thread finished" << std::endl;
}

void SimulationManager::processCommands() {
    std::lock_guard<std::mutex> lock(m_commandMutex);

    while (!m_commandQueue.empty()) {
        SimulationCommandData cmd = m_commandQueue.front();
        m_commandQueue.pop();

        switch (cmd.command) {
            case SimulationCommand::START:
                // Already handled by startSimulation()
                break;

            case SimulationCommand::PAUSE:
                m_paused = true;
                {
                    std::lock_guard<std::mutex> stateLock(m_stateMutex);
                    m_currentState.isPaused = true;
                }
                break;

            case SimulationCommand::RESUME:
                m_paused = false;
                {
                    std::lock_guard<std::mutex> stateLock(m_stateMutex);
                    m_currentState.isPaused = false;
                }
                break;

            case SimulationCommand::QUIT:
                m_running = false;
                break;
        }
    }
}

void SimulationManager::updateSimulation() {
    QElapsedTimer simStepTimer;
    simStepTimer.start();
    if (!m_physXEngine) {
        return;
    }

    // Get the current write buffer
    NodeLocationData* writeBuffer = m_writeBuffer.load();
    writeBuffer->nodeLocations.clear(); // Clear previous data
    writeBuffer->isDirty = true;

    // Perform PhysX simulation step
     m_physXEngine->stepSimulationExtended(m_timeStepMS / 1000.0f);

    // Update our state from PhysX
    {
        std::lock_guard<std::mutex> lock(m_stateMutex);
        m_currentState.time += m_timeStepMS / 1000.0f;
        m_currentState.stepCount++;

        // Update object positions/velocities from PhysX
        for (const auto& pair : m_actorToNode) {
            PxRigidActor* actor = pair.first;
            CadNode* node = pair.second;
            if (!actor || !node) continue;

            PxTransform physxPose = actor->getGlobalPose();
            
            // Convert PhysX pose back to CAD coordinates
            PxTransform cadPose = convertPhysXPoseToCAD(physxPose);
            PxVec3 pos = cadPose.p;
            PxQuat quat = cadPose.q;

            // Convert CAD pose to OpenCASCADE gp_Trsf
            gp_Trsf trsf;
            // Set rotation (quaternion to rotation matrix)
            double qx = quat.x, qy = quat.y, qz = quat.z, qw = quat.w;
            double xx = qx * qx, yy = qy * qy, zz = qz * qz;
            double xy = qx * qy, xz = qx * qz, yz = qy * qz;
            double wx = qw * qx, wy = qw * qy, wz = qw * qz;

            double m[3][3];
            m[0][0] = 1.0 - 2.0 * (yy + zz);
            m[0][1] = 2.0 * (xy - wz);
            m[0][2] = 2.0 * (xz + wy);
            m[1][0] = 2.0 * (xy + wz);
            m[1][1] = 1.0 - 2.0 * (xx + zz);
            m[1][2] = 2.0 * (yz - wx);
            m[2][0] = 2.0 * (xz - wy);
            m[2][1] = 2.0 * (yz + wx);
            m[2][2] = 1.0 - 2.0 * (xx + yy);

            trsf.SetValues(
                m[0][0], m[0][1], m[0][2], pos.x,
                m[1][0], m[1][1], m[1][2], pos.y,
                m[2][0], m[2][1], m[2][2], pos.z
            );

            // Store location in write buffer instead of directly modifying node
            TopLoc_Location newLocation(trsf);
            writeBuffer->nodeLocations[node] = newLocation;
            
            // Debug output for first few updates
            if (m_currentState.stepCount < 10) {
                qDebug() << "[PhysX] Updated node" << node->name.c_str() 
                         << "step:" << m_currentState.stepCount
                         << "PhysX pos:" << physxPose.p.x << physxPose.p.y << physxPose.p.z
                         << "CAD pos:" << pos.x << pos.y << pos.z;
            }
        }
    }

    // Reset scene building flag after first simulation step
    if (m_sceneBuilding && m_currentState.stepCount == 1) {
        m_sceneBuilding = false;
        qDebug() << "[SimulationManager] Scene building protection disabled after first simulation step";
    }

    // Mark write buffer as complete and swap buffers
    writeBuffer->isDirty = false;
    writeBuffer->version++;
    
    // Swap read and write buffers atomically
    NodeLocationData* oldReadBuffer = m_readBuffer.exchange(writeBuffer);
    m_writeBuffer.store(oldReadBuffer);
    m_buffersSwapped = true;

    // Notify GUI of update
    notifyGUIUpdate();

    // Log PVD connection status periodically and attempt reconnection if needed
    if (m_currentState.stepCount % 60 == 0) { // Every 60 steps (about 1 second at 60 FPS)
        bool pvdConnected = isPvdConnected();        
        // Try to reconnect if disconnected
        if (!pvdConnected && m_physXEngine && m_physXEngine->m_pvdTransport) {
            qDebug() << "[SimulationManager] Attempting PVD reconnection...";
            bool reconnected = m_physXEngine->m_pvd->connect(*m_physXEngine->m_pvdTransport, PxPvdInstrumentationFlag::eALL);
            if (reconnected) {
                qDebug() << "[SimulationManager] PVD reconnected successfully";
            }
        }
    }

    qint64 elapsedMs = simStepTimer.elapsed();
}

void SimulationManager::sendCommand(SimulationCommand command) {
    std::lock_guard<std::mutex> lock(m_commandMutex);

    SimulationCommandData cmd;
    cmd.command = command;

    m_commandQueue.push(cmd);
    m_commandCV.notify_one();
}


void SimulationManager::notifyGUIUpdate() {
    if (m_updateCallback) {
        SimulationState stateCopy;
        {
            std::lock_guard<std::mutex> lock(m_stateMutex);
            stateCopy = m_currentState;
        }

        // Note: In a real Qt application, you might want to use QMetaObject::invokeMethod
        // to ensure the callback runs on the main thread
        m_updateCallback(stateCopy);
    }
}


const std::unordered_map<CadNode*, TopLoc_Location>& SimulationManager::getLatestNodeLocations() const {
    return m_readBuffer.load()->nodeLocations;
}

void SimulationManager::markUpdatesProcessed() {
    m_buffersSwapped = false;
}

bool SimulationManager::isPvdConnected() const {
    if (m_physXEngine && m_physXEngine->m_pvd) {
        return m_physXEngine->m_pvd->isConnected();
    }
    return false;
}

void SimulationManager::connectPvd() {
    if (m_physXEngine && m_physXEngine->m_pvd) {
        // Try to reconnect PVD using existing transport
        if (m_physXEngine->m_pvdTransport) {
            bool connected = m_physXEngine->m_pvd->connect(*m_physXEngine->m_pvdTransport, PxPvdInstrumentationFlag::eALL);
            if (connected) {
                qDebug() << "PVD reconnected successfully";
            } else {
                qDebug() << "PVD reconnection failed - make sure PVD is running";
            }
        } else {
            qDebug() << "PVD transport not available for reconnection";
        }
    }
}

void SimulationManager::collectPhysicsNodes(const std::shared_ptr<CadNode>& root, std::vector<CadNode*>& outNodes) {
    if (!root) return;
    if (root->type == CadNodeType::Physics) {
        outNodes.push_back(root.get());
    }
    for (const auto& child : root->children) {
        collectPhysicsNodes(child, outNodes);
    }
}
