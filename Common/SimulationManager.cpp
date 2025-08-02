#include "SimulationManager.h"
#include "HelperFunctions.h"
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
#include "BezierDragChainSolver.h"
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>

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
#include "extensions/PxJoint.h"
#include "extensions/PxRevoluteJoint.h"

// Define PxPi if not already defined
#ifndef PxPi
#define PxPi 3.14159265358979323846f
#endif

// Define PxClamp if not already defined
#ifndef PxClamp
#define PxClamp(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
#endif

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

    // Initialize gravity for PhysX (Z-up coordinate system) - using value from PhysXEngineOld.cpp
    // Gravity points down in Z direction in PhysX (same as CAD coordinates)
    m_gravity = PxVec3(0.0f, 0.0f, -9806.65f);  // Use mm/s² units like in PhysXEngineOld.cpp
    
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
    
    // Create ground plane in PhysX coordinates (Z-up, same as CAD)
    // Ground plane Z coordinate in PhysX system
    PxVec3 groundPos(0, 0, groundPlaneY);  // PhysX coordinates: (X, Y, Z)
    
    m_groundPlane = m_physics->createRigidStatic(PxTransform(groundPos));
    PxShape* groundShape = m_physics->createShape(PxBoxGeometry(groundPlaneSize, groundPlaneSize, groundPlaneThickness), *m_material);

    // Use enhanced contact offset values from PhysXEngineOld.cpp for better collision detection
    float enhancedContactOffset = m_contactOffset * 2.0f;  // 5.0f * 2.0f = 10.0f
    float enhancedRestOffset = m_restOffset * 1.5f;        // 1.0f * 1.5f = 1.5f
    groundShape->setContactOffset(enhancedContactOffset);
    groundShape->setRestOffset(enhancedRestOffset);
    groundShape->setSimulationFilterData(PxFilterData(0, 0, 1, 0));

    m_groundPlane->attachShape(*groundShape);
    m_scene->addActor(*m_groundPlane);
    groundShape->release();

    qDebug() << "Created ground plane at pos:" << groundPos.x << groundPos.y << groundPos.z
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
                                     std::unordered_map<PxRigidDynamic*, CadNode*>& actorToNode,
                                     std::unordered_map<PxRigidDynamic*, std::shared_ptr<CadNode>>& actorToSegmentNode)
{
    qDebug() << "[PhysX] buildSceneFromNodes called with" << physicsNodes.size() << "nodes";
    
    for (CadNode* node : physicsNodes) {
        qDebug() << "[PhysX] Processing node:" << QString::fromStdString(node->name) << "type:" << static_cast<int>(node->type);
        
        if (node->type == CadNodeType::Physics) {
            qDebug() << "[PhysX] Processing Physics node:" << QString::fromStdString(node->name);
            // Handle Physics nodes (existing code)
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
                // Prepare vertices - PhysX and CAD use the same coordinate system
                std::vector<PxVec3> pxVertices(hull.vertices.size());
                for (size_t i = 0; i < hull.vertices.size(); ++i) {
                    pxVertices[i] = PxVec3((float)hull.vertices[i][0], (float)hull.vertices[i][1], (float)hull.vertices[i][2]);
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

            // 2. Create rigid body with proper transform
            PxTransform pose;
            
            // Extract transform from node->loc - PhysX and CAD use the same coordinate system
            if (node->loc.IsIdentity()) {
                pose = PxTransform(PxVec3(0,0,0), PxQuat(0,0,0,1));
            } else {
                // Convert OpenCASCADE transform to PhysX transform
                gp_Trsf trsf = node->loc.Transformation();
                
                // Extract position
                gp_Pnt origin = trsf.TranslationPart();
                PxVec3 pos((float)origin.X(), (float)origin.Y(), (float)origin.Z());
                
                // Extract rotation matrix
                gp_Mat rotMat = trsf.VectorialPart();
                PxMat33 rotMat33(
                    PxVec3((float)rotMat.Value(1,1), (float)rotMat.Value(1,2), (float)rotMat.Value(1,3)),
                    PxVec3((float)rotMat.Value(2,1), (float)rotMat.Value(2,2), (float)rotMat.Value(2,3)),
                    PxVec3((float)rotMat.Value(3,1), (float)rotMat.Value(3,2), (float)rotMat.Value(3,3))
                );
                
                pose = PxTransform(pos, PxQuat(rotMat33));
            }
            
            PxRigidDynamic* actor = m_physics->createRigidDynamic(pose);
            
            qDebug() << "[PhysX] Created actor for node" << node->name.c_str() 
                     << "at pos:" << pose.p.x << pose.p.y << pose.p.z;

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
        else if (node->type == CadNodeType::Connection) {
            // Handle Connection nodes (drag chains)
            qDebug() << "[PhysX] Processing Connection node:" << QString::fromStdString(node->name);
            
            // Check if the connection node has valid data
            auto connectionData = node->asConnection();
            if (connectionData) {
                qDebug() << "[PhysX] Connection data valid - Type:" << static_cast<int>(connectionData->connectionType)
                         << "Settings JSON length:" << connectionData->creationSettingsJson.length();
            } else {
                qDebug() << "[PhysX] WARNING: Connection node has no connection data!";
            }
            
            createDragChainFromConnection(node, nodeToActor, actorToNode, actorToSegmentNode, m_parent);
        }
        else {
            qDebug() << "[PhysX] Skipping node of type:" << static_cast<int>(node->type);
        }
    }
}

void PhysXEngine::createDragChainFromConnection(CadNode* connectionNode,
                                             std::unordered_map<CadNode*, PxRigidDynamic*>& nodeToActor,
                                             std::unordered_map<PxRigidDynamic*, CadNode*>& actorToNode,
                                             std::unordered_map<PxRigidDynamic*, std::shared_ptr<CadNode>>& actorToSegmentNode,
                                             SimulationManager* simManager)
{
    qDebug() << "[PhysX] SIMULATION - createDragChainFromConnection called for:" << QString::fromStdString(connectionNode->name);
    qDebug() << "[PhysX] SIMULATION - Connection node children count at start:" << connectionNode->children.size();
    qDebug() << "[PhysX] SIMULATION - Function call count check - this should only appear once per connection";
    qDebug() << "[PhysX] SIMULATION - Connection node address:" << connectionNode;
    qDebug() << "[PhysX] SIMULATION - Current m_connectionSegments size:" << m_parent->m_connectionSegments.size();
    
    // Check if this connection node has already been processed
    if (m_parent->m_processedConnections.find(connectionNode) != m_parent->m_processedConnections.end()) {
        qDebug() << "[PhysX] SIMULATION - Connection node already processed, skipping duplicate call";
        return;
    }
    
    // Mark this connection node as processed
    m_parent->m_processedConnections.insert(connectionNode);
    qDebug() << "[PhysX] SIMULATION - Marked connection node as processed";
    
    // Clean up any existing segments for this connection node
    qDebug() << "[PhysX] SIMULATION - Cleaning up existing segments for connection:" << QString::fromStdString(connectionNode->name);
    
    // First, clean up any existing segment children from the connection node
    qDebug() << "[PhysX] SIMULATION - Connection node has" << connectionNode->children.size() << "children before cleanup";
    auto childIt = connectionNode->children.begin();
    while (childIt != connectionNode->children.end()) {
        if ((*childIt) && (*childIt)->name.find("_Segment_") != std::string::npos) {
            qDebug() << "[PhysX] SIMULATION - Removing existing segment child:" << QString::fromStdString((*childIt)->name);
            childIt = connectionNode->children.erase(childIt);
        } else {
            ++childIt;
        }
    }
    qDebug() << "[PhysX] SIMULATION - Connection node has" << connectionNode->children.size() << "children after cleanup";
    
    // Then clean up from the tracking map
    auto it = m_parent->m_connectionSegments.find(connectionNode);
    if (it != m_parent->m_connectionSegments.end()) {
        qDebug() << "[PhysX] SIMULATION - Found" << it->second.size() << "existing segments in tracking map to remove";
        for (auto& segmentNode : it->second) {
            if (segmentNode) {
                // Remove from PhysX scene if it has an actor
                auto actorIt = nodeToActor.find(segmentNode.get());
                if (actorIt != nodeToActor.end()) {
                    m_scene->removeActor(*actorIt->second);
                    nodeToActor.erase(actorIt);
                }
            }
        }
        m_parent->m_connectionSegments.erase(it);
    }
    
    if (!connectionNode || connectionNode->type != CadNodeType::Connection) {
        qDebug() << "[PhysX] Invalid connection node for drag chain creation";
        qDebug() << "[PhysX] Node type:" << static_cast<int>(connectionNode->type);
        return;
    }

    ConnectionNodeData* connData = connectionNode->asConnection();
    if (!connData) {
        qDebug() << "[PhysX] No connection data found for node:" << QString::fromStdString(connectionNode->name);
        return;
    }

    qDebug() << "[PhysX] Creating drag chain from connection:" << QString::fromStdString(connectionNode->name);
    qDebug() << "[PhysX] Connection type:" << static_cast<int>(connData->connectionType);
    qDebug() << "[PhysX] Pitch length:" << connData->pitchLength;
    qDebug() << "[PhysX] Max bend radius:" << connData->maxBendRadius;
    qDebug() << "[PhysX] Creation settings JSON length:" << connData->creationSettingsJson.length();
    qDebug() << "[PhysX] Using shared segment generation for consistency with preview";

    // Parse the creation settings JSON to get the solver data
    if (connData->creationSettingsJson.isEmpty()) {
        qDebug() << "[PhysX] No creation settings JSON found for connection:" << QString::fromStdString(connectionNode->name);
        qDebug() << "[PhysX] Attempting to create drag chain with default settings...";
        
        // Try to create drag chain with default settings
        QJsonObject defaultSettings;
        defaultSettings["connectionName"] = QString::fromStdString(connectionNode->name);
        defaultSettings["connectionType"] = static_cast<int>(connData->connectionType);
        defaultSettings["isFlexible"] = connData->isFlexible;
        defaultSettings["maxBendRadius"] = connData->maxBendRadius;
        defaultSettings["pitchLength"] = connData->pitchLength;
        
        // Add default control points
        QJsonArray controlPointsArray;
        QJsonObject controlPoint;
        controlPoint["x"] = 100.0;
        controlPoint["y"] = 0.0;
        controlPoint["z"] = 50.0;
        controlPointsArray.append(controlPoint);
        defaultSettings["controlPoints"] = controlPointsArray;
        
        // Add default visualization settings
        QJsonObject visualization;
        QJsonObject startPoint;
        startPoint["x"] = 0.0;
        startPoint["y"] = 0.0;
        startPoint["z"] = 0.0;
        visualization["startPoint"] = startPoint;
        
        QJsonObject endPoint;
        endPoint["x"] = 200.0;
        endPoint["y"] = 0.0;
        endPoint["z"] = 0.0;
        visualization["endPoint"] = endPoint;
        defaultSettings["visualization"] = visualization;
        
        QJsonDocument doc(defaultSettings);
        connData->creationSettingsJson = QString::fromUtf8(doc.toJson(QJsonDocument::Compact));
        
        qDebug() << "[PhysX] Created default settings:" << connData->creationSettingsJson;
    }

    QJsonDocument doc = QJsonDocument::fromJson(connData->creationSettingsJson.toUtf8());
    if (!doc.isObject()) {
        qDebug() << "[PhysX] Failed to parse creation settings JSON for connection:" << QString::fromStdString(connectionNode->name);
        qDebug() << "[PhysX] JSON content:" << connData->creationSettingsJson;
        return;
    }

    QJsonObject settings = doc.object();
    qDebug() << "[PhysX] Successfully parsed JSON settings";
    
    // Extract control points from settings
    std::vector<QVector3D> controlPoints;
    if (settings.contains("controlPoints")) {
        QJsonArray controlPointsArray = settings["controlPoints"].toArray();
        qDebug() << "[PhysX] Found" << controlPointsArray.size() << "control points in settings";
        for (const auto& pointObj : controlPointsArray) {
            QJsonObject point = pointObj.toObject();
            QVector3D controlPoint(
                point["x"].toDouble(),
                point["y"].toDouble(),
                point["z"].toDouble()
            );
            controlPoints.push_back(controlPoint);
            qDebug() << "[PhysX] Control point:" << controlPoint;
        }
    } else {
        qDebug() << "[PhysX] No control points found in settings";
    }

        // Extract connection point references from settings (same as preview)
    CadNode* point1 = nullptr;
    CadNode* point2 = nullptr;
    CadNode* rootNode = nullptr;
    
    qDebug() << "[PhysX] SIMULATION - Checking for connection points in settings:";
    qDebug() << "[PhysX] SIMULATION - Settings keys:" << settings.keys();
    qDebug() << "[PhysX] SIMULATION - Contains connectionPoint1:" << settings.contains("connectionPoint1");
    qDebug() << "[PhysX] SIMULATION - Contains connectionPoint2:" << settings.contains("connectionPoint2");
    
    if (settings.contains("connectionPoint1") && settings.contains("connectionPoint2")) {
        // Get connection point references from settings
        QString point1Name = settings["connectionPoint1"].toString();
        QString point2Name = settings["connectionPoint2"].toString();
        
        // Use the new unique names that were set during connection creation
        point1Name = "Connection_Point_1";
        point2Name = "Connection_Point_2";
        
        qDebug() << "[PhysX] SIMULATION - Found connection point names:";
        qDebug() << "[PhysX] SIMULATION - Point1 name:" << point1Name;
        qDebug() << "[PhysX] SIMULATION - Point2 name:" << point2Name;
        
        // Look for connection points as children of the connection node itself
        qDebug() << "[PhysX] SIMULATION - Looking for connection points as children of connection node";
        qDebug() << "[PhysX] SIMULATION - Connection node children count:" << connectionNode->children.size();
        
        for (const auto& child : connectionNode->children) {
            if (child) {
                QString childName = QString::fromStdString(child->name);
                qDebug() << "[PhysX] SIMULATION - Child:" << childName << "type:" << static_cast<int>(child->type);
                
                if (childName == point1Name) {
                    point1 = child.get();
                    qDebug() << "[PhysX] SIMULATION - Found point1 as child:" << childName;
                }
                if (childName == point2Name) {
                    point2 = child.get();
                    qDebug() << "[PhysX] SIMULATION - Found point2 as child:" << childName;
                }
            }
        }
        
        // Use the connection node itself as the root for position calculations
        rootNode = connectionNode;
        
        qDebug() << "[PhysX] Found connection points:";
        qDebug() << "[PhysX] Point1 name:" << point1Name << "found:" << (point1 != nullptr);
        qDebug() << "[PhysX] Point2 name:" << point2Name << "found:" << (point2 != nullptr);
        if (point1) {
            qDebug() << "[PhysX] Point1 actual name:" << QString::fromStdString(point1->name);
            qDebug() << "[PhysX] Point1 type:" << static_cast<int>(point1->type);
            qDebug() << "[PhysX] Point1 local position:" << point1->loc.Transformation().TranslationPart().X() 
                     << point1->loc.Transformation().TranslationPart().Y() 
                     << point1->loc.Transformation().TranslationPart().Z();
        }
        if (point2) {
            qDebug() << "[PhysX] Point2 actual name:" << QString::fromStdString(point2->name);
            qDebug() << "[PhysX] Point2 type:" << static_cast<int>(point2->type);
            qDebug() << "[PhysX] Point2 local position:" << point2->loc.Transformation().TranslationPart().X() 
                     << point2->loc.Transformation().TranslationPart().Y() 
                     << point2->loc.Transformation().TranslationPart().Z();
        }
    }
    
    // Use shared function to get connection point positions (same as preview)
    QVector3D startPoint(0, 0, 0);
    QVector3D endPoint(0, 0, 0);
    
    if (!point1 || !point2 || !rootNode) {
        qDebug() << "[PhysX] ERROR: Connection points not found, cannot create drag chain";
        qDebug() << "[PhysX] ERROR: point1:" << (point1 != nullptr);
        qDebug() << "[PhysX] ERROR: point2:" << (point2 != nullptr);
        qDebug() << "[PhysX] ERROR: rootNode:" << (rootNode != nullptr);
        return;
    }
    
    // Use the same method as the preview to calculate connection point positions
    // Find the actual root node to use for position calculation
    CadNode* actualRootNode = nullptr;
    CadNode* current = connectionNode;
    while (current->parent) {
        current = current->parent;
    }
    actualRootNode = current;
    
    qDebug() << "[PhysX] SIMULATION - Using shared function for position calculation";
    qDebug() << "[PhysX] SIMULATION - Root node:" << QString::fromStdString(actualRootNode->name);
    qDebug() << "[PhysX] SIMULATION - Has node updates:" << m_parent->hasNodeUpdates();
    qDebug() << "[PhysX] SIMULATION - Latest node locations count:" << m_parent->getLatestNodeLocations().size();
    qDebug() << "[PhysX] SIMULATION - Connection node name:" << QString::fromStdString(connectionNode->name);
    qDebug() << "[PhysX] SIMULATION - Connection node local position:" << connectionNode->loc.Transformation().TranslationPart().X() 
             << connectionNode->loc.Transformation().TranslationPart().Y() 
             << connectionNode->loc.Transformation().TranslationPart().Z();
    
    // Calculate connection node's global position
    TopLoc_Location connectionNodeGlobalLoc = findAccumulatedLocation(connectionNode, actualRootNode, TopLoc_Location());
    if (!connectionNodeGlobalLoc.IsIdentity()) {
        gp_Trsf connectionTrsf = connectionNodeGlobalLoc.Transformation();
        QVector3D connectionNodeGlobalPos(connectionTrsf.TranslationPart().X(), 
                                         connectionTrsf.TranslationPart().Y(), 
                                         connectionTrsf.TranslationPart().Z());
        qDebug() << "[PhysX] SIMULATION - Connection node global position:" << connectionNodeGlobalPos;
    }
    
    // Use the shared function (same as preview)
    // Pass real update functions to get accurate node positions
    std::pair<QVector3D, QVector3D> positions = getConnectionPointPositionsShared(
        point1,
        point2,
        actualRootNode,
        [this]() { return m_parent->hasNodeUpdates(); }, // Use real node update check
        [this]() -> const std::unordered_map<CadNode*, TopLoc_Location>& { 
            return m_parent->getLatestNodeLocations(); // Use real node locations
        }
    );
    
    startPoint = positions.first;
    endPoint = positions.second;
    
    qDebug() << "[PhysX] SIMULATION - Using shared function for position calculation";
    qDebug() << "[PhysX] SIMULATION - Point1 name:" << QString::fromStdString(point1->name);
    qDebug() << "[PhysX] SIMULATION - Point2 name:" << QString::fromStdString(point2->name);
    qDebug() << "[PhysX] SIMULATION - Start point:" << startPoint;
    qDebug() << "[PhysX] SIMULATION - End point:" << endPoint;
    
    // Debug: Show the actual node locations being used
    const auto& nodeLocations = m_parent->getLatestNodeLocations();
    auto point1It = nodeLocations.find(point1);
    auto point2It = nodeLocations.find(point2);
    if (point1It != nodeLocations.end()) {
        gp_Trsf trsf1 = point1It->second.Transformation();
        QVector3D point1Pos(trsf1.TranslationPart().X(), trsf1.TranslationPart().Y(), trsf1.TranslationPart().Z());
        qDebug() << "[PhysX] SIMULATION - Point1 actual location:" << point1Pos;
        qDebug() << "[PhysX] SIMULATION - Point1 calculated vs actual:" << startPoint << "vs" << point1Pos;
        qDebug() << "[PhysX] SIMULATION - Point1 difference:" << (startPoint - point1Pos);
    }
    if (point2It != nodeLocations.end()) {
        gp_Trsf trsf2 = point2It->second.Transformation();
        QVector3D point2Pos(trsf2.TranslationPart().X(), trsf2.TranslationPart().Y(), trsf2.TranslationPart().Z());
        qDebug() << "[PhysX] SIMULATION - Point2 actual location:" << point2Pos;
        qDebug() << "[PhysX] SIMULATION - Point2 calculated vs actual:" << endPoint << "vs" << point2Pos;
        qDebug() << "[PhysX] SIMULATION - Point2 difference:" << (endPoint - point2Pos);
    }
    
    qDebug() << "[PhysX] Control points count:" << controlPoints.size();

        // Use the same waypoint-based approach as the preview for consistency
    std::vector<BezierDragChainSegment> segments;
    
    // Use shared function to generate segments (same as preview)
    // This ensures simulation segments match the preview exactly
    qDebug() << "[PhysX] SIMULATION - About to call generateSharedSegments with" << controlPoints.size() << "control points";
    segments = BezierDragChainSolver::generateSharedSegments(startPoint, endPoint, controlPoints, connData->pitchLength, connData->maxBendRadius, true);
    qDebug() << "[PhysX] SIMULATION - generateSharedSegments returned" << segments.size() << "segments";
    
    // Debug: Show segment details
    for (size_t i = 0; i < segments.size(); ++i) {
        const auto& segment = segments[i];
        qDebug() << "[PhysX] SIMULATION - Created segment" << i << ":" << segment.startPoint << "->" << segment.endPoint;
    }
    
    qDebug() << "[PhysX] Generated" << segments.size() << "straight-line segments for simulation";

    if (segments.empty()) {
        qDebug() << "[PhysX] Failed to generate drag chain segments for connection:" << QString::fromStdString(connectionNode->name);
        return;
    }

    qDebug() << "[PhysX] Generated" << segments.size() << "segments for drag chain";
    
    // Debug: Show first and last segment positions for comparison with preview
    if (!segments.empty()) {
        qDebug() << "[PhysX] SIMULATION segments - First:" << segments[0].startPoint << "->" << segments[0].endPoint;
        qDebug() << "[PhysX] SIMULATION segments - Last:" << segments.back().startPoint << "->" << segments.back().endPoint;
        qDebug() << "[PhysX] SIMULATION - Expected start point:" << startPoint << "vs First segment start:" << segments[0].startPoint;
        qDebug() << "[PhysX] SIMULATION - Expected end point:" << endPoint << "vs Last segment end:" << segments.back().endPoint;
    } else {
        qDebug() << "[PhysX] ERROR: No segments generated!";
    }

    // Create PhysX actors for each segment
    std::vector<PxRigidDynamic*> chainActors;
    std::vector<std::shared_ptr<CadNode>> segmentNodes; // Store visual nodes for position updates
    
    // Store segments for this connection node to prevent duplicates
    m_parent->m_connectionSegments[connectionNode] = std::vector<std::shared_ptr<CadNode>>();
    
    qDebug() << "[PhysX] SIMULATION - Creating PhysX actors for" << segments.size() << "segments";
    qDebug() << "[PhysX] SIMULATION - Connection node current children count:" << connectionNode->children.size();
    
    for (size_t i = 0; i < segments.size(); ++i) {
        const auto& segment = segments[i];
        
        // Use exact length from the BezierDragChainSegment (same as preview)
        float segmentLength = static_cast<float>(segment.length);
        float segmentWidth = 20.0f;  // Default width
        float segmentHeight = 15.0f; // Default height
        
        // Extract dimensions from settings if available
        if (settings.contains("chainWidth")) {
            segmentWidth = settings["chainWidth"].toDouble();
        }
        if (settings.contains("chainHeight")) {
            segmentHeight = settings["chainHeight"].toDouble();
        }

        // Use exact positions from the BezierDragChainSegment (same as preview)
        QVector3D segmentStart = segment.startPoint;
        QVector3D segmentEnd = segment.endPoint;
        QVector3D segmentDirection = (segmentEnd - segmentStart).normalized();
        QVector3D segmentCenter = (segmentStart + segmentEnd) * 0.5f;
        
        // Debug: Check segment positions (should match preview exactly)
        qDebug() << "[PhysX] Segment" << i << "startPoint:" << segmentStart << "endPoint:" << segmentEnd;
        qDebug() << "[PhysX] Segment" << i << "center:" << segmentCenter;
        
        // Create box geometry for the segment
        PxBoxGeometry boxGeom(segmentLength * 0.5f, segmentWidth * 0.5f, segmentHeight * 0.5f);
        PxShape* shape = m_physics->createShape(boxGeom, *m_material);
        
        // Set material properties
        shape->setMaterials(&m_material, 1);
        shape->setContactOffset(m_contactOffset);
        shape->setRestOffset(m_restOffset);
        
        // Create actor at segment center position
        PxVec3 segmentPos((float)segmentCenter.x(), (float)segmentCenter.y(), (float)segmentCenter.z());
        
        // Calculate rotation to align segment with direction
        // Convert QVector3D to PxVec3 for PhysX
        PxVec3 segmentDir((float)segmentDirection.x(), (float)segmentDirection.y(), (float)segmentDirection.z());
        
        // Create rotation matrix to align segment with its direction
        PxQuat rotation;
        if (segmentDir.magnitudeSquared() > 1e-6f) {
            // Normalize the direction vector
            segmentDir.normalize();
            
            // Create a rotation from the default X-axis to the segment direction
            PxVec3 defaultAxis(1, 0, 0); // Default X-axis
            PxVec3 cross = defaultAxis.cross(segmentDir);
            
            if (cross.magnitudeSquared() > 1e-6f) {
                cross.normalize();
                float dot = defaultAxis.dot(segmentDir);
                float angle = acosf(PxClamp(dot, -1.0f, 1.0f));
                rotation = PxQuat(angle, cross);
            } else {
                // Vectors are parallel or anti-parallel
                float dot = defaultAxis.dot(segmentDir);
                if (dot > 0) {
                    rotation = PxQuat(0, 0, 0, 1); // Identity
                } else {
                    rotation = PxQuat(PxPi, PxVec3(0, 0, 1)); // 180 degree rotation around Z
                }
            }
        } else {
            rotation = PxQuat(0, 0, 0, 1); // Identity rotation
        }
        
        PxTransform pose(segmentPos, rotation);
        PxRigidDynamic* actor = m_physics->createRigidDynamic(pose);
        actor->attachShape(*shape);
        shape->release(); // actor now owns the shape
        
        // Set mass and configure actor
        float segmentMass = 1.0f; // Default mass per segment
        actor->setMass(segmentMass);
        configureDynamicActor(actor);
        
        // Make segments kinematic so they're not affected by gravity
        // This prevents them from falling due to gravity
        actor->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
        
        // Add to scene
        m_scene->addActor(*actor);
        chainActors.push_back(actor);
        
        // Create visual geometry for this segment
        auto segmentNode = std::make_shared<CadNode>();
        segmentNode->name = connectionNode->name + "_Segment_" + std::to_string(i);
        segmentNode->type = CadNodeType::Custom;
        segmentNode->visible = true;
        segmentNode->color = CADNodeColor::fromSRGB(255, 100, 100); // Red color for segments
        
        qDebug() << "[PhysX] SIMULATION - Creating segment node:" << QString::fromStdString(segmentNode->name) << "for connection:" << QString::fromStdString(connectionNode->name);
        
        // Create a simple custom data for visualization
        auto customData = std::make_shared<CustomNodeData>();
        segmentNode->data = customData;
        
        // Position segments at the same global coordinates as the preview
        // Since the preview works correctly, we should position segments at the same global coordinates
        qDebug() << "[PhysX] Segment" << i << "global center:" << segmentCenter;
        
        // Set the segment position directly at the global coordinates (same as preview)
        gp_Trsf trsf;
        trsf.SetTranslation(gp_Vec(segmentCenter.x(), segmentCenter.y(), segmentCenter.z()));
        segmentNode->loc = TopLoc_Location(trsf);
        
        // Add to the root node so segments can be positioned at global coordinates
        // This ensures segments appear at the same position as the preview
        actualRootNode->children.push_back(segmentNode);
        segmentNodes.push_back(segmentNode);
        
        // Set parent pointer to root node
        segmentNode->parent = actualRootNode;
        
        // Store mapping for position updates
        actorToSegmentNode[actor] = segmentNode;
        
        // Add to tracking map to prevent duplicates
        m_parent->m_connectionSegments[connectionNode].push_back(segmentNode);
        
        qDebug() << "[PhysX] SIMULATION - Added segment node to root node. Total children:" << actualRootNode->children.size();
        
        qDebug() << "[PhysX] Created segment" << i << "at pos:" << segmentPos.x << segmentPos.y << segmentPos.z
                 << "length:" << segmentLength << "direction:" << segmentDir.x << segmentDir.y << segmentDir.z
                 << "(using shared segment generation)";
    }

    // Create revolute joints between segments
    for (size_t i = 0; i < chainActors.size() - 1; ++i) {
        PxRigidDynamic* actor1 = chainActors[i];
        PxRigidDynamic* actor2 = chainActors[i + 1];
        
        // Calculate joint position (midpoint between segments)
        PxTransform pose1 = actor1->getGlobalPose();
        PxTransform pose2 = actor2->getGlobalPose();
        PxVec3 jointPos = (pose1.p + pose2.p) * 0.5f;
        
        // Create revolute joint with proper local frames
        PxTransform localFrame1 = PxTransform(PxVec3(0, 0, 0)); // Local frame at actor1 origin
        PxTransform localFrame2 = PxTransform(PxVec3(0, 0, 0)); // Local frame at actor2 origin
        
        PxRevoluteJoint* joint = PxRevoluteJointCreate(*m_physics, actor1, localFrame1, actor2, localFrame2);
        
        if (joint) {
            // Configure joint limits if needed - simplified for compatibility
            // Note: Joint configuration may vary between PhysX versions
            // joint->setLimit(PxJointLimitPair(-PxPi, PxPi)); // Allow full rotation
            // joint->setRevoluteFlag(PxRevoluteJointFlag::eLIMIT_ENABLED, true);
            
            // Set joint axis (perpendicular to segment direction)
            PxVec3 jointAxis(0, 0, 1); // Z-axis for rotation
            // joint->setLocalPose(PxJointActorIndex::eACTOR0, PxTransform(PxVec3(0, 0, 0), PxQuat(0, 0, 0, 1)));
            // joint->setLocalPose(PxJointActorIndex::eACTOR1, PxTransform(PxVec3(0, 0, 0), PxQuat(0, 0, 0, 1)));
            
            qDebug() << "[PhysX] Created revolute joint between segments" << i << "and" << (i + 1);
        } else {
            qDebug() << "[PhysX] Failed to create revolute joint between segments" << i << "and" << (i + 1);
        }
    }

    // Store mappings for segment nodes only (don't map connection node to avoid position updates)
    if (!chainActors.empty()) {
        // Map each segment actor to its corresponding segment node
        for (size_t i = 0; i < chainActors.size() && i < segmentNodes.size(); ++i) {
            actorToNode[chainActors[i]] = segmentNodes[i].get();
            nodeToActor[segmentNodes[i].get()] = chainActors[i];
        }
        
        qDebug() << "[PhysX] Successfully created drag chain with" << chainActors.size() << "segments and" 
                 << (chainActors.size() - 1) << "joints for connection:" << QString::fromStdString(connectionNode->name);
        
        // Debug: Check if segment nodes were created
        qDebug() << "[PhysX] Root node has" << actualRootNode->children.size() << "children (including segment nodes)";
        for (size_t i = 0; i < actualRootNode->children.size(); ++i) {
            const auto& child = actualRootNode->children[i];
            if (child && child->name.find("Segment") != std::string::npos) {
                qDebug() << "[PhysX] Segment child" << i << ":" << QString::fromStdString(child->name) 
                         << "Type:" << static_cast<int>(child->type)
                         << "Visible:" << child->visible;
            }
        }
    } else {
        qDebug() << "[PhysX] WARNING: No chain actors created for connection:" << QString::fromStdString(connectionNode->name);
    }
    
    qDebug() << "[PhysX] SIMULATION - createDragChainFromConnection finished for:" << QString::fromStdString(connectionNode->name);
    qDebug() << "[PhysX] SIMULATION - Final root node children count:" << actualRootNode->children.size();
    for (size_t i = 0; i < actualRootNode->children.size(); ++i) {
        const auto& child = actualRootNode->children[i];
        if (child && child->name.find("Segment") != std::string::npos) {
            qDebug() << "[PhysX] SIMULATION - Final segment child" << i << ":" << QString::fromStdString(child->name) 
                     << "Type:" << static_cast<int>(child->type);
        }
    }
}

SimulationManager::SimulationManager(const std::shared_ptr<CadNode>& m_CadNodeRootIn) :
    materialManager(new MaterialManager(nullptr)),
    m_CadNodeRoot(m_CadNodeRootIn) {
    std::cout << "[SimulationManager::ctor] Received root node type: " << static_cast<int>(m_CadNodeRootIn->type)
              << ", name: " << m_CadNodeRootIn->name << std::endl;
    qDebug() << "[SimulationManager::ctor] Received root node type:" << static_cast<int>(m_CadNodeRootIn->type)
             << ", name:" << QString::fromStdString(m_CadNodeRootIn->name);
    // Initialize state
    m_currentState.time = 0.0f;
    m_currentState.isPaused = true;
    m_currentState.stepCount = 0;
}

void SimulationManager::buildPhysXSceneFromNodes() {
    qDebug() << "[buildPhysXSceneFromNodes] Build physics scene called";
    
    // Set scene building flag to prevent GUI rendering during this process
    m_sceneBuilding = true;
    
    m_nodeToActor.clear();
    m_actorToNode.clear();
    m_actorToSegmentNode.clear(); // Clear segment mappings
    
    if (!m_physXEngine) {
        m_physXEngine = std::make_unique<PhysXEngine>(this);
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

    qDebug() << "[buildPhysXSceneFromNodes] Found" << physicsNodes.size() << "physics nodes for simulation";
    
    // Count different types of nodes
    int physicsCount = 0;
    int connectionCount = 0;
    for (const auto& node : physicsNodes) {
        qDebug() << "[buildPhysXSceneFromNodes] Node:" << QString::fromStdString(node->name) 
                 << "type:" << static_cast<int>(node->type);
        if (node->type == CadNodeType::Physics) {
            physicsCount++;
        } else if (node->type == CadNodeType::Connection) {
            connectionCount++;
        }
    }
    
    qDebug() << "[buildPhysXSceneFromNodes] Physics nodes:" << physicsCount << "Connection nodes:" << connectionCount;

    // Validate and fix connection nodes before processing
    for (CadNode* node : physicsNodes) {
        if (node->type == CadNodeType::Connection) {
            auto connectionData = node->asConnection();
            if (!connectionData) {
                qDebug() << "[buildPhysXSceneFromNodes] WARNING: Connection node has no data, creating default data...";
                
                // Create default connection data
                auto defaultData = std::make_shared<ConnectionNodeData>();
                defaultData->connectionType = ConnectionNodeData::ConnectionType::DragChain;
                defaultData->pitchLength = 300.0;
                defaultData->maxBendRadius = 50.0;
                defaultData->segmentCount = 3;
                defaultData->isFlexible = true;
                
                // Create default settings
                QJsonObject defaultSettings;
                defaultSettings["connectionName"] = QString::fromStdString(node->name);
                defaultSettings["connectionType"] = 0; // DragChain
                defaultSettings["isFlexible"] = true;
                defaultSettings["maxBendRadius"] = 50.0;
                defaultSettings["pitchLength"] = 300.0;
                
                // Add default control points
                QJsonArray controlPointsArray;
                QJsonObject controlPoint;
                controlPoint["x"] = 100.0;
                controlPoint["y"] = 0.0;
                controlPoint["z"] = 50.0;
                controlPointsArray.append(controlPoint);
                defaultSettings["controlPoints"] = controlPointsArray;
                
                // Add default visualization settings
                QJsonObject visualization;
                QJsonObject startPoint;
                startPoint["x"] = 0.0;
                startPoint["y"] = 0.0;
                startPoint["z"] = 0.0;
                visualization["startPoint"] = startPoint;
                
                QJsonObject endPoint;
                endPoint["x"] = 200.0;
                endPoint["y"] = 0.0;
                endPoint["z"] = 0.0;
                visualization["endPoint"] = endPoint;
                defaultSettings["visualization"] = visualization;
                
                QJsonDocument doc(defaultSettings);
                defaultData->creationSettingsJson = QString::fromUtf8(doc.toJson(QJsonDocument::Compact));
                
                node->data = defaultData;
                qDebug() << "[buildPhysXSceneFromNodes] Created default connection data for node:" << QString::fromStdString(node->name);
            }
        }
    }
    
    // Delegate scene building to PhysXEngine
    m_physXEngine->buildSceneFromNodes(physicsNodes, m_nodeToActor, m_actorToNode, m_actorToSegmentNode);

    // Check if any connection nodes were processed
    bool connectionNodesFound = false;
    for (CadNode* node : physicsNodes) {
        if (node->type == CadNodeType::Connection) {
            connectionNodesFound = true;
            
            // Debug: Check the connection node data
            auto connectionData = node->asConnection();
            if (connectionData) {
                qDebug() << "[buildPhysXSceneFromNodes] Connection node:" << QString::fromStdString(node->name)
                         << "Type:" << static_cast<int>(connectionData->connectionType)
                         << "Pitch length:" << connectionData->pitchLength
                         << "Max bend radius:" << connectionData->maxBendRadius
                         << "Segment count:" << connectionData->segmentCount
                         << "Settings JSON length:" << connectionData->creationSettingsJson.length();
                
                // Check if the settings JSON is valid
                if (!connectionData->creationSettingsJson.isEmpty()) {
                    QJsonDocument doc = QJsonDocument::fromJson(connectionData->creationSettingsJson.toUtf8());
                    if (doc.isObject()) {
                        QJsonObject settings = doc.object();
                        qDebug() << "[buildPhysXSceneFromNodes] Settings JSON is valid, contains:"
                                 << settings.keys();
                    } else {
                        qDebug() << "[buildPhysXSceneFromNodes] WARNING: Settings JSON is not valid!";
                    }
                } else {
                    qDebug() << "[buildPhysXSceneFromNodes] WARNING: Settings JSON is empty!";
                }
            } else {
                qDebug() << "[buildPhysXSceneFromNodes] WARNING: Connection node has no connection data!";
            }
        }
    }
    
    if (!connectionNodesFound) {
        qDebug() << "[buildPhysXSceneFromNodes] No connection nodes found in normal processing, trying force creation...";
        forceDragChainCreation();
        
        // If still no connection nodes found, create a test one
        if (m_actorToSegmentNode.empty()) {
            qDebug() << "[buildPhysXSceneFromNodes] Still no connection nodes found, creating test connection node...";
            createTestConnectionNode();
            
            // Try force creation again with the test node
            forceDragChainCreation();
        }
    } else {
        // Connection nodes were found, check if they were actually processed
        int processedConnections = 0;
        for (const auto& pair : m_nodeToActor) {
            if (pair.first->type == CadNodeType::Connection) {
                processedConnections++;
            }
        }
        
        qDebug() << "[buildPhysXSceneFromNodes] Found" << connectionNodesFound << "connection nodes, processed" << processedConnections;
        
        // Only call force creation if no connections were actually processed
        if (processedConnections == 0) {
            qDebug() << "[buildPhysXSceneFromNodes] Connection nodes found but not processed, trying force creation...";
            forceDragChainCreation();
        } else {
            qDebug() << "[buildPhysXSceneFromNodes] Connection nodes were successfully processed, skipping force creation to avoid duplicates";
        }
    }

    qDebug() << "[buildPhysXSceneFromNodes] Started simulation with" << physicsNodes.size() << "nodes";
    qDebug() << "[buildPhysXSceneFromNodes] Created" << m_nodeToActor.size() << "actors";
    qDebug() << "[buildPhysXSceneFromNodes] Created" << m_actorToSegmentNode.size() << "segment nodes";
    
    // Debug: Check if any connection nodes were actually processed
    int processedConnections = 0;
    for (const auto& pair : m_nodeToActor) {
        if (pair.first->type == CadNodeType::Connection) {
            processedConnections++;
        }
    }
    qDebug() << "[buildPhysXSceneFromNodes] Processed" << processedConnections << "connection nodes";
    
    // Scene building complete, but keep flag true until first simulation step
    // The flag will be reset in updateSimulation() after the first step
}

void SimulationManager::startSimulation() {
    if (m_running) {
        std::cout << "Simulation is already running" << std::endl;
        return;
    }

    // Debug output to verify which root node is being used
    qDebug() << "[startSimulation] Using root node type:" << static_cast<int>(m_CadNodeRoot->type)
             << ", name:" << QString::fromStdString(m_CadNodeRoot->name);

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
    menu->addSeparator();
    QAction* testDragChainAction = menu->addAction("Test Drag Chain Creation");
    QAction* forceDragChainAction = menu->addAction("Force Drag Chain Creation");
    QAction* createTestConnectionAction = menu->addAction("Create Test Connection Node");
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
    QObject::connect(testDragChainAction, &QAction::triggered, mainWindow, [this]() {
        this->testDragChainCreation();
    });
    QObject::connect(forceDragChainAction, &QAction::triggered, mainWindow, [this]() {
        this->forceDragChainCreation();
    });
    QObject::connect(createTestConnectionAction, &QAction::triggered, mainWindow, [this]() {
        this->createTestConnectionNode();
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
            
            // Get PhysX pose (same coordinate system as CAD)
            PxVec3 pos = physxPose.p;
            PxQuat quat = physxPose.q;

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
                         << "pos:" << pos.x << pos.y << pos.z;
            }
        }
        
        // Update drag chain segment positions
            qDebug() << "[PhysX] Updating" << m_actorToSegmentNode.size() << "drag chain segment positions";
    
    // Only log this every 60 frames to avoid spam
    static int frameCounter = 0;
    frameCounter++;
    if (frameCounter % 60 == 0) {
        qDebug() << "[PhysX] Frame" << frameCounter << "- Updating" << m_actorToSegmentNode.size() << "drag chain segments";
        
        // Also log connection node updates
        int connectionUpdates = 0;
        for (const auto& pair : m_nodeToActor) {
            if (pair.first->type == CadNodeType::Connection) {
                connectionUpdates++;
            }
        }
        qDebug() << "[PhysX] Frame" << frameCounter << "- Updating" << connectionUpdates << "connection nodes";
    }
    
    for (const auto& pair : m_actorToSegmentNode) {
            PxRigidActor* actor = pair.first;
            std::shared_ptr<CadNode> segmentNode = pair.second;
            if (!actor || !segmentNode) {
                qDebug() << "[PhysX] WARNING: Invalid actor or segment node in mapping";
                continue;
            }

            PxTransform physxPose = actor->getGlobalPose();
            
            // Get PhysX pose (same coordinate system as CAD)
            PxVec3 pos = physxPose.p;
            PxQuat quat = physxPose.q;

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

            // Store location in write buffer
            TopLoc_Location newLocation(trsf);
            writeBuffer->nodeLocations[segmentNode.get()] = newLocation;
            
            // Debug output for first few updates
            if (m_currentState.stepCount < 10) {
                qDebug() << "[PhysX] Updated segment node" << segmentNode->name.c_str() 
                         << "step:" << m_currentState.stepCount
                         << "pos:" << pos.x << pos.y << pos.z;
            }
            
            // Always log the first update to confirm segments are being updated
            if (m_currentState.stepCount == 1) {
                qDebug() << "[PhysX] First update - segment node" << segmentNode->name.c_str() 
                         << "position:" << pos.x << pos.y << pos.z;
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
    
    // Also collect Connection nodes for drag chain creation
    if (root->type == CadNodeType::Connection) {
        qDebug() << "[collectPhysicsNodes] Found Connection node:" << QString::fromStdString(root->name);
        
        // Check if the connection node has valid data
        auto connectionData = root->asConnection();
        if (connectionData) {
            qDebug() << "[collectPhysicsNodes] Connection data found - Type:" << static_cast<int>(connectionData->connectionType)
                     << "Pitch length:" << connectionData->pitchLength
                     << "Max bend radius:" << connectionData->maxBendRadius
                     << "Segment count:" << connectionData->segmentCount
                     << "Settings JSON length:" << connectionData->creationSettingsJson.length();
        } else {
            qDebug() << "[collectPhysicsNodes] WARNING: Connection node has no connection data!";
        }
        
        outNodes.push_back(root.get());
    }
    
    for (const auto& child : root->children) {
        collectPhysicsNodes(child, outNodes);
    }
}

void SimulationManager::testDragChainCreation()
{
    qDebug() << "[SimulationManager] Testing drag chain creation...";
    
    // Find all connection nodes in the tree
    std::vector<CadNode*> connectionNodes;
    std::function<void(const std::shared_ptr<CadNode>&)> findConnections = 
        [&](const std::shared_ptr<CadNode>& node) {
            if (node->type == CadNodeType::Connection) {
                connectionNodes.push_back(node.get());
                qDebug() << "[SimulationManager] Found connection node:" << QString::fromStdString(node->name);
            }
            for (const auto& child : node->children) {
                findConnections(child);
            }
        };
    
    findConnections(m_CadNodeRoot);
    
    qDebug() << "[SimulationManager] Found" << connectionNodes.size() << "connection nodes";
    
    if (connectionNodes.empty()) {
        qDebug() << "[SimulationManager] No connection nodes found for testing";
        qDebug() << "[SimulationManager] Creating a test connection node...";
        
        // Create a test connection node
        auto testConnectionNode = std::make_shared<CadNode>();
        testConnectionNode->name = "TestConnection";
        testConnectionNode->type = CadNodeType::Connection;
        testConnectionNode->visible = true; // Make sure it's visible
        
        // Create connection data
        auto connectionData = std::make_shared<ConnectionNodeData>();
        connectionData->connectionType = ConnectionNodeData::ConnectionType::DragChain;
        connectionData->pitchLength = 300.0;
        connectionData->maxBendRadius = 50.0;
        connectionData->segmentCount = 3;
        
        // Create test JSON settings
        QJsonObject settings;
        settings["connectionName"] = "TestConnection";
        settings["connectionType"] = 0; // DragChain
        settings["isFlexible"] = true;
        settings["maxBendRadius"] = 50.0;
        settings["pitchLength"] = 300.0;
        
        // Add control points
        QJsonArray controlPointsArray;
        QJsonObject controlPoint;
        controlPoint["x"] = 100.0;
        controlPoint["y"] = 0.0;
        controlPoint["z"] = 50.0;
        controlPointsArray.append(controlPoint);
        settings["controlPoints"] = controlPointsArray;
        
        // Add visualization settings
        QJsonObject visualization;
        QJsonObject startPoint;
        startPoint["x"] = 0.0;
        startPoint["y"] = 0.0;
        startPoint["z"] = 0.0;
        visualization["startPoint"] = startPoint;
        
        QJsonObject endPoint;
        endPoint["x"] = 200.0;
        endPoint["y"] = 0.0;
        endPoint["z"] = 0.0;
        visualization["endPoint"] = endPoint;
        settings["visualization"] = visualization;
        
        QJsonDocument doc(settings);
        connectionData->creationSettingsJson = QString::fromUtf8(doc.toJson(QJsonDocument::Compact));
        
        testConnectionNode->data = connectionData;
        
        qDebug() << "[SimulationManager] Created test connection node with settings:" << connectionData->creationSettingsJson;
        
            // Add the test connection node to the root
    const_cast<CadNode*>(m_CadNodeRoot.get())->children.push_back(testConnectionNode);
    
    // Make sure the test connection node is visible
    testConnectionNode->visible = true;
    testConnectionNode->color = CADNodeColor::fromSRGB(100, 100, 255); // Blue color
    
    qDebug() << "[SimulationManager] Added test connection node to root tree";
    qDebug() << "[SimulationManager] Test connection node visible:" << testConnectionNode->visible;
    qDebug() << "[SimulationManager] Test connection node color:" << testConnectionNode->color.r << testConnectionNode->color.g << testConnectionNode->color.b;
        
        // Test the drag chain creation with this test node
        if (!m_physXEngine) {
            qDebug() << "[SimulationManager] PhysX engine not initialized, creating one for testing";
            m_physXEngine = std::make_unique<PhysXEngine>();
            if (!m_physXEngine->initializePhysX(m_CadNodeRoot)) {
                qDebug() << "[SimulationManager] Failed to initialize PhysX engine for testing";
                return;
            }
        }
        
        // Create temporary mappings for testing
        std::unordered_map<CadNode*, PxRigidDynamic*> nodeToActor;
        std::unordered_map<PxRigidDynamic*, CadNode*> actorToNode;
        std::unordered_map<PxRigidDynamic*, std::shared_ptr<CadNode>> actorToSegmentNode; // Pass empty map for testing
        
        // Test drag chain creation with the test node
        m_physXEngine->createDragChainFromConnection(testConnectionNode.get(), nodeToActor, actorToNode, actorToSegmentNode, this);
        
        qDebug() << "[SimulationManager] Test completed. Created" << nodeToActor.size() << "actors";
        return;
    }
    
    // Test the first connection node
    CadNode* testNode = connectionNodes[0];
    qDebug() << "[SimulationManager] Testing drag chain creation for node:" << QString::fromStdString(testNode->name);
    
    if (!m_physXEngine) {
        qDebug() << "[SimulationManager] PhysX engine not initialized, creating one for testing";
        m_physXEngine = std::make_unique<PhysXEngine>();
        if (!m_physXEngine->initializePhysX(m_CadNodeRoot)) {
            qDebug() << "[SimulationManager] Failed to initialize PhysX engine for testing";
            return;
        }
    }
    
    // Create temporary mappings for testing
    std::unordered_map<CadNode*, PxRigidDynamic*> nodeToActor;
    std::unordered_map<PxRigidDynamic*, CadNode*> actorToNode;
    std::unordered_map<PxRigidDynamic*, std::shared_ptr<CadNode>> actorToSegmentNode; // Pass empty map for testing
    
    // Test drag chain creation
            m_physXEngine->createDragChainFromConnection(testNode, nodeToActor, actorToNode, actorToSegmentNode, this);
    
    qDebug() << "[SimulationManager] Test completed. Created" << nodeToActor.size() << "actors";
}

void SimulationManager::forceDragChainCreation()
{
    qDebug() << "[SimulationManager] Force drag chain creation called...";
    
    if (!m_physXEngine) {
        qDebug() << "[SimulationManager] PhysX engine not initialized, creating one";
        m_physXEngine = std::make_unique<PhysXEngine>();
        if (!m_physXEngine->initializePhysX(m_CadNodeRoot)) {
            qDebug() << "[SimulationManager] Failed to initialize PhysX engine";
            return;
        }
    }
    
    // Find all connection nodes in the tree
    std::vector<CadNode*> connectionNodes;
    std::function<void(const std::shared_ptr<CadNode>&)> findConnections = 
        [&](const std::shared_ptr<CadNode>& node) {
            if (node->type == CadNodeType::Connection) {
                connectionNodes.push_back(node.get());
                qDebug() << "[SimulationManager] Found connection node:" << QString::fromStdString(node->name);
            }
            for (const auto& child : node->children) {
                findConnections(child);
            }
        };
    
    findConnections(m_CadNodeRoot);
    
    qDebug() << "[SimulationManager] Found" << connectionNodes.size() << "connection nodes for force creation";
    
    if (connectionNodes.empty()) {
        qDebug() << "[SimulationManager] No connection nodes found for force creation";
        return;
    }
    
    // Create drag chains for all connection nodes
    for (CadNode* connectionNode : connectionNodes) {
        qDebug() << "[SimulationManager] Force creating drag chain for:" << QString::fromStdString(connectionNode->name);
        m_physXEngine->createDragChainFromConnection(connectionNode, m_nodeToActor, m_actorToNode, m_actorToSegmentNode, this);
    }
    
    qDebug() << "[SimulationManager] Force drag chain creation completed. Total actors:" << m_nodeToActor.size();
    
    // Force update the OpenGL viewer to show the new drag chains
    // Note: This would need to be called from the GUI thread
    qDebug() << "[SimulationManager] Drag chain creation completed - check OpenGL viewer for visual updates";
    
    // Mark that we have updates to process
    m_buffersSwapped = true;
    
    // Force the OpenGL viewer to refresh its cache
    qDebug() << "[SimulationManager] Marked buffers as dirty, OpenGL viewer should refresh";
}

void SimulationManager::createTestConnectionNode()
{
    qDebug() << "[SimulationManager] Creating test connection node...";
    
    // Create a test connection node
    auto testConnectionNode = std::make_shared<CadNode>();
    testConnectionNode->name = "TestConnection";
    testConnectionNode->type = CadNodeType::Connection;
    testConnectionNode->visible = true;
    
    // Create connection data
    auto connectionData = std::make_shared<ConnectionNodeData>();
    connectionData->connectionType = ConnectionNodeData::ConnectionType::DragChain;
    connectionData->pitchLength = 300.0;
    connectionData->maxBendRadius = 50.0;
    connectionData->segmentCount = 3;
    
    // Create test JSON settings
    QJsonObject settings;
    settings["connectionName"] = "TestConnection";
    settings["connectionType"] = 0; // DragChain
    settings["isFlexible"] = true;
    settings["maxBendRadius"] = 50.0;
    settings["pitchLength"] = 300.0;
    
    // Add control points
    QJsonArray controlPointsArray;
    QJsonObject controlPoint;
    controlPoint["x"] = 100.0;
    controlPoint["y"] = 0.0;
    controlPoint["z"] = 50.0;
    controlPointsArray.append(controlPoint);
    settings["controlPoints"] = controlPointsArray;
    
    // Add visualization settings
    QJsonObject visualization;
    QJsonObject startPoint;
    startPoint["x"] = 0.0;
    startPoint["y"] = 0.0;
    startPoint["z"] = 0.0;
    visualization["startPoint"] = startPoint;
    
    QJsonObject endPoint;
    endPoint["x"] = 200.0;
    endPoint["y"] = 0.0;
    endPoint["z"] = 0.0;
    visualization["endPoint"] = endPoint;
    settings["visualization"] = visualization;
    
    QJsonDocument doc(settings);
    connectionData->creationSettingsJson = QString::fromUtf8(doc.toJson(QJsonDocument::Compact));
    
    testConnectionNode->data = connectionData;
    
    // Add to the root node
    m_CadNodeRoot->children.push_back(testConnectionNode);
    
    qDebug() << "[SimulationManager] Created test connection node with settings:" << connectionData->creationSettingsJson;
    qDebug() << "[SimulationManager] Added test connection node to tree. Total children:" << m_CadNodeRoot->children.size();
}
