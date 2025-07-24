#ifndef PHYSXENGINE_H
#define PHYSXENGINE_H

#include "irobodk.h"
#include "iitem.h"
#include "IPhysicsEngine.h"
#include "PxPhysicsAPI.h"
#include "foundation/PxVec3.h"

// Forward declarations
class ObjectPropertiesManager;

using namespace physx;

/**
 * @brief PhysX implementation of the physics engine interface
 */
class PhysXEngine : public IPhysicsEngine
{
    Q_OBJECT

public:
    explicit PhysXEngine(RoboDK* rdk, QObject* parent = nullptr);
    
    // Constructor that accepts existing PhysX components (for compatibility with old system)
    PhysXEngine(RoboDK* rdk, 
                PxFoundation* foundation, 
                PxPhysics* physics, 
                PxScene* scene, 
                PxMaterial* material,
                PxRigidStatic* groundPlane,
                QObject* parent = nullptr);
    
    ~PhysXEngine() override;

    // IPhysicsEngine implementation
    bool initialize() override;
    void cleanup() override;
    
    bool addObject(Item item) override;
    bool removeObject(Item item) override;
    bool isObjectInSimulation(Item item) const override;
    
    // Robot management
    bool addRobot(Item robot) override;
    bool removeRobot(Item robot) override;
    bool isRobotInSimulation(Item robot) const override;
    void updateRobotJoints(Item robot) override;
    
    bool convertToKinematic(Item item) override;
    bool convertToDynamic(Item item) override;
    bool isObjectGrabbedByRobot(Item item) const override;
    
    // Soft body management
    bool addSoftBody(Item item, const SoftBodyConfig& config) override;
    bool removeSoftBody(Item item) override;
    bool isSoftBodyInSimulation(Item item) const override;
    bool hasSoftBodyDebugData(Item item) const;
    void updateSoftBodyFromRoboDK(Item item) override;
    void updateRoboDKFromSoftBody(Item item) override;
    
    // Detect if an object is being grabbed by the robot (regardless of simulation state)
    bool detectRobotGrab(Item item) const;
    
    void updatePhysicsFromRoboDK(Item item) override;
    void updateRoboDKFromPhysics(Item item) override;
    
    void stepSimulation(float deltaTime) override;
    
    // Extended simulation step that includes sleep detection and actor management
    void stepSimulationExtended(float deltaTime);
    
    void setGravity(const PxVec3& gravity) override;
    PxVec3 getGravity() const override;
    
    // Material management
    bool setObjectMaterial(Item item, float staticFriction, float dynamicFriction, float restitution) override;
    
    // Configuration methods for SceneConfigurationDialog
    void setSolverIterations(int positionIterations, int velocityIterations);
    void getSolverIterations(int& positionIterations, int& velocityIterations) const;
    void setGlobalMaterialProperties(float staticFriction, float dynamicFriction, float restitution);
    void getGlobalMaterialProperties(float& staticFriction, float& dynamicFriction, float& restitution) const;
    void setPCMEnabled(bool enabled);
    bool isPCMEnabled() const;
    void setStabilizationEnabled(bool enabled);
    bool isStabilizationEnabled() const;
    void setContactOffset(float offset);
    float getContactOffset() const;
    void setRestOffset(float offset);
    float getRestOffset() const;
    void setSleepThreshold(float threshold);
    float getSleepThreshold() const;
    void setStabilizationThreshold(float threshold);
    float getStabilizationThreshold() const;
    void setCCDEnabled(bool enabled);
    bool isCCDEnabled() const;
    void setWakeDistance(float distance);
    float getWakeDistance() const;
    
    int getActorCount() const override;
    void enableDebugVisualization(bool enable) override;
    bool isDebugVisualizationEnabled() const override;
    
    QString getEngineName() const override;
    QString getEngineVersion() const override;
    
    // Manual object validation (for external use)
    void validateObjects() override;
    
    // Debug visualization methods (public for plugin access)
    void renderDebugGeometry();
    void drawCollisionGeometry();
    void drawDeformableMeshes(); // Always draw deformable mesh wireframes
    void drawActorGeometry(PxRigidActor* actor, const PxTransform& pose, bool isSleeping = false, bool isRobot = false);
    void drawShapeGeometry(const PxShape* shape, const PxTransform& pose, bool isSleeping = false, bool isRobot = false);
    void drawTriangleMesh(const PxTriangleMesh* mesh, const PxTransform& pose, bool isSleeping = false, bool isRobot = false);
    void drawConvexMesh(const PxConvexMesh* mesh, const PxTransform& pose, bool isSleeping = false, bool isRobot = false);
    void drawBoxGeometry(const PxBoxGeometry& box, const PxTransform& pose, bool isSleeping = false, bool isRobot = false);
    void drawGroundPlane();
    
    // Static member access methods (moved from global variables)
    static void setRoboDK(RoboDK* rdk);
    static RoboDK* getRoboDK();
    static void setLoadingDone(bool done);
    static bool isLoadingDone();
    static void setStackZ(PxReal z);
    static PxReal getStackZ();
    
    // Object properties manager access
    void setObjectPropertiesManager(ObjectPropertiesManager* manager);

private:
    // PhysX components (moved from global variables)
    PxDefaultAllocator m_allocator;
    PxDefaultErrorCallback m_errorCallback;
    PxFoundation* m_foundation;
    PxPhysics* m_physics;
    PxCpuDispatcher* m_dispatcher;
    PxScene* m_scene;
    PxMaterial* m_material;
    PxMaterial* m_kinematicMaterial;  // Low-friction material for kinematic actors (robots)
    PxPvd* m_pvd;
    PxRigidStatic* m_groundPlane;
    PxCudaContextManager* m_cudaContextManager;  // CUDA context for deformable volumes
    
    // Global variables for physics simulation
    static PxReal m_stackZ;
    static RoboDK* m_rdk;
    static bool m_isLoadingDone;
    
    // Object tracking (moved from global gPhysXActors)
    struct PhysXObjectData {
        PxRigidActor* actor;
        bool isDynamic;
        bool isKinematic;
        bool isGrabbedByRobot;
        
        // Default constructor for std::map
        PhysXObjectData() : actor(nullptr), isDynamic(false), isKinematic(false), isGrabbedByRobot(false) {}
        
        // Constructor with parameters
        PhysXObjectData(PxRigidActor* a, bool dynamic = false) 
            : actor(a), isDynamic(dynamic), isKinematic(!dynamic), isGrabbedByRobot(false) {}
    };
    
    std::map<Item, PhysXObjectData> m_objects;
    
    // Robot tracking
    struct RobotData {
        std::map<int, PxRigidDynamic*> jointActors; // joint index -> actor
        bool isInSimulation;
        
        RobotData() : isInSimulation(false) {}
    };
    std::map<Item, RobotData> m_robots;
    
    // Soft body tracking
    struct SoftBodyData {
        PxDeformableVolume* softBody; // Updated to use PxDeformableVolume*
        SoftBodyConfig config;
        bool isInSimulation;
        
        // Initial transform used when creating the deformable volume
        PxTransform initialTransform;
        
        // Pinned host buffers for GPU data management
        PxVec4* simPositionInvMassPinned;
        PxVec4* simVelocityPinned;
        PxVec4* collPositionInvMassPinned;
        PxVec4* restPositionPinned;
        
        // Auxiliary data for deformable volume
        PxDeformableVolumeAuxData* auxData;
        
        // Mesh objects that must remain valid for the lifetime of the soft body
        PxDeformableVolumeMesh* deformableVolumeMesh;
        PxShape* shape;
        PxDeformableVolumeMaterial* material;
        
        // Debug mesh data for visualization
        std::vector<PxVec3> debugMeshVertices;
        std::vector<PxU32> debugMeshIndices;
        Mat debugMeshPose;
        bool hasDebugMesh;
        
        // Attachment point data for debug visualization
        std::vector<PxVec3> attachmentPoints;
        std::vector<PxVec3> rigidBodyPoints;
        bool hasAttachmentPoints;
        
        SoftBodyData() : softBody(nullptr), isInSimulation(false), 
                        initialTransform(PxTransform(PxVec3(0,0,0), PxQuat(PxIdentity))),
                        simPositionInvMassPinned(nullptr), simVelocityPinned(nullptr),
                        collPositionInvMassPinned(nullptr), restPositionPinned(nullptr),
                        auxData(nullptr), deformableVolumeMesh(nullptr), shape(nullptr), material(nullptr),
                        hasDebugMesh(false), hasAttachmentPoints(false) {}
    };
    std::map<Item, SoftBodyData> m_softBodies;
    
    // Configuration
    PxVec3 m_gravity;
    bool m_debugVisualizationEnabled;
    
    // Scene configuration settings
    int m_solverPositionIterations;
    int m_solverVelocityIterations;
    float m_globalStaticFriction;
    float m_globalDynamicFriction;
    float m_globalRestitution;
    bool m_pcmEnabled;
    bool m_stabilizationEnabled;
    float m_contactOffset;
    float m_restOffset;
    float m_sleepThreshold;
    float m_stabilizationThreshold;
    bool m_ccdEnabled;
    float m_wakeDistance;
    
    // Flags to track ownership
    bool m_ownsFoundation;
    bool m_ownsPhysics;
    bool m_ownsScene;
    bool m_ownsMaterial;
    bool m_ownsKinematicMaterial;
    bool m_ownsGroundPlane;
    
    // Object properties manager for center of mass visualization
    ObjectPropertiesManager* m_objectPropertiesManager;
    
    // Global cooking parameters for all mesh creation
    PxCookingParams m_globalCookingParams;
    
    // Helper methods
    bool initializePhysX();
    void cleanupPhysX();
    bool createMaterial();
    bool createKinematicMaterial();
    void setupContactFilter();
    bool createGroundPlane();
    void configureDynamicActor(PxRigidDynamic* actor);
    void initializeDefaults();
    void wakeUpNearbyDynamicObjects(PxRigidDynamic* kinematicActor, const PxTransform& newPose);
    
    // Object validation and cleanup
    void validateAndRemoveInvalidObjects();
    
    // Custom contact filter
    static PxFilterFlags simulationFilterShader(
        PxFilterObjectAttributes attributes0, PxFilterData filterData0,
        PxFilterObjectAttributes attributes1, PxFilterData filterData1,
        PxPairFlags& pairFlags, const void* constantBlock, PxU32 constantBlockSize);
    
    // Coordinate conversion helpers
    PxTransform convertRoboDKPoseToPhysX(const Mat& robodkPose);
    Mat convertPhysXPoseToRoboDK(const PxTransform& physxPose);
    
    // STL coordinate system conversion (for meshes loaded from STL files)
    PxTransform convertRoboDKPoseToPhysX_STL(const Mat& robodkPose);
    Mat convertPhysXPoseToRoboDK_STL(const PxTransform& physxPose);
    
    // Helper conversion functions
    static PxVec3 convertSTLToPhysX(const PxVec3& stlVertex);
    static PxVec3 convertPhysXToSTL(const PxVec3& physxVertex);
    static Mat quaternionToMatrix(const PxQuat& q);
    
    // Mesh extraction helpers
    static tMatrix2D* extractFromTriangleMesh(const PxTriangleMesh* mesh);
    static tMatrix2D* extractFromConvexMesh(const PxConvexMesh* mesh);
    static tMatrix2D* createBoxMesh(PxVec3 halfExtents);
    static tMatrix2D* extractShapeMesh(const PxShape* shape);
    
    // Validation helpers for debug visualization
    static bool isValidVector(const PxVec3& vec);
    static bool isValidTransform(const PxTransform& transform);
    void drawCenterOfMass();
    
    // Soft body helper methods
    bool extractGeometryFromItem(Item item, std::vector<PxVec3>& vertices, std::vector<PxU32>& indices);
    void updateDeformablePose(Item item, const PxTransform& pose);
    void drawDeformableGeometry(PxDeformableVolume* softBody, const PxTransform& initialTransform, bool isSleeping = false, bool drawCollisionMesh = false);
    void drawTetrahedronMesh(PxTetrahedronMesh* tetMesh, const PxTransform& pose, bool isSleeping = false);
    void drawTransformedTetrahedronMesh(PxTetrahedronMesh* tetMesh, const PxVec4* transformedVertices, PxU32 numVertices, bool isSleeping = false);
    void drawDeformableCollisionMesh(PxDeformableVolume* softBody, const PxTransform& initialTransform, bool isSleeping = false);
    
    // Stress calculation helper
    float calculateTetrahedronStress(const PxVec3& restV0, const PxVec3& restV1, const PxVec3& restV2, const PxVec3& restV3,
                                   const PxVec3& deformedV0, const PxVec3& deformedV1, const PxVec3& deformedV2, const PxVec3& deformedV3);
    void createCylindricalSurfaceMesh(float radius, float length, int numSegments, 
                                     std::vector<PxVec3>& vertices, std::vector<PxU32>& indices);
    void createSimpleCylindricalMesh(float radius, float length, int numSegments,
                                     std::vector<PxVec3>& vertices, std::vector<PxU32>& indices);
    void createBasicCylindricalMesh(float radius, float length, int numSegments,
                                     std::vector<PxVec3>& vertices, std::vector<PxU32>& indices);
    void createSimpleCylindricalMeshV2(float radius, float length, int numSegments,
                                     std::vector<PxVec3>& vertices, std::vector<PxU32>& indices);
    void createSimpleBoxMesh(float width, float height, 
                           PxArray<PxVec3>& vertices, PxArray<PxU32>& indices);
    void createSimpleTetrahedronMesh(float size, 
                                   std::vector<PxVec3>& vertices, std::vector<PxU32>& indices);
    bool validateMeshQuality(const std::vector<PxVec3>& vertices, const std::vector<PxU32>& indices);
    void visualizeMeshInRoboDK(const std::vector<PxVec3>& vertices, const std::vector<PxU32>& indices, const Mat& pose);
    void drawDebugMeshes();
    void cleanupFailedSoftBody(Item item);
    void drawCollisionBounds();
    
    // Soft body attachment methods
    void convertCollisionToSim(PxDeformableVolume* deformableVolume, PxU32* tetId, PxVec4* barycentric, PxU32 size);
    void attachSoftBodyToOverlappingRigidBodies(PxDeformableVolume* softBody, const PxTransform& softBodyTransform, SoftBodyData& softBodyData);
    void drawSoftBodyAttachmentPoints();
    
    // Attachment-specific collision control methods
    bool disableSoftBodyCollisionWithSurface(Item softBodyItem, Item attachmentSurfaceItem) override;
    bool enableSoftBodyCollisionWithSurface(Item softBodyItem, Item attachmentSurfaceItem) override;

};

    // Helper functions (declared here but implemented in cpp file)
    bool LoadBinarySTL(const std::string& filename, std::vector<PxVec3>& outVertices, std::vector<PxU32>& outIndices);
    PxConvexMesh* CreateConvexMesh(PxPhysics* physics, const std::vector<PxVec3>& vertices, const PxCookingParams& cookingParams);
    PxVec3 convertSTLToPhysX(const PxVec3& stlVertex);
    PxTriangleMesh* CreateTriangleMesh(PxPhysics* physics, const std::vector<PxVec3>& vertices, const std::vector<PxU32>& indices, const PxCookingParams& cookingParams);
    
    // VHACD-based convex decomposition
    std::vector<PxConvexMesh*> CreateVHACDConvexMeshes(PxPhysics* physics, const std::vector<PxVec3>& vertices, const std::vector<PxU32>& indices, const PxCookingParams& cookingParams, bool isRobotMesh = false);

#endif // PHYSXENGINE_H 
