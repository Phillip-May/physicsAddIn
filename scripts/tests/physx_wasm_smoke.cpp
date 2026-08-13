#include <PxPhysicsAPI.h>

#include <cmath>
#include <cstdio>

using namespace physx;

int main() {
    PxDefaultAllocator allocator;
    PxDefaultErrorCallback errors;
    PxFoundation* foundation = PxCreateFoundation(PX_PHYSICS_VERSION, allocator, errors);
    if (!foundation) return 10;

    PxTolerancesScale scale;
    PxPhysics* physics = PxCreatePhysics(PX_PHYSICS_VERSION, *foundation, scale, false, nullptr);
    if (!physics) return 11;

    // Zero workers is the browser configuration: PhysX runs submitted tasks on this thread and
    // therefore needs neither WebAssembly pthreads nor cross-origin isolation headers.
    PxDefaultCpuDispatcher* dispatcher = PxDefaultCpuDispatcherCreate(0);
    PxSceneDesc sceneDesc(scale);
    sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
    sceneDesc.cpuDispatcher = dispatcher;
    sceneDesc.filterShader = PxDefaultSimulationFilterShader;
    PxScene* scene = physics->createScene(sceneDesc);
    PxMaterial* material = physics->createMaterial(0.6f, 0.5f, 0.0f);
    if (!dispatcher || !scene || !material) return 12;

    PxRigidStatic* floor = physics->createRigidStatic(PxTransform(PxVec3(0.0f, -0.5f, 0.0f)));
    PxShape* floorShape = physics->createShape(PxBoxGeometry(5.0f, 0.5f, 5.0f), *material, true);
    floor->attachShape(*floorShape);
    floorShape->release();
    scene->addActor(*floor);

    PxRigidDynamic* box = physics->createRigidDynamic(PxTransform(PxVec3(0.0f, 3.0f, 0.0f)));
    PxShape* boxShape = physics->createShape(PxBoxGeometry(0.5f, 0.5f, 0.5f), *material, true);
    box->attachShape(*boxShape);
    boxShape->release();
    box->setMass(1.0f);
    box->setMassSpaceInertiaTensor(PxVec3(1.0f / 6.0f));
    scene->addActor(*box);

    for (int step = 0; step < 360; ++step) {
        scene->simulate(1.0f / 120.0f);
        scene->fetchResults(true);
    }
    const float finalY = box->getGlobalPose().p.y;
    // Box half-extent 0.5 makes 0.5 the exact resting height; the band absorbs solver settle
    // and contact-offset slack, which bias upward.
    const bool restingOnFloor = std::isfinite(finalY) && finalY > 0.45f && finalY < 0.60f;
    std::printf("PhysX WASM smoke: final_y=%.6f result=%s\n",
                static_cast<double>(finalY), restingOnFloor ? "PASS" : "FAIL");

    box->release();
    floor->release();
    material->release();
    scene->release();
    dispatcher->release();
    physics->release();
    foundation->release();
    return restingOnFloor ? 0 : 13;
}
