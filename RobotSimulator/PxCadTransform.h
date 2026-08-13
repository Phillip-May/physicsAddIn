#pragma once

#ifdef ROBOTSIM_WITH_PHYSX

#include "../Common/CadNode.h"

#include <PxPhysicsAPI.h>

constexpr float kMmToM = 0.001f;

inline physx::PxTransform toPxTransform(const CadTransform& value) {
    const physx::PxMat33 rotation(physx::PxVec3(static_cast<float>(value.values[0]),
                                                static_cast<float>(value.values[4]),
                                                static_cast<float>(value.values[8])),
                                  physx::PxVec3(static_cast<float>(value.values[1]),
                                                static_cast<float>(value.values[5]),
                                                static_cast<float>(value.values[9])),
                                  physx::PxVec3(static_cast<float>(value.values[2]),
                                                static_cast<float>(value.values[6]),
                                                static_cast<float>(value.values[10])));
    return physx::PxTransform(physx::PxVec3(static_cast<float>(value.values[3]) * kMmToM,
                                            static_cast<float>(value.values[7]) * kMmToM,
                                            static_cast<float>(value.values[11]) * kMmToM),
                              physx::PxQuat(rotation));
}

inline CadTransform fromPxTransform(const physx::PxTransform& value) {
    const physx::PxMat33 rotation(value.q);
    CadTransform result;
    result.values = {{rotation.column0.x, rotation.column1.x, rotation.column2.x,
                      value.p.x / kMmToM,
                      rotation.column0.y, rotation.column1.y, rotation.column2.y,
                      value.p.y / kMmToM,
                      rotation.column0.z, rotation.column1.z, rotation.column2.z,
                      value.p.z / kMmToM}};
    return result;
}

#endif
