# Minimal PhysX dependency surface for the Qt-free RobotSimulator desktop target.
isEmpty(PHYSX_ROOT): PHYSX_ROOT = C:/PhysX-110.1-omni-and-physx-5.9.0/physx
isEmpty(PHYSX_PLATFORM): PHYSX_PLATFORM = win.x86_64.vc142.md
isEmpty(LOCAL_ENV_PRI_INCLUDED) {
    LOCAL_ENV_PRI_INCLUDED = 1
    exists($$PWD/local-env.pri): include($$PWD/local-env.pri)
}

CONFIG(release, debug|release): PHYSX_BUILD_CONFIG = release
else: PHYSX_BUILD_CONFIG = debug

PHYSX_BIN_DIR = $${PHYSX_ROOT}/bin/$${PHYSX_PLATFORM}/$${PHYSX_BUILD_CONFIG}
!exists($${PHYSX_ROOT}/include/PxPhysicsAPI.h) {
    error("PhysX 5.9 headers not found. Set PHYSX_ROOT in build/local-env.pri.")
}
!exists($${PHYSX_BIN_DIR}/PhysX_64.lib) {
    error("PhysX 5.9 libraries not found in $${PHYSX_BIN_DIR}.")
}

DEFINES += ROBOTSIM_WITH_PHYSX PX_PHYSX_STATIC_LIB
INCLUDEPATH += \
    $${PHYSX_ROOT}/include \
    $${PHYSX_ROOT}/source/foundation/include \
    $${PHYSX_ROOT}/source/physx/include
LIBS += -L$${PHYSX_BIN_DIR} \
    -lPhysX_64 \
    -lPhysXFoundation_64 \
    -lPhysXCommon_64 \
    -lPhysXCooking_64 \
    -lPVDRuntime_64 \
    -lPhysXExtensions_static_64 \
    $${PHYSX_BIN_DIR}/PhysXPvdSDK_static_64.lib
