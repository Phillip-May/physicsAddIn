# Shared third-party dependency configuration for the standalone app and RoboDK plugin.
#
# Machine-specific overrides belong in build/local-env.pri, which is ignored by git.
# Start from build/local-env.pri.example when setting up another workstation.

REPO_ROOT = $$clean_path($$PWD/..)

isEmpty(OCCT_ROOT): OCCT_ROOT = C:/OpenCASCADE-7.6.0-vc14-64/opencascade-7.6.0
isEmpty(PHYSX_ROOT): PHYSX_ROOT = C:/PhysX-110.1-omni-and-physx-5.9.0/physx
isEmpty(PHYSX_PLATFORM): PHYSX_PLATFORM = win.x86_64.vc142.md
isEmpty(COACD_ROOT): COACD_ROOT = $${REPO_ROOT}/external/CoACD

# Guarded: RobotSimulator's .pri set also includes local-env.pri, and a target reading both would
# otherwise apply its CONFIG += lines twice.
isEmpty(LOCAL_ENV_PRI_INCLUDED) {
    LOCAL_ENV_PRI_INCLUDED = 1
    exists($$PWD/local-env.pri): include($$PWD/local-env.pri)
}

CONFIG(release, debug|release) {
    BUILD_CONFIG = release
} else {
    BUILD_CONFIG = debug
}

!exists($${OCCT_ROOT}/inc/Standard.hxx) {
    error("OpenCascade headers not found. Set OCCT_ROOT in build/local-env.pri or run scripts/bootstrap_deps.ps1.")
}

DEFINES += CADNODE_ENABLE_OCCT

!exists($${PHYSX_ROOT}/include/PxPhysicsAPI.h) {
    error("PhysX headers not found. Set PHYSX_ROOT in build/local-env.pri or run scripts/bootstrap_deps.ps1.")
}

!exists($${COACD_ROOT}/public/coacd.h) {
    error("CoACD headers not found. Set COACD_ROOT in build/local-env.pri or run scripts/bootstrap_deps.ps1.")
}

INCLUDEPATH += \
    $${OCCT_ROOT}/inc \
    $${PHYSX_ROOT}/include \
    $${PHYSX_ROOT}/include/cooking \
    $${PHYSX_ROOT}/source/foundation/include \
    $${PHYSX_ROOT}/source/physx/include \
    $${COACD_ROOT}/public

HEADERS += $${COACD_ROOT}/public/coacd.h

LIBS += -L$${OCCT_ROOT}/win64/vc14/lib

OCCT_CORE_LIBS = \
    TKernel TKMath TKBRep TKSTEP TKSTEP209 TKSTEPAttr TKSTEPBase TKIGES TKXSBase \
    TKShHealing TKTopAlgo TKGeomBase TKGeomAlgo TKG2d TKG3d TKMesh TKXCAF TKXDESTEP \
    TKXDEIGES TKCAF TKLCAF TKCDF TKV3d TKOpenGl TKService TKStd TKStdL TKXml \
    TKXmlL TKXmlTObj TKXmlXCAF TKBin TKBinL TKBinTObj TKBinXCAF TKTObj TKFeat \
    TKFillet TKHLR TKMessageModel TKOffset TKPrim TKRWMesh TKVRML TKXMesh

for(occt_lib, OCCT_CORE_LIBS): LIBS += -l$${occt_lib}

CONFIG(occt_draw) {
    OCCT_DRAW_LIBS = TKTObjDRAW TKDCAF TKDFBrowser TKDraw TKOpenGlTest TKQADraw \
        TKShapeView TKTopTest TKViewerTest TKXSDRAW
    for(occt_lib, OCCT_DRAW_LIBS): LIBS += -l$${occt_lib}
}

CONFIG(occt_inspector) {
    OCCT_INSPECTOR_LIBS = TKMessageView TKTreeModel TKView TKVInspector
    for(occt_lib, OCCT_INSPECTOR_LIBS): LIBS += -l$${occt_lib}
}

CONFIG(occt_vtk) {
    OCCT_VTK_LIBS = TKIVtk TKIVtkDraw
    for(occt_lib, OCCT_VTK_LIBS): LIBS += -l$${occt_lib}
}

CONFIG(occt_opengles) {
    OCCT_OPENGLES_LIBS = TKOpenGles TKOpenGlesTest
    for(occt_lib, OCCT_OPENGLES_LIBS): LIBS += -l$${occt_lib}
}

COACD_BUILD_DIR = $${COACD_ROOT}/buildMD
!exists($${COACD_BUILD_DIR}/Release/coacd.lib) {
    error("CoACD release libraries not found. Run scripts/bootstrap_deps.ps1.")
}

LIBS += \
    $${COACD_BUILD_DIR}/Release/coacd.lib \
    $${COACD_BUILD_DIR}/_deps/boost-build/libs/random/Release/libboost_random-vc142-mt-x64-1_81.lib \
    $${COACD_BUILD_DIR}/_deps/zlib-build/Release/zlibstatic.lib \
    $${COACD_BUILD_DIR}/_deps/boost-build/libs/iostreams/Release/libboost_iostreams-vc142-mt-x64-1_81.lib \
    $${COACD_BUILD_DIR}/msvc_19.29_cxx20_64_md_release/tbb12.lib \
    $${COACD_BUILD_DIR}/_deps/openvdb-build/openvdb/openvdb/Release/libopenvdb.lib \
    $${COACD_BUILD_DIR}/_deps/spdlog-build/Release/spdlog.lib

PHYSX_BIN_DIR = $${PHYSX_ROOT}/bin/$${PHYSX_PLATFORM}/$${BUILD_CONFIG}
!exists($${PHYSX_BIN_DIR}/PhysX_64.lib) {
    error("PhysX libraries not found in $${PHYSX_BIN_DIR}. Run scripts/bootstrap_deps.ps1.")
}

DEFINES += PX_PHYSX_STATIC_LIB
LIBS += -L$${PHYSX_BIN_DIR} \
    -lPhysX_64 \
    -lPhysXFoundation_64 \
    -lPhysXCommon_64 \
    -lPVDRuntime_64 \
    -lPhysXCooking_64

LIBS += \
    $${PHYSX_BIN_DIR}/PhysXPvdSDK_static_64.lib \
    $${PHYSX_BIN_DIR}/PhysXExtensions_static_64.lib

CONFIG(physx_gpu) {
    DEFINES += PHYSX_ENABLE_GPU
    LIBS += -lPhysXGpu_64
} else {
    CONFIG += physx_cpu
}
