TEMPLATE = app
CONFIG += qt console c++17
QT += core widgets gui opengl 

LIBS += -lopengl32
LIBS += -lglu32

TARGET = QtCadViewer
INCLUDEPATH += .

# OpenCascade 7.6.0 paths (user: adjust if needed)
INCLUDEPATH += C:/OpenCASCADE-7.6.0-vc14-64/opencascade-7.6.0/inc
LIBS += -LC:/OpenCASCADE-7.6.0-vc14-64/opencascade-7.6.0/win64/vc14/lib \
    -lTKernel -lTKMath -lTKBRep -lTKSTEP -lTKSTEP209 -lTKSTEPAttr -lTKSTEPBase -lTKIGES -lTKXSBase -lTKShHealing -lTKTopAlgo -lTKGeomBase -lTKGeomAlgo -lTKG2d -lTKG3d -lTKMesh -lTKXCAF -lTKXDESTEP -lTKXDEIGES -lTKCAF -lTKLCAF -lTKCDF -lTKV3d -lTKOpenGl -lTKService -lTKStd -lTKStdL -lTKXml -lTKXmlL -lTKXmlTObj -lTKXmlXCAF -lTKBin -lTKBinL -lTKBinTObj -lTKBinXCAF -lTKTObj -lTKTObjDRAW -lTKDCAF -lTKDFBrowser -lTKDraw -lTKFeat -lTKFillet -lTKHLR -lTKIVtk -lTKIVtkDraw -lTKMessageModel -lTKMessageView -lTKOffset -lTKOpenGles -lTKOpenGlesTest -lTKOpenGlTest -lTKPrim -lTKQADraw -lTKRWMesh -lTKShapeView -lTKTopTest -lTKTreeModel -lTKView -lTKViewerTest -lTKVInspector -lTKVRML -lTKXMesh -lTKXSBase -lTKXSDRAW

# VHACD Library Integration
INCLUDEPATH += ../physicsAddIn/v-hacd-4.1.0/include

# CoACD Library Integration
INCLUDEPATH += ../external/CoACD/public
LIBS += C:/Users/Admin/Documents/physicsAddIn/external/CoACD/buildMD/Release/coacd.lib \
    C:/Users/Admin/Documents/physicsAddIn/external/CoACD/buildMD/_deps/boost-build/libs/random/Release/libboost_random-vc142-mt-x64-1_81.lib \
    C:/Users/Admin/Documents/physicsAddIn/external/CoACD/buildMD/_deps/zlib-build/Release/zlibstatic.lib \
    C:/Users/Admin/Documents/physicsAddIn/external/CoACD/buildMD/_deps/boost-build/libs/iostreams/Release/libboost_iostreams-vc142-mt-x64-1_81.lib \
    C:/Users/Admin/Documents/physicsAddIn/external/CoACD/buildMD/msvc_19.29_cxx20_64_md_release/tbb12.lib \
    C:/Users/Admin/Documents/physicsAddIn/external/CoACD/buildMD/_deps/openvdb-build/openvdb/openvdb/Release/libopenvdb.lib \
    C:/Users/Admin/Documents/physicsAddIn/external/CoACD/buildMD/_deps/spdlog-build/Release/spdlog.lib

# PhysX Library Integration (copied from PluginPhysics.pro)
INCLUDEPATH += C:/PhysX-107.0-physx-5.6.0/PhysX/include \
               C:/PhysX-107.0-physx-5.6.0/PhysX/include/cooking \
               C:/PhysX-107.0-physx-5.6.0/PhysX/source/foundation/include \
               C:/PhysX-107.0-physx-5.6.0/PhysX/source/physx/include

SOURCES += \
    HelperFunctions.cpp \
    RailJsonEditorDialog.cpp \
    main.cpp \
    CadOpenGLWidget.cpp \
    CadTreeModel.cpp \
    XCAFLabelTreeModel.cpp \
    CustomModelTreeModel.cpp \
    SimulationManager.cpp

HEADERS += \
    ../external/CoACD/public/coacd.h \
    CadNode.h \
    CadOpenGLWidget.h \
    CadTreeModel.h \
    HelperFunctions.h \
    RailJsonEditorDialog.h \
    XCAFLabelTreeModel.h \
    CustomModelTreeModel.h \
    SimulationManager.h
FORMS +=

include(../Common/Common.pri)
INCLUDEPATH += "../Common"


CONFIG(release, debug|release) {
DEFINES += PX_PHYSX_STATIC_LIB \
           _NDEBUG # or NDEBUG for release builds
LIBS += -LC:/PhysX-107.0-physx-5.6.0/physx/bin/win.x86_64.vc142.md/release/ -lPhysX_64 \
                                           -lPhysXFoundation_64 \
                                           -lPhysXCommon_64 \
                                           -lPVDRuntime_64 \
                                           -lPhysXExtensions_static_64 \
                                           -lPhysXCooking_64
LIBS += C:/PhysX-107.0-physx-5.6.0/physx/bin/win.x86_64.vc142.md/release/PhysXPvdSDK_static_64.lib
LIBS += C:/PhysX-107.0-physx-5.6.0/physx/bin/win.x86_64.vc142.md/release/PhysXExtensions_static_64.lib
} else {
DEFINES += PX_PHYSX_STATIC_LIB \
           _DEBUG # or NDEBUG for release builds
LIBS += -LC:/PhysX-107.0-physx-5.6.0/physx/bin/win.x86_64.vc142.md/debug/ -lPhysX_64 \
                                -lPhysXFoundation_64 \
                                -lPhysXCommon_64 \
                                -lPVDRuntime_64 \
                                -lPhysXExtensions_static_64 \
                                -lPhysXCooking_64
LIBS += C:/PhysX-107.0-physx-5.6.0/physx/bin/win.x86_64.vc142.md/debug/PhysXPvdSDK_static_64.lib
LIBS += C:/PhysX-107.0-physx-5.6.0/physx/bin/win.x86_64.vc142.md/debug/PhysXExtensions_static_64.lib
}
