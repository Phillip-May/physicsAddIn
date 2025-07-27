#----------------- HELP --------------
# Help about RoboDK plugins here:
# https://robodk.com/CreatePlugin

# Clear some space in the General Messages window
message(".")
message(".")
message(".")
message(".")
message(".")
message("Useful tip that helps development: Enter RoboDK as executable and pass the argument -PLUGINSLOAD to start with all available plugins")
# Example to reload all plugins:
# C:/RoboDK/bin/RoboDK.exe "-PLUGINSLOAD"
# Example to load the plugin on the fly:
# C:/RoboDK/bin/RoboDK.exe "-PLUGINLOAD=C:/RoboDK/bin/plugins/pluginexample.dll"
# You can also select Tools-PlugIns and manually load a plugin
#------------------------------------


#----------------- TEMPLATE --------- (Qt Plugin App template)
# Important: Do not change these values (unless you know what you are doing)
TEMPLATE        = lib
CONFIG         += plugin
CONFIG += c++17
#------------------------------------


# Add any Qt libraries you would like to use:
#QT += core gui
QT += widgets
QT += network   # Allows using QTcpSocket
QT += opengl
LIBS += -lopengl32 -lglu32

# Define your plugin name (name of the DLL file generated)
TARGET          = PluginPhysics


#-----------------------------------------------------
# Define the location to place the plugin library (release and/or debug binaries)
exists( "$$PWD/../../destdir_rdk_plugins.pri" ) {
include("$$PWD/../../destdir_rdk_plugins.pri")
DESTDIR = $$DESTDIR_RDK_PLUGINS
} else {
#-----------------------------------------------------
CONFIG(release, debug|release) {

    message("Using release binaries.")
    message("Select Projects-Run-Executable and set to C:/RoboDK/bin/RoboDK.exe ")
    win32{
        #Default path on Windows
        DESTDIR  = C:/RoboDK/bin/plugins
    } else {
    macx {
        # Default path on MacOS
        DESTDIR  = ~/RoboDK-Dev/Deploy/RoboDK.app/Contents/MacOS/plugins
    } else {
        #Default path on Linux
        DESTDIR  = ~/RoboDK/bin/plugins
    }
    }

} else {

    message("Using debug binaries: Make sure you start the debug version of RoboDK ( C:/RoboDK/bind/ ). ")
    message("Select Projects-Run-Executable and set to C:/RoboDK/bind/RoboDK.exe ")
    message("(send us an email at info@robodk.com to obtain debug binaries that should go to the bind directory)")
    win32{
        #Default path on Windows (debug)
        DESTDIR  = C:/RoboDK/bind/plugins
    } else {
    macx {
        # Default path on MacOS (debug)
        DESTDIR  = ~/RoboDK-Dev/Deploy/RoboDK.app/Contents/MacOS/plugins
    } else {
        #Default path on Linux (debug)
        DESTDIR  = ~/RoboDK/bind/plugins
    }
    }

}
}

#--------------------------
# Add header and source files (use File->New File or Project and add your files)
# This can be modified manually or automatically by Qt Creator
HEADERS += \
    IPhysicsEngine.h \
    pluginPhysics.h \
    CadViewerDialog.h

SOURCES += \
    pluginPhysics.cpp \
    CadViewerDialog.cpp

include(../Common/Common.pri)


#--------------------------
# Header and source files required by any Qt application as a RoboDK plugin
# Do not change this section
HEADERS += \
    ../robodk_interface/iitem.h \
    ../robodk_interface/irobodk.h\
    ../robodk_interface/iapprobodk.h \
    ../robodk_interface/robodktypes.h \
    ../robodk_interface/robodktools.h \

SOURCES += \
    ../robodk_interface/robodktools.cpp \
    ../robodk_interface/robodktypes.cpp

INCLUDEPATH += ../Common

INCLUDEPATH += ../robodk_interface


#--------------------------
# VHACD Library Integration
# Add VHACD include path
INCLUDEPATH += $$PWD/v-hacd-4.1.0/include

#--------------------------
# OpenCascade 7.6.0 Library Integration
INCLUDEPATH += C:/OpenCASCADE-7.6.0-vc14-64/opencascade-7.6.0/inc

#--------------------------
# CoACD Library Integration
INCLUDEPATH += ../external/CoACD/public

#--------------------------
# Include paths (adjust to your PhysX SDK install location)
INCLUDEPATH += C:/PhysX-107.0-physx-5.6.0/PhysX/include \
               C:/PhysX-107.0-physx-5.6.0/PhysX/include/cooking \
               C:/PhysX-107.0-physx-5.6.0/PhysX/source/foundation/include \
               C:/PhysX-107.0-physx-5.6.0/PhysX/source/physx/include

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
           # OpenCascade Libraries
           LIBS += -LC:/OpenCASCADE-7.6.0-vc14-64/opencascade-7.6.0/win64/vc14/lib \
               -lTKernel -lTKMath -lTKBRep -lTKSTEP -lTKSTEP209 -lTKSTEPAttr -lTKSTEPBase -lTKIGES -lTKXSBase -lTKShHealing -lTKTopAlgo -lTKGeomBase -lTKGeomAlgo -lTKG2d -lTKG3d -lTKMesh -lTKXCAF -lTKXDESTEP -lTKXDEIGES -lTKCAF -lTKLCAF -lTKCDF -lTKV3d -lTKOpenGl -lTKService -lTKStd -lTKStdL -lTKXml -lTKXmlL -lTKXmlTObj -lTKXmlXCAF -lTKBin -lTKBinL -lTKBinTObj -lTKBinXCAF -lTKTObj -lTKTObjDRAW -lTKDCAF -lTKDFBrowser -lTKDraw -lTKFeat -lTKFillet -lTKHLR -lTKIVtk -lTKIVtkDraw -lTKMessageModel -lTKMessageView -lTKOffset -lTKOpenGles -lTKOpenGlesTest -lTKOpenGlTest -lTKPrim -lTKQADraw -lTKRWMesh -lTKShapeView -lTKTopTest -lTKTreeModel -lTKView -lTKViewerTest -lTKVInspector -lTKVRML -lTKXMesh -lTKXSBase -lTKXSDRAW
           
           # CoACD Libraries
           LIBS += C:/Users/Admin/Documents/physicsAddIn/external/CoACD/buildMD/Release/coacd.lib \
               C:/Users/Admin/Documents/physicsAddIn/external/CoACD/buildMD/_deps/boost-build/libs/random/Release/libboost_random-vc142-mt-x64-1_81.lib \
               C:/Users/Admin/Documents/physicsAddIn/external/CoACD/buildMD/_deps/zlib-build/Release/zlibstatic.lib \
               C:/Users/Admin/Documents/physicsAddIn/external/CoACD/buildMD/_deps/boost-build/libs/iostreams/Release/libboost_iostreams-vc142-mt-x64-1_81.lib \
               C:/Users/Admin/Documents/physicsAddIn/external/CoACD/buildMD/msvc_19.29_cxx20_64_md_release/tbb12.lib \
               C:/Users/Admin/Documents/physicsAddIn/external/CoACD/buildMD/_deps/openvdb-build/openvdb/openvdb/Release/libopenvdb.lib \
               C:/Users/Admin/Documents/physicsAddIn/external/CoACD/buildMD/_deps/spdlog-build/Release/spdlog.lib
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
LIBS += -L$$PWD/v-hacd-4.1.0/build/ -lVHACD

# OpenCascade Libraries (same for debug)
LIBS += -LC:/OpenCASCADE-7.6.0-vc14-64/opencascade-7.6.0/win64/vc14/lib \
    -lTKernel -lTKMath -lTKBRep -lTKSTEP -lTKSTEP209 -lTKSTEPAttr -lTKSTEPBase -lTKIGES -lTKXSBase -lTKShHealing -lTKTopAlgo -lTKGeomBase -lTKGeomAlgo -lTKG2d -lTKG3d -lTKMesh -lTKXCAF -lTKXDESTEP -lTKXDEIGES -lTKCAF -lTKLCAF -lTKCDF -lTKV3d -lTKOpenGl -lTKService -lTKStd -lTKStdL -lTKXml -lTKXmlL -lTKXmlTObj -lTKXmlXCAF -lTKBin -lTKBinL -lTKBinTObj -lTKBinXCAF -lTKTObj -lTKTObjDRAW -lTKDCAF -lTKDFBrowser -lTKDraw -lTKFeat -lTKFillet -lTKHLR -lTKIVtk -lTKIVtkDraw -lTKMessageModel -lTKMessageView -lTKOffset -lTKOpenGles -lTKOpenGlesTest -lTKOpenGlTest -lTKPrim -lTKQADraw -lTKRWMesh -lTKShapeView -lTKTopTest -lTKTreeModel -lTKView -lTKViewerTest -lTKVInspector -lTKVRML -lTKXMesh -lTKXSBase -lTKXSDRAW

# CoACD Libraries (same for debug)
LIBS += C:/Users/Admin/Documents/physicsAddIn/external/CoACD/buildMD/Release/coacd.lib \
    C:/Users/Admin/Documents/physicsAddIn/external/CoACD/buildMD/_deps/boost-build/libs/random/Release/libboost_random-vc142-mt-x64-1_81.lib \
    C:/Users/Admin/Documents/physicsAddIn/external/CoACD/buildMD/_deps/zlib-build/Release/zlibstatic.lib \
    C:/Users/Admin/Documents/physicsAddIn/external/CoACD/buildMD/_deps/boost-build/libs/iostreams/Release/libboost_iostreams-vc142-mt-x64-1_81.lib \
    C:/Users/Admin/Documents/physicsAddIn/external/CoACD/buildMD/msvc_19.29_cxx20_64_md_release/tbb12.lib \
    C:/Users/Admin/Documents/physicsAddIn/external/CoACD/buildMD/_deps/openvdb-build/openvdb/openvdb/Release/libopenvdb.lib \
    C:/Users/Admin/Documents/physicsAddIn/external/CoACD/buildMD/_deps/spdlog-build/Release/spdlog.lib
} 
