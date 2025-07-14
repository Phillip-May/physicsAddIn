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
    PhysXEngine.h \
    MaterialManager.h \
    MaterialEditorDialog.h \
    ObjectPropertiesDialog.h \
    ObjectPropertiesManager.h \
    SceneConfigurationDialog.h \
    SoftBodyConfigDialog.h \
    SnippetImmUtils.h \
    SnippetPVD.h \
    SnippetPrint.h \
    SnippetUtils.h \
    pluginPhysics.h

SOURCES += \
    PhysXEngine.cpp \
    MaterialManager.cpp \
    MaterialEditorDialog.cpp \
    ObjectPropertiesDialog.cpp \
    ObjectPropertiesManager.cpp \
    SceneConfigurationDialog.cpp \
    SoftBodyConfigDialog.cpp \
    SnippetImmUtils.cpp \
    SnippetUtils.cpp \
    pluginPhysics.cpp


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

INCLUDEPATH += ../robodk_interface


#--------------------------
# VHACD Library Integration
# Add VHACD include path
INCLUDEPATH += $$PWD/v-hacd-4.1.0/include


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