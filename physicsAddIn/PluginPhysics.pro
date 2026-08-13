

#----------------- TEMPLATE --------- (Qt Plugin App template)
# Important: Do not change these values (unless you know what you are doing)
TEMPLATE        = lib
CONFIG         += plugin
CONFIG += c++17
#------------------------------------


# Qt Widgets only. RoboDK 6.0 ships Qt 5.15, which is what loads this, so nothing here may need a
# Qt module RoboDK does not already carry beside its executable.
QT += widgets

# Generated from library/packages by build_robodk_plugin.ps1. The entries are already-compressed zips.
RESOURCES += builtin_packages.qrc
QMAKE_RESOURCE_FLAGS += -no-compress

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
    win32{
        #Default path on Windows (debug)
        DESTDIR  = C:/RoboDK/bind/plugins
    } else {
    macx {
        # Default path on MacOS (debug)
        DESTDIR  = ~/RoboDK-Dev/Deploy/RoboDK.app/Contents/MacOS/plugins
    } else {
        #Default path on Linux (debug)
        DESTDIR  = ~/RoboDK/bin/plugins
    }
    }

}
}

!isEmpty(PLUGIN_DESTDIR) {
    DESTDIR = $$PLUGIN_DESTDIR
}

#--------------------------
HEADERS += \
    pluginPhysics.h \
    PhysicsPanel.h \
    ConveyorPropertiesPanel.h \
    LibraryDock.h \
    LibraryItems.h \
    LibraryPlacementTool.h \
    LibraryPlacer.h \
    PhysicsIcons.h \
    PlacedItem.h \
    PlacedItemAxes.h \
    PlacedItemPicker.h \
    PlacedItemPropertiesPanel.h \
    PhysicsWorld.h \
    RoboDkBridge.h \
    RoboDkConveyorHost.h \
    ConveyorScenery.h \
    ../Common/AccessoryGeometry.h \
    ../Common/AccessoryPropertySchema.h \
    ../Common/CadNode.h \
    ../Common/CadNodeDraw.h \
    ../Common/CadNodePackage.h \
    ../Common/ConveyorCore.h \
    ../Common/ConveyorGeometry.h \
    ../Common/LibraryCatalogue.h \
    ../Common/MountingSnap.h \
    ../Common/PlacedMechanismSchema.h \
    ../Common/PlacementSession.h \
    ../Common/RobotRuntime.h \
    ../Common/SelectionStyle.h \
    ../Common/ViewRay.h \
    ../RobotSimulator/ConveyorPhysics.h

SOURCES += \
    pluginPhysics.cpp \
    PhysicsPanel.cpp \
    ConveyorPropertiesPanel.cpp \
    LibraryDock.cpp \
    LibraryItems.cpp \
    LibraryPlacementTool.cpp \
    LibraryPlacer.cpp \
    PhysicsIcons.cpp \
    PlacedItem.cpp \
    PlacedItemAxes.cpp \
    PlacedItemPicker.cpp \
    PlacedItemPropertiesPanel.cpp \
    PhysicsWorld.cpp \
    RoboDkBridge.cpp \
    RoboDkConveyorHost.cpp \
    ConveyorScenery.cpp \
    ../Common/AccessoryGeometry.cpp \
    ../Common/AccessoryPropertySchema.cpp \
    ../Common/CadNodeDraw.cpp \
    ../Common/CadNodePackage.cpp \
    ../Common/ConveyorCore.cpp \
    ../Common/ConveyorGeometry.cpp \
    ../Common/LibraryCatalogue.cpp \
    ../Common/MountingSnap.cpp \
    ../Common/PlacedMechanismSchema.cpp \
    ../Common/PlacementSession.cpp \
    ../Common/RobotRuntime.cpp \
    ../Common/ViewRay.cpp \
    ../RobotSimulator/ConveyorPhysics.cpp

# Do not include Common/Common.pri or build/deps.pri: plugin dependencies must resolve from RoboDK's
# bin directory, and the host's OCCT version is not stable. QtCadViewer owns the OCCT viewer.
INCLUDEPATH += ../Common ../RobotSimulator

# nlohmann/json and miniz: CadNode.h serialises through JsonCompat.h.
include(../build/thirdparty.pri)

# PhysX alone, which also supplies ROBOTSIM_WITH_PHYSX. Without that define every ConveyorPhysics
# entry point compiles to a stub reporting "unavailable in this build", giving a plugin that loads,
# runs and silently simulates nothing.
include(../build/physx-runtime.pri)


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
