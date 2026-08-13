# CadNode.h serialises through nlohmann and CadNodePackage reads zips with miniz, so every
# target that compiles Common needs these. RobotSimulator includes the same file directly.
include($$PWD/../build/thirdparty.pri)

HEADERS += \
    $$PWD/MaterialEditorDialog.h \
    $$PWD/MaterialManager.h \
    $$PWD/ObjectPropertiesDialog.h \
    $$PWD/CadDecomposition.h \
    $$PWD/CadNode.h \
    $$PWD/CadNodeQtAdapter.h \
    $$PWD/CadNodeOps.h \
    $$PWD/CadNodePackage.h \
    $$PWD/CadOpenGLWidget.h \
    $$PWD/CadViewerWiring.h \
    $$PWD/CadXcafIo.h \
    $$PWD/CustomModelTreeModel.h \
    $$PWD/RailJsonEditorDialog.h \
    $$PWD/RobotRuntime.h \
    $$PWD/SimulationManager.h \
    $$PWD/XCAFLabelTreeModel.h

SOURCES += \
    $$PWD/MaterialEditorDialog.cpp \
    $$PWD/MaterialManager.cpp \
    $$PWD/ObjectPropertiesDialog.cpp \
    $$PWD/CadNodePackage.cpp \
    $$PWD/CadOpenGLWidget.cpp \
    $$PWD/CustomModelTreeModel.cpp \
    $$PWD/CadXcafIo.cpp \
    $$PWD/CadNodeOps.cpp \
    $$PWD/CadDecomposition.cpp \
    $$PWD/CadViewerWiring.cpp \
    $$PWD/RailJsonEditorDialog.cpp \
    $$PWD/RobotRuntime.cpp \
    $$PWD/SimulationManager.cpp \
    $$PWD/XCAFLabelTreeModel.cpp
