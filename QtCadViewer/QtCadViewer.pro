TEMPLATE = app
CONFIG += qt console c++17 physx_cpu
QT += core widgets gui opengl

LIBS += -lopengl32
LIBS += -lglu32

TARGET = QtCadViewer
INCLUDEPATH += .

include(../build/deps.pri)

SOURCES += \
    main.cpp

FORMS +=

include(../Common/Common.pri)
INCLUDEPATH += "../Common"
