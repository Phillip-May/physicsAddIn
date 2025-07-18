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
    -lTKernel -lTKMath -lTKBRep -lTKSTEP -lTKSTEP209 -lTKSTEPAttr -lTKSTEPBase -lTKIGES -lTKXSBase -lTKShHealing -lTKTopAlgo -lTKGeomBase -lTKGeomAlgo -lTKG2d -lTKG3d -lTKMesh -lTKXCAF -lTKXDESTEP -lTKXDEIGES -lTKCAF -lTKLCAF -lTKCDF -lTKV3d -lTKOpenGl -lTKService

SOURCES += \
    main.cpp \
    CadOpenGLWidget.cpp \
    CadTreeModel.cpp \
    XCAFLabelTreeModel.cpp

HEADERS += \
    CadOpenGLWidget.h \
    CadTreeModel.h \
    CadNode.h \
    XCAFLabelTreeModel.h
FORMS += 