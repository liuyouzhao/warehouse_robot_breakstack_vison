#-------------------------------------------------
#
# Project created by QtCreator 2017-05-09T15:28:34
#
#-------------------------------------------------

QT       -= core gui

TARGET = hal
TEMPLATE = lib

DEFINES += HAL_LIBRARY

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS CONFIG_HAL_GRAPH_GL_X11

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    ../../hal/camera/cam3d/hal_ni_camera3d.cpp \
    ../../hal/camera/hal_cam.cpp \
    ../../hal/graph/hal_gl.cpp \
    ../../hal/robotarm/aubo_robot_arm.cpp

HEADERS += \
    ../../hal/robotarm/aubo_robot_arm.h \
    ../../hal/graph/hal_gl.h \
    ../../hal/camera/hal_cam.h \
    ../../hal/camera/cam3d/hal_ni_camera3d.h

unix {
    target.path = ../../libs
    INSTALLS += target
}

INCLUDEPATH += ../../thrdprty/include/openni2
INCLUDEPATH += ../../hal/
INCLUDEPATH += ../../hal/camera
INCLUDEPATH += ../../hal/camera/cam3d
INCLUDEPATH += ../../hal/graph
INCLUDEPATH += ../../include
INCLUDEPATH += ../../

LIBS += -L../../libs/x86qt
LIBS += -lsys -lGL -lglut

QMAKE_POST_LINK = ../../config/setup.sh
