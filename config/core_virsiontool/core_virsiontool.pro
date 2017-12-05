#-------------------------------------------------
#
# Project created by QtCreator 2017-05-09T15:41:35
#
#-------------------------------------------------

QT       -= core gui

TARGET = core_virsiontool
TEMPLATE = lib

DEFINES += CORE_VIRSIONTOOL_LIBRARY UNIX

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    ../../core/VisionTool/boxIsolate.cpp \
    ../../core/VisionTool/cam2Arm.cpp \
    ../../core/VisionTool/CameraPara.cpp \
    ../../core/VisionTool/Coordinate.cpp \
    ../../core/VisionTool/PlaneFinder.cpp \
    ../../core/VisionTool/PlaneFit2.cpp \
    ../../core/VisionTool/Rotation.cpp \
    ../../core/VisionTool/vt_depthTrans.cpp \
    ../../core/VisionTool/vt_io.cpp \
    ../../core/VisionTool/vt_pointCloud.cpp \
    ../../core/VisionTool/vt_visual.cpp

HEADERS += \
    ../../core/VisionTool/boxIsolate.h \
    ../../core/VisionTool/cam2Arm.h \
    ../../core/VisionTool/CameraPara.h \
    ../../core/VisionTool/Coordinate.h \
    ../../core/VisionTool/PlaneFinder.h \
    ../../core/VisionTool/PlaneFit2.h \
    ../../core/VisionTool/resource.h \
    ../../core/VisionTool/Rotation.h \
    ../../core/VisionTool/type.h \
    ../../core/VisionTool/vt_depthTrans.h \
    ../../core/VisionTool/vt_io.h \
    ../../core/VisionTool/vt_pointCloud.h \
    ../../core/VisionTool/vt_visual.h

unix {
    target.path = /usr/lib
    INSTALLS += target
}

DISTFILES += \
    ../../core/VisionTool/3DTool.rc \
    ../../core/VisionTool/Tools3D.vcxproj

QMAKE_POST_LINK = ../../config/setup.sh
