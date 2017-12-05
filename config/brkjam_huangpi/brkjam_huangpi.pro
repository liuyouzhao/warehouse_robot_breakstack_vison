QT -= core
QT -= gui

CONFIG += c++11

TARGET = brkjam_huangpi
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += \
    ../../app/brkjam_huangpi/bjam_main.cpp \
    ../../app/brkjam_huangpi/bjam_state.cpp \
    ../../app/brkjam_huangpi/bjam_ltjudgement.cpp \
    ../../app/brkjam_huangpi/bjam_depth_proc.cpp \
    ../../app/brkjam_huangpi/bjam_rgb_proc.cpp

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS UNIX EYEONLY

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

HEADERS += \
    ../../app/brkjam_huangpi/bjam_main.h \
    ../../app/brkjam_huangpi/bjam_state.h \
    ../../app/brkjam_huangpi/bjam_ltjudgement.h \
    ../../app/brkjam_huangpi/bjam_depth_proc.h \
    ../../app/brkjam_huangpi/bjam_rgb_proc.h

INCLUDEPATH += ../../core/VisionTool
INCLUDEPATH += ../../core/recognition
INCLUDEPATH += ../../thrdprty/include/openni2
INCLUDEPATH += ../../sys/utils
INCLUDEPATH += ../../sys
INCLUDEPATH += ../../hal/
INCLUDEPATH += ../../hal/camera
INCLUDEPATH += ../../hal/camera/cam3d
INCLUDEPATH += ../../hal/graph
INCLUDEPATH += ../../include
INCLUDEPATH += ../../

LIBS += -L../../libs/x86qt
LIBS += -L../../thrdprty/libs/orbbec
LIBS += -lsys -lhal -lcore_virsiontool -lcore_recognition
LIBS += -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_contrib
LIBS += -lOpenNI2
LIBS += -lGL -lglut
