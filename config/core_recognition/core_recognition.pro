#-------------------------------------------------
#
# Project created by QtCreator 2017-05-09T15:06:15
#
#-------------------------------------------------

QT       -= core gui

TARGET = core_recognition
TEMPLATE = lib

DEFINES += CORE_RECOGNITION_LIBRARY

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
    ../../core/recognition/bpl_config.cpp \
    ../../core/recognition/bpl_learner_prm.cpp \
    ../../core/recognition/bpl_learner_token.cpp \
    ../../core/recognition/bpl_main.cpp \
    ../../core/recognition/bpl_param.cpp \
    ../../core/recognition/improc.cpp \
    ../../core/recognition/mdl_box.cpp \
    ../../core/recognition/mdl_world.c \
    ../../core/recognition/bpl_tmplt.cpp \
    ../../core/recognition/bpl_tmplt_creator.cpp \
    ../../core/recognition/bpl_voting.cpp \
    ../../core/recognition/bpl_transformer.cpp \
    ../../core/recognition/bpl_token2image.cpp \
    ../../core/recognition/bpl_box_shape_proc.cpp \
    ../../core/recognition/bayesian.cpp

HEADERS += \
    ../../core/recognition/bpl_config.h \
    ../../core/recognition/bpl_learner_prm.h \
    ../../core/recognition/bpl_learner_token.h \
    ../../core/recognition/bpl_main.h \
    ../../core/recognition/bpl_param.h \
    ../../core/recognition/mdl_box.h \
    ../../core/recognition/bpl_tmplt.h \
    ../../core/recognition/bpl_tmplt_creator.h \
    ../../core/recognition/bpl_voting.h \
    ../../core/recognition/bpl_transformer.h \
    ../../core/recognition/bpl_token2image.h \
    ../../core/recognition/bpl_box_shape_proc.h \
    ../../core/recognition/bayesian.h

unix {
    target.path = ../../libs
    INSTALLS += target
}

LIBS += -L../../libs/x86qt
LIBS += -lsys

INCLUDEPATH += ../../sys/utils

QMAKE_POST_LINK = ../../config/setup.sh
