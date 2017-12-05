#-------------------------------------------------
#
# Project created by QtCreator 2017-05-09T15:09:51
#
#-------------------------------------------------

QT       -= core gui

TARGET = sys
TEMPLATE = lib

DEFINES += SYS_LIBRARY

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
    ../../sys/net/sys_linux_net_broadcast.cpp \
    ../../sys/net/sys_linux_net_tcp.cpp \
    ../../sys/utils/base64.cpp \
    ../../sys/utils/mm.cpp \
    ../../sys/utils/strctfcvt.cpp \
    ../../sys/utils/jconf.cpp \
    ../../sys/utils/cjson.c \
    ../../sys/utils/ipc.cpp

HEADERS += \
    ../../sys/utils/cjson.h \
    ../../sys/utils/mm.h \
    ../../sys/net/sys_linux_net_broadcast.h \
    ../../sys/net/sys_linux_net_tcp.h \
    ../../sys/utils/jconf.h \
    ../../sys/utils/ipc.h

#unix {
#    target.path = ../../libs
#    INSTALLS += target
#}

INCLUDEPATH += ../../include
QMAKE_POST_LINK = ../../config/setup.sh
