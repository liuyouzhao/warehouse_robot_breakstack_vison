QT -= gui

CONFIG += c++11 console
CONFIG -= app_bundle

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS USE_DNN=1

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += main.cpp \
    train_A.cpp \
    rotate.cpp \
    run_hmm.cpp \
    train_B.cpp \
    octave_file_generator.cpp \
    binary.cpp

LIBS += -L/usr/local/lib

LIBS += -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_contrib -lopencv_ml
QMAKE_LFLAGS += -Wl,-rpath=/usr/local/lib
