# Prefix platform
PLATFORM=x86
BUILD=build
BOARD       =
CPU         =

# Makefile main logic
ROOT=$(shell pwd)
ifeq ($(BASE_PATH), )
BASE_PATH=$(ROOT)/../../
endif

# include global config
include $(BASE_PATH)/config/defconfig

BASE=$(BASE_PATH)
PRE_ROOT=$(BASE)/libs

# toolchain gcc prefix
CC    =g++
AS    =as
LD    =g++
OC    =cp
LDIR  =
LDIR2 =
TAIL = cpp

# output binary path
OUT = out
OUT_DIR = $(BASE)/$(OUT)/$(PLATFORM)
OUT_BIN = test_segoffline

# compiling and linking flags
C_CFLAGS += -I./ -I/usr/include/ -I$(BASE)/thrdprty/include/openni2
C_CFLAGS += -I./graph
C_CFLAGS += -I$(BASE)/include/hal -I$(BASE)/core/VisionTool
C_CFLAGS += -I$(BASE)/include/hal -I$(BASE)/core/Segmentation3D
C_CFLAGS += -O0 -fPIC -gdwarf-3
C_CFLAGS += -DUNIX -DGLX_GLXEXT_LEGACY


AFLAGS=

## normal
LDFLAGS = -L$(BASE)/libs -L/usr/local/lib -Wl,-rpath $(BASE)/libs
LDFLAGS += -lc -lm -lopencv_core -lopencv_highgui -fpic -lcore -lopencv_imgproc -lopencv_contrib

## configs
ifeq ($(CONFIG_HAL_CAMERA_CAM3D_ORBBEC), y)
LDFLAGS += -L$(BASE)/thrdprty/libs/orbbec
LDFLAGS += -Wl,-rpath $(BASE)/thrdprty/libs/orbbec
endif
ifeq ($(CONFIG_HAL_GRAPH_GL_X11), y)
C_CFLAGS += -DCONFIG_HAL_GRAPH_GL_X11
endif


CFLAGS= $(C_CFLAGS)

LD_LIBRARY_PATH = $(PRE_ROOT)
