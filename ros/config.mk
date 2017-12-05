# Prefix platform
PLATFORM=x86
BUILD=build
BOARD       =
CPU         =

# Makefile main logic
ROOT=$(shell pwd)
ifeq ($(BASE_PATH), )
BASE_PATH=../
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
OUT_BIN = libros_fw.so

# compiling and linking flags
C_CFLAGS += -I$(BASE)/include -std=c++0x
C_CFLAGS += -Os -fPIC

AFLAGS=

## normal
LDFLAGS = -lc -lm -lroslib -lstdc++ -fpic \
          -shared
LDFLAGS += -L$(CONFIG_EXTRA_ROS_PATH)/lib -Wl,-rpath $(CONFIG_EXTRA_ROS_PATH)/lib


CFLAGS= $(C_CFLAGS)

LD_LIBRARY_PATH = $(PRE_ROOT)
