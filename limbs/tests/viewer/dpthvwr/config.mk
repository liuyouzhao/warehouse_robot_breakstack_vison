# Prefix platform
PLATFORM=x86
BUILD=build
BOARD       =
CPU         =

# Makefile main logic
$(info BASE_PATH: $(BASE_PATH))
ROOT=$(shell pwd)
ifeq ($(BASE_PATH), )
BASE_PATH=../../../
endif
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
OUT_BIN = dpthvwr

# compiling and linking flags
C_CFLAGS += -I./ -Os
C_CFLAGS += -I$(BASE)/hal/graph -I$(BASE)/hal
C_CFLAGS += -I$(BASE)/hal/camera/cam3d -I$(BASE)/include/openni2 -I$(BASE)/hal/camera
AFLAGS=
CFLAGS= $(C_CFLAGS)
LDFLAGS = -L$(BASE)/libs -lc -lm -lhal -lGL -lGLU
LDFLAGS += -Wl,-rpath $(BASE)/libs