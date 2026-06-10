BUILDDIR		:= build
TARGET			:= icm20948
SRC				:= main.cpp
SRC				+= ICM-20948.cpp
SRC				+= ../mylibraries/Linux/i2c.cpp
INC				:= ../mylibraries/Common
INC				+= ../mylibraries/Linux
CROSS_COMPILE	:= aarch64-linux-gnu-
LDFLAGS			?= -static
DEF				?= DEBUG

include BuildServices/Makefile.common