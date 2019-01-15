# Target specific macros
TARGET = SE_Robot
TARGET_SOURCES := \
	./SE_Robot.c

TOPPERS_OSEK_OIL_SOURCE := ./prog.oil

O_PATH ?= build

include ../ecrobot/lejos_osek.tmf
