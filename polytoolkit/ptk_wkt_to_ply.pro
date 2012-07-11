TEMPLATE = app
CONFIG += console debug
CONFIG -= qt
SOURCES += ptk_wkt_to_ply.cpp
TARGET = ptk_wkt_to_ply

# required libs
LIBS += -lgdal -lCGAL_Core -lCGAL -lmpfr -lgmp -lboost_thread

# rply to read and write ply files
HEADERS += rply/rply.h
SOURCES += rply/rply.c

QMAKE_CXXFLAGS += -std=c++0x
