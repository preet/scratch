TEMPLATE = app
CONFIG += console debug
CONFIG -= qt
SOURCES += ptk_simplify_wkt.cpp
TARGET = ptk_simplify_wkt

# required libs
LIBS += -lgdal -lCGAL_Core -lCGAL -lmpfr -lgmp -lboost_thread

QMAKE_CXXFLAGS += -std=c++0x
