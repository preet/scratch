TEMPLATE = app
CONFIG += console debug
CONFIG -= qt
HEADERS += clipper/clipper.hpp
SOURCES += ptk_quadify_wkt.cpp \
           clipper/clipper.cpp
TARGET = ptk_quadify_wkt

# required libs
LIBS += -lgdal -lCGAL_Core -lCGAL -lmpfr -lgmp -lboost_thread

QMAKE_CXXFLAGS += -std=c++0x
