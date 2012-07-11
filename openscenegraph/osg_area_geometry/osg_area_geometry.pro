#-------------------------------------------------
#
# Project created by QtCreator 2011-09-26T02:24:03
#
#-------------------------------------------------

CONFIG += qt link_pkgconfig debug
PKGCONFIG += openthreads openscenegraph
TARGET = osg_area_geometry
SOURCES += main.cpp
TEMPLATE = app
LIBS += -losmscout
QT += core

QMAKE_CXXFLAGS += -std=c++0x
