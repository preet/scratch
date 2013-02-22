CONFIG -= qt
CONFIG += link_pkgconfig
PKGCONFIG += openthreads openscenegraph
TEMPLATE = app

HEADERS += CommonFunctions.h
SOURCES += main.cpp CommonFunctions.cpp

