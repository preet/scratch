TEMPLATE = app
CONFIG += console debug
CONFIG -= qt
SOURCES += coastline2csv.cpp
LIBS += -lz -lpthread -lprotobuf-lite -losmpbf
INCLUDEPATH += /home/preet/Dev/env/sys/osmium/include

QMAKE_CXXFLAGS += -std=c++11
