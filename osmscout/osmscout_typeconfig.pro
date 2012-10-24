TEMPLATE = app
CONFIG += console debug
CONFIG -= qt
SOURCES += osmscout_typeconfig.cpp

#libosmscout
INCLUDEPATH += /home/preet/Dev/env/sys/libosmscout/include
LIBS += -L/home/preet/Dev/env/sys/libosmscout/lib -losmscout

QMAKE_CXXFLAGS += -std=c++0x

