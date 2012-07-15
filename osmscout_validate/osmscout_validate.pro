TEMPLATE = app
CONFIG += console debug
CONFIG -= qt
SOURCES += main.cpp

#libosmscout
INCLUDEPATH += /home/preet/Documents/libosmscout/include
LIBS += -L/home/preet/Documents/libosmscout/lib -losmscout

QMAKE_CXXFLAGS += -std=c++0x

