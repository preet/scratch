TEMPLATE = app
CONFIG += console debug
CONFIG -= qt
SOURCES += osmscout_adm.cpp
#DEFINES += "USE_BOOST=1"

#libosmscout
LIBOSMSCOUT_PATH = /home/preet/Dev/env/sys/libosmscout
INCLUDEPATH += $${LIBOSMSCOUT_PATH}/include
LIBS += -L/home/preet/Dev/env/sys/libosmscout/lib -losmscout
LIBS += -L$${LIBOSMSCOUT_PATH}/lib -losmscout

QMAKE_CXXFLAGS += -std=c++11
