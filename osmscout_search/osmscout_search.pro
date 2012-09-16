LIBOSMSCOUT_PATH = /home/preet/Dev/env/sys/libosmscout

TEMPLATE = app
CONFIG += console debug
CONFIG -= qt
SOURCES += main.cpp
DEFINES += "USE_BOOST=1"

#libosmscout
INCLUDEPATH += $${LIBOSMSCOUT_PATH}/include
LIBS += -L/home/preet/Dev/env/sys/libosmscout/lib -losmscout
LIBS += -L$${LIBOSMSCOUT_PATH}/lib -losmscout

