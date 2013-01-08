TEMPLATE = app
CONFIG += console debug
CONFIG -= qt
SOURCES += osmscout_query.cpp
DEFINES += "USE_BOOST=1"

#libosmscout
LIBOSMSCOUT_PATH = /home/preet/Dev/env/sys/libosmscout
INCLUDEPATH += $${LIBOSMSCOUT_PATH}/include
LIBS += -L/home/preet/Dev/env/sys/libosmscout/lib -losmscout
LIBS += -L$${LIBOSMSCOUT_PATH}/lib -losmscout
