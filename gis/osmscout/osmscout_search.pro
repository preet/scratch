LIBOSMSCOUT_PATH = /home/preet/Dev/env/sys/libosmscout

TEMPLATE = app
CONFIG -= qt
SOURCES += osmscout_search.cpp

#libosmscout
DEFINES += USE_BOOST
INCLUDEPATH += /home/preet/Dev/env/sys/boost-1.53
INCLUDEPATH += $${LIBOSMSCOUT_PATH}/include
LIBS += -L$${LIBOSMSCOUT_PATH}/lib -losmscout
