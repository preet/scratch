PATH_OSMSCOUT = /home/preet/Dev/env/sys/libosmscout

TEMPLATE = app
CONFIG += console debug
CONFIG -= qt
SOURCES += osmscout_json_import.cpp

#boost
DEFINES += USE_BOOST
INCLUDEPATH += /home/preet/Dev/env/sys/boost-1.53

#jansson
LIBS += -ljansson

#libosmscout
INCLUDEPATH += $${PATH_OSMSCOUT}/include
LIBS += -L$${PATH_OSMSCOUT}/lib -losmscout
LIBS += -L$${PATH_OSMSCOUT}/lib -losmscoutimport
