PATH_OSMSCOUT = /home/preet/Dev/env/sys/libosmscout
PATH_MARISA = /home/preet/Dev/env/sys/marisa

TEMPLATE = app
CONFIG += console debug
CONFIG -= qt
SOURCES += osmscout_json_import.cpp

#boost
DEFINES += USE_BOOST
INCLUDEPATH += /home/preet/Dev/env/sys/boost-1.55

#jansson
LIBS += -ljansson

#libosmscout
INCLUDEPATH += $${PATH_OSMSCOUT}/include
LIBS += -L$${PATH_OSMSCOUT}/lib -losmscout
LIBS += -L$${PATH_OSMSCOUT}/lib -losmscoutimport

#marisa
INCLUDEPATH += $${PATH_MARISA}/include
LIBS += -L$${PATH_MARISA}/lib -lmarisa
