LIBOSMSCOUT_PATH = /home/preet/Dev/env/sys/libosmscout

TEMPLATE = app
CONFIG += console debug
CONFIG -= qt
SOURCES += osmscout_import.cpp
DEFINES += "USE_BOOST=1"
TARGET = import_json

#jansson
LIBS += -ljansson

# boost
INCLUDEPATH += /home/preet/Dev/env/sys/boost-1.51

#libosmscout
INCLUDEPATH += /home/preet/Dev/env/sys/boost-1.50
INCLUDEPATH += $${LIBOSMSCOUT_PATH}/include
LIBS += -L$${LIBOSMSCOUT_PATH}/lib -losmscout

#libosmscout-import
LIBS += -L$${LIBOSMSCOUT_PATH}/lib -losmscoutimport

QMAKE_CXXFLAGS += -msse2    # what the hell
