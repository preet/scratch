TEMPLATE = app
TARGET = test_iso14230
QT += core

SOURCES += test_iso14230.cpp

# obdref lib
INCLUDEPATH += /home/preet/Dev/build/obdref
LIBS += -L/home/preet/Dev/build/obdref -lobdref -lv8
