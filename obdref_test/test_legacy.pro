TEMPLATE = app
TARGET = test_legacy
QT += core

SOURCES += test_legacy.cpp

# obdref lib
INCLUDEPATH += /home/preet/Dev/build/obdref
LIBS += -L/home/preet/Dev/build/obdref -lobdref -lv8
