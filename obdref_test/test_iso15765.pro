TEMPLATE = app
TARGET = test_iso15765
QT += core

SOURCES += test_iso15765.cpp

# obdref lib
INCLUDEPATH += /home/preet/Dev/build/obdref
LIBS += -L/home/preet/Dev/build/obdref -lobdref -lv8
