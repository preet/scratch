TEMPLATE = app
TARGET = test_random_input
QT += core

SOURCES += test_random_input.cpp

# obdref lib
INCLUDEPATH += /home/preet/Dev/build/obdref
LIBS += -L/home/preet/Dev/build/obdref -lobdref -lv8
