TEMPLATE    = app
TARGET      = runme
CONFIG      -= qt

INCLUDEPATH += $${PWD}

HEADERS += smlog.h
SOURCES += smlog.cpp test_smlog.cpp

QMAKE_CXXFLAGS += -std=c++11
