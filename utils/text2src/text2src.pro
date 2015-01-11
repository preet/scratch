TEMPLATE    = app
TARGET      = text2src
CONFIG      -= qt

INCLUDEPATH += $${PWD}

SOURCES += text2src.cpp

# need these flags for gcc 4.8.x bug for threads
QMAKE_CXXFLAGS += -std=c++11
