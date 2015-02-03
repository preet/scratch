TEMPLATE    = app
TARGET      = asio
CONFIG      -= qt

SOURCES += main.cpp

PATH_KS = /home/preet/Dev/projects/ks/ks
INCLUDEPATH += $${PATH_KS}

# asio
include($${PATH_KS}/thirdparty/asio/asio.pri)

# need these flags for gcc 4.8.x bug
QMAKE_LFLAGS += -Wl,--no-as-needed
LIBS += -lpthread
QMAKE_CXXFLAGS += -std=c++11
