TEMPLATE    = app
TARGET      = asio
CONFIG      -= qt

SOURCES += main2.cpp

PATH_KS = /home/preet/Dev/projects/ks/kscore
INCLUDEPATH += $${PATH_KS}

HEADERS += $${PATH_KS}/ks/KsLog.h
SOURCES += $${PATH_KS}/ks/KsLog.cpp

# asio
include($${PATH_KS}/ks/thirdparty/asio/asio.pri)

# need these flags for gcc 4.8.x bug
QMAKE_LFLAGS += -Wl,--no-as-needed
LIBS += -lpthread
QMAKE_CXXFLAGS += -std=c++11
