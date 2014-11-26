TEMPLATE    = app
TARGET      = runme
CONFIG      -= qt

HEADERS += \
    ThreadPool.h

SOURCES += \
    ThreadPool.cpp

SOURCES += main.cpp

# need these flags for gcc 4.8.x bug
QMAKE_LFLAGS += -Wl,--no-as-needed
LIBS += -lpthread
QMAKE_CXXFLAGS += -std=c++11
