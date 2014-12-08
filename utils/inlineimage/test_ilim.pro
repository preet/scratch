TEMPLATE    = app
TARGET      = runme
CONFIG      -= qt

INCLUDEPATH += $${PWD}

SOURCES += test_ilim.cpp

# need these flags for gcc 4.8.x bug for threads
# QMAKE_LFLAGS += -Wl,--no-as-needed
# LIBS += -lpthread
# QMAKE_CXXFLAGS += -std=c++11
