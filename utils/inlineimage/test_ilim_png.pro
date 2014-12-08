TEMPLATE    = app
QT += core quick

INCLUDEPATH += $${PWD}

HEADERS += lodepng/lodepng.h
SOURCES += lodepng/lodepng.cpp

HEADERS += test_ilim_png.h
SOURCES += test_ilim_png.cpp

# need these flags for gcc 4.8.x bug for threads
# QMAKE_LFLAGS += -Wl,--no-as-needed
# LIBS += -lpthread
 QMAKE_CXXFLAGS += -std=c++11
