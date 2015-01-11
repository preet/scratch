TEMPLATE    = app
TARGET      = runme
CONFIG      -= qt

INCLUDEPATH += $${_PRO_FILE_PWD_}

SOURCES += test1_triangle.cpp

# sdl
PATH_SDL = /home/preet/Dev/env/sys/sdl2
INCLUDEPATH += $${PATH_SDL}/include
DEFINES += _REENTRANT
LIBS += -L$${PATH_SDL}/lib -lSDL2

# glad opengl function loader
PATH_GLAD = $${_PRO_FILE_PWD_}/opengl_2_1
INCLUDEPATH += $${PATH_GLAD}
HEADERS += $${PATH_GLAD}/glad/glad.h
SOURCES += $${PATH_GLAD}/glad/glad.c
LIBS += -ldl

# need these flags for gcc 4.8.x bug for threads
QMAKE_LFLAGS += -Wl,--no-as-needed
LIBS += -lpthread
QMAKE_CXXFLAGS += -std=c++11
