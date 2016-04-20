TEMPLATE    = app
TARGET      = runme
CONFIG      -= qt

INCLUDEPATH += $${_PRO_FILE_PWD_}

SOURCES += test_inputxy.cpp


!android {
    PATH_SDL = /home/preet/Dev/env/sys/sdl2
    PATH_SDL_LIB = /home/preet/Dev/env/sys/sdl2/lib
    PATH_SDL_INCLUDE = /home/preet/Dev/env/sys/sdl2/include
}
android {
    PATH_SDL = /home/preet/Dev/env/android/sdl2
    PATH_SDL_LIB = /home/preet/Dev/env/android/sdl2/libs/armeabi-v7a
    PATH_SDL_INCLUDE = /home/preet/Dev/env/android/sdl2/include
}

linux {
    !android {
        # sdl
        INCLUDEPATH += $${PATH_SDL_INCLUDE}
        DEFINES += _REENTRANT
        LIBS += -L$${PATH_SDL_LIB} -lSDL2
        QMAKE_RPATHDIR += $${PATH_SDL_LIB} # So that libs in /usr/lib aren't prioritized

        # glad opengl function loader
        PATH_GLAD = $${_PRO_FILE_PWD_}/opengl_2_1
        INCLUDEPATH += $${PATH_GLAD}
        HEADERS += $${PATH_GLAD}/glad/glad.h
        SOURCES += $${PATH_GLAD}/glad/glad.c
        LIBS += -ldl
    }
}

android {
    # sdl
    SOURCES += SDL_android_main.c

    INCLUDEPATH += $${PATH_SDL_INCLUDE}
    DEFINES += ENV_ANDROID_SDL
    DEFINES += _REENTRANT
    LIBS += -L$${PATH_SDL_LIB} -lSDL2
    LIBS += -lGLESv2 -lEGL
}




# need these flags for gcc 4.8.x bug for threads
#QMAKE_LFLAGS += -Wl,--no-as-needed
!android {
    LIBS += -lpthread
}
QMAKE_CXXFLAGS += -std=c++11
