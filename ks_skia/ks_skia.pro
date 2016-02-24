TEMPLATE    = app
TARGET      = ks_skia


# sdl
PATH_SDL = /home/preet/Dev/env/sys/sdl2
PATH_SDL_LIB = /home/preet/Dev/env/sys/sdl2/lib
PATH_SDL_INCLUDE = /home/preet/Dev/env/sys/sdl2/include

#PATH_SDL = /home/preet/Dev/env/android/sdl2
#PATH_SDL_LIB = /home/preet/Dev/env/sys/sdl2/libs/armeabi-v7a
#PATH_SDL_INCLUDE = /home/preet/Dev/env/android/sdl2/include

# ks
PATH_KS = /home/preet/Dev/projects/ks
include($${PATH_KS}/ks_test/ks.pri)

# skia
include($${_PRO_FILE_PWD_}/skia.pri)

# ks_skia
PATH_KS_SKIA = $${_PRO_FILE_PWD_}
SOURCES += $${PATH_KS_SKIA}/main.cpp
