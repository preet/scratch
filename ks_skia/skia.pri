
# skia
PATH_SKIA_INCLUDE = /home/preet/Packages/skia/skia/include
PATH_SKIA_INCLUDE_PRIVATE = /home/preet/Packages/skia/skia/include/private
PATH_SKIA_SRC = /home/preet/Packages/skia/skia/src

INCLUDEPATH += $${PATH_SKIA_INCLUDE}
INCLUDEPATH += $${PATH_SKIA_INCLUDE_PRIVATE}



# sources

# The skia CMakeLists removes the following
# animator
# nacl
# svg
# views
# xml

# In addition get rid of
# c (c api?)
# doc
# pdf
# device/xps

#android {
#    # not sure what this is for
#    SOURCES += $${PATH_SKIA_SRC}/android/*cpp
#}

# ========================================================= #

win32 {
DEFINES += "SK_BUILD_FOR_WIN32=1"
}
android {
DEFINES += "SK_BUILD_FOR_ANDROID=1"
}
unix:!mac {
DEFINES += "SK_BUILD_FOR_UNIX=1"
}
mac:!unix {
DEFINES += "SK_BUILD_FOR_MAC=1"
}
ios {
DEFINES += "SK_BUILD_FOR_IOS=1"
}

# ========================================================= #

INCLUDEPATH += $${PATH_SKIA_INCLUDE}/android
INCLUDEPATH += $${PATH_SKIA_INCLUDE}/client
INCLUDEPATH += $${PATH_SKIA_INCLUDE}/client/android
INCLUDEPATH += $${PATH_SKIA_INCLUDE}/codec
INCLUDEPATH += $${PATH_SKIA_INCLUDE}/config
INCLUDEPATH += $${PATH_SKIA_INCLUDE}/core
INCLUDEPATH += $${PATH_SKIA_INCLUDE}/effects
INCLUDEPATH += $${PATH_SKIA_INCLUDE}/gpu
INCLUDEPATH += $${PATH_SKIA_INCLUDE}/images
INCLUDEPATH += $${PATH_SKIA_INCLUDE}/pathops
INCLUDEPATH += $${PATH_SKIA_INCLUDE}/ports
INCLUDEPATH += $${PATH_SKIA_INCLUDE}/private
INCLUDEPATH += $${PATH_SKIA_INCLUDE}/utils

INCLUDEPATH += $${PATH_SKIA_SRC}/android
INCLUDEPATH += $${PATH_SKIA_SRC}/codec
INCLUDEPATH += $${PATH_SKIA_SRC}/core
INCLUDEPATH += $${PATH_SKIA_SRC}/effects
INCLUDEPATH += $${PATH_SKIA_SRC}/fonts
INCLUDEPATH += $${PATH_SKIA_SRC}/gpu
INCLUDEPATH += $${PATH_SKIA_SRC}/gpu/batches
INCLUDEPATH += $${PATH_SKIA_SRC}/gpu/gl/builders
INCLUDEPATH += $${PATH_SKIA_SRC}/gpu/gl
INCLUDEPATH += $${PATH_SKIA_SRC}/gpu/glsl
INCLUDEPATH += $${PATH_SKIA_SRC}/gpu/effects
INCLUDEPATH += $${PATH_SKIA_SRC}/gpu/text
INCLUDEPATH += $${PATH_SKIA_SRC}/image
INCLUDEPATH += $${PATH_SKIA_SRC}/images
INCLUDEPATH += $${PATH_SKIA_SRC}/lazy
INCLUDEPATH += $${PATH_SKIA_SRC}/opts
INCLUDEPATH += $${PATH_SKIA_SRC}/pathops
INCLUDEPATH += $${PATH_SKIA_SRC}/ports
INCLUDEPATH += $${PATH_SKIA_SRC}/sfnt
INCLUDEPATH += $${PATH_SKIA_SRC}/utils
INCLUDEPATH += $${PATH_SKIA_SRC}/utils/mac
INCLUDEPATH += $${PATH_SKIA_SRC}/utils/ios
INCLUDEPATH += $${PATH_SKIA_SRC}/utils/win
INCLUDEPATH += $${PATH_SKIA_SRC}/../third_party/etc1
INCLUDEPATH += $${PATH_SKIA_SRC}/../third_party/ktx

# core
SOURCES += $$files($${PATH_SKIA_SRC}/core/*cpp)
SOURCES -= $${PATH_SKIA_SRC}/core/SkForceCPlusPlusLinking.cpp # This file is empty and is only used to trick GYP.

!contains(QT_ARCH,arm) {
   SOURCES -= $${PATH_SKIA_SRC}/core/*Arm*cpp
}

# thirdparty
SOURCES += \
   $$files($${PATH_SKIA_SRC}/../third_party/etc1/*cpp) \
   $$files($${PATH_SKIA_SRC}/../third_party/ktx/*cpp)

# codec
SOURCES += $$files($${PATH_SKIA_SRC}/codec/*cpp)
SOURCES -= $$files($${PATH_SKIA_SRC}/codec/*Raw*)
SOURCES -= $$files($${PATH_SKIA_SRC}/codec/*Gif*)
SOURCES -= $$files($${PATH_SKIA_SRC}/codec/*Jpeg*)
SOURCES -= $$files($${PATH_SKIA_SRC}/codec/*Png*)
SOURCES -= $$files($${PATH_SKIA_SRC}/codec/*Ico*)
SOURCES -= $$files($${PATH_SKIA_SRC}/codec/*Webp*)

# effects
SOURCES += $$files($${PATH_SKIA_SRC}/effects/*cpp)
SOURCES += $$files($${PATH_SKIA_SRC}/effects/gradients/*cpp)

# fonts
SOURCES += $$files($${PATH_SKIA_SRC}/fonts/*cpp)
SOURCES -= $${PATH_SKIA_SRC}/fonts/SkFontMgr_fontconfig.cpp

# gpu
SOURCES += $$files($${PATH_SKIA_SRC}/gpu/*cpp)
SOURCES += $$files($${PATH_SKIA_SRC}/gpu/batches/*cpp)
SOURCES += $$files($${PATH_SKIA_SRC}/gpu/effects/*cpp)
SOURCES += $$files($${PATH_SKIA_SRC}/gpu/gl/*cpp)
SOURCES += $$files($${PATH_SKIA_SRC}/gpu/gl/builders/*cpp)

android {
   $$files(SOURCES += $${PATH_SKIA_SRC}/gpu/gl/android/*cpp)
}
win32 {
   # TODO, no idea how to set this up
   $$files(SOURCES += $${PATH_SKIA_SRC}/gpu/gl/angle/*cpp)
   $$files(SOURCES += $${PATH_SKIA_SRC}/gpu/gl/win/*cpp)
}
mac:!ios {
   SOURCES += $$files($${PATH_SKIA_SRC}/gpu/gl/mac/*cpp)
}
unix:!mac {
   SOURCES += $$files($${PATH_SKIA_SRC}/gpu/gl/glx/*cpp)
}
ios {
   SOURCES += $$files($${PATH_SKIA_SRC}/gpu/gl/iOS/*cpp)
}

SOURCES += $$files($${PATH_SKIA_SRC}/gpu/glsl/*cpp)
SOURCES += $$files($${PATH_SKIA_SRC}/gpu/text/*cpp)

SOURCES -= $${PATH_SKIA_SRC}/gpu/GrContextFactory.cpp # For internal testing only.
SOURCES -= $${PATH_SKIA_SRC}/gpu/gl/GrGLCreateNativeInterface_none.cpp
SOURCES -= $${PATH_SKIA_SRC}/gpu/gl/GrGLDefaultInterface_none.cpp
SOURCES -= $$files($${PATH_SKIA_SRC}/gpu/gl/SkCreatePlatformGLContext*.cpp) # For internal testing only.


# image
SOURCES += $$files($${PATH_SKIA_SRC}/image/*cpp)

# images
SOURCES += $$files($${PATH_SKIA_SRC}/images/*cpp)
SOURCES -= $$files($${PATH_SKIA_SRC}/images/SkForceLinking.cpp)
SOURCES -= $$files($${PATH_SKIA_SRC}/images/*gif*)
SOURCES -= $$files($${PATH_SKIA_SRC}/images/*jpeg*)
SOURCES -= $$files($${PATH_SKIA_SRC}/images/*Jpeg*)
SOURCES -= $$files($${PATH_SKIA_SRC}/images/*png*)
SOURCES -= $$files($${PATH_SKIA_SRC}/images/*webp*)

# lazy
SOURCES += $$files($${PATH_SKIA_SRC}/lazy/*cpp)

# opts
# Put together from opts.gyp and opts.gypi (TODO)
SOURCES_OPTS_NONE = $$files($${PATH_SKIA_SRC}/opts/*none.cpp)

SOURCES_OPTS_ARMV7 = $$files($${PATH_SKIA_SRC}/opts/*arm.cpp)

SOURCES_OPTS_NEON = $$files($${PATH_SKIA_SRC}/opts/*neon.cpp)

SOURCES_OPTS_MIPS = $$files($${PATH_SKIA_SRC}/opts/*mips*cpp)
SOURCES_OPTS_MIPS += $$files($${PATH_SKIA_SRC}/opts/SkBlitMask_opts_none.cpp)

SOURCES_OPTS_SSE2 = $$files($${PATH_SKIA_SRC}/opts/*SSE2*cpp)
SOURCES_OPTS_SSE2 += $$files($${PATH_SKIA_SRC}/opts/opts_check_x86.cpp)

SOURCES_OPTS_SSSE3 = $$files($${PATH_SKIA_SRC}/opts/*SSSE3*cpp)
SOURCES_OPTS_SSSE3 += $$files($${PATH_SKIA_SRC}/opts/*ssse3*cpp)

SOURCES_OPTS_SSE41 = $$files($${PATH_SKIA_SRC}/opts/*SSE4*cpp)
SOURCES_OPTS_SSE41 += $$files($${PATH_SKIA_SRC}/opts/*sse4*cpp)

# To specify compilation flags for individual files,
# http://stackoverflow.com/questions/14604242/how-to-specify-separate-compilation-options-for-different-targets-in-qmake
# http://stackoverflow.com/questions/27683777/how-to-specify-compiler-flag-to-a-single-source-file-with-qmake

contains(QT_ARCH,x86_64) {
   compile_sse2.input = SOURCES_OPTS_SSE2
   compile_sse2.output = ${QMAKE_FILE_BASE}.o
   compile_sse2.commands = $${QMAKE_CXX} $(CXXFLAGS) $(INCPATH) -c ${QMAKE_FILE_NAME} -msse2

   compile_ssse3.input = SOURCES_OPTS_SSSE3
   compile_ssse3.output = ${QMAKE_FILE_BASE}.o
   compile_ssse3.commands = $${QMAKE_CXX} $(CXXFLAGS) $(INCPATH) -c ${QMAKE_FILE_NAME} -mssse3

   compile_sse41.input = SOURCES_OPTS_SSE41
   compile_sse41.output = ${QMAKE_FILE_BASE}.o
   compile_sse41.commands = $${QMAKE_CXX} $(CXXFLAGS) $(INCPATH) -c ${QMAKE_FILE_NAME} -msse4.1

   QMAKE_EXTRA_COMPILERS += compile_sse2
   QMAKE_EXTRA_COMPILERS += compile_ssse3
   QMAKE_EXTRA_COMPILERS += compile_sse41
}
contains(QT_ARCH,arm) {
   # I think this assumes armv7, dunno what to do
   # for arm version less than 7
   compile_armv7.input = SOURCES_OPTS_ARMV7
   compile_armv7.output = ${QMAKE_FILE_BASE}.o
   compile_armv7.commands = $${QMAKE_CXX} $(CXXFLAGS) $(INCPATH) -c ${QMAKE_FILE_NAME} -fomit-frame-pointer

   compile_neon.input = SOURCES_OPTS_NEON
   compile_neon.output = ${QMAKE_FILE_BASE}.o
   compile_neon.commands = $${QMAKE_CXX} $(CXXFLAGS) $(INCPATH) -c ${QMAKE_FILE_NAME} -fomit-frame-pointer -mfpu=neon

   QMAKE_EXTRA_COMPILERS += compile_armv7
   QMAKE_EXTRA_COMPILERS += compile_neon
}
contains(QT_ARCH,mips) {
   SOURCES += SOURCES_OPTS_MIPS
}

# pathops
SOURCES += $$files($${PATH_SKIA_SRC}/pathops/*cpp)

# ports
SOURCES += \
$${PATH_SKIA_SRC}/ports/SkDiscardableMemory_none.cpp \
$${PATH_SKIA_SRC}/ports/SkFontHost_FreeType_common.cpp \
$${PATH_SKIA_SRC}/ports/SkFontHost_FreeType.cpp \
$${PATH_SKIA_SRC}/ports/SkFontMgr_custom.cpp \
$${PATH_SKIA_SRC}/ports/SkFontMgr_empty_factory.cpp \
$${PATH_SKIA_SRC}/ports/SkGlobalInitialization_default.cpp \
##$${PATH_SKIA_SRC}/ports/SkImageDecoder_empty.cpp \
$${PATH_SKIA_SRC}/ports/SkImageGenerator_skia.cpp \
$${PATH_SKIA_SRC}/ports/SkMemory_malloc.cpp \
$${PATH_SKIA_SRC}/ports/SkOSEnvironment.cpp
##$${PATH_SKIA_SRC}/ports/SkImageDecoder_CG.cpp \ # some mac thing
##$${PATH_SKIA_SRC}/ports/SkImageDecoder_WIC.cpp \ # some win thing
##$${PATH_SKIA_SRC}/ports/SkImageGenerator_none.cpp \

android {
SOURCES += $${PATH_SKIA_SRC}/ports/SkDebug_android.cpp
} else:win32 {
SOURCES += $${PATH_SKIA_SRC}/ports/SkDebug_win.cpp
}
#else {
#}
SOURCES += $${PATH_SKIA_SRC}/ports/SkDebug_stdio.cpp
SOURCES += $${PATH_SKIA_SRC}/ports/SkOSFile_stdio.cpp

win32 {
SOURCES += \
   $${PATH_SKIA_SRC}/ports/SkOSFile_win.cpp \
   $${PATH_SKIA_SRC}/ports/SkOSLibrary_win.cpp \
   $${PATH_SKIA_SRC}/ports/SkTime_win.cpp \
   $${PATH_SKIA_SRC}/ports/SkTLS_win.cpp
} else {
SOURCES += \
   $${PATH_SKIA_SRC}/ports/SkOSFile_posix.cpp \
   $${PATH_SKIA_SRC}/ports/SkOSLibrary_posix.cpp \
   $${PATH_SKIA_SRC}/ports/SkTime_Unix.cpp \
   $${PATH_SKIA_SRC}/ports/SkTLS_pthread.cpp
}

# sfnt
SOURCES += $$files($${PATH_SKIA_SRC}/sfnt/*cpp)

# utils
SOURCES += $$files($${PATH_SKIA_SRC}/utils/*cpp)

win32 {
   SOURCES += $$files($${PATH_SKIA_SRC}/utils/win/*cpp)
} else:mac {
   SOURCES += $$files($${PATH_SKIA_SRC}/utils/mac/*cpp)
}

!win32 {
SOURCES -= $$files($${PATH_SKIA_SRC}/utils/*_win*)
}

!unix {
SOURCES -= $$files($${PATH_SKIA_SRC}/utils/*_pthread*)
}

SOURCES -= $$files($${PATH_SKIA_SRC}/utils/*Lua*)

LIBS += -lGLU -lGL -lX11
