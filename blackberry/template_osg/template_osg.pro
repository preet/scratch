##########################
# START PROJECT CONFIG

LIBS += -lbps -lscreen -lGLESv2 -lEGL -lfreetype -lpng
DEFINES += USING_GL20  # for bbutil

SOURCES += \
    main.cpp \
    bbutil.c

HEADERS += \
    bbutil.h

# openscenegraph
OSGDIR = temp
device {
    OSGDIR = /home/preet/Documents/openscenegraph-arm
}
simulator {
    OSGDIR = /home/preet/Documents/openscenegraph-x86
}
OGRPLUGINSDIR = $${OSGDIR}/lib/osgPlugins-3.1.3
DEFINES += OSG_LIBRARY_STATIC
INCLUDEPATH += $${OSGDIR}/include
LIBS += -L$${OSGDIR}/lib -losgViewerd
LIBS += -L$${OSGDIR}/lib -losgGAd
LIBS += -L$${OSGDIR}/lib -losgUtild
LIBS += -L$${OSGDIR}/lib -losgDBd
LIBS += -L$${OSGDIR}/lib -losgd
LIBS += -L$${OSGDIR}/lib -lOpenThreadsd

#liblzma
HEADERS +=  openctm/liblzma/Alloc.h \
            openctm/liblzma/LzFind.h \
            openctm/liblzma/LzHash.h \
            openctm/liblzma/LzmaEnc.h \
            openctm/liblzma/LzmaLib.h \
            openctm/liblzma/NameMangle.h \
            openctm/liblzma/Types.h

SOURCES +=  openctm/liblzma/Alloc.c \
            openctm/liblzma/LzFind.c \
            openctm/liblzma/LzmaDec.c \
            openctm/liblzma/LzmaEnc.c \
            openctm/liblzma/LzmaLib.c

# openctm
HEADERS += openctm/openctmpp.h \
           openctm/openctm.h \
           openctm/internal.h

SOURCES += openctm/stream.c \
           openctm/openctm.c \
           openctm/compressRAW.c \
           openctm/compressMG2.c \
           openctm/compressMG1.c

# bar descriptor, icon, splashscreen
pkg_files.path = $${OUT_PWD}
pkg_files.files += \
    bar_descriptor.xml \
    icon.png \
    splashscreen.png \
    models

INSTALLS += pkg_files

# END PROJECT CONFIG
##########################
