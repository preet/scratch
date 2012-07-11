##########################
# START CONFIG FOR QT

QT += declarative opengl

device|simulator {

    # directory where qt and bb libs are
    # (depends on device or simulator build config)

    QTDIR = temp
    device   {
       QTDIR = /home/preet/Documents/qnx/qt-4.8-arm
    }
    simulator {
       QTDIR = /home/preet/Documents/qnx/qt-4.8-x86
    }

    # define qt lib files and install path
    dep_libs.path = $${OUT_PWD}/deploy/lib
    dep_libs.files += $${QTDIR}/lib/*.so.4

    # define bb lib file and install path
    dep_plugins.path = $${OUT_PWD}/deploy/plugins/platforms
    dep_plugins.files = $${QTDIR}/plugins/platforms/libblackberry.so

    INSTALLS += dep_libs dep_plugins
}

# END CONFIG FOR QT
##########################

##########################
# START PROJECT CONFIG

device|simulator {
    DEFINES += DEV_PLAYBOOK
}

!device {
    !simulator {
        DEFINES += DEV_PC
    }
}

INCLUDEPATH += /home/preet/Downloads/Packages/glm

SOURCES += \
    main.cpp \
    qdectoucharea.cpp \
    qdecviewportitem.cpp \
    demotexture.cpp

#    demohellotri.cpp \
#    demomvp.cpp \
#    democube.cpp \
#    demomodel.cpp \
#    modelviewer.cpp \
#    demotexture.cpp


HEADERS += \
    qdectoucharea.h \
    qdecviewportitem.h \
    demotexture.h

#    demohellotri.h \
#    demomvp.h \
#    democube.h \
#    demomodel.h \
#    modelviewer.h \
#    demotexture.h

OTHER_FILES += \
    ui/main.qml \
    bar_descriptor.xml

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
    splashscreen.png

# define other resources and install path
res_local.path = $${OUT_PWD}
res_local.files += \
   ui \
   shaders \
   models \
   textures

INSTALLS += pkg_files res_local

# END PROJECT CONFIG
##########################

