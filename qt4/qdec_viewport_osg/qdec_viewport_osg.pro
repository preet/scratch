##########################
# START PROJECT CONFIG

# qt
QT += declarative opengl

device|simulator {

    # directory where qt and bb libs are
    # (depends on device or simulator build config)

    QTDIR = temp
    OSGDIR = temp

    device   {
       QTDIR = /home/preet/Dev/env/qnx/qt4-arm
       OSGDIR = /home/preet/Dev/env/qnx/osg-arm
    }
    simulator {
       QTDIR = MISSING
       OSGDIR = MISSING
    }

    # define qt lib files and install path
    dep_libs.path = $${OUT_PWD}/deploy/lib
    dep_libs.files += $${QTDIR}/lib/*.so.4
    dep_libs.files += $${OSGDIR}/lib/*.so

    # define bb lib file and install path
    dep_plugins.path = $${OUT_PWD}/deploy/plugins/platforms
    dep_plugins.files = $${QTDIR}/plugins/platforms/libblackberry.so

    INSTALLS += dep_libs dep_plugins
}


device|simulator {
    DEFINES += DEV_PLAYBOOK

    # openscenegraph
    INCLUDEPATH += $${OSGDIR}/include

    LIBS += -L$${OSGDIR}/lib/osgdb_freetype.so
    LIBS += -L$${OSGDIR}/lib/osgdb_jpeg.so
    LIBS += -L$${OSGDIR}/lib/osgdb_png.so

    LIBS += -L$${OSGDIR}/lib -losgViewerd
    LIBS += -L$${OSGDIR}/lib -losgTextd
    LIBS += -L$${OSGDIR}/lib -losgGAd
    LIBS += -L$${OSGDIR}/lib -losgUtild
    LIBS += -L$${OSGDIR}/lib -losgDBd
    LIBS += -L$${OSGDIR}/lib -losgd
    LIBS += -L$${OSGDIR}/lib -lOpenThreadsd
}

!device {
    !simulator {
        DEFINES += DEV_PC

#CONFIG += link_pkgconfig
#PKGCONFIG += openthreads openscenegraph

#openscenegraph
    # mobile version for OpenGL ES 2
    DEFINES += GL_MOBILE
    OSGDIR = /home/preet/Dev/env/sys/osg-modern
    OSGLIBDIR = /home/preet/Dev/env/sys/osg-modern/lib64
    INCLUDEPATH += $${OSGDIR}/include
    LIBS += -L$${OSGLIBDIR}/osgdb_freetyperd.so
    LIBS += -L$${OSGLIBDIR}/osgdb_jpegrd.so
    LIBS += -L$${OSGLIBDIR}/osgdb_pngrd.so
    LIBS += -L$${OSGLIBDIR} -losgViewerrd
    LIBS += -L$${OSGLIBDIR} -losgTextrd
    LIBS += -L$${OSGLIBDIR} -losgGArd
    LIBS += -L$${OSGLIBDIR} -losgUtilrd
    LIBS += -L$${OSGLIBDIR} -losgDBrd
    LIBS += -L$${OSGLIBDIR} -losgrd
    LIBS += -L$${OSGLIBDIR} -lOpenThreadsrd
    }
}

SOURCES += \
    main.cpp \
    qdectoucharea.cpp \
    qdecviewportitem.cpp \
    qdecviewportosg.cpp


HEADERS += \
    qdectoucharea.h \
    qdecviewportitem.h \
    qdecviewportosg.h
    
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
res_local.files += *.ttf \
    ui \
    shaders \
    models \
    textures

INSTALLS += pkg_files res_local

# END PROJECT CONFIG
##########################

