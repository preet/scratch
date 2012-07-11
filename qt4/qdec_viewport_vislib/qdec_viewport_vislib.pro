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
       QTDIR = /home/preet/Documents/qnx/qt-4.8-arm
       OSGDIR = /home/preet/Documents/qnx/osg-arm-rel
    }
    simulator {
       QTDIR = /home/preet/Documents/qnx/qt-4.8-x86
       OSGDIR = /home/preet/Documents/qnx/osg-x86-rel
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
}

!device {
    !simulator {
        DEFINES += DEV_PC

        #visualization library
        VLDIR = /home/preet/Documents/visualizationlibrary
        VLLIBDIR = /home/preet/Documents/visualizationlibrary/lib
            INCLUDEPATH += $${VLDIR}/include
            LIBS += -L$${VLLIBDIR} -lVLCore
            LIBS += -L$${VLLIBDIR} -lVLGraphics
    }
}

SOURCES += \
    main.cpp \
    qdectoucharea.cpp \
    qdecviewportitem.cpp \
    qdecviewportvislib.cpp


HEADERS += \
    qdectoucharea.h \
    qdecviewportitem.h \
    qdecviewportvislib.h
    
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
   models

INSTALLS += pkg_files res_local

# END PROJECT CONFIG
##########################

