CONFIG += qt
SOURCES += main.cpp
TEMPLATE = app

#openscenegraph
OSGDIR = /home/preet/Documents/openscenegraph
OSGLIBDIR = /home/preet/Documents/openscenegraph/lib64
    INCLUDEPATH += $${OSGDIR}/include
    LIBS += -L$${OSGLIBDIR} -losgViewer
    LIBS += -L$${OSGLIBDIR} -losgText
    LIBS += -L$${OSGLIBDIR} -losgGA
    LIBS += -L$${OSGLIBDIR} -losgUtil
    LIBS += -L$${OSGLIBDIR} -losgDB
    LIBS += -L$${OSGLIBDIR} -losg
    LIBS += -L$${OSGLIBDIR} -lOpenThreads

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
           
# define other resources and install path
res_local.path = $${OUT_PWD}
res_local.files += \
   shaders \
   models

INSTALLS += res_local
