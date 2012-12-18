TEMPLATE = app
CONFIG += link_pkgconfig debug
PKGCONFIG += openthreads openscenegraph

OPENCTM = ../../gis/polytoolkit/openctm
INCLUDEPATH += $${OPENCTM}

#liblzma
HEADERS +=  $${OPENCTM}/liblzma/Alloc.h \
            $${OPENCTM}/liblzma/LzFind.h \
            $${OPENCTM}/liblzma/LzHash.h \
            $${OPENCTM}/liblzma/LzmaEnc.h \
            $${OPENCTM}/liblzma/LzmaLib.h \
            $${OPENCTM}/liblzma/NameMangle.h \
            $${OPENCTM}/liblzma/Types.h

SOURCES +=  $${OPENCTM}/liblzma/Alloc.c \
            $${OPENCTM}/liblzma/LzFind.c \
            $${OPENCTM}/liblzma/LzmaDec.c \
            $${OPENCTM}/liblzma/LzmaEnc.c \
            $${OPENCTM}/liblzma/LzmaLib.c

# openctm
HEADERS += $${OPENCTM}/openctmpp.h \
           $${OPENCTM}/openctm.h \
           $${OPENCTM}/internal.h

SOURCES += $${OPENCTM}/stream.c \
           $${OPENCTM}/openctm.c \
           $${OPENCTM}/compressRAW.c \
           $${OPENCTM}/compressMG2.c \
           $${OPENCTM}/compressMG1.c


SOURCES += main.cpp
