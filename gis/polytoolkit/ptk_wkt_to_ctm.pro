TEMPLATE = app
CONFIG += console debug
CONFIG -= qt
TARGET = ptk_wkt_to_ctm

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

# ptk_wkt_to_ctm
SOURCES += ptk_wkt_to_ctm.cpp

# required libs
LIBS += -lgdal -lCGAL_Core -lCGAL -lmpfr -lgmp -lboost_thread

# c++0x
QMAKE_CXXFLAGS += -std=c++0x
