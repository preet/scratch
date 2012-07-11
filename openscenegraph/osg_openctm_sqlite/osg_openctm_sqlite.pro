CONFIG += link_pkgconfig debug
PKGCONFIG += openthreads openscenegraph
TARGET = osg_openctm_sqlite
SOURCES += osg_openctm_sqlite.cpp
TEMPLATE = app

# kompex sqlite wrapper
DEFINES += SQLITE_OMIT_LOAD_EXTENSION

HEADERS += \
    kompex/sqlite3.h \
    kompex/KompexSQLiteStreamRedirection.h \
    kompex/KompexSQLiteStatement.h \
    kompex/KompexSQLitePrerequisites.h \
    kompex/KompexSQLiteException.h \
    kompex/KompexSQLiteDatabase.h \
    kompex/KompexSQLiteBlob.h

SOURCES += \
    kompex/sqlite3.c \
    kompex/KompexSQLiteStatement.cpp \
    kompex/KompexSQLiteDatabase.cpp \
    kompex/KompexSQLiteBlob.cpp

# can define SQLITE_THREADSAFE=0 to avoid this dep
LIBS += -lpthread

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

QMAKE_CXXFLAGS += -std=c++0x
