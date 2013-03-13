CONFIG   += console link_pkgconfig
CONFIG   -= qt
TEMPLATE = app

# main
SOURCES += shptk_meshdb.cpp

# kompex
PATH_KOMPEX = /home/preet/Dev/scratch/thirdparty/kompex
INCLUDEPATH += $${PATH_KOMPEX}
DEFINES += SQLITE_OMIT_LOAD_EXTENSION
HEADERS += \
    $${PATH_KOMPEX}/sqlite3.h \
    $${PATH_KOMPEX}/KompexSQLiteStreamRedirection.h \
    $${PATH_KOMPEX}/KompexSQLiteStatement.h \
    $${PATH_KOMPEX}/KompexSQLitePrerequisites.h \
    $${PATH_KOMPEX}/KompexSQLiteException.h \
    $${PATH_KOMPEX}/KompexSQLiteDatabase.h \
    $${PATH_KOMPEX}/KompexSQLiteBlob.h

SOURCES += \
    $${PATH_KOMPEX}/sqlite3.c \
    $${PATH_KOMPEX}/KompexSQLiteStatement.cpp \
    $${PATH_KOMPEX}/KompexSQLiteDatabase.cpp \
    $${PATH_KOMPEX}/KompexSQLiteBlob.cpp

LIBS += -lpthread

# openctm
PATH_OPENCTM = /home/preet/Dev/scratch/gis/polytoolkit/openctm
INCLUDEPATH += $${PATH_OPENCTM}

    #liblzma
    HEADERS +=  $${PATH_OPENCTM}/liblzma/Alloc.h \
                $${PATH_OPENCTM}/liblzma/LzFind.h \
                $${PATH_OPENCTM}/liblzma/LzHash.h \
                $${PATH_OPENCTM}/liblzma/LzmaEnc.h \
                $${PATH_OPENCTM}/liblzma/LzmaLib.h \
                $${PATH_OPENCTM}/liblzma/NameMangle.h \
                $${PATH_OPENCTM}/liblzma/Types.h

    SOURCES +=  $${PATH_OPENCTM}/liblzma/Alloc.c \
                $${PATH_OPENCTM}/liblzma/LzFind.c \
                $${PATH_OPENCTM}/liblzma/LzmaDec.c \
                $${PATH_OPENCTM}/liblzma/LzmaEnc.c \
                $${PATH_OPENCTM}/liblzma/LzmaLib.c

    # openctm
    HEADERS += $${PATH_OPENCTM}/openctmpp.h \
               $${PATH_OPENCTM}/openctm.h \
               $${PATH_OPENCTM}/internal.h

    SOURCES += $${PATH_OPENCTM}/stream.c \
               $${PATH_OPENCTM}/openctm.c \
               $${PATH_OPENCTM}/compressRAW.c \
               $${PATH_OPENCTM}/compressMG2.c \
               $${PATH_OPENCTM}/compressMG1.c

# gdal/ogr and cgal
LIBS += -lgdal -lCGAL_Core -lCGAL -lmpfr -lgmp -lboost_thread
QMAKE_CXXFLAGS += -frounding-math -fno-strict-aliasing
