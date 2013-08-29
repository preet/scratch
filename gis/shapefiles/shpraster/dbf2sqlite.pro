QT       += core
#QT       -= gui

TARGET = dbf2sqlite
CONFIG   += console
CONFIG   -= app_bundle
TEMPLATE = app

# avoid linking in dl since we dont use it
DEFINES += SQLITE_OMIT_LOAD_EXTENSION

# statically include kompex
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

# statically include shapelib
HEADERS += \
    shapelib/shapefil.h

SOURCES += \
    shapelib/shpopen.c \
    shapelib/shptree.c \
    shapelib/dbfopen.c \
    shapelib/safileio.c

# main
SOURCES += dbf2sqlite.cpp
