TEMPLATE = lib
CONFIG += staticlib

HEADERS += \
    sqlite3ext.h \
    sqlite3.h \
    KompexSQLiteStreamRedirection.h \
    KompexSQLiteStatement.h \
    KompexSQLitePrerequisites.h \
    KompexSQLiteException.h \
    KompexSQLiteDatabase.h \
    KompexSQLiteBlob.h

SOURCES += \
    sqlite3.c \
    shell.c \
    KompexSQLiteStatement.cpp \
    KompexSQLiteDatabase.cpp \
    KompexSQLiteBlob.cpp
