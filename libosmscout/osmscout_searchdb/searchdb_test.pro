PATH_OSMSCOUT = /home/preet/Dev/env/sys/libosmscout

TEMPLATE = app
QT += core
CONFIG += console debug
SOURCES += searchdb_test.cpp

#boost
DEFINES += USE_BOOST
INCLUDEPATH += /home/preet/Dev/env/sys/boost-1.53

# avoid linking in dl since we dont use it
# is this really needed?
DEFINES += SQLITE_OMIT_LOAD_EXTENSION

# kompex
PATH_KOMPEX = /home/preet/Dev/env/sys/kompex
INCLUDEPATH += $${PATH_KOMPEX}/include
HEADERS += \
    $${PATH_KOMPEX}/include/sqlite3.h \
    $${PATH_KOMPEX}/include/KompexSQLiteStreamRedirection.h \
    $${PATH_KOMPEX}/include/KompexSQLiteStatement.h \
    $${PATH_KOMPEX}/include/KompexSQLitePrerequisites.h \
    $${PATH_KOMPEX}/include/KompexSQLiteException.h \
    $${PATH_KOMPEX}/include/KompexSQLiteDatabase.h \
    $${PATH_KOMPEX}/include/KompexSQLiteBlob.h

LIBS += -L$${PATH_KOMPEX}/lib -lkompex

#libosmscout
INCLUDEPATH += $${PATH_OSMSCOUT}/include
LIBS += -L$${PATH_OSMSCOUT}/lib -losmscout
LIBS += -L$${PATH_OSMSCOUT}/lib -losmscoutimport
