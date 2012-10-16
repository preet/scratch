QT       += core
#QT       -= gui

CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app

# statically include shapelib
HEADERS += \
    shapelib/shapefil.h

SOURCES += \
    shapelib/shpopen.c \
    shapelib/shptree.c \
    shapelib/dbfopen.c \
    shapelib/safileio.c

# link against pugixml
LIBS += -lpugixml

# main
SOURCES += shpadmin2osm.cpp
