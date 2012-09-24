QT       += core
#QT       -= gui

TARGET = paint_shapefile
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

# main
SOURCES += main.cpp
