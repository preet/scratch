QT       += core

TARGET = wkt2bin
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app

# we need gdal's ogr tools
LIBS += -lgdal

# main
SOURCES += wkt2bin.cpp
