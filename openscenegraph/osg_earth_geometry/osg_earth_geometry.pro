#-------------------------------------------------
#
# Project created by QtCreator 2011-09-26T02:24:03
#
#-------------------------------------------------

CONFIG += qt link_pkgconfig
PKGCONFIG += openthreads openscenegraph
TARGET = osg_earth_geometry
TEMPLATE = app
SOURCES += main.cpp
#QT += core

moreFiles.path = $$OUT_PWD
moreFiles.files += earth_map.jpg
INSTALLS += moreFiles
