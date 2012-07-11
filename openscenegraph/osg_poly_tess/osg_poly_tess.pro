CONFIG += link_pkgconfig debug
PKGCONFIG += openthreads openscenegraph
TARGET = osg_poly_tess
SOURCES += main.cpp
TEMPLATE = app
LIBS += -losmscout
