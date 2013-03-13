CONFIG   += console link_pkgconfig
CONFIG   -= qt
TEMPLATE = app

# main
SOURCES += shptk_meshgen.cpp

# clipper
PATH_CLIPPER = /home/preet/Dev/projects/osmsrender/osmsrender/thirdparty/clipper
INCLUDEPATH += $${PATH_CLIPPER}
HEADERS += $${PATH_CLIPPER}/clipper.hpp
SOURCES += $${PATH_CLIPPER}/clipper.cpp

# openscenegraph (for debug only)
PKGCONFIG += openthreads openscenegraph

# gdal/ogr and cgal
LIBS += -lgdal -lCGAL_Core -lCGAL -lmpfr -lgmp -lboost_thread
QMAKE_CXXFLAGS += -frounding-math -fno-strict-aliasing
