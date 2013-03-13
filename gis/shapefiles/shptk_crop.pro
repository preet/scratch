CONFIG   += console
CONFIG   -= qt
TEMPLATE = app

# main
SOURCES += shptk_crop.cpp

# clipper
PATH_CLIPPER = /home/preet/Dev/projects/osmsrender/osmsrender/thirdparty/clipper
INCLUDEPATH += $${PATH_CLIPPER}
HEADERS += $${PATH_CLIPPER}/clipper.hpp
SOURCES += $${PATH_CLIPPER}/clipper.cpp

# ogr
LIBS += -lgdal -lCGAL_Core -lCGAL -lmpfr -lgmp -lboost_thread


