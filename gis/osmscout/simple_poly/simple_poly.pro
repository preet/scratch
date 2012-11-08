TEMPLATE = app
CONFIG += console debug
CONFIG -= qt
LIBS += -losmscout
SOURCES += main.cpp

# poly2tri
#HEADERS +=  poly2tri/poly2tri.h \
#            poly2tri/common/shapes.h \
#            poly2tri/common/utils.h \
#            poly2tri/sweep/advancing_front.h \
#            poly2tri/sweep/cdt.h \
#            poly2tri/sweep/sweep.h \
#            poly2tri/sweep/sweep_context.h

#SOURCES +=  poly2tri/common/shapes.cc \
#            poly2tri/sweep/advancing_front.cc \
#            poly2tri/sweep/cdt.cc \
#            poly2tri/sweep/sweep.cc \
#            poly2tri/sweep/sweep_context.cc

QMAKE_CXXFLAGS += -std=c++0x

