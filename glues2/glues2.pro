TEMPLATE = lib
CONFIG += debug
CONFIG -= qt
#DEFINES += _GLES2_

# QMAKE_CXXFLAGS += -std=c++0x

# glu-es base
HEADERS += \
#    glues_registry.h \
#    glues_quad.h \
#    glues_project.h \
#    glues_mipmap.h \
    glues_error.h \
    glues.h
#    glu.h

SOURCES += \
#    glues_registry.c \
#    glues_quad.c \
#    glues_project.c \
#    glues_mipmap.c \
    glues_error.c

# glu-es tessellator
HEADERS += \
    libtess/tessmono.h \
    libtess/tess.h \
    libtess/sweep.h \
    libtess/render.h \
    libtess/priorityq-sort.h \
    libtess/priorityq-heap.i \
    libtess/priorityq-heap.h \
    libtess/priorityq.h \
    libtess/normal.h \
    libtess/mesh.h \
    libtess/memalloc.h \
    libtess/geom.h \
    libtess/dict-list.h \
    libtess/dict.h

SOURCES += \
    libtess/tessmono.c \
    libtess/tess.c \
    libtess/sweep.c \
    libtess/render.c \
    libtess/priorityq.c \
    libtess/normal.c \
    libtess/mesh.c \
    libtess/memalloc.c \
    libtess/geom.c \
    libtess/dict.c
