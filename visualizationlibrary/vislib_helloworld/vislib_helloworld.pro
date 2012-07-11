CONFIG += qt
QT += core gui opengl
TEMPLATE = app

HEADERS += \
    App_RotatingCube.hpp \
    Qt4Widget.hpp

SOURCES += \
    Qt4Widget.cpp \
    Qt4_example.cpp

#visualization library
VLDIR = /home/preet/Documents/visualizationlibrary
VLLIBDIR = /home/preet/Documents/visualizationlibrary/lib
    INCLUDEPATH += $${VLDIR}/include
    LIBS += -L$${VLLIBDIR} -lVLCore
    LIBS += -L$${VLLIBDIR} -lVLGraphics
#    LIBS += -L$${VLLIBDIR} -lVLCore-d

##liblzma
#HEADERS +=  openctm/liblzma/Alloc.h \
#            openctm/liblzma/LzFind.h \
#            openctm/liblzma/LzHash.h \
#            openctm/liblzma/LzmaEnc.h \
#            openctm/liblzma/LzmaLib.h \
#            openctm/liblzma/NameMangle.h \
#            openctm/liblzma/Types.h

#SOURCES +=  openctm/liblzma/Alloc.c \
#            openctm/liblzma/LzFind.c \
#            openctm/liblzma/LzmaDec.c \
#            openctm/liblzma/LzmaEnc.c \
#            openctm/liblzma/LzmaLib.c

## openctm
#HEADERS += openctm/openctmpp.h \
#           openctm/openctm.h \
#           openctm/internal.h

#SOURCES += openctm/stream.c \
#           openctm/openctm.c \
#           openctm/compressRAW.c \
#           openctm/compressMG2.c \
#           openctm/compressMG1.c
           
## define other resources and install path
#res_local.path = $${OUT_PWD}
#res_local.files += \
#   shaders \
#   models

#INSTALLS += res_local
