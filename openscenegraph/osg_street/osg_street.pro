CONFIG += qt link_pkgconfig
PKGCONFIG += openthreads openscenegraph
SOURCES += main.cpp
TEMPLATE = app
           
# define other resources and install path
res_local.path = $${OUT_PWD}
res_local.files += \
   shaders \
   models

INSTALLS += res_local

#libosmscout
LIBOSMSCOUT_PATH = /home/preet/Dev/env/sys/libosmscout
INCLUDEPATH += $${LIBOSMSCOUT_PATH}/include
LIBS += -L/home/preet/Dev/env/sys/libosmscout/lib -losmscout
LIBS += -L$${LIBOSMSCOUT_PATH}/lib -losmscout

#
QMAKE_CXXFLAGS = -std=c++11
