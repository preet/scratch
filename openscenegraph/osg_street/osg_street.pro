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

#
#QMAKE_CXXFLAGS = -std=c++0x
