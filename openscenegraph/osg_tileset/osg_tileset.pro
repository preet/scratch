TEMPLATE = app

# need these flags for gcc 4.8.x bug
QMAKE_LFLAGS += -Wl,--no-as-needed
LIBS += -lpthread
QMAKE_CXXFLAGS += -std=c++11

#openscenegraph
PATH_OPENSCENEGRAPH = /home/preet/Dev/env/sys/osg-3.2.1
PATH_OPENSCENEGRAPH_LIB = /home/preet/Dev/env/sys/osg-3.2.1/lib64
DEFINES += DEV_GL_DESKTOP

INCLUDEPATH += $${PATH_OPENSCENEGRAPH}/include
LIBS += -L$${PATH_OPENSCENEGRAPH_LIB}/osgdb_freetyperd.so
LIBS += -L$${PATH_OPENSCENEGRAPH_LIB}/osgdb_jpegrd.so
LIBS += -L$${PATH_OPENSCENEGRAPH_LIB}/osgdb_pngrd.so
LIBS += -L$${PATH_OPENSCENEGRAPH_LIB} -losgViewer
LIBS += -L$${PATH_OPENSCENEGRAPH_LIB} -losgText
LIBS += -L$${PATH_OPENSCENEGRAPH_LIB} -losgGA
LIBS += -L$${PATH_OPENSCENEGRAPH_LIB} -losgUtil
LIBS += -L$${PATH_OPENSCENEGRAPH_LIB} -losgDB
LIBS += -L$${PATH_OPENSCENEGRAPH_LIB} -losg
LIBS += -L$${PATH_OPENSCENEGRAPH_LIB} -lOpenThreads

HEADERS += \
        ViewController.hpp \
        MiscUtils.h \
        GeometryUtils.h \
        OSGUtils.h \
        ThreadPool.h \
        TileLL.h \
        TileDataSourceLL.h \
        TileImageSourceLL.h \
        TileVisibilityLL.h \
        TileVisibilityLLPixelsPerMeter.h \
        TileSetLL.h \
        DataSetTilesLL.h
	
SOURCES += \
        GeometryUtils.cpp \
        MiscUtils.cpp \
        OSGUtils.cpp \
        ThreadPool.cpp \
        TileLL.cpp \
        TileDataSourceLL.cpp \
        TileImageSourceLL.cpp \
        TileVisibilityLLPixelsPerMeter.cpp \
        TileSetLL.cpp \
        DataSetTilesLL.cpp

SOURCES += main.cpp
#SOURCES += test_surftilevxorder.cpp
#SOURCES += test_tri_clipping.cpp
#SOURCES += debug.cpp
#SOURCES += test_proj_clip_speed.cpp
#SOURCES += test_tileclosestpoint.cpp

