TEMPLATE = app
CONFIG -= qt

SOURCES += \
   main.cpp
   
ifiles.path = $${OUT_PWD}
ifiles.files += *.glsl
INSTALLS += ifiles

#openscenegraph
OSGDIR = /home/preet/Dev/env/sys/osg-3.1.8
OSGLIBDIR = /home/preet/Dev/env/sys/osg-3.1.8/lib64
INCLUDEPATH += $${OSGDIR}/include
LIBS += -L$${OSGLIBDIR} -losgViewerrd
LIBS += -L$${OSGLIBDIR} -losgTextrd
LIBS += -L$${OSGLIBDIR} -losgGArd
LIBS += -L$${OSGLIBDIR} -losgUtilrd
LIBS += -L$${OSGLIBDIR} -losgDBrd
LIBS += -L$${OSGLIBDIR} -losgrd
LIBS += -L$${OSGLIBDIR} -lOpenThreadsrd
