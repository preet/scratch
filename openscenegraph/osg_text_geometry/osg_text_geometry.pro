TEMPLATE = app
SOURCES += main.cpp

        #openscenegraph
        OSGDIR = /home/preet/Documents/openscenegraph
        OSGLIBDIR = /home/preet/Documents/openscenegraph/lib64
            INCLUDEPATH += $${OSGDIR}/include
            LIBS += -L$${OSGLIBDIR} -losgViewer
            LIBS += -L$${OSGLIBDIR} -losgText
            LIBS += -L$${OSGLIBDIR} -losgGA
            LIBS += -L$${OSGLIBDIR} -losgUtil
            LIBS += -L$${OSGLIBDIR} -losgDB
            LIBS += -L$${OSGLIBDIR} -losg
            LIBS += -L$${OSGLIBDIR} -lOpenThreads

ifonts.path = $$OUT_PWD
ifonts.files += *.ttf

ishaders.path = $$OUT_PWD
ishaders.files += shaders/*

INSTALLS += ifonts ishaders
