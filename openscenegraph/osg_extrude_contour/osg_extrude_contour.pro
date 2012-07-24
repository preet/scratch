TEMPLATE = app
SOURCES += main.cpp

        #openscenegraph
        OSGDIR = /home/preet/Documents/sadf
        OSGLIBDIR = /home/preet/Documents/osdfpp11/lib64
            INCLUDEPATH += $${OSGDIR}/include
            LIBS += -L$${OSGLIBDIR} -losgViewer
            LIBS += -L$${OSGLIBDIR} -losgText
            LIBS += -L$${OSGLIBDIR} -losgGA
            LIBS += -L$${OSGLIBDIR} -losgUtil
            LIBS += -L$${OSGLIBDIR} -losgDB
            LIBS += -L$${OSGLIBDIR} -losg
            LIBS += -L$${OSGLIBDIR} -lOpenThreads


