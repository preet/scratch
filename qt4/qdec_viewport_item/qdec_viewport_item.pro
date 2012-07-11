##########################
# START PROJECT CONFIG

# qt
QT += declarative opengl

#openscenegraph
OSGDIR = /home/preet/Documents/openscenegraph
OSGLIBDIR = /home/preet/Documents/openscenegraph/lib64
INCLUDEPATH += $${OSGDIR}/include

LIBS += -L$${OSGLIBDIR}/osgdb_freetype.so
LIBS += -L$${OSGLIBDIR}/osgdb_jpeg.so
LIBS += -L$${OSGLIBDIR}/osgdb_png.so

LIBS += -L$${OSGLIBDIR} -losgViewer
LIBS += -L$${OSGLIBDIR} -losgText
LIBS += -L$${OSGLIBDIR} -losgGA
LIBS += -L$${OSGLIBDIR} -losgUtil
LIBS += -L$${OSGLIBDIR} -losgDB
LIBS += -L$${OSGLIBDIR} -losg
LIBS += -L$${OSGLIBDIR} -lOpenThreads

SOURCES += \
    main.cpp \
    qdectoucharea.cpp \
    qdecviewportitem.cpp \
    qdecviewportosg.cpp


HEADERS += \
    qdectoucharea.h \
    qdecviewportitem.h \
    qdecviewportosg.h

OTHER_FILES += \
    ui/main.qml \
    bar_descriptor.xml

# bar descriptor, icon, splashscreen
pkg_files.path = $${OUT_PWD}
pkg_files.files += \
    bar_descriptor.xml \
    icon.png \
    splashscreen.png

# define other resources and install path
res_local.path = $${OUT_PWD}
res_local.files += *.ttf \
    ui \
    shaders \
    models \
    textures

INSTALLS += pkg_files res_local

# END PROJECT CONFIG
##########################

