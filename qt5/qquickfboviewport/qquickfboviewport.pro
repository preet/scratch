QT += quick


HEADERS += \
    qquickfboviewportosg.h

SOURCES += \
    qquickfboviewportosg.cpp \
    main.cpp

OTHER_FILES += \
    main.qml \
    bar-descriptor.xml

qmlfiles.path = $${OUT_PWD}
qmlfiles.files = main.qml

INSTALLS += qmlfiles

!qnx {
    DEFINES += DEV_DESKTOP
#    OSGDIR = /home/preet/Dev/env/sys/osg-3.1.8
#    OSGLIBDIR = /home/preet/Dev/env/sys/osg-3.1.8/lib64

    OSGDIR = /home/preet/Dev/env/sys/osg-modern
    OSGLIBDIR = /home/preet/Dev/env/sys/osg-modern/lib64

    INCLUDEPATH += $${OSGDIR}/include
    LIBS += -L$${OSGLIBDIR} -losgViewerrd
    LIBS += -L$${OSGLIBDIR} -losgTextrd
    LIBS += -L$${OSGLIBDIR} -losgGArd
    LIBS += -L$${OSGLIBDIR} -losgUtilrd
    LIBS += -L$${OSGLIBDIR} -losgDBrd
    LIBS += -L$${OSGLIBDIR} -losgrd
    LIBS += -L$${OSGLIBDIR} -lOpenThreadsrd
}

qnx {
    DEFINES += DEV_PLAYBOOK

    #
    # note:
    # to resolve osg/qt conflict;
    # might cause some weirdness if passing
    # GLdoubles to OpenGL (so dont do it!)

#    DEFINES += "GL_DOUBLE=0x140A"
#    DEFINES += "GLdouble=double"
#    DEFINES += DEV_GL_ES2

    OSGDIR = /home/preet/Dev/env/qnx/osg-arm-rel
    OSGLIBDIR = /home/preet/Dev/env/qnx/osg-arm-rel/lib

    INCLUDEPATH += $${OSGDIR}/include
    LIBS += -L$${OSGLIBDIR} -losgViewer
    LIBS += -L$${OSGLIBDIR} -losgText
    LIBS += -L$${OSGLIBDIR} -losgGA
    LIBS += -L$${OSGLIBDIR} -losgUtil
    LIBS += -L$${OSGLIBDIR} -losgDB
    LIBS += -L$${OSGLIBDIR} -losg
    LIBS += -L$${OSGLIBDIR} -lOpenThreads

    QMAKE_LFLAGS += '-Wl,-rpath,\'./app/native/lib\''

    OSGDIR = /home/preet/Dev/env/qnx/osg-arm-rel/lib

    PACKAGE_ARGS = \
        $${PWD}/bar-descriptor.xml $$TARGET \
        -e $$[QT_INSTALL_LIBS]/libQt5Core.so.5 lib/libQt5Core.so.5 \
        -e $$[QT_INSTALL_LIBS]/libQt5Gui.so.5 lib/libQt5Gui.so.5 \
        -e $$[QT_INSTALL_LIBS]/libQt5OpenGL.so.5 lib/libQt5OpenGL.so.5 \
        -e $$[QT_INSTALL_LIBS]/libQt5Network.so.5 lib/libQt5Network.so.5 \
        -e $$[QT_INSTALL_LIBS]/libQt5Widgets.so.5 lib/libQt5Widgets.so.5 \
        -e $$[QT_INSTALL_LIBS]/libQt5Quick.so.5 lib/libQt5Quick.so.5 \
        -e $$[QT_INSTALL_LIBS]/libQt5Qml.so.5 lib/libQt5Qml.so.5 \
        -e $$[QT_INSTALL_LIBS]/libQt5Sql.so.5 lib/libQt5Sql.so.5 \
        -e $$[QT_INSTALL_LIBS]/libQt5V8.so.5 lib/libQt5V8.so.5 \
        -e $$[QT_INSTALL_LIBS]/libQt5XmlPatterns.so.5 lib/libQt5XmlPatterns.so.5 \
        -e $$[QT_INSTALL_PLUGINS]/platforms/libqqnx.so plugins/platforms/libqqnx.so \
        -e $$[QT_INSTALL_QML]/ qml/ \
        -e $${OSGDIR}/libosgViewer.so lib/libosgViewer.so \
        -e $${OSGDIR}/libosgText.so lib/libosgText.so \
        -e $${OSGDIR}/libosgGA.so lib/libosgGA.so \
        -e $${OSGDIR}/libosgUtil.so lib/libosgUtil.so \
        -e $${OSGDIR}/libosgDB.so lib/libosgDB.so \
        -e $${OSGDIR}/libosg.so lib/libosg.so \
        -e $${OSGDIR}/libOpenThreads.so lib/libOpenThreads.so \
        -e $${PWD}/main.qml res/main.qml

    package.target = $${TARGET}.bar
    package.depends = $$TARGET
    package.commands = blackberry-nativepackager \
        -package $${TARGET}.bar \
        -devMode -debugToken /home/preet/.rim/bbndk/debugtoken.bar \
        $${PACKAGE_ARGS}

    QMAKE_EXTRA_TARGETS += package

    OTHER_FILES += bar-descriptor.xml
}
