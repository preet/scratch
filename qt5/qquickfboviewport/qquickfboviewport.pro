QT += quick

HEADERS += \
    qquickfboviewportosg.h

SOURCES += \
    qquickfboviewportosg.cpp \
    main.cpp

OTHER_FILES += \
    main.qml \
    bar-descriptor.xml \
    android/AndroidManifest.xml \
    android/version.xml \
    android/res/values-id/strings.xml \
    android/res/values-pl/strings.xml \
    android/res/values-ms/strings.xml \
    android/res/values-rs/strings.xml \
    android/res/values-nl/strings.xml \
    android/res/values-ru/strings.xml \
    android/res/values-fr/strings.xml \
    android/res/values-zh-rCN/strings.xml \
    android/res/values-pt-rBR/strings.xml \
    android/res/values-zh-rTW/strings.xml \
    android/res/values-es/strings.xml \
    android/res/layout/splash.xml \
    android/res/values-de/strings.xml \
    android/res/values-it/strings.xml \
    android/res/values-fa/strings.xml \
    android/res/values-nb/strings.xml \
    android/res/values-el/strings.xml \
    android/res/values-ro/strings.xml \
    android/res/values-et/strings.xml \
    android/res/values-ja/strings.xml \
    android/res/values/libs.xml \
    android/res/values/strings.xml \
    android/src/org/kde/necessitas/ministro/IMinistroCallback.aidl \
    android/src/org/kde/necessitas/ministro/IMinistro.aidl \
    android/src/org/qtproject/qt5/android/bindings/QtApplication.java \
    android/src/org/qtproject/qt5/android/bindings/QtActivity.java

qmlfiles.path = $${OUT_PWD}
qmlfiles.files = main.qml

resfiles.path = $${OUT_PWD}
resfiles.files = res/*

#INSTALLS += qmlfiles

env_dev   {
    DEFINES += ENV_DEV
    DEFINES += ENV_GL   # desktop gl
    OSGDIR = /home/preet/Dev/env/sys/osg-3.1.8
    OSGLIBDIR = /home/preet/Dev/env/sys/osg-3.1.8/lib64

#    OSGDIR = /home/preet/Dev/env/sys/osg-modern
#    OSGLIBDIR = /home/preet/Dev/env/sys/osg-modern/lib64

    INCLUDEPATH += $${OSGDIR}/include
    #LIBS += -L$${OSGLIBDIR}/osgdb_freetyperd.so
    #LIBS += -L$${OSGLIBDIR}/osgdb_jpegrd.so
    #LIBS += -L$${OSGLIBDIR}/osgdb_pngrd.so
    LIBS += -L$${OSGLIBDIR} -losgViewerrd
    LIBS += -L$${OSGLIBDIR} -losgTextrd
    LIBS += -L$${OSGLIBDIR} -losgGArd
    LIBS += -L$${OSGLIBDIR} -losgUtilrd
    LIBS += -L$${OSGLIBDIR} -losgDBrd
    LIBS += -L$${OSGLIBDIR} -losgrd
    LIBS += -L$${OSGLIBDIR} -lOpenThreadsrd
}

env_android   {
    DEFINES += ENV_ANDROID
    DEFINES += ENV_GLES2

    OSGDIR = /home/preet/Dev/env/android/osg-git-debug
    OSGLIBDIR = /home/preet/Dev/env/android/osg-git-debug/lib

    INCLUDEPATH += $${OSGDIR}/include
    LIBS += -L$${OSGLIBDIR}/osgPlugins-3.3.0 -losgdb_freetype
    LIBS += -L$${OSGLIBDIR}/osgPlugins-3.3.0 -losgdb_jpeg
    LIBS += -L$${OSGLIBDIR}/osgPlugins-3.3.0 -losgdb_png
    LIBS += -L$${OSGLIBDIR} -losgViewer
    LIBS += -L$${OSGLIBDIR} -losgText
    LIBS += -L$${OSGLIBDIR} -losgGA
    LIBS += -L$${OSGLIBDIR} -losgUtil
    LIBS += -L$${OSGLIBDIR} -losgDB
    LIBS += -L$${OSGLIBDIR} -losg
    LIBS += -L$${OSGLIBDIR} -lOpenThreads
}

env_blackberry   {
    DEFINES += ENV_BLACKBERRY
    DEFINES += ENV_GLES2

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

RESOURCES += \
    ui.qrc
