QT += quick

HEADERS += fboinsgrenderer.h
SOURCES += fboinsgrenderer.cpp main.cpp

HEADERS += logorenderer.h
SOURCES += logorenderer.cpp

RESOURCES += textureinsgnode.qrc

OTHER_FILES += \
    main.qml

qnx {
    QMAKE_LFLAGS += '-Wl,-rpath,\'./app/native/lib\''

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
        -e /home/preet/snrub.png res/snrub.png \
        -e $$[QT_INSTALL_QML]/ qml/

    package.target = $${TARGET}.bar
    package.depends = $$TARGET
    package.commands = blackberry-nativepackager \
        -package $${TARGET}.bar \
        -devMode -debugToken /home/preet/.rim/bbndk/debugtoken.bar \
        $${PACKAGE_ARGS}

    QMAKE_EXTRA_TARGETS += package

    OTHER_FILES += bar-descriptor.xml
}
