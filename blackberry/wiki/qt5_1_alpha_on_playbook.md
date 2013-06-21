This is a set of notes for building and deploying Qt 5.1 for the Playbook on a Linux machine. As of this writing, Qt 5.1 is in alpha. It is assumed that the BlackBerry Playbook OS NDK (currently at v2.1) is installed, and that the NDK environment is used while building:

>     $ source /where/you/installed/the/ndk/bbndk-env.sh

For building and deploying applications, QtCreator 2.7 is used.

More generic instructions are available on the Qt website:

[Building Qt5 for the Playbook](http://qt-project.org/wiki/Building-Qt5-for-the-Playbook)

[Building and Deploying Qt Apps to the Playbook](http://qt-project.org/wiki/Building_and_Deploying_Qt_Applications_to_the_Playbook)

***

**Building**
* Grab the [Qt 5.1 Source](http://download.qt-project.org/development_releases/qt/5.1/)
* Extract it somewhere and cd into it from a terminal with the bbndk environment
* **Do not do a top level configure**, instead build and make qtbase first

>     $ cd qtbase
>     $ source /where/you/installed/the/ndk/bbndk-env.sh
>     $ ./configure -prefix /where/you/want/to/install/qt5 -device blackberry-playbook-armv7le -opengl es2 -nomake docs -nomake examples -nomake demos -nomake tests -opensource -confirm-license -release -no-neon -no-c++11
>     $ make && make install

* Now you need to build some additional qt5 modules:
* _qtimageformats_
* _qtjsbackend_
* _qtxmlpatterns_
* _qtdeclarative (this is QtQuick 2)_
* _qtgraphicaleffects_
* _qtsensors [optional]_
* _qtmultimedia [optional]_
* _qtscript [optional for QtQuick 1]_
* _qtquick1 [optional for legacy QtQuick 1]_



* Export the path to the new qtbase install you just made, then cd into any modules you want and build and install them

>     $ export PATH=/where/you/installed/qt5/bin:$PATH
>     $ qmake && make && make install


***

**Adding Kit to QtCreator 2.7**
* Open up QtCreator
* Tools > Options
* Build and Run Section
* Qt Versions Tab
* Click Add... 
    * Locate the qmake you just built
    * Version name: something like "Qt 5.1 Alpha - Playbook Device"
    * BlackBerry Native SDK: "/where/you/installed/BBNDK"
* Kits Tab
* Click Add... 
    * Name: something like "Playbook Device (Qt 5.1 Alpha)"
    * Device type: BlackBerry Device
    * Device: BlackBerry Device
    * Compiler: GCC
    * Debugger: "/where/you/installed/BBNDK/host/linux/x86/usr/bin/ntoarm-gdb"
    * Qt version: What you created earlier in Qt Versions (Qt 5.1 Alpha - Playbook Device)
    * Qt mkspec: (empty)
* Click Apply, exit with OK

***

**QtCreator Project / Build Generation**
* See the following sample project file and bar file. **Note that there are slight differences compared to the documentation at Qt-project due to some name changes in Qt 5.1**.

* **pro file:**
* The nativepackager will error out if there are any path errors so make sure everything is correct.

        QT += quick
        
        HEADERS += main.cpp
        
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
                -e $$[QT_INSTALL_QML]/ qml/
                -e $$[QT_INSTALL_IMPORTS]/ imports/   # only if you built/use the qtquick1 modules
        
            package.target = $${TARGET}.bar
            package.depends = $$TARGET
            package.commands = blackberry-nativepackager \
                -package $${TARGET}.bar \
                -devMode -debugToken /path/to/debugtoken.bar \
                $${PACKAGE_ARGS}
        
            QMAKE_EXTRA_TARGETS += package
        
            OTHER_FILES += bar-descriptor.xml
        }

* **bar file:**
* Note the QML_FIXED_ANIMATION_STEP and QML_FORCE_THREADED_RENDER environment variables. You may see slow performance for OpenGL / QtQuick applications on the device. Play with these variables to hopefully increase performance. Setting both to ON and 1 respectively improved performance for me. 
* The above is probably related to a bug which may or may not be fixed for the Playbook [QTBUG-28487](https://bugreports.qt-project.org/browse/QTBUG-28487)


        <?xml version='1.0' encoding='UTF-8' standalone='no'?>
        <qnx xmlns="http://www.qnx.com/schemas/application/1.0">
            <id>com.example.textureinsgnode</id>
            <name>textureinsgnode</name>
            <versionNumber>1.0.0</versionNumber>
            <description>DESCRIPTION</description>
            <initialWindow>
                <aspectRatio>landscape</aspectRatio>
                <autoOrients>false</autoOrients>
                <systemChrome>none</systemChrome>
                <transparent>false</transparent>
            </initialWindow>
            <author>Individual</author>
            <copyright>COPYRIGHT</copyright>
            <env var="QML2_IMPORT_PATH" value="app/native/qml"/>
            <env var="QT_PLUGIN_PATH" value="app/native/plugins"/>
            <env var="LD_LIBRARY_PATH" value="app/native/lib"/>
            <env var="QML_FIXED_ANIMATION_STEP" value="no"/>
            <env var="QML_FORCE_THREADED_RENDER" value="1"/>
            <!-- <arg>-style</arg> -->
            <!-- <arg>qnxlight</arg> -->
            <action system="true">run_native</action>
            <!-- textureinsgnode should point to the project binary, path can be relative -->
            <asset entry="true" type="Qnx/Elf" path="textureinsgnode">textureinsgnode</asset>
            <asset path="%QT_INSTALL_LIBS%">lib</asset>
            <asset path="%QT_INSTALL_PLUGINS%">plugins</asset>
            <asset path="%QT_INSTALL_QML%">imports</asset>
            <!-- These values will be replaced when deploying in Qt Creator -->
            <authorId>gYAAgOTuT0B9ytlqvWI4hESaSfo</authorId>
        </qnx>

***

**Deploying**
* Build your project with QtCreator
* In the build directory, call

>    $ make targetname.bar

* Now you have a *.bar package you can deploy to the device.

***

**Blackberry NDK Commands**
* Install to device

>    $ blackberry-deploy -installApp -device 192.168.2.10 -password mypass myapp.bar

* Get stdout log from device

>    $ blackberry-deploy -getfile logs/log stdout.log -device 192.168.x.x -password mypass myapp.bar

* Get core dump from device

>    $ blackberry-deploy -getfile logs/myapp.core myapp.core -device 192.168.x.x -password mypass myapp.bar
