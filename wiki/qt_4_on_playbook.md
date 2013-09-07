This document describes how to setup Qt 4.8.x and the BlackBerry NDK 2.x to deploy Qt applications to a PlayBook. Arch Linux (x64) was used, though you should be able to follow a similar process for any modern Linux distribution. Much of this information was taken from [the QNX page on qt-project.org](http://qt-project.org/wiki/QNX). I'm assuming that the normal QtSDK (Qt and QtCreator) are being used for development.

## Setup and Install

### Install Qt 4.8.x for Linux (ie, Qt for the dev machine)
1. Qt should be an available package for your distribution. If you don't already have it, install it.
2. Install Qt-Creator (should also be an available package).

### Download and install Java
The BlackBerry NDK requires a 32-bit JRE. For 64 bit Arch, there's a [bin32-jre AUR package](http://aur.archlinux.org/packages.php?ID=18646) available. Make sure to use the latest PKGBUILD (attached in the comments section of the preceding link). You'll also need the lib32-gtk2 package. For 64 bit Ubuntu, you only need the ia32-libs package ([See the BBNDK release notes](https://bdsc.webapps.blackberry.com/native/download/releasenotes/) for more info). If you're running 32-bit linux, you only need the JRE package for your distribution.

### Download and install BlackBerry Native Development Kit
1. Download from https://bdsc.webapps.blackberry.com/native/download/
2. Make the downloaded file executable:  
`$ chmod +x installer-bbndk-2.0.1-linux-...bin`
2. Execute the installer
3. Given that $$BBNDK is your installation directory, add the BlackBerry NDK's environment vars to your own environment (ie. in ~/.bashrc):  
`$ source $BBNDK/bbndk-env.sh`

### Generate/Register Signing Keys
1. Request signing keys on BlackBerry [website](https://www.blackberry.com/SignedKeys/) [blackberry.com]
2. Once you receive them, save the keys in the ~/.rim folder.
3. Register the keys by running the following in the same folder:  
`$ blackberry-signer -register -storepass <newpass> -csjpin <csjpin> client-RDK-<XXXXX>.csj
client-PBDT-<XXXX>.csj`
    * "newpass" is a new password
    * "csjpin" is the PIN code you entered on the key application website

Registering the keys should generate three files: author.p12, barsigner.csk, barsigner.db. **BACK THESE FILES UP**. You can copy/paste these three files to develop on multiple machines.

### Create and Install Debug Token
1. Get your PlayBook's Device PIN, the unique hex code that can be found under Settings -> About -> Hardware (Dropdown Menu). Make sure the device is in Developer Mode.
2. Create the token:  
`$ blackberry-debugtokenrequest -storepass <pass> -devicepin <devicepin> debugtoken.bar`
    * "pass" is the signing key password (entered above)
    * "devicepin" is what we noted down earlier
3. Install the debug token to the device:  
`$ blackberry-nativepackage -installDebugToken <debugtoken> -device <device_ip> [-password <device_pass>]`
    * "debugtoken" path to debugtoken.rar
    * "device_ip" PlayBook's IP address
    * "device_pass" is the device's password
    * note that debug tokens expire after 1 month

### Download and Install VMWare Player
1. VMWare Player must be installed from the vmware website. Locate and download the relevant Linux 64-bit installer.
2. Make the downloaded file executable:  
`$ chmod +x VMWare-edition-version.release.arch...`
3. Run the install **as root**
4. Note: These instruction are for Arch Linux only. Set "System service scripts directory" to:  
`/etc/rc.d`  
5. Keep going forward with the installer. Ignore the error regarding "rc*.d style init script directories".
6. Create links for the following daemons **as root**:  
`$ ln -s /etc/init.d/vmware /etc/rc.d/`  
`$ ln -s /etc/init.d/vmware-workstation-server /etc/rc.d/`  
7. Apply/install [vmware-patch 3.4-1 from AUR](https://aur.archlinux.org/packages.php?K=vmware-patch&SeB=x)
8. Add vmware to the list of startup daemons in /etc/rc.conf.

* note: Full vmware on Arch Linux instructions are on the [Arch Wiki](https://wiki.archlinux.org/index.php/VMware). 

### Download and Install Playbook Simulator
1. Download the BlackBerry Tablet OS Simulator (same link as the NDK)
2. Make the downloaded file executable:  
`$ chmod +x BlackBerryPlayBookSimulator-Installer-2.0.1-358-Linux...bin`
3. Run the installer  
    * note that the simulator is just a bunch of VMWare images

### Download, Compile and Install Qt 4.8.x
**Warning!** The version of Qt I pulled from git had some buggy behaviour with respect to using Qt on the Playbook! The libs included with Qt 4.8.2 seem to be working alright. The Qt 4.8.2 stable sources are available from the Qt website ... you may want to just use that.

1. Get the latest stable Qt source code (or heed my advice and grab the more stable 4.8.2 sources):  
`$ git clone git://gitorious.org/qt/qt.git`
2. Copy this source folder into two new folders (or just create one and rename the original), one for each build (x86 and arm). I named the folders "qt-x86" and "qt-arm", but it doesn't matter. You can also choose whatever install directories you want in steps 3 and 4 (replace your own install path for the -prefix-install argument, just don't use any paths your system might use to install its own Qt packages)
3. Build Qt for ARM:  
`$ cd qt-arm`  
`$ ./configure -opensource -confirm-license -qpa -iconv -shared -release -xplatform unsupported/blackberry-armv7le-qcc -little-endian -arch arm -largefile -nomake examples -xmlpatterns -no-webkit -no-neon -no-rpath -opengl es2 -prefix-install -prefix $$INSTALLPATH/qt-4.8-bb-arm`  
`$ make`  
`$ sudo make install`
4. Build Qt for x86 (for the Simulator):  
`$ cd qt-x86`  
`$ ./configure -opensource -confirm-license -qpa -iconv -shared -release -xplatform unsupported/blackberry-x86-qcc -little-endian -arch i386 -largefile -nomake examples -xmlpatterns -no-webkit -no-rpath -opengl es2 -prefix-install -prefix $$INSTALLPATH/qt-4.8-bb-x86`  
`$ make`  
`$ sudo make install`

## Project Configuration
Currently Qt libs are packaged as part of the application before they are deployed to the device (Tablet OS 2.x). In the future, Qt will be included as part of the BlackBerry OS. For now however, we need to include both Qt libs and the 'libblackberry.so' plugin with every application. These files can be found inside the versions of Qt for ARM and x86 we compiled and installed earlier. 

Note that we have two targets -- the actual device, and the simulator. To be clear, if we want to package an application for the device, we use the libs from the version of Qt we built for ARM. If we want to package an application for the simulator, we use the libs from the version of Qt we built for x86. As this is kind of hectic to keep track of manually, we'll try to setup a *.pro file that will copy over the required libs to the build directory based on the target. 

### QtCreator Setup
First, we need to setup both targets with QtCreator so they can be compiled. These instructions were followed using QtCreator 2.5.1.

1. Start QtCreator
2. Open up the Options Dialog from Tools -> Options
3. Under the Build & Run Section, click on the Qt Versions tab
4. You should already have one version of Qt installed (System)
5. Add qmake for the PlayBook device:  
    * $$INSTALLPATH/qt-4.8-bb-arm/bin/qmake  
6. You'll get a warning that no compatible toolchain for the new qmake was found. In the Tool Chains tab:  
    * Add a GCC Tool Chain
    * Set its compiler path to $$BBNDK/host/linux/x86/user/bin/ntoarmv7-g++
    * Set the ABI to look like: "custom" "arm"-"linux"-"generic"-"elf"-"32bit"
    * Leave the debugger blank
    * Leave the mkspec as default
7. Back in the Qt Versions tab, add qmake for the PlayBook simulator:  
    * $$INSTALLPATH/qt-4.8-bb-x86/bin/qmake  
8. There shouldn't be a need to set up the tool chain for the simulator's qmake. Hit Apply/OK.

### QtCreator Project Setup
We need to make a few changes to default projects created by QtCreator to be able to conveniently build for both the simulator and device. We need to specify both the qmake we're going to use and the libs we want to copy over. In this section, we'll set up a small Hello World project in QML and deploy it to both the simulator and the device. There are a lot of initial steps but they are quick to implement after going through them all once.

1. Start up QtCreator from the command line (this is just so we have the BBNDK environment variables available -- there's probably a way to define this in QtCreator directly)
2. Create a new project; for this example select "Non-Qt Project -> Plain C++ Project"
3. Name the project "qml_helloworld" and save it wherever you want
4. Use/select the version control method you want (or disable it) and hit Next
5. Under Target Setup, you should see two targets: "Desktop" and "Embedded Linux". Disable Desktop and enable Embedded Linux
6. You'll see a bunch of build configurations under Embedded Linux, one debug and release config for each version of Qt you have installed on your system. Enable only the release config for the versions that we compiled earlier (qt-4.8-bb-arm, and qt-4.8-bb-x86). Note that we didn't build in debug symbols when we compiled -- I think that debug and release both result in the same binaries.
7. (optionally) Change the build paths to something a bit more sane (I used 'build_dev_qml_helloworld' and 'build_sim_qml_helloworld'). Remember that the build directories need to be at the same folder level as the source directory.
8. Hit OK to create the project
9. Select Projects in the left menu bar, and ensure you're in the "Targets" tab
10. Choose "Qt 4.8.x qt-4.8-bb-arm" from the Edit Build Configuration drop down
11. Rename the build configuration to something descriptive like "Playbook Device"
12. Under General, ensure that the Qt version is correct (it should be qt-4.8-bb-arm) and set the Tool Chain to be the specific GCC toolchain we created in the previous section (on my machine, it showed up as just "GCC", not "GCC x86 32/64 bit")
13. Under "Build Steps", click the Details button for the qmake build step and add "CONFIG+=device" for the Additional arguments option
14. Under "Build Steps" click the Details button for the Make build step and add "install" for the Make arguments option
15. Choose "Qt 4.8.x qt-4.8-bb-x86" from the Edit Build Configuration drop down 
16. Rename the build configuration to something like "Playbook Simulator"
17. The Tool chain should be set to "GCC (x86 32bit)". Ignore the "Warning: The tool chain suggests using another mkspec" message under "Build Steps"
18. Under "Build Steps", click the Details button for the qmake build step and add "CONFIG+=simulator" for the Additional arguments option
19. Under "Build Steps" click the Details button for the Make build step and add "install" for the Make arguments option

### Project Files
Our simple demo project will consist of five files: main.cpp, main.qml, bar_descriptor.xml, icon.png and splashscreen.png. I placed main.cpp and bar_descriptor.xml in the root project directory. I created subdirectories for the qml file (deploy/qml) and images (deploy/images). You're free to organize the project however you deem fit as long as you make sure the project structure is correctly reflected in both the *.pro file and the BlackBerry bar descriptor. PlayBook icons are 86x86 pixels and splash screens are 1024x600 (device resolution). Here's the source for all relevant files (find your own images for icon.png and splashscreen.png):

**main.cpp**

    #include <QApplication>
    #include <QtDeclarative/QtDeclarative>
    #include <QtDeclarative/QDeclarativeView>
    
    int main(int argc, char *argv[])
    {
       QApplication app(argc,argv);
       QDeclarativeView mainView;
       mainView.setSource(QString("app/native/qml/main.qml"));
       mainView.show();
       return app.exec();
    }

**main.qml**

    import QtQuick 1.1
    
    Rectangle
    {
        id: layer_base;
        width: 1024;
        height: 600;
        color: "#333333";   // dark gray
    
        Text
        {
            anchors.horizontalCenter: parent.horizontalCenter;
            anchors.verticalCenter: parent.verticalCenter;
            horizontalAlignment: Text.AlignHCenter;
            text: "Hello World!";
            color:"#72E84A";    // lime green
            font.pointSize: 24;
            font.bold: true;
        }
    }

**qml_helloworld.pro**

    TEMPLATE = app
    TARGET = qml_helloworld
    DEPENDPATH += .
    INCLUDEPATH += .
    
    # we're using QtQuick (QML) so we need to
    # explicitly include the declarative module
    QT += declarative
    
    # source files
    SOURCES += main.cpp
    
    # header files
    
    # other files
    OTHER_FILES += \
       deploy/qml/main.qml
    
    # directory where qt and bb libs are
    # (depends on device or simulator build config)
    QTDIR = temp
    
    device   {
       QTDIR = /your/path/to/qt-4.8-arm
    }
    simulator {
       QTDIR = /your/path/to/qt-4.8-x86
    }
    
    # define qt lib files and install path
    res_lib.path = $${OUT_PWD}/deploy/lib
    res_lib.files = $${QTDIR}/lib/*.so.4
    
    # define bb lib file and install path
    res_plugins.path = $${OUT_PWD}/deploy/plugins/platforms
    res_plugins.files = $${QTDIR}/plugins/platforms/libblackberry.so
    
    # define bar descriptor file and install path
    res_bar_desc.path = $${OUT_PWD}
    res_bar_desc.files = bar_descriptor.xml
    
    # define other resources and install path
    res_local.path = $${OUT_PWD}/deploy
    res_local.files += \
       deploy/images \
       deploy/qml
    
    INSTALLS += res_lib res_plugins res_bar_desc res_local

**bar_descriptor.xml**

    <?xml version="1.0" encoding="utf-8" standalone="no"?>

    <qnx>
      <id>org.demo.qml_helloworld</id>
      <name>QML HelloWorld</name>
      <versionNumber>1.0.0</versionNumber>
      <description>A simple QML Hello World demo application</description>
    
      <initialWindow>
      <systemChrome>none</systemChrome>
          <transparent>false</transparent>
          <autoOrients>true</autoOrients>
          <aspectRatio>landscape</aspectRatio>
       </initialWindow>
    
       <env var="QT_PLUGIN_PATH" value="app/native/plugins"/>
       <env var="LD_LIBRARY_PATH" value="app/native/lib"/>
    
       <arg>-platform</arg>
       <arg>blackberry</arg>
       <author>canurabus</author>
       <action system="true">run_native</action>
       <category>core.games</category>
    
       <asset entry="true" path="qml_helloworld" type="Qnx/Elf">qml_helloworld</asset>
       <asset path="deploy/images/splashscreen.png">splashscreen.png</asset>
       <asset path="deploy/images/icon.png">icon.png</asset>
       <asset path="deploy/lib">lib</asset>
       <asset path="deploy/plugins">plugins</asset>
       <asset path="deploy/qml">qml</asset>
       <icon><image>icon.png</image></icon>
       <splashscreen>splashscreen.png</splashscreen>
    </qnx>

After creating/adding the above files to the project, you should be able to build the project with both the "Playbook Simulator" and "Playbook Device" configurations. The active config for building can be selected from the left menu bar, above the Run button. You can build the project by pressing the Build button in the bottom left corner of the QtCreator window. After building both configurations, you should have two output directories. Following the above config, they would show up as:  
  * build_dev_qml_helloworld
  * build_sim_qml_helloworld

### Creating the app bar file package
Use blackberry-nativepackager to create the bar file for the simulator or device:  
`$ blackberry-nativepackager -devMode -debugToken ~/.rim/debugtoken.bar -package qml_helloworld.bar bar_descriptor.xml`

### Deploying to the Simulator  
1. Start VMWare Player
2. Open up the Playbook virtual machine (its in the path where Playbook Simulator was installed)
3. The simulator is already in developer mode. Click the developer icon in the top right to get the device's IP address
4. Send the bar file to the device using blackberry-deploy. Note that the device password for the simulator can be anything.   
`$ blackberry-deploy -installApp -device <device ip address> -password <device password> qml_helloworld.bar`
5. The simulator dashboard should now have your app installed.
6. If all went well, starting the app should show a simple Hello World message.

### Deploying to a PlayBook
1. Make sure the device is on and in developer mode
2. Click the developer icon in the top right to get the device's IP address
4. Send the bar file to the device using blackberry-deploy.  
`$ blackberry-deploy -installApp -device <device ip address> -password <device password> qml_helloworld.bar`
5. The dashboard should now show your app installed.
6. If all went well, starting the app should show a simple Hello World message.

### Debugging
It's possible the app didn't work. If something is wrong, the app might seem to start up (ie, show the loading screen) and then immediately crash back to the dashboard. We can check the application's log to see what went wrong:

`$ blackberry-deploy -getfile logs/log log.txt -device <device ip> -password <device pass> qml_helloworld.bar`

Some common problems are missing/wrong libs (ie you deployed simulator libs to the device or vice-versa) and incorrect asset paths. Note that in addition to the log, you can also get the *.core file [executablename.core] in the logs directory for post mortem debugging.

### Application file structure  
It can be helpful to see how the application directory is laid out to help with debugging or planning out your application's file structure. There's more information on file structure available on [RIM's website](https://developer.blackberry.com/native/documentation/com.qnx.doc.native_sdk.devguide/com.qnx.doc.native_sdk.devguide/topic/accessible_folders.html). You can ssh to the device/simulator to browse your app's directory as well:

1. To ssh to the device, you need a 4096 rsa key
`$ ssh-keygen -b 4096 -t rsa -f /your/path/to/key`
2. Then use blackberry-connect (as the application itself notes, leave it running)
`$ blackberry-connect <device ip> -password <device password> -sshPublicKey /your/path/to/key.pub`
3. In another terminal window, ssh to the device/simulator as devuser (you need to specify the ssh key you created earlier)
`$ ssh -i id_rsa devuser@deviceip`
