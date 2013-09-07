(on 2.3.3+ apparently?)

get the android sdk tools

make sure that API levels 10 and 11 are explicitly selected and installed

get the android ndk

         # can clean a repo directory after configuring with
         # git clean -dfx (this will permanently delete all files
         # created during the config/build process!)

         # always clean the directory after a bad config/build

         #dont know if this is necessary
         export JAVA_HOME=/usr/lib/jvm/java-7-openjdk

         # configure qtbase
         ./configure \
         -opensource \
         -confirm-license \
         -prefix /home/preet/Dev/env/android/qt-5.1-git-debug \
         -debug \
         -xplatform android-g++ \
         -nomake tests \
         -nomake examples \
         -android-ndk /home/preet/Dev/env/sys/android-ndk-r8e \
         -android-sdk /home/preet/Dev/env/sys/android-sdk-linux \
         -android-ndk-host linux-x86_64 \
         -android-toolchain-version <e.g. 4.7>


i built the following modules (export path to newly built qt ^, then qmake && make && install individual modules)

qtimageformats
qtjsbackend
qtdeclarative (this is QtQuick 2)
qtsensors
qtmultimedia


then start up qtcreator

tools > options

android, specify NDK, SDK, Ant and OpenJDK paths -- you get a little warning that Qt versions for architectures are missing

now go to Build & Run and add the qt version we just built by finding the corresponding qmake in the Qt Versions tab. note that you shouldn't have to specify a compiler because the previous step with the Android sdk/ndk paths should have created kits with all the right paths already
