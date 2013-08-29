###libosmscout build-qmake
*last tested libosmscout rev: 3588c8e944afe0d99074e5c98059f588326c4bee, Jul 21 2013 *
Notes for building libosmscout with qmake. Libosmscout uses libtools to configure the build process based on the current environment, ie:

`[user@box libosmscout]: ./autogen.sh && ./configure`

Calling configure creates two system dependent files in the build tree:

 * include/private/Config.h
 * include/CoreFeatures.h

These files need to be generated before libosmscout can be built with qmake. Some handy options to pass to configure are:

**--disable-see2-support**  disables SSE2 instructions (yes the typo is intentional)
**--disable-cpp0x-support** disables C++11 support including thread unordered_set and unordered_map

If there are issues running libtools in a specific environment, generate the files with ./configure on the development machine and open and configure each file manually to reflect the target environment.

***
###Build using qmake
Once Config.h and CoreFeatures.h have been generated, libosmscout requires a minor fix to build with qmake. The libosmscout source tree has two sets of identically named files which is incompatible with qmake's build process. One set of files need to be renamed and the changes need to be reflected in the source.

* rename **include/util/Parser.h** to **include/util/ParserUtil.h**
* rename **include/util/Parser.cpp** to **include/util/ParserUtil.cpp**
* modify **include/TypeConfig.h** to reflect the file name changes
* modify **include/util/ParserUtil.cpp** to reflect the file name changes

Once these changes have been made, we need to set up qmake: 

* Create a qmake *.pro project file. The following instructions assume the pro file was created in the libosmscout directory.
* TEMPLATE = lib
* CONFIG -= qt
* CONFIG += plugin  (add this line to avoid soname versioning)
* TARGET = osmscout
* INCLUDEPATH += include
* INCLUDEPATH += src
* Add all the header and source files as required. This can be automated with the unix find util. ie, to list all header files to copy paste into the project file, you can use:

  `find include -name \*.h -printf %p' \\'\\n`

* Add install targets for the header and library files. See the included project file for an example.

After completing the *.pro file, libosmscout can be built by generating a makefile with qmake and then calling make:

* `qmake libosmscout.pro`
* `make`
* `make install`

***
###Building against Boost instead of C++11
Typically, unordered_map and unordered_set can only be used with libosmscout if C++11 is available on the target. Unordered_map and unordered_set may provide better performance compared to the fallback map and set in certain situations. For targets that don't have C++11, we can use Boost as a substitute which has header only drop-in replacements for unordered_map and unordered_set. To build against boost do the following before building:

1. Replace **HashMap.h** and **HashSet.h** in the  libosmscout source tree with the ones provided in this repo:
  * include/util/HashSet.h
  * include/util/HashMap.h
<br><br>
2. Pass the **--disable-cpp0x-support** flag to ./configure when configuring the build.

3. In the *.pro file, include the path to boost (1.5+) and add the define "USE_BOOST":
 * INCLUDEPATH +=/path/to/boost-1.5x
 * DEFINES += USE_BOOST

