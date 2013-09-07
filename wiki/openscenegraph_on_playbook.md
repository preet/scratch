This page describes one possible OpenSceneGraph (version 3.1.8 as of this writing) build config for the Playbook (Tablet OS 2.0.1)

***
* Get the source (from openscenegraph.com, github.com/openscenegraph, etc.) and extract/clone it to a directory

>     $ /home/user/osg-src

* I remove most plugins because they tend to require a lot of small fixes to compile with the Playbook. 
* My osg-src/src/osgPlugins/CMakeLists.txt looks like this:

         #---------------------------------------------------
         # OSG CMAKE SUPPORT
         # (C) by Michael Wagner, mtw@shared-reality.com 2005
         # (C) Eric Wing, Luigi Calori and Robert Osfield 2006-2007
         #---------------------------------------------------

         PROJECT(OSG_PLUGINS_MASTER)

         IF(NOT DYNAMIC_OPENSCENEGRAPH)
             ADD_DEFINITIONS(-DOSG_LIBRARY_STATIC)
         ENDIF()

         IF(NOT MSVC)
             SET(LIBRARY_OUTPUT_PATH "${LIBRARY_OUTPUT_PATH}/${OSG_PLUGINS}")
             SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${OSG_PLUGINS}")
         ENDIF()

         SET(CMAKE_SHARED_MODULE_PREFIX ${OSG_PLUGIN_PREFIX})

         IF(MSVC80 OR MSVC90)
             IF(NOT OSG_MSVC_GENERATE_PLUGINS_AND_WRAPPERS_MANIFESTS)
                 SET(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} /MANIFEST:NO")
             ENDIF()
         ENDIF()

         SET(TARGET_DEFAULT_PREFIX "osgdb_")
         SET(TARGET_DEFAULT_LABEL_PREFIX "Plugins")
         SET(TARGET_COMMON_LIBRARIES
             OpenThreads
             osg
             osgDB
             osgUtil
         )

         ############################################################
         #
         #  NodeKit/Psudo loader plugins
         #
         ADD_SUBDIRECTORY(osga)
         ADD_SUBDIRECTORY(rot)
         ADD_SUBDIRECTORY(scale)
         ADD_SUBDIRECTORY(trans)
         ADD_SUBDIRECTORY(normals)
         ADD_SUBDIRECTORY(revisions)
         ADD_SUBDIRECTORY(view)
         ADD_SUBDIRECTORY(shadow)
         ADD_SUBDIRECTORY(terrain)

         ############################################################
         #
         #  Main native plugins
         #

         ADD_SUBDIRECTORY(osg)
         ADD_SUBDIRECTORY(ive)

         ############################################################
         #
         #  Shader plugins
         #

         ADD_SUBDIRECTORY(glsl)

         ############################################################
         #
         #  Image plugins
         #

         IF(JPEG_FOUND)
             ADD_SUBDIRECTORY(jpeg)
         ENDIF()

         IF(PNG_FOUND AND OSG_CPP_EXCEPTIONS_AVAILABLE)
             ADD_SUBDIRECTORY(png)
         ENDIF()


         ############################################################
         #
         #  3rd party 3d plugins
         #

         IF(ZLIB_FOUND)
             ADD_SUBDIRECTORY(gz)
         ENDIF()

         ADD_SUBDIRECTORY(tgz)

         IF(FREETYPE_FOUND)
             ADD_SUBDIRECTORY(freetype)
         ENDIF()

         IF(ZLIB_FOUND)
             ADD_SUBDIRECTORY(zip)
         ENDIF()



         ##########to get all the variables of Cmake
         #GET_CMAKE_PROPERTY(MYVARS VARIABLES)
         #FOREACH(myvar ${MYVARS})
         #    FILE(APPEND ${CMAKE_CURRENT_BINARY_DIR}/AllVariables.txt
         #        "${myvar} -->${${myvar}}<-\n"
         #    )
         #ENDFOREACH(myvar)

* Open CMakeLists.txt in osg-src/OpenThreads and make the following modifications 
* (at line 12):

         IF((NOT ANDROID) AND (NOT QNX))
             INCLUDE(CheckAtomicOps)
         ENDIF()

* (line 67):

         IF((NOT ANDROID) AND (NOT QNX))
         # Use our modified version of FindThreads.cmake which has Sproc hacks.
             FIND_PACKAGE(Threads)
         ENDIF()

* (line 91, add):

         ELSEIF(QNX)
             SUBDIRS(pthreads)    

* Open up osg-src/CMakeLists.txt

* (line 74) ... this is just for added convenience when deploying since sonames and lib versioning doesn't really work when creating packages/apps

         SET(OPENSCENEGRAPH_SONAMES FALSE)
         SET(OPENTHREADS_SONAMES FALSE)

* (after line 512) ... we do this to cut down on the number packages CMake looks for:

         ELSEIF(QNX)
         # Common to all platforms except android:
             FIND_PACKAGE(FreeType)
             FIND_PACKAGE(ZLIB)

***

* Now we need to make changes to the source.
* Open osg-src/include/osg/Math. Replace the cmath and cfloat includes:

         #if defined(__QNXNTO__)   
            #include <math.h>
            #include <float.h>
         #else
            #include <cmath>
            #include <cfloat>
         #endif

* Scroll down to where the isNaN function is defined and add

         #elif defined(__QNXNTO__)
             inline bool isNaN(float v) { return isnan(v); }
             inline bool isNaN(double v) { return isnan(v); }

* Open up osg-src/src/osgUtil/tristripper/include/public_types.h
* Add the standard c style headers for stdlib:

         #if defined(__QNXNTO__)
            #include <stdlib.h>
         #else
            #include <cstdlib>
         #endif

* Open up osg-src/include/osg/Array
* Comment out lines 474 and 511 (virtual void apply(const GLdouble&)), ie sometimes GLdouble is redefined as GLfloat when OpenGLES2 is used and this will generate an error when compiling applications if it isn't commented out here.

* Open up osg-src/include/osg/State
* edit the applyModeOnTexUnit function:

                    #ifndef OSG_GLES2_AVAILABLE
                    // with opengl es 2, GL_TEXTURE_2D is always enabled
                    // and setting it explicitly causes an error
                        if (enabled) glEnable(mode);
                        else glDisable(mode);
                    #endif

* Open up osg-src/include/osg/State
* Edit the State::initializeExtensionProcs() function where max texture units are queried:

        #ifdef OSG_GLES2_AVAILABLE 
           _glMaxTextureCoords=1;
        #else
           glGetIntegerv(GL_MAX_TEXTURE_COORDS,&_glMaxTextureCoords);
        #endif

***

* Add the BlackBerry NDK environment to your shell

>     $ source /home/user/bbndk/bbndk-env.sh

* Create a build directory

>     $ mkdir /home/user/osg-build

* Run CMake from the build directory

>     $ cmake-gui ../osg-src

* Press Configure and choose "Specify toolchain file"
* Use this as a toolchain file (rename it 'something'.cmake). Note that this a hack pretty much and not a good toolchain file, it just overloads the required options for a specific OpenSceneGraph build. You have to replace the relevant paths with your install location of the blackberry ndk.

         SET(CMAKE_SYSTEM_NAME QNX)
         SET(CMAKE_SYSTEM_VERSION 1)
         SET(CMAKE_SYSTEM_PROCESSOR ntoarm)
         SET(QNX 1)

         # Setup compilers. We use CACHE STRING FORCE on the flags to
         # ensure they are used by CMake when compiling initial tests
         SET(CMAKE_C_COMPILER /home/preet/Dev/env/sys/bbndk-2.1.0/host/linux/x86/usr/bin/qcc)
         SET(CMAKE_C_FLAGS "-Vgcc_ntoarmv7le -fstack-protector -fstack-protector-all -Wno-psabi -Wall -W -D_REENTRANT" CACHE STRING "Playbook QCC C Flags" FORCE)

         SET(CMAKE_CXX_COMPILER /home/preet/Dev/env/sys/bbndk-2.1.0/host/linux/x86/usr/bin/qcc)
         SET(CMAKE_CXX_FLAGS "-Vgcc_ntoarmv7le -lang-c++ -fstack-protector -fstack-protector-all -Wno-psabi -Wall -W -D_REENTRANT" CACHE STRING "Playbook QCC C++ Flags" FORCE)

         SET(CMAKE_FIND_ROOT_PATH /home/preet/Dev/env/sys/bbndk-2.1.0/target/qnx6/armle-v7)
         SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
         SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
         SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

         SET(CMAKE_AR /home/preet/Dev/env/sys/bbndk-2.1.0/host/linux/x86/usr/bin/ntoarmv7-ar CACHE FILEPATH "" FORCE)
         SET(CMAKE_LINKER /home/preet/Dev/env/sys/bbndk-2.1.0/host/linux/x86/usr/bin/ntoarmv7-ld CACHE FILEPATH "" FORCE)
         SET(CMAKE_RANLIB /home/preet/Dev/env/sys/bbndk-2.1.0/host/linux/x86/usr/bin/ntoarmv7-ranlib CACHE FILIEPATH "" FORCE)
         SET(CMAKE_NM /home/preet/Dev/env/sys/bbndk-2.1.0/host/linux/x86/usr/bin/ntoarmv7-nm CACHE FILEPATH "" FORCE)
         SET(CMAKE_OBJCOPY /home/preet/Dev/env/sys/bbndk-2.1.0/host/linux/x86/usr/bin/ntoarmv7-objcopy CACHE FILEPATH "" FORCE)
         SET(CMAKE_OBJDUMP /home/preet/Dev/env/sys/bbndk-2.1.0/host/linux/x86/usr/bin/ntoarmv7-objdump CACHE FILEPATH "" FORCE)
         SET(CMAKE_STRIP /home/preet/Dev/env/sys/bbndk-2.1.0/host/linux/x86/usr/bin/ntoarmv7-strip CACHE FILEPATH "")
         SET(CMAKE_MAKE_PROGRAM /home/preet/Dev/env/sys/bbndk-2.1.0/host/linux/x86/usr/bin/make CACHE FILEPATH "" FORCE)

         SET(CMAKE_SHARED_LINKER_FLAGS "-Wl,-rpath-link,/home/preet/Dev/env/sys/bbndk-2.1.0/target/qnx6/armle-v7/lib -Wl,-rpath-link,/home/preet/Dev/env/sys/bbndk-2.1.0/target/qnx6/armle-v7/usr/lib -Wl,-O1" CACHE STRING "")
         SET(CMAKE_EXE_LINKER_FLAGS "-Wl,-rpath-link,/home/preet/Dev/env/sys/bbndk-2.1.0/target/qnx6/armle-v7/lib -Wl,-rpath-link,/home/preet/Dev/env/sys/bbndk-2.1.0/target/qnx6/armle-v7/usr/lib -Wl,-O1" CACHE STRING "") 
         SET(CMAKE_INSTALL_PREFIX "/home/preet/Dev/env/qnx/osg-arm-rel" CACHE STRING "")

         # for osg

         SET(BUILD_OSG_APPLICATIONS OFF CACHE BOOL "" FORCE)
         SET(OSG_WINDOWING_SYSTEM "None" CACHE STRING "" FORCE)
         SET(OSG_USE_QT OFF CACHE BOOL "" FORCE)

         #opengl
         SET(OPENGL_INCLUDE_DIR /home/preet/Dev/env/sys/bbndk-2.1.0/target/qnx6/usr/include/GLES2 CACHE FILEPATH "" FORCE)
         SET(OPENGL_egl_LIBRARY /home/preet/Dev/env/sys/bbndk-2.1.0/target/qnx6/armle-v7/usr/lib/libEGL.so CACHE FILEPATH "" FORCE)
         SET(OPENGL_gl_LIBRARY /home/preet/Dev/env/sys/bbndk-2.1.0/target/qnx6/armle-v7/usr/lib/libGLESv2.so CACHE FILEPATH "" FORCE)

         SET(OSG_GL1_AVAILABLE OFF CACHE BOOL "" FORCE)
         SET(OSG_GL2_AVAILABLE OFF CACHE BOOL "" FORCE)
         SET(OSG_GL3_AVAILABLE OFF CACHE BOOL "" FORCE)
         SET(OSG_GLES1_AVAILABLE OFF CACHE BOOL "" FORCE)
         SET(OSG_GLES2_AVAILABLE ON CACHE BOOL "" FORCE)
         SET(OSG_GL_LIBRARY_STATIC OFF CACHE BOOL "" FORCE)
         SET(OSG_GL_DISPLAYLISTS_AVAILABLE OFF CACHE BOOL "" FORCE)
         SET(OSG_GL_MATRICES_AVAILABLE OFF CACHE BOOL "" FORCE)
         SET(OSG_GL_VERTEX_FUNCS_AVAILABLE OFF CACHE BOOL "" FORCE)
         SET(OSG_GL_VERTEX_ARRAY_FUNCS_AVAILABLE OFF CACHE BOOL "" FORCE)
         SET(OSG_GL_FIXED_FUNCTION_AVAILABLE OFF CACHE BOOL "" FORCE)

         # freetype
         SET(FREETYPE_INCLUDE_DIR_freetype2 /home/preet/Dev/env/sys/bbndk-2.1.0/target/qnx6/usr/include/freetype2 CACHE FILEPATH "")
         SET(FREETYPE_INCLUDE_DIR_ft2build /home/preet/Dev/env/sys/bbndk-2.1.0/target/qnx6/usr/include CACHE FILEPATH "")
         SET(FREETYPE_LIBRARY /home/preet/Dev/env/sys/bbndk-2.1.0/target/qnx6/armle-v7/usr/lib/libfreetype.so CACHE FILEPATH "")

         # jpg
         SET(JPEG_INCLUDE_DIR /home/preet/Dev/env/sys/bbndk-2.1.0/target/qnx6/usr/include CACHE FILEPATH "")
         SET(JPEG_LIBRARY /home/preet/Dev/env/sys/bbndk-2.1.0/target/qnx6/armle-v7/lib/libjpeg.so CACHE FILEPATH "")

         # zlib
         SET(ZLIB_INCLUDE_DIR /home/preet/Dev/env/sys/bbndk-2.1.0/target/qnx6/usr/include CACHE FILEPATH "")
         SET(ZLIB_LIBRARY /home/preet/Dev/env/sys/bbndk-2.1.0/target/qnx6/armle-v7/usr/lib/libz.so CACHE FILEPATH "")

         # png
         SET(PNG_LIBRARY /home/preet/Dev/env/sys/bbndk-2.1.0/target/qnx6/armle-v7/usr/lib/libpng.so CACHE FILEPATH "")
         SET(PNG_PNG_INCLUDE_DIR /home/preet/Dev/env/sys/bbndk-2.1.0/target/qnx6/usr/include CACHE FILEPATH "")

* Press Finish
* Press Configure twice
* Press Generate
* Check the Advanced checkbox (next to the Search text input)
* Close the CMake-gui dialog
* In the build directory, run (make sure the blackberry ndk environment is set)
>     $ make
>     $ make install
