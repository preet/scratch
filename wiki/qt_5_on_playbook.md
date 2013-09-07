This page describes one possible OpenSceneGraph (version 3.1.8 as of this writing) build config for Linux that builds OSG only against OpenGL 3 (no deprecated functionality).

***
* Get the source (from openscenegraph.com, github.com/openscenegraph, etc.)
* Create a build directory

>     $ /home/preet/Downloads/packages/BUILD/osg-src     // assume this is the source dir
>     $ mkdir /home/preet/Downloads/packages/BUILD/osg-build

* Run CMake from the build directory

>     $ cmake-gui ../osg-src

* Press Configure and use Unix Native 
* Check the Advanced checkbox (next to the Search text input)
* Set/adjust the following options

>     CMAKE_INSTALL_PREFIX : /your/install/path
>     CMAKE_BUILD_TYPE: RelWithDebInfo (or Debug, Release)
>     OPENGL_INCLUDE_DIR: add path to /GL3/gl3.h if its not in /usr/include
>     OSG_GL1_AVAILABLE: NO
>     OSG_GL2_AVAILABLE: NO
>     OSG_GL3_AVAILABLE: YES
>     OSG_GLES1_AVAILABLE: NO
>     OSG_GLES2_AVAILABLE: NO
>     OSG_GL_DISPLAYLISTS_AVAILABLE: NO
>     OSG_GL_FIXED_FUNCTION_AVAILABLE: NO
>     OSG_GL_LIBRARY_STATIC: NO
>     OSG_GL_MATRICES_AVAILABLE: NO
>     OSG_GL_VERTEX_ARRAY_FUNCS_AVAILABLE: NO
>     OSG_GL_VERTEX_FUNCS_AVAILABLE: NO

* Press Configure again
* Press Generate
* Close the CMake-gui dialog
* In the build, run
>     $ make && make install
