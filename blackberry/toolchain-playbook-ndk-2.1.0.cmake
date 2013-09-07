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



