# Example usage:
# Place Android.mk in directory with corresponding source files.
# Run  
# ndk-build NDK_PROJECT_PATH=. \
# APP_BUILD_SCRIPT=Android.mk \
# TARGET_ARCH_ABI=armeabi-v7a \
# TARGET_PLATFORM=android-9 \
# V=1 NDK_LOG=1 -B

# NDK_PROJECT_PATH and APP_BUILD_SCRIPT must be specified
# if building without an Application.mk.

LOCAL_PATH:= $(call my-dir)

LOCAL_CFLAGS += -W -Wall
LOCAL_CFLAGS += -fPIC -DPIC
LOCAL_CFLAGS += "-DDARWIN_NO_CARBON"
LOCAL_CFLAGS += "-DFT2_BUILD_LIBRARY"
LOCAL_CFLAGS += -O2
	
# copy headers freetype/include to freetype2/freetype/include
	
LOCAL_SRC_FILES := \
	src/base/ftbbox.c \
	src/base/ftbitmap.c \
	src/base/ftglyph.c \
	src/base/ftstroke.c \
	src/base/ftxf86.c \
	src/base/ftbase.c \
	src/base/ftsystem.c \
	src/base/ftinit.c \
	src/base/ftgasp.c \
	src/raster/raster.c \
	src/sfnt/sfnt.c \
	src/smooth/smooth.c \
	src/autofit/autofit.c \
	src/truetype/truetype.c \
	src/cff/cff.c \
	src/psnames/psnames.c \
	src/pshinter/pshinter.c

LOCAL_C_INCLUDES := \
	$(LOCAL_PATH)/builds \
	$(LOCAL_PATH)/include

LOCAL_MODULE := freetype
LOCAL_MODULE_FILENAME := libfreetype

include $(BUILD_SHARED_LIBRARY)

## Its not possible to build a static library 
## with the ndk without an Application.mk file
## unless we create another fake shared lib that
## depends on the static lib.

## If this lib is built as static, the following
## code will output libfake.so in ./libs and the
## static libxxx.a in ./obj/local/.

#include $(CLEAR_VARS)
#LOCAL_MODULE := fake
#LOCAL_STATIC_LIBRARIES := libxxx
#include $(BUILD_SHARED_LIBRARY) 
