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

#LOCAL_CFLAGS := 
#LOCAL_CFLAGS := -std=gnu89 -fvisibility=hidden -ftrapv ## -fomit-frame-pointer

# LOCAL_COPY_* commands don't seem to work
LOCAL_COPY_HEADERS_TO := libpng14
LOCAL_COPY_HEADERS := \
	png.h \
	pngconf.h
	
LOCAL_SRC_FILES := \
	pngrtran.c \
	png.c \
	pngwrite.c \
	pngmem.c \
	pngget.c \
	pngrio.c \
	pngset.c \
	pngrutil.c \
	pngtrans.c \
	pngwio.c \
	pngread.c \
	pngtest.c \
	pngerror.c \
	pngwutil.c \
	pngpread.c \
	pngwtran.c

LOCAL_C_INCLUDES := $(LOCAL_PATH)
LOCAL_LDLIBS := -lz
LOCAL_MODULE := png
LOCAL_MODULE_FILENAME := libpng

include $(BUILD_SHARED_LIBRARY)

## Its not possible to build a static library 
## with the ndk without an Application.mk file
## unless we create another fake shared lib that
## depends on the static lib.

## If libpng is built as static, the following
## code will output libfake.so in ./libs and the
## static libpng.a in ./obj/local/.

#include $(CLEAR_VARS)
#LOCAL_MODULE := fake
#LOCAL_STATIC_LIBRARIES := libpng
#include $(BUILD_SHARED_LIBRARY) 
