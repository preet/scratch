# Example usage:
# Place Android.mk in directory with corresponding source files.
# Run  
# ndk-build NDK_PROJECT_PATH=. \
# APP_BUILD_SCRIPT=Android.mk \
# TARGET_ARCH_ABI=armeabi-v7a \
# TARGET_PLATFORM=android-9 \
# V=1 NDK_LOG=1 -B

LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)

# LOCAL_COPY_* commands don't seem to work
# so you have to copy them manually
LOCAL_COPY_HEADERS_TO := include
LOCAL_COPY_HEADERS := \
	jconfig.h \
	jpeglib.h \
	jmorecfg.h \
	jerror.h \
	jpegint.h

LOCAL_SRC_FILES := \
	jerror.c \
	jmemmgr.c \
	jcapimin.c \
	jcparam.c \
	jdapimin.c \
	jdhuff.c \
	jfdctflt.c \
	jdcoefct.c \
	jidctint.c \
	jdcolor.c \
	jchuff.c \
	jcphuff.c \
	jidctfst.c \
	jutils.c \
	jidctflt.c \
	jcmainct.c \
	jcdctmgr.c \
	jddctmgr.c \
	jdatadst.c \
	jdinput.c \
	jdtrans.c \
	jdmaster.c \
	jdsample.c \
	jcmarker.c \
	jcomapi.c \
	jdmerge.c \
	jcinit.c \
	jdapistd.c \
	jdatasrc.c \
	jdphuff.c \
	jfdctfst.c \
	jccoefct.c \
	jctrans.c \
	jccolor.c \
	jcprepct.c \
	jidctred.c \
	jquant2.c \
	transupp.c \
	jdmainct.c \
	jcmaster.c \
	jquant1.c \
	jmemansi.c \
	jcsample.c \
	jdmarker.c \
	jcapistd.c \
	jfdctint.c \
	jdpostct.c

LOCAL_C_INCLUDES := $(LOCAL_PATH)
LOCAL_MODULE := jpeg
LOCAL_MODULE_FILENAME := libjpeg

include $(BUILD_SHARED_LIBRARY)
