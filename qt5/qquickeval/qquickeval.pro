QT += core quick

HEADERS += \
   helper.h

SOURCES += \
   helper.cpp \
   main.cpp

#
HEADERS += helper_testqtsigslotspeeds.h
SOURCES += helper_testqtsigslotspeeds.cpp

## test qtimer
#HEADERS += helper_testqtimerblock.h
#SOURCES += helper_testqtimerblock.cpp

# test ks
HEADERS += helper_testkssigslotspeeds.h
SOURCES += helper_testkssigslotspeeds.cpp


PATH_KS = /home/preet/Dev/projects/ks/ks
INCLUDEPATH += /home/preet/Dev/projects/ks

HEADERS += \
    $${PATH_KS}/KsGlobal.h \
    $${PATH_KS}/KsLog.h \
    $${PATH_KS}/KsEvent.h \
    $${PATH_KS}/KsThread.h \
    $${PATH_KS}/KsObject.h \
    $${PATH_KS}/KsSignal.h

SOURCES += \
    $${PATH_KS}/KsLog.cpp \
    $${PATH_KS}/KsThread.cpp \
    $${PATH_KS}/KsObject.cpp

include($${PATH_KS}/thirdparty/asio/asio.pri)


RESOURCES += \
    res.qrc

ANDROID_PACKAGE_SOURCE_DIR = $$PWD/android

OTHER_FILES += \
    android/AndroidManifest.xml

## need these flags for gcc 4.8.x bug
!android: {
    # NOTE: Android's C lib contains its own pthread implementation
    # so we dont explicitly link against pthreads
    QMAKE_LFLAGS += -Wl,--no-as-needed
    LIBS += -lpthread
}
QMAKE_CXXFLAGS += -std=c++11
