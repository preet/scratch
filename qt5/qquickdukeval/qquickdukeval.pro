QT += core quick

#duktape
INCLUDEPATH += ../../dukeval
HEADERS += ../../dukeval/duktape.h
SOURCES += ../../dukeval/duktape.c

HEADERS += \
   helper.h

SOURCES += \
   helper.cpp \
   main.cpp

RESOURCES += \
    res.qrc

ANDROID_PACKAGE_SOURCE_DIR = $$PWD/android

OTHER_FILES += \
    android/AndroidManifest.xml
