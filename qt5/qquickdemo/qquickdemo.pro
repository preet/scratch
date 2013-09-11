QT += core quick

HEADERS += \
    helper.h

SOURCES += \
    helper.cpp \
    main.cpp

OTHER_FILES += \
    main.qml \
    android/AndroidManifest.xml \
    android/version.xml \
    android/res/values-id/strings.xml \
    android/res/values-pl/strings.xml \
    android/res/values-ms/strings.xml \
    android/res/values-rs/strings.xml \
    android/res/values-nl/strings.xml \
    android/res/values-ru/strings.xml \
    android/res/values-fr/strings.xml \
    android/res/values-zh-rCN/strings.xml \
    android/res/values-pt-rBR/strings.xml \
    android/res/values-zh-rTW/strings.xml \
    android/res/values-es/strings.xml \
    android/res/layout/splash.xml \
    android/res/values-de/strings.xml \
    android/res/values-it/strings.xml \
    android/res/values-fa/strings.xml \
    android/res/values-nb/strings.xml \
    android/res/values-el/strings.xml \
    android/res/values-ro/strings.xml \
    android/res/values-et/strings.xml \
    android/res/values-ja/strings.xml \
    android/res/values/libs.xml \
    android/res/values/strings.xml \
    android/src/org/kde/necessitas/ministro/IMinistroCallback.aidl \
    android/src/org/kde/necessitas/ministro/IMinistro.aidl \
    android/src/org/qtproject/qt5/android/bindings/QtApplication.java \
    android/src/org/qtproject/qt5/android/bindings/QtActivity.java

INCLUDEPATH += /home/preet/Dev/scratch/android/libhelloworld

android {
    DEFINES += ENV_ANDROID
    LIBS += -lQt5PlatformSupport
    #LIBS += -L/home/preet/Dev/env/android/qt-5-git-debug/plugins/platforms/android -lqtforandroid
#    LIBS += -L/home/preet/Dev/build/android/libhelloworld/ -lhelloworld
}
else {
    DEFINES += ENV_DEV
    LIBS += -L/home/preet/Dev/build/qt5/libhelloworld/ -lhelloworld
}

RESOURCES += \
    res.qrc
