##########################
# START CONFIG FOR QT

#CONFIG -= qt
QT += declarative opengl

device|simulator {

    # directory where qt and bb libs are
    # (depends on device or simulator build config)

    QTDIR = temp
    device   {
       QTDIR = /home/preet/Documents/qnx/qt-4.8-arm
    }
    simulator {
       QTDIR = /home/preet/Documents/qnx/qt-4.8-x86
    }

    # define qt lib files and install path
    dep_libs.path = $${OUT_PWD}/deploy/lib
    dep_libs.files += $${QTDIR}/lib/*.so.4

    # define bb lib file and install path
    dep_plugins.path = $${OUT_PWD}/deploy/plugins/platforms
    dep_plugins.files = $${QTDIR}/plugins/platforms/libblackberry.so

    INSTALLS += dep_libs dep_plugins
}

# END CONFIG FOR QT
##########################

##########################
# START PROJECT CONFIG

SOURCES += \
    main.cpp \
    qdectoucharea.cpp

HEADERS += \
    qdectoucharea.h

OTHER_FILES += \
    ui/main.qml

#LIBS += -lscreen -limg -lgestures

# bar descriptor, icon, splashscreen
pkg_files.path = $${OUT_PWD}
pkg_files.files += \
    bar_descriptor.xml

# define other resources and install path
res_local.path = $${OUT_PWD}/deploy
res_local.files += \
    ui

INSTALLS += pkg_files res_local

# END PROJECT CONFIG
##########################
