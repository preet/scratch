##########################
# START CONFIG FOR QT
# directory where qt and bb libs are
# (depends on device or simulator build config)
QT += opengl declarative
QTDIR = temp

device   {
    QTDIR = /home/preet/Dev/env/qnx/qt4-arm
}
simulator {

}

# define qt lib files and install path
dep_libs.path = $${OUT_PWD}/deploy/lib
dep_libs.files += $${QTDIR}/lib/*.so.4
dep_libs.files += $${QTDIR}/lib/*.so.1

# define bb lib file and install path
dep_plugins.path = $${OUT_PWD}/deploy/plugins/platforms
dep_plugins.files += $${QTDIR}/plugins/platforms/libblackberry.so

INSTALLS += dep_libs dep_plugins

# END CONFIG FOR QT
##########################

##########################
# START PROJECT CONFIG

SOURCES += \
    main.cpp

OTHER_FILES += \
    ui/main.qml

# bar descriptor, icon, splashscreen
pkg_files.path = $${OUT_PWD}
pkg_files.files += \
    bar_descriptor.xml \
    icon.png \
    splashscreen.png

# define other resources and install path
res_local.path = $${OUT_PWD}/deploy
res_local.files += \
    ui

INSTALLS += pkg_files res_local

# END PROJECT CONFIG
##########################
