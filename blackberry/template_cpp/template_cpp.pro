##########################
# START CONFIG FOR QT

# directory where qt and bb libs are
# (depends on device or simulator build config)
#QTDIR = temp

#device    {
#   QTDIR = /home/preet/Documents/qnx/qt-4.8-arm
#}
#simulator {
#   QTDIR = /home/preet/Documents/qnx/qt-4.8-x86
#}

## define qt lib files and install path
#dep_libs.path = $${OUT_PWD}/deploy/lib
#dep_libs.files = $${QTDIR}/lib/*.so.4

## define bb lib file and install path
#dep_plugins.path = $${OUT_PWD}/deploy/plugins/platforms
#dep_plugins.files = $${QTDIR}/plugins/platforms/libblackberry.so

#INSTALLS += dep_libs dep_plugins

# END CONFIG FOR QT
##########################

##########################
# START PROJECT CONFIG
CONFIG -= qt
#INCLUDEPATH += /home/preet/Downloads/Packages/boost_1_49_0
INCLUDEPATH += /home/preet/Downloads/Packages/glm

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

INSTALLS += pkg_files

# END PROJECT CONFIG
##########################
