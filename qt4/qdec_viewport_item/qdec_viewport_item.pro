##########################
# START CONFIG FOR QT

CONFIG += qt3d
QT += declarative opengl

# END CONFIG FOR QT
##########################

##########################
# START PROJECT CONFIG
CONFIG += link_pkgconfig
PKGCONFIG += openthreads openscenegraph
TARGET = qdec_viewport_item

INCLUDEPATH += /home/preet/Downloads/Packages/glm

SOURCES += \
    main.cpp \
    qdecviewportitem.cpp

HEADERS += \
    qdecviewportitem.h

OTHER_FILES += \
    ui/main.qml

# define other resources and install path
res_local.path = $${OUT_PWD}
res_local.files += \
   ui

INSTALLS += res_local

# END PROJECT CONFIG
##########################
