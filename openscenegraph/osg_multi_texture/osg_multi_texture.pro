CONFIG += link_pkgconfig debug
PKGCONFIG += openthreads openscenegraph
TARGET = osg_multi_texture
SOURCES += main.cpp
TEMPLATE = app

QMAKE_CXXFLAGS += -std=c++0x

OTHER_FILES += \
    textures/mdblue.png \
    textures/mdgrn.png \
    textures/mdorg.png \
    textures/mdpurple.png \
    textures/mdred.png \
    textures/mdylw.png

moreFiles.path = $$OUT_PWD\textures
moreFiles.files += \
    textures/mdblue.png \
    textures/mdgrn.png \
    textures/mdorg.png \
    textures/mdpurple.png \
    textures/mdred.png \
    textures/mdylw.png \
    textures/earth_front.jpg \
    textures/earth_back.jpg \
    textures/earth_top.jpg \
    textures/earth_btm.jpg \
    textures/earth_left.jpg \
    textures/earth_right.jpg

INSTALLS += moreFiles
