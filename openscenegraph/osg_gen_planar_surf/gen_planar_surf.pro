CONFIG += link_pkgconfig debug
PKGCONFIG += openthreads openscenegraph
TARGET = gen_planar_surf
SOURCES += main.cpp
TEMPLATE = app

QMAKE_CXXFLAGS += -std=c++0x

moreFiles.path = $$OUT_PWD/textures
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
    textures/earth_right.jpg \
    textures/p_x.jpg \
    textures/p_y.jpg \
    textures/p_z.jpg \
    textures/n_x.jpg \
    textures/n_y.jpg \
    textures/n_z.jpg

INSTALLS += moreFiles
