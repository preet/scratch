##########################
# START PROJECT CONFIG
CONFIG -= qt
LIBS += -lbps -lscreen -lGLESv2 -lEGL -lfreetype -lpng
DEFINES += USING_GL20  # for bbutil

INCLUDEPATH += ./utils


SOURCES += \
    utils/esShader.c \
    utils/esShapes.c \
    utils/esTransform.c \
    utils/esUtil.c \
    utils/esUtil_TGA.c \
    utils/esUtil_qnx.c \
    main.c

HEADERS += \
    utils/esUtil.h \
    utils/esUtil_qnx.h

# bar descriptor, icon, splashscreen
pkg_files.path = $${OUT_PWD}
pkg_files.files += \
    bar_descriptor.xml \
    icon.png \
    splashscreen.png

INSTALLS += pkg_files

# END PROJECT CONFIG
##########################
