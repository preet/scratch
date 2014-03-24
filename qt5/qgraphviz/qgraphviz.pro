QT += core

# graphviz
CONFIG += link_pkgconfig
PKGCONFIG += libcdt libgvc libcgraph

HEADERS += \
    QGVGraph.h

SOURCES += \
    QGVGraph.cpp \
    main.cpp
