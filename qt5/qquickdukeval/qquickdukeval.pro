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
