TEMPLATE = app
CONFIG -= qt
CONFIG += console debug
LIBS += -lbluetooth
TARGET = btserver
HEADERS += \
btserver.h \
simplescrolling.h

SOURCES += \
btserver.c \
simplescrolling.c
