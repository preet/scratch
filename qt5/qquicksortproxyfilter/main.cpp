#include <QGuiApplication>
#include <QQuickView>
#include <QTimer>
#include <QMutex>

#include "helper.h"

int main(int argc, char **argv)
{
    QGuiApplication app(argc,argv);
    QQuickView * view = new QQuickView;
    Helper * helper = new Helper(view);
    int rval = app.exec();

    delete helper;
    delete view;

    return rval;
}
