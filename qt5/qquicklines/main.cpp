#include <QtGui>
#include <QtQuick>

#include "QQuickLine.h"

int main(int argc, char **argv)
{
    QGuiApplication app(argc,argv);
    qmlRegisterType<QQuickLineItem>("QtQuickLines",1,0,"Line");

    QQuickView * view = new QQuickView;
    view->setSource(QUrl("/home/preet/Dev/scratch/qt5/qquickcubicbezier/main.qml"));
    view->show();
    int rval = app.exec();

    //delete helper;
    delete view;

    return rval;
}
