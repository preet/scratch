#include <QGuiApplication>
#include <QQuickView>
#include <QTimer>

// #include "simpleobject.h"

int main(int argc, char **argv)
{
    QGuiApplication app(argc, argv);

    QQuickView * view = new QQuickView;
    view->setResizeMode(QQuickView::SizeRootObjectToView);
    view->setSource(QUrl("qrc:/main.qml"));
    view->showFullScreen();
//    view->showMaximized();
//    view->show();

    return app.exec();
}
