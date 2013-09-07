#include <QGuiApplication>
#include <QQuickView>
#include <QTimer>

#include "helper.h"

int main(int argc, char **argv)
{
    QGuiApplication app(argc, argv);

    QQuickView * view = new QQuickView;
    Helper * helper = new Helper(view);
    view->setResizeMode(QQuickView::SizeRootObjectToView);
    view->setSource(QUrl("qrc:/main.qml"));


//    view->showFullScreen();
//    view->showMaximized();
    view->show();

    return app.exec();
}
