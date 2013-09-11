#include <QGuiApplication>

#include <QtQuick/QQuickView>

#include "qquickfboviewportosg.h"

int main(int argc, char **argv)
{
    QGuiApplication app(argc, argv);

    qmlRegisterType<QQuickFBOViewportOSG>("ViewportItems", 1, 0, "ViewportItem");

    QQuickView view;
    view.setResizeMode(QQuickView::SizeRootObjectToView);
    view.setSource(QUrl("qrc:/main.qml"));
    view.show();

//#ifdef ENV_DEV
//    view.show();
//#endif

//#ifdef ENV_ANDROID
//    view.showFullScreen();
//#endif

    return app.exec();
}
