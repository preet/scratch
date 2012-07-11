#include <QApplication>
#include <QtOpenGL/QGLWidget>
#include <QtDeclarative/QtDeclarative>
#include <QtDeclarative/QDeclarativeView>

#include "qdectoucharea.h"
#include "qdecviewportosg.h"

int main(int argc, char *argv[])
{
    QApplication app(argc,argv);
    qmlRegisterType<QDecTouchPoint>("TouchItems",1,0,"TouchPoint");
    qmlRegisterType<QDecTouchArea>("TouchItems",1,0,"TouchArea");
    qmlRegisterType<QDecViewportOSG>("ViewportItems",1,0,"ViewportItem");

    QDeclarativeView mainView;
    mainView.setViewportUpdateMode(QGraphicsView::FullViewportUpdate);

    QGLWidget *glWidget = new QGLWidget;
    qDebug() << "Current Context:" << glWidget->context()->format();

    QString resPrefix;
        #ifdef DEV_PLAYBOOK
        resPrefix = "app/native/";
        #endif

    mainView.setViewport(glWidget);
    mainView.setSource(resPrefix + "ui/main.qml");
    mainView.show();
//    mainView.showFullScreen();

    return app.exec();
}
