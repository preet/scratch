#include <QApplication>
#include <QtOpenGL/QGLWidget>
#include <QtDeclarative/QtDeclarative>
#include <QtDeclarative/QDeclarativeView>

#include "qdecviewportitem.h"

int main(int argc, char *argv[])
{
    QApplication app(argc,argv);
    qmlRegisterType<QDecViewportItem>("ViewportItems",1,0,"ViewportItem");

    QDeclarativeView mainView;
    mainView.setViewportUpdateMode(QGraphicsView::FullViewportUpdate);

    QGLWidget *glWidget = new QGLWidget;
    mainView.setViewport(glWidget);
    mainView.setSource(QString("ui/main.qml"));
    mainView.show();

    return app.exec();
}
