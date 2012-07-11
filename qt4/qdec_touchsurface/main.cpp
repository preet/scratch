#include <QtGui/QApplication>
#include <QtOpenGL/QGLWidget>
#include <QtDeclarative/QtDeclarative>
#include <QtDeclarative/QDeclarativeView>

#include "qdectoucharea.h"

int main(int argc, char *argv[])
{
    QApplication app(argc,argv);
    qmlRegisterType<QDecTouchPoint>("TouchItems",1,0,"TouchPoint");
    qmlRegisterType<QDecTouchArea>("TouchItems",1,0,"TouchArea");

    QDeclarativeView mainView;
    mainView.setSource(QString("app/native/ui/main.qml"));
    mainView.show();

    return app.exec();
}
