#include <QtGui/QApplication>
#include <QtOpenGL/QGLWidget>
#include <QtDeclarative/QtDeclarative>
#include <QtDeclarative/QDeclarativeView>
#include <Qt3DQuick/qdeclarativeview3d.h>

#include <iostream>

int main(int argc, char *argv[])
{
    QApplication app(argc,argv);
    QDeclarativeView3D mainView;

    mainView.setSource(QString("app/native/ui/main.qml"));
    mainView.show();

    return app.exec();
}
