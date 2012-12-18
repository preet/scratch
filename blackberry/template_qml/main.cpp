#include <QtGui/QApplication>
#include <QtOpenGL/QGLWidget>
#include <QtDeclarative/QtDeclarative>
#include <QtDeclarative/QDeclarativeView>
#include <QDebug>

#include <iostream>

int main(int argc, char *argv[])
{
    QApplication app(argc,argv);
    QDeclarativeView mainView;

    QDir myPath;
    qDebug() << "absolutePath " << myPath.absolutePath();
    qDebug() << "canonicalPath " << myPath.canonicalPath();
    qDebug() << "entryList:";
    QStringList eList = myPath.entryList();
    for(int i=0; i < eList.size(); i++)   {
        qDebug() << " * " << eList[i];
    }

    mainView.setSource(QString("app/native/ui/main.qml"));
    mainView.show();

    return app.exec();
}
