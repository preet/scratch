#include <QApplication>
#include <QtOpenGL/QGLWidget>
#include <QtDeclarative/QtDeclarative>
#include <QtDeclarative/QDeclarativeView>

#include "qdectoucharea.h"
#include "demotexture.h"

int main(int argc, char *argv[])
{
    QApplication app(argc,argv);
    qmlRegisterType<QDecTouchPoint>("TouchItems",1,0,"TouchPoint");
    qmlRegisterType<QDecTouchArea>("TouchItems",1,0,"TouchArea");
    qmlRegisterType<DemoTexture>("ViewportItems",1,0,"ViewportItem");


    QDeclarativeView mainView;
    mainView.setViewportUpdateMode(QGraphicsView::FullViewportUpdate);

    QGLWidget *glWidget = new QGLWidget;

    QString resPrefix;
        #ifdef DEV_PLAYBOOK
        resPrefix = "app/native/";
        #endif

        // remember right now there's no active opengl
        // context, so you can't call this here!
//        const GLubyte* pGPU = glGetString(GL_RENDERER);
//        const GLubyte* pVersion = glGetString(GL_VERSION);
//        const GLubyte* pShaderVersion = glGetString(GL_SHADING_LANGUAGE_VERSION);

//        qDebug() << "GPU: " << QString((char*)pGPU).trimmed();
//        qDebug() << "OpenGL: " << QString((char*)pVersion).trimmed();
//        qDebug() << "GLSL: " << QString((char*)pShaderVersion);

    mainView.setViewport(glWidget);
    mainView.setSource(resPrefix + "ui/main.qml");
    mainView.show();

    return app.exec();
}
