#include <QApplication>
#include <QtOpenGL/QGLWidget>
#include <QtDeclarative/QtDeclarative>
#include <QtDeclarative/QDeclarativeView>

//#include "vlqglwidget.h"
#include "qdectoucharea.h"
#include "qdecviewportvislib.h"

int main(int argc, char *argv[])
{
    QApplication app(argc,argv);
    qmlRegisterType<QDecTouchPoint>("TouchItems",1,0,"TouchPoint");
    qmlRegisterType<QDecTouchArea>("TouchItems",1,0,"TouchArea");
    qmlRegisterType<QDecViewportVisLib>("ViewportItems",1,0,"ViewportItem");

    QDeclarativeView mainView;
    mainView.setViewportUpdateMode(QGraphicsView::FullViewportUpdate);

    QGLWidget *glWidget = new QGLWidget;

    QString resPrefix;
        #ifdef DEV_PLAYBOOK
        resPrefix = "app/native/";
        #endif


        //    const GLubyte* pGPU = glGetString(GL_RENDERER);
        //    const GLubyte* pVersion = glGetString(GL_VERSION);
        //    const GLubyte* pShaderVersion = glGetString(GL_SHADING_LANGUAGE_VERSION);
        //    qDebug() << "m_glVertexBuffer: " << m_glVertexBuffer;
        //    qDebug() << "GPU: " << QString((char*)pGPU).trimmed();
        //    qDebug() << "OpenGL: " << QString((char*)pVersion).trimmed();
        //    qDebug() << "GLSL: " << QString((char*)pShaderVersion);

    mainView.setViewport(glWidget);
    mainView.setSource(resPrefix + "ui/main.qml");
    mainView.show();
//    mainView.showFullScreen();

    return app.exec();
}
