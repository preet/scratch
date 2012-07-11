#ifndef QDECVIEWPORT_H
#define QDECVIEWPORT_H

#ifdef DEV_PC
    #define GL_GLEXT_PROTOTYPES
    #include <GL/gl.h>
    #include <GL/glext.h>
#endif

#ifdef DEV_PLAYBOOK
    #define GL_RGBA8 GL_RGBA
    #include <GLES2/gl2.h>
#endif

#include <iostream>
#include <QTimer>
#include <QFile>
#include <QString>
#include <QGLWidget>
#include <QDeclarativeItem>
#include <QGLFramebufferObject>
#include <QGraphicsSceneMouseEvent>

class QDecViewportItem : public QDeclarativeItem
{
    Q_OBJECT

public:
    QDecViewportItem(QDeclarativeItem *parent = 0);
    void paint(QPainter *defPainter,
               const QStyleOptionGraphicsItem *style,
               QWidget *widget);

public slots:
    void updateViewport();

protected:
    virtual void initViewport() = 0;
    virtual void drawViewport() = 0;
    QString readFileAsQString(QString const &myFile);
    void debugGLContext();

    bool m_initFailed;
    QString m_resPrefix;
    QGLFormat m_qglFormat;

private:
    bool m_initViewport;
    QTimer m_updateTimer;
    QGLFramebufferObject *m_frameBufferObj;
};

#endif
