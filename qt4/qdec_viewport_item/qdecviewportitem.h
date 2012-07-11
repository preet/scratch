#ifndef QDECVIEWPORT_H
#define QDECVIEWPORT_H

#include <glm/glm.hpp>

#include <QTimer>
#include <QColor>
#include <QGLWidget>
#include <QDeclarativeItem>
#include <QGLFramebufferObject>
#include <QGLShaderProgram>

//#include <osg/ref_ptr>
//#include <osgDB/ReadFile>
//#include <osg/AnimationPath>
//#include <osgViewer/Viewer>
//#include <osg/MatrixTransform>
//#include <osg/ShapeDrawable>

#define highp
#define mediump
#define lowp

class QDecViewportItem : public QDeclarativeItem
{
    Q_OBJECT
    Q_PROPERTY(QString mode READ getMode WRITE setMode)

public:
    QDecViewportItem(QDeclarativeItem *parent = 0);
    QString getMode();
    void setMode(QString const &);
    void paint(QPainter *defPainter,
               const QStyleOptionGraphicsItem *style,
               QWidget *widget);

public slots:
    void updateViewport();

private:
    void initViewport();
    void drawViewport();

    bool m_useFrameBuffer;
    bool m_initViewport;
    QTimer m_updateTimer;
    QGLFramebufferObject *m_frameBufferObj;

    int m_vertexLocation;
    int m_matrixLocation;
    int m_colorLocation;
    QGLShaderProgram program;

//    osgViewer::Viewer m_osg_viewer;
};

#endif
