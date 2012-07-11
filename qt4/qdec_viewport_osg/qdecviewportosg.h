#ifndef MODELVIEWER_H
#define MODELVIEWER_H

#include <QtDeclarative/QDeclarativeTypeInfo>

#include "qdecviewportitem.h"
#include "openctm/openctm.h"

#include <osgViewer/Viewer>
#include <osgText/Text>
#include <osgDB/ReadFile>
#include <osgGA/TrackballManipulator>
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>
#include <osg/Program>
#include <osg/Shader>
#include <osg/Uniform>

//#define QT_OPENGL_ES2   // uh, probably not needed?

class QDecViewportOSG : public QDecViewportItem
{
    Q_OBJECT

public:
    explicit QDecViewportOSG(QDeclarativeItem *parent = 0);

public slots:
    void onMousePressed(int nx, int ny, int button);
    void onMouseMoved(int nx, int ny, int button);
    void onMouseReleased(int nx, int ny, int button);

signals:

private:
    void initViewport();
    void drawViewport();
    QString m_shaderPrefix;

    // openscenegraph
    osg::StateSet * m_osg_stateset;
    osgViewer::Viewer * m_osg_viewer;
    osgViewer::GraphicsWindowEmbedded * m_osg_window;
};

#endif
