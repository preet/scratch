#ifndef QDECVIEWPORT_VISLIB
#define QDECVIEWPORT_VISLIB

#include "qdecviewportitem.h"

// visualization library
#include <vlCore/VisualizationLibrary.hpp>
#include <vlCore/GlobalSettings.hpp>
#include <vlGraphics/OpenGLContext.hpp>
#include <vlGraphics/Rendering.hpp>
#include <vlGraphics/SceneManagerActorTree.hpp>
#include <vlGraphics/Geometry.hpp>
#include <vlGraphics/GeometryPrimitives.hpp>
#include <vlGraphics/Light.hpp>
#include <vlGraphics/TrackballManipulator.hpp>

class VLGLContext : public vl::OpenGLContext
{
public:
    VLGLContext()      {}
    void swapBuffers() {}
    void makeCurrent() {}
    void update()      {}
};

class QDecViewportVisLib : public QDecViewportItem
{
    Q_OBJECT

public:
    explicit QDecViewportVisLib(QDeclarativeItem *parent = 0);

public slots:

signals:

private:
    void initViewport();
    void drawViewport();
    QString m_shaderPrefix;

    // vis lib
    VLGLContext m_vl_glContext;
    vl::ref<vl::Rendering> m_vl_rendering;
    vl::ref<vl::SceneManagerActorTree> m_vl_sceneMan;
    vl::ref<vl::Transform> m_vl_xform;
};

#endif
