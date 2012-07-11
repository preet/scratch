#include "qdecviewportitem.h"

QDecViewportItem::QDecViewportItem(QDeclarativeItem *parent) :
    QDeclarativeItem(parent), m_initViewport(false)
{
    m_useFrameBuffer = true;
    setFlag(QGraphicsItem::ItemHasNoContents, false);
}

QString QDecViewportItem::getMode()
{   return m_useFrameBuffer ? QString("framebuffer") : QString("direct");   }

void QDecViewportItem::setMode(const QString &myMode)
{
    if(myMode == "direct")
    {   m_useFrameBuffer = false;   }
    else if(myMode == "framebuffer")
    {   m_useFrameBuffer = true;   }
}

void QDecViewportItem::paint(QPainter *qPainter,
                             const QStyleOptionGraphicsItem *qStyle,
                             QWidget *qWidget)
{
    Q_UNUSED(qStyle);
    Q_UNUSED(qWidget);

    if(!m_initViewport)
    {   this->initViewport();   }

    if (m_useFrameBuffer)
    {
        QRectF localBounds = boundingRect();
        QRectF sceneBounds = mapRectToScene(localBounds);

        // draw fbo contents to qdeclarative item

        // notes:
        // for some reason, this doesn't work unless
        // we draw the texture to the fbo first, and
        // *then* paint the fbo itself; so we're behind by
        // a frame (but this isn't a big deal).

        // if we do it the other way around, the context's
        // paint engine gets disabled/null for some reason,
        // even if qPainter->paintEngine()->isActive() returns
        // true, and QGL::Context will give us an error
        if(qPainter->paintEngine()->type() == QPaintEngine::OpenGL2 &&
                qPainter->paintEngine()->isActive())
        {
            m_frameBufferObj->drawTexture(localBounds,
                m_frameBufferObj->texture());
        }

        // draw viewport contents to fbo
        QPainter fboPainter(m_frameBufferObj);
        fboPainter.beginNativePainting();
        m_frameBufferObj->bind();
        this->drawViewport();
        m_frameBufferObj->release();
        fboPainter.endNativePainting();
        fboPainter.end();
    }
    else
    {
        // draw viewport contents to item
        qPainter->beginNativePainting();
        this->drawViewport();
        qPainter->endNativePainting();
    }
}

void QDecViewportItem::updateViewport()
{   this->update();   }

void QDecViewportItem::initViewport()
{
    // create the frambuffer object
    m_frameBufferObj =
            new QGLFramebufferObject(width(),height(),
                QGLFramebufferObject::CombinedDepthStencil,
                                     GL_TEXTURE_2D,
                                     GL_RGBA8);



    program.addShaderFromSourceCode(QGLShader::Vertex,
        "attribute highp vec4 vertex;\n"
        "uniform highp mat4 matrix;\n"
        "void main(void)\n"
        "{\n"
        "   gl_Position = vertex;\n"
        "}");
    program.addShaderFromSourceCode(QGLShader::Fragment,
        "uniform mediump vec4 color;\n"
        "void main(void)\n"
        "{\n"
        "   gl_FragColor = vec4(1.0,0.5,0.0,1.0);\n"
        "}");
    program.link();
    program.bind();
    m_vertexLocation = program.attributeLocation("vertex");
    m_matrixLocation = program.uniformLocation("matrix");
    m_colorLocation = program.uniformLocation("color");
    qDebug() << program.log();
    program.release();

//    osg::ref_ptr<osg::ShapeDrawable> shapeCube = new osg::ShapeDrawable;
//    shapeCube->setShape(new osg::Box(osg::Vec3(0,0,0),8));

//    osg::ref_ptr<osg::Geode> geodeCube = new osg::Geode;
//    geodeCube->addDrawable(shapeCube.get());

//    osg::ref_ptr<osg::MatrixTransform> nodeXform = new osg::MatrixTransform;
//    nodeXform->addChild(geodeCube.get());
//    nodeXform->addUpdateCallback(new osg::AnimationPathCallback(osg::Vec3(0,0,0),
//                                                                osg::Z_AXIS,
//                                                                osg::inDegrees(30.0)));
//    osg::ref_ptr<osg::Group> groupRoot = new osg::Group;
//    groupRoot->addChild(nodeXform.get());

//    osg::ref_ptr<osgViewer::GraphicsWindowEmbedded> osgWindow =
//            new osgViewer::GraphicsWindowEmbedded(0,0,width(),height());
//    m_osg_viewer.getCamera()->setViewport(0,0,width(),height());
//    m_osg_viewer.getCamera()->setGraphicsContext(osgWindow.get());
//    m_osg_viewer.setThreadingModel(osgViewer::Viewer::SingleThreaded);
//    m_osg_viewer.setSceneData(groupRoot.get());
//    m_osg_viewer.getCamera()->setViewMatrixAsLookAt(osg::Vec3d(-20.0, 15.0, 10.0),
//                                                    osg::Vec3d(0.0, 0.0, 0.0),
//                                                    osg::Vec3d(0.0, 0.0, 1.0));

    // connect the timer
    connect(&m_updateTimer,SIGNAL(timeout()),
            this, SLOT(updateViewport()));

    m_updateTimer.setInterval(1000);
    m_updateTimer.start();
    m_initViewport = true;
}

void QDecViewportItem::drawViewport()
{
//    m_osg_viewer.frame();

    program.bind();
    GLfloat const triangleVertices[] = {
        0.0f,  0.0f,  0.0f,
        0.0f, 1.0f, 0.0f,
        1.0f,  0.0f, 0.0f
    };

    program.enableAttributeArray(m_vertexLocation);
    program.setAttributeArray(m_vertexLocation, triangleVertices, 3);

    glDrawArrays(GL_TRIANGLES, 0, 3);

    program.disableAttributeArray(m_vertexLocation);
    program.release();
}

