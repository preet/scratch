#include "qqviewportosg.h"


QSGFBONode::QSGFBONode(QQuickWindow * window) :
    m_fbo(NULL),
    m_texture(NULL),
    m_window(window),
    m_logoRenderer(NULL)
{
    connect(m_window,SIGNAL(beforeRendering()),
            this,SLOT(onRenderFBO()),
            Qt::DirectConnection);
}

QSGFBONode::~QSGFBONode()
{
    delete m_texture;
    delete m_fbo;
    delete m_logoRenderer;
}

void QSGFBONode::onRenderFBO()
{
    QSize size = rect().size().toSize();

    // Create the FBO if it doesn't exist
    if(!m_fbo)   {
        this->initNode();
    }

    m_render_frame_mutex.lock();
    if(m_render_frame)   {
        m_render_frame = false;
        m_render_frame_mutex.unlock();

        // Bind rendering to the SG Node FBO
        m_fbo->bind();

        // Render the frame
        glViewport(0, 0, size.width(), size.height());
        m_logoRenderer->render();

        // Release rendering back to system FBO
        m_fbo->release();
    }
    else
    {   m_render_frame_mutex.unlock();   }
}

void QSGFBONode::onQueueFrame()
{
    m_render_frame_mutex.lock();
    m_render_frame = true;
    m_render_frame_mutex.unlock();
}

void QSGFBONode::initNode()
{
    QSize size = rect().size().toSize();
    QOpenGLFramebufferObjectFormat format;
    format.setAttachment(QOpenGLFramebufferObject::CombinedDepthStencil);
    m_fbo = new QOpenGLFramebufferObject(size, format);
    m_texture = m_window->createTextureFromId(m_fbo->texture(), size);

    m_logoRenderer = new LogoRenderer();
    m_logoRenderer->initialize();

    this->setTexture(m_texture);
}


QQuickFBOViewport::QQuickFBOViewport()
{
    setFlag(ItemHasContents,true);

    QTimer * updTimer = new QTimer;
    connect(updTimer,SIGNAL(timeout()),
            this,SLOT(onRenderFrame()));
    updTimer->start(60);
}

void QQuickFBOViewport::onRenderFrame()
{
    emit queueFrame();
    this->update();
}

QSGNode * QQuickFBOViewport::updatePaintNode(QSGNode * oldNode,
                                             UpdatePaintNodeData * updData)
{
    QSGFBONode * fboNode = static_cast<QSGFBONode*>(oldNode);
    if(!fboNode)   {
        fboNode = new QSGFBONode(window());
        connect(this,SIGNAL(queueFrame()),
                fboNode,SLOT(onQueueFrame()));
    }
    fboNode->setRect(boundingRect());

    return fboNode;
}
