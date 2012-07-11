#include "qdecviewportitem.h"

QDecViewportItem::QDecViewportItem(QDeclarativeItem *parent) :
    QDeclarativeItem(parent),
    m_initViewport(false),
    m_initFailed(false)
{
    setFlag(QGraphicsItem::ItemHasNoContents, false);

    #ifdef DEV_PC
    m_resPrefix = "";
    #endif

    #ifdef DEV_PLAYBOOK
    m_resPrefix = "app/native/";
    #endif
}

void QDecViewportItem::paint(QPainter *qPainter,
                             const QStyleOptionGraphicsItem *qStyle,
                             QWidget *qWidget)
{
    Q_UNUSED(qStyle);
    Q_UNUSED(qWidget);




    if(!m_initViewport)
    {
        if(!m_initFailed)
        {
            // create the fbo
//            m_frameBufferObj =
//                    new QGLFramebufferObject(width(),height(),
//                        QGLFramebufferObject::CombinedDepthStencil, // play with this value?
//                                             GL_TEXTURE_2D,
//                                             GL_RGBA);

            // run the implemented init method
//            std::cout << "=================================================================\n";
//            std::cout << "=================================================================\n";
//            std::cout << "INITIALIZATION START\n";
//            std::cout << "=================================================================\n";
//            std::cout << "=================================================================\n";
            this->initViewport();
//            std::cout << "=================================================================\n";
//            std::cout << "=================================================================\n";
//            std::cout << "INITIALIZATION END\n";
//            std::cout << "=================================================================\n";
//            std::cout << "=================================================================\n";

            if(m_initFailed)
            {   return;   }

            // connect the timer
            connect(&m_updateTimer,SIGNAL(timeout()),
                    this, SLOT(updateViewport()));

            m_updateTimer.setInterval(100);
            m_updateTimer.start();
            m_initViewport = true;
        }
        else
        {   return;   }
    }

    QRectF localBounds = boundingRect();
    QRectF sceneBounds = mapRectToScene(localBounds);

    // notes:
    // for some reason, this doesn't work unless
    // we draw the texture to the fbo first, and
    // *then* paint the fbo itself; so we're behind by
    // a frame (but this isn't a big deal)

    // if we do it the other way around, the context's
    // paint engine gets disabled/null even if
    // qPainter->paintEngine()->isActive() returns
    // true, and QGL::Context will give us an error

    // also, keep in mind that using an fbo like this
    // every frame is pretty expensive, especially for
    // a mobile target, so we may get slow downs
//    if(qPainter->paintEngine()->type() == QPaintEngine::OpenGL2 &&
//            qPainter->paintEngine()->isActive())
//    {
//        m_frameBufferObj->drawTexture(localBounds,
//            m_frameBufferObj->texture());
//    }

    // beginNativePainting

    // flushes the painting pipeline and prepares for the
    // user issuing commands directly to the underlying
    // graphics context:

    // * blending is disabled

    // * depth, stencil and scissor tests are disabled

    // * active texture unit is reset to 0

    // * depth mask, depth function and the clear depth
    //   are reset to their default values

    // * stencil mask, stencil operation and stencil
    //   function are reset to their default values

    // * current color is reset to solid white

    // draw scene contents to the fbo
//    std::cout << "=================================================================\n";
//    std::cout << "=================================================================\n";
//    std::cout << "BEGINNATIVEPAINTING START\n";
//    std::cout << "=================================================================\n";
//    std::cout << "=================================================================\n";

    qPainter->beginNativePainting();
//    m_frameBufferObj->bind();
//    std::cout << "=================================================================\n";
//    std::cout << "DRAW VIEWPORT START\n";
//    std::cout << "=================================================================\n";
    this->drawViewport();
//    std::cout << "=================================================================\n";
//    std::cout << "DRAW VIEWPORT END\n";
//    std::cout << "=================================================================\n";
//    m_frameBufferObj->release();
    qPainter->endNativePainting();
//    std::cout << "=================================================================\n";
//    std::cout << "=================================================================\n";
//    std::cout << "BEGINNATIVEPAINTING END\n";
//    std::cout << "=================================================================\n";
//    std::cout << "=================================================================\n";
}

void QDecViewportItem::updateViewport()
{   this->update();   }

QString QDecViewportItem::readFileAsQString(const QString &myFilePath)
{
    QFile myFile(myFilePath);
    myFile.open(QIODevice::ReadOnly);

    QTextStream textStream(&myFile);
    return textStream.readAll();
}
