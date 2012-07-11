#include "qdecviewportitem.h"

QDecViewportItem::QDecViewportItem(QDeclarativeItem *parent) :
    QDeclarativeItem(parent),
    m_initViewport(false),
    m_initFailed(false)
{
    setFlag(QGraphicsItem::ItemHasNoContents, false);

    #ifdef DEV_PC
    m_resPrefix = "";
    setAcceptedMouseButtons(Qt::LeftButton | Qt::RightButton | Qt::MidButton);
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
            // save opengl context format
            QGLWidget * thisWidget = qobject_cast<QGLWidget*>(qWidget);
            m_qglFormat = thisWidget->format();
//            this->debugGLContext();

            // create the fbo
//            m_frameBufferObj =
//                    new QGLFramebufferObject(width(),height(),
//                        QGLFramebufferObject::CombinedDepthStencil, // play with this value?
//                                             GL_TEXTURE_2D,
//                                             GL_RGBA);

            // run the implemented init method
                        std::cout << "=================================================================\n";
                        std::cout << "=================================================================\n";
                        std::cout << "INITIALIZATION START\n";
                        std::cout << "=================================================================\n";
                        std::cout << "=================================================================\n";
                        this->initViewport();
                        std::cout << "=================================================================\n";
                        std::cout << "=================================================================\n";
                        std::cout << "INITIALIZATION END\n";
                        std::cout << "=================================================================\n";
                        std::cout << "=================================================================\n";

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
//    if(qPainter->paintEngine()->type() == QPaintEngine::OpenGL2 &&
//            qPainter->paintEngine()->isActive())
//    {
//        m_frameBufferObj->drawTexture(localBounds,
//            m_frameBufferObj->texture());
//    }

//    // draw viewport contents to fbo
//    QPainter fboPainter(m_frameBufferObj);
//    fboPainter.beginNativePainting();
//    m_frameBufferObj->bind();


        std::cout << "=================================================================\n";
        std::cout << "=================================================================\n";
        std::cout << "BEGINNATIVEPAINTING START\n";
        std::cout << "=================================================================\n";
        std::cout << "=================================================================\n";

        qPainter->beginNativePainting();
    //    m_frameBufferObj->bind();
        std::cout << "=================================================================\n";
        std::cout << "DRAW VIEWPORT START\n";
        std::cout << "=================================================================\n";
        this->drawViewport();
        std::cout << "=================================================================\n";
        std::cout << "DRAW VIEWPORT END\n";
        std::cout << "=================================================================\n";
    //    m_frameBufferObj->release();
        qPainter->endNativePainting();
        std::cout << "=================================================================\n";
        std::cout << "=================================================================\n";
        std::cout << "BEGINNATIVEPAINTING END\n";
        std::cout << "=================================================================\n";
        std::cout << "=================================================================\n";


//    m_frameBufferObj->release();
//    fboPainter.endNativePainting();
//    fboPainter.end();
}

void QDecViewportItem::updateViewport()
{   this->update();   }

void QDecViewportItem::debugGLContext()
{
    qDebug() << "==========================";
    qDebug() << "Current Rendering Context:";
    qDebug() << "==========================";

    qDebug() << "Red Buffer Size: " << m_qglFormat.redBufferSize();
    qDebug() << "Green Buffer Size: " << m_qglFormat.greenBufferSize();
    qDebug() << "Blue Buffer Size: " << m_qglFormat.blueBufferSize();

    qDebug() << "Alpha Buffer: " << m_qglFormat.alpha();
    if(m_qglFormat.alpha())  {
        qDebug() << "-> Buffer Size: " << m_qglFormat.alphaBufferSize();
    }

    qDebug() << "Accumulation Buffer: " << m_qglFormat.accum();
    if(m_qglFormat.accum())  {
        qDebug() << "-> Buffer Size: " << m_qglFormat.accumBufferSize();
    }

    qDebug() << "Depth Buffer: " << m_qglFormat.depth();
    if(m_qglFormat.depth())  {
        qDebug() << "-> Buffer Size: " << m_qglFormat.depthBufferSize();
    }

    qDebug() << "Stencil Buffer: " << m_qglFormat.stencil();
    if(m_qglFormat.stencil())  {
        qDebug() << "-> Buffer Size: " << m_qglFormat.stencilBufferSize();
    }

    qDebug() << "Double Buffering: " << m_qglFormat.doubleBuffer();
    qDebug() << "==========================";
}

QString QDecViewportItem::readFileAsQString(const QString &myFilePath)
{
    QFile myFile(myFilePath);
    myFile.open(QIODevice::ReadOnly);

    QTextStream textStream(&myFile);
    return textStream.readAll();
}
