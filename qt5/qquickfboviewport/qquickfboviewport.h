#ifndef QQUICKFBOVIEWPORT_H
#define QQUICKFBOVIEWPORT_H

#include <QTimer>
#include <QMutex>
#include <QQuickItem>
#include <QQuickWindow>
#include <QSGSimpleTextureNode>
#include <QtGui/QOpenGLFramebufferObject>

class QSGFBONode : public QObject, public QSGSimpleTextureNode
{
    Q_OBJECT

public:
    QSGFBONode(QQuickWindow * window);
    ~QSGFBONode();

public slots:
    // onRenderFBO
    // * called whenever QQuickWindow::beforeRendering()
    //   is emitted
    // * creates/sets up the FBO and QSGSimpleTextureNode
    //   if they don't exist
    // * renders custom opengl to the FBO if a frame
    //   is queued up
    void onRenderFBO();

    // onQueueFrame
    // * sets a flag that will render a frame using
    //   the internal renderer to the FBO the next
    //   time 'onRenderFBO' is called
    void onQueueFrame();

private:
    virtual void renderFrame() = 0;
    void initNode();


    QOpenGLFramebufferObject    * m_fbo;
    QSGTexture                  * m_texture;
    QQuickWindow                * m_window;

    bool                          m_render_frame;
    QMutex                        m_render_frame_mutex;
};



class QQuickFBOViewport : public QQuickItem
{
    Q_OBJECT

public:
    QQuickFBOViewport();

public slots:
    // onRenderFrame
    // * queues a new frame for the viewport by calling
    //   QQuickItem::update() and sending a signal to
    //   this item's node to render the viewport
    void onRenderFrame();

signals:
    void queueFrame();


protected:
    // updatePaintNode
    // * triggered after a call to QQuickItem::update()
    QSGNode * updatePaintNode(QSGNode *, UpdatePaintNodeData *);

private:

};

#endif // QQUICKFBOVIEWPORT_H
