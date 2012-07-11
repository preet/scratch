#ifndef VLQGLWIDGET_H
#define VLQGLWIDGET_H

// vl includes
#include <vlGraphics/OpenGLContext.hpp>

// qt includes
#include <QGLWidget>

class VLQGLWidget : public QGLWidget, public vl::OpenGLContext
{
    Q_OBJECT

public:
    VLQGLWidget(QWidget *parent=0);
    void swapBuffers();
    void makeCurrent();
    void update();
};

#endif // VLQGLWIDGET_H
