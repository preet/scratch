#include "vlqglwidget.h"

VLQGLWidget::VLQGLWidget(QWidget *parent) :
    QGLWidget(parent)
{
    // vl::OpenGLContext::setContinuousUpdate(false)
    // if the opengl context is a widget, this function sets
    // whether its area is continuously updated each frame
    this->setContinuousUpdate(false);

    // let qt take care of object destruction
    vl::OpenGLContext::setAutomaticDelete(false);
}

void VLQGLWidget::update()
{   QGLWidget::update();   }

void VLQGLWidget::swapBuffers()
{   QGLWidget::swapBuffers();   }

void VLQGLWidget::makeCurrent()
{   QGLWidget::makeCurrent();   }
