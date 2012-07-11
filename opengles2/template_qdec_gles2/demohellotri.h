#ifndef DEMOHELLOTRI_H
#define DEMOHELLOTRI_H
#include "qdecviewportitem.h"
#include <QGLShaderProgram>

class DemoHelloTri : public QDecViewportItem
{
    Q_OBJECT
public:
    explicit DemoHelloTri(QDeclarativeItem *parent = 0);
    
private:
    void initViewport();
    void drawViewport();

    GLuint loadShader(GLenum type, const char *shaderSrc);

    GLuint m_glProg;
};

#endif // DEMOHELLOTRI_H
