#ifndef DEMOMVP_H
#define DEMOMVP_H

#include "qdecviewportitem.h"


class DemoMVP : public QDecViewportItem
{
    Q_OBJECT
public:
    explicit DemoMVP(QDeclarativeItem *parent = 0);
    
private:
    void initViewport();
    void drawViewport();

    GLuint loadShader(GLenum type, const char *shaderSrc);

    GLuint m_glProg;                    // GL shader program object
    GLuint m_glVertexBuffer;

    GLuint m_gl_hdl_prog;               // handle to program
    GLuint m_gl_hdl_vbuffer;            // handle to vertex buffer

    GLint m_gl_idx_attrib0;

    GLfloat * m_glVertexData;             //
};

#endif
