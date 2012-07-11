#ifndef DEMOCUBE_H
#define DEMOCUBE_H

#include "qdecviewportitem.h"
#include <QGraphicsSceneMouseEvent>

class DemoCube : public QDecViewportItem
{
    Q_OBJECT
public:
    explicit DemoCube(QDeclarativeItem *parent = 0);
    
private:
    void initViewport();
    void drawViewport();
    void mousePressEvent(QGraphicsSceneMouseEvent *event);
    void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
    GLuint loadShader(GLenum type, const char *shaderSrc);

    // opengl stuff
    GLuint m_gl_hdl_prog;               // handle to program
    GLuint m_gl_hdl_vbo_pos;            // handle to vertex buffer for position
    GLuint m_gl_hdl_vbo_clr;            // handle to vertex buffer for color

    GLint m_gl_idx_attrib0;             // index for generic vertex attribute 0
    GLint m_gl_idx_attrib1;             // index for generic vertex attribute 1
    GLint m_gl_loc_xf_mvp;              // location of uniform xf_mvp within program

    GLfloat * m_gl_vpos;                // vertex position data
    GLfloat * m_gl_vcolor;              // vertex color data

    //
    glm::mat4 m_xf_view;
    glm::vec3 m_lk_cursorVec;

    //
    glm::vec3 m_cam_eye;
    glm::vec3 m_cam_up;
    glm::vec3 m_cam_viewpoint;
};

#endif
