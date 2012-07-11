#ifndef DEMOMODEL_H
#define DEMOMODEL_H

#include <iostream>
#include "qdecviewportitem.h"
#include "openctm/openctm.h"

class DemoModel : public QDecViewportItem
{
    Q_OBJECT

public:
    explicit DemoModel(QDeclarativeItem *parent = 0);

public slots:
    void onBeginViewRotate(qreal nx, qreal ny);
    void onBeginViewPan(qreal nx, qreal ny);
    void onCalcViewRotate(qreal nx, qreal ny);
    void onCalcViewRotate(qreal rotateAngle);
    void onCalcViewPan(qreal nx, qreal ny);
    void onCalcViewZoom(qreal scaleFactor);

signals:
    
private:
    void initViewport();
    void drawViewport();

    void viewRotate(glm::vec3 const &spinAxis,glm::float_t spinAngleDeg);
    void viewPan(glm::vec3 const &trVector);
    void viewZoom(glm::float_t diffDegFovY);

    void mousePressEvent(QGraphicsSceneMouseEvent *event);
    void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);

    GLuint loadShader(GLenum type, const char *shaderSrc);

    // opengl stuff
    GLuint m_gl_hdl_prog;               // handle to program
    GLuint m_gl_hdl_vbo_pos;            // handle to vertex buffer for position
    GLuint m_gl_hdl_vbo_nrm;            // handle to vertex buffer for normal
    GLuint m_gl_hdl_ibo;                // handle to index buffer object
    unsigned int m_gl_tri_count;

    GLint m_gl_idx_attrib0;             // index for generic vertex attribute 0
    GLint m_gl_idx_attrib1;             // index for generic vertex attribute 1
    GLint m_gl_loc_xf_mvp;              // location of uniform xf_mvp within program

    GLfloat * m_gl_vpos;                // vertex position data
    GLfloat * m_gl_vcolor;              // vertex color data

    //
    glm::mat4 m_xf_view;
    glm::mat4 m_xf_model;

    glm::vec3 m_lk_rotate_vec;
    glm::vec2 m_lk_pan_pos;

    glm::vec3 m_lk_cursorVec;
    glm::vec2 m_lk_cursorPos;
    Qt::MouseButton m_c_mousebtn;

    //
    glm::vec3 m_cam_eye;
    glm::vec3 m_cam_up;
    glm::vec3 m_cam_viewpoint;
    GLfloat m_cam_near;
    GLfloat m_cam_far;
    GLfloat m_cam_fovy;
    GLfloat m_cam_aspect;

    // model dims
    GLfloat m_min_x;
    GLfloat m_max_x;
    GLfloat m_min_y;
    GLfloat m_max_y;
    GLfloat m_min_z;
    GLfloat m_max_z;
};

#endif
