#ifndef MODELVIEWER_H
#define MODELVIEWER_H

#include "qdecviewportitem.h"
#include "openctm/openctm.h"

class ModelViewer : public QDecViewportItem
{
    Q_OBJECT

public:
    explicit ModelViewer(QDeclarativeItem *parent = 0);

public slots:
    void onBeginViewRotate(qreal nx, qreal ny);
    void onBeginViewPan(qreal nx, qreal ny);
    void onBeginViewZoom(qreal nx, qreal ny);

    void onCalcViewRotate(qreal nx, qreal ny);
    void onCalcViewPan(qreal nx, qreal ny);
    void onCalcViewZoom(qreal nx, qreal ny);

    void onCalcViewRotate(qreal rotateAngle);
    void onCalcViewZoom(qreal scaleFactor);

signals:
    
private:
    void initViewport();
    void drawViewport();

    void mousePressEvent(QGraphicsSceneMouseEvent *event);
    void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);

    GLuint loadShader(GLenum type, const char *shaderSrc);

    GLuint m_gl_hdl_prog;               // handle to program
    GLuint m_gl_hdl_vbo_pos;            // handle to vertex buffer for position
    GLuint m_gl_hdl_vbo_nrm;            // handle to vertex buffer for normal
    GLuint m_gl_hdl_ibo;                // handle to index buffer object
    unsigned int m_gl_tri_count;        // number of triangles in mesh

    GLint m_gl_idx_attrib0;             // index for generic vertex attribute 0
    GLint m_gl_idx_attrib1;             // index for generic vertex attribute 1
    GLint m_gl_loc_xf_mvp;              // location of uniform xf_mvp within program

    glm::mat4 m_xf_view;                // view transform matrix
    glm::mat4 m_xf_model;               // model transform matrix

    glm::vec3 m_lk_rotate_vec;          // last known arcball rotation vector
    glm::vec2 m_lk_pan_pos;             // last known pan input postion
    glm::vec2 m_lk_zoom_pos;            // last known zoom input position (only applies to mouse)
    Qt::MouseButton m_c_mousebtn;       // current mousebutton state

    // camera
    glm::vec3 m_cam_eye;
    glm::vec3 m_cam_up;
    glm::vec3 m_cam_viewpoint;
    glm::float_t m_cam_near;
    glm::float_t m_cam_far;
    glm::float_t m_cam_fovy;
    glm::float_t m_cam_aspect;

    // model bounding box
    glm::float_t m_min_x;
    glm::float_t m_max_x;
    glm::float_t m_min_y;
    glm::float_t m_max_y;
    glm::float_t m_min_z;
    glm::float_t m_max_z;
    glm::float_t m_max_extents;
};

#endif
