#include "modelviewer.h"

ModelViewer::ModelViewer(QDeclarativeItem *parent) :
    QDecViewportItem(parent)
{
    setFlag(QGraphicsItem::ItemHasNoContents, false);
    m_gl_hdl_prog = 0;
    m_initFailed = false;

    // setup initial camera
    m_cam_eye = glm::vec3(0,0,-10);
    m_cam_viewpoint = glm::vec3(0,0,0);
    m_cam_up = glm::vec3(0,1,0);

    glm::vec3 cam_side = glm::cross((m_cam_viewpoint-m_cam_eye),m_cam_up);
    m_cam_up = glm::normalize(glm::cross((m_cam_viewpoint-m_cam_eye),cam_side));
    if(m_cam_up.y < 0)   {
        m_cam_up.x *= -1;
        m_cam_up.y *= -1;
        m_cam_up.z *= -1;
    }
}

void ModelViewer::initViewport()
{
    QByteArray byteArray;

    // create program object
    m_gl_hdl_prog = glCreateProgram();
    if(!m_gl_hdl_prog)   {
        qDebug() << "OpenGL: Failed to create program object";
        m_initFailed = true;
        return;
    }

    // setup vertex shader
    QString vShader = readFileAsQString(m_resPrefix + "shaders/model_vert.glsl");
    byteArray = vShader.toUtf8();
    GLchar * vShaderStr = byteArray.data();
    GLuint vertexShader = loadShader(GL_VERTEX_SHADER,vShaderStr);
    glAttachShader(m_gl_hdl_prog,vertexShader);

    // setup fragment shader
    QString fShader = readFileAsQString(m_resPrefix + "shaders/model_frag.glsl");
    byteArray = fShader.toUtf8();
    GLchar * fShaderStr = byteArray.data();
    GLuint fragmentShader = loadShader(GL_FRAGMENT_SHADER,fShaderStr);
    glAttachShader(m_gl_hdl_prog,fragmentShader);

    // bind position and color to generic vertex attribute index 0 and 1
    m_gl_idx_attrib0 = 1;
    m_gl_idx_attrib1 = 0;
    glBindAttribLocation(m_gl_hdl_prog,m_gl_idx_attrib0,"a_position");
    glBindAttribLocation(m_gl_hdl_prog,m_gl_idx_attrib1,"a_normal");

    // link the program
    GLint progWasLinked;
    glLinkProgram(m_gl_hdl_prog);
    glGetProgramiv(m_gl_hdl_prog,GL_LINK_STATUS,&progWasLinked);
    if(!progWasLinked)   {
        GLint infoLen = 0;
        glGetProgramiv(m_gl_hdl_prog,GL_INFO_LOG_LENGTH,&infoLen);

        if(infoLen > 1)   {
            char infoLog[infoLen];
            glGetProgramInfoLog(m_gl_hdl_prog,infoLen,NULL,infoLog);
            qDebug() << "OpenGL: Error linking program: \n"
                     << QString(infoLog);
        }
        glDeleteProgram(m_gl_hdl_prog);
        m_initFailed = true;
        return;
    }

    // get uniform locations
    m_gl_loc_xf_mvp = glGetUniformLocation(m_gl_hdl_prog,"xf_mvp");

    // build mesh
    std::vector<glm::vec3> list_verts;
    std::vector<glm::vec3> list_norms;
    std::vector<unsigned int> list_idxs;

    GLfloat minX = 1e5; GLfloat maxX = -1e5;
    GLfloat minY = 1e5; GLfloat maxY = -1e5;
    GLfloat minZ = 1e5; GLfloat maxZ = -1e5;

    // get ctm model data
    CTMcontext meshContext;
    CTMuint meshVertCount,meshTriCount;
    CTMuint const *meshIndices;
    CTMfloat const *meshVertices;
    CTMfloat const *meshNormals;
    QString meshPath = m_resPrefix + "models/dragon_100k.ctm";

    qDebug() << "Loading Model...";
    meshContext = ctmNewContext(CTM_IMPORT);
    ctmLoad(meshContext,meshPath.toLocal8Bit().data());
    if(ctmGetError(meshContext) == CTM_NONE)   {
        // access the mesh data
        meshVertCount = ctmGetInteger(meshContext,CTM_VERTEX_COUNT);
        meshIndices = ctmGetIntegerArray(meshContext,CTM_INDICES);

        meshVertices = ctmGetFloatArray(meshContext,CTM_VERTICES);
        meshNormals = ctmGetFloatArray(meshContext,CTM_NORMALS);
        meshTriCount = ctmGetInteger(meshContext,CTM_TRIANGLE_COUNT);

        if(ctmGetInteger(meshContext,CTM_HAS_NORMALS) == CTM_TRUE)   {
            qDebug() << "Normals";
        }
        qDebug() << "Info: Mesh has" << meshVertCount << "vertices";
        qDebug() << "Info: Mesh has" << meshTriCount << "triangles";

        m_gl_tri_count = meshTriCount;

        list_verts.resize(meshVertCount);
        list_norms.resize(meshVertCount);
        list_idxs.resize(meshTriCount*3);

        unsigned int vIdx=0;
        unsigned int nIdx=0;
        for(int i=0; i < list_verts.size(); i++)   {
            glm::vec3 vx;
            vx.x = meshVertices[vIdx]; vIdx++;
            vx.y = meshVertices[vIdx]; vIdx++;
            vx.z = meshVertices[vIdx]; vIdx++;
            list_verts[i] = vx;

            minX = glm::min(minX,vx.x);
            maxX = glm::max(maxX,vx.x);

            minY = glm::min(minY,vx.y);
            maxY = glm::max(maxY,vx.y);

            minZ = glm::min(minZ,vx.z);
            maxZ = glm::max(maxZ,vx.z);

            glm::vec3 nx;
            nx.x = meshNormals[nIdx]; nIdx++;
            nx.y = meshNormals[nIdx]; nIdx++;
            nx.z = meshNormals[nIdx]; nIdx++;
            list_norms[i] = nx;
        }

        for(int i=0; i < list_idxs.size(); i++)   {
            list_idxs[i] = meshIndices[i];
        }

        // define mesh center
        glm::vec3 meshCenter((maxX+minX)/2.0f,
                             (maxY+minY)/2.0f,
                             (maxZ+minZ)/2.0f);

        // setup camera
        m_xf_model = glm::translate(m_xf_model,-1.0f*meshCenter);
        m_xf_model = glm::rotate(m_xf_model,-90.0f,glm::vec3(1,0,0));

        m_cam_aspect = 128.0f/75.0f;
        m_cam_fovy = 45.0f;

        // try to fit model into frame... not being accurate,
        // with this math; some arbitrary stuff going on
        GLfloat fovy = m_cam_fovy;
        GLfloat fovx = 2.0f*glm::atan(m_cam_aspect*glm::tan(m_cam_fovy/2.0f));
        GLfloat xHalf = (maxX-minX)/2.0f;
        GLfloat yHalf = (maxY-minY)/2.0f;
        GLfloat zHalf = (maxZ-minZ)/2.0f;
        GLfloat xOffset = xHalf / glm::tan(fovx/2.0);
        GLfloat yOffset = yHalf / glm::tan(fovy/2.0);
        GLfloat zOffset = zHalf;

        GLfloat maxOffset = xOffset;
        maxOffset = glm::max(maxOffset,yOffset);
        maxOffset = glm::max(maxOffset,zOffset);
        m_cam_eye.z = -1.85f*maxOffset;

        m_xf_view = glm::lookAt(m_cam_eye,
                                m_cam_viewpoint,
                                m_cam_up);

//        qDebug() << "EYE:" << m_cam_eye.x << "," << m_cam_eye.y << "," << m_cam_eye.z;
//        qDebug() << "CTR:" << m_cam_viewpoint.x << "," << m_cam_viewpoint.y << "," << m_cam_viewpoint.z;
//        qDebug() << "UP :" << m_cam_up.x << "," << m_cam_up.y << "," << m_cam_up.z;

//        for(int i=0; i < 4; i++)  {
//            for(int j=0; j < 4; j++)  {
//                qDebug() << m_xf_view[i][j];
//            }
//        }


        GLfloat minExtent = xHalf;
        minExtent = glm::min(minExtent,yHalf);
        minExtent = glm::min(minExtent,zHalf);
        minExtent *= 2.0f;
        m_cam_near = minExtent / 4.0f;

        GLfloat maxExtent = xHalf;
        maxExtent = glm::max(maxExtent,yHalf);
        maxExtent = glm::max(maxExtent,zHalf);
        maxExtent *= 2.0f;
        m_cam_far = maxExtent * 4.0f;

        // save bounding box
        m_min_x = minX;
        m_min_y = minY;
        m_min_z = minZ;

        m_max_x = maxX;
        m_max_y = maxY;
        m_max_z = maxZ;

        // max extents along xyz
        m_max_extents = glm::max(m_max_x-m_min_x,m_max_y-m_min_y);
        m_max_extents = glm::max(m_max_extents,m_max_z-m_min_z);
    }
    else   {
        qDebug() << "Error: Couldn't read CTM file";
        qDebug() << ctmErrorString(ctmGetError(meshContext));
        m_initFailed = true;
        return;
    }

    // create and fill vbo with position data
    glGenBuffers(1,&m_gl_hdl_vbo_pos);
    glBindBuffer(GL_ARRAY_BUFFER,m_gl_hdl_vbo_pos);
    glBufferData(GL_ARRAY_BUFFER,
                 sizeof(glm::vec3)*(list_verts.size()),
                 &list_verts[0], GL_STATIC_DRAW);

    // create and fill vbo with normal data
    glGenBuffers(1,&m_gl_hdl_vbo_nrm);
    glBindBuffer(GL_ARRAY_BUFFER,m_gl_hdl_vbo_nrm);
    glBufferData(GL_ARRAY_BUFFER,
                 sizeof(glm::vec3)*(list_norms.size()),
                 &list_verts[0], GL_STATIC_DRAW);

    // create and fill ibo with indices
    glGenBuffers(1,&m_gl_hdl_ibo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,m_gl_hdl_ibo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 sizeof(unsigned int)*(list_idxs.size()),
                 &list_idxs[0], GL_STATIC_DRAW);

    // unbind all buffers for BABY UPSET QPAINTER
    // WAHWAHWAH I LIKE TO CRASH ALL THE TIME
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,0);
    glBindBuffer(GL_ARRAY_BUFFER,0);
    glDisable(GL_CULL_FACE);
    glDisable(GL_DEPTH_TEST);
}

void ModelViewer::drawViewport()
{
    glm::mat4 matProj = glm::perspective(m_cam_fovy,m_cam_aspect,m_cam_near,m_cam_far);
    glm::mat4 matModelViewProj = matProj*m_xf_view*m_xf_model;

    glEnable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_SCISSOR_TEST);

    glViewport(0,0,this->width(),this->height());

    glClear(GL_DEPTH_BUFFER_BIT);

    glUseProgram(m_gl_hdl_prog);

    // update xf matrix uniform
    glUniformMatrix4fv(m_gl_loc_xf_mvp,1,GL_FALSE,&matModelViewProj[0][0]);

    // enable position vbo and map to attrib0
    glEnableVertexAttribArray(m_gl_idx_attrib0);
    glBindBuffer(GL_ARRAY_BUFFER,m_gl_hdl_vbo_pos);
    glVertexAttribPointer(m_gl_idx_attrib0,3,GL_FLOAT,GL_FALSE,0,(void*)0);

    // enable normal vbo and map to attrib1
    glEnableVertexAttribArray(m_gl_idx_attrib1);
    glBindBuffer(GL_ARRAY_BUFFER,m_gl_hdl_vbo_nrm);
    glVertexAttribPointer(m_gl_idx_attrib1,3,GL_FLOAT,GL_FALSE,0,(void*)0);

    // bind the element index
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,m_gl_hdl_ibo);

    // draw!
    glDrawElements(GL_TRIANGLES,
                   m_gl_tri_count*3,
                   GL_UNSIGNED_INT,
                   (void*)0);

    // disable attributes
    glDisableVertexAttribArray(m_gl_idx_attrib0);
    glDisableVertexAttribArray(m_gl_idx_attrib1);

    // unbind all buffers for BABY UPSET QPAINTER
    // WAHWAHWAH I LIKE TO CRASH ALL THE TIME
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,0);
    glBindBuffer(GL_ARRAY_BUFFER,0);

    glDisable(GL_CULL_FACE);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_SCISSOR_TEST);
}

void ModelViewer::onBeginViewRotate(qreal nx, qreal ny)
{
    // get first arcball vector
    double cx = 2*nx/this->width() - 1;
    double cy = 1 - 2*ny/this->height();

    // find the normal vector representing
    // mouse cursor on our virtual track ball
    double czsq = 1 - cx*cx - cy*cy;
    double cz = (czsq > 0) ? glm::sqrt(czsq) : 0;

    m_lk_rotate_vec.x = cx;
    m_lk_rotate_vec.y = cy;
    m_lk_rotate_vec.z = cz;
    m_lk_rotate_vec = glm::normalize(m_lk_rotate_vec);
}

void ModelViewer::onBeginViewPan(qreal nx, qreal ny)
{
    m_lk_pan_pos.x = nx/this->width();
    m_lk_pan_pos.y = ny/this->height();
}

void ModelViewer::onBeginViewZoom(qreal nx, qreal ny)
{
    m_lk_zoom_pos.x = nx/this->width();
    m_lk_zoom_pos.y = ny/this->height();
}

void ModelViewer::onCalcViewRotate(qreal nx, qreal ny)
{
    // calculate normalized cursor position
    double cx = 2*nx/this->width() - 1;
    double cy = 1 - 2*ny/this->height();

    // find the normal vector representing
    // mouse cursor on our virtual track ball
    double czsq = 1 - cx*cx - cy*cy;
    double cz = (czsq > 0) ? glm::sqrt(czsq) : 0;

    glm::vec3 c_rotate_vec(cx,cy,cz);
    c_rotate_vec = glm::normalize(c_rotate_vec);

    // calculate dist2 between the current and prev vectors
    glm::float_t fDist2 = glm::pow(c_rotate_vec.x-m_lk_rotate_vec.x,2.0f) +
                          glm::pow(c_rotate_vec.y-m_lk_rotate_vec.y,2.0f) +
                          glm::pow(c_rotate_vec.z-m_lk_rotate_vec.z,2.0f);

    if(fDist2 > 1.0e-5)  {          // need reasonable threshold
        // get inverse view matrix to transform our axis/angle
        // from eye space (touch/mouse input coords) to model space
        glm::mat3 xf_inv_view = glm::inverse(glm::mat3(m_xf_view));

        // get axis, angle; note xf_spinaxis is transformed to model space
        glm::float_t angleRads = glm::acos(glm::min(1.0f,glm::dot(c_rotate_vec,m_lk_rotate_vec)));
        angleRads *= 1.50f;         // just makes rotation a bit faster
        glm::vec3 axis_cam = glm::cross(m_lk_rotate_vec,c_rotate_vec);
        glm::vec3 xf_spinaxis = xf_inv_view * axis_cam;

        // create and apply the transformation using camera
        // view point as an offset vector
        glm::mat4 xf_arcball(1.0f);
        xf_arcball = glm::translate(xf_arcball,-1.0f*m_cam_viewpoint);
        xf_arcball = glm::rotate(xf_arcball,glm::degrees(angleRads),xf_spinaxis);
        xf_arcball = glm::translate(xf_arcball,1.0f*m_cam_viewpoint);
        m_xf_view = m_xf_view * xf_arcball;

        // save last known vector
        m_lk_rotate_vec = c_rotate_vec;
    }
}

void ModelViewer::onCalcViewRotate(qreal rotateAngle)
{

}

void ModelViewer::onCalcViewPan(qreal nx, qreal ny)
{
    glm::vec2 c_pan_pos;
    c_pan_pos.x = nx/this->width();
    c_pan_pos.y = ny/this->height();

    glm::vec3 xAxis(-1.0f,0.0f,0.0f);
    glm::vec3 yAxis(0.0f,1.0f,0.0f);
    xAxis *= ((m_lk_pan_pos.x-c_pan_pos.x) * (m_max_extents));
    yAxis *= ((m_lk_pan_pos.y-c_pan_pos.y) * (m_max_extents));

    glm::mat3 xf_inv_view = glm::inverse(glm::mat3(m_xf_view));
    glm::vec3 xf_translate = xAxis+yAxis;
    xf_translate = xf_inv_view * xf_translate;

    m_xf_view = glm::translate(m_xf_view,xf_translate);
    m_lk_pan_pos = c_pan_pos;

    // we must update the viewpoint after a pan
    m_cam_viewpoint += xf_translate;
}

void ModelViewer::onCalcViewZoom(qreal nx, qreal ny)
{
    glm::vec2 c_zoom_pos;
    c_zoom_pos.x = nx/this->width();
    c_zoom_pos.y = ny/this->height();

    glm::vec3 zAxis(0.0,0.0,1.0);
    zAxis *= ((m_lk_zoom_pos.y-c_zoom_pos.y) * m_max_extents);

    glm::mat3 xf_inv_view = glm::inverse(glm::mat3(m_xf_view));
    glm::vec3 xf_translate = xf_inv_view * zAxis;

    m_xf_view = glm::translate(m_xf_view,xf_translate);
    m_lk_zoom_pos = c_zoom_pos;
}

void ModelViewer::onCalcViewZoom(qreal scaleFactor)
{
    if(scaleFactor > 0 && scaleFactor != 1)  {
        // map a doubling of the scale to moving our
        // camera towards the viewpoint by some amount

        // and map a halving of the scale to moving the
        // camera away from the viewpoint by that amount

        glm::vec3 zAxis(0.0,0.0,1.0);
        glm::float_t zMag = glm::abs(scaleFactor-1);

        if(scaleFactor < 1)  {
            zMag *= -1.0;
        }

        zAxis *= (zMag * m_max_extents);

        glm::mat3 xf_inv_view = glm::inverse(glm::mat3(m_xf_view));
        glm::vec3 xf_translate = xf_inv_view * zAxis;

        m_xf_view = glm::translate(m_xf_view,xf_translate);
    }
}

void ModelViewer::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    // rotation
    if(event->button() == Qt::LeftButton)   {
        this->onBeginViewRotate(event->pos().x(),event->pos().y());
        m_c_mousebtn = Qt::LeftButton;
    }
    // pan
    else if(event->button() == Qt::MiddleButton)   {
        this->onBeginViewPan(event->pos().x(),event->pos().y());
        m_c_mousebtn = Qt::MiddleButton;
    }
    // zoom
    else if(event->button() == Qt::RightButton)   {
        this->onBeginViewZoom(event->pos().x(),event->pos().y());
        m_c_mousebtn = Qt::RightButton;
    }
}

void ModelViewer::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{ 
    // rotate
    if(m_c_mousebtn == Qt::LeftButton)   {
        this->onCalcViewRotate(event->pos().x(),event->pos().y());
    }
    // pan
    else if(m_c_mousebtn == Qt::MiddleButton)   {
        this->onCalcViewPan(event->pos().x(),event->pos().y());
    }
    // zoom
    else if(m_c_mousebtn == Qt::RightButton)   {
        this->onCalcViewZoom(event->pos().x(),event->pos().y());
    }
}

void ModelViewer::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{   m_c_mousebtn = Qt::NoButton;   }

GLuint ModelViewer::loadShader(GLenum type, const char *shaderSrc)
{
    GLuint shader;
    GLint compileStatus;

    // create the shader object
    shader = glCreateShader(type);
    if(shader == 0)   {
        return 0;
    }

    // load the shader source
    glShaderSource(shader,1,&shaderSrc,NULL);
    glCompileShader(shader);
    glGetShaderiv(shader,GL_COMPILE_STATUS,&compileStatus);

    if(!compileStatus)   {
        GLint infoLen = 0;
        glGetShaderiv(shader,GL_INFO_LOG_LENGTH,&infoLen);

        if(infoLen > 1)   {
            char infoLog[infoLen];
            glGetShaderInfoLog(shader,infoLen,NULL,infoLog);
            qDebug() << "Error compiling shader:\n"
                     << QString(infoLog);
        }

        glDeleteShader(shader);
        return 0;
    }

    return shader;
}
