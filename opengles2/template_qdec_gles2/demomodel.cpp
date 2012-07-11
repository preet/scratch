#include "demomodel.h"

DemoModel::DemoModel(QDeclarativeItem *parent) :
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

void DemoModel::initViewport()
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
    m_gl_idx_attrib0 = 0;
    m_gl_idx_attrib1 = 1;
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
    QString meshPath = m_resPrefix + "models/heptoroid_140k.ctm";

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

        qDebug() << "EYE:" << m_cam_eye.x << "," << m_cam_eye.y << "," << m_cam_eye.z;
        qDebug() << "CTR:" << m_cam_viewpoint.x << "," << m_cam_viewpoint.y << "," << m_cam_viewpoint.z;
        qDebug() << "UP :" << m_cam_up.x << "," << m_cam_up.y << "," << m_cam_up.z;

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
    }
    else   {
        qDebug() << "Error: Couldn't load model";
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

void DemoModel::drawViewport()
{
    glm::mat4 matProj = glm::perspective(m_cam_fovy,m_cam_aspect,m_cam_near,m_cam_far);
    glm::mat4 matModelViewProj = matProj*m_xf_view*m_xf_model;

    glEnable(GL_DEPTH_TEST);
    glClearColor(0.0,0.0,0.0,0.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

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
}

void DemoModel::onBeginViewRotate(qreal nx, qreal ny)
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

void DemoModel::onBeginViewPan(qreal nx, qreal ny)
{
    m_lk_pan_pos.x = nx/this->width();
    m_lk_pan_pos.y = ny/this->height();
}

void DemoModel::onCalcViewRotate(qreal nx, qreal ny)
{
    // calculate normalized cursor position
    double cx = 2*nx/this->width() - 1;
    double cy = 1 - 2*ny/this->height();

    // find the normal vector representing
    // mouse cursor on our virtual track ball
    double czsq = 1 - cx*cx - cy*cy;
    double cz = (czsq > 0) ? glm::sqrt(czsq) : 0;

    glm::vec3 c_rotate_vec;
    c_rotate_vec.x = cx;
    c_rotate_vec.y = cy;
    c_rotate_vec.z = cz;
    c_rotate_vec = glm::normalize(c_rotate_vec);

    // calculate dist2 between the current and prev vectors
    glm::float_t fDist2 = glm::pow(c_rotate_vec.x-m_lk_rotate_vec.x,2.0f) +
                          glm::pow(c_rotate_vec.y-m_lk_rotate_vec.y,2.0f) +
                          glm::pow(c_rotate_vec.z-m_lk_rotate_vec.z,2.0f);

    if(fDist2 > 1.0e-5)  {
        // calculate the spin axis using the cross product
        // of the current and previous cursor vecs
        glm::vec3 xf_axis(glm::cross(m_lk_rotate_vec,c_rotate_vec));
        xf_axis = glm::normalize(xf_axis);

        // calculate the angle between the current
        // and previous cursor vecs
        glm::float_t angleDegs = glm::acos(glm::dot(c_rotate_vec,m_lk_rotate_vec));
        angleDegs *= 180/3.1416;
        this->viewRotate(xf_axis,angleDegs);
        m_lk_rotate_vec = c_rotate_vec;

//        qDebug() << "->" << "Dist:" << fDist2 <<  "Angle:" << angleDegs << ", Axis:"
//                     << xf_axis.x << "," << xf_axis.y << "," << xf_axis.z;
    }
}

void DemoModel::onCalcViewRotate(qreal rotateAngle)
{

}

void DemoModel::onCalcViewPan(qreal nx, qreal ny)
{
    glm::vec2 c_pan_pos;
    c_pan_pos.x = nx/this->width();
    c_pan_pos.y = ny/this->height();

    glm::vec3 xAxis(-1.0f,0.0f,0.0f);
    glm::vec3 yAxis(0.0f,1.0f,0.0f);

    GLfloat maxExtents = glm::max(m_max_x-m_min_x,m_max_y-m_min_y);

    xAxis *= ((m_lk_pan_pos.x-c_pan_pos.x) * (maxExtents));
    yAxis *= ((m_lk_pan_pos.y-c_pan_pos.y) * (maxExtents));
    glm::vec3 trVector = xAxis + yAxis;
    this->viewPan(trVector);
    m_lk_pan_pos = c_pan_pos;
}

void DemoModel::onCalcViewZoom(qreal scaleFactor)
{
    if(scaleFactor > 0)
    {   this->viewZoom(scaleFactor);   }
}

void DemoModel::viewRotate(const glm::vec3 &spinAxis, glm::float_t spinAngleDeg)
{
    glm::mat4 xf_cam;
    xf_cam = glm::translate(xf_cam,1.0f*m_cam_eye);
    xf_cam = glm::rotate(xf_cam,spinAngleDeg,spinAxis);
    xf_cam = glm::translate(xf_cam,-1.0f*m_cam_eye);
    m_xf_view = xf_cam * m_xf_view;
}

void DemoModel::viewPan(const glm::vec3 &trVector)
{
    glm::mat4 xf_translate;
    xf_translate = glm::translate(glm::mat4(1.0f),trVector);
    m_xf_view = xf_translate*m_xf_view;
}

void DemoModel::viewZoom(glm::float_t scaleFactor)
{
    // get view direction (ie "look at" vector)
    // should be the view matrix's 'z' axis
    glm::vec4 vecLookAt = m_xf_view[2];
    glm::vec3 vecZoomAlong(vecLookAt.x,
                           vecLookAt.y,
                           vecLookAt.z);


    if(scaleFactor > 1 && m_cam_fovy > 5)   {
        // zoom in
        m_cam_fovy /= scaleFactor;
    }
    else if(scaleFactor < 1 && m_cam_fovy < 165)   {
        // zoom out
        m_cam_fovy /= scaleFactor;
    }
    // view perspective calculated based on
    // m_cam_fovy each frame
}

void DemoModel::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    // rotation: get first arcball vector
    if(event->button() == Qt::LeftButton)   {
        // calculate normalized cursor position
        double cx = 2*event->pos().x()/this->width() - 1;
        double cy = 1 - 2*event->pos().y()/this->height();

        // find the normal vector representing
        // mouse cursor on our virtual track ball
        double czsq = 1 - cx*cx - cy*cy;
        double cz = (czsq > 0) ? sqrt(czsq) : 0;

//        qDebug() << cx <<","<<cy<<","<<cz;

        m_lk_cursorVec.x = cx;
        m_lk_cursorVec.y = cy;
        m_lk_cursorVec.z = cz;
        m_lk_cursorVec = glm::normalize(m_lk_cursorVec);
        m_c_mousebtn = Qt::LeftButton;
    }
    // pan
    else if(event->button() == Qt::MiddleButton)   {
        m_lk_cursorPos.x = event->pos().x()/this->width();
        m_lk_cursorPos.y = event->pos().y()/this->height();
        m_c_mousebtn = Qt::MiddleButton;
    }
    else if(event->button() == Qt::RightButton)   {
        m_lk_cursorPos.x = event->pos().x()/this->width();
        m_lk_cursorPos.y = event->pos().y()/this->height();
        m_c_mousebtn = Qt::RightButton;
    }
}

//void DemoModel::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
//{
//    // rotate
//    if(m_c_mousebtn == Qt::LeftButton)   {
//        // calculate normalized cursor position
//        double cx = 2*event->pos().x()/this->width() - 1;
//        double cy = 1 - 2*event->pos().y()/this->height();

//        // find the normal vector representing
//        // mouse cursor on our virtual track ball
//        double czsq = 1 - cx*cx - cy*cy;
//        double cz = (czsq > 0) ? sqrt(czsq) : 0;

//        glm::vec3 c_cursorVec;
//        c_cursorVec.x = cx;
//        c_cursorVec.y = cy;
//        c_cursorVec.z = cz;
//        c_cursorVec = glm::normalize(c_cursorVec);

//        // calculate the spin axis using the cross product
//        // of the current and previous cursor vecs
//        glm::vec3 xf_axis(glm::cross(m_lk_cursorVec,c_cursorVec));
//        xf_axis = glm::normalize(xf_axis);

//        // calculate the angle between the current
//        // and previous cursor vecs
//        glm::float_t angleDegs = glm::acos(glm::dot(c_cursorVec,m_lk_cursorVec));
//        angleDegs *= 180/3.1416;

//        // calculate axis in terms of world coordinates
//        glm::mat3 xf_view_inv = glm::inverse(glm::mat3(m_xf_view));
//        xf_axis = xf_view_inv * xf_axis;

//        glm::vec4 origin(0,0,0,1);
//        origin = m_xf_view * origin;

//        glm::vec3 trVector = glm::vec3(origin);

//        glm::mat4 xf_cam;
//        xf_cam = glm::translate(xf_cam,1.0f*trVector);
//        xf_cam = glm::rotate(xf_cam,angleDegs,xf_axis);
//        xf_cam = glm::translate(xf_cam,-1.0f*trVector);
//        m_xf_view = m_xf_view*xf_cam;

//        m_lk_cursorVec = c_cursorVec;
//    }
//    // pan
//    else if(m_c_mousebtn == Qt::MiddleButton)   {

//        glm::vec2 c_cursorPos;
//        c_cursorPos.x = event->pos().x()/this->width();
//        c_cursorPos.y = event->pos().y()/this->height();

//        glm::vec3 xAxis(-1.0f,0.0f,0.0f);
//        glm::vec3 yAxis(0.0f,1.0f,0.0f);

//        GLfloat maxExtents = glm::max(m_max_x-m_min_x,m_max_y-m_min_y);

//        xAxis *= ((m_lk_cursorPos.x-c_cursorPos.x) * (maxExtents));
//        yAxis *= ((m_lk_cursorPos.y-c_cursorPos.y) * (maxExtents));

//        glm::mat4 xf_translate(1.0f);
//        xf_translate = glm::translate(xf_translate,xAxis+yAxis);
//        m_xf_view = xf_translate*m_xf_view;

////        qDebug() << m_xf_view[3].x <<
////                    m_xf_view[3].y <<
////                    m_xf_view[3].z;

//        m_cam_viewpoint = m_cam_viewpoint+xAxis+yAxis;
//        m_lk_cursorPos = c_cursorPos;
//    }
//    // zoom
//    else if(m_c_mousebtn == Qt::RightButton)   {

//        glm::vec2 c_cursorPos;
//        c_cursorPos.x = event->pos().x()/this->width();
//        c_cursorPos.y = event->pos().y()/this->height();

//        glm::vec3 zAxis(0.0,0.0,1.0);
//        glm::float_t maxExtents = glm::max(m_max_x-m_min_x,m_max_y-m_min_y);
//        zAxis *= ((m_lk_cursorPos.y-c_cursorPos.y) * maxExtents);

//        glm::mat4 xf_translate;
//        xf_translate = glm::translate(glm::mat4(1.0f),zAxis);
//        m_xf_view = xf_translate*m_xf_view;

//        m_cam_viewpoint = m_cam_viewpoint+zAxis;
//        m_lk_cursorPos = c_cursorPos;
//    }
//}


void DemoModel::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
    // rotate
    if(m_c_mousebtn == Qt::LeftButton)   {
        // calculate normalized cursor position
        double cx = 2*event->pos().x()/this->width() - 1;
        double cy = 1 - 2*event->pos().y()/this->height();

        // find the normal vector representing
        // mouse cursor on our virtual track ball
        double czsq = 1 - cx*cx - cy*cy;
        double cz = (czsq > 0) ? sqrt(czsq) : 0;

        glm::vec3 c_cursorVec(cx,cy,cz);
        c_cursorVec = glm::normalize(c_cursorVec);

        // get view inverse
        glm::mat3 xf_inv_view = glm::inverse(glm::mat3(m_xf_view));

        // get angle, axis
        glm::float_t angleRads = glm::acos(glm::min(1.0f,glm::dot(c_cursorVec,m_lk_cursorVec)));
        glm::vec3 axis_cam = glm::cross(m_lk_cursorVec,c_cursorVec);
        glm::vec3 xf_spinaxis = xf_inv_view * axis_cam;

        qDebug() << ">" << m_cam_viewpoint.x
                 << "," << m_cam_viewpoint.y
                 << "," << m_cam_viewpoint.z;

        glm::mat4 xf_arcball(1.0f);
        xf_arcball = glm::translate(xf_arcball,-1.0f*m_cam_viewpoint);
        xf_arcball = glm::rotate(xf_arcball,glm::degrees(angleRads),xf_spinaxis);
        xf_arcball = glm::translate(xf_arcball,1.0f*m_cam_viewpoint);
        m_xf_view = m_xf_view * xf_arcball;

//        glm::mat4 m_xf_rotate;
//        m_xf_rotate = glm::rotate(m_xf_rotate,glm::degrees(angleRads),xf_spinaxis);
//        m_xf_view = m_xf_view * m_xf_rotate;

        m_lk_cursorVec = c_cursorVec;
    }
    // pan
    else if(m_c_mousebtn == Qt::MiddleButton)   {

        glm::vec2 c_cursorPos;
        c_cursorPos.x = event->pos().x()/this->width();
        c_cursorPos.y = event->pos().y()/this->height();

        glm::vec3 xAxis(-1.0f,0.0f,0.0f);
        glm::vec3 yAxis(0.0f,1.0f,0.0f);
        glm::float_t maxExtents = glm::max(m_max_x-m_min_x,m_max_y-m_min_y);
        xAxis *= ((m_lk_cursorPos.x-c_cursorPos.x) * (maxExtents));
        yAxis *= ((m_lk_cursorPos.y-c_cursorPos.y) * (maxExtents));

        glm::mat3 xf_inv_view = glm::inverse(glm::mat3(m_xf_view));
        glm::vec3 xf_translate = xAxis+yAxis;
        xf_translate = xf_inv_view * xf_translate;

        m_xf_view = glm::translate(m_xf_view,xf_translate);
        m_lk_cursorPos = c_cursorPos;

        // update viewpoint
        m_cam_viewpoint += xf_translate;
    }
    // zoom
    else if(m_c_mousebtn == Qt::RightButton)   {

        glm::vec2 c_cursorPos;
        c_cursorPos.x = event->pos().x()/this->width();
        c_cursorPos.y = event->pos().y()/this->height();

        glm::vec3 zAxis(0.0,0.0,1.0);
        glm::float_t maxExtents = glm::max(m_max_x-m_min_x,m_max_y-m_min_y);
        zAxis *= ((m_lk_cursorPos.y-c_cursorPos.y) * maxExtents);

        glm::mat3 xf_inv_view = glm::inverse(glm::mat3(m_xf_view));
        glm::vec3 xf_translate = xf_inv_view * zAxis;

        m_xf_view = glm::translate(m_xf_view,xf_translate);
        m_lk_cursorPos = c_cursorPos;

        // update viewpoint
//        m_cam_viewpoint += xf_translate;
    }
}

void DemoModel::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{   m_c_mousebtn = Qt::NoButton;   }

GLuint DemoModel::loadShader(GLenum type, const char *shaderSrc)
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
