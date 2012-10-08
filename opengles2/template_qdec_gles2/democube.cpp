#include "democube.h"

DemoCube::DemoCube(QDeclarativeItem *parent) :
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

    m_xf_view = glm::lookAt(m_cam_eye,m_cam_viewpoint,m_cam_up);
}

void DemoCube::initViewport()
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
    QString vShader = readFileAsQString(m_resPrefix + "shaders/cube_vert.glsl");
    byteArray = vShader.toUtf8();
    GLchar * vShaderStr = byteArray.data();
    GLuint vertexShader = loadShader(GL_VERTEX_SHADER,vShaderStr);
    glAttachShader(m_gl_hdl_prog,vertexShader);

    // setup fragment shader
    QString fShader = readFileAsQString(m_resPrefix + "shaders/cube_frag.glsl");
    byteArray = fShader.toUtf8();
    GLchar * fShaderStr = byteArray.data();
    GLuint fragmentShader = loadShader(GL_FRAGMENT_SHADER,fShaderStr);
    glAttachShader(m_gl_hdl_prog,fragmentShader);

    // bind position and color to generic vertex attribute index 0 and 1
    m_gl_idx_attrib0 = 0;
    m_gl_idx_attrib1 = 1;
    glBindAttribLocation(m_gl_hdl_prog,m_gl_idx_attrib0,"a_position");
    glBindAttribLocation(m_gl_hdl_prog,m_gl_idx_attrib1,"a_color");

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

    // create vertex data
    const GLfloat g_vertex_buffer_data[] = {
            -1.0f,-1.0f,-1.0f,
            -1.0f,-1.0f, 1.0f,
            -1.0f, 1.0f, 1.0f,
             1.0f, 1.0f,-1.0f,
            -1.0f,-1.0f,-1.0f,
            -1.0f, 1.0f,-1.0f,
             1.0f,-1.0f, 1.0f,
            -1.0f,-1.0f,-1.0f,
             1.0f,-1.0f,-1.0f,
             1.0f, 1.0f,-1.0f,
             1.0f,-1.0f,-1.0f,
            -1.0f,-1.0f,-1.0f,
            -1.0f,-1.0f,-1.0f,
            -1.0f, 1.0f, 1.0f,
            -1.0f, 1.0f,-1.0f,
             1.0f,-1.0f, 1.0f,
            -1.0f,-1.0f, 1.0f,
            -1.0f,-1.0f,-1.0f,
            -1.0f, 1.0f, 1.0f,
            -1.0f,-1.0f, 1.0f,
             1.0f,-1.0f, 1.0f,
             1.0f, 1.0f, 1.0f,
             1.0f,-1.0f,-1.0f,
             1.0f, 1.0f,-1.0f,
             1.0f,-1.0f,-1.0f,
             1.0f, 1.0f, 1.0f,
             1.0f,-1.0f, 1.0f,
             1.0f, 1.0f, 1.0f,
             1.0f, 1.0f,-1.0f,
            -1.0f, 1.0f,-1.0f,
             1.0f, 1.0f, 1.0f,
            -1.0f, 1.0f,-1.0f,
            -1.0f, 1.0f, 1.0f,
             1.0f, 1.0f, 1.0f,
            -1.0f, 1.0f, 1.0f,
             1.0f,-1.0f, 1.0f
        };

    static const GLfloat g_color_buffer_data[] = {
        0.583f,  0.771f,  0.014f,
        0.609f,  0.115f,  0.436f,
        0.327f,  0.483f,  0.844f,
        0.822f,  0.569f,  0.201f,
        0.435f,  0.602f,  0.223f,
        0.310f,  0.747f,  0.185f,
        0.597f,  0.770f,  0.761f,
        0.559f,  0.436f,  0.730f,
        0.359f,  0.583f,  0.152f,
        0.483f,  0.596f,  0.789f,
        0.559f,  0.861f,  0.639f,
        0.195f,  0.548f,  0.859f,
        0.014f,  0.184f,  0.576f,
        0.771f,  0.328f,  0.970f,
        0.406f,  0.615f,  0.116f,
        0.676f,  0.977f,  0.133f,
        0.971f,  0.572f,  0.833f,
        0.140f,  0.616f,  0.489f,
        0.997f,  0.513f,  0.064f,
        0.945f,  0.719f,  0.592f,
        0.543f,  0.021f,  0.978f,
        0.279f,  0.317f,  0.505f,
        0.167f,  0.620f,  0.077f,
        0.347f,  0.857f,  0.137f,
        0.055f,  0.953f,  0.042f,
        0.714f,  0.505f,  0.345f,
        0.783f,  0.290f,  0.734f,
        0.722f,  0.645f,  0.174f,
        0.302f,  0.455f,  0.848f,
        0.225f,  0.587f,  0.040f,
        0.517f,  0.713f,  0.338f,
        0.053f,  0.959f,  0.120f,
        0.393f,  0.621f,  0.362f,
        0.673f,  0.211f,  0.457f,
        0.820f,  0.883f,  0.371f,
        0.982f,  0.099f,  0.879f
    };

    // create and fill vbo with position data
    glGenBuffers(1,&m_gl_hdl_vbo_pos);
    glBindBuffer(GL_ARRAY_BUFFER,m_gl_hdl_vbo_pos);
    glBufferData(GL_ARRAY_BUFFER, sizeof(g_vertex_buffer_data),
                 g_vertex_buffer_data, GL_STATIC_DRAW);

    // create and fill vbo with color data
    glGenBuffers(1,&m_gl_hdl_vbo_clr);
    glBindBuffer(GL_ARRAY_BUFFER,m_gl_hdl_vbo_clr);
    glBufferData(GL_ARRAY_BUFFER, sizeof(g_color_buffer_data),
                 g_color_buffer_data, GL_STATIC_DRAW);

    // unbind all buffer for BABY UPSET QPAINTER
    // WAHWAHWAH I LIKE TO CRASH ALL THE TIME
    glBindBuffer(GL_ARRAY_BUFFER,0);
    glDisable(GL_CULL_FACE);
    glDisable(GL_DEPTH_TEST);
}

void DemoCube::drawViewport()
{
    glm::mat4 matProj = glm::perspective(45.0f,128.0f/75.0f,0.1f,100.0f);
    glm::mat4 matModel = glm::mat4(1.0f);
    glm::mat4 matModelViewProj = matProj*m_xf_view*matModel;

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

    // enable color vbo and map to attrib1
    glEnableVertexAttribArray(m_gl_idx_attrib1);
    glBindBuffer(GL_ARRAY_BUFFER,m_gl_hdl_vbo_clr);
    glVertexAttribPointer(m_gl_idx_attrib1,3,GL_FLOAT,GL_FALSE,0,(void*)0);

    glDrawArrays(GL_TRIANGLES,0,12*3);

    // disable attributes
    glDisableVertexAttribArray(m_gl_idx_attrib0);
    glDisableVertexAttribArray(m_gl_idx_attrib1);

    // unbind all buffer for BABY UPSET QPAINTER
    // WAHWAHWAH I LIKE TO CRASH ALL THE TIME
    glBindBuffer(GL_ARRAY_BUFFER,0);
    glDisable(GL_CULL_FACE);
    glDisable(GL_DEPTH_TEST);
}

void DemoCube::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    // calculate normalized cursor position
    double cx = 2*event->pos().x()/this->width() - 1;
    double cy = 1 - 2*event->pos().y()/this->height();

    // find the normal vector representing
    // mouse cursor on our virtual track ball
    double czsq = 1 - cx*cx - cy*cy;
    double cz = (czsq > 0) ? glm::sqrt(czsq) : 0;

    m_lk_cursorVec.x = cx;
    m_lk_cursorVec.y = cy;
    m_lk_cursorVec.z = cz;
    glm::normalize(m_lk_cursorVec);


}

void DemoCube::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
    // calculate normalized cursor position
    double cx = 2*event->pos().x()/this->width() - 1;
    double cy = 1 - 2*event->pos().y()/this->height();

    // find the normal vector representing
    // mouse cursor on our virtual track ball
    double czsq = 1 - cx*cx - cy*cy;
    double cz = (czsq > 0) ? glm::sqrt(czsq) : 0;

    glm::vec3 c_cursorVec;
    c_cursorVec.x = cx;
    c_cursorVec.y = cy;
    c_cursorVec.z = cz;
    c_cursorVec = glm::normalize(c_cursorVec);

    // calculate the spin axis using the cross product
    // of the current and previous cursor vecs
    glm::vec3 xf_axis(glm::cross(m_lk_cursorVec,c_cursorVec));
    xf_axis = glm::normalize(xf_axis);

    // calculate the angle between the current
    // and previous cursor vecs
    glm::float_t angleDegs = glm::acos(glm::dot(c_cursorVec,m_lk_cursorVec));
    angleDegs *= 180/3.1416;

    // create a 4x4 matrix representing the rotation
    glm::mat4 xf_cam;
    xf_cam = glm::translate(xf_cam,1.0f*m_cam_eye);
    xf_cam = glm::rotate(xf_cam,angleDegs,xf_axis);
    xf_cam = glm::translate(xf_cam,-1.0f*m_cam_eye);
    m_xf_view = xf_cam * m_xf_view;

    m_lk_cursorVec = c_cursorVec;

    qDebug() << angleDegs;
}

GLuint DemoCube::loadShader(GLenum type, const char *shaderSrc)
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
