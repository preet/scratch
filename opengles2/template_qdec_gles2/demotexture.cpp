#include "demotexture.h"

DemoTexture::DemoTexture(QDeclarativeItem *parent) :
    QDecViewportItem(parent)
{
    setFlag(QGraphicsItem::ItemHasNoContents, false);
    m_gl_hdl_prog = 0;
    m_initFailed = false;
}

void DemoTexture::initViewport()
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
    QString vShader = readFileAsQString(m_resPrefix + "shaders/texture_vert.glsl");
    byteArray = vShader.toUtf8();
    GLchar * vShaderStr = byteArray.data();
    GLuint vertexShader = loadShader(GL_VERTEX_SHADER,vShaderStr);
    glAttachShader(m_gl_hdl_prog,vertexShader);

    // setup fragment shader
    QString fShader = readFileAsQString(m_resPrefix + "shaders/texture_frag.glsl");
    byteArray = fShader.toUtf8();
    GLchar * fShaderStr = byteArray.data();
    GLuint fragmentShader = loadShader(GL_FRAGMENT_SHADER,fShaderStr);
    glAttachShader(m_gl_hdl_prog,fragmentShader);

    // bind position and color to generic vertex attribute index 0 and 1
    m_gl_idx_attrib0 = 0;
    m_gl_idx_attrib1 = 1;
    glBindAttribLocation(m_gl_hdl_prog,m_gl_idx_attrib0,"a_position");
    glBindAttribLocation(m_gl_hdl_prog,m_gl_idx_attrib1,"a_texcoord");

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
    m_gl_loc_texsampler = glGetUniformLocation(m_gl_hdl_prog,"s_tex");

    // build geometry (simple plane)
    std::vector<glm::vec3> listVertices(4);
    listVertices[0] = glm::vec3(-1.0f,0.0f,1.0f);
    listVertices[1] = glm::vec3(-1.0f,0.0f,-1.0f);
    listVertices[2] = glm::vec3(1.0f,0.0f,-1.0f);
    listVertices[3] = glm::vec3(1.0f,0.0f,1.0f);

    std::vector<glm::vec2> listTexCoords(4);
    listTexCoords[0] = glm::vec2(0.0f,1.0f);
    listTexCoords[1] = glm::vec2(0.0f,0.0f);
    listTexCoords[2] = glm::vec2(1.0f,0.0f);
    listTexCoords[3] = glm::vec2(1.0f,1.0f);

    std::vector<unsigned int> listIndices(6);
    listIndices[0] = 0;
    listIndices[1] = 1;
    listIndices[2] = 2;

    listIndices[3] = 0;
    listIndices[4] = 2;
    listIndices[5] = 3;

    // create texture
    QImage textureImage;
    QImage textureImageGL;
    textureImage.load("textures/harle.jpg","JPG");
    textureImageGL = QGLWidget::convertToGLFormat(textureImage);

    // 2 x 2 Image, 3 bytes per pixel(R, G, B)
//    GLubyte pixels[4 * 3] =
//    {
//    255,
//    0,
//    0, // Red
//    0, 255,
//    0, // Green
//    0,
//    0, 255, // Blue
//    255, 255,
//    0 // Yellow
//    };

    // create opengl texture object and get its handle
    glGenTextures(1,&m_gl_hdl_tex);

    // bind the texture and load it
    glBindTexture(GL_TEXTURE_2D,m_gl_hdl_tex);
    glTexImage2D(GL_TEXTURE_2D,
                 0,
                 GL_RGBA,
                 textureImageGL.width(),
                 textureImageGL.height(),
                 0,
                 GL_RGBA,
                 GL_UNSIGNED_BYTE,
                 textureImageGL.bits());
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_NEAREST);

    // create and fill vbo with position data
    glGenBuffers(1,&m_gl_hdl_vbo_pos);
    glBindBuffer(GL_ARRAY_BUFFER,m_gl_hdl_vbo_pos);
    glBufferData(GL_ARRAY_BUFFER,
                 sizeof(glm::vec3)*(listVertices.size()),
                 &listVertices[0], GL_STATIC_DRAW);

    // create and fill vbo with position data
    glGenBuffers(1,&m_gl_hdl_vbo_tex);
    glBindBuffer(GL_ARRAY_BUFFER,m_gl_hdl_vbo_tex);
    glBufferData(GL_ARRAY_BUFFER,
                 sizeof(glm::vec2)*(listTexCoords.size()),
                 &listTexCoords[0], GL_STATIC_DRAW);

    // create and fill ibo with indices
    glGenBuffers(1,&m_gl_hdl_ibo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,m_gl_hdl_ibo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 sizeof(unsigned int)*(listIndices.size()),
                 &listIndices[0], GL_STATIC_DRAW);

    // unbind all buffer for BABY UPSET QPAINTER
    // WAHWAHWAH I LIKE TO CRASH ALL THE TIME
    glBindBuffer(GL_ARRAY_BUFFER,0);
    glDisable(GL_CULL_FACE);
    glDisable(GL_DEPTH_TEST);
}

void DemoTexture::drawViewport()
{
    glm::mat4 matProj = glm::perspective(45.0f,128.0f/75.0f,0.1f,100.0f);
    glm::mat4 matView = glm::lookAt(glm::vec3(3.0f,3.0f,3.0f),
                                    glm::vec3(0.0f,0.0f,0.0f),
                                    glm::vec3(0.0f,1.0f,0.0f));
    glm::mat4 matModel = glm::mat4(1.0f);
    glm::mat4 matModelViewProj = matProj*matView*matModel;

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

    // enable texture vbo and map to attrib1
    glEnableVertexAttribArray(m_gl_idx_attrib1);
    glBindBuffer(GL_ARRAY_BUFFER,m_gl_hdl_vbo_tex);
    glVertexAttribPointer(m_gl_idx_attrib1,2,GL_FLOAT,GL_FALSE,0,(void*)0);

    // bind the element index
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,m_gl_hdl_ibo);

    // set the active texture unit [to 0] and bind our texture
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D,m_gl_hdl_tex);

    // update the texture sampler uniform
    glUniform1i(m_gl_loc_texsampler,1);

    // draw!
    glDrawElements(GL_TRIANGLES,
                   2*3,
                   GL_UNSIGNED_INT,
                   (void*)0);

    // disable attributes
    glDisableVertexAttribArray(m_gl_idx_attrib0);
    glDisableVertexAttribArray(m_gl_idx_attrib1);

    // unbind all buffer for BABY UPSET QPAINTER
    // WAHWAHWAH I LIKE TO CRASH ALL THE TIME
    glBindBuffer(GL_ARRAY_BUFFER,0);
    glDisable(GL_CULL_FACE);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_SCISSOR_TEST);
}

GLuint DemoTexture::loadShader(GLenum type, const char *shaderSrc)
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
