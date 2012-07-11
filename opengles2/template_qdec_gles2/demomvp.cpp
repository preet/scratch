#include "demomvp.h"

DemoMVP::DemoMVP(QDeclarativeItem *parent) :
    QDecViewportItem(parent)
{
    setFlag(QGraphicsItem::ItemHasNoContents, false);
    m_glProg = 0;
    m_initFailed = false;
}

void DemoMVP::initViewport()
{
    QByteArray byteArray;

    // create program object
    m_glProg = glCreateProgram();
    if(!m_glProg)   {
        qDebug() << "OpenGL: Failed to create program object";
        m_initFailed = true;
        return;
    }

    // setup vertex shader
    QString vShader = readFileAsQString(m_resPrefix + "shaders/mvp_vert.glsl");
    byteArray = vShader.toUtf8();
    GLchar * vShaderStr = byteArray.data();
    GLuint vertexShader = loadShader(GL_VERTEX_SHADER,vShaderStr);
    glAttachShader(m_glProg,vertexShader);

    // setup fragment shader
    QString fShader = readFileAsQString(m_resPrefix + "shaders/mvp_frag.glsl");
    byteArray = fShader.toUtf8();
    GLchar * fShaderStr = byteArray.data();
    GLuint fragmentShader = loadShader(GL_FRAGMENT_SHADER,fShaderStr);
    glAttachShader(m_glProg,fragmentShader);

    // bind "vPosition" attribute to generic vertex attribute index 0
    glBindAttribLocation(m_glProg,0,"vPosition");

    // link the program
    GLint progWasLinked;
    glLinkProgram(m_glProg);
    glGetProgramiv(m_glProg,GL_LINK_STATUS,&progWasLinked);
    if(!progWasLinked)   {
        GLint infoLen = 0;
        glGetProgramiv(m_glProg,GL_INFO_LOG_LENGTH,&infoLen);

        if(infoLen > 1)   {
            char infoLog[infoLen];
            glGetProgramInfoLog(m_glProg,infoLen,NULL,infoLog);
            qDebug() << "OpenGL: Error linking program: \n"
                     << QString(infoLog);
        }
        glDeleteProgram(m_glProg);
        m_initFailed = true;
        return;
    }

    // create vertex data
    m_glVertexData = new GLfloat[9];

    m_glVertexData[0] = -1.0f;
    m_glVertexData[1] = -1.0f;
    m_glVertexData[2] = 0.0f;

    m_glVertexData[3] = 1.0f;
    m_glVertexData[4] = -1.0f;
    m_glVertexData[5] = 0.0f;

    m_glVertexData[6] = 0.0f;
    m_glVertexData[7] = 1.0f;
    m_glVertexData[8] = 0.0f;

//    const GLubyte* pGPU = glGetString(GL_RENDERER);
//    const GLubyte* pVersion = glGetString(GL_VERSION);
//    const GLubyte* pShaderVersion = glGetString(GL_SHADING_LANGUAGE_VERSION);
//    qDebug() << "m_glVertexBuffer: " << m_glVertexBuffer;
//    qDebug() << "GPU: " << QString((char*)pGPU).trimmed();
//    qDebug() << "OpenGL: " << QString((char*)pVersion).trimmed();
//    qDebug() << "GLSL: " << QString((char*)pShaderVersion);

    // create vertex buffer and copy vertex data over
    glGenBuffers(1,&m_glVertexBuffer);
    glBindBuffer(GL_ARRAY_BUFFER,m_glVertexBuffer);
    glBufferData(GL_ARRAY_BUFFER,sizeof(GLfloat)*9,
                 m_glVertexData,GL_STATIC_DRAW);


    // unbind all buffer for BABY UPSET QPAINTER
    // WAHWAHWAH I LIKE TO CRASH ALL THE TIME
    glBindBuffer(GL_ARRAY_BUFFER,0);

}

void DemoMVP::drawViewport()
{
    glm::mat4 matProj = glm::perspective(45.0f,128.0f/75.0f,0.1f,100.0f);
    glm::mat4 matView = glm::lookAt(glm::vec3(2,2,2),glm::vec3(0,0,0),glm::vec3(0,1,0));
    glm::mat4 matModel = glm::mat4(1.0f);
    glm::mat4 matModelViewProj = matProj*matView*matModel;

    glUseProgram(m_glProg);

    // xf matrix uniform
    GLuint vshMatId = glGetUniformLocation(m_glProg,"xfModelViewProj");
    glUniformMatrix4fv(vshMatId,1,GL_FALSE,&matModelViewProj[0][0]);

    // following 'indices' should be index from glGetAttribLocation,
    // not m_glVertexBuffer

    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER,m_glVertexBuffer);
    glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,0,(void*)0);
    glDrawArrays(GL_TRIANGLES,0,3);
    glDisableVertexAttribArray(0);

    // unbind all buffer for BABY UPSET QPAINTER
    // WAHWAHWAH I LIKE TO CRASH ALL THE TIME
    glBindBuffer(GL_ARRAY_BUFFER,0);
}

GLuint DemoMVP::loadShader(GLenum type, const char *shaderSrc)
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
