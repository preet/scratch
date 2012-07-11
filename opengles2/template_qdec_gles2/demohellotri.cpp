#include "demohellotri.h"

DemoHelloTri::DemoHelloTri(QDeclarativeItem *parent) :
    QDecViewportItem(parent)
{
    setFlag(QGraphicsItem::ItemHasNoContents, false);
    m_glProg = 0;
    m_initFailed = false;
}

void DemoHelloTri::initViewport()
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
    QString vShader = readFileAsQString(m_resPrefix + "shaders/hellotri_vert.glsl");
    byteArray = vShader.toUtf8();
    GLchar * vShaderStr = byteArray.data();
    GLuint vertexShader = loadShader(GL_VERTEX_SHADER,vShaderStr);
    glAttachShader(m_glProg,vertexShader);

    // setup fragment shader
    QString fShader = readFileAsQString(m_resPrefix + "shaders/hellotri_frag.glsl");
    byteArray = fShader.toUtf8();
    GLchar * fShaderStr = byteArray.data();
    GLuint fragmentShader = loadShader(GL_FRAGMENT_SHADER,fShaderStr);
    glAttachShader(m_glProg,fragmentShader);

    // bind vPosition (vector position) to attribute 0
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
}

void DemoHelloTri::drawViewport()
{
    GLfloat vVertices[] =
    { 0.0f, 0.5f, 0.0f, -0.5f, -0.5f, 0.0f, 0.5f, -0.5f, 0.0f };

    glUseProgram(m_glProg);
    glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,0,vVertices);
    glEnableVertexAttribArray(0);
    glDrawArrays(GL_TRIANGLES,0,3);
}

GLuint DemoHelloTri::loadShader(GLenum type, const char *shaderSrc)
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
