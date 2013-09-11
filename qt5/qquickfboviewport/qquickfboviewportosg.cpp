#include "qquickfboviewportosg.h"

#ifdef ENV_DEV
const char * path_osg_logo = "/home/preet/Dev/scratch/qt5/qquickfboviewport/res/osglogo.png";
const char * path_qt_logo = "/home/preet/Dev/scratch/qt5/qquickfboviewport/res/qtlogo.jpg";
const char * path_font = "/home/preet/Dev/scratch/qt5/qquickfboviewport/res/DroidSans-Bold.ttf";
#endif

#ifdef ENV_ANDROID
const char * path_osg_logo = "/storage/emulated/0/stuff/osglogo.png";
const char * path_qt_logo = "/storage/emulated/0/stuff/qtlogo.jpg";
const char * path_font = "/storage/emulated/0/stuff/DroidSans-Bold.ttf";
#endif

// http://tomeko.net/online_tools/cpp_text_escape.php?lang=en
const char * vertex_shader_tex =
        "// VERTEX SHADER\n"
        "\n"
        "#ifdef GL_ES\n"
        "    // vertex shader defaults for types are:\n"
        "    // precision highp float;\n"
        "    // precision highp int;\n"
        "    // precision lowp sampler2D;\n"
        "    // precision lowp samplerCube;\n"
        "#else\n"
        "    // with default (non ES) OpenGL shaders, precision\n"
        "    // qualifiers aren't used -- we explicitly set them\n"
        "    // to be defined as 'nothing' so they are ignored\n"
        "    #define lowp\n"
        "    #define mediump\n"
        "    #define highp\n"
        "#endif\n"
        "\n"
        "varying mediump vec2 TexCoord0;\n"
        "\n"
        "void main()\n"
        "{\n"
        "    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;\n"
        "    TexCoord0 = gl_MultiTexCoord0.xy;\n"
        "}\n"
        "";

const char * frag_shader_tex =
        "// FRAGMENT SHADER\n"
        "\n"
        "#ifdef GL_ES\n"
        "    // the fragment shader in ES 2 doesn't have a\n"
        "    // default precision qualifier for floats so\n"
        "    // it needs to be explicitly specified\n"
        "    precision mediump float;\n"
        "\n"
        "    // note: highp may not be available for float types in\n"
        "    // the fragment shader -- use the following to set it:\n"
        "    // #ifdef GL_FRAGMENT_PRECISION_HIGH\n"
        "    // precision highp float;\n"
        "    // #else\n"
        "    // precision mediump float;\n"
        "    // #endif\n"
        "\n"
        "    // fragment shader defaults for other types are:\n"
        "    // precision mediump int;\n"
        "    // precision lowp sampler2D;\n"
        "    // precision lowp samplerCube;\n"
        "#else\n"
        "    // with default (non ES) OpenGL shaders, precision\n"
        "    // qualifiers aren't used -- we explicitly set them\n"
        "    // to be defined as 'nothing' so they are ignored\n"
        "    #define lowp\n"
        "    #define mediump\n"
        "    #define highp\n"
        "#endif\n"
        "\n"
        "varying vec2 TexCoord0;\n"
        "\n"
        "// set to zero by default\n"
        "uniform sampler2D Texture;\n"
        "\n"
        "void main()\n"
        "{\n"
        "    gl_FragColor = vec4(texture2D(Texture,TexCoord0));\n"
        "    //gl_FragColor = vec4(1.0,1.0,1.0,1.0);\n"
        "}\n"
        "";

const char * vertex_shader_text =
        "#ifdef GL_ES\n"
        "    // vertex shader defaults for types are:\n"
        "    // precision highp float;\n"
        "    // precision highp int;\n"
        "    // precision lowp sampler2D;\n"
        "    // precision lowp samplerCube;\n"
        "#else\n"
        "    // with default (non ES) OpenGL shaders, precision\n"
        "    // qualifiers aren't used -- we explicitly set them\n"
        "    // to be defined as 'nothing' so they are ignored\n"
        "    #define lowp\n"
        "    #define mediump\n"
        "    #define highp\n"
        "#endif\n"
        "\n"
        "// varyings\n"
        "varying mediump vec4 VertexColor;\n"
        "varying mediump vec2 TexCoord0;\n"
        "\n"
        "// uniforms\n"
        "uniform vec4 Color;\n"
        "\n"
        "void main()\n"
        "{\n"
        "    VertexColor = Color;\n"
        "    TexCoord0 = gl_MultiTexCoord0.xy;\n"
        "    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;\n"
        "}";

const char * frag_shader_text =
        "#ifdef GL_ES\n"
        "    // the fragment shader in ES 2 doesn't have a\n"
        "    // default precision qualifier for floats so\n"
        "    // it needs to be explicitly specified\n"
        "    precision mediump float;\n"
        "\n"
        "    // note: highp may not be available for float types in\n"
        "    // the fragment shader -- use the following to set it:\n"
        "    // #ifdef GL_FRAGMENT_PRECISION_HIGH\n"
        "    // precision highp float;\n"
        "    // #else\n"
        "    // precision mediump float;\n"
        "    // #endif\n"
        "\n"
        "    // fragment shader defaults for other types are:\n"
        "    // precision mediump int;\n"
        "    // precision lowp sampler2D;\n"
        "    // precision lowp samplerCube;\n"
        "#else\n"
        "    // with default (non ES) OpenGL shaders, precision\n"
        "    // qualifiers aren't used -- we explicitly set them\n"
        "    // to be defined as 'nothing' so they are ignored\n"
        "    #define lowp\n"
        "    #define mediump\n"
        "    #define highp\n"
        "#endif\n"
        "\n"
        "// varyings\n"
        "varying vec4 VertexColor;\n"
        "varying vec2 TexCoord0;\n"
        "\n"
        "// uniforms\n"
        "uniform sampler2D GlyphTexture;\n"
        "\n"
        "void main()\n"
        "{\n"
        "    gl_FragColor = VertexColor * texture2D(GlyphTexture,TexCoord0).aaaa;\n"
        "}";




class RotateCB : public osg::NodeCallback
{
public:
    RotateCB() : m_angle(0)
    {}

    virtual void operator()(osg::Node * node, osg::NodeVisitor * nv)
    {
        osg::Matrix xf;
        xf.makeRotate(m_angle,osg::Vec3(0,0,1));

        osg::MatrixTransform * xfNode =
                dynamic_cast<osg::MatrixTransform*>(node);
        xfNode->setMatrix(xf);

        m_angle+=0.1;
        if(m_angle > 2*3.141592)   {
            m_angle = 0;
        }
    }

private:
    double m_angle;
};


QSGFBONodeOSG::QSGFBONodeOSG(QQuickWindow * window) :
    m_fbo(NULL),
    m_texture(NULL),
    m_window(window)
{
    // The GL context used for rendering the scenegraph
    // is bound when the beforeRendering() signal is sent,
    // so we can render the OSG scene to our FBO.
    connect(m_window,SIGNAL(beforeRendering()),
            this,SLOT(onRenderFBO()),
            Qt::DirectConnection);
}

QSGFBONodeOSG::~QSGFBONodeOSG()
{
    delete m_texture;
    delete m_fbo;
}

void QSGFBONodeOSG::onRenderFBO()
{
    QSize size = rect().size().toSize();

    // Create the FBO if it doesn't exist
    if(!m_fbo)   {
        this->initNode();
    }

    m_render_frame_mutex.lock();
    if(m_render_frame)   {
        m_render_frame = false;

        // Bind rendering to the SG Node FBO
        m_fbo->bind();

        // Render the frame
        this->renderFrame();

        // Release rendering back to qt
        m_fbo->release();
    }
    m_render_frame_mutex.unlock();

    // Don't need this as we call makeDirty()
    // in updatePaintNode()
    //
    // m_window->update();
}

void QSGFBONodeOSG::onQueueFrame()
{
    // Don't queue any frame updates unless we
    // are done rendering the previous frame
    if(m_render_frame_mutex.tryLock())   {
        m_render_frame = true;
        m_render_frame_mutex.unlock();
    }
}

void QSGFBONodeOSG::renderFrame()
{
    m_osg_viewer->getCamera()->getGraphicsContext()->getState()->reset();
    m_osg_viewer->getCamera()->getGraphicsContext()->getState()->apply(m_osg_stateset);
    m_osg_viewer->frame();
    m_osg_viewer->getCamera()->getGraphicsContext()->getState()->captureCurrentState(*m_osg_stateset);

    // dunno about this one
    glPixelStorei(GL_UNPACK_ALIGNMENT,4);

    // qt passes values to VertexAttrib 3,4,5 manually
    // so we should disable them when we use em
    glDisableVertexAttribArray(0);
    glDisableVertexAttribArray(1);
    glDisableVertexAttribArray(2);
    glDisableVertexAttribArray(3);
    glDisableVertexAttribArray(4);
    glDisableVertexAttribArray(5);

    // GL_DEPTH_TEST needs to be enabled when we render with
    // openscenegraph, but it must be disabled before passing
    // things back to QPainter
    glDisable(GL_DEPTH_TEST);
//    glDisable(GL_CULL_FACE);
    glBindTexture(GL_TEXTURE_2D,0);
}

void QSGFBONodeOSG::initNode()
{
    QSize size = rect().size().toSize();
    QOpenGLFramebufferObjectFormat format;
    format.setAttachment(QOpenGLFramebufferObject::CombinedDepthStencil);
    m_fbo = new QOpenGLFramebufferObject(size, format);
    m_texture = m_window->createTextureFromId(m_fbo->texture(), size);

    // init renderer here
    this->initOSG();

    this->setTexture(m_texture);
}

void QSGFBONodeOSG::initOSG()
{
    QSize size = rect().size().toSize();


////    osgDB::Registry::instance()->setLibraryFilePathList("/home/preet/Dev/env/sys/osg-3.1.8/lib64/somepath");
//    osgDB::Registry::instance()->setLibraryFilePathList("/data/app-lib/org.qtproject.example.qquickfboviewport-1");


//    osgDB::Registry::instance()->setLibraryFilePathList(
//                "/storage/emulated/0/stuff");

//    QString path_app = QCoreApplication::applicationDirPath() + "/";
//    QDir appdir(path_app);
//    QStringList listFiles = appdir.entryList();

//    if(listFiles.size() > 0)   {
//        for(int i=0; i < listFiles.size(); i++)   {
//            qDebug() << "##: " << listFiles[i];
//        }
//    }
//    else   {
//        qDebug() << "##: Nope!";
//    }


//    osgDB::Registry::instance()->setLibraryFilePathList(".");

//    osgDB::FilePathList filePathList
//            = osgDB::Registry::instance()->getLibraryFilePathList();

//    for(size_t i=0; i < filePathList.size(); i++)    {
//        qDebug() << "#: " << QString::fromStdString(filePathList[i]);
//    }

//    osg::setNotifyLevel(osg::INFO);
//    osg::setNotifyHandler(new DebugRedirect());

    // shaders
    QString shaderPrefix;
#ifdef ENV_GL
    shaderPrefix = "#version 120\n";
#endif
#ifdef ENV_GLES2
    shaderPrefix = "#version 100\n";
#endif

    // model shader
    osg::ref_ptr<osg::Program> shProgram = new osg::Program;

    QString vShader(vertex_shader_tex);
    vShader.prepend(shaderPrefix);
    shProgram->addShader(new osg::Shader(osg::Shader::VERTEX,vShader.toStdString()));

    QString fShader(frag_shader_tex);
    fShader.prepend(shaderPrefix);
    shProgram->addShader(new osg::Shader(osg::Shader::FRAGMENT,fShader.toStdString()));

    // text shader
    osg::ref_ptr<osg::Program> shText = new osg::Program;
    vShader = QString(vertex_shader_text);
    vShader.prepend(shaderPrefix);
    fShader = QString(frag_shader_text);
    fShader.prepend(shaderPrefix);
    shText->addShader(new osg::Shader(osg::Shader::VERTEX,vShader.toStdString()));
    shText->addShader(new osg::Shader(osg::Shader::FRAGMENT,fShader.toStdString()));

    // scene
    osg::ref_ptr<osg::Group> groupRoot = new osg::Group;
    groupRoot->getOrCreateStateSet()->setMode(GL_BLEND,osg::StateAttribute::ON);

    // image
    osg::ref_ptr<osg::Image> imgOsgLogo = new osg::Image;
    imgOsgLogo = osgDB::readImageFile(path_osg_logo);
    bool imgOk = imgOsgLogo.valid() && imgOsgLogo->valid();
    if(!imgOk)   {
        qDebug() << "ERROR: Image Invalid!";
    }
    else   {
        // texture
        osg::ref_ptr<osg::Texture2D> txOsgLogo = new osg::Texture2D;
        txOsgLogo->setImage(imgOsgLogo);
        txOsgLogo->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR);
        txOsgLogo->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);
        txOsgLogo->setWrap(osg::Texture2D::WRAP_S,osg::Texture2D::CLAMP_TO_EDGE);
        txOsgLogo->setWrap(osg::Texture2D::WRAP_T,osg::Texture2D::CLAMP_TO_EDGE);

        // geometry
        osg::ref_ptr<osg::Geometry> gmPlane = new osg::Geometry;
        buildGeometryPlane(gmPlane.get());

        // geode
        osg::ref_ptr<osg::Geode> gdPlane = new osg::Geode;
        gdPlane->addDrawable(gmPlane);

        // xf
        osg::ref_ptr<osg::MatrixTransform> xfPlane = new osg::MatrixTransform;
        xfPlane->setMatrix(osg::Matrix::identity());
        xfPlane->setUpdateCallback(new RotateCB);
        xfPlane->addChild(gdPlane);

        // state
        osg::ref_ptr<osg::Uniform> uTexSampler =
                new osg::Uniform("Texture",0);

        osg::StateSet * ss = gdPlane->getOrCreateStateSet();
        ss->setAttributeAndModes(shProgram);
        ss->addUniform(uTexSampler);
        #ifdef ENV_GLES2
        ss->setTextureAttributeAndModes(0,txOsgLogo);
        #endif

        groupRoot->addChild(xfPlane);
    }

    // text
    {
        QString textMessage("Qt+OSG on \nAndroid!");

        // text
        osg::ref_ptr<osgText::Text> gmText = new osgText::Text;
        gmText->setFont(path_font);
        gmText->setCharacterSize(0.25f);
        gmText->setPosition(osg::Vec3(0,0,-1.5));
        gmText->setAlignment(osgText::TextBase::CENTER_CENTER);
        gmText->setAxisAlignment(osgText::TextBase::SCREEN);
        gmText->setText(textMessage.toStdString());

        // geode
        osg::ref_ptr<osg::Geode> gdText = new osg::Geode;
        gdText->addDrawable(gmText);

        // state
        osg::ref_ptr<osg::Uniform> textColor =
                new osg::Uniform("Color",osg::Vec4(0.37,0.77,0.35,1));

        osg::ref_ptr<osg::Uniform> textTexture =
                new osg::Uniform("GlyphTexture",0);

        osg::StateSet * ss = gdText->getOrCreateStateSet();
        ss->addUniform(textColor);
        ss->addUniform(textTexture);
        ss->setAttributeAndModes(shText);

        groupRoot->addChild(gdText);
    }

    // create viewer and embedded window
    m_osg_viewer = new osgViewer::Viewer;
    m_osg_window = m_osg_viewer->setUpViewerAsEmbeddedInWindow(0,0,size.width(),size.height());

    // tell osg to insert uniforms and attributes in shaders
    m_osg_window->getState()->setUseModelViewAndProjectionUniforms(true);
    m_osg_window->getState()->setUseVertexAttributeAliasing(true);

    // config viewer
    m_osg_viewer->setCameraManipulator(new osgGA::TrackballManipulator);
    m_osg_viewer->setThreadingModel(osgViewer::Viewer::SingleThreaded);
    m_osg_viewer->setSceneData(groupRoot);
    m_osg_viewer->getCamera()->setClearColor(osg::Vec4(0.17,0.17,0.17,1.0));
    m_osg_viewer->realize();

    // setup state
    m_osg_stateset = new osg::StateSet;

    m_osg_viewer->getCamera()->getGraphicsContext()->getState()->setCheckForGLErrors(osg::State::ONCE_PER_ATTRIBUTE);

    // setup camera
//    m_osg_viewer->getCamera()->setClearColor(osg::Vec4(0,0,0,0));
//    m_osg_viewer->getCamera()->setClearMask(GL_DEPTH_BUFFER_BIT);
}

void QSGFBONodeOSG::buildGeometryPlane(osg::Geometry * gmNode)
{
    osg::ref_ptr<osg::Vec3Array> gmListVx = new osg::Vec3Array;
    gmListVx->push_back(osg::Vec3(-1,0,1));  // top left
    gmListVx->push_back(osg::Vec3(-1,0,-1)); // bottom left
    gmListVx->push_back(osg::Vec3(1,0,-1));  // bottom right
    gmListVx->push_back(osg::Vec3(1,0,1));   // top right

    osg::ref_ptr<osg::Vec2Array> gmListTx = new osg::Vec2Array;
    gmListTx->push_back(osg::Vec2(0,1));     // top left
    gmListTx->push_back(osg::Vec2(0,0));     // bottom left
    gmListTx->push_back(osg::Vec2(1,0));     // bottom right
    gmListTx->push_back(osg::Vec2(1,1));     // top right

    osg::ref_ptr<osg::DrawElementsUByte> gmListIx =
            new osg::DrawElementsUByte(GL_TRIANGLES,6);
    gmListIx->at(0) = 0;
    gmListIx->at(1) = 1;
    gmListIx->at(2) = 2;

    gmListIx->at(3) = 0;
    gmListIx->at(4) = 2;
    gmListIx->at(5) = 3;

    gmNode->setVertexArray(gmListVx);
    gmNode->setTexCoordArray(0,gmListTx);
    gmNode->addPrimitiveSet(gmListIx);

//    // pre osg-3.1.8
//    gmNode->setNormalArray(gmListNx);
//    gmNode->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);
//    gmNode->setColorArray(gmListCx);
//    gmNode->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

    // osg-3.1.8
//    gmOct->setNormalArray(gmListNx,osg::Array::BIND_PER_VERTEX);
//    gmOct->setColorArray(gmListCx,osg::Array::BIND_PER_VERTEX);
//    gmNode->addPrimitiveSet(gmListIx);
}


// ============================================================== //
// ============================================================== //

QQuickFBOViewportOSG::QQuickFBOViewportOSG()
{
    // The ItemHasContents flag must be set to true
    // for update() to trigger updatePaintNode()
    setFlag(QQuickItem::ItemHasContents,true);

    // We can't directly control the rate at which the
    // (qt) scenegraph renders frames -- other nodes may
    // be updated at arbitrary intervals based on their
    // state (ie are they dirty,etc)

    // We *can* force the scene graph to render frames
    // at a minimum rate by marking this QQuickItem's
    // node dirty with a timer.
    connect(&m_timer,SIGNAL(timeout()),
            this,SLOT(onRenderFrame()));

    m_timer.start(48);
}

QQuickFBOViewportOSG::~QQuickFBOViewportOSG()
{}

void QQuickFBOViewportOSG::onRenderFrame()
{
    // The queueFrame will emit a signal from this QQuickItem
    // to its child QSGFBONodeOSG. The node will set internal
    // state forcing a render to the FBO when Qt next decides
    // to render by listening to QQuickWindow::beforeRendering()
    emit queueFrame();

    // We call update here to invoke updatePaintNode() and
    // set the QSGFBONodeOSG's state to dirty.
    // This will make Qt render a new frame.
    this->update();
}

QSGNode * QQuickFBOViewportOSG::updatePaintNode(QSGNode * oldNode,
                                                UpdatePaintNodeData * updData)
{
    QSGFBONodeOSG * fboNode = static_cast<QSGFBONodeOSG*>(oldNode);
    if(!fboNode)   {
        fboNode = new QSGFBONodeOSG(window());

        // Note: We want the main thread (the thread that owns this
        //       QQuickItem) to execute QSGFBONodeOSG::onQueueFrame().
        connect(this,SIGNAL(queueFrame()),
                fboNode,SLOT(onQueueFrame()),
                Qt::DirectConnection);
    }
    // Handle resize
    fboNode->setRect(boundingRect());

    // Mark the node as dirty to force Qt's scenegraph to
    // render a new frame (and thus update the FBO)
    fboNode->markDirty(QSGNode::DirtyMaterial);

    return fboNode;
}
