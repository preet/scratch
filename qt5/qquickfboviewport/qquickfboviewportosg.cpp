#include "qquickfboviewportosg.h"

const char * vertex_shader =
        "// VERTEX SHADER\n"
        "\n"
        "// notes:\n"
        "// to maintain compatibility, the version\n"
        "// preprocessor call needs to be added to the\n"
        "// beginning of this file by the (cpu) compiler:\n"
        "//\n"
        "// \"#version 100\" for OpenGL ES 2 and\n"
        "// \"#version 120\" (or higher) for desktop OpenGL\n"
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
        "// varyings\n"
        "varying mediump vec4 VertexColor;\n"
        "\n"
        "void main()\n"
        "{\n"
        "   // default params\n"
        "   vec4 LightPosition = vec4(0.0, 0.0, 0.0, 1.0);\n"
        "   vec3 LightColor = vec3(1.0, 1.0, 1.0);\n"
        "   vec3 DiffuseColor = vec3(gl_Color);\n"
        "//   vec3 DiffuseColor = abs(normalize(vec3(gl_Vertex)));\n"
        "   float Alpha = gl_Color.w;\n"
        "\n"
        "   // find the vector from the given vertex to the light source\n"
        "   vec4 vertexInWorldSpace = gl_ModelViewMatrix * vec4(gl_Vertex);\n"
        "   vec3 normalInWorldSpace = normalize(gl_NormalMatrix * gl_Normal);\n"
        "   vec3 lightDirn = normalize(vec3(LightPosition-vertexInWorldSpace));\n"
        "\n"
        "   // calculate final vertex color\n"
        "   VertexColor = vec4(DiffuseColor * max(dot(lightDirn,normalInWorldSpace),0.0), Alpha);\n"
        "\n"
        "   // calculate projected vertex position\n"
        "   gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;\n"
        "}\n"
        "";

const char * frag_shader =
        "// FRAGMENT SHADER\n"
        "\n"
        "// notes:\n"
        "// to maintain compatibility, the version\n"
        "// preprocessor call needs to be added to the\n"
        "// beginning of this file by the (cpu) compiler:\n"
        "//\n"
        "// \"#version 100\" for OpenGL ES 2 and\n"
        "// \"#version 120\" (or higher) for desktop OpenGL\n"
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
        "// varyings\n"
        "varying vec4 VertexColor;\n"
        "\n"
        "// uniforms\n"
        "\n"
        "void main()\n"
        "{\n"
        "    gl_FragColor = VertexColor;\n"
        "}\n"
        "";


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


    // shaders
    QString shaderPrefix;
#ifdef DEV_DESKTOP
    shaderPrefix = "#version 120\n";
#endif
#ifdef DEV_PLAYBOOK
    shaderPrefix = "#version 100\n";
#endif

    osg::ref_ptr<osg::Program> shProgram = new osg::Program;

    QString vShader(vertex_shader);
    vShader.prepend(shaderPrefix);
    shProgram->addShader(new osg::Shader(osg::Shader::VERTEX,vShader.toStdString()));

    QString fShader(frag_shader);
    fShader.prepend(shaderPrefix);
    shProgram->addShader(new osg::Shader(osg::Shader::FRAGMENT,fShader.toStdString()));

    // geometry
    osg::ref_ptr<osg::Geometry> gmOct = new osg::Geometry;
    buildGeometryOct(gmOct.get());

    // geode
    osg::ref_ptr<osg::Geode> gdOct = new osg::Geode;
    gdOct->addDrawable(gmOct);

    // xf
    osg::ref_ptr<osg::MatrixTransform> xfOct = new osg::MatrixTransform;
    xfOct->setMatrix(osg::Matrix::identity());
    xfOct->setUpdateCallback(new RotateCB);
    xfOct->addChild(gdOct);

    // state
    osg::StateSet * ss = gdOct->getOrCreateStateSet();
    ss->setAttributeAndModes(shProgram);

    // scene
    osg::ref_ptr<osg::Group> groupRoot = new osg::Group;
    groupRoot->addChild(xfOct);

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
    m_osg_viewer->realize();

    // setup state
    m_osg_stateset = new osg::StateSet;

//    m_osg_viewer->getCamera()->getGraphicsContext()->getState()->setCheckForGLErrors(osg::State::ONCE_PER_ATTRIBUTE);

    // setup camera
//    m_osg_viewer->getCamera()->setClearColor(osg::Vec4(0,0,0,0));
//    m_osg_viewer->getCamera()->setClearMask(GL_DEPTH_BUFFER_BIT);
}

void QSGFBONodeOSG::buildGeometryOct(osg::Geometry * gmNode)
{
    double ppushsz = 1.0/2;
    double npushsz = ppushsz*-1.0;
    osg::ref_ptr<osg::Vec3Array> gmListVx = new osg::Vec3Array;
    gmListVx->push_back(osg::Vec3(ppushsz,0,0));     // 0 +x
    gmListVx->push_back(osg::Vec3(0,ppushsz,0));     // 1 +y
    gmListVx->push_back(osg::Vec3(0,0,ppushsz));     // 2 +z
    gmListVx->push_back(osg::Vec3(npushsz,0,0));     // 3 -x
    gmListVx->push_back(osg::Vec3(0,npushsz,0));     // 4 -y
    gmListVx->push_back(osg::Vec3(0,0,npushsz));     // 5 -z

    osg::ref_ptr<osg::Vec3Array> gmListNx = new osg::Vec3Array;
    for(size_t i=0; i < gmListVx->size(); i++)   {
        osg::Vec3 nx = gmListVx->at(i);
        nx.normalize();
        gmListNx->push_back(nx);
    }

    osg::ref_ptr<osg::DrawElementsUByte> gmListIx =
            new osg::DrawElementsUByte(GL_TRIANGLES);
    gmListIx->push_back(0);
    gmListIx->push_back(1);
    gmListIx->push_back(2);

    gmListIx->push_back(1);
    gmListIx->push_back(3);
    gmListIx->push_back(2);

    gmListIx->push_back(3);
    gmListIx->push_back(4);
    gmListIx->push_back(2);

    gmListIx->push_back(4);
    gmListIx->push_back(0);
    gmListIx->push_back(2);

    gmListIx->push_back(1);
    gmListIx->push_back(0);
    gmListIx->push_back(5);

    gmListIx->push_back(3);
    gmListIx->push_back(1);
    gmListIx->push_back(5);

    gmListIx->push_back(4);
    gmListIx->push_back(3);
    gmListIx->push_back(5);

    gmListIx->push_back(0);
    gmListIx->push_back(4);
    gmListIx->push_back(5);

    osg::ref_ptr<osg::Vec4Array> gmListCx = new osg::Vec4Array;
    gmListCx->push_back(osg::Vec4(1,0,0,1));
    gmListCx->push_back(osg::Vec4(0,1,0,1));
    gmListCx->push_back(osg::Vec4(0,0,1,1));
    gmListCx->push_back(osg::Vec4(1,1,0,1));
    gmListCx->push_back(osg::Vec4(0,1,1,1));
    gmListCx->push_back(osg::Vec4(1,0,1,1));

    // add data to geometry node
    gmNode->setVertexArray(gmListVx);

    // pre osg-3.1.8
    gmNode->setNormalArray(gmListNx);
    gmNode->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);
    gmNode->setColorArray(gmListCx);
    gmNode->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

    // osg-3.1.8
//    gmOct->setNormalArray(gmListNx,osg::Array::BIND_PER_VERTEX);
//    gmOct->setColorArray(gmListCx,osg::Array::BIND_PER_VERTEX);
    gmNode->addPrimitiveSet(gmListIx);
}

void QSGFBONodeOSG::buildGeometrySphere(osg::Geometry *gmNode)
{

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
