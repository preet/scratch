#include "qdecviewportosg.h"

QDecViewportOSG::QDecViewportOSG(QDeclarativeItem *parent) :
    QDecViewportItem(parent)
{
    #ifdef DEV_PLAYBOOK
        m_shaderPrefix = "#version 100\n";
    #endif

    #ifdef DEV_PC
        m_shaderPrefix = "#version 120\n";
    #endif

}

void QDecViewportOSG::onMousePressed(int nx, int ny, int button)
{
    // osgGA mouse button numbering:
    // 1 for left, 2 for middle, 3 for right
    unsigned int osgMouseBtn = 0;
    switch(button)
    {
    case Qt::LeftButton:
        osgMouseBtn = 1;
        break;
    case Qt::MiddleButton:
        osgMouseBtn = 2;
        break;
    case Qt::RightButton:
        osgMouseBtn = 3;
        break;
    default:
        break;
    }

    // add event to osg manipulator
    m_osg_window->getEventQueue()->mouseButtonPress(nx,ny,osgMouseBtn);
}

void QDecViewportOSG::onMouseMoved(int nx, int ny, int button)
{
    // add event to osg manipulator
    Q_UNUSED(button);
    m_osg_window->getEventQueue()->mouseMotion(nx,ny);
}

void QDecViewportOSG::onMouseReleased(int nx, int ny, int button)
{
    // osgGA mouse button numbering:
    // 1 for left, 2 for middle, 3 for right
    unsigned int osgMouseBtn = 0;
    switch(button)
    {
    case Qt::LeftButton:
        osgMouseBtn = 1;
        break;
    case Qt::MiddleButton:
        osgMouseBtn = 2;
        break;
    case Qt::RightButton:
        osgMouseBtn = 3;
        break;
    default:
        break;
    }

    // add event to osg manipulator
    m_osg_window->getEventQueue()->mouseButtonRelease(nx,ny,osgMouseBtn);
}

void QDecViewportOSG::initViewport()
{
    // set debug severity
    osg::setNotifyLevel(osg::INFO);

    // text geometry
    std::string textStr("Hello World");
    osg::ref_ptr<osgText::Text> geomText = new osgText::Text;
    geomText->setFont("DroidSans-Bold.ttf");
    geomText->setCharacterSize(20.0f);
    geomText->setAxisAlignment(osgText::TextBase::SCREEN);
    geomText->setText(textStr);

    // text node
    osg::ref_ptr<osg::Geode> geodeText = new osg::Geode;
    geodeText->addDrawable(geomText);

    // setup shaders
    osg::ref_ptr<osg::Program> shProgram = new osg::Program;
    shProgram->setName("TextShader");

    QString vShader = readFileAsQString(m_resPrefix + "shaders/Text_vert.glsl");
    vShader.prepend(m_shaderPrefix);
    shProgram->addShader(new osg::Shader(osg::Shader::VERTEX,vShader.toStdString()));

    QString fShader = readFileAsQString(m_resPrefix + "shaders/Text_frag.glsl");
    vShader.prepend(m_shaderPrefix);
    shProgram->addShader(new osg::Shader(osg::Shader::FRAGMENT,fShader.toStdString()));

    osg::ref_ptr<osg::Uniform> myColor = new osg::Uniform("Color",osg::Vec4(0,1,1,1));
    osg::ref_ptr<osg::Uniform> myTexture = new osg::Uniform("GlyphTexture",0);

    // enable shader program
    osg::StateSet *ss = geodeText->getOrCreateStateSet();
    ss->addUniform(myColor);
    ss->addUniform(myTexture);
    ss->setAttributeAndModes(shProgram,osg::StateAttribute::ON);
    ss->setMode(GL_BLEND,osg::StateAttribute::ON);

    // add stuff to scene
    osg::ref_ptr<osg::Group> groupRoot = new osg::Group;
    groupRoot->addChild(geodeText);

    // create viewer and embedded window
    m_osg_viewer = new osgViewer::Viewer;
    m_osg_window = m_osg_viewer->setUpViewerAsEmbeddedInWindow(0,0,this->width(),this->height());

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
    m_osg_viewer->getCamera()->setClearColor(osg::Vec4(0,0,0,0));
    m_osg_viewer->getCamera()->setClearMask(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);



}

void QDecViewportOSG::drawViewport()
{
    m_osg_viewer->getCamera()->getGraphicsContext()->getState()->reset();
    m_osg_viewer->getCamera()->getGraphicsContext()->getState()->apply(m_osg_stateset);
    m_osg_viewer->frame();
    m_osg_viewer->getCamera()->getGraphicsContext()->getState()->captureCurrentState(*m_osg_stateset);

    // dunno about this one
    glPixelStorei(GL_UNPACK_ALIGNMENT,4);

    // qt passes values to VertexAttrib 3,4,5 manually
    // so we should disable them when we use em
    glDisableVertexAttribArray(3);
    glDisableVertexAttribArray(4);
    glDisableVertexAttribArray(5);

    // GL_DEPTH_TEST needs to be enabled when we render with
    // openscenegraph, but it must be disabled before passing
    // things back to QPainter
    glDisable(GL_DEPTH_TEST);

    //glDisable(GL_CULL_FACE);

//    glDisableVertexAttribArray(4);
//    glDisableVertexAttribArray(5);
//    m_osg_viewer->getCamera()->getGraphicsContext()->getState()->print(std::cout);
}


