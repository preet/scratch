#include "qdecviewportvislib.h"

QDecViewportVisLib::QDecViewportVisLib(QDeclarativeItem *parent) :
    QDecViewportItem(parent)
{
    #ifdef DEV_PLAYBOOK
        m_shaderPrefix = "#version 100\n";
    #endif

    #ifdef DEV_PC
        m_shaderPrefix = "#version 120\n";
    #endif
}

void QDecViewportVisLib::initViewport()
{
    // initialize gl
    vl::VisualizationLibrary::init(true);
    vl::globalSettings()->setCheckOpenGLStates(false);
    vl::globalSettings()->setVerbosityLevel(vl::VEL_VERBOSITY_DEBUG);
    vl::initializeOpenGL();

    // set context size
    m_vl_glContext.initGLContext(true);
    m_vl_glContext.setContinuousUpdate(false);
    m_vl_glContext.framebuffer()->setWidth(this->width());
    m_vl_glContext.framebuffer()->setHeight(this->height());

    // init scene
    m_vl_rendering = new vl::Rendering;
    m_vl_sceneMan = new vl::SceneManagerActorTree;
    m_vl_rendering->sceneManagers()->push_back(m_vl_sceneMan.get());

    // [setup the scene]

    // create an xform and bind it to the scene
    m_vl_xform = new vl::Transform;
    m_vl_rendering->transform()->addChild(m_vl_xform.get());

    // teapot geometry
    vl::ref<vl::Geometry> geomTeapot = vl::makeTeapot(vl::vec3(0,0,0),15);
    geomTeapot->computeNormals();

    // teapot effect
    vl::ref<vl::Effect> fxTeapot = new vl::Effect;
    fxTeapot->shader()->enable(vl::EN_DEPTH_TEST);
    fxTeapot->shader()->setRenderState(new vl::Light,0);
    fxTeapot->shader()->enable(vl::EN_LIGHTING);
    fxTeapot->shader()->gocMaterial()->setDiffuse(vl::royalblue);
    fxTeapot->shader()->gocMaterial()->setSpecular(vl::white);
    fxTeapot->shader()->gocMaterial()->setShininess(60);

    // add teapot to the scene
    m_vl_sceneMan->tree()->addActor(geomTeapot.get(),fxTeapot.get(),m_vl_xform.get());


    // [setup rendering, camera and viewport]

    // set render target
    m_vl_rendering->renderer()->setFramebuffer(m_vl_glContext.framebuffer());

    // setup camera
    m_vl_rendering->camera()->viewport()->set(0,0,this->width(),this->height());
    m_vl_rendering->camera()->viewport()->setClearColor(vl::white);

    vl::vec3 eye(0,10,20);
    vl::vec3 center(0,0,0);
    vl::vec3 up(0,1,0);
    vl::mat4 xf_view = vl::mat4::getLookAt(eye,center,up);
    m_vl_rendering->camera()->setViewMatrix(xf_view);
}

void QDecViewportVisLib::drawViewport()
{
    m_vl_rendering->render();
}


