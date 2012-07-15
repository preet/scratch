#include "viewport.h"

Viewport::Viewport(QWidget *parent) :
    QGLWidget(parent)
{}

QSize Viewport::sizeHint() const
{   return QSize(800,480);   }

void Viewport::onLoadMap(const QString &mapPath, const QString &stylePath)
{
    // this should be called after initializeGL
    qDebug() << "Loading Map...";

//    // load database
//    m_databaseParam = new osmscout::DatabaseParameter;
//    m_database = new osmscout::Database(*m_databaseParam);
//    if(m_database->Open(mapPath.toStdString()))
//    {   qDebug() << "INFO: Opened Database Successfully";   }
//    else
//    {   qDebug() << "ERROR: Could not open database";   }

//    // load style data
//    osmscout::RenderStyleConfigReader styleConfigReader(stylePath.toStdString(),
//                                                        m_database->GetTypeConfig(),
//                                                        m_listStyleConfigs);
//    if(styleConfigReader.HasErrors())
//    {   qDebug() << "ERROR: Could not read style config";   }
//    else
//    {   qDebug() << "INFO: Opened Style Configs successfully";   }

//    // load map renderer
//    m_mapRenderer = new osmscout::MapRendererOSG(m_database);
////    m_mapRenderer = new osmscout::MapRendererOSG(m_database,m_osg_viewer);
//    m_mapRenderer->SetRenderStyleConfigs(m_listStyleConfigs);

//    // init scene
//    osmscout::PointLLA scene1(43.66,-79.377,500);
//    m_mapRenderer->InitializeScene(scene1,osmscout::CAM_2D);

//    // setup initial camera position
//    osmscout::Camera const * myCamera = m_mapRenderer->GetCamera();
//    m_osg_viewer->getCamera()->setClearColor(osg::Vec4(0.1,0.1,0.1,1.0));
//    m_osg_viewer->getCameraManipulator()->setHomePosition(osg::Vec3(myCamera->eye.x,
//                                                                    myCamera->eye.y,
//                                                                    myCamera->eye.z),
//                                                          osg::Vec3(myCamera->viewPt.x,
//                                                                    myCamera->viewPt.y,
//                                                                    myCamera->viewPt.z),
//                                                          osg::Vec3(myCamera->up.x,
//                                                                    myCamera->up.y,
//                                                                    myCamera->up.z));
//    m_osg_viewer->getCameraManipulator()->home(0);
//    m_osg_viewer->getCamera()->setComputeNearFarMode(osgUtil::CullVisitor::DO_NOT_COMPUTE_NEAR_FAR);
//    m_osg_viewer->getCamera()->setProjectionMatrixAsPerspective(30,1.67,
//                                                                myCamera->nearDist,
//                                                                myCamera->farDist);
//    m_osg_viewer->realize();
//    updateGL();
}

void Viewport::initializeGL()
{
    // setup openscenegraph viewport
    m_osg_viewer = new osgViewer::Viewer;
    m_osg_viewer->setThreadingModel(osgViewer::Viewer::SingleThreaded);
    m_osg_window = m_osg_viewer->setUpViewerAsEmbeddedInWindow(0,0,this->width(),this->height());
    m_osg_viewer->setCameraManipulator(new osgGA::TrackballManipulator);
    m_osg_viewer->realize();
}

void Viewport::paintGL()
{   m_osg_viewer->frame();   }

void Viewport::mousePressEvent(QMouseEvent *event)
{
    qDebug() << "Press";
    int nx = event->pos().x();
    int ny = event->pos().y();
    int button = event->button();

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

void Viewport::mouseMoveEvent(QMouseEvent *event)
{
    qDebug() << "Move";
    int nx = event->pos().x();
    int ny = event->pos().y();
    int button = event->button();

    // add event to osg manipulator
    Q_UNUSED(button);
    m_osg_window->getEventQueue()->mouseMotion(nx,ny);
}

void Viewport::mouseReleaseEvent(QMouseEvent *event)
{
    qDebug() << "Release";
    int nx = event->pos().x();
    int ny = event->pos().y();
    int button = event->button();

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
