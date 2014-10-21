#include <iostream>
#include <iomanip>

#include <OSGUtils.h>
#include <TileSetLLByPixelArea.h>
#include <DataSetTilesLL.h>
#include <osg/MatrixTransform>

osg::Vec4 const K_BLUE = osg::Vec4(0,0.35,0.75,1.0);
osg::Vec4 const K_GREEN = osg::Vec4(0.18,0.71,0.227,1.0);
osg::Vec4 const K_YELLOW = osg::Vec4(0.96,0.86,0.08,1.0);

int main()
{
    std::cout << std::fixed;
    std::cout << std::setprecision(10);

    // View0 root
    osg::ref_ptr<osg::Group> gp_root0 = new osg::Group;

    // View1 root
    osg::ref_ptr<osg::Group> gp_root1 = new osg::Group;

    // =============================================================== //

    // axes
    auto gp_axes = BuildAxesGeometry("axes",100000.0);
    osg::ref_ptr<osg::MatrixTransform> xf = new osg::MatrixTransform;
    xf->setMatrix(osg::Matrixd::translate(osg::Vec3d(4828360.3862586031,-4168117.6413568845,-13474.2768839712)));
    xf->addChild(gp_axes);
    gp_root0->addChild(xf);
    gp_root1->addChild(xf);

    // =============================================================== //

    // disable lighting and enable blending
    gp_root0->getOrCreateStateSet()->setMode( GL_LIGHTING,osg::StateAttribute::OFF);
    gp_root0->getOrCreateStateSet()->setMode( GL_BLEND,osg::StateAttribute::ON);
    gp_root0->getOrCreateStateSet()->setMode( GL_DEPTH_TEST,osg::StateAttribute::OFF);

    gp_root1->getOrCreateStateSet()->setMode( GL_LIGHTING,osg::StateAttribute::OFF);
    gp_root1->getOrCreateStateSet()->setMode( GL_BLEND,osg::StateAttribute::ON);
    gp_root1->getOrCreateStateSet()->setMode( GL_DEPTH_TEST,osg::StateAttribute::OFF);

    // setup view
    osgViewer::CompositeViewer viewer;
    viewer.setThreadingModel(osgViewer::ViewerBase::SingleThreaded);

    // Create view 0
    {
        osgViewer::View* view = new osgViewer::View;
        viewer.addView( view );

        view->setUpViewInWindow( 10, 10, 640, 480 );
        view->setSceneData( gp_root0.get() );

        // fix camera
        osg::Vec3d const eye(4828360.3862586031,-4168117.6413568845,-13474.2768839712);
        osg::Vec3d const vpt(4828360.1656606756,-4168117.4469039082,-13473.3210991657);
        osg::Vec3d const up(0.7241156165,-0.6238520007,0.2940497494);
        double const fovy = 29.1484313724;
        double const ar = 1.3333333333;
        double const zn = 5.0509941806;
        double const zf = 100000.0281295590;

        osg::Camera * cam = view->getCamera();
        cam->setViewMatrixAsLookAt(eye,vpt,up);
        cam->setProjectionMatrixAsPerspective(fovy,ar,zn,zf);
        cam->setClearColor(osg::Vec4(0.1,0.1,0.1,1.0));
    }

    // Create view 1 (this view shows View0's frustum)
    {
        osgViewer::View* view = new osgViewer::View;
        viewer.addView( view );

        view->setUpViewInWindow( 650, 10, 640, 480 );
        view->setSceneData( gp_root1.get() );
        view->getCamera()->setClearColor(osg::Vec4(0.1,0.1,0.1,1.0));

        osg::ref_ptr<osgGA::TrackballManipulator> view_manip =
                new osgGA::TrackballManipulator;
        view_manip->setMinimumDistance(100);

        view->setCameraManipulator(view_manip);
    }

    // =============================================================== //

    // create frustum geometry
    Frustum frustum;
    osg::Camera const * cam = viewer.getView(0)->getCamera();
    auto gp_frustum = BuildFrustumNode("frustum",cam,frustum);
    gp_root1->addChild(gp_frustum);

    // create tiles
    GeoBounds tile_bounds(-180,0,-90,0);
    auto gp_tile = BuildGeoBoundsSurfaceNode("tile",tile_bounds,K_BLUE,0,true);

    GeoBounds view_bounds(-41.6849456469,-39.9034447781,-0.0108214253,2.6748972854);
    auto gp_view = BuildGeoBoundsSurfaceNode("view",view_bounds,K_GREEN,1,true);

    GeoBounds xsec_bounds(-41.6849456469,-39.9034447781,-0.0108214253,0.0000000000);
    auto gp_xsec = BuildGeoBoundsSurfaceNode("xsec",xsec_bounds,K_YELLOW,2,true);

//    gp_root0->addChild(gp_tile);
    gp_root0->addChild(gp_view);
    gp_root0->addChild(gp_xsec);

//    gp_root1->addChild(gp_tile);
    gp_root1->addChild(gp_view);
    gp_root1->addChild(gp_xsec);

    // =============================================================== //

    std::cout << "[starting render loop...]" << std::endl;

    while(!viewer.done())
    {
        viewer.frame();
    }

    return 0;
}
