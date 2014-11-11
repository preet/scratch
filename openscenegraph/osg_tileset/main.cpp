#include <iostream>
#include <iomanip>

#include <ViewController.hpp>
#include <OSGUtils.h>
#include <DataSetTilesLL.h>

int main()
{
    // debug output
    std::cout << std::fixed;
    std::cout << std::setprecision(10);

    // earth surface geometry
    auto gp_earth = BuildEarthSurfaceNode(
                "earth",osg::Vec4(0.35,0.35,0.35,0.1),false);

    // axes
    auto gp_axes = BuildAxesGeometry("axes",RAD_AV*1.15);

    // create TileSet

    // create dataset
    osg::ref_ptr<osg::Group> gp_tiles = new osg::Group;
    std::unique_ptr<scratch::DataSetTilesLL> dataset(
                new scratch::DataSetTilesLL(nullptr,gp_tiles));

    // setup view
    osgViewer::CompositeViewer viewer;
    viewer.setThreadingModel(osgViewer::ViewerBase::SingleThreaded);

    // View0 root
    osg::ref_ptr<osg::Group> gp_root0 = new osg::Group;
    gp_root0->addChild(gp_earth);
    gp_root0->addChild(gp_tiles);
    gp_root0->addChild(gp_axes);

    // View1 root
    osg::ref_ptr<osg::Group> gp_root1 = new osg::Group;
    gp_root1->addChild(gp_earth);
    gp_root1->addChild(gp_tiles);
    gp_root1->addChild(gp_axes);

    // disable lighting and enable blending
    gp_root0->getOrCreateStateSet()->setMode( GL_LIGHTING,osg::StateAttribute::OFF);
    gp_root0->getOrCreateStateSet()->setMode( GL_BLEND,osg::StateAttribute::ON);
    gp_root0->getOrCreateStateSet()->setMode( GL_DEPTH_TEST,osg::StateAttribute::OFF);

    gp_root1->getOrCreateStateSet()->setMode( GL_LIGHTING,osg::StateAttribute::OFF);
    gp_root1->getOrCreateStateSet()->setMode( GL_BLEND,osg::StateAttribute::ON);
    gp_root1->getOrCreateStateSet()->setMode( GL_DEPTH_TEST,osg::StateAttribute::OFF);

    // Create View 0
    {
        osgViewer::View* view = new osgViewer::View;
        viewer.addView( view );

        view->setUpViewInWindow( 1920-640-20, 10, 640, 480 );
        view->setSceneData( gp_root0.get() );

        // setup initial camera
        osg::Camera * camera = view->getCamera();
        osg::Vec3d eye(4.5*RAD_AV,0,0);
        osg::Vec3d vpt(0,0,0);
        osg::Vec3d up(0,0,1);
        camera->setViewMatrixAsLookAt(eye,vpt,up);
        camera->setClearColor(osg::Vec4(0.1,0.1,0.1,1.0));

        view->addEventHandler(new ViewController(640,480));
    }

    // Create view 1 (this view shows View0's frustum)
    {
        osgViewer::View* view = new osgViewer::View;
        viewer.addView( view );

        view->setUpViewInWindow( 1920-640-20, 10+600, 640, 480 );
        view->setSceneData( gp_root1.get() );
        view->getCamera()->setClearColor(osg::Vec4(0.1,0.1,0.1,1.0));

        osg::ref_ptr<osgGA::TrackballManipulator> view_manip =
                new osgGA::TrackballManipulator;
        view_manip->setMinimumDistance(100);

        view->setCameraManipulator(view_manip);
    }

    std::cout << "[starting render loop...]" << std::endl;

    while(!viewer.done())
    {
        // current camera params
        osg::Vec3d eye,vpt,up;
        osg::Camera * camera = viewer.getView(0)->getCamera();
        camera->getViewMatrixAsLookAt(eye,vpt,up);

        //
        dataset->Update(camera);

        double far_dist,near_dist;
        if(!CalcCameraNearFarDist(eye,vpt-eye,20000.0,near_dist,far_dist)) {
            far_dist=0.0;
            near_dist=0.0;
        }

        // the camera eye must be above the planet's surface
        if(eye.length() <= RAD_AV) {
            viewer.frame();
            continue;
        }

        // Build a new frustum node
        Frustum frustum;
        auto new_frustum = BuildFrustumNode("frustum",
                                            camera,
                                            frustum,
                                            near_dist,
                                            far_dist);

        std::vector<GeoBounds> list_b;
        Plane horizon_plane = CalcHorizonPlane(eye);
        std::vector<osg::Vec3d> list_ecef;
        CalcProjFrustumPoly(frustum,horizon_plane,list_ecef);
        CalcMinGeoBoundsFromLLAPoly(ConvECEFToLLA(eye),
                                    ConvListECEFToLLA(list_ecef),
                                    list_b);

        osg::ref_ptr<osg::Group> gp_poly = new osg::Group;
        gp_poly->setName("poly");
        for(size_t i=0; i < list_b.size(); i++) {
            GeoBounds const &b=list_b[i];
            gp_poly->addChild(BuildGeoBoundsNode("-",b,osg::Vec4(1,1,1,1)));
        }
        gp_poly->addChild(BuildSurfacePolyNode("fsp",list_ecef,osg::Vec4(1,1,1,1)));


        // Update gp_root0
        for(size_t i=0; i < gp_root0->getNumChildren(); i++)
        {
            std::string const name =
                    gp_root0->getChild(i)->getName();

            if(name == "poly") {
                gp_root0->removeChild(i);
                i--;
            }
        }
        gp_root0->addChild(gp_poly);

        // Update gp_root1
        for(size_t i=0; i < gp_root1->getNumChildren(); i++)
        {
             std::string const name =
                     gp_root1->getChild(i)->getName();

             if(name == "frustum") {
                 gp_root1->removeChild(i);
                 i--;
             }
             else if(name == "poly") {
                 gp_root1->removeChild(i);
                 i--;
             }
        }
        // add new nodes
        gp_root1->addChild(new_frustum);
        gp_root1->addChild(gp_poly);

        //
        viewer.frame();
    }

    std::cout << "[exit...]" << std::endl;
    return 0;
}
