#include <iostream>

#include <ViewController.hpp>
#include <OSGUtils.h>
#include <TileSetLLByPixelArea.h>
#include <DataSetTilesLL.h>

int main()
{
    // earth surface geometry
    auto gp_earth = BuildEarthSurfaceNode(
                "earth",osg::Vec4(0.35,0.35,0.35,0.1),true);

    // create root tiles
    std::vector<TileSetLL::RootTileDesc> list_root_tiles;
    list_root_tiles.emplace_back(0,-180,180,-90,90);

    // create TileSet
    TileSetLLByPixelArea::Options opts;
    std::unique_ptr<TileSetLL> tileset(
                new TileSetLLByPixelArea(640,480,opts,list_root_tiles));

    // create dataset
    osg::ref_ptr<osg::Group> gp_tiles = new osg::Group;
    std::unique_ptr<DataSetTiles> dataset(
                new DataSetTilesLL(gp_tiles,std::move(tileset)));

    // setup view
    osgViewer::CompositeViewer viewer;
    viewer.setThreadingModel(osgViewer::ViewerBase::SingleThreaded);

    // View0 root
    osg::ref_ptr<osg::Group> gp_root0 = new osg::Group;
    gp_root0->addChild(gp_earth);
    gp_root0->addChild(gp_tiles);

    // View1 root
    osg::ref_ptr<osg::Group> gp_root1 = new osg::Group;
    gp_root1->addChild(gp_earth);
    gp_root1->addChild(gp_tiles);

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

        view->setUpViewInWindow( 10, 10, 640, 480 );
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

        view->setUpViewInWindow( 650, 10, 640, 480 );
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

        // Update gp_root0
        for(size_t i=0; i < gp_root0->getNumChildren(); i++)
        {
//            std::string const name =
//                    gp_root1->getChild(i)->getName();
        }
        // add new nodes

        // Update gp_root1
        for(size_t i=0; i < gp_root1->getNumChildren(); i++)
        {
             std::string const name =
                     gp_root1->getChild(i)->getName();

             if(name == "frustum") {
                 gp_root1->removeChild(i);
                 i--;
             }
        }
        // add new nodes
        gp_root1->addChild(new_frustum);

        //
        viewer.frame();
    }

    std::cout << "[exit...]" << std::endl;
    return 0;
}
