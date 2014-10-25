#include <iostream>
#include <OSGUtils.h>

osg::Vec4 const K_CYAN = osg::Vec4(0,1,1,1);
osg::Vec4 const K_MAGENTA = osg::Vec4(1,0,1,1);
osg::Vec4 const K_YELLOW = osg::Vec4(1,1,0,1);
osg::Vec4 const K_RED = osg::Vec4(1,0,0,1);
osg::Vec4 const K_GREEN = osg::Vec4(0,1,0,1);
osg::Vec4 const K_BLUE = osg::Vec4(0,0,1,1);
osg::Vec4 const K_GRAY = osg::Vec4(0.22,0.22,0.22,1);
osg::Vec4 const K_LIGHTGRAY = osg::Vec4(0.5,0.5,0.5,1);

bool RemoveChildByName(osg::Group * gp_parent,
                       std::string const &name)
{
    for(size_t i=0; i < gp_parent->getNumChildren(); i++)
    {
        std::string const node_name =
                gp_parent->getChild(i)->getName();

        if(node_name == name) {
            gp_parent->removeChild(i);
            return true;
        }
    }

    return false;
}

int main()
{
    // view root
    osg::ref_ptr<osg::Group> gp_root = new osg::Group;

    // earth surface geometry
    auto gp_earth = BuildEarthSurfaceNode("earth",K_GRAY);
    gp_root->addChild(gp_earth);

    // tile
    GeoBounds tile_bounds(-20,20,-20,20);
    auto gp_tile = BuildGeoBoundsSurfaceNode(
                "tile",tile_bounds,K_LIGHTGRAY,0,true,1,1);
    gp_root->addChild(gp_tile);

    // disable lighting and enable blending
    gp_root->getOrCreateStateSet()->setMode( GL_LIGHTING,osg::StateAttribute::OFF);
    gp_root->getOrCreateStateSet()->setMode( GL_BLEND,osg::StateAttribute::ON);
    gp_root->getOrCreateStateSet()->setMode( GL_DEPTH_TEST,osg::StateAttribute::OFF);

    // setup view
    double const view_width  = 600;
    double const view_height = 360;
    osgViewer::Viewer viewer;
    viewer.setThreadingModel(osgViewer::ViewerBase::SingleThreaded);
    viewer.setUpViewInWindow(1920-600-100,100,view_width,view_height);
    viewer.setSceneData(gp_root);
    viewer.getCamera()->setClearColor(osg::Vec4(0.1,0.1,0.1,1.0));

    osg::ref_ptr<osgGA::TrackballManipulator> view_manip =
            new osgGA::TrackballManipulator;
    view_manip->setMinimumDistance(100);
    viewer.setCameraManipulator(view_manip);

    // create color list
    std::vector<osg::Vec4> const list_cx = {
        K_RED,
        K_GREEN,
        K_BLUE,
        K_CYAN,
        K_MAGENTA,
        K_YELLOW
    };

    // get tile tri verts
    std::vector<osg::Vec3d> list_vx;
    std::vector<osg::Vec2d> list_tx;
    std::vector<uint16_t> list_ix;
    BuildEarthSurface(tile_bounds.minLon,
                      tile_bounds.maxLon,
                      tile_bounds.minLat,
                      tile_bounds.maxLat,
                      1,
                      1,
                      list_vx,
                      list_tx,
                      list_ix);

    std::cout << "[staring render loop...]" << std::endl;

    while(!viewer.done()) {

        RemoveChildByName(gp_root,"tile_vx");

        osg::ref_ptr<osg::Group> gp_tile_vx = new osg::Group;
        gp_tile_vx->setName("tile_vx");

        for(size_t i=0; i < list_ix.size()/2; i++) {
            auto gp = BuildFacingCircleNode(std::to_string(i),
                                            list_vx[list_ix[i]],
                                            250000,
                                            8,
                                            list_cx[i]);
            gp_tile_vx->addChild(gp);
        }

        gp_root->addChild(gp_tile_vx);

        viewer.frame();
    }


    std::cout << "[exit...]" << std::endl;
    return 0;
}
