// sys includes
#include <math.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <sys/time.h>
#include <cassert>
#include <memory>
#include <numeric>
#include <chrono>

//
#include <osgnodes.hpp>

void BuildViewExtentsMinCamDist(VxTile * t,
                                Frustum const &frustum,
                                Plane const &horizon_plane)
{
    t->_fvis = CalcFrustumOBBIntersectSAT(frustum,t->obb);
    t->_hvis = !(CalcOBBOutsidePlane(horizon_plane,t->obb));


//    if(t->level == (K_MAX_LOD-1)) {
//        return;
//    }

//    // this distance overestimates a bit; we should
//    // render the bounding box to be sure
//    double const eye_dist2 =
//            CalcMinDist2PointOBB(frustum.eye,t->obb);

//    double const lod_dist2 =
//            K_LIST_LOD_DIST[t->level+1]*
//            K_LIST_LOD_DIST[t->level+1];

//    if(eye_dist2 < lod_dist2) {
//        // LEFT,TOP
//        t->tile_LT = BuildChildTile(t,0);
//        BuildViewExtentsMinCamDist(t->tile_LT.get(),frustum);

//        // LEFT,BTM
//        t->tile_LB = BuildChildTile(t,1);
//        BuildViewExtentsMinCamDist(t->tile_LB.get(),frustum);

//        // RIGHT,BTM
//        t->tile_RB = BuildChildTile(t,2);
//        BuildViewExtentsMinCamDist(t->tile_RB.get(),frustum);

//        // RIGHT,TOP
//        t->tile_RT = BuildChildTile(t,3);
//        BuildViewExtentsMinCamDist(t->tile_RT.get(),frustum);
//    }
}

void ClearViewExtents(std::vector<VxTile*> const &list_base_vx_tiles)
{
    for(auto vx_tile : list_base_vx_tiles)
    {
        vx_tile->tile_LT = nullptr;
        vx_tile->tile_LB = nullptr;
        vx_tile->tile_RB = nullptr;
        vx_tile->tile_RT = nullptr;
    }
}

void BuildViewExtents(std::vector<VxTile*> &list_base_vx_tiles,
                      Frustum const &frustum,
                      Plane const &horizon_plane)
{
    ClearViewExtents(list_base_vx_tiles);

    for(auto vx_tile : list_base_vx_tiles)
    {
        BuildViewExtentsMinCamDist(vx_tile,frustum,horizon_plane);
    }
}


//
int main()
{
    Frustum frustum;
    Plane horizon_plane;

    // Celestial body geometry
    auto gp_celestial = BuildCelestialSurfaceNode();

    // Base view extents
    std::vector<VxTile*> list_base_vx_tiles = BuildBaseViewExtents(5);

    osgViewer::CompositeViewer viewer;
    viewer.setThreadingModel(osgViewer::ViewerBase::SingleThreaded);

    // View0 root
    osg::ref_ptr<osg::Group> gp_root0 = new osg::Group;
    gp_root0->addChild(gp_celestial);

    // View1 root
    osg::ref_ptr<osg::Group> gp_root1 = new osg::Group;
    gp_root1->addChild(gp_celestial);

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
        view->getCamera()->setClearColor(osg::Vec4(0.1,0.1,0.1,1.0));
        view->setCameraManipulator( new osgGA::TrackballManipulator );
    }

    // Create view 1 (this view shows View0's frustum)
    {
        osgViewer::View* view = new osgViewer::View;
        viewer.addView( view );

        view->setUpViewInWindow( 10, 510, 640, 480 );
        view->setSceneData( gp_root1.get() );
        view->getCamera()->setClearColor(osg::Vec4(0.1,0.1,0.1,1.0));
        view->setCameraManipulator( new osgGA::TrackballManipulator );
    }

    while(!viewer.done())
    {
        osg::Camera * camera = viewer.getView(0)->getCamera();

        osg::Vec3d eye,vpt,up;
        camera->getViewMatrixAsLookAt(eye,vpt,up);

        // new camera frustum node
        auto new_frustum = BuildFrustumNode(camera,frustum);

        // new horizon plane node
        auto new_horizon = BuildHorizonPlaneNode(camera,horizon_plane);

        // new lod rings
        auto new_lodrings = BuildLodRingsNode(eye);

        // new min cam dist line node
        auto new_mincamdistline = BuildMinCamDistLineNode(eye);

        // new vxtiles
        BuildViewExtents(list_base_vx_tiles,frustum,horizon_plane);
        auto new_vxtiles = BuildViewExtentsNode(list_base_vx_tiles);
//        auto new_vxtilesobb = BuildViewExtentsOBBNode(list_base_vx_tiles);

        // temp new frustumobbproj
        // auto new_frustumobbproj = BuildFrustumOBBProjNode(frustum,list_base_vx_tiles[0]->obb);

        // Update gp_root0
        {
            for(size_t i=0; i < gp_root0->getNumChildren(); i++) {
                std::string const name = gp_root0->getChild(i)->getName();
                if(name == "vxtiles") {
                    gp_root0->removeChild(i);
                    i--;
                }
                else if(name == "vxtilesobb") {
                    gp_root0->removeChild(i);
                    i--;
                }
                else if(name == "horizonplane") {
                    gp_root0->removeChild(i);
                    i--;
                }
            }
            gp_root0->addChild(new_vxtiles);
//            gp_root0->addChild(new_vxtilesobb);
            gp_root0->addChild(new_horizon);
        }

        // Update gp_root1
        {
            for(size_t i=0; i < gp_root1->getNumChildren(); i++) {
                std::string const name = gp_root1->getChild(i)->getName();
                // Remove prev camera node
                if(name == "frustum") {
                    gp_root1->removeChild(i);
                    i--;
                }
                else if(name == "mincamdistline") {
                    gp_root1->removeChild(i);
                    i--;
                }
                else if(name == "lodrings") {
                    gp_root1->removeChild(i);
                    i--;
                }
                else if(name == "vxtiles") {
                    gp_root1->removeChild(i);
                    i--;
                }
                else if(name == "vxtilesobb") {
                    gp_root1->removeChild(i);
                    i--;
                }
                else if(name == "horizonplane") {
                    gp_root1->removeChild(i);
                    i--;
                }
                else if(name == "frustumobbproj") {
                    gp_root1->removeChild(i);
                    i--;
                }
            }
            // Add new nodes
            gp_root1->addChild(new_frustum);
            gp_root1->addChild(new_mincamdistline);
            gp_root1->addChild(new_lodrings);
            gp_root1->addChild(new_vxtiles);
//            gp_root1->addChild(new_vxtilesobb);
            gp_root1->addChild(new_horizon);
            //gp_root1->addChild(new_frustumobbproj);
        }

        viewer.frame();
    }

    return 0;
}
