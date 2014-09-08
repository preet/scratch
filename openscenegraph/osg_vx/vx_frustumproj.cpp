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
#include <projutil.hpp>

//
int main()
{
    Frustum frustum;
    Plane horizon_plane;

    // Celestial body geometry
    auto gp_celestial = BuildCelestialSurfaceNode();

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

        osg::ref_ptr<osgGA::TrackballManipulator> view_manip =
                new osgGA::TrackballManipulator;
        view_manip->setMinimumDistance(100);

        view->setCameraManipulator(view_manip);

    }

    // Create view 1 (this view shows View0's frustum)
    {
        osgViewer::View* view = new osgViewer::View;
        viewer.addView( view );

        view->setUpViewInWindow( 10, 510, 640, 480 );
        view->setSceneData( gp_root1.get() );
        view->getCamera()->setClearColor(osg::Vec4(0.1,0.1,0.1,1.0));

        osg::ref_ptr<osgGA::TrackballManipulator> view_manip =
                new osgGA::TrackballManipulator;
        view_manip->setMinimumDistance(100);

        view->setCameraManipulator(view_manip);
    }

    while(!viewer.done())
    {
        osg::Camera * camera = viewer.getView(0)->getCamera();

        osg::Vec3d eye,vpt,up;
        camera->getViewMatrixAsLookAt(eye,vpt,up);

        // new horizon plane node
        auto new_horizon = BuildHorizonPlaneNode(camera,horizon_plane);

        // new camera frustum node

        double far_dist,near_dist;
        if(!CalcCameraNearFarDist(eye,vpt-eye,20000.0,near_dist,far_dist)) {
            far_dist=0.0;
            near_dist=0.0;
        }

        auto new_frustum = BuildFrustumNode(camera,frustum,near_dist,far_dist);

        osg::ref_ptr<osg::Group> new_lodsurfacepoly;
        osg::ref_ptr<osg::Group> new_frustumsurfacepoly;
        osg::ref_ptr<osg::Group> new_projcenter;

        if(eye.length() > RAD_AV) {

            osg::Vec3d proj_center = CalcGnomonicProjOrigin(horizon_plane);
            Plane tangent_plane = horizon_plane;
            tangent_plane.p = horizon_plane.n * RAD_AV;
            tangent_plane.d = tangent_plane.n * tangent_plane.p;

            std::vector<osg::Vec3d> poly_frustum_surf;
            CalcProjFrustumPoly(frustum,horizon_plane,poly_frustum_surf);

            std::vector<osg::Vec3d> poly_frustum_tangent;
            if(!CalcGnomonicProjPoly(poly_frustum_surf,
                                     proj_center,
                                     tangent_plane,
                                     poly_frustum_tangent)) {
                poly_frustum_tangent.clear();
            }

            std::vector<std::vector<osg::Vec3d>> list_polys_xsec(K_LIST_LOD_DIST.size());
            std::vector<std::vector<osg::Vec3d>> list_polys_tangent(K_LIST_LOD_DIST.size());
            for(size_t i=0; i < K_LIST_LOD_DIST.size(); i++) {
                std::vector<osg::Vec3d> poly_lod_sphere;
                CalcProjSpherePoly(horizon_plane,eye,K_LIST_LOD_DIST[i],poly_lod_sphere);

                // xsec
                if(!CalcGnomonicProjIntersectionPoly(proj_center,
                                                     tangent_plane,
                                                     poly_frustum_surf,
                                                     poly_lod_sphere,
                                                     list_polys_xsec[i],
                                                     list_polys_tangent[i]))
                {
                    list_polys_xsec[i].clear();
                    //list_polys_tangent[i].clear();
                }
            }




            // lodsurfacepoly
            new_lodsurfacepoly = new osg::Group;
            for(size_t i=0; i < list_polys_xsec.size(); i++) {
                new_lodsurfacepoly->addChild(BuildSurfacePoly(list_polys_xsec[i],
                                                              K_COLOR_TABLE[i],
                                                              eye.length()/1200.0));
            }

            // frustumsurfacepoly
            {
                new_frustumsurfacepoly = BuildSurfacePoly(poly_frustum_surf,
                                                          osg::Vec4(0.75,0.5,1.0,1.0),
                                                          eye.length()/1200.0);
            }

            new_projcenter = new osg::Group;
//            new_projcenter = BuildRingNode(proj_center,
//                                           osg::Vec4(1.0,0.0,0.0,1.0),
//                                           eye.length()/300.0);
        }
        else {
            new_lodsurfacepoly = new osg::Group;
            new_frustumsurfacepoly = new osg::Group;
            new_projcenter = new osg::Group;
//            std::cout << "###: _in" << std::endl;
        }

        new_lodsurfacepoly->setName("lodsurfacepoly");
        new_frustumsurfacepoly->setName("frustumsurfacepoly");
        new_projcenter->setName("projcenter");





        // new lod rings
//        auto new_lodrings = BuildLodRingsNode(eye);

        // new min cam dist line node
        auto new_mincamdistline = BuildMinCamDistLineNode(eye);

        // Update gp_root0
        {
            for(size_t i=0; i < gp_root0->getNumChildren(); i++) {
                std::string const name = gp_root0->getChild(i)->getName();
                if(name == "horizonplane") {
                    gp_root0->removeChild(i);
                    i--;
                }
                else if(name == "frustumsurfacepoly") {
                    gp_root0->removeChild(i);
                    i--;
                }
                else if(name == "lodsurfacepoly") {
                    gp_root0->removeChild(i);
                    i--;
                }
                else if(name == "projcenter") {
                    gp_root0->removeChild(i);
                    i--;
                }
            }
            gp_root0->addChild(new_horizon);
            gp_root0->addChild(new_frustumsurfacepoly);
            gp_root0->addChild(new_lodsurfacepoly);
            gp_root0->addChild(new_projcenter);
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
                else if(name == "horizonplane") {
                    gp_root1->removeChild(i);
                    i--;
                }
                else if(name == "frustumsurfacepoly") {
                    gp_root1->removeChild(i);
                    i--;
                }
                else if(name == "lodsurfacepoly") {
                    gp_root1->removeChild(i);
                    i--;
                }
                else if(name == "projcenter") {
                    gp_root1->removeChild(i);
                    i--;
                }
            }
            // Add new nodes
            gp_root1->addChild(new_frustum);
            gp_root1->addChild(new_mincamdistline);
//            gp_root1->addChild(new_lodrings);
            gp_root1->addChild(new_horizon);
            gp_root1->addChild(new_frustumsurfacepoly);
            gp_root1->addChild(new_lodsurfacepoly);
            gp_root1->addChild(new_projcenter);
        }

        viewer.frame();

    }
    return 0;
}

