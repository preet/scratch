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
#include <iomanip>

//
#include <osgnodes.hpp>
#include <projutil.hpp>



double const K_GT_NEPS = -1E-6;
double const K_GT_EPS = 1E-6;

void GenTilesForPoly(std::vector<Edge> const &list_poly_edges,
                     uint8_t const max_level,
                     std::vector<PointLLA> &list_lla,
                     std::set<double> &list_used_lons,
                     std::set<double> &list_used_lats,
                     VxTile * tile)
{
    if(tile->level == max_level) {
        bool outside;
        for(auto const &lla : list_lla) {
            outside =
                    lla.lon < tile->minLon-K_GT_EPS ||
                    lla.lon > tile->maxLon+K_GT_EPS ||
                    lla.lat < tile->minLat-K_GT_EPS ||
                    lla.lat > tile->maxLat+K_GT_EPS;

            if(!outside) {
                return;
            }
        }

        tile->_s = false;
        return;
    }

    //
    if(list_used_lons.count(tile->midLon) == 0)
    {
        // Get midLon plane
        Plane plane_lon;
        plane_lon.n = (tile->ecef_MM)^(osg::Vec3d(0,0,1));
        plane_lon.p = tile->ecef_MM;
        plane_lon.d = plane_lon.n*plane_lon.p;

        for(auto const &edge : list_poly_edges) {
            // Check if the edge intersects the lat plane
            double u;
            osg::Vec3d xsec_pt;
            auto const xsec_type =
                    CalcLinePlaneIntersection(edge.a,
                                              edge.a+edge.dirn_ab,
                                              plane_lon,
                                              xsec_pt,
                                              u);

            if((xsec_type == XSEC_COINCIDENT) ||
               (xsec_type == XSEC_TRUE && u >= (0-K_GT_EPS) && u <= (1+K_GT_EPS)))
            {
                PointLLA xsec_lla = ConvECEFToLLA(xsec_pt);
                xsec_lla.alt = 0.0;
                if(fabs(xsec_lla.lon) == 180.0) {
                    xsec_lla.lon = 180.0;
                    list_lla.push_back(xsec_lla);
                    xsec_lla.lon = -180.0;
                    list_lla.push_back(xsec_lla);
                }
                else {
                    list_lla.push_back(xsec_lla);
                }
            }
        }

        // save lon
        list_used_lons.insert(tile->midLon);
        if(tile->midLon > 0) {
            list_used_lons.insert(tile->midLon-180.0);
        }
        else if(tile->midLon < 0) {
            list_used_lons.insert(tile->midLon+180.0);
        }
        else {
            list_used_lons.insert(tile->midLon-180.0);
            list_used_lons.insert(tile->midLon+180.0);
        }
    }

    //
    if(list_used_lats.count(tile->midLat) == 0)
    {
        // Get midLat plane
        Plane plane_lat;
        plane_lat.n = osg::Vec3d(0,0,1);
        plane_lat.p = tile->ecef_MM;
        plane_lat.d = plane_lat.n*plane_lat.p;

        for(auto const &edge : list_poly_edges) {
            // Check if the edge intersects the lon plane
            double u;
            osg::Vec3d xsec_pt;
            auto const xsec_type =
                    CalcLinePlaneIntersection(edge.a,
                                              edge.a+edge.dirn_ab,
                                              plane_lat,
                                              xsec_pt,
                                              u);

            if((xsec_type == XSEC_COINCIDENT) ||
               (xsec_type == XSEC_TRUE && u >= (0-K_GT_EPS) && u <= (1+K_GT_EPS)))
            {
                PointLLA xsec_lla = ConvECEFToLLA(xsec_pt);
                xsec_lla.alt = 0.0;
                list_lla.push_back(xsec_lla);
            }
        }

        // save lat
        list_used_lats.insert(tile->midLat);
    }

    //
    bool outside;
    for(auto const &lla : list_lla) {
        outside =
                lla.lon < tile->minLon-K_GT_EPS ||
                lla.lon > tile->maxLon+K_GT_EPS ||
                lla.lat < tile->minLat-K_GT_EPS ||
                lla.lat > tile->maxLat+K_GT_EPS;

        if(!outside) {
            tile->tile_LT = BuildChildTile(tile,0);
            tile->tile_LB = BuildChildTile(tile,1);
            tile->tile_RB = BuildChildTile(tile,2);
            tile->tile_RT = BuildChildTile(tile,3);

            GenTilesForPoly(list_poly_edges,
                            max_level,
                            list_lla,
                            list_used_lons,
                            list_used_lats,
                            tile->tile_LT.get());

            GenTilesForPoly(list_poly_edges,
                            max_level,
                            list_lla,
                            list_used_lons,
                            list_used_lats,
                            tile->tile_LB.get());

            GenTilesForPoly(list_poly_edges,
                            max_level,
                            list_lla,
                            list_used_lons,
                            list_used_lats,
                            tile->tile_RB.get());

            GenTilesForPoly(list_poly_edges,
                            max_level,
                            list_lla,
                            list_used_lons,
                            list_used_lats,
                            tile->tile_RT.get());

            break;
        }
    }

    if(outside) {
        tile->_s = false;
    }


//    // quadrant 0 - LT
//    for(auto const &lla : list_lla) {
//        outside = lla.lon < tile->minLon-K_GT_EPS ||
//                  lla.lon > tile->midLon+K_GT_EPS ||
//                  lla.lat < tile->midLat-K_GT_EPS ||
//                  lla.lat > tile->maxLat+K_GT_EPS;
//        if(!outside) {
//            tile->tile_LT = BuildChildTile(tile,0);
//            GenTilesForPoly(list_poly_edges,
//                            max_level,
//                            list_lla,
//                            list_used_lons,
//                            list_used_lats,
//                            tile->tile_LT.get());
//            break;
//        }
//    }

//    // quadrant 1 - LB
//    for(auto const &lla : list_lla) {
//        outside = lla.lon < tile->minLon-K_GT_EPS ||
//                  lla.lon > tile->midLon+K_GT_EPS ||
//                  lla.lat < tile->minLat-K_GT_EPS ||
//                  lla.lat > tile->midLat+K_GT_EPS;
//        if(!outside) {
//            tile->tile_LB = BuildChildTile(tile,1);
//            GenTilesForPoly(list_poly_edges,
//                            max_level,
//                            list_lla,
//                            list_used_lons,
//                            list_used_lats,
//                            tile->tile_LB.get());
//            break;
//        }
//    }

//    // quadrant 2 - RB
//    for(auto const &lla : list_lla) {
//        outside = lla.lon < tile->midLon-K_GT_EPS ||
//                  lla.lon > tile->maxLon+K_GT_EPS ||
//                  lla.lat < tile->minLat-K_GT_EPS ||
//                  lla.lat > tile->midLat+K_GT_EPS;
//        if(!outside) {
//            tile->tile_RB = BuildChildTile(tile,2);
//            GenTilesForPoly(list_poly_edges,
//                            max_level,
//                            list_lla,
//                            list_used_lons,
//                            list_used_lats,
//                            tile->tile_RB.get());
//            break;
//        }
//    }

//    // quadrant 3 - RT
//    for(auto const &lla : list_lla) {
//        outside = lla.lon < tile->midLon-K_GT_EPS ||
//                  lla.lon > tile->maxLon+K_GT_EPS ||
//                  lla.lat < tile->midLat-K_GT_EPS ||
//                  lla.lat > tile->maxLat+K_GT_EPS;
//        if(!outside) {
//            tile->tile_RT = BuildChildTile(tile,3);
//            GenTilesForPoly(list_poly_edges,
//                            max_level,
//                            list_lla,
//                            list_used_lons,
//                            list_used_lats,
//                            tile->tile_RT.get());
//            break;
//        }
//    }
}

std::vector<Edge> GetListEdgesForPoly(std::vector<osg::Vec3d> const &poly)
{
    std::vector<Edge> list_poly_edges;
    for(size_t i=1; i < poly.size(); i++) {
        Edge edge;
        edge.a = poly[i-1];
        edge.dirn_ab = poly[i]-edge.a;
        list_poly_edges.push_back(edge);
    }
    // last edge
    {
        Edge edge;
        edge.a = poly[poly.size()-1];
        edge.dirn_ab = poly[0]-edge.a;
        list_poly_edges.push_back(edge);
    }

    return list_poly_edges;
}






int main()
{


//    std::vector<PointLLA> list_lla;
//    list_lla.emplace_back(170,70);
//    list_lla.emplace_back(-180,65);
//    list_lla.emplace_back(-170,70);
//    list_lla.emplace_back(-180,75);

//    std::vector<osg::Vec3d> poly_temp;
//    poly_temp.push_back(ConvLLAToECEF(list_lla[0]));
//    poly_temp.push_back(ConvLLAToECEF(list_lla[1]));
//    poly_temp.push_back(ConvLLAToECEF(list_lla[2]));
//    poly_temp.push_back(ConvLLAToECEF(list_lla[3]));

//    std::vector<Edge> list_poly_edges = GetListEdgesForPoly(poly_temp);

//    std::set<double> list_used_lons;
//    std::set<double> list_used_lats;

//    GenTilesForPoly(list_poly_edges,
//                    7,
//                    list_lla,
//                    list_used_lons,
//                    list_used_lats,
//                    root_tile.get());

//    auto gp_poly = BuildSurfacePoly(poly_temp,osg::Vec4(1,1,1,1));

//    osg::ref_ptr<osg::Group> gp_vx = new osg::Group;
//    BuildViewExtentsGeometryAsPolyLines(root_tile.get(),gp_vx.get());


    Frustum frustum;
    Plane horizon_plane;

    // Celestial body geometry
    auto gp_celestial = BuildCelestialSurfaceNode();

    osgViewer::CompositeViewer viewer;
    viewer.setThreadingModel(osgViewer::ViewerBase::SingleThreaded);

    // View0 root
    osg::ref_ptr<osg::Group> gp_root0 = new osg::Group;
    gp_root0->addChild(gp_celestial);
//    gp_root0->addChild(gp_poly);
//    gp_root0->addChild(gp_vx);

    // View1 root
    osg::ref_ptr<osg::Group> gp_root1 = new osg::Group;
    gp_root1->addChild(gp_celestial);
//    gp_root1->addChild(gp_poly);
//    gp_root1->addChild(gp_vx);

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

        view->setUpViewInWindow( 650, 10, 640, 480 );
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
        (void)new_horizon;

        // new camera frustum node
        double far_dist,near_dist;
        if(!CalcCameraNearFarDist(eye,vpt-eye,20000.0,near_dist,far_dist)) {
            far_dist=0.0;
            near_dist=0.0;
        }
        auto new_frustum = BuildFrustumNode(camera,frustum,near_dist,far_dist);

        osg::ref_ptr<osg::Group> new_frustumsurfpoly;
        osg::ref_ptr<osg::Group> new_tilevx;
        osg::ref_ptr<osg::Group> new_pt_0_0;
        osg::ref_ptr<osg::Group> new_pt_90_0;
        osg::ref_ptr<osg::Group> new_debug;

        if(eye.length() > RAD_AV) {
            // proj view extents
            std::vector<osg::Vec3d> poly_frustum_surf;
            CalcProjFrustumPoly(frustum,horizon_plane,poly_frustum_surf);

            std::vector<PointLLA> poly_frustum_lla;
            for(auto const &vx : poly_frustum_surf) {
                poly_frustum_lla.push_back(ConvECEFToLLA(vx));
            }

            auto poly_frustum_edges = GetListEdgesForPoly(poly_frustum_surf);

            // tile view extents
            std::unique_ptr<VxTile> root_tile(BuildRootTile(0,-180,-90,180,90));

            std::set<double> list_used_lons;
            std::set<double> list_used_lats;
            GenTilesForPoly(poly_frustum_edges,
                            6,
                            poly_frustum_lla,
                            list_used_lons,
                            list_used_lats,
                            root_tile.get());

            new_debug = new osg::Group;
            new_debug->setName("debug");
            for(auto &lla : poly_frustum_lla) {
                lla.alt = 0;
                new_debug->addChild(BuildFacingCircle(ConvLLAToECEF(lla),
                                                      eye.length()/1000.0,
                                                      4,
                                                      osg::Vec4(1,0,1,1)));
            }

            // new tilevx
            new_tilevx = new osg::Group;
            BuildViewExtentsGeometryAsPolyLines(root_tile.get(),new_tilevx.get());

            // new frustumsurfproj
            new_frustumsurfpoly = BuildSurfacePoly(poly_frustum_surf,
                                                   osg::Vec4(0.75,0.5,1.0,1.0),
                                                   eye.length()/1200.0);

            // new 0_0_pt
            new_pt_0_0 = BuildFacingCircle(ConvLLAToECEF(PointLLA(0,0)),
                                           eye.length()/150.0,
                                           8,
                                           osg::Vec4(1,0,0,1));
            // new 90_0_pt
            new_pt_90_0 = BuildFacingCircle(ConvLLAToECEF(PointLLA(90,0)),
                                            eye.length()/150.0,
                                            8,
                                            osg::Vec4(0,1,0,1));
        }
        else {
            new_frustumsurfpoly = new osg::Group;
            new_tilevx = new osg::Group;
            new_pt_0_0 = new osg::Group;
            new_pt_90_0 = new osg::Group;
            new_debug = new osg::Group;
        }

        new_frustumsurfpoly->setName("frustumsurfpoly");
        new_tilevx->setName("tilevx");
        new_pt_0_0->setName("pt_0_0");
        new_pt_90_0->setName("pt_90_0");
        new_debug->setName("debug");

        // Update gp_root0
        {
            for(size_t i=0; i < gp_root0->getNumChildren(); i++) {
                std::string const name = gp_root0->getChild(i)->getName();
                if(name == "frustumsurfpoly") {
                    gp_root0->removeChild(i);
                    i--;
                }
                else if(name == "tilevx") {
                    gp_root0->removeChild(i);
                    i--;
                }
                else if(name == "pt_0_0") {
                    gp_root0->removeChild(i);
                    i--;
                }
                else if(name =="pt_90_0") {
                    gp_root0->removeChild(i);
                    i--;
                }
                else if(name == "debug") {
                    gp_root0->removeChild(i);
                    i--;
                }
            }
            gp_root0->addChild(new_frustumsurfpoly);
            gp_root0->addChild(new_tilevx);
            gp_root0->addChild(new_pt_0_0);
            gp_root0->addChild(new_pt_90_0);
            gp_root0->addChild(new_debug);
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
                else if(name == "frustumsurfpoly") {
                    gp_root1->removeChild(i);
                    i--;
                }
                else if(name == "tilevx") {
                    gp_root1->removeChild(i);
                    i--;
                }
                else if(name == "pt_0_0") {
                    gp_root1->removeChild(i);
                    i--;
                }
                else if(name =="pt_90_0") {
                    gp_root1->removeChild(i);
                    i--;
                }
                else if(name == "debug") {
                    gp_root1->removeChild(i);
                    i--;
                }
            }
            // Add new nodes
            gp_root1->addChild(new_frustum);
            gp_root1->addChild(new_frustumsurfpoly);
            gp_root1->addChild(new_tilevx);
            gp_root1->addChild(new_pt_0_0);
            gp_root1->addChild(new_pt_90_0);
            gp_root1->addChild(new_debug);
        }
        viewer.frame();
    }
    return 0;
}

