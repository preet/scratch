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

bool CalcLonPlaneIntersection(std::vector<Edge> const &list_poly_edges,
                              osg::Vec3d const &ecef_lon)
{
    Plane plane_lon;
    plane_lon.n = ecef_lon^(osg::Vec3d(0,0,1));
    plane_lon.p = ecef_lon;
    plane_lon.d = plane_lon.n*plane_lon.p;

    for(auto const &edge : list_poly_edges) {
        // Check if the edge intersects the lon plane
        double u;
        auto const xsec_type = CalcLinePlaneIntersection(edge,plane_lon,u);

        if((xsec_type == XSEC_TRUE && u >= (0-K_GT_EPS) && u <= (1+K_GT_EPS)) ||
           (xsec_type == XSEC_COINCIDENT))
        {
            return true;
        }
    }
    return false;
}

bool CalcLatPlaneIntersection(std::vector<Edge> const &list_poly_edges,
                              osg::Vec3d const &ecef_lat)
{
    Plane plane_lat;
    plane_lat.n = osg::Vec3d(0,0,1);
    plane_lat.p = ecef_lat;
    plane_lat.d = plane_lat.n*plane_lat.p;

    for(auto const &edge : list_poly_edges) {
        // Check if the edge intersects the lon plane
        double u;
        auto const xsec_type = CalcLinePlaneIntersection(edge,plane_lat,u);

        if((xsec_type == XSEC_TRUE && u >= (0-K_GT_EPS) && u <= (1+K_GT_EPS)) ||
           (xsec_type == XSEC_COINCIDENT))
        {
            return true;
        }
    }
    return false;
}

bool CalcTilePolyIntersection(std::vector<Edge> const &list_poly_edges,
                              VxTile const * tile)
{
    // min_lon
    if(CalcLonPlaneIntersection(list_poly_edges,*(tile->p_ecef_LT)) || // min_lon
       CalcLonPlaneIntersection(list_poly_edges,*(tile->p_ecef_RT)) || // max_lon
       CalcLatPlaneIntersection(list_poly_edges,*(tile->p_ecef_LT)) || // min_lat
       CalcLatPlaneIntersection(list_poly_edges,*(tile->p_ecef_LB)))   // max_lat
    {
        return true;
    }

    return false;
}

// checks whether the center of the tile is contained by the poly
bool CalcTilePolyContained(std::vector<osg::Vec2d> const &list_proj_poly,
                           VxTile const * tile,
                           Plane const &horizon_plane,
                           Plane const &tangent_plane,
                           osg::Vec3d const &proj_center)
{
    // Ensure that the tile midpoint is above the horizon plane
    if(CalcPointPlaneSignedDistance(tile->ecef_MM,horizon_plane) < 0) {
        return false;
    }

    // Use gnomonic projection to project the midpoint to proj_plane
    double u;
    osg::Vec3d xsec;
    if(!CalcRayPlaneIntersection(proj_center,
                                 tile->ecef_MM-proj_center,
                                 tangent_plane,
                                 xsec,
                                 u)) {
        return false;
    }

    // Transform to xy plane
    osg::Matrixd xf_tangent_to_xy;
    xf_tangent_to_xy.makeRotate(tangent_plane.n,osg::Vec3d(0,0,1));
    xsec = xsec * xf_tangent_to_xy;

    // check mid_lon, mid_lat within list_poly_planes
    if(CalcPointInPoly(list_proj_poly,osg::Vec2d(xsec.x(),xsec.y()))) {
        return true;
    }

    return false;
}

void GenBaseTilesForGeoBounds(GeoBounds const &geobounds,
                              uint8_t const level,
                              double const lon_div_degs,
                              double const lat_div_degs,
                              std::vector<std::unique_ptr<VxTile>> &list_tiles)
{
    double const start_lon = floor(geobounds.minLon/lon_div_degs) * lon_div_degs;
    double const start_lat = floor(geobounds.minLat/lat_div_degs) * lat_div_degs;

    for(double lon = start_lon;
        lon < geobounds.maxLon;
        lon += lon_div_degs)
    {
        for(double lat = start_lat;
            lat < geobounds.maxLat;
            lat += lat_div_degs)
        {
            list_tiles.emplace_back(BuildRootTile(level,
                                                  lon,
                                                  lat,
                                                  lon+lon_div_degs,
                                                  lat+lat_div_degs));
        }
    }
}

int main()
{
    Frustum frustum;
    Plane horizon_plane;

//    {
//        std::cout << "#: " << CalcValidAngleDegs(-195,DEG_180_180) << std::endl;
//        std::cout << "#: " << CalcValidAngleDegs(195,DEG_180_180) << std::endl;
//        std::cout << "#: " << CalcValidAngleDegs(-365,DEG_180_180) << std::endl;
//        std::cout << "#: " << CalcValidAngleDegs(365,DEG_180_180) << std::endl;
//        return 0;
//    }

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

        // new lod rings node
        auto new_lodrings = BuildLodRingsNode(eye);

        osg::ref_ptr<osg::Group> new_frustumsurfpoly;
        osg::ref_ptr<osg::Group> new_lodsurfpoly;
        osg::ref_ptr<osg::Group> new_debug;
        osg::ref_ptr<osg::Group> new_pt_0_0;
        osg::ref_ptr<osg::Group> new_pt_90_0;

        if(eye.length() > RAD_AV)
        {
            // Create projection view extents
            osg::Vec3d proj_center = CalcGnomonicProjOrigin(horizon_plane);
            Plane tangent_plane = horizon_plane;
            tangent_plane.p = horizon_plane.n * RAD_AV;
            tangent_plane.d = tangent_plane.n * tangent_plane.p;

            std::vector<osg::Vec3d> poly_frustum_surf;
            CalcProjFrustumPoly(frustum,horizon_plane,poly_frustum_surf);

            std::vector<PointLLA> poly_frustum_lla;
            for(auto const &vx : poly_frustum_surf) {
                poly_frustum_lla.push_back(ConvECEFToLLA(vx));
            }

            // new frustumsurfproj
            new_frustumsurfpoly = BuildSurfacePoly(poly_frustum_surf,
                                                   osg::Vec4(0.75,0.5,1.0,1.0),
                                                   eye.length()/1200.0);

            // Find the minimum lod that encompasses
            // the frustum proj poly
            size_t min_lod=0;
            size_t max_lod=K_LIST_LOD_DIST.size()-1;
            for(size_t i=0; i < K_LIST_LOD_DIST.size(); i++)
            {
                bool contained = true;
                double dist2 =
                        K_LIST_LOD_DIST[max_lod-i]*
                        K_LIST_LOD_DIST[max_lod-i];

                for(auto const &vx : poly_frustum_surf) {
                    if((eye-vx).length2() > dist2) {
                        contained = false;
                        break;
                    }
                }

                if(contained) {
                    min_lod = max_lod-i;
                    break;
                }
            }

            // Intersect gnomonic projections of distance spheres
            // and the camera frustum
            std::vector<std::vector<osg::Vec3d>> list_polys_xsec;

            for(size_t i=min_lod+1; i <= max_lod; i++)
            {
                std::vector<osg::Vec3d> poly_lod_sphere;
                CalcProjSpherePoly(horizon_plane,eye,K_LIST_LOD_DIST[i],poly_lod_sphere);

                std::vector<osg::Vec3d> poly_xsec,poly_xsec_tangent;
                if(CalcGnomonicProjIntersectionPoly(proj_center,
                                                    tangent_plane,
                                                    poly_frustum_surf,
                                                    poly_lod_sphere,
                                                    poly_xsec,
                                                    poly_xsec_tangent))
                {
                    list_polys_xsec.push_back(poly_xsec);
                }
            }

            // new lodsurfpoly
            new_lodsurfpoly = new osg::Group;
            for(size_t i=0; i < list_polys_xsec.size(); i++) {
                new_lodsurfpoly->addChild(BuildSurfacePoly(list_polys_xsec[i],
                                                           K_COLOR_TABLE[i],
                                                           eye.length()/1200.0));
            }

            std::cout << "###: list_polys_xsec sz: " << list_polys_xsec.size() << std::endl;

            // geobb tiles
            new_debug = new osg::Group;
//            if(furthest_lod > 2) {
//                std::vector<GeoBounds> list_frustum_geobb;
//                if(CalcMinGeoBoundsFromLLAPoly(ConvECEFToLLA(eye),
//                                               poly_frustum_lla,
//                                               list_frustum_geobb))
//                {
//                    std::vector<std::unique_ptr<VxTile>> list_tiles;
//                    for(auto const &geobounds : list_frustum_geobb) {
//                        GenBaseTilesForGeoBounds(geobounds,furthest_lod,
//                                                 360.0/pow(2,furthest_lod),
//                                                 180.0/pow(2,furthest_lod),
//                                                 list_tiles);
//                    }
//                    std::cout << std::endl;

//                    for(auto const &vxtile : list_tiles) {
//                        BuildTilesAsPolyLines(vxtile.get(),new_debug);
//                    }
//                }
//            }




            // temp
            {
//                osg::Camera * camera2 = viewer.getView(1)->getCamera();
//                osg::Vec3d eye2,vpt2,up2;
//                camera2->getViewMatrixAsLookAt(eye2,vpt2,up2);

//                osg::Vec3d xsec_near,xsec_far;
//                CalcRayEarthIntersection(eye2,-eye2,xsec_near,xsec_far);
//                new_debug->addChild(BuildFacingCircle(xsec_near,
//                                                      eye.length()/150.0,
//                                                      8,
//                                                      osg::Vec4(1,1,0,1)));

//                osg::Vec3d proj_center = CalcGnomonicProjOrigin(horizon_plane);
//                Plane tangent_plane = horizon_plane;
//                tangent_plane.p = horizon_plane.n * RAD_AV;
//                tangent_plane.d = tangent_plane.n * tangent_plane.p;

//                osg::Vec2d pnpoly(0,0);

//                if(CalcPointPlaneSignedDistance(xsec_near,horizon_plane) > 0) {
//                    std::vector<osg::Vec2d> list_xsec_near_2d;
//                    std::vector<osg::Vec3d> list_xsec_near;
//                    list_xsec_near.push_back(xsec_near);
//                    if(CalcGnomonicProjPolyXY(list_xsec_near,
//                                              proj_center,
//                                              tangent_plane,
//                                              list_xsec_near_2d))
//                    {
//                        pnpoly = list_xsec_near_2d[0];
//                        osg::Vec3d test_pt(pnpoly.x(),
//                                           pnpoly.y(),
//                                           0);
//                        auto temp_gp = BuildFacingCircle(test_pt,
//                                                         eye.length()/150,
//                                                         4,
//                                                         osg::Vec4(1,1,0,1));
//                        new_debug->addChild(temp_gp);
//                    }
//                }

//                std::vector<osg::Vec2d> list_vx_xy;
//                if(CalcGnomonicProjPolyXY(poly_frustum_surf,
//                                          proj_center,
//                                          tangent_plane,
//                                          list_vx_xy))
//                {
//                    std::vector<osg::Vec3d> list_temp_vx;
//                    list_temp_vx.reserve(list_vx_xy.size());
//                    for(auto const &vx_xy : list_vx_xy) {
//                        list_temp_vx.emplace_back(vx_xy.x(),vx_xy.y(),0);
//                    }

//                    osg::Vec4 tempcolor(0,0,1,1);
//                    if(pnpoly.length2() > 0) {
//                        if(CalcPointInPoly(list_vx_xy,pnpoly)) {
//                            tempcolor.x() = 1;
//                        }
//                    }

//                    auto temp_gp = BuildSurfacePoly(list_temp_vx,tempcolor);
//                    new_debug->addChild(temp_gp);
//                }
//                poly_frustum_surf.pop_back();
            }

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
            new_lodsurfpoly = new osg::Group;
            new_pt_0_0 = new osg::Group;
            new_pt_90_0 = new osg::Group;
            new_debug = new osg::Group;
        }

        new_frustumsurfpoly->setName("frustumsurfpoly");
        new_lodsurfpoly->setName("lodsurfpoly");
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
                else if(name == "lodsurfpoly") {
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
            gp_root0->addChild(new_lodsurfpoly);
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
                else if(name == "lodsurfpoly") {
                    gp_root1->removeChild(i);
                    i--;
                }
                else if(name == "lodrings") {
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
            gp_root1->addChild(new_lodrings);
            gp_root1->addChild(new_frustumsurfpoly);
            gp_root1->addChild(new_lodsurfpoly);
            gp_root1->addChild(new_pt_0_0);
            gp_root1->addChild(new_pt_90_0);
            gp_root1->addChild(new_debug);
        }
        viewer.frame();
    }
    return 0;
}

