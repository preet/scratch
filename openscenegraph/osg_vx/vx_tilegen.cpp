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
#include <tileutil.hpp>

// From Real-Time Collision Detection p. 210
bool CalcPlanePlaneIntersection(Plane const &p1,
                                Plane const &p2,
                                osg::Vec3d &xsec_p,
                                osg::Vec3d &xsec_d)
{
    // xsec line direction
    xsec_d = p1.n^p2.n;

    // if d is ~0, the planes are parallel
    double denom = xsec_d*xsec_d;
    if(denom < 1E-3) {
        return false;
    }

    // xsec line point
    xsec_p = ((p2.n*p1.d - p1.n*p2.d)^xsec_d) / denom;
    return true;
}

bool CalcCameraNearFarDist(osg::Vec3d const &eye,
                           osg::Vec3d const &view_dirn,
                           double const pin_surf_dist_m,
                           double &dist_near,
                           double &dist_far)
{
    double const radius = RAD_AV;
    double const eye_dist2 = eye.length2();

    if(eye_dist2 > (radius*radius)) {
        // The near distance is set to the length of the
        // the line segment of the eye projected onto
        // the plane with n==view_dirn and p==(0,0,0)
        // less the radius
        Plane plane;
        plane.n = (view_dirn*-1.0);
        plane.n.normalize();
        plane.p = osg::Vec3d(0,0,0);
        plane.d = (plane.n*plane.p);

        osg::Vec3d xsec = CalcPointPlaneProjection(eye,plane);
        dist_near = (eye-xsec).length() - (radius+pin_surf_dist_m);

        if(dist_near < 1.0) {
            dist_near = 1.0;
        }

        // The far distance is set to the tangential distance
        // from the eye to the horizon (see horizon plane)
        dist_far = sqrt(eye_dist2 - radius*radius);
        return true;
    }
    return false;
}

void CalcProjFrustumPoly(Frustum const &frustum,
                         Plane const &horizon_plane,
                         std::vector<osg::Vec3d> &list_ecef)
{
    // Check if the planet is in the view frustum
    if(CalcSphereOutsideFrustumExact(frustum,osg::Vec3d(0,0,0),RAD_AV)) {
        return;
    }

    // Use 8 points on the view frustum edges to find
    // the LLA polygon:
    std::vector<osg::Vec3d> list_ecef_xsec(8);
    list_ecef_xsec[0] = frustum.list_vx[4]; // BL
    list_ecef_xsec[1] = (frustum.list_vx[4]+frustum.list_vx[5])*0.5;
    list_ecef_xsec[2] = frustum.list_vx[5]; // BR
    list_ecef_xsec[3] = (frustum.list_vx[5]+frustum.list_vx[6])*0.5;
    list_ecef_xsec[4] = frustum.list_vx[6]; // TR
    list_ecef_xsec[5] = (frustum.list_vx[6]+frustum.list_vx[7])*0.5;
    list_ecef_xsec[6] = frustum.list_vx[7]; // TL
    list_ecef_xsec[7] = (frustum.list_vx[7]+frustum.list_vx[4])*0.5;

    list_ecef.clear();

    osg::Vec3d const far_center = (frustum.list_vx[4]+frustum.list_vx[6])*0.5;
    osg::Vec3d const view_dirn = (far_center-frustum.eye);

    for(size_t i=0; i < list_ecef_xsec.size(); i++)
    {
        osg::Vec3d const &ray_pt = frustum.eye;
        osg::Vec3d const ray_dirn = list_ecef_xsec[i]-frustum.eye;

        osg::Vec3d xsec_near,xsec_far;
        if(CalcRayEarthIntersection(ray_pt,
                                    ray_dirn,
                                    xsec_near,
                                    xsec_far))
        {
            double const u_near = (xsec_near-ray_pt)*ray_dirn;
            double const u_far = (xsec_far-ray_pt)*ray_dirn;
            if((u_near > 0) && (u_far > 0))
            {
                if((xsec_near-frustum.eye)*view_dirn > 0.0) {
                    list_ecef_xsec[i] = xsec_near;
                }
                else {
                    list_ecef_xsec[i] = xsec_far;
                }
                list_ecef.push_back(list_ecef_xsec[i]);
                continue;
            }
        }
        // Project rays that missed the planetary
        // body onto the horizon plane
        osg::Vec3d const ecef_horizon =
                CalcPointPlaneProjection(list_ecef_xsec[i],
                                         horizon_plane);

        if(!CalcRayEarthIntersection(ecef_horizon,
                                     horizon_plane.p - ecef_horizon,
                                     xsec_near,
                                     xsec_far))
        {
            // should never get here
            std::cout << "###: FATAL" << std::endl;
            return;
        }

        list_ecef_xsec[i] = xsec_near;
    }

    list_ecef = list_ecef_xsec;
}

struct GeoTileXSecPoly
{
    std::vector<Edge> list_edges;
    std::vector<osg::Vec3d> list_add_vx;
};

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
        return;
    }

    // Get midLat plane
    Plane plane_lat;
    plane_lat.n = osg::Vec3d(0,0,1);
    plane_lat.p = tile->ecef_MM;
    plane_lat.d = plane_lat.n*plane_lat.p;

    // Get midLon plane
    Plane plane_lon;
    plane_lon.n = (tile->ecef_MM)^(osg::Vec3d(0,0,1));
    plane_lon.p = tile->ecef_MM;
    plane_lon.d = plane_lon.n*plane_lon.p;

    // Intersect each polygon edge with plane_lat and plane_lon
    for(size_t i=0; i < list_poly_edges.size(); i++) {

        Edge const &edge = list_poly_edges[i];

        // plane_lat
        {
            // Check if the edge intersects the plane
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
                list_lla.push_back(ConvECEFToLLA(xsec_pt));
            }
        }

        // plane_lon
        {
            // Check if the edge intersects the plane
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
    }

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

    list_used_lats.insert(tile->midLat);

//    for(auto const &lla : list_lla) {
//        std::cout << "###: lla: " << lla.lon << ", " << lla.lat << std::endl;
//    }

    // quadrant 0 - LT
    bool outside;
    for(auto const &lla : list_lla) {
        outside = lla.lon < tile->minLon-K_GT_EPS ||
                  lla.lon > tile->midLon+K_GT_EPS ||
                  lla.lat < tile->midLat-K_GT_EPS ||
                  lla.lat > tile->maxLat+K_GT_EPS;
        if(!outside) {
            tile->tile_LT = BuildChildTile(tile,0);
            GenTilesForPoly(list_poly_edges,
                            max_level,
                            list_lla,
                            list_used_lons,
                            list_used_lats,
                            tile->tile_LT.get());
            break;
        }
    }

    // quadrant 1 - LB
    for(auto const &lla : list_lla) {
        outside = lla.lon < tile->minLon-K_GT_EPS ||
                  lla.lon > tile->midLon+K_GT_EPS ||
                  lla.lat < tile->minLat-K_GT_EPS ||
                  lla.lat > tile->midLat+K_GT_EPS;
        if(!outside) {
            tile->tile_LB = BuildChildTile(tile,1);
            GenTilesForPoly(list_poly_edges,
                            max_level,
                            list_lla,
                            list_used_lons,
                            list_used_lats,
                            tile->tile_LB.get());
            break;
        }
    }

    // quadrant 2 - RB
    for(auto const &lla : list_lla) {
        outside = lla.lon < tile->midLon-K_GT_EPS ||
                  lla.lon > tile->maxLon+K_GT_EPS ||
                  lla.lat < tile->minLat-K_GT_EPS ||
                  lla.lat > tile->midLat+K_GT_EPS;
        if(!outside) {
            tile->tile_RB = BuildChildTile(tile,2);
            GenTilesForPoly(list_poly_edges,
                            max_level,
                            list_lla,
                            list_used_lons,
                            list_used_lats,
                            tile->tile_RB.get());
            break;
        }
    }

    // quadrant 3 - RT
    for(auto const &lla : list_lla) {
        outside = lla.lon < tile->midLon-K_GT_EPS ||
                  lla.lon > tile->maxLon+K_GT_EPS ||
                  lla.lat < tile->midLat-K_GT_EPS ||
                  lla.lat > tile->maxLat+K_GT_EPS;
        if(!outside) {
            tile->tile_RT = BuildChildTile(tile,3);
            GenTilesForPoly(list_poly_edges,
                            max_level,
                            list_lla,
                            list_used_lons,
                            list_used_lats,
                            tile->tile_RT.get());
            break;
        }
    }
}








int main()
{
    std::unique_ptr<VxTile> root_tile(BuildRootTile(0,-180,-90,180,90));

    std::vector<PointLLA> list_lla;
    list_lla.emplace_back(170,70);
    list_lla.emplace_back(-180,65);
    list_lla.emplace_back(-170,70);
    list_lla.emplace_back(-180,75);

    std::vector<osg::Vec3d> poly_temp;
    poly_temp.push_back(ConvLLAToECEF(list_lla[0]));
    poly_temp.push_back(ConvLLAToECEF(list_lla[1]));
    poly_temp.push_back(ConvLLAToECEF(list_lla[2]));
    poly_temp.push_back(ConvLLAToECEF(list_lla[3]));

    std::vector<Edge> list_poly_edges;
    for(size_t i=1; i < poly_temp.size(); i++) {
        Edge edge;
        edge.a = poly_temp[i-1];
        edge.dirn_ab = poly_temp[i]-edge.a;
        list_poly_edges.push_back(edge);
    }
    // last edge
    {
        Edge edge;
        edge.a = poly_temp[poly_temp.size()-1];
        edge.dirn_ab = poly_temp[0]-edge.a;
        list_poly_edges.push_back(edge);
    }

    std::set<double> list_used_lons;
    std::set<double> list_used_lats;

    GenTilesForPoly(list_poly_edges,
                    6,
                    list_lla,
                    list_used_lons,
                    list_used_lats,
                    root_tile.get());

    auto gp_poly = BuildSurfacePoly(poly_temp,osg::Vec4(1,1,1,1));

    osg::ref_ptr<osg::Group> gp_vx = new osg::Group;
    BuildViewExtentsGeometryAsPolyLines(root_tile.get(),gp_vx.get());


    Frustum frustum;
    Plane horizon_plane;

    // Celestial body geometry
    auto gp_celestial = BuildCelestialSurfaceNode();

    osgViewer::CompositeViewer viewer;
    viewer.setThreadingModel(osgViewer::ViewerBase::SingleThreaded);

    // View0 root
    osg::ref_ptr<osg::Group> gp_root0 = new osg::Group;
    gp_root0->addChild(gp_celestial);
    gp_root0->addChild(gp_poly);
    gp_root0->addChild(gp_vx);

    // View1 root
    osg::ref_ptr<osg::Group> gp_root1 = new osg::Group;
    gp_root1->addChild(gp_celestial);
    gp_root1->addChild(gp_poly);
    gp_root1->addChild(gp_vx);

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

    // Create root tile
//    std::unique_ptr<VxTile> root_tile(BuildRootTile(0,-180,-90,180,90));

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

        // new mindcamdistline
        auto new_mincamdistline = BuildMinCamDistLineNode(eye);
        new_mincamdistline->getOrCreateStateSet()->setMode(
                    GL_DEPTH_TEST,
                    osg::StateAttribute::ON |
                    osg::StateAttribute::OVERRIDE);

        osg::ref_ptr<osg::Group> new_frustumsurfpoly;
        osg::ref_ptr<osg::Group> new_00pt;
        osg::ref_ptr<osg::Group> new_900pt;

        if(eye.length() > RAD_AV) {
            // new frustumsurfproj
            std::vector<osg::Vec3d> poly_frustum_surf;
            CalcProjFrustumPoly(frustum,horizon_plane,poly_frustum_surf);
            new_frustumsurfpoly = BuildSurfacePoly(poly_frustum_surf,
                                                   osg::Vec4(0.75,0.5,1.0,1.0),
                                                   eye.length()/1200.0);

            // new 00pt
            new_00pt = BuildFacingCircle(ConvLLAToECEF(PointLLA(0,0)),
                                         eye.length()/200.0,
                                         8,
                                         osg::Vec4(1,0,0,1));

            new_900pt = BuildFacingCircle(ConvLLAToECEF(PointLLA(90,0)),
                                          eye.length()/200.0,
                                          8,
                                          osg::Vec4(0,1,0,1));
        }
        else {
            new_frustumsurfpoly = new osg::Group;
            new_00pt = new osg::Group;
            new_900pt = new osg::Group;
        }

        new_frustumsurfpoly->setName("frustumsurfpoly");
        new_00pt->setName("00pt");
        new_900pt->setName("900pt");

        // Update gp_root0
        {
            for(size_t i=0; i < gp_root0->getNumChildren(); i++) {
                std::string const name = gp_root0->getChild(i)->getName();
                if(name == "horizonplane") {
                    gp_root0->removeChild(i);
                    i--;
                }
                else if(name == "frustumsurfpoly") {
                    gp_root0->removeChild(i);
                    i--;
                }
                else if(name == "00pt") {
                    gp_root0->removeChild(i);
                    i--;
                }
                else if(name =="900pt") {
                    gp_root0->removeChild(i);
                    i--;
                }
            }
            gp_root0->addChild(new_horizon);
            gp_root0->addChild(new_frustumsurfpoly);
            gp_root0->addChild(new_00pt);
            gp_root0->addChild(new_900pt);
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
                else if(name == "horizonplane") {
                    gp_root1->removeChild(i);
                    i--;
                }
                else if(name == "mincamdistline") {
                    gp_root1->removeChild(i);
                    i--;
                }
                else if(name == "frustumsurfpoly") {
                    gp_root1->removeChild(i);
                    i--;
                }
                else if(name == "00pt") {
                    gp_root1->removeChild(i);
                    i--;
                }
                else if(name =="900pt") {
                    gp_root1->removeChild(i);
                    i--;
                }
            }
            // Add new nodes
            gp_root1->addChild(new_frustum);
            gp_root1->addChild(new_horizon);
            gp_root1->addChild(new_mincamdistline);
            gp_root1->addChild(new_frustumsurfpoly);
            gp_root1->addChild(new_00pt);
            gp_root1->addChild(new_900pt);
        }

        viewer.frame();

    }
    return 0;
}

