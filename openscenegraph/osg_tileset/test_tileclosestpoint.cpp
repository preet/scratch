#include <iostream>
#include <iomanip>
#include <chrono>
#include <cassert>

#include <OSGUtils.h>
#include <TileSetLLByPixelArea.h>
#include <DataSetTilesLL.h>
#include <osg/MatrixTransform>

osg::Vec4 const K_CYAN = osg::Vec4(0,1,1,1);
osg::Vec4 const K_MAGENTA = osg::Vec4(1,0,1,1);
osg::Vec4 const K_YELLOW = osg::Vec4(1,1,0,1);
osg::Vec4 const K_BLUE = osg::Vec4(0,0,1,1);
osg::Vec4 const K_GREEN = osg::Vec4(0,1,0,1);
osg::Vec4 const K_RED = osg::Vec4(1,0,0,1);
osg::Vec4 const K_RBLUE = osg::Vec4(0,0.35,0.75,1.0);
osg::Vec4 const K_RGREEN = osg::Vec4(0.18,0.71,0.227,1.0);
osg::Vec4 const K_DARK_GRAY = osg::Vec4(0.1,0.1,0.1,1.0);
osg::Vec4 const K_GRAY = osg::Vec4(0.2,0.2,0.2,1.0);
osg::Vec4 const K_LIGHT_GRAY = osg::Vec4(0.5,0.5,0.5,1.0);
osg::Vec4 const K_WHITE = osg::Vec4(1.0,1.0,1.0,1.0);

double const K_TILE_MIN_LON = 0;
double const K_TILE_MAX_LON = 5.625;
double const K_TILE_MIN_LAT = 0;
double const K_TILE_MAX_LAT = 5.625;

double CalcMinPointLineDistance(osg::Vec3d const &pt,
                                osg::Vec3d const &a,
                                osg::Vec3d const &b,
                                bool clamp=false)
{
    static const double k_eps = 1E-8;

    osg::Vec3d const dirn = b-a;

    double const u_den = dirn*dirn;
    if(fabs(u_den) < k_eps) {
        return (pt-a).length();
    }

    double const u = ((pt*dirn)-(a*dirn))/u_den;

    if(clamp) {
        if(u < 0) {
            return (pt-a).length();
        }
        else if(u > 1) {
            return (pt-b).length();
        }
    }

    return (pt-(a+(dirn*u))).length();
}

osg::Vec3d CalcPointLineProjection(osg::Vec3d const &pt,
                                   osg::Vec3d const &a,
                                   osg::Vec3d const &b,
                                   bool clamp=false)
{
    static const double k_eps = 1E-8;

    osg::Vec3d const dirn = b-a;

    double const u_den = dirn*dirn;
    if(fabs(u_den) < k_eps) {
        return a;
    }

    double const u = ((pt*dirn)-(a*dirn))/u_den;

    if(clamp) {
        if(u < 0) {
            return a;
        }
        else if(u > 1) {
            return b;
        }
    }

    return (a+(dirn*u));
}

inline void CalcTilePlanes(GeoBounds const &tile_bounds,
                           Plane &plane_min_lon,
                           Plane &plane_max_lon,
                           Plane &plane_min_lat,
                           Plane &plane_max_lat)
{
    plane_min_lon = CalcLonPlane(tile_bounds.minLon,true,true);
    plane_max_lon = CalcLonPlane(tile_bounds.maxLon,false,true);
    plane_min_lat = CalcLatPlane(tile_bounds.minLat,false);
    plane_max_lat = CalcLatPlane(tile_bounds.maxLat,true);
}

inline bool CalcPointWithinTilePlanes(osg::Vec3d const &point,
                                      Plane const &plane_min_lon,
                                      Plane const &plane_max_lon,
                                      Plane const &plane_min_lat,
                                      Plane const &plane_max_lat)
{
    static const double k_eps = 1E-5; // meters

    bool outside =
            (((point-plane_min_lon.p)*plane_min_lon.n) > k_eps) ||
            (((point-plane_max_lon.p)*plane_max_lon.n) > k_eps) ||
            (((point-plane_min_lat.p)*plane_min_lat.n) > k_eps) ||
            (((point-plane_max_lat.p)*plane_max_lat.n) > k_eps);

    return (!outside);
}

Circle CalcCircleForLatPlane(Plane const &plane_lat)
{
    Circle c;
    c.normal = plane_lat.n;
    c.center = CalcPointLineProjection(plane_lat.p,
                                       K_ZERO_VEC,
                                       osg::Vec3d(0,0,1),
                                       false);
    c.radius = (c.center-plane_lat.p).length();
    return c;
}

Circle CalcCircleForLonPlane(Plane const &plane_lon)
{
    Circle c;
    c.normal = plane_lon.n;
    c.center = K_ZERO_VEC;
    c.radius = RAD_AV;

    return c;
}

osg::Vec3d CalcGeoBoundsClosestPoint(LLA const &lla_distal,
                                     osg::Vec3d const &ecef_distal,
                                     GeoBounds const &bounds)
{
    // If the LLA of the distal point falls within
    // the bounds, return the corresponding ecef
    if(CalcWithinGeoBounds(bounds,lla_distal)) {
        LLA lla_closest = lla_distal;
        lla_closest.alt = 0.0;
        return ConvLLAToECEF(lla_closest);
    }

    // Generate a list of possible closest points from
    // tile corner points and tile edges
    std::vector<osg::Vec3d> list_closest;

    // Add the corner points of the tile
    list_closest.push_back(ConvLLAToECEF(LLA(bounds.minLon,bounds.minLat)));
    list_closest.push_back(ConvLLAToECEF(LLA(bounds.maxLon,bounds.minLat)));
    list_closest.push_back(ConvLLAToECEF(LLA(bounds.maxLon,bounds.maxLat)));
    list_closest.push_back(ConvLLAToECEF(LLA(bounds.minLon,bounds.maxLat)));

    // Add the closest points on the great circle arc
    // edges for each tile (and filter out points that
    // aren't along the edges of hte tile)
    Plane const plane_min_lon = CalcLonPlane(bounds.minLon,true,true);
    Plane const plane_max_lon = CalcLonPlane(bounds.maxLon,false,true);
    Plane const plane_min_lat = CalcLatPlane(bounds.minLat,false);
    Plane const plane_max_lat = CalcLatPlane(bounds.maxLat,true);

    std::vector<Circle> list_circles;
    list_circles.reserve(4);
    list_circles.push_back(CalcCircleForLonPlane(plane_min_lon));
    list_circles.push_back(CalcCircleForLonPlane(plane_max_lon));
    list_circles.push_back(CalcCircleForLatPlane(plane_min_lat));
    list_circles.push_back(CalcCircleForLatPlane(plane_max_lat));

    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

    for(auto const &circle : list_circles) {
        osg::Vec3d closest = ClosestPointCirclePoint(circle,ecef_distal);
        if(CalcPointWithinTilePlanes(closest,
                                     plane_min_lon,
                                     plane_max_lon,
                                     plane_min_lat,
                                     plane_max_lat))
        {
            list_closest.push_back(closest);
        }
    }

    // Calculate all the dist2s for the closest points
    std::vector<double> list_closest_dist2;
    list_closest_dist2.reserve(list_closest.size());
    for(auto const &closest : list_closest) {
        list_closest_dist2.push_back((ecef_distal-closest).length2());
    }

    double abs_closest_dist2 = list_closest_dist2[0];
    osg::Vec3d abs_closest = list_closest[0];

    for(size_t i=1; i < list_closest_dist2.size(); i++) {
        if(list_closest_dist2[i] < abs_closest_dist2) {
            abs_closest_dist2 = list_closest_dist2[i];
            abs_closest = list_closest[i];
        }
    }

    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end-start;
//    std::cout << "took: " << elapsed_seconds.count()*1000.0 << " ms" << std::endl;

    return abs_closest;
}

double CalcPixelsPerMeter(double const dist,
                          double const screen_height_px,
                          osg::Camera const * cam)
{
    osg::Vec3d eye,vpt,up;
    cam->getViewMatrixAsLookAt(eye,vpt,up);

    double fovy_degs,ar,znear,zfar;
    cam->getProjectionMatrixAsPerspective(fovy_degs,ar,znear,zfar);

    double fovy_rads = fovy_degs * K_PI/180.0;

    double px_per_m = screen_height_px/(2*dist*tan(fovy_rads*0.5));
    return px_per_m;
}


int main()
{
    std::cout << std::fixed;
    std::cout << std::setprecision(8);

    // View0 root
    osg::ref_ptr<osg::Group> gp_root0 = new osg::Group;

    // View1 root
    osg::ref_ptr<osg::Group> gp_root1 = new osg::Group;

    // =============================================================== //

    // debug
    GeoBounds debug_bounds(-180.0,180.0,-90.0,90.0);
//    GeoBounds debug_bounds(0.0,360.0/pow(2,0),0.0,360.0/pow(2,1));
    double debug_area_m2 = CalcGeoBoundsArea(debug_bounds);
    double debug_mpx = debug_area_m2/(256.0*256.0); // m^2/tx
//    std::cout << "mpx: " << sqrt(debug_mpx) << std::endl;

    // =============================================================== //

    // earth surface geometry
    auto gp_earth = BuildEarthSurfaceNode("earth",K_GRAY);
    gp_root0->addChild(gp_earth);
    gp_root1->addChild(gp_earth);

    // axes
    auto gp_axes = BuildAxesGeometry("axes",RAD_AV);
    gp_root0->addChild(gp_axes);
    gp_root1->addChild(gp_axes);

    // tile
    GeoBounds tile_bounds(K_TILE_MIN_LON,
                          K_TILE_MAX_LON,
                          K_TILE_MIN_LAT,
                          K_TILE_MAX_LAT);
    auto gp_tile = BuildGeoBoundsSurfaceNode(
                "tile",tile_bounds,K_RBLUE,1);
    gp_root0->addChild(gp_tile);
    gp_root1->addChild(gp_tile);
    double const tile_area_m2 = CalcGeoBoundsArea(tile_bounds);

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
        view->getCamera()->setClearColor(osg::Vec4(0.1,0.1,0.1,1.0));

        osg::ref_ptr<osgGA::TrackballManipulator> view_manip =
                new osgGA::TrackballManipulator;
//        view_manip->setMinimumDistance(100);

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
//        view_manip->setMinimumDistance(100);

        view->setCameraManipulator(view_manip);
    }

    // =============================================================== //

    std::cout << "[starting render loop...]" << std::endl;

    while(!viewer.done())
    {
        // current camera params
        osg::Vec3d eye,vpt,up;
        osg::Camera * camera = viewer.getView(0)->getCamera();
        camera->getViewMatrixAsLookAt(eye,vpt,up);

        osg::Vec3d vdn = vpt-eye;
        vdn.normalize();

        LLA lla_eye = ConvECEFToLLA(eye);

        // ============================================================ //

        // new frustum node
        Frustum frustum;
        auto new_frustum = BuildFrustumNode(
                    "frustum",camera,frustum);

        // new closestpoint node
        osg::Vec3d closestpoint =
                CalcGeoBoundsClosestPoint(lla_eye,eye,tile_bounds);
        auto new_closestpoint = BuildFacingCircleNode(
                    "closestpoint",closestpoint,50000,6,K_RED);

        // ============================================================ //

        double px_m = CalcPixelsPerMeter((eye-closestpoint).length(),
                                         480,
                                         camera);

        double px2_m2 = px_m*px_m;
        double tile_px_area = tile_area_m2*px2_m2;
        std::cout << "#: tile_px_area: " << tile_px_area << std::endl;

        // Update gp_root0
        for(size_t i=0; i < gp_root0->getNumChildren(); i++)
        {
            std::string const name =
                    gp_root0->getChild(i)->getName();

            if(name == "closestpoint") {
                gp_root0->removeChild(i);
                i--;
            }
        }
        gp_root0->addChild(new_closestpoint);

        // Update gp_root1
        for(size_t i=0; i < gp_root1->getNumChildren(); i++)
        {
             std::string const name =
                     gp_root1->getChild(i)->getName();

             if(name == "frustum") {
                 gp_root1->removeChild(i);
                 i--;
             }
             else if(name == "closestpoint") {
                 gp_root1->removeChild(i);
                 i--;
             }
        }
        gp_root1->addChild(new_frustum);
        gp_root1->addChild(new_closestpoint);


        viewer.frame();
    }

    std::cout << "[end]" << std::endl;
    return 0;
}
