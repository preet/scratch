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

osg::ref_ptr<osg::Group> BuildFrustumProjAsTrisNode(std::string const &name,
                                                    std::vector<osg::Vec3d> const &list_frustum_vx)
{
    if(list_frustum_vx.size() != 8) {
        osg::ref_ptr<osg::Group> gp = new osg::Group;
        return gp;
    }

    // 0,1,7
    static const std::vector<uint16_t> list_ix = {
        0,1,7,
        1,2,3,
        3,4,5,
        5,6,7,
        5,7,1,
        5,1,3
    };

    static const std::vector<osg::Vec4> list_cx = {
        K_RED,K_RED,K_RED,
        K_GREEN,K_GREEN,K_GREEN,
        K_BLUE,K_BLUE,K_BLUE,
        K_CYAN,K_CYAN,K_CYAN,
        K_MAGENTA,K_MAGENTA,K_MAGENTA,
        K_YELLOW,K_YELLOW,K_YELLOW,
    };

    osg::ref_ptr<osg::Vec3dArray> vx_array =
            new osg::Vec3dArray;
    for(auto const &ix : list_ix) {
        vx_array->push_back(list_frustum_vx[ix]);
    }

    osg::ref_ptr<osg::Vec4Array> cx_array =
            new osg::Vec4Array;
    for(auto const &cx : list_cx) {
        cx_array->push_back(cx);
    }

    osg::ref_ptr<osg::Geometry> gm = new osg::Geometry;
    gm->setVertexArray(vx_array);
    gm->setColorArray(cx_array,osg::Array::BIND_PER_VERTEX);
    gm->addPrimitiveSet(new osg::DrawArrays(GL_TRIANGLES,0,vx_array->size()));

    osg::ref_ptr<osg::Geode> gd = new osg::Geode;
    gd->addDrawable(gm);

    osg::ref_ptr<osg::Group> gp = new osg::Group;
    gp->addChild(gd);
    gp->setName(name);

    return gp;
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

    // plane_hemisphere is the plane with a normal vector
    // that points away from mid_lon, with a point at (0,0,0)

    double mid_lon = (tile_bounds.minLon+tile_bounds.maxLon)*0.5;
    LLA lla_hs;
    lla_hs.lon = mid_lon;
    lla_hs.lat = 0.0;
    lla_hs.alt = 0.0;
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

// expect tris in CCW order -- normals are facing outward
std::vector<Plane>
CalcFrustumPolyTriPlanes(std::vector<osg::Vec3d> const &list_frustum_vx,
                         bool normalize)
{

    static const std::vector<uint16_t> list_ix = {
        0,1,7,
        1,2,3,
        3,4,5,
        5,6,7,
        5,7,1,
        5,1,3
    };

    std::vector<Plane> list_tri_planes;
    list_tri_planes.reserve(18);

    for(size_t i=0; i < list_ix.size(); i+=3) {
        // Each plane is the plane that goes through
        // the points of a triangle edge, and (0,0,0)
        // since each edge represents a great arc

        osg::Vec3d const &v0 = list_frustum_vx[list_ix[i+0]];
        osg::Vec3d const &v1 = list_frustum_vx[list_ix[i+1]];
        osg::Vec3d const &v2 = list_frustum_vx[list_ix[i+2]];

        Plane plane0;
        plane0.n = (v1-v0)^v0;
        if(normalize) {
            plane0.n.normalize();
        }
        plane0.p = v0;
        plane0.d = plane0.n*plane0.p;

        Plane plane1;
        plane1.n = (v2-v1)^v1;
        if(normalize) {
            plane1.n.normalize();
        }
        plane1.p = v1;
        plane1.d = plane1.n*plane1.p;

        Plane plane2;
        plane2.n = (v0-v2)^v2;
        if(normalize) {
            plane2.n.normalize();
        }
        plane2.p = v2;
        plane2.d = plane2.n*plane2.p;

        list_tri_planes.push_back(plane0);
        list_tri_planes.push_back(plane1);
        list_tri_planes.push_back(plane2);
    }

    return list_tri_planes;
}

std::vector<Plane>
CalcFrustumEdgePlanes(std::vector<osg::Vec3d> const &list_frustum_vx,
                      bool normalize)
{
    std::vector<Plane> list_edge_planes;
    list_edge_planes.reserve(list_frustum_vx.size());

    for(size_t i=1; i < list_frustum_vx.size(); i++) {
        // Each plane is the plane that goes through
        // the points of a single edge, and (0,0,0)
        // since each edge represents a great arc
        osg::Vec3d const &v0 = list_frustum_vx[i-1];
        osg::Vec3d const &v1 = list_frustum_vx[i-0];

        // Normal faces outward
        Plane plane0;
        plane0.n = (v1-v0)^v0;
        if(normalize) {
            plane0.n.normalize();
        }
        plane0.p = v0;
        plane0.d = plane0.n*plane0.p;

        list_edge_planes.push_back(plane0);
    }

    return list_edge_planes;
}

bool CalcFrustumTileIntersection(std::vector<osg::Vec3d> const &list_frustum_vx,
                                 std::vector<GeoBounds> const &list_frustum_bounds,
                                 std::vector<Plane> const &list_frustum_tri_planes,
                                 GeoBounds const &tile_bounds)
{
    // TODO determine a good tolerance (this is in meters)
    static const double k_eps = 0.0;

    if(list_frustum_vx.size() != 8) {
        return false;
    }

    // Three tests:
    // 0. GeoBounds intersection
    //  * if the geobounds of the frustum poly and tile
    //    do not intersect, there is no intersection

    bool xsec = false;
    GeoBounds xsec_bounds;
    for(auto const &frustum_bounds : list_frustum_bounds) {
        if(CalcGeoBoundsIntersection(frustum_bounds,
                                     tile_bounds,
                                     xsec_bounds))
        {
            if(xsec_bounds == frustum_bounds) {
                // Frustum poly is within the tile
                std::cout << "#: XSEC TYPE 1" << std::endl;
                return true;
            }

            xsec = true;
            break;
        }
    }

    if(!xsec) {
        return false;
    }

    // Check to see if the frustum poly edges intersect
    // with any tile planes

    // TODO tile planes should be part of a cached
    //      Eval structure
    Plane plane_min_lon;
    Plane plane_max_lon;
    Plane plane_min_lat;
    Plane plane_max_lat;
    CalcTilePlanes(tile_bounds,
                   plane_min_lon,
                   plane_max_lon,
                   plane_min_lat,
                   plane_max_lat);

    size_t xsec_count;
    std::vector<osg::Vec3d> list_xsec,list_xsec_min_lon;
    list_xsec.reserve(list_frustum_vx.size());

    // TODO:
    // Would it be faster to compute intersections
    // against all the tile edge planes everytime?

    // min_lon
    {
        xsec_count = CalcPlanePolyIntersection(plane_min_lon,
                                               list_frustum_vx,
                                               list_xsec);
        for(size_t i=0; i < xsec_count; i++) {
            if(CalcPointWithinTilePlanes(list_xsec[list_xsec.size()-1-i],
                                         plane_min_lon,
                                         plane_max_lon,
                                         plane_min_lat,
                                         plane_max_lat)) {
                std::cout << "#: XSEC TYPE 2" << std::endl;
                return true;
            }
        }

        // Save the xsec points for the next test if required
        list_xsec_min_lon = list_xsec;
    }

    // max_lon
    {
        xsec_count = CalcPlanePolyIntersection(plane_max_lon,
                                               list_frustum_vx,
                                               list_xsec);

        for(size_t i=0; i < xsec_count; i++) {
            if(CalcPointWithinTilePlanes(list_xsec[list_xsec.size()-1-i],
                                         plane_min_lon,
                                         plane_max_lon,
                                         plane_min_lat,
                                         plane_max_lat)) {
                std::cout << "#: XSEC TYPE 2" << std::endl;
                return true;
            }
        }
    }

    // max_lat
    {
        xsec_count = CalcPlanePolyIntersection(plane_min_lat,
                                               list_frustum_vx,
                                               list_xsec);

        for(size_t i=0; i < xsec_count; i++) {
            if(CalcPointWithinTilePlanes(list_xsec[list_xsec.size()-1-i],
                                         plane_min_lon,
                                         plane_max_lon,
                                         plane_min_lat,
                                         plane_max_lat)) {
                std::cout << "#: XSEC TYPE 2" << std::endl;
                return true;
            }
        }
    }

    // min_lat
    {
        xsec_count = CalcPlanePolyIntersection(plane_max_lat,
                                               list_frustum_vx,
                                               list_xsec);

        for(size_t i=0; i < xsec_count; i++) {
            if(CalcPointWithinTilePlanes(list_xsec[list_xsec.size()-1-i],
                                         plane_min_lon,
                                         plane_max_lon,
                                         plane_min_lat,
                                         plane_max_lat)) {
                std::cout << "#: XSEC TYPE 2" << std::endl;
                return true;
            }
        }
    }

    // Check to see if the frustum poly completely
    // contains the tile (the previous test verified
    // the the tile isn't partially contained)

    // We have two ways to do this:

    // 1. Test a single point using point-in-poly with
    //    a longitude line instead of a ray

    // 2. Test the triangulated frustum poly's triangle
    //    edge planes to see if they contain a point

    // 1 seems more error prone. 2 is easier to implement.
    // Both seem to be working ok, keeping 1 for reference
    // even though 2 is being used

    // == 1 == //

//    if((list_frustum_bounds[0].minLat == -90.0) &&
//       (list_frustum_bounds[0].maxLat == 90.0))
//    {
//        // If the lat range is -90 to 90, we cant do the
//        // test; assume tile is within the frustum poly
//        std::cout << "#: XSEC TYPE 3_FULL_LAT_RANGE";
//        return true;
//    }

//    // min_lon's hemisphere plane
//    Plane plane_hs;
//    plane_hs.n = ConvLLAToECEF(LLA(tile_bounds.minLon,0.0,0.0));
//    plane_hs.n.normalize();
//    plane_hs.p = K_ZERO_VEC;
//    plane_hs.d = plane_hs.n*plane_hs.p;

//    if(list_frustum_bounds[0].minLat > -90.0) {
//        // Get the number of intersections from
//        // -90 to tile.min_lat
//        size_t xsec_crossings=0;
//        for(auto const &xsec : list_xsec_min_lon) {
//            // The xsec is a crossing if it is in front of
//            // min_lon's hemisphere and below the min_lat plane
//            bool valid_crossing =
//                    (((xsec-plane_hs.p)*plane_hs.n) >= 0) &&
//                    (((xsec-plane_min_lat.p)*plane_min_lat.n) > 0);

//            if(valid_crossing) {
//                xsec_crossings++;
//            }
//        }
//        // An odd num of crossings means the point
//        // is inside the poly
//        if(!(xsec_crossings%2 == 0)) {
//            std::cout << "#: XSEC TYPE 3_1" << std::endl;
//            return true;
//        }
//    }
//    else {
//        // Get the number of intersections from
//        // +90 to tile.max_lat
//        size_t xsec_crossings=0;
//        for(auto const &xsec : list_xsec_min_lon) {
//            // The xsec is a crossing if it is in front of
//            // min_lon's hemisphere and above the max_lat plane
//            bool valid_crossing =
//                    (((xsec-plane_hs.p)*plane_hs.n) >= 0) &&
//                    (((xsec-plane_max_lat.p)*plane_max_lat.n) > 0);

//            if(valid_crossing) {
//                xsec_crossings++;
//            }
//        }
//        // An odd num of crossings means the point
//        // is inside the poly
//        if(!(xsec_crossings%2 == 0)) {
//            std::cout << "#: XSEC TYPE 3_2" << std::endl;
//            return true;
//        }
//    }

    // == 2 == //

    // Check to see if any of the triangle regions of
    // the frustum poly contains a point from the tile
    {
        // Test point from tile
        osg::Vec3d const ecef = ConvLLAToECEF(
                    LLA((tile_bounds.minLon+tile_bounds.maxLon)*0.5,
                        (tile_bounds.minLat+tile_bounds.maxLat)*0.5));

        for(size_t i=0; i < list_frustum_tri_planes.size(); i+=3) {
            Plane const &plane0 = list_frustum_tri_planes[i+0];
            Plane const &plane1 = list_frustum_tri_planes[i+1];
            Plane const &plane2 = list_frustum_tri_planes[i+2];

            bool outside =
                    ((ecef-plane0.p)*plane0.n > k_eps) ||
                    ((ecef-plane1.p)*plane1.n > k_eps) ||
                    ((ecef-plane2.p)*plane2.n > k_eps);

            if(!outside) {
                std::cout << "#: XSEC_TYPE 3_" << i/3 << std::endl;
                return true;
            }
        }
    }

    return false;
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

    // earth surface geometry
    auto gp_earth = BuildEarthSurfaceNode("earth",K_GRAY);
    gp_root0->addChild(gp_earth);
    gp_root1->addChild(gp_earth);

    // axes
    auto gp_axes = BuildAxesGeometry("axes",RAD_AV);
    gp_root0->addChild(gp_axes);
    gp_root1->addChild(gp_axes);

    // tile
//    GeoBounds tile_bounds(-40,0,40,60);
    GeoBounds tile_bounds(-180,0,-90,-45);
    auto gp_tile = BuildGeoBoundsNode(
                "tile",tile_bounds,K_RBLUE);
    gp_root0->addChild(gp_tile);
    gp_root1->addChild(gp_tile);

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

    // =============================================================== //

    std::cout << "[starting render loop...]" << std::endl;

//    LLA lla0; lla0.lon = -170.0; lla0.lat = 20; lla0.alt = 0;
//    LLA lla1; lla1.lon = -170.0; lla1.lat = 30; lla1.alt = 0;
//    LLA lla2; lla2.lon = 180.0; lla2.lat = 30; lla2.alt = 0;
//    LLA lla3; lla3.lon = 180.0; lla3.lat = 20; lla3.alt = 0;
//    LLA lla4; lla4.lon = 180.0; lla4.lat = 90; lla4.alt = 0;

//    std::vector<LLA> list_test_lla;
//    list_test_lla.push_back(lla0);
//    list_test_lla.push_back(lla1);
//    list_test_lla.push_back(lla2);
//    list_test_lla.push_back(lla3);
//    list_test_lla.push_back(lla4);

//    std::vector<GeoBounds> list_test_bounds;
//    CalcMinGeoBoundsFromLLAPoly(ConvECEFToLLA(osg::Vec3d(RAD_AV*1.5,0,0)),
//                                list_test_lla,
//                                list_test_bounds);

//    for(auto const &test_bounds : list_test_bounds) {
//        std::cout << "lon: [" << test_bounds.minLon << ", " << test_bounds.maxLon << "] "
//                  << "lat: [" << test_bounds.minLat << ", " << test_bounds.maxLat << std::endl;
//    }

    while(!viewer.done())
    {
        // current camera params
        osg::Vec3d eye,vpt,up;
        osg::Camera * camera = viewer.getView(0)->getCamera();
        camera->getViewMatrixAsLookAt(eye,vpt,up);

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

        Plane horizon_plane = CalcHorizonPlane(eye);

        // new frustum node
        Frustum frustum;
        auto new_frustum = BuildFrustumNode("frustum",
                                            camera,
                                            frustum,
                                            near_dist,
                                            far_dist);

        // calc frustum proj poly
        std::vector<osg::Vec3d> list_frustum_ecef;
        CalcProjFrustumPoly(frustum,horizon_plane,list_frustum_ecef);

        std::vector<LLA> list_frustum_lla =
                ConvListECEFToLLA(list_frustum_ecef);

        // calc frustum tri planes
        std::vector<Plane> list_tri_planes =
                CalcFrustumPolyTriPlanes(list_frustum_ecef,true);

        std::vector<Plane> list_edge_planes =
                CalcFrustumEdgePlanes(list_frustum_ecef,true);

        // calc frustum poly geo bounds
        std::vector<GeoBounds> list_frustum_bounds;
        CalcMinGeoBoundsFromLLAPoly(ConvECEFToLLA(eye),
                                    list_frustum_lla,
                                    list_frustum_bounds);

        uint8_t frustum_pole=0;
        for(auto const &bounds : list_frustum_bounds) {
            if(bounds.maxLat == 90.0) {
                frustum_pole = 1;
                break;
            }

            if(bounds.minLat == -90.0) {
                frustum_pole = 2;
                break;
            }
        }

        // ============================================================ //

        osg::ref_ptr<osg::Group> new_frustumtris = new osg::Group;
        new_frustumtris->setName("frustumtris");
        bool ftxsec = CalcFrustumTileIntersection(list_frustum_ecef,
                                                  list_frustum_bounds,
                                                  list_edge_planes,
                                                  tile_bounds);

        {
            osg::Vec4 const color = (ftxsec) ? K_RED : K_WHITE;
            auto gp_tri = BuildSurfacePolyNode(std::to_string(0),
                                               list_frustum_ecef,
                                               color);
            new_frustumtris->addChild(gp_tri);
        }

//        auto new_frustumtris = BuildFrustumProjAsTrisNode(
//                    "frustumtris",list_frustum_ecef);

        // ============================================================ //

        // Update gp_root0
        for(size_t i=0; i < gp_root0->getNumChildren(); i++)
        {
            std::string const name =
                    gp_root0->getChild(i)->getName();

            if(name == "frustum") {
                gp_root0->removeChild(i);
                i--;
            }
            else if(name == "frustumtris") {
                gp_root0->removeChild(i);
                i--;
            }
        }
        gp_root0->addChild(new_frustum);
        gp_root0->addChild(new_frustumtris);

        // Update gp_root1
        for(size_t i=0; i < gp_root1->getNumChildren(); i++)
        {
             std::string const name =
                     gp_root1->getChild(i)->getName();

             if(name == "frustum") {
                 gp_root1->removeChild(i);
                 i--;
             }
             else if(name == "frustumtris") {
                 gp_root1->removeChild(i);
                 i--;
             }
        }
        gp_root1->addChild(new_frustum);
        gp_root1->addChild(new_frustumtris);


        viewer.frame();
    }

    return 0;
}
