#include <iostream>
#include <iomanip>
#include <chrono>

#include <OSGUtils.h>
#include <TileSetLLByPixelArea.h>
#include <DataSetTilesLL.h>
#include <osg/MatrixTransform>

osg::Vec4 const K_BLUE = osg::Vec4(0,0,1,1);
osg::Vec4 const K_GREEN = osg::Vec4(0,1,0,1);
osg::Vec4 const K_RED = osg::Vec4(1,0,0,1);
osg::Vec4 const K_RBLUE = osg::Vec4(0,0.35,0.75,1.0);
osg::Vec4 const K_RGREEN = osg::Vec4(0.18,0.71,0.227,1.0);
osg::Vec4 const K_YELLOW = osg::Vec4(1,1,0,1);
osg::Vec4 const K_WHITE = osg::Vec4(1,1,1,1);
osg::Vec4 const K_DARK_GRAY = osg::Vec4(0.1,0.1,0.1,1.0);
osg::Vec4 const K_GRAY = osg::Vec4(0.2,0.2,0.2,1.0);
osg::Vec4 const K_LIGHT_GRAY = osg::Vec4(0.5,0.5,0.5,1.0);

void CalcTilePlanes(GeoBounds const &tile_bounds,
                    Plane &plane_min_lon,
                    Plane &plane_max_lon,
                    Plane &plane_min_lat,
                    Plane &plane_max_lat,
                    Plane &plane_hemisphere)
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

    plane_hemisphere.p = K_ZERO_VEC;
    plane_hemisphere.n = K_ZERO_VEC-ConvLLAToECEF(lla_hs);
    plane_hemisphere.n.normalize();
    plane_hemisphere.d = plane_hemisphere.n*plane_hemisphere.p;
}

bool CalcPointWithinTilePlanes(osg::Vec3d const &point,
                               Plane const &plane_min_lon,
                               Plane const &plane_max_lon,
                               Plane const &plane_min_lat,
                               Plane const &plane_max_lat,
                               Plane const &plane_hemisphere)
{
    static const double k_eps = 1E-5; // meters

    bool outside =
            (((point-plane_min_lon.p)*plane_min_lon.n) > k_eps) ||
            (((point-plane_max_lon.p)*plane_max_lon.n) > k_eps) ||
            (((point-plane_min_lat.p)*plane_min_lat.n) > k_eps) ||
            (((point-plane_max_lat.p)*plane_max_lat.n) > k_eps);

    return (!outside);
}


bool CalcFrustumTileIntersection(std::vector<LLA> const &list_frustum_lla,
                                 std::vector<osg::Vec3d> const &list_frustum_vx,
                                 uint8_t const frustum_pole,
                                 GeoBounds const &tile_bounds,
                                 GeoBounds &xsec_bounds,
                                 std::vector<LLA> &list_xsec_lla)
{
    Plane plane_min_lon;
    Plane plane_max_lon;
    Plane plane_min_lat;
    Plane plane_max_lat;
    Plane plane_hemisphere;

    CalcTilePlanes(tile_bounds,
                   plane_min_lon,
                   plane_max_lon,
                   plane_min_lat,
                   plane_max_lat,
                   plane_hemisphere);

//    std::cout << "plane_min_lat 0: " << std::endl
//              << "min_lat: " << tile_bounds.minLat << std::endl
//              << "N: " << plane_min_lat.n << std::endl
//              << "P: " << plane_min_lat.p << std::endl
//              << "D: " << plane_min_lat.d << std::endl
//              << std::endl << std::endl;

    std::vector<osg::Vec3d> list_xsec;
    list_xsec.reserve(list_frustum_vx.size());

//    std::cout << "plane_min_lat: 1" << std::endl
//              << "min_lat: " << tile_bounds.minLat << std::endl
//              << "N: " << plane_min_lat.n << std::endl
//              << "P: " << plane_min_lat.p << std::endl
//              << "D: " << plane_min_lat.d << std::endl
//              << std::endl << std::endl;



//    for(auto const &xsec : list_xsec) {
//        LLA lla = ConvECEFToLLA(xsec);

//        if(CalcPointWithinTilePlanes(xsec,
//                                     plane_min_lon,
//                                     plane_max_lon,
//                                     plane_min_lat,
//                                     plane_max_lat,
//                                     plane_hemisphere))
//        {
//        }
//        list_xsec_lla.push_back(lla);
//    }

    CalcPlanePolyIntersection(plane_min_lon,
                              list_frustum_vx,
                              list_xsec);

    CalcPlanePolyIntersection(plane_max_lon,
                              list_frustum_vx,
                              list_xsec);

    CalcPlanePolyIntersection(plane_max_lat,
                              list_frustum_vx,
                              list_xsec);

    CalcPlanePolyIntersection(plane_min_lat,
                              list_frustum_vx,
                              list_xsec);


    // Add xsec points that fall within tile_bounds
    for(auto const &xsec : list_xsec) {
        LLA lla = ConvECEFToLLA(xsec);

        if(CalcPointWithinTilePlanes(xsec,
                                     plane_min_lon,
                                     plane_max_lon,
                                     plane_min_lat,
                                     plane_max_lat,
                                     plane_hemisphere))
        {
            list_xsec_lla.push_back(lla);
        }
    }


    return true;
}

bool CalcFrustumXsecBounds(std::vector<LLA> const &list_frustum_lla,
                           std::vector<osg::Vec3d> const &list_frustum_vx,
                           GeoBounds const &tile_bounds,
                           std::vector<osg::Vec3d> &list_xsec,
                           GeoBounds &xsec_bounds)
{
    list_xsec.clear();
    list_xsec.reserve(list_frustum_vx.size());

    std::vector<LLA> list_xsec_lla;

    // Calculate xsec points with each plane
    // of tile_bounds

    // tile_bounds.min_lon
    {
        Plane plane_lon = CalcLonPlane(tile_bounds.minLon);
        CalcPlanePolyIntersection(plane_lon,
                                  list_frustum_vx,
                                  list_xsec);
    }

    // tile_bounds.max_lon
    {
        Plane plane_lon = CalcLonPlane(tile_bounds.maxLon);
        CalcPlanePolyIntersection(plane_lon,
                                  list_frustum_vx,
                                  list_xsec);
    }

    // tile_bounds.min_lon
    {
        Plane plane_lat = CalcLatPlane(tile_bounds.minLat);
        CalcPlanePolyIntersection(plane_lat,
                                  list_frustum_vx,
                                  list_xsec);
    }

    // tile_bounds.min_lon
    {
        Plane plane_lat = CalcLatPlane(tile_bounds.maxLat);
        CalcPlanePolyIntersection(plane_lat,
                                  list_frustum_vx,
                                  list_xsec);
    }

    //
    LLA lla_front;
    lla_front.lon = (tile_bounds.minLon+tile_bounds.maxLon)*0.5;
    lla_front.lat = 0;
    lla_front.alt = 0;

    Plane plane_front;
    plane_front.n = ConvLLAToECEF(lla_front);
    plane_front.p = osg::Vec3d(0,0,0);
    plane_front.d = plane_front.n*plane_front.p;

    // Calculate xsec_bounds
    // We don't have to use CalcMinGeoBounds because
    // the points are guaranteed to be within tile_bounds
    // and a max,min bounds should suffice
    uint8_t lla_in_bounds=0;
    xsec_bounds.minLon = 180;
    xsec_bounds.maxLon = -180;
    xsec_bounds.minLat = 90;
    xsec_bounds.maxLat = -90;

    // consider xsec points within tile_bounds
    for(auto const &xsec : list_xsec) {
        LLA lla = ConvECEFToLLA(xsec);
        if(CalcWithinGeoBounds(tile_bounds,lla)) {
            // todo clamp lla


            lla_in_bounds++;
            xsec_bounds.minLon = std::min(xsec_bounds.minLon,lla.lon);
            xsec_bounds.maxLon = std::max(xsec_bounds.maxLon,lla.lon);
            xsec_bounds.minLat = std::min(xsec_bounds.minLat,lla.lat);
            xsec_bounds.maxLat = std::max(xsec_bounds.maxLat,lla.lat);
        }
    }

    // consider list_frustum_lla within tile_bounds
    for(auto const &lla : list_frustum_lla) {
        if(CalcWithinGeoBounds(tile_bounds,lla)) {
            // todo clamp lla

            list_xsec.push_back(ConvLLAToECEF(lla));
            lla_in_bounds++;

            xsec_bounds.minLon = std::min(xsec_bounds.minLon,lla.lon);
            xsec_bounds.maxLon = std::max(xsec_bounds.maxLon,lla.lon);
            xsec_bounds.minLat = std::min(xsec_bounds.minLat,lla.lat);
            xsec_bounds.maxLat = std::max(xsec_bounds.maxLat,lla.lat);
        }
    }

//    for(auto const &x : list_xsec) {
//        std::cout << "lon: " << ConvECEFToLLA(x).lon
//                  << "lat: " << ConvECEFToLLA(x).lat
//                  << ", min_lon: " << xsec_bounds.minLon << std::endl;
//    }

    return (lla_in_bounds > 0);
}

int main()
{
    std::cout << std::fixed;
    std::cout << std::setprecision(18);

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
    GeoBounds tile_bounds(-40,0,40,60);
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

        // new frustum surface poly node
        std::vector<osg::Vec3d> list_frustum_ecef;
        CalcProjFrustumPoly(frustum,horizon_plane,list_frustum_ecef);
        auto new_frustumpoly = BuildSurfacePolyNode(
                    "frustumpoly",list_frustum_ecef,K_WHITE);

        std::vector<LLA> list_frustum_lla =
                ConvListECEFToLLA(list_frustum_ecef);


        // new frustum geobounds node
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

        osg::ref_ptr<osg::Group> new_frustumbounds = new osg::Group;
        new_frustumbounds->setName("frustumbounds");
        for(size_t i=0; i < list_frustum_bounds.size(); i++) {
            auto gp_frustumbounds =
                    BuildGeoBoundsNode(std::to_string(i),
                                       list_frustum_bounds[i],
                                       K_WHITE);
            new_frustumbounds->addChild(gp_frustumbounds);
        }

        // new xsec bounds node
        GeoBounds xsec_bounds;
        osg::ref_ptr<osg::Group> new_xsecbounds = new osg::Group;
        new_xsecbounds->setName("xsecbounds");
        for(size_t i=0; i < list_frustum_bounds.size(); i++) {
            if(CalcGeoBoundsIntersection(tile_bounds,
                                         list_frustum_bounds[i],
                                         xsec_bounds))
            {
                auto gp_xsecbounds = BuildGeoBoundsNode(std::to_string(i),
                                                        xsec_bounds,
                                                        K_RGREEN);
                new_xsecbounds->addChild(gp_xsecbounds);
                break;
            }
        }

        // new poly xsec bounds node
        osg::ref_ptr<osg::Group> new_polyxsecbounds = new osg::Group;
        new_polyxsecbounds->setName("polyxsecbounds");

        GeoBounds poly_xsec_bounds;

//        std::vector<osg::Vec3d> list_xsec;
//        if(CalcFrustumXsecBounds(list_frustum_lla,
//                                 list_frustum_ecef,
//                                 tile_bounds,
//                                 list_xsec,
//                                 poly_xsec_bounds))
//        {
//            auto gp_polyxsecbounds =
//                    BuildGeoBoundsNode("0",
//                                       poly_xsec_bounds,
//                                       K_YELLOW);
//            new_polyxsecbounds->addChild(gp_polyxsecbounds);
//        }

//        for(auto const &xsec : list_xsec) {
//            auto gp_xsec = BuildFacingCircleNode("0",xsec,100000,8,K_RED);
//            new_polyxsecbounds->addChild(gp_xsec);
//        }

        std::vector<LLA> list_xsec_lla;
        if(CalcFrustumTileIntersection(list_frustum_lla,
                                       list_frustum_ecef,
                                       frustum_pole,
                                       tile_bounds,
                                       poly_xsec_bounds,
                                       list_xsec_lla))
        {
//            std::cout << "#: 3.1: " << std::endl;
//            std::cout << "  tile.lon [" << poly_xsec_bounds.minLon
//                      << "," << poly_xsec_bounds.maxLon << "]"
//                      << ", tile.lat [" << poly_xsec_bounds.minLat
//                      << "," << poly_xsec_bounds.maxLat << "]" << std::endl;



//            auto gp_polyxsecbounds =
//                    BuildGeoBoundsNode("0",
//                                       poly_xsec_bounds,
//                                       K_YELLOW);

//            new_polyxsecbounds->addChild(gp_polyxsecbounds);
        }

        for(auto const &xsec_lla : list_xsec_lla) {
            auto gp_xsec = BuildFacingCircleNode(
                        "0",ConvLLAToECEF(xsec_lla),100000,8,K_RED);
            new_polyxsecbounds->addChild(gp_xsec);
        }


//        // ============== test
//        auto list_ranges = CalcLonRange(list_frustum_lla);
//        std::cout << "-" << std::endl;
//        for(auto const &range : list_ranges) {
//            std::cout << "range //: " << range.first << "," << range.second << std::endl;
//        }

//        for(auto const &bb : list_frustum_bounds) {
//            std::cout << "range bb: " << bb.minLon << "," << bb.maxLon << std::endl;
//        }

        // Update gp_root0
        for(size_t i=0; i < gp_root0->getNumChildren(); i++)
        {
            std::string const name =
                    gp_root0->getChild(i)->getName();

            if(name == "frustumpoly") {
                gp_root0->removeChild(i);
                i--;
            }
            else if(name == "frustumbounds") {
                gp_root0->removeChild(i);
                i--;
            }
            else if(name == "xsecbounds") {
                gp_root0->removeChild(i);
            }
            else if(name == "polyxsecbounds") {
                gp_root0->removeChild(i);
            }
        }
        gp_root0->addChild(new_frustumpoly);
        gp_root0->addChild(new_frustumbounds);
        gp_root0->addChild(new_xsecbounds);
        gp_root0->addChild(new_polyxsecbounds);

        // Update gp_root1
        for(size_t i=0; i < gp_root1->getNumChildren(); i++)
        {
             std::string const name =
                     gp_root1->getChild(i)->getName();

             if(name == "frustum") {
                 gp_root1->removeChild(i);
                 i--;
             }
             else if(name == "frustumpoly") {
                 gp_root1->removeChild(i);
                 i--;
             }
             else if(name == "frustumbounds") {
                 gp_root1->removeChild(i);
                 i--;
             }
             else if(name == "xsecbounds") {
                 gp_root1->removeChild(i);
                 i--;
             }
             else if(name == "polyxsecbounds") {
                 gp_root1->removeChild(i);
             }
        }
        gp_root1->addChild(new_frustum);
        gp_root1->addChild(new_frustumpoly);
        gp_root1->addChild(new_frustumbounds);
        gp_root1->addChild(new_xsecbounds);
        gp_root1->addChild(new_polyxsecbounds);

        viewer.frame();
    }

    return 0;
}
