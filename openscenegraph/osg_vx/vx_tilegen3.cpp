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

bool GeoBoundsIsValid(GeoBounds const &b)
{
    if(b.minLon == 0 && b.maxLon == 0 && b.minLat == 0 && b.maxLat == 0) {
        return false;
    }
    return true;
}

bool CheckTileOverlapsGeoBounds(GeoBounds const &geobb,
                                std::unique_ptr<VxTile> &tile)
{
    if(tile->minLon > geobb.maxLon ||
       tile->maxLon < geobb.minLon ||
       tile->minLat > geobb.maxLat ||
       tile->maxLat < geobb.minLat)
    {
        return false;
    }

    return true;
}

bool CheckTileOverlapsGeoBounds(std::vector<GeoBounds> const &list_geobb,
                                std::unique_ptr<VxTile> &tile)
{
    for(auto const &geobb: list_geobb)
    {
        if(!(tile->minLon > geobb.maxLon ||
             tile->maxLon < geobb.minLon ||
             tile->minLat > geobb.maxLat ||
             tile->maxLat < geobb.minLat))
        {
            return true;
        }
    }
    return false;
}

void GenQuadTreeForGeoBounds(std::vector<std::vector<GeoBounds>> const &list_lod_geobb,
                             std::unique_ptr<VxTile> &tile)
{
    // If the next lod's geobounds exist and this tile
    // intersects with them, subdivide the tile
    if(CheckTileOverlapsGeoBounds(list_lod_geobb[tile->level+1],tile))
    {
        tile->tile_LT = BuildChildTile(tile.get(),0,false);
        tile->tile_LB = BuildChildTile(tile.get(),1,false);
        tile->tile_RB = BuildChildTile(tile.get(),2,false);
        tile->tile_RT = BuildChildTile(tile.get(),3,false);

        GenQuadTreeForGeoBounds(list_lod_geobb,tile->tile_LT);
        GenQuadTreeForGeoBounds(list_lod_geobb,tile->tile_LB);
        GenQuadTreeForGeoBounds(list_lod_geobb,tile->tile_RB);
        GenQuadTreeForGeoBounds(list_lod_geobb,tile->tile_RT);

        // Draw this tile if it has any empty children
        if(tile->tile_LT == nullptr ||
           tile->tile_LB == nullptr ||
           tile->tile_RB == nullptr ||
           tile->tile_RT == nullptr)
        {
            tile->_gp = BuildTileGeometry(tile.get());
        }
    }
    else {
        // Else draw this tile if it intersects with the
        // current lod geobounds
        if(CheckTileOverlapsGeoBounds(list_lod_geobb[tile->level],tile)) {
            tile->_gp = BuildTileGeometry(tile.get());
        }
        else {
            tile = nullptr;
        }
    }
}

void GenQuadTreeForGeoBounds(std::vector<GeoBounds> const &list_level_geobb,
                             std::unique_ptr<VxTile> &tile)
{


    if(GeoBoundsIsValid(list_level_geobb[tile->level+1])) {
        // Else subdivide if this tile overlaps with the
        // next level's GeoBounds
        if(CheckTileOverlapsGeoBounds(list_level_geobb[tile->level+1],tile)) {
            tile->tile_LT = BuildChildTile(tile.get(),0,false);
            tile->tile_LB = BuildChildTile(tile.get(),1,false);
            tile->tile_RB = BuildChildTile(tile.get(),2,false);
            tile->tile_RT = BuildChildTile(tile.get(),3,false);

            GenQuadTreeForGeoBounds(list_level_geobb,tile->tile_LT);
            GenQuadTreeForGeoBounds(list_level_geobb,tile->tile_LB);
            GenQuadTreeForGeoBounds(list_level_geobb,tile->tile_RB);
            GenQuadTreeForGeoBounds(list_level_geobb,tile->tile_RT);

            // Draw this tile if it has any empty children
            if(tile->tile_LT == nullptr ||
               tile->tile_LB == nullptr ||
               tile->tile_RB == nullptr ||
               tile->tile_RT == nullptr)
            {
                tile->_gp = BuildTileGeometry(tile.get());
            }
        }
        else {
            // If the next level's GeoBounds are invalid, we
            // don't subdivide further, only check if this
            // tile is within this level's GeoBounds
            if(CheckTileOverlapsGeoBounds(list_level_geobb[tile->level],tile)) {
                tile->_gp = BuildTileGeometry(tile.get());
            }
            else {
                tile = nullptr;
            }
        }
    }
    else {
        // If the next level's GeoBounds are invalid, we
        // don't subdivide further, only check if this
        // tile is within this level's GeoBounds
        if(CheckTileOverlapsGeoBounds(list_level_geobb[tile->level],tile)) {
            tile->_gp = BuildTileGeometry(tile.get());
        }
        else {
            tile = nullptr;
        }
    }
}

void AddQuadTreeGeometryToGroup(std::unique_ptr<VxTile> const &tile,
                                osg::ref_ptr<osg::Group> &gp)
{
    gp->addChild(tile->_gp);

    if(tile->tile_LT) {
        AddQuadTreeGeometryToGroup(tile->tile_LT,gp);
    }

    if(tile->tile_LB) {
        AddQuadTreeGeometryToGroup(tile->tile_LB,gp);
    }

    if(tile->tile_RB) {
        AddQuadTreeGeometryToGroup(tile->tile_RB,gp);
    }

    if(tile->tile_RT) {
        AddQuadTreeGeometryToGroup(tile->tile_RT,gp);
    }
}

uint32_t GetTileCountForGeoBounds(GeoBounds const &tile_bounds,
                                  uint8_t const level,
                                  uint8_t const root_lon_divs,
                                  uint8_t const root_lat_divs)
{
    // todo quicker int-only pow()
    double const lon_div_degs = 360.0/(pow(2,level)*root_lon_divs);
    double const lat_div_degs = 180.0/(pow(2,level)*root_lat_divs);

    uint32_t const start_x = floor((tile_bounds.minLon-(-180.0))/lon_div_degs);
    uint32_t const end_x = ceil((tile_bounds.maxLon-(-180.0))/lon_div_degs);

    uint32_t const start_y = floor((tile_bounds.minLat-(-90.0))/lat_div_degs);
    uint32_t const end_y = ceil((tile_bounds.maxLat-(-90.0))/lat_div_degs);

    uint32_t const x_tiles = (end_x-start_x);
    uint32_t const y_tiles = (end_y-start_y);
    return x_tiles*y_tiles;
}

bool CheckGeoBoundsTileCountLessThan(std::vector<GeoBounds> const &list_geobb,
                                     uint8_t const level,
                                     uint8_t const root_lon_divs,
                                     uint8_t const root_lat_divs,
                                     uint8_t const max_tiles)
{
    double const lon_div_degs = 360.0/(pow(2,level)*root_lon_divs);
    double const lat_div_degs = 180.0/(pow(2,level)*root_lat_divs);

    uint32_t num_tiles = 0;

    for(auto const &geobb : list_geobb)
    {
        uint32_t const start_x = floor((geobb.minLon-(-180.0))/lon_div_degs);
        uint32_t const end_x = ceil((geobb.maxLon-(-180.0))/lon_div_degs);

        uint32_t const start_y = floor((geobb.minLat-(-90.0))/lat_div_degs);
        uint32_t const end_y = ceil((geobb.maxLat-(-90.0))/lat_div_degs);

        uint32_t const x_tiles = (end_x-start_x);
        uint32_t const y_tiles = (end_y-start_y);
        num_tiles += (x_tiles*y_tiles);
    }

    if(num_tiles > max_tiles) {
//        for(auto const &geobb : list_geobb) {
//            uint32_t const start_x = floor((geobb.minLon-(-180.0))/lon_div_degs);
//            uint32_t const end_x = ceil((geobb.maxLon-(-180.0))/lon_div_degs);

//            uint32_t const start_y = floor((geobb.minLat-(-90.0))/lat_div_degs);
//            uint32_t const end_y = ceil((geobb.maxLat-(-90.0))/lat_div_degs);

//            uint32_t const x_tiles = (end_x-start_x);
//            uint32_t const y_tiles = (end_y-start_y);
//        std::cout   << "##: lvl:" << int(level)
//                    << ", lon: (" << geobb.minLon << "," << geobb.maxLon << ")"
//                    << ", lat: (" << geobb.minLat << "," << geobb.maxLat << ")"
//                    << ", xt: " << x_tiles
//                    << ", yt: " << y_tiles
//                    << ", num_tiles: " << num_tiles << std::endl;
//        }
        std::cout << "###: !: " << num_tiles << std::endl;
        return false;
    }
    return true;
}

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

        PointLLA lla_eye = ConvECEFToLLA(eye);

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


        if(eye.length() <= RAD_AV)
        {
            viewer.frame();
            continue;
        }

        // Get projection center and tangent plane
        osg::Vec3d proj_center = CalcGnomonicProjOrigin(horizon_plane);
        Plane tangent_plane = horizon_plane;
        tangent_plane.p = horizon_plane.n * RAD_AV;
        tangent_plane.d = tangent_plane.n * tangent_plane.p;

        // Get the projection of the camera frustum on
        // the planet surface
        std::vector<osg::Vec3d> poly_frustum_surf;
        CalcProjFrustumPoly(frustum,horizon_plane,poly_frustum_surf);

        // Find the minimum lod that encompasses the
        // frustum proj poly
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

        // An empty entry in list_polys_xsec signals a stop
        // condition for quadtree generation, so we have one
        // extra entry at the end
        std::vector<std::vector<osg::Vec3d>> list_polys_xsec(max_lod+2);

        for(size_t i=0; i <= min_lod; i++) {
            list_polys_xsec[i] = poly_frustum_surf;
        }

        for(size_t i=min_lod+1; i <= max_lod; i++)
        {
            std::vector<osg::Vec3d> poly_lod_sphere;
            CalcProjSpherePoly(horizon_plane,eye,K_LIST_LOD_DIST[i],poly_lod_sphere);
            if(poly_lod_sphere.empty()) {
                break;
            }

            std::vector<osg::Vec3d> poly_xsec_tangent;
            CalcGnomonicProjIntersectionPoly(proj_center,
                                             tangent_plane,
                                             poly_frustum_surf,
                                             poly_lod_sphere,
                                             list_polys_xsec[i],
                                             poly_xsec_tangent);
        }

        // Create geo bounds for all the xsec polys
        std::vector<std::vector<GeoBounds>> list_lod_geobb(max_lod+2);
        for(size_t i=0; i < list_polys_xsec.size(); i++) {
            std::vector<osg::Vec3d> const &xsec_poly = list_polys_xsec[i];
            if(!xsec_poly.empty()) {
                std::vector<PointLLA> list_lla;
                list_lla.reserve(xsec_poly.size());
                for(auto const &vx : xsec_poly) {
                    list_lla.push_back(ConvECEFToLLA(vx));
                }
                std::vector<GeoBounds> list_geobb;
                CalcMinGeoBoundsFromLLAPoly(lla_eye,list_lla,list_geobb);

                if(!list_geobb.empty()) {
                    if(CheckGeoBoundsTileCountLessThan(list_geobb,i,2,1,16)) {
                        list_lod_geobb[i] = list_geobb;
                    }
                    else {
                        break;
                    }
                }
            }
        }

        // Build quadtree from geo bounds
        std::unique_ptr<VxTile> root_tile0(BuildRootTile(0,-180.0,-90.0,0.0,90.0));
        std::unique_ptr<VxTile> root_tile1(BuildRootTile(0,0.0,-90.0,180.0,90.0));
        GenQuadTreeForGeoBounds(list_lod_geobb,root_tile0);
        GenQuadTreeForGeoBounds(list_lod_geobb,root_tile1);

        // ==================================================== //

        // new lod rings node
        auto new_lodrings = BuildLodRingsNode(eye);

        // new frustumsurfproj
        auto new_frustumsurfpoly =
                BuildSurfacePoly(poly_frustum_surf,
                                 osg::Vec4(0.75,0.5,1.0,1.0),
                                 eye.length()/1200.0);
        new_frustumsurfpoly->setName("frustumsurfpoly");

        // new lodsurfpoly
        osg::ref_ptr<osg::Group> new_lodsurfpoly = new osg::Group;
        for(size_t i=0; i < list_polys_xsec.size(); i++) {
            new_lodsurfpoly->addChild(BuildSurfacePoly(list_polys_xsec[i],
                                                       K_COLOR_TABLE[i],
                                                       eye.length()/1200.0));
        }
        new_lodsurfpoly->setName("lodsurfpoly");

        // new 0_0_pt
        auto new_pt_0_0 = BuildFacingCircle(ConvLLAToECEF(PointLLA(0,0)),
                                            eye.length()/150.0,
                                            8,
                                            osg::Vec4(1,0,0,1));
        new_pt_0_0->setName("pt_0_0");

        // new 90_0_pt
        auto new_pt_90_0 = BuildFacingCircle(ConvLLAToECEF(PointLLA(90,0)),
                                             eye.length()/150.0,
                                             8,
                                             osg::Vec4(0,1,0,1));
        new_pt_90_0->setName("pt_90_0");

        // new vx tiles node
        osg::ref_ptr<osg::Group> new_vxtiles = new osg::Group;
        if(root_tile0) {
            AddQuadTreeGeometryToGroup(root_tile0,new_vxtiles);
        }
        if(root_tile1) {
            AddQuadTreeGeometryToGroup(root_tile1,new_vxtiles);
        }
        new_vxtiles->setName("vxtiles");


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
                else if(name == "vxtiles") {
                    gp_root0->removeChild(i);
                    i--;
                }
            }
            gp_root0->addChild(new_frustumsurfpoly);
            gp_root0->addChild(new_lodsurfpoly);
            gp_root0->addChild(new_pt_0_0);
            gp_root0->addChild(new_pt_90_0);
            gp_root0->addChild(new_vxtiles);
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
                else if(name == "vxtiles") {
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
            gp_root1->addChild(new_vxtiles);
        }
        viewer.frame();
    }
    return 0;
}

