//
#include <map>
#include <mutex>

//
#include <osgnodes.hpp>
#include <projutil.hpp>

//
#include <osg/Switch>

std::mutex mutex_map_tile_geometry_cache;
std::map<uint64_t,std::pair<bool,osg::ref_ptr<osg::Switch>>> map_tile_geometry_cache;


uint64_t K_LIST_TWO_EXP[32] = {
    1,
    2,
    4,
    8,
    16,
    32,
    64,
    128,
    256,
    512,
    1024,
    2048,
    4096,
    8192,
    16384,
    32768,
    65536,
    131072,
    262144,
    524288,
    1048576,
    2097152,
    4194304,
    8388608,
    16777216,
    33554432,
    67108864,
    134217728,
    268435456,
    536870912,
    1073741824,
    2147483648
};

class XYTile
{
public:
//    XYTile(uint64_t id,GeoBounds const &root) :
//        id(id),
//        level((id & 0xFF000000000000) >> 48),
//        x((id & 0x00FFFFFF000000) >> 24),
//        y(id & 0x00000000FFFFFF),
//        min_lon(GetLon(root,level,x)),
//        max_lon(GetLon(root,level,x+1)),
//        min_lat(GetLat(root,level,y)),
//        max_lat(GetLat(root,level,y+1)),
//        tile_LT(nullptr),
//        tile_LB(nullptr),
//        tile_RB(nullptr),
//        tile_RT(nullptr),
//        gm(nullptr)
//    {}

//    XYTile(uint8_t level,
//           uint32_t x,
//           uint32_t y,
//           GeoBounds const &root) :
//        id(GetIdFromLevelXY(level,x,y)),
//        level(level),
//        x(x),
//        y(y),
//        min_lon(GetLon(root,level,x)),
//        max_lon(GetLon(root,level,x+1)),
//        min_lat(GetLat(root,level,y)),
//        max_lat(GetLat(root,level,y+1)),
//        tile_LT(nullptr),
//        tile_LB(nullptr),
//        tile_RB(nullptr),
//        tile_RT(nullptr),
//        gm(nullptr)
//    {}

    // root constructor
    XYTile(uint8_t level,
           uint32_t x,
           uint32_t y,
           double min_lon,
           double max_lon,
           double min_lat,
           double max_lat) :
        id(GetIdFromLevelXY(level,x,y)),
        level(level),
        x(x),
        y(y),
        parent(nullptr),
        min_lon(min_lon),
        max_lon(max_lon),
        min_lat(min_lat),
        max_lat(max_lat),
        tile_LT(nullptr),
        tile_LB(nullptr),
        tile_RB(nullptr),
        tile_RT(nullptr),
        gm(nullptr)
    {
        // empty
    }

    // child constructor
    XYTile(uint8_t level,
           uint32_t x,
           uint32_t y,
           XYTile * parent) :
        id(GetIdFromLevelXY(level,x,y)),
        level(level),
        x(x),
        y(y),
        parent(parent),
        min_lon(GetLon(parent,x)),
        max_lon(GetLon(parent,x+1)),
        min_lat(GetLat(parent,y)),
        max_lat(GetLat(parent,y+1)),
        tile_LT(nullptr),
        tile_LB(nullptr),
        tile_RB(nullptr),
        tile_RT(nullptr),
        gm(nullptr)
    {
        // empty
    }

    ~XYTile()
    {
        // Mark geometry unused
        auto it = map_tile_geometry_cache.find(id);
        if(it != map_tile_geometry_cache.end()) {
            it->second.first = false;
        }
    }

    static uint64_t GetIdFromLevelXY(uint8_t level,
                                     uint32_t x,
                                     uint32_t y)
    {
        uint64_t level64 = level;
        uint64_t x64 = x;
        uint64_t y64 = y;
        uint64_t tile_id = 0;
        tile_id |= level64 << 48;
        tile_id |= x64 << 24;
        tile_id |= y64;

        return tile_id;
    }

    static double GetLon(XYTile const * parent,
                         uint32_t x)
    {
        return parent->min_lon + (parent->max_lon-parent->min_lon)*(x-parent->x*2)*0.5;
    }

    static double GetLat(XYTile const * parent,
                         uint32_t y)
    {
        return parent->min_lat + (parent->max_lat-parent->min_lat)*(y-parent->y*2)*0.5;
    }

//    static double GetLon(GeoBounds const &root,
//                         uint8_t level,
//                         uint32_t x)
//    {
//        double lon_div_degs = (root.maxLon-root.minLon)/K_LIST_TWO_EXP[level];
//        return (root.minLon + x*lon_div_degs);
//    }

//    static double GetLat(GeoBounds const &root,
//                         uint8_t level,
//                         uint32_t y)
//    {
//        double lat_div_degs = (root.maxLat-root.minLat)/K_LIST_TWO_EXP[level];
//        return (root.minLat + y*lat_div_degs);
//    }

    // id:
    // z  x      y
    // FF FFFFFF FFFFFF
    // z: tile level (8 bits)
    // x: tile x (24 bits)
    // y: tile y (24 bits)
    uint64_t const id;

    uint8_t  const level;
    uint32_t const x;
    uint32_t const y;

    XYTile * parent;

    double const min_lon;
    double const max_lon;
    double const min_lat;
    double const max_lat;


    std::unique_ptr<XYTile> tile_LT;
    std::unique_ptr<XYTile> tile_LB;
    std::unique_ptr<XYTile> tile_RB;
    std::unique_ptr<XYTile> tile_RT;

    osg::ref_ptr<osg::Switch> gm;
    uint8_t clip;
    uint8_t tx_level;
};

void CalcGnomonicProjPolys(Plane const &horizon_plane,
                           Frustum const &frustum,
                           osg::Vec3d const &eye,
                           std::vector<osg::Vec3d> &poly_frustum_surf,
                           std::vector<std::vector<osg::Vec3d>> &list_polys_xsec,
                           std::vector<std::vector<GeoBounds>> &list_lod_geobb);

void GenQuadTreeForGeoBounds(std::vector<std::vector<GeoBounds>> const &list_lod_geobb,
                             std::unique_ptr<XYTile> &tile);

void GenGmListFromQuadTree(std::unique_ptr<XYTile> &tile,
                           osg::Group * gp);

int main()
{
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

    // Create root tiles
    std::unique_ptr<XYTile> root_tile(new XYTile(0,0,0,-180.0,180.0,-90.0,90.0));

    // ==================================================== //
    // ==================================================== //

    while(!viewer.done())
    {
        // current camera params
        osg::Camera * camera = viewer.getView(0)->getCamera();

        osg::Vec3d eye,vpt,up;
        camera->getViewMatrixAsLookAt(eye,vpt,up);

        double far_dist,near_dist;
        if(!CalcCameraNearFarDist(eye,vpt-eye,20000.0,near_dist,far_dist)) {
            far_dist=0.0;
            near_dist=0.0;
        }

        // horizon plane
        Plane horizon_plane;
        CalcHorizonPlane(eye,horizon_plane);

        // Build a new frustum node
        Frustum frustum;
        auto new_frustum = BuildFrustumNode(camera,frustum,near_dist,far_dist);

        // The camera eye must be above the planet's surface
        if(eye.length() <= RAD_AV) {
            viewer.frame();
            continue;
        }

        // TODO
        // should be able to map any given distance
        // to specific lods since distances will
        // actually correspond to meters/px or whatever

        // Calculate view extents
        std::vector<osg::Vec3d> poly_frustum_surf;
        std::vector<std::vector<osg::Vec3d>> list_polys_xsec;
        std::vector<std::vector<GeoBounds>> list_lod_geobb;
        CalcGnomonicProjPolys(horizon_plane,
                              frustum,
                              eye,
                              poly_frustum_surf,
                              list_polys_xsec,
                              list_lod_geobb);


        // Create the quad tree
        mutex_map_tile_geometry_cache.lock();
        GenQuadTreeForGeoBounds(list_lod_geobb,root_tile);
        std::cout << "###: gm cache sz: " << map_tile_geometry_cache.size() << std::endl;
        mutex_map_tile_geometry_cache.unlock();


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

        // new vxtiles
        osg::ref_ptr<osg::Group> new_vxtiles = new osg::Group;
        new_vxtiles->setName("vxtiles");
        GenGmListFromQuadTree(root_tile,new_vxtiles);

        // ==================================================== //


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

void CalcGnomonicProjPolys(Plane const &horizon_plane,
                           Frustum const &frustum,
                           osg::Vec3d const &eye,
                           std::vector<osg::Vec3d> &poly_frustum_surf,
                           std::vector<std::vector<osg::Vec3d>> &list_polys_xsec,
                           std::vector<std::vector<GeoBounds>> &list_lod_geobb)
{
    // Get projection center and tangent plane
    osg::Vec3d proj_center = CalcGnomonicProjOrigin(horizon_plane);
    Plane tangent_plane = horizon_plane;
    tangent_plane.p = horizon_plane.n * RAD_AV;
    tangent_plane.d = tangent_plane.n * tangent_plane.p;

    // Get the projection of the camera frustum on
    // the planet surface
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
    list_polys_xsec.clear();
    list_polys_xsec.resize(max_lod+1);

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

    PointLLA lla_eye = ConvECEFToLLA(eye);

    // Create geo bounds for all the xsec polys
    // move to own method
    list_lod_geobb.clear();
    list_lod_geobb.resize(list_polys_xsec.size());
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
                list_lod_geobb[i] = list_geobb;
            }
        }
    }
}

bool CheckTileOverlapsGeoBounds(std::vector<GeoBounds> const &list_geobb,
                                std::unique_ptr<XYTile> &tile)
{
    for(auto const &geobb: list_geobb)
    {
        if(!(tile->min_lon > geobb.maxLon ||
             tile->max_lon < geobb.minLon ||
             tile->min_lat > geobb.maxLat ||
             tile->max_lat < geobb.minLat))
        {
            return true;
        }
    }
    return false;
}



void UpdateXYTileTextureCoords(XYTile * tile, XYTile const * tx_tile)
{
    double mid_lon = (tile->max_lon+tile->min_lon)*0.5;
    double mid_lat = (tile->max_lat+tile->min_lat)*0.5;

    double tx_width_inv  = 1.0/(tx_tile->max_lon-tx_tile->min_lon);
    double tx_height_inv = 1.0/(tx_tile->max_lat-tx_tile->min_lat);

    osg::Geode * gd = static_cast<osg::Geode*>(tile->gm->getChild(0));

    // LT
    {
        double const min_lon = tile->min_lon;
        double const max_lon = mid_lon;
        double const min_lat = mid_lat;
        double const max_lat = tile->max_lat;

        double s_start = (min_lon-tx_tile->min_lon)*tx_width_inv;
        double s_end = (max_lon-tx_tile->min_lon)*tx_width_inv;

        double t_start = 1.0-((max_lat-tx_tile->min_lat)*tx_height_inv);
        double t_end = 1.0-((min_lat-tx_tile->min_lat)*tx_height_inv);

        double s_delta = s_end-s_start;
        double t_delta = t_end-t_start;

        osg::Geometry * gm =
                static_cast<osg::Geometry*>(gd->getDrawable(0));

        osg::Vec2dArray * list_tx =
                static_cast<osg::Vec2dArray*>(gm->getTexCoordArray(0));

        osg::Vec2dArray const * list_tx_ref =
                static_cast<osg::Vec2dArray*>(gm->getTexCoordArray(1));

        for(size_t i=0; i < list_tx_ref->size(); i++) {
            osg::Vec2d const &tx_ref = list_tx_ref->at(i);
            osg::Vec2d &tx = list_tx->at(i);
            tx.x() = (tx_ref.x()*s_delta) + s_start;
            tx.y() = (tx_ref.y()*t_delta) + t_start;
        }
    }

    // LB
    {
        double const min_lon = tile->min_lon;
        double const max_lon = mid_lon;
        double const min_lat = tile->min_lat;
        double const max_lat = mid_lat;

        double s_start = (min_lon-tx_tile->min_lon)*tx_width_inv;
        double s_end = (max_lon-tx_tile->min_lon)*tx_width_inv;

        double t_start = 1.0-((max_lat-tx_tile->min_lat)*tx_height_inv);
        double t_end = 1.0-((min_lat-tx_tile->min_lat)*tx_height_inv);

        double s_delta = s_end-s_start;
        double t_delta = t_end-t_start;

        osg::Geometry * gm =
                static_cast<osg::Geometry*>(gd->getDrawable(0));

        osg::Vec2dArray * list_tx =
                static_cast<osg::Vec2dArray*>(gm->getTexCoordArray(0));

        osg::Vec2dArray const * list_tx_ref =
                static_cast<osg::Vec2dArray*>(gm->getTexCoordArray(1));

        for(size_t i=0; i < list_tx_ref->size(); i++) {
            osg::Vec2d const &tx_ref = list_tx_ref->at(i);
            osg::Vec2d &tx = list_tx->at(i);
            tx.x() = (tx_ref.x()*s_delta) + s_start;
            tx.y() = (tx_ref.y()*t_delta) + t_start;
        }
    }

    // RB
    {
        double const min_lon = mid_lon;
        double const max_lon = tile->max_lon;
        double const min_lat = tile->min_lat;
        double const max_lat = mid_lat;

        double s_start = (min_lon-tx_tile->min_lon)*tx_width_inv;
        double s_end = (max_lon-tx_tile->min_lon)*tx_width_inv;

        double t_start = 1.0-((max_lat-tx_tile->min_lat)*tx_height_inv);
        double t_end = 1.0-((min_lat-tx_tile->min_lat)*tx_height_inv);

        double s_delta = s_end-s_start;
        double t_delta = t_end-t_start;

        osg::Geometry * gm =
                static_cast<osg::Geometry*>(gd->getDrawable(0));

        osg::Vec2dArray * list_tx =
                static_cast<osg::Vec2dArray*>(gm->getTexCoordArray(0));

        osg::Vec2dArray const * list_tx_ref =
                static_cast<osg::Vec2dArray*>(gm->getTexCoordArray(1));

        for(size_t i=0; i < list_tx_ref->size(); i++) {
            osg::Vec2d const &tx_ref = list_tx_ref->at(i);
            osg::Vec2d &tx = list_tx->at(i);
            tx.x() = (tx_ref.x()*s_delta) + s_start;
            tx.y() = (tx_ref.y()*t_delta) + t_start;
        }
    }

    // RT
    {
        double const min_lon = mid_lon;
        double const max_lon = tile->max_lon;
        double const min_lat = mid_lat;
        double const max_lat = tile->max_lat;

        double s_start = (min_lon-tx_tile->min_lon)*tx_width_inv;
        double s_end = (max_lon-tx_tile->min_lon)*tx_width_inv;

        double t_start = 1.0-((max_lat-tx_tile->min_lat)*tx_height_inv);
        double t_end = 1.0-((min_lat-tx_tile->min_lat)*tx_height_inv);

        double s_delta = s_end-s_start;
        double t_delta = t_end-t_start;

        osg::Geometry * gm =
                static_cast<osg::Geometry*>(gd->getDrawable(0));

        osg::Vec2dArray * list_tx =
                static_cast<osg::Vec2dArray*>(gm->getTexCoordArray(0));

        osg::Vec2dArray const * list_tx_ref =
                static_cast<osg::Vec2dArray*>(gm->getTexCoordArray(1));

        for(size_t i=0; i < list_tx_ref->size(); i++) {
            osg::Vec2d const &tx_ref = list_tx_ref->at(i);
            osg::Vec2d &tx = list_tx->at(i);
            tx.x() = (tx_ref.x()*s_delta) + s_start;
            tx.y() = (tx_ref.y()*t_delta) + t_start;
        }
    }

    tile->tx_level = tx_tile->level;
}

void BuildXYTileGeometry(XYTile * tile)
{
    tile->gm = nullptr;
    tile->gm = new osg::Switch;
    tile->gm->getOrCreateStateSet()->setRenderBinDetails(-101 + tile->level,"RenderBin");
    tile->gm->getOrCreateStateSet()->setMode(GL_CULL_FACE,
                                             osg::StateAttribute::ON |
                                             osg::StateAttribute::OVERRIDE);

    // If the texture level

    double mid_lon = (tile->max_lon+tile->min_lon)*0.5;
    double mid_lat = (tile->max_lat+tile->min_lat)*0.5;

//    std::cout << "###: "
//                 "min_lon: " << tile->min_lon << ", "
//              << "max_lon: " << tile->max_lon << ", "
//              << "min_lat: " << tile->min_lat << ", "
//              << "max_lat: " << tile->max_lat << std::endl;

    // Determine the number of surface divisions
    // for the tile geometry:
    // 0 - 32
    // 1 - 32
    // 2 - 32
    // 3 - 32
    // 4 - 16
    // 5 - 8
    // 6 - 4
    // 7 - 2
    // 8 - 1
    uint32_t surf_divs = 256/K_LIST_TWO_EXP[tile->level];
    surf_divs = std::min(surf_divs,static_cast<uint32_t>(32));

    // Draw the tile in four segments so we
    // can apply a specific clip
    uint32_t half_lon_segments = surf_divs/2;
    uint32_t half_lat_segments = half_lon_segments/2;

    // Build geometry for all four child tiles

    // LT
    {
        double const min_lon = tile->min_lon;
        double const max_lon = mid_lon;
        double const min_lat = mid_lat;
        double const max_lat = tile->max_lat;

        osg::ref_ptr<osg::Vec3dArray> list_vx = new osg::Vec3dArray;
        osg::ref_ptr<osg::Vec2dArray> list_tx = new osg::Vec2dArray;
        osg::ref_ptr<osg::Vec4Array>  list_cx = new osg::Vec4Array;
        osg::ref_ptr<osg::DrawElementsUShort> list_ix =
                new osg::DrawElementsUShort(GL_TRIANGLES);

        BuildEarthSurfaceGeometry(min_lon,
                                  min_lat,
                                  max_lon,
                                  max_lat,
                                  half_lon_segments,
                                  half_lat_segments,
                                  list_vx.get(),
                                  list_tx.get(),
                                  list_ix.get());

        osg::Vec4 cx = K_COLOR_TABLE[tile->level];
        cx.r() *= 0.5;
        cx.g() *= 0.5;
        cx.b() *= 0.5;
        list_cx->push_back(cx);

        osg::ref_ptr<osg::Geometry> gm = new osg::Geometry;
        gm->setVertexArray(list_vx);
        gm->setTexCoordArray(0,list_tx,osg::Array::BIND_PER_VERTEX);
        gm->setTexCoordArray(1,list_tx,osg::Array::BIND_PER_VERTEX);
        gm->setColorArray(list_cx,osg::Array::BIND_OVERALL);
        gm->addPrimitiveSet(list_ix);

        osg::ref_ptr<osg::Geode> gd = new osg::Geode;
        gd->addDrawable(gm);
        tile->gm->addChild(gd);
    }

    // LB
    {
        double const min_lon = tile->min_lon;
        double const max_lon = mid_lon;
        double const min_lat = tile->min_lat;
        double const max_lat = mid_lat;

        osg::ref_ptr<osg::Vec3dArray> list_vx = new osg::Vec3dArray;
        osg::ref_ptr<osg::Vec2dArray> list_tx = new osg::Vec2dArray;
        osg::ref_ptr<osg::Vec4Array>  list_cx = new osg::Vec4Array;
        osg::ref_ptr<osg::DrawElementsUShort> list_ix =
                new osg::DrawElementsUShort(GL_TRIANGLES);

        BuildEarthSurfaceGeometry(min_lon,
                                  min_lat,
                                  max_lon,
                                  max_lat,
                                  half_lon_segments,
                                  half_lat_segments,
                                  list_vx.get(),
                                  list_tx.get(),
                                  list_ix.get());

        osg::Vec4 cx = K_COLOR_TABLE[tile->level];
        cx.r() *= 0.625;
        cx.g() *= 0.625;
        cx.b() *= 0.625;
        list_cx->push_back(cx);

        osg::ref_ptr<osg::Geometry> gm = new osg::Geometry;
        gm->setVertexArray(list_vx);
        gm->setTexCoordArray(0,list_tx,osg::Array::BIND_PER_VERTEX);
        gm->setTexCoordArray(1,list_tx,osg::Array::BIND_PER_VERTEX);
        gm->setColorArray(list_cx,osg::Array::BIND_OVERALL);
        gm->addPrimitiveSet(list_ix);

        osg::ref_ptr<osg::Geode> gd = new osg::Geode;
        gd->addDrawable(gm);
        tile->gm->addChild(gd);
    }

    // RB
    {
        double const min_lon = mid_lon;
        double const max_lon = tile->max_lon;
        double const min_lat = tile->min_lat;
        double const max_lat = mid_lat;

        osg::ref_ptr<osg::Vec3dArray> list_vx = new osg::Vec3dArray;
        osg::ref_ptr<osg::Vec2dArray> list_tx = new osg::Vec2dArray;
        osg::ref_ptr<osg::Vec4Array>  list_cx = new osg::Vec4Array;
        osg::ref_ptr<osg::DrawElementsUShort> list_ix =
                new osg::DrawElementsUShort(GL_TRIANGLES);

        BuildEarthSurfaceGeometry(min_lon,
                                  min_lat,
                                  max_lon,
                                  max_lat,
                                  half_lon_segments,
                                  half_lat_segments,
                                  list_vx.get(),
                                  list_tx.get(),
                                  list_ix.get());

        osg::Vec4 cx = K_COLOR_TABLE[tile->level];
        cx.r() *= 0.75;
        cx.g() *= 0.75;
        cx.b() *= 0.75;
        list_cx->push_back(cx);

        osg::ref_ptr<osg::Geometry> gm = new osg::Geometry;
        gm->setVertexArray(list_vx);
        gm->setTexCoordArray(0,list_tx,osg::Array::BIND_PER_VERTEX);
        gm->setTexCoordArray(1,list_tx,osg::Array::BIND_PER_VERTEX);
        gm->setColorArray(list_cx,osg::Array::BIND_OVERALL);
        gm->addPrimitiveSet(list_ix);

        osg::ref_ptr<osg::Geode> gd = new osg::Geode;
        gd->addDrawable(gm);
        tile->gm->addChild(gd);
    }

    // RT
    {
        double const min_lon = mid_lon;
        double const max_lon = tile->max_lon;
        double const min_lat = mid_lat;
        double const max_lat = tile->max_lat;

        osg::ref_ptr<osg::Vec3dArray> list_vx = new osg::Vec3dArray;
        osg::ref_ptr<osg::Vec2dArray> list_tx = new osg::Vec2dArray;
        osg::ref_ptr<osg::Vec4Array>  list_cx = new osg::Vec4Array;
        osg::ref_ptr<osg::DrawElementsUShort> list_ix =
                new osg::DrawElementsUShort(GL_TRIANGLES);

        BuildEarthSurfaceGeometry(min_lon,
                                  min_lat,
                                  max_lon,
                                  max_lat,
                                  half_lon_segments,
                                  half_lat_segments,
                                  list_vx.get(),
                                  list_tx.get(),
                                  list_ix.get());

        osg::Vec4 cx = K_COLOR_TABLE[tile->level];
        cx.r() *= 1.0;
        cx.g() *= 1.0;
        cx.b() *= 1.0;
        list_cx->push_back(cx);

        osg::ref_ptr<osg::Geometry> gm = new osg::Geometry;
        gm->setVertexArray(list_vx);
        gm->setTexCoordArray(0,list_tx,osg::Array::BIND_PER_VERTEX);
        gm->setTexCoordArray(1,list_tx,osg::Array::BIND_PER_VERTEX);
        gm->setColorArray(list_cx,osg::Array::BIND_OVERALL);
        gm->addPrimitiveSet(list_ix);

        osg::ref_ptr<osg::Geode> gd = new osg::Geode;
        gd->addDrawable(gm);
        tile->gm->addChild(gd);
    }

    tile->tx_level = tile->level+1;
}

void GetOrCreateTileGeometry(XYTile * tile)
{
    auto it = map_tile_geometry_cache.find(tile->id);
    if(it != map_tile_geometry_cache.end()) {
        it->second.first = true;
        tile->gm = it->second.second;
    }
    else {
        if(map_tile_geometry_cache.size() == 48) {
            for(it = map_tile_geometry_cache.begin();
                it!= map_tile_geometry_cache.end(); ++it)
            {
                if(it->second.first == false) {
                    map_tile_geometry_cache.erase(it->first);
                    break;
                }
            }
        }

        if(map_tile_geometry_cache.size() < 48) {
            BuildXYTileGeometry(tile);
            std::pair<uint64_t,std::pair<bool,osg::ref_ptr<osg::Switch>>> ins_data;
            ins_data.first = tile->id;
            ins_data.second.first = true;
            ins_data.second.second = tile->gm;
            map_tile_geometry_cache.insert(ins_data);
        }
    }
}

void GenQuadTreeForGeoBounds(std::vector<std::vector<GeoBounds>> const &list_lod_geobb,
                             std::unique_ptr<XYTile> &tile)
{
    // If the next lod's geobounds exist and this tile
    // intersects with them, subdivide the tile
    if(CheckTileOverlapsGeoBounds(list_lod_geobb[tile->level+1],tile))
    {
        // Note:
        // Creating this tile's geometry before subdividing
        // will have different characteristics for how tile
        // images are fetched

        // Need to try both before and after subdivision to
        // see which works better
        if(tile->gm == nullptr) {
            GetOrCreateTileGeometry(tile.get());
        }

//        // Update the texture level if necessary
//        if(tile->tx_level != tile->level) {
//            // Try and find the right image ...
//        }

        uint8_t const lv = tile->level+1;
        uint32_t const x = tile->x*2;
        uint32_t const y = tile->y*2;

        if(tile->tile_LT == nullptr) {
            tile->tile_LT.reset(new XYTile(lv,x,y+1,tile.get()));
        }
        if(tile->tile_LB == nullptr) {
            tile->tile_LB.reset(new XYTile(lv,x,y,tile.get()));
        }
        if(tile->tile_RB == nullptr) {
            tile->tile_RB.reset(new XYTile(lv,x+1,y,tile.get()));
        }
        if(tile->tile_RT == nullptr) {
            tile->tile_RT.reset(new XYTile(lv,x+1,y+1,tile.get()));
        }

        GenQuadTreeForGeoBounds(list_lod_geobb,tile->tile_LT);
        GenQuadTreeForGeoBounds(list_lod_geobb,tile->tile_LB);
        GenQuadTreeForGeoBounds(list_lod_geobb,tile->tile_RB);
        GenQuadTreeForGeoBounds(list_lod_geobb,tile->tile_RT);

        // Set the clip visibility
        if(tile->gm) { // its possible gm_cache was full
            tile->gm->setValue(0,(tile->tile_LT == nullptr));
            tile->gm->setValue(1,(tile->tile_LB == nullptr));
            tile->gm->setValue(2,(tile->tile_RB == nullptr));
            tile->gm->setValue(3,(tile->tile_RT == nullptr));
        }
    }
    else {
        // Else draw this tile if it intersects with the
        // current lod geobounds
        if(CheckTileOverlapsGeoBounds(list_lod_geobb[tile->level],tile))
        {
            if(tile->gm == nullptr) {
                GetOrCreateTileGeometry(tile.get());
            }

            // Set the clip visibility
            if(tile->gm) {  // its possible gm_cache was full
                tile->gm->setAllChildrenOn();
            }

            tile->tile_LT = nullptr;
            tile->tile_LB = nullptr;
            tile->tile_RB = nullptr;
            tile->tile_RT = nullptr;
        }
        else {
            tile = nullptr;
        }
    }
}

void GenGmListFromQuadTree(std::unique_ptr<XYTile> &tile,
                           osg::Group * gp)
{
    if(!(tile->gm == nullptr)) {
        gp->addChild(tile->gm);
    }
    if(tile->tile_LT) {
        GenGmListFromQuadTree(tile->tile_LT,gp);
    }
    if(tile->tile_LB) {
        GenGmListFromQuadTree(tile->tile_LB,gp);
    }
    if(tile->tile_RB) {
        GenGmListFromQuadTree(tile->tile_RB,gp);
    }
    if(tile->tile_RT) {
        GenGmListFromQuadTree(tile->tile_RT,gp);
    }
}
