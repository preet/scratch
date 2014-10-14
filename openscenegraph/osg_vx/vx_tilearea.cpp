#include <iostream>
#include <vector>
#include <chrono>

#include <osg/io_utils>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/PrimitiveSet>
#include <osg/PolygonMode>
#include <osgViewer/Viewer>
#include <osg/Material>
#include <osg/MatrixTransform>
#include <osg/AutoTransform>
#include <osgViewer/CompositeViewer>
#include <osgGA/TrackballManipulator>
#include <osg/ShapeDrawable>

#define K_PI 3.141592653589
#define K_DEG2RAD K_PI/180.0
#define K_RAD2DEG 180.0/K_PI

// epsilon error
#define K_EPS 1E-11
#define K_NEPS -1E-11

// average radius
#define RAD_AV 6371000.0

struct PointLLA
{
    PointLLA() :
        lon(0),lat(0),alt(0) {}

    PointLLA(double lon, double lat) :
        lon(lon),lat(lat),alt(0) {}

    PointLLA(double lon, double lat, double alt) :
        lon(lon),lat(lat),alt(alt) {}

    double lon;
    double lat;
    double alt;
};

PointLLA ConvECEFToLLA(const osg::Vec3d &pointECEF)
{
    double radius = pointECEF.length();

    PointLLA pointLLA;
    pointLLA.lon = (atan2(pointECEF.y(),pointECEF.x()))*K_RAD2DEG;
    pointLLA.lat = (asin(pointECEF.z()/radius))*K_RAD2DEG;
    pointLLA.alt = radius-RAD_AV;

    return pointLLA;
}

osg::Vec3d ConvLLAToECEF(const PointLLA &pointLLA)
{
    // remember to convert deg->rad
    double sinLat = sin(pointLLA.lat * K_DEG2RAD);
    double sinLon = sin(pointLLA.lon * K_DEG2RAD);
    double cosLat = cos(pointLLA.lat * K_DEG2RAD);
    double cosLon = cos(pointLLA.lon * K_DEG2RAD);
    double radius = RAD_AV+pointLLA.alt;

    osg::Vec3d pointECEF;
    pointECEF.x() = cosLat * cosLon * radius;
    pointECEF.y() = cosLat * sinLon * radius;
    pointECEF.z() = sinLat * radius;

    return pointECEF;
}

void BuildEarthSurfaceGeometry(double min_lon,
                               double max_lon,
                               double min_lat,
                               double max_lat,
                               uint16_t lon_segments,
                               uint16_t lat_segments,
                               std::vector<osg::Vec3d> &list_vx,
                               std::vector<osg::Vec2d> &list_tx,
                               std::vector<uint16_t> &list_ix)
{
    double const lon_step = (max_lon-min_lon)/lon_segments;
    double const lat_step = (max_lat-min_lat)/lat_segments;

    list_vx.clear();
    list_tx.clear();
    list_ix.clear();

    // build vertex attributes
    list_vx.reserve((lat_segments+1)*(lon_segments+1));
    list_tx.reserve((lat_segments+1)*(lon_segments+1));
    for(uint16_t i=0; i <= lat_segments; i++)   {
        for(uint16_t j=0; j <= lon_segments; j++)   {
            // surface vertex
            PointLLA lla;
            lla.lon = (j*lon_step)+min_lon;
            lla.lat = (i*lat_step)+min_lat;
            lla.alt = 0.0;
            list_vx.push_back(ConvLLAToECEF(lla));

            // surface tex coord
            list_tx.push_back(osg::Vec2d(double(j)/lon_segments,
                                         double(i)/lat_segments));
        }
    }

    // stitch faces together
    list_ix.reserve(lon_segments*lat_segments*6);
    uint16_t v_idx=0;
    for(uint16_t i=0; i < lat_segments; i++)   {
        for(uint16_t j=0; j < lon_segments; j++)   {
            list_ix.push_back(v_idx);
            list_ix.push_back(v_idx+lon_segments+2);
            list_ix.push_back(v_idx+lon_segments+1);

            list_ix.push_back(v_idx);
            list_ix.push_back(v_idx+1);
            list_ix.push_back(v_idx+lon_segments+2);

            v_idx++;
        }
        v_idx++;
    }
}

void BuildEarthSurfaceGeometry(double min_lon,
                               double max_lon,
                               double min_lat,
                               double max_lat,
                               uint16_t lon_segments,
                               uint16_t lat_segments,
                               std::vector<osg::Vec3d> &list_vx,
                               std::vector<uint16_t> &list_ix)
{
    double const lon_step = (max_lon-min_lon)/lon_segments;
    double const lat_step = (max_lat-min_lat)/lat_segments;

    list_vx.clear();
    list_ix.clear();

    // build vertex attributes
    list_vx.reserve((lat_segments+1)*(lon_segments+1));
    for(uint16_t i=0; i <= lat_segments; i++)   {
        for(uint16_t j=0; j <= lon_segments; j++)   {
            // surface vertex
            PointLLA lla;
            lla.lon = (j*lon_step)+min_lon;
            lla.lat = (i*lat_step)+min_lat;
            lla.alt = 0.0;
            list_vx.push_back(ConvLLAToECEF(lla));
        }
    }

    // stitch faces together
    list_ix.reserve(lon_segments*lat_segments*6);
    uint16_t v_idx=0;
    for(uint16_t i=0; i < lat_segments; i++)   {
        for(uint16_t j=0; j < lon_segments; j++)   {
            list_ix.push_back(v_idx);
            list_ix.push_back(v_idx+lon_segments+2);
            list_ix.push_back(v_idx+lon_segments+1);

            list_ix.push_back(v_idx);
            list_ix.push_back(v_idx+1);
            list_ix.push_back(v_idx+lon_segments+2);

            v_idx++;
        }
        v_idx++;
    }
}

osg::ref_ptr<osg::Group> BuildSurfaceTile(double min_lon,
                                          double max_lon,
                                          double min_lat,
                                          double max_lat,
                                          uint16_t lon_segments,
                                          uint16_t lat_segments)
{
    osg::ref_ptr<osg::Group> gp = new osg::Group;

    std::vector<osg::Vec3d> list_vx;
    std::vector<osg::Vec2d> list_tx;
    std::vector<uint16_t> list_ix;
    BuildEarthSurfaceGeometry(min_lon,
                              max_lon,
                              min_lat,
                              max_lat,
                              lon_segments,
                              lat_segments,
                              list_vx,
                              list_tx,
                              list_ix);

    osg::ref_ptr<osg::Vec3dArray> vx_array = new osg::Vec3dArray;
    vx_array->reserve(list_vx.size());

    osg::ref_ptr<osg::Vec3Array> nx_array = new osg::Vec3Array;
    nx_array->reserve(list_vx.size());

    osg::ref_ptr<osg::Vec4Array> cx_array = new osg::Vec4Array;
    cx_array->push_back(osg::Vec4(1,1,1,1));

    for(auto const &vx : list_vx) {
        vx_array->push_back(vx);

        osg::Vec3 nx(vx.x(),vx.y(),vx.z());
        nx.normalize();
        nx_array->push_back(nx);
    }

    osg::ref_ptr<osg::DrawElementsUShort> ix_array =
            new osg::DrawElementsUShort(GL_TRIANGLES);
    for(auto const ix : list_ix) {
        ix_array->push_back(ix);
    }

    // geometry
    osg::ref_ptr<osg::Geometry> gm = new osg::Geometry;
    gm->setVertexArray(vx_array);
    gm->setNormalArray(nx_array,osg::Array::Binding::BIND_PER_VERTEX);
    gm->setColorArray(cx_array,osg::Array::Binding::BIND_OVERALL);
    gm->addPrimitiveSet(ix_array);

    // geode
    osg::ref_ptr<osg::Geode> gd = new osg::Geode;
    gd->addDrawable(gm);

    // save
    gp->addChild(gd);
    gp->setName("surftile");

    return gp;
}

bool ConvWorldToScreen(double screen_width,
                       double screen_height,
                       osg::Matrixd const &mvp,
                       osg::Vec3d const &world,
                       osg::Vec2d &screen)
{
    osg::Vec4d world4(world.x(),world.y(),world.z(),1.0);
    osg::Vec4d clip4 = world4 * mvp;
    if(fabs(clip4.w()) < K_EPS) {
        return false;
    }

    osg::Vec4d ndc4 = clip4/clip4.w();
    screen.x() = (ndc4.x()+1.0)*0.5*screen_width;
    screen.y() = (1.0-ndc4.y())*0.5*screen_height;
    return true;
}

bool ConvWorldToNDC(osg::Matrixd const &mvp,
                    osg::Vec3d const &world,
                    osg::Vec2d &ndc)
{
    osg::Vec4d world4(world.x(),world.y(),world.z(),1.0);
    osg::Vec4d clip4 = world4 * mvp;
    if(fabs(clip4.w()) < K_EPS) {
        return false;
    }

    osg::Vec4d ndc4 = clip4/clip4.w();
    ndc.x() = ndc4.x();
    ndc.y() = ndc4.y();
    return true;
}

std::pair<bool,osg::Vec2d> ConvWorldToNDC(osg::Matrixd const &mvp,
                                          osg::Vec3d const &world)
{
    osg::Vec4d world4(world.x(),world.y(),world.z(),1.0);
    osg::Vec4d clip4 = world4 * mvp;
    if(fabs(clip4.w()) < K_EPS) {
        return std::pair<bool,osg::Vec2d>(false,osg::Vec2d(0,0));
    }

    osg::Vec4d ndc4 = clip4/clip4.w();
    return std::pair<bool,osg::Vec2d>(true,osg::Vec2d(ndc4.x(),ndc4.y()));
}

double const K_MAX_POS_DBL = std::numeric_limits<double>::max();
double const K_MIN_NEG_DBL = -K_MAX_POS_DBL;

bool CalcOutside(osg::Vec2d const &polyA_axis_dirn,
                 osg::Vec2d const &polyA_axis_pt,
                 std::vector<osg::Vec2d> const &polyB_list_vx)
{
    for(auto const &vx : polyB_list_vx) {
        double t = polyA_axis_dirn * (vx - polyA_axis_pt);
        if(!(t > 0)) {
            return false;
        }
    }
    return true;
}

inline osg::Vec2d CalcPerpendicular(osg::Vec2d v)
{
    std::swap(v.x(),v.y());
    v.y() *= -1.0;
    return v;
}

bool CalcTriangleNDCRectIntersection(std::vector<osg::Vec2d> const &tri,
                                     std::vector<osg::Vec2d> const &rect)
{
    osg::Vec2d const &min = rect[0];
    osg::Vec2d const &max = rect[2];

    // First check if any vertices from the
    // triangle lie in the NDC window bounds
    for(size_t i=0; i < 3; i++) {
        bool outside =
                (tri[i].x() < min.x()) ||
                (tri[i].x() > max.x()) ||
                (tri[i].y() < min.y()) ||
                (tri[i].y() > max.y());

        if(!outside) {
            return true;
        }
    }

    // Next do SAT
    // (test triangle against rectangle edge normals)
    // left
    if(CalcOutside(osg::Vec2d(-1.0,0.0),min,tri)) {
        return false;
    }
    // bottom
    if(CalcOutside(osg::Vec2d(0.0,-1.0),min,tri)) {
        return false;
    }
    // right
    if(CalcOutside(osg::Vec2d(1.0,0.0),max,tri)) {
        return false;
    }
    // top
    if(CalcOutside(osg::Vec2d(0.0,1.0),max,tri)) {
        return false;
    }

    // (test rectangle against triangle edge normals)
    if(CalcOutside(CalcPerpendicular(tri[1]-tri[0]),tri[0],rect)) {
        return false;
    }
    if(CalcOutside(CalcPerpendicular(tri[2]-tri[1]),tri[1],rect)) {
        return false;
    }
    if(CalcOutside(CalcPerpendicular(tri[0]-tri[2]),tri[2],rect)) {
        return false;
    }

    return true;
}


double CalcTriangleArea(osg::Vec2d const &vxA,
                        osg::Vec2d const &vxB,
                        osg::Vec2d const &vxC)
{
    // http://www.mathopenref.com/coordtrianglearea.html
    return fabs((vxA.x()*(vxB.y()-vxC.y()) +
                 vxB.x()*(vxC.y()-vxA.y()) +
                 vxC.x()*(vxA.y()-vxB.y()))/2.0);
}

double CalcTileNDCArea(osg::Camera const * camera,
                       std::vector<osg::Vec3d> const &list_vx,
                       std::vector<uint16_t> const &list_ix,
                       std::vector<osg::Vec3d> const &list_tri_nx)
{


    osg::Matrixd const mvp =
            camera->getViewMatrix() *
            camera->getProjectionMatrix();

    osg::Vec3d eye,vpt,up;
    camera->getViewMatrixAsLookAt(eye,vpt,up);

    osg::Vec3d view_dirn = vpt-eye;

    // Convert tile ecef positions to NDC
    std::vector<std::pair<bool,osg::Vec2d>> list_ndc;
    list_ndc.reserve(list_vx.size());

    for(auto const &vx : list_vx) {
        list_ndc.emplace_back(ConvWorldToNDC(mvp,vx));
    }

    // NDC bounding rectangle
    static const std::vector<osg::Vec2d> ndc_rect = {
        osg::Vec2d(-1,-1),    // bl
        osg::Vec2d( 1,-1),    // br
        osg::Vec2d( 1, 1),    // tr
        osg::Vec2d(-1, 1)     // tl
    };

    // For each triangle
    double area=0.0;
    for(size_t i=0; i < list_ix.size(); i+=3) {
        // If this is a valid, front facing triangle
        if((list_tri_nx[i/3]*view_dirn < 0) &&
           list_ndc[list_ix[i+0]].first &&
           list_ndc[list_ix[i+1]].first &&
           list_ndc[list_ix[i+2]].first)
        {
            // Check if its within the view frustum
            std::vector<osg::Vec2d> tri = {
                list_ndc[list_ix[i+0]].second,
                list_ndc[list_ix[i+1]].second,
                list_ndc[list_ix[i+2]].second
            };

            if(CalcTriangleNDCRectIntersection(tri,ndc_rect)) {
                area += CalcTriangleArea(tri[0],tri[1],tri[2]);
            }
        }
    }

    return area;
}

void UpdateSurfaceTileColor(osg::Group * gp_surftile,
                            osg::Vec4 const &color)
{
    osg::Geode * gd = static_cast<osg::Geode*>(gp_surftile->getChild(0));
    osg::Geometry * gm = static_cast<osg::Geometry*>(gd->getDrawable(0));

    osg::ref_ptr<osg::Vec4Array> cx_array = new osg::Vec4Array;
    cx_array->push_back(color);

    gm->setColorArray(cx_array,osg::Array::Binding::BIND_OVERALL);
}

int main()
{
    // define a tile
    double min_lon = -180;
    double max_lon = 180;
    double min_lat = -90.0;
    double max_lat = 90.0;
    double mid_lon = (min_lon+max_lon)*0.5;
    double mid_lat = (min_lat+max_lat)*0.5;

    // get tile vertices
    osg::Vec3d ecef_LT = ConvLLAToECEF(PointLLA(min_lon,max_lat));
    osg::Vec3d ecef_LB = ConvLLAToECEF(PointLLA(min_lon,min_lat));
    osg::Vec3d ecef_RB = ConvLLAToECEF(PointLLA(max_lon,min_lat));
    osg::Vec3d ecef_RT = ConvLLAToECEF(PointLLA(max_lon,max_lat));
    osg::Vec3d ecef_MM = ConvLLAToECEF(PointLLA(mid_lon,mid_lat));
    std::vector<osg::Vec3d> list_ecef;
    list_ecef.push_back(ecef_LT);
    list_ecef.push_back(ecef_LB);
    list_ecef.push_back(ecef_RB);
    list_ecef.push_back(ecef_RT);
    list_ecef.push_back(ecef_MM);

    // surface tile geometry
    auto gp_surftile = BuildSurfaceTile(min_lon,
                                        max_lon,
                                        min_lat,
                                        max_lat,
                                        16,8);

    // tile pixel area estimation geometry
    std::vector<osg::Vec3d> list_vx;
    std::vector<uint16_t> list_ix;
    BuildEarthSurfaceGeometry(min_lon,
                              max_lon,
                              min_lat,
                              max_lat,
                              16,
                              16,
                              list_vx,
                              list_ix);

    // calculate tile normals
    std::vector<osg::Vec3d> list_tri_nx;
    list_tri_nx.reserve(list_ix.size()/3);
    for(size_t i=0; i < list_ix.size(); i+=3) {
        // triangle normal
        osg::Vec3d const &v0 = list_vx[list_ix[i+0]];
        osg::Vec3d const &v1 = list_vx[list_ix[i+1]];
        osg::Vec3d const &v2 = list_vx[list_ix[i+2]];
        list_tri_nx.push_back((v1-v0)^(v2-v0));

        // TODO normalize? dont think its necessary
    }

    // root
    osg::ref_ptr<osg::Group> gp_root = new osg::Group;
    gp_root->addChild(gp_surftile);

    // setup state
    gp_root->getOrCreateStateSet()->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
    gp_root->getOrCreateStateSet()->setMode(GL_BLEND,osg::StateAttribute::ON);
    gp_root->getOrCreateStateSet()->setMode(GL_DEPTH_TEST,osg::StateAttribute::ON);
    gp_root->getOrCreateStateSet()->setMode(GL_CULL_FACE,osg::StateAttribute::OFF);

    // create view
    double const view_width  = 600;
    double const view_height = 360;
    double const view_area = view_width*view_height;
    osgViewer::Viewer viewer;
    viewer.setThreadingModel(osgViewer::ViewerBase::SingleThreaded);
    viewer.setUpViewInWindow(100,100,view_width,view_height);
    viewer.setSceneData(gp_root);
    viewer.getCamera()->setClearColor(osg::Vec4(0.1,0.1,0.1,1.0));

    osg::ref_ptr<osgGA::TrackballManipulator> view_manip =
            new osgGA::TrackballManipulator;
    view_manip->setMinimumDistance(100);
    viewer.setCameraManipulator(view_manip);

    while(!viewer.done())
    {
        std::chrono::time_point<std::chrono::system_clock> start, end;
        start = std::chrono::system_clock::now();
        double ndc_area = CalcTileNDCArea(viewer.getCamera(),
                                          list_vx,
                                          list_ix,
                                          list_tri_nx);
        end = std::chrono::system_clock::now();

        // ndc_area bounds are -1 to +1 in x and y with an area
        // of 4 units;
        double px_area = ndc_area * (view_area/4.0);

        std::chrono::duration<double> elapsed_seconds = end-start;
        std::cout << ": px_area: " << px_area
                  << ", took: " << elapsed_seconds.count()*1000.0 << " ms" << std::endl;
        viewer.frame();
    }

    return 0;
}
