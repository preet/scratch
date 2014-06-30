// sys includes
#include <math.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <sys/time.h>

// osg includes
#include <osg/Vec3>
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
#include <osg/io_utils>

// geometric defines
#define K_PI 3.141592653589
#define K_DEG2RAD K_PI/180.0
#define K_RAD2DEG 180.0/K_PI

// epsilon error
#define K_EPS 1E-11
#define K_NEPS -1E-11

// average radius
#define RAD_AV 6371000

// WGS84 ellipsoid parameters
// (http://en.wikipedia.org/wiki/WGS_84)
#define ELL_SEMI_MAJOR 6378137.0            // meters
#define ELL_SEMI_MAJOR_EXP2 40680631590769.0

#define ELL_SEMI_MINOR 6356752.3142         // meters
#define ELL_SEMI_MINOR_EXP2 40408299984087.1

#define ELL_F 1.0/298.257223563
#define ELL_ECC_EXP2 6.69437999014e-3
#define ELL_ECC2_EXP2 6.73949674228e-3

// circumference
#define CIR_EQ 40075017.0   // around equator  (meters)
#define CIR_MD 40007860.0   // around meridian (meters)
#define CIR_AV 40041438.0   // average (meters)

size_t const K_MAX_LOD = 18;
double const K_MAX_LOD_AREA_REF_M = 150*150;
double FOVY_DEGS = 35.0;

double const K_MAX_POS_DBL = std::numeric_limits<double>::max();
double const K_MIN_NEG_DBL = -K_MAX_POS_DBL;

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

struct GeoBounds
{
    double minLon;
    double minLat;
    double maxLon;
    double maxLat;
};

GeoBounds g_geobounds;

PointLLA ConvECEFToLLA(const osg::Vec3d &pointECEF)
{
    PointLLA pointLLA;

    double p = (sqrt(pow(pointECEF.x(),2) + pow(pointECEF.y(),2)));
    double th = atan2(pointECEF.z()*ELL_SEMI_MAJOR, p*ELL_SEMI_MINOR);
    double sinTh = sin(th);
    double cosTh = cos(th);

    // calc longitude
    pointLLA.lon = atan2(pointECEF.y(), pointECEF.x());

    // calc latitude
    pointLLA.lat = atan2(pointECEF.z() +
                         ELL_ECC2_EXP2*ELL_SEMI_MINOR*sinTh*sinTh*sinTh,
                         p - ELL_ECC_EXP2*ELL_SEMI_MAJOR*cosTh*cosTh*cosTh);
    // calc altitude
    double sinLat = sin(pointLLA.lat);
    double N = ELL_SEMI_MAJOR / (sqrt(1-(ELL_ECC_EXP2*sinLat*sinLat)));
    pointLLA.alt = (p/cos(pointLLA.lat)) - N;

    // convert from rad to deg
    pointLLA.lon = pointLLA.lon * K_RAD2DEG;
    pointLLA.lat = pointLLA.lat * K_RAD2DEG;

    return pointLLA;
}

osg::Vec3d ConvLLAToECEF(const PointLLA &pointLLA)
{
    osg::Vec3d pointECEF;

    // remember to convert deg->rad
    double sinLat = sin(pointLLA.lat * K_DEG2RAD);
    double sinLon = sin(pointLLA.lon * K_DEG2RAD);
    double cosLat = cos(pointLLA.lat * K_DEG2RAD);
    double cosLon = cos(pointLLA.lon * K_DEG2RAD);

    // v = radius of curvature (meters)
    double v = ELL_SEMI_MAJOR / (sqrt(1-(ELL_ECC_EXP2*sinLat*sinLat)));
    pointECEF.x() = (v + pointLLA.alt) * cosLat * cosLon;
    pointECEF.y() = (v + pointLLA.alt) * cosLat * sinLon;
    pointECEF.z() = ((1-ELL_ECC_EXP2)*v + pointLLA.alt)*sinLat;

    return pointECEF;
}

void CalcQuadraticEquationReal(double a, double b, double c,
                               std::vector<double> &listRoots)
{
    // check discriminant
    double myDiscriminant = b*b - 4*a*c;

    if(myDiscriminant > 0)
    {
        double qSeg1 = (-1*b)/(2*a);
        double qSeg2 = sqrt(myDiscriminant)/(2*a);
        listRoots.push_back(qSeg1+qSeg2);
        listRoots.push_back(qSeg1-qSeg2);
    }
}

bool CalcRayEarthIntersection(osg::Vec3d const &rayPoint,
                              osg::Vec3d const &rayDirn,
                              osg::Vec3d &xsecNear,
                              osg::Vec3d &xsecFar)
{
    double a = ((rayDirn.x()*rayDirn.x()) / ELL_SEMI_MAJOR_EXP2) +
               ((rayDirn.y()*rayDirn.y()) / ELL_SEMI_MAJOR_EXP2) +
               ((rayDirn.z()*rayDirn.z()) / ELL_SEMI_MINOR_EXP2);

    double b = (2*rayPoint.x()*rayDirn.x()/ELL_SEMI_MAJOR_EXP2) +
               (2*rayPoint.y()*rayDirn.y()/ELL_SEMI_MAJOR_EXP2) +
               (2*rayPoint.z()*rayDirn.z()/ELL_SEMI_MINOR_EXP2);

    double c = ((rayPoint.x()*rayPoint.x()) / ELL_SEMI_MAJOR_EXP2) +
               ((rayPoint.y()*rayPoint.y()) / ELL_SEMI_MAJOR_EXP2) +
               ((rayPoint.z()*rayPoint.z()) / ELL_SEMI_MINOR_EXP2) - 1;

    std::vector<double> listRoots;
    CalcQuadraticEquationReal(a,b,c,listRoots);
    if(!listRoots.empty())
    {
        if(listRoots.size() == 1)   {
            xsecNear.x() = rayPoint.x() + listRoots[0]*rayDirn.x();
            xsecNear.y() = rayPoint.y() + listRoots[0]*rayDirn.y();
            xsecNear.z() = rayPoint.z() + listRoots[0]*rayDirn.z();
            xsecFar = xsecNear;
            return true;
        }
        else   {
            xsecNear.x() = rayPoint.x() + listRoots[0]*rayDirn.x();
            xsecNear.y() = rayPoint.y() + listRoots[0]*rayDirn.y();
            xsecNear.z() = rayPoint.z() + listRoots[0]*rayDirn.z();

            xsecFar.x() = rayPoint.x() + listRoots[1]*rayDirn.x();
            xsecFar.y() = rayPoint.y() + listRoots[1]*rayDirn.y();
            xsecFar.z() = rayPoint.z() + listRoots[1]*rayDirn.z();

            if((rayPoint-xsecNear).length2() > (rayPoint-xsecFar).length2())
            {
                osg::Vec3d temp = xsecNear;
                xsecNear = xsecFar;
                xsecFar = temp;
            }
            return true;
        }
    }
    return false;
}

bool BuildEarthSurfaceGeometry(double minLon, double minLat,
                               double maxLon, double maxLat,
                               size_t lonSegments,
                               size_t latSegments,
                               std::vector<osg::Vec3d> &vertexArray,
                               std::vector<osg::Vec2d> &texCoords,
                               std::vector<size_t> &triIdx)
{
    if((!(minLon < maxLon)) || (!(minLat < maxLat)))   {
        return false;
    }

    double lonStep = (maxLon-minLon)/lonSegments;
    double latStep = (maxLat-minLat)/latSegments;

    vertexArray.clear();
    texCoords.clear();
    triIdx.clear();

    // build vertex attributes
//    vertexArray.reserve((latSegments+1)*(lonSegments+1));
//    texCoords.reserve((latSegments+1)*(lonSegments+1));
    for(size_t i=0; i <= latSegments; i++)   {
        for(size_t j=0; j <= lonSegments; j++)   {
            // surface vertex
            vertexArray.push_back(ConvLLAToECEF(PointLLA((j*lonStep)+minLon,
                                                         (i*latStep)+minLat,0.0)));
            // surface tex coord
            texCoords.push_back(osg::Vec2d(double(j)/lonSegments,
                                           double(i)/latSegments));
        }
    }

    // stitch faces together
    // TODO: optimize push_back
//    triIdx.reserve(lonSegments*latSegments*6);
    size_t vIdx=0;
    for(size_t i=0; i < latSegments; i++)   {
        for(size_t j=0; j < lonSegments; j++)   {
            triIdx.push_back(vIdx);
            triIdx.push_back(vIdx+lonSegments+2);
            triIdx.push_back(vIdx+lonSegments+1);

            triIdx.push_back(vIdx);
            triIdx.push_back(vIdx+1);
            triIdx.push_back(vIdx+lonSegments+2);

            vIdx++;
        }
        vIdx++;
    }

    return true;
}

osg::ref_ptr<osg::Group> CreateEarthSurfaceGeometryNode()
{
    std::vector<osg::Vec3d> list_vx;
    std::vector<osg::Vec2d> list_tx;
    std::vector<size_t> list_ix;

    BuildEarthSurfaceGeometry(g_geobounds.minLon,
                              g_geobounds.minLat,
                              g_geobounds.maxLon,
                              g_geobounds.maxLat,
                              8,8,
                              list_vx,
                              list_tx,
                              list_ix);

    osg::ref_ptr<osg::Vec3dArray> vx_array =
            new osg::Vec3dArray;
    for(auto const &vx : list_vx) {
        vx_array->push_back(vx);
    }

    osg::ref_ptr<osg::Vec4Array> cx_array =
            new osg::Vec4Array;
    cx_array->push_back(osg::Vec4(0.5,0.5,0.5,1.0));

    osg::ref_ptr<osg::DrawElementsUShort> ix_array =
            new osg::DrawElementsUShort(GL_TRIANGLES);
    for(auto const ix : list_ix) {
        ix_array->push_back(ix);
    }

    // geometry
    osg::ref_ptr<osg::Geometry> gm = new osg::Geometry;
    gm->setVertexArray(vx_array.get());
    gm->setColorArray(cx_array,osg::Array::Binding::BIND_OVERALL);
    gm->addPrimitiveSet(ix_array.get());

    // geode
    osg::ref_ptr<osg::Geode> gd = new osg::Geode;
    gd->addDrawable(gm.get());

    // group
    osg::ref_ptr<osg::Group> gp = new osg::Group;
    osg::ref_ptr<osg::PolygonMode> polygonMode = new osg::PolygonMode;
    polygonMode->setMode(osg::PolygonMode::FRONT_AND_BACK,
                         osg::PolygonMode::LINE);
    gp->getOrCreateStateSet()->setAttribute(polygonMode.get());

    gp->addChild(gd.get());

    return gp;
}

osg::ref_ptr<osg::Group> CreateAABBNode()
{
    std::vector<osg::Vec3d> tile_vx(5);
    tile_vx[0] = ConvLLAToECEF(PointLLA(g_geobounds.minLon,g_geobounds.maxLat)); // tl
    tile_vx[1] = ConvLLAToECEF(PointLLA(g_geobounds.minLon,g_geobounds.minLat)); // bl
    tile_vx[2] = ConvLLAToECEF(PointLLA(g_geobounds.maxLon,g_geobounds.minLat)); // br
    tile_vx[3] = ConvLLAToECEF(PointLLA(g_geobounds.maxLon,g_geobounds.maxLat)); // tr
    tile_vx[4] = ConvLLAToECEF(PointLLA((g_geobounds.minLon+g_geobounds.maxLon)*0.5,
                                        (g_geobounds.minLat+g_geobounds.maxLat)*0.5));

    osg::Vec3d min(K_MAX_POS_DBL,K_MAX_POS_DBL,K_MAX_POS_DBL);
    osg::Vec3d max(K_MIN_NEG_DBL,K_MIN_NEG_DBL,K_MIN_NEG_DBL);
    for(auto const & vx : tile_vx) {
        min.x() = std::min(min.x(),vx.x());
        min.y() = std::min(min.y(),vx.y());
        min.z() = std::min(min.z(),vx.z());

        max.x() = std::max(max.x(),vx.x());
        max.y() = std::max(max.y(),vx.y());
        max.z() = std::max(max.z(),vx.z());
    }

    // geometry
    osg::ref_ptr<osg::Vec3dArray> list_vx = new osg::Vec3dArray(8);
    list_vx->at(0) = osg::Vec3d(max.x(),min.y(),min.z()); // tl
    list_vx->at(1) = osg::Vec3d(min.x(),min.y(),min.z()); // bl
    list_vx->at(2) = osg::Vec3d(min.x(),max.y(),min.z()); // br
    list_vx->at(3) = osg::Vec3d(max.x(),max.y(),min.z()); // tr
    list_vx->at(4) = osg::Vec3d(max.x(),min.y(),max.z()); // tl
    list_vx->at(5) = osg::Vec3d(min.x(),min.y(),max.z()); // bl
    list_vx->at(6) = osg::Vec3d(min.x(),max.y(),max.z()); // br
    list_vx->at(7) = osg::Vec3d(max.x(),max.y(),max.z()); // tr

    uint16_t list_ix_top[4] = { 0, 1, 2, 3 };
    uint16_t list_ix_btm[4] = { 4, 5, 6, 7 };
    uint16_t list_ix_sides[8] = {0, 4, 1, 5, 2, 6, 3, 7 };

    osg::ref_ptr<osg::Vec4Array> list_cx = new osg::Vec4Array;
    list_cx->push_back(osg::Vec4(0,1,1,0.5));

    osg::ref_ptr<osg::Geometry> gm = new osg::Geometry;
    gm->setVertexArray(list_vx);
    gm->setColorArray(list_cx,osg::Array::BIND_OVERALL);
    gm->addPrimitiveSet(new osg::DrawElementsUShort(osg::PrimitiveSet::LINE_LOOP,4,list_ix_top));
    gm->addPrimitiveSet(new osg::DrawElementsUShort(osg::PrimitiveSet::LINE_LOOP,4,list_ix_btm));
    gm->addPrimitiveSet(new osg::DrawElementsUShort(osg::PrimitiveSet::LINES,8,list_ix_sides));

    osg::ref_ptr<osg::Geode> gd = new osg::Geode;
    gd->addDrawable(gm);

    osg::ref_ptr<osg::Group> gp = new osg::Group;
    gp->addChild(gd);

    return gp;
}

osg::ref_ptr<osg::Group> CreateOBBNode()
{
    osg::Vec3d const ecef_tl = ConvLLAToECEF(PointLLA(g_geobounds.minLon,g_geobounds.maxLat)); // tl
    osg::Vec3d const ecef_bl = ConvLLAToECEF(PointLLA(g_geobounds.minLon,g_geobounds.minLat)); // bl
    osg::Vec3d const ecef_br = ConvLLAToECEF(PointLLA(g_geobounds.maxLon,g_geobounds.minLat)); // br
    osg::Vec3d const ecef_tr = ConvLLAToECEF(PointLLA(g_geobounds.maxLon,g_geobounds.maxLat)); // tr
    osg::Vec3d const ecef_mm = ConvLLAToECEF(PointLLA((g_geobounds.minLon+g_geobounds.maxLon)*0.5,
                                                      (g_geobounds.minLat+g_geobounds.maxLat)*0.5));

    std::vector<osg::Vec3d const *> tile_vx(5);
    tile_vx[0] = &ecef_tl;
    tile_vx[1] = &ecef_bl;
    tile_vx[2] = &ecef_br;
    tile_vx[3] = &ecef_tr;
    tile_vx[4] = &ecef_mm;

    // 1. Determine the orthonormal basis
    osg::Vec3d bbox_ori[3];

    // At the poles the distance in 'x' (lon) might
    // be zero or too small so choose the vec that
    // has a greater magnitude:
    osg::Vec3d const x_top = ecef_tl-ecef_tr;
    osg::Vec3d const x_btm = ecef_bl-ecef_br;

    bbox_ori[0] = (x_top.length2() > x_btm.length2()) ? x_top : x_btm;
    bbox_ori[2] = ecef_mm;
    bbox_ori[1] = bbox_ori[2]^bbox_ori[0];

    bbox_ori[0].normalize();
    bbox_ori[1].normalize();
    bbox_ori[2].normalize();

    // 2. Get the min and max corners
    osg::Vec3d min_rst(K_MAX_POS_DBL,K_MAX_POS_DBL,K_MAX_POS_DBL);
    osg::Vec3d max_rst(K_MIN_NEG_DBL,K_MIN_NEG_DBL,K_MIN_NEG_DBL);

    // Project all the vertices of the model onto the
    // orthonormal vectors and get min/max along them
    for(auto vx_ptr : tile_vx) {
        osg::Vec3d const &vx = *vx_ptr;
        double const dist_r = bbox_ori[0]*vx;
        double const dist_s = bbox_ori[1]*vx;
        double const dist_t = bbox_ori[2]*vx;

        min_rst.x() = std::min(min_rst.x(),dist_r);
        min_rst.y() = std::min(min_rst.y(),dist_s);
        min_rst.z() = std::min(min_rst.z(),dist_t);

        max_rst.x() = std::max(max_rst.x(),dist_r);
        max_rst.y() = std::max(max_rst.y(),dist_s);
        max_rst.z() = std::max(max_rst.z(),dist_t);
    }

    // 3. Calc center and extents
    osg::Vec3d bbox_center((bbox_ori[0] * (min_rst.x()+max_rst.x())*0.5) +
                           (bbox_ori[1] * (min_rst.y()+max_rst.y())*0.5) +
                           (bbox_ori[2] * (min_rst.z()+max_rst.z())*0.5));

    osg::Vec3d bbox_ext((max_rst-min_rst)*0.5);

    // debug start

    osg::ref_ptr<osg::Vec3dArray> list_vx = new osg::Vec3dArray(8);
    list_vx->at(0) =
            bbox_center +
            osg::Vec3d(bbox_ori[0]*-bbox_ext.x() +
                       bbox_ori[1]*bbox_ext.y() +
                       bbox_ori[2]*bbox_ext.z());

    list_vx->at(1) =
            bbox_center +
            osg::Vec3d(bbox_ori[0]*-bbox_ext.x()+
                       bbox_ori[1]*-bbox_ext.y()+
                       bbox_ori[2]*bbox_ext.z());

    list_vx->at(2) =
            bbox_center +
            osg::Vec3d(bbox_ori[0]*bbox_ext.x()+
                       bbox_ori[1]*-bbox_ext.y()+
                       bbox_ori[2]*bbox_ext.z());

    list_vx->at(3) =
            bbox_center +
            osg::Vec3d(bbox_ori[0]*bbox_ext.x()+
                       bbox_ori[1]*bbox_ext.y()+
                       bbox_ori[2]*bbox_ext.z());

    list_vx->at(4) =
            bbox_center +
            osg::Vec3d(bbox_ori[0]*-bbox_ext.x() +
                       bbox_ori[1]*bbox_ext.y() +
                       bbox_ori[2]*-bbox_ext.z());

    list_vx->at(5) =
            bbox_center +
            osg::Vec3d(bbox_ori[0]*-bbox_ext.x()+
                       bbox_ori[1]*-bbox_ext.y()+
                       bbox_ori[2]*-bbox_ext.z());

    list_vx->at(6) =
            bbox_center +
            osg::Vec3d(bbox_ori[0]*bbox_ext.x()+
                       bbox_ori[1]*-bbox_ext.y()+
                       bbox_ori[2]*-bbox_ext.z());

    list_vx->at(7) =
            bbox_center +
            osg::Vec3d(bbox_ori[0]*bbox_ext.x()+
                       bbox_ori[1]*bbox_ext.y()+
                       bbox_ori[2]*-bbox_ext.z());


    osg::ref_ptr<osg::Vec4Array> list_cx = new osg::Vec4Array;
    list_cx->push_back(osg::Vec4(1,0.5,0,0.5));

    uint16_t list_ix_top_face[4] = { 0, 1, 2, 3 };
    uint16_t list_ix_btm_face[4] = { 4, 5, 6, 7 };
    uint16_t list_ix_edges[8] = { 0, 4, 1, 5, 2, 6, 3, 7 };

    osg::ref_ptr<osg::Geometry> gm = new osg::Geometry;
    gm->setVertexArray(list_vx);
    gm->setColorArray(list_cx,osg::Array::BIND_OVERALL);
    gm->addPrimitiveSet(new osg::DrawElementsUShort(osg::PrimitiveSet::LINE_LOOP,4,list_ix_top_face));
    gm->addPrimitiveSet(new osg::DrawElementsUShort(osg::PrimitiveSet::LINE_LOOP,4,list_ix_btm_face));
    gm->addPrimitiveSet(new osg::DrawElementsUShort(osg::PrimitiveSet::LINES,8,list_ix_edges));


    osg::ref_ptr<osg::Geode> gd = new osg::Geode;
    gd->addDrawable(gm);

    osg::ref_ptr<osg::Group> gp = new osg::Group;
    gp->addChild(gd);

    return gp;
    // debug end
}

int main(int argc, const char *argv[])
{
    (void)argc;
    (void)argv;

    // set bounds
    g_geobounds.minLon = -35;
    g_geobounds.maxLon = 0;

    g_geobounds.minLat = 50;
    g_geobounds.maxLat = 75;

    auto gp_earthsurf = CreateEarthSurfaceGeometryNode();
    auto gp_aabb = CreateAABBNode();
    auto gp_obb = CreateOBBNode();

    // root
    osg::ref_ptr<osg::Group> gp_root = new osg::Group;
    gp_root->addChild(gp_earthsurf);
    gp_root->addChild(gp_aabb);
    gp_root->addChild(gp_obb);

    // disable lighting and enable blending
    gp_root->getOrCreateStateSet()->setMode( GL_LIGHTING,osg::StateAttribute::OFF);
    gp_root->getOrCreateStateSet()->setMode( GL_BLEND,osg::StateAttribute::ON);
    gp_root->getOrCreateStateSet()->setMode( GL_DEPTH_TEST,osg::StateAttribute::OFF);

    // start viewer
    osgViewer::Viewer viewer;
    viewer.getCamera()->setClearColor(osg::Vec4(0.15,0.15,0.15,1.0));
    viewer.setThreadingModel(osgViewer::ViewerBase::SingleThreaded);
    viewer.setUpViewInWindow(100,100,800,480);
    viewer.setSceneData(gp_root.get());
    return viewer.run();

    return 0;
}



