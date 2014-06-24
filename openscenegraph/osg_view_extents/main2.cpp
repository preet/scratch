// sys includes
#include <math.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <sys/time.h>
#include <cassert>
#include <memory>
#include <chrono>

// osg includes
#include <osg/Vec3>
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

// obb
#include "obb.hpp"
#include "tri.hpp"

// geometric defines!
// PI!
#define K_PI 3.141592653589
#define K_DEG2RAD K_PI/180.0
#define K_RAD2DEG 180.0/K_PI

// epsilon error
#define K_EPS 1E-11
#define K_NEPS -1E-11

// average radius
#define RAD_AV 6371000.0

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
uint16_t const K_MAX_VX_LOD = 6;
uint16_t const K_MIN_VX_LOD = 2;

std::vector<osg::Vec4> const K_COLOR_TABLE {
    {0., 0., 0., 1.},
    {41/255., 41/255., 41/255., 0.},
    {102/255., 102/255., 102/255., 0.},
    {140/255., 140/255., 140/255., 0.},
    {200/255., 200/255., 200/255., 0.},
    {66/255., 206/255., 252/255., 0.},
    {124/255., 160/255., 252/255., 0.},
    {173/255., 146/255., 252/255., 0.},
    {255/255., 120/255., 252/255., 0.},
    {255/255., 117/255., 172/255., 0.},
    {255/255., 142/255., 107/255., 0.},
    {252/255., 174/255., 91/255., 0.},
    {252/255., 194/255., 0/255., 0.},
    {202/255., 245/255., 29/255., 0.},
    {0/255., 191/255., 0/255., 0.},
    {100/255., 245/255., 174/255., 0.},
    {0/255., 235/255., 231/255., 0.},
    {255/255., 255/255., 255/255., 0.}
};

struct VxTile;
std::vector<std::vector<VxTile*>> g_list_lod_vxtiles(K_MAX_VX_LOD);

std::vector<double> CalcLodDistances();
std::vector<double> g_list_lod_dist = CalcLodDistances();

std::vector<osg::Vec3d> g_list_frustum_plane_norms(6);
std::vector<osg::Vec3d> g_list_frustum_plane_pts(6);
std::vector<osg::Vec3d> g_list_frustum_culling_test_pts;
std::vector<osg::Vec4> g_list_frustum_culling_test_colors;

osg::Vec3d g_horizon_plane_norm;
osg::Vec3d g_horizon_plane_pt;

volatile bool g_calc_view_extents = false;

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

enum TileGmClip
{
    TILE_TL = 1 << 0,
    TILE_BL = 1 << 1,
    TILE_BR = 1 << 2,
    TILE_TR = 1 << 3
};

struct VxTile
{
    double minLon;
    double maxLon;

    double minLat;
    double maxLat;

    double midLat;
    double midLon;

    osg::Vec3d ecef_tl;
    osg::Vec3d ecef_bl;
    osg::Vec3d ecef_br;
    osg::Vec3d ecef_tr;
    osg::Vec3d ecef_tm;
    osg::Vec3d ecef_bm;
    osg::Vec3d ecef_ml;
    osg::Vec3d ecef_mr;
    osg::Vec3d ecef_mm;

    osg::Vec3d bbox_center;
    osg::Vec3d bbox_ext;

    uint16_t level;

    std::unique_ptr<VxTile> top_left;
    std::unique_ptr<VxTile> top_right;
    std::unique_ptr<VxTile> btm_left;
    std::unique_ptr<VxTile> btm_right;

    osg::ref_ptr<osg::Geometry> gm_tl;
    osg::ref_ptr<osg::Geometry> gm_bl;
    osg::ref_ptr<osg::Geometry> gm_br;
    osg::ref_ptr<osg::Geometry> gm_tr;
    uint8_t gm_clip;
    bool gm_dirty;

    VxTile() :
        level(0),
        gm_tl(nullptr),
        gm_bl(nullptr),
        gm_br(nullptr),
        gm_tr(nullptr),
        gm_clip(0),
        gm_dirty(true)

    {
        // empty
    }
};

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
    // The solution for intersection points between a ray
    // and the Earth's surface is a quadratic equation

    // first calculate the quadratic equation params:
    // a(x^2) + b(x) + c

    // a, b and c are found by substituting the parametric
    // equation of a line into the equation for a ellipsoid
    // and solving in terms of the line's parameter

    // * http://en.wikipedia.org/wiki/Ellipsoid
    // * http://gis.stackexchange.com/questions/20780/...
    //   ...point-of-intersection-for-a-ray-and-earths-surface

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

//bool CalcApproxBoundingSphere(std::vector<osg::Vec3d> const &list_vx,
//                              osg::Vec3d & sphere_center,
//                              double & sphere_radius)
//{
//    if(list_vx.size() < 3) {
//        return false;
//    }

//    // From Real-Time Collision Detection by Ericson p 89

//    // Get the bbox min and max
//    osg::Vec3d min = list_vx[0];
//    osg::Vec3d max = list_vx[0];

//    for(auto const & vx : list_vx) {
//        min.x() = std::min(min.x(),vx.x());
//        min.y() = std::min(min.y(),vx.y());
//        min.z() = std::min(min.z(),vx.z());

//        max.x() = std::max(max.x(),vx.x());
//        max.y() = std::max(max.y(),vx.y());
//        max.z() = std::max(max.z(),vx.z());
//    }

//    // Let the center of the sphere be the bbox center
//    sphere_center = (min+max)*0.5;

//    // Set the radius as the furthest distance between
//    // the center and any single point in list_vx
//    sphere_radius = 0;
//    for(auto const & vx : list_vx) {
//        sphere_radius = std::max(sphere_radius,(vx-sphere_center).length2());
//    }

//    sphere_radius = sqrt(sphere_radius);
//    return true;
//}

void CalcBoundingBox(std::vector<osg::Vec3d> const &list_vx,
                     osg::Vec3d & bbox_center,
                     osg::Vec3d & bbox_extents)
{
    // Get the bbox min and max
    osg::Vec3d min = list_vx[0];
    osg::Vec3d max = list_vx[0];

    for(auto const & vx : list_vx) {
        min.x() = std::min(min.x(),vx.x());
        min.y() = std::min(min.y(),vx.y());
        min.z() = std::min(min.z(),vx.z());

        max.x() = std::max(max.x(),vx.x());
        max.y() = std::max(max.y(),vx.y());
        max.z() = std::max(max.z(),vx.z());
    }

    bbox_center = (min+max)*0.5;
    bbox_extents = (max-bbox_center);
}

void CalcBoundingBox(std::vector<osg::Vec3d const *> const &list_vx,
                     osg::Vec3d & bbox_center,
                     osg::Vec3d & bbox_extents)
{
    // Get the bbox min and max
    osg::Vec3d min = *(list_vx[0]);
    osg::Vec3d max = *(list_vx[0]);

    for(auto const & vx : list_vx) {
        min.x() = std::min(min.x(),vx->x());
        min.y() = std::min(min.y(),vx->y());
        min.z() = std::min(min.z(),vx->z());

        max.x() = std::max(max.x(),vx->x());
        max.y() = std::max(max.y(),vx->y());
        max.z() = std::max(max.z(),vx->z());
    }

    bbox_center = (min+max)*0.5;
    bbox_extents = (max-bbox_center);
}

bool CalcHorizonPlane(osg::Vec3d const &eye,
                      osg::Vec3d &horizon_norm,
                      osg::Vec3d &horizon_pt)
{
    // We need to clamp eye_length such that the
    // eye is outside of the celestial body surface

    // It would be more accurate to do a RayBodyIntersection
    double eye_length = eye.length();
    if(eye_length < (RAD_AV*1.2)) {
        return false;
    }
    double const inv_dist = 1.0/eye_length;

    horizon_norm = eye*inv_dist;
    horizon_pt = horizon_norm * (RAD_AV*RAD_AV*inv_dist);
    return true;
}

//double CalcDistPointPlane(osg::Vec3d const &plane_norm,
//                          osg::Vec3d const &plane_pt,
//                          osg::Vec3d const &distal_pt)
//{
//    double a = plane_norm.x();
//    double b = plane_norm.y();
//    double c = plane_norm.z();
//    double d = -1 * (a*plane_pt.x() + b*plane_pt.y() + c*plane_pt.z());

//    double distance = (a*distal_pt.x() + b*distal_pt.y() + c*distal_pt.z() + d) /
//        sqrt(a*a + b*b + c*c);

//    return distance;
//}

double CalcDistPointPlane(osg::Vec3d const &plane_norm,
                          osg::Vec3d const &plane_pt,
                          osg::Vec3d const &distal_pt)
{
    // From Real-Time Collision by Christner Ericson p. 127

    // calculate d
    double const d = plane_norm*plane_pt;
    return ((plane_norm*distal_pt)-d)/(plane_norm*plane_norm);
}

//bool CalcFrustumIntersectsSphere(osg::Vec3d const &sphere_center,
//                                 double const sphere_radius)
//{
//    // Calculate the signed distance between the
//    // sphere_center and the frustum plane

//    for(size_t i=0; i < 6; i++) {
//        double const dist = CalcDistPointPlane(g_list_frustum_plane_norms[i],
//                                               g_list_frustum_plane_pts[i],
//                                               sphere_center);

//        if(dist > sphere_radius) {
//            // its outside this plane
//            return false;
//        }
//    }
//    return true;
//}

bool CalcFrustumIntersectsBox(osg::Vec3d const &bbox_center,
                              osg::Vec3d const &bbox_ext)
{
    for(size_t i=0; i < 6; i++) {
        // min distance between bbox center and plane
        double const dist = CalcDistPointPlane(g_list_frustum_plane_norms[i],
                                               g_list_frustum_plane_pts[i],
                                               bbox_center);

        // find the proj of the maximally distal point of the bbox
        // in the direction of the plane normal in the bbox onto
        // the plane normal ?
        double const r =
                bbox_ext.x()*fabs(g_list_frustum_plane_norms[i].x()) +
                bbox_ext.y()*fabs(g_list_frustum_plane_norms[i].y()) +
                bbox_ext.z()*fabs(g_list_frustum_plane_norms[i].z());

        if(dist > r) {
            // bbox is outside this plane
            return false;
        }
    }
    return true;
}

bool CalcBBoxIsBehindPlane(osg::Vec3d const &plane_normal,
                           osg::Vec3d const &plane_pt,
                           osg::Vec3d const &bbox_center,
                           osg::Vec3d const &bbox_ext)
{
    // min distance between bbox center and plane
    double const dist = CalcDistPointPlane(plane_normal,
                                           plane_pt,
                                           bbox_center);

    // find the proj of the maximally distal point of the bbox
    // in the direction of the plane normal in the bbox onto
    // the plane normal ?
    double const r =
            bbox_ext.x()*fabs(plane_normal.x()) +
            bbox_ext.y()*fabs(plane_normal.y()) +
            bbox_ext.z()*fabs(plane_normal.z());

    if(dist < -r) {
        // bbox is completely behind the plane
        return true;
    }
    return false;
}

std::vector<double> CalcLodDistances()
{
    std::vector<double> list_lod_dist;

    for(size_t i=0; i < K_MAX_LOD; i++) {
        double dist = (sqrt(K_MAX_LOD_AREA_REF_M)*0.5 /
                       tan(FOVY_DEGS*0.5*K_DEG2RAD)) * pow(2,i);

        list_lod_dist.push_back(dist);
    }

    std::reverse(list_lod_dist.begin(),
                 list_lod_dist.end());

    return list_lod_dist;
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

osg::ref_ptr<osg::AutoTransform> BuildLodRingsNode()
{
    osg::ref_ptr<osg::Geode> gd_rings = new osg::Geode;
    gd_rings->getOrCreateStateSet()->setMode( GL_LIGHTING,
                                              osg::StateAttribute::OFF |
                                              osg::StateAttribute::PROTECTED );

    // For each ring
    for(size_t i=0; i < K_MAX_LOD; i++) {
        // Get the distance for this LOD
        double dist = g_list_lod_dist[i];
        osg::Vec4 color = K_COLOR_TABLE[i];

        // Create a ring of vertices
        osg::ref_ptr<osg::Vec3dArray> list_vx = new osg::Vec3dArray((i+1)*2 + 12);
        double const rotate_by_rads = (2.0*K_PI/list_vx->size());

        for(size_t j=0; j < list_vx->size(); j++){
            list_vx->at(j) = osg::Vec3d(
                        dist*cos(rotate_by_rads*j),
                        dist*sin(rotate_by_rads*j),
                        0);
        }

        osg::ref_ptr<osg::Vec4Array> list_cx = new osg::Vec4Array;
        list_cx->push_back(color);

        osg::ref_ptr<osg::Geometry> gm = new osg::Geometry;
        gm->setVertexArray(list_vx);
        gm->setColorArray(list_cx,osg::Array::BIND_OVERALL);
        gm->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_LOOP,0,list_vx->size()));

        gd_rings->addDrawable(gm.get());
    }

    osg::ref_ptr<osg::AutoTransform> xf_rings = new osg::AutoTransform;
    xf_rings->addChild(gd_rings);
    xf_rings->setAutoRotateMode(osg::AutoTransform::ROTATE_TO_SCREEN);

    return xf_rings;
}

osg::ref_ptr<osg::Group> BuildHorizonPlaneNode(osg::Camera * camera)
{
    osg::ref_ptr<osg::Group> gp_horizon_plane = new osg::Group;

    gp_horizon_plane->setName("horizonplane");

    osg::Vec3d eye,vpt,up;
    if(camera) {
        camera->getViewMatrixAsLookAt(eye,vpt,up);

        osg::Vec3d horizon_norm,horizon_pt;
        if(CalcHorizonPlane(eye,horizon_norm,horizon_pt)) {
            // save
            g_horizon_plane_norm = horizon_norm;
            g_horizon_plane_pt = horizon_pt;

            // Draw the plane as a circle centered on horizon_pt
            // with radius RAD_AV*0.5
            osg::ref_ptr<osg::Vec3dArray> list_vx = new osg::Vec3dArray(16);
            double const rotate_by_rads = (2.0*K_PI/list_vx->size());
            double const dist = RAD_AV*1.1;

            // Rotate the circle so that its normal is
            // aligned to the horizon_norm
            osg::Matrixd rotate_to_horizon =
                    osg::Matrixd::rotate(osg::Vec3d(0,0,1),
                                         horizon_norm);

            for(size_t i=0; i < 16; i++) {
                list_vx->at(i) = osg::Vec3d(
                            dist*cos(rotate_by_rads*i),
                            dist*sin(rotate_by_rads*i),
                            0);

                list_vx->at(i) = list_vx->at(i) * rotate_to_horizon;
                list_vx->at(i) = list_vx->at(i) + horizon_pt;
            }

            osg::ref_ptr<osg::Vec4Array> list_cx = new osg::Vec4Array;
            list_cx->push_back(osg::Vec4(0.4,0.5,0.9,1.0)); // smurple

            osg::ref_ptr<osg::Geometry> gm = new osg::Geometry;
            gm->setVertexArray(list_vx);
            gm->setColorArray(list_cx,osg::Array::BIND_OVERALL);
            gm->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_LOOP,0,list_vx->size()));

            osg::ref_ptr<osg::Geode> gd = new osg::Geode;
            gd->addDrawable(gm);

            gp_horizon_plane->addChild(gd);
        }
    }

    return gp_horizon_plane;
}

osg::ref_ptr<osg::MatrixTransform> BuildFrustumNode(osg::Camera * camera)
{
    // Projection and ModelView matrices

    osg::Matrixd proj;
    osg::Matrixd mv;
    osg::Matrixd vm;

    if (camera)
    {
        proj = camera->getProjectionMatrix();
        mv = camera->getViewMatrix();
        vm = camera->getViewMatrix();
    }
    else
    {
        // Create some kind of reasonable default Projection matrix.
        proj.makePerspective( 30., 1., 1., 10. );
        // leave mv as identity
    }

    osg::Matrixd const mv_inv = osg::Matrixd::inverse( mv );
    osg::Matrixd const vm_inv = osg::Matrixd::inverse(vm);

    // Get near and far from the Projection matrix.
    const double near = proj(3,2) / (proj(2,2)-1.0);
    const double far = proj(3,2) / (1.0+proj(2,2));

    // Get the sides of the near plane.
    const double nLeft = near * (proj(2,0)-1.0) / proj(0,0);
    const double nRight = near * (1.0+proj(2,0)) / proj(0,0);
    const double nTop = near * (1.0+proj(2,1)) / proj(1,1);
    const double nBottom = near * (proj(2,1)-1.0) / proj(1,1);

    // Get the sides of the far plane.
    const double fLeft = far * (proj(2,0)-1.0) / proj(0,0);
    const double fRight = far * (1.0+proj(2,0)) / proj(0,0);
    const double fTop = far * (1.0+proj(2,1)) / proj(1,1);
    const double fBottom = far * (proj(2,1)-1.0) / proj(1,1);

    // Our vertex array needs only 9 vertices: The origin, and the
    // eight corners of the near and far planes.
    osg::ref_ptr<osg::Vec3dArray> v = new osg::Vec3dArray;
    v->resize( 21 );

    // near and far are negated because the opengl
    // camera is at (0,0,0) with the view dirn pointing
    // down the -Z axis

    osg::Vec3d NBL(nLeft,nBottom,-near);
    NBL = NBL * mv_inv;

    osg::Vec3d NBR(nRight,nBottom,-near);
    NBR = NBR *mv_inv;

    osg::Vec3d NTR(nRight,nTop,-near);
    NTR = NTR * mv_inv;

    osg::Vec3d NTL(nLeft,nTop,-near);
    NTL = NTL * mv_inv;

    osg::Vec3d FBL(fLeft, fBottom, -far);
    FBL = FBL * mv_inv;

    osg::Vec3d FBR(fRight, fBottom, -far);
    FBR = FBR * mv_inv;

    osg::Vec3d FTR(fRight, fTop, -far);
    FTR = FTR * mv_inv;

    osg::Vec3d FTL(fLeft, fTop, -far);
    FTL = FTL* mv_inv;

    // get the normals for the frustum planes
    osg::Vec3d p_left = NBL+ (FTL-NBL)*0.5;
    osg::Vec3d d_left = (FTL-NTL)^(NBL-NTL); d_left.normalize();

    osg::Vec3d p_right = (NBR+FTR)*0.5;
    osg::Vec3d d_right = (NTR-FTR)^(FBR-FTR); d_right.normalize();

    osg::Vec3d p_top = (NTL+FTR)*0.5;
    osg::Vec3d d_top = (FTR-NTR)^(NTL-NTR); d_top.normalize();

    osg::Vec3d p_btm = (NBL+FBR)*0.5;
    osg::Vec3d d_btm = (FBL-NBL)^(NBR-NBL); d_btm.normalize();

    osg::Vec3d p_near = (NBL+NTR)*0.5;
    osg::Vec3d d_near = (NTL-NTR)^(NBR-NTR); d_near.normalize();

    osg::Vec3d p_far = (FBL+FTR)*0.5;
    osg::Vec3d d_far = (FTR-FBL)^(FBL-FTL); d_far.normalize();

    // save
    {
        g_list_frustum_plane_pts[0] = p_left;
        g_list_frustum_plane_pts[1] = p_right;
        g_list_frustum_plane_pts[2] = p_top;
        g_list_frustum_plane_pts[3] = p_btm;
        g_list_frustum_plane_pts[4] = p_near;
        g_list_frustum_plane_pts[5] = p_far;

        g_list_frustum_plane_norms[0] = d_left;
        g_list_frustum_plane_norms[1] = d_right;
        g_list_frustum_plane_norms[2] = d_top;
        g_list_frustum_plane_norms[3] = d_btm;
        g_list_frustum_plane_norms[4] = d_near;
        g_list_frustum_plane_norms[5] = d_far;

    }

    // get a length to show the normals
    double const normal_length = (FTR-FBR).length()*0.5;
    d_left *= normal_length;
    d_right *= normal_length;
    d_top *= normal_length;
    d_btm *= normal_length;
    d_near *= normal_length;
    d_far *= normal_length;

    v->at(0).set(0.,0.,0.);
    v->at(0) = v->at(0) * mv_inv;

    v->at(1) = NBL;
    v->at(2) = NBR;
    v->at(3) = NTR;
    v->at(4) = NTL;

    v->at(5) = FBL;
    v->at(6) = FBR;
    v->at(7) = FTR;
    v->at(8) = FTL;

    v->at(9) = p_left;
    v->at(10) = p_left+d_left;

    v->at(11) = p_right;
    v->at(12) = p_right+d_right;

    v->at(13) = p_top;
    v->at(14) = p_top+d_top;

    v->at(15) = p_btm;
    v->at(16) = p_btm+d_btm;

    v->at(17) = p_near;
    v->at(18) = p_near+d_near;

    v->at(19) = p_far;
    v->at(20) = p_far+d_far;


    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
    geom->setUseDisplayList( false );
    geom->setVertexArray( v );

    osg::ref_ptr<osg::Vec4Array> c = new osg::Vec4Array;
    c->push_back(osg::Vec4(1,1,1,1));
    geom->setColorArray( c, osg::Array::BIND_OVERALL );

    GLushort idxLines[8] = {
        0, 5, 0, 6, 0, 7, 0, 8 };
    GLushort idxLoops0[4] = {
        1, 2, 3, 4 };
    GLushort idxLoops1[4] = {
        5, 6, 7, 8 };
    GLushort idxNormals[12] = {
        9,10,11,12,13,14,15,16,17,18,19,20 };
    geom->addPrimitiveSet( new osg::DrawElementsUShort( osg::PrimitiveSet::LINES, 8, idxLines ) );
    geom->addPrimitiveSet( new osg::DrawElementsUShort( osg::PrimitiveSet::LINE_LOOP, 4, idxLoops0 ) );
    geom->addPrimitiveSet( new osg::DrawElementsUShort( osg::PrimitiveSet::LINE_LOOP, 4, idxLoops1 ) );
    geom->addPrimitiveSet( new osg::DrawElementsUShort( osg::PrimitiveSet::LINES, 12, idxNormals ) );

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->addDrawable( geom );

    // Create parent MatrixTransform to transform the view volume by
    // the inverse ModelView matrix.
    osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform;
    mt->setName("frustum");
//    mt->setMatrix( mv_inv );
    mt->addChild( geode );

    return mt;
}


osg::ref_ptr<osg::Group> BuildCelestialSurfaceNode()
{
    std::vector<osg::Vec3d> list_vx;
    std::vector<osg::Vec2d> list_tx;
    std::vector<size_t> list_ix;

    BuildEarthSurfaceGeometry(-180,-90,
                              180,90,
                              32,16,
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
    cx_array->push_back(osg::Vec4(0.25,0.25,0.25,1.0));

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
    gp->addChild(gd.get());

    // enable wireframe and disable lighting
    osg::ref_ptr<osg::PolygonMode> polygonMode = new osg::PolygonMode;
    polygonMode->setMode(osg::PolygonMode::FRONT_AND_BACK,
                         osg::PolygonMode::LINE);

    gp->getOrCreateStateSet()->setAttribute(polygonMode.get());
    gp->setName("celestialbody");

    return gp;
}

osg::ref_ptr<osg::Group> BuildFrustumCullingTest()
{
    if(g_list_frustum_culling_test_colors.empty()) {
        for(size_t i=0; i < 25; i++) {
            g_list_frustum_culling_test_colors.push_back(osg::Vec4(1,1,1,1));
        }
    }

    if(g_list_frustum_culling_test_pts.empty()) {
        g_list_frustum_culling_test_pts.resize(25);
    }

    int k = 0;

    osg::ref_ptr<osg::Group> gp = new osg::Group;

    for(size_t n=0; n < 5; n++)
    {
        for(size_t m=0; m < 5; m++)
        {
            osg::ref_ptr<osg::Vec3dArray> list_vx =
                    new osg::Vec3dArray(10);

            double const radius = 100;
            double const rotate_by_rads = (2.0*K_PI/list_vx->size());

            for(size_t i=0; i < list_vx->size(); i++) {
                list_vx->at(i) = osg::Vec3d(
                            radius*cos(rotate_by_rads*i),
                            0,
                            radius*sin(rotate_by_rads*i));
            }

            osg::ref_ptr<osg::Vec4Array> list_cx = new osg::Vec4Array;
            list_cx->push_back(g_list_frustum_culling_test_colors[k]);

            osg::ref_ptr<osg::Geometry> gm = new osg::Geometry;
            gm->setVertexArray(list_vx);
            gm->setColorArray(list_cx,osg::Array::BIND_OVERALL);
            gm->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_LOOP,0,list_vx->size()));

            osg::ref_ptr<osg::Geode> gd = new osg::Geode;
            gd->addDrawable(gm.get());

            osg::ref_ptr<osg::MatrixTransform> xf = new osg::MatrixTransform;
            xf->setMatrix(osg::Matrixd::translate(osg::Vec3d(n*250,0.0,m*250)));
//            xf->setAutoRotateMode(osg::AutoTransform::ROTATE_TO_SCREEN);
//            xf->setPosition(osg::Vec3d(n*250,m*250,0));
            xf->addChild(gd);

            gp->addChild(xf);

            g_list_frustum_culling_test_pts[k] = (osg::Vec3d(n*250,0.0,m*250));

            k++;
        }
    }

    gp->setName("culltest");
    return gp;
}

void BuildTile(VxTile * tile,
               double minLon,
               double minLat,
               double maxLon,
               double maxLat,
               uint16_t level)
{
    tile->minLon = minLon;
    tile->minLat = minLat;
    tile->maxLon = maxLon;
    tile->maxLat = maxLat;
    tile->midLon = (minLon+maxLon)*0.5;
    tile->midLat = (minLat+maxLat)*0.5;

    PointLLA const lla_tl(tile->minLon,tile->maxLat);
    PointLLA const lla_bl(tile->minLon,tile->minLat);
    PointLLA const lla_br(tile->maxLon,tile->minLat);
    PointLLA const lla_tr(tile->maxLon,tile->maxLat);
    PointLLA const lla_mm(tile->midLon,tile->midLat);
    PointLLA const lla_tm(tile->midLon,tile->maxLat);
    PointLLA const lla_bm(tile->midLon,tile->minLat);
    PointLLA const lla_ml(tile->minLon,tile->midLat);
    PointLLA const lla_mr(tile->maxLon,tile->midLat);

    tile->ecef_tl = ConvLLAToECEF(lla_tl);
    tile->ecef_bl = ConvLLAToECEF(lla_bl);
    tile->ecef_br = ConvLLAToECEF(lla_br);
    tile->ecef_tr = ConvLLAToECEF(lla_tr);
    tile->ecef_mm = ConvLLAToECEF(lla_mm);
    tile->ecef_tm = ConvLLAToECEF(lla_tm);
    tile->ecef_bm = ConvLLAToECEF(lla_bm);
    tile->ecef_ml = ConvLLAToECEF(lla_ml);
    tile->ecef_mr = ConvLLAToECEF(lla_mr);

    std::vector<osg::Vec3d> list_vx;
    list_vx.push_back(tile->ecef_tl);
    list_vx.push_back(tile->ecef_bl);
    list_vx.push_back(tile->ecef_br);
    list_vx.push_back(tile->ecef_tr);

    CalcBoundingBox(list_vx,
                    tile->bbox_center,
                    tile->bbox_ext);

    tile->level = level;
}

void BuildTile(VxTile * tile,
               double minLon,
               double minLat,
               double maxLon,
               double maxLat,
               osg::Vec3d const &ecef_tl,
               osg::Vec3d const &ecef_bl,
               osg::Vec3d const &ecef_br,
               osg::Vec3d const &ecef_tr,
               uint16_t level)
{
    tile->minLon = minLon;
    tile->minLat = minLat;
    tile->maxLon = maxLon;
    tile->maxLat = maxLat;
    tile->midLon = (minLon+maxLon)*0.5;
    tile->midLat = (minLat+maxLat)*0.5;

    tile->ecef_tl = ecef_tl;
    tile->ecef_bl = ecef_bl;
    tile->ecef_br = ecef_br;
    tile->ecef_tr = ecef_tr;

    PointLLA const lla_mm(tile->midLon,tile->midLat);
    PointLLA const lla_tm(tile->midLon,tile->maxLat);
    PointLLA const lla_bm(tile->midLon,tile->minLat);
    PointLLA const lla_ml(tile->minLon,tile->midLat);
    PointLLA const lla_mr(tile->maxLon,tile->midLat);

    tile->ecef_mm = ConvLLAToECEF(lla_mm);
    tile->ecef_tm = ConvLLAToECEF(lla_tm);
    tile->ecef_bm = ConvLLAToECEF(lla_bm);
    tile->ecef_ml = ConvLLAToECEF(lla_ml);
    tile->ecef_mr = ConvLLAToECEF(lla_mr);

    std::vector<osg::Vec3d> list_vx;
    list_vx.push_back(tile->ecef_tl);
    list_vx.push_back(tile->ecef_bl);
    list_vx.push_back(tile->ecef_br);
    list_vx.push_back(tile->ecef_tr);

    CalcBoundingBox(list_vx,
                    tile->bbox_center,
                    tile->bbox_ext);

    tile->level = level;
}

void SplitTile(VxTile * parent)
{
    // Split this tile into 4   
    PointLLA lla_cen;
    lla_cen.lon = (parent->minLon + parent->maxLon)*0.5;
    lla_cen.lat = (parent->minLat + parent->maxLat)*0.5;
    lla_cen.alt = 0;

    PointLLA const lla_tm(lla_cen.lon,parent->maxLat);
    PointLLA const lla_bm(lla_cen.lon,parent->minLat);
    PointLLA const lla_ml(parent->minLon,lla_cen.lat);
    PointLLA const lla_mr(parent->maxLon,lla_cen.lat);

    osg::Vec3d const & ecef_cen = parent->ecef_mm;
    osg::Vec3d const & ecef_tm = parent->ecef_tm;
    osg::Vec3d const & ecef_bm = parent->ecef_bm;
    osg::Vec3d const & ecef_ml = parent->ecef_ml;
    osg::Vec3d const & ecef_mr = parent->ecef_mr;

    uint16_t const next_level = parent->level + 1;

    // TOP-LEFT
    {
        auto & tl = parent->top_left;
        tl.reset(new VxTile);

        BuildTile(tl.get(),lla_ml.lon,lla_ml.lat,lla_tm.lon,lla_tm.lat,parent->ecef_tl,ecef_ml,ecef_cen,ecef_tm,next_level);
    }

    // TOP-RIGHT
    {
        auto & tr = parent->top_right;
        tr.reset(new VxTile);

        BuildTile(tr.get(),lla_cen.lon,lla_cen.lat,parent->maxLon,parent->maxLat,ecef_tm,ecef_cen,ecef_mr,parent->ecef_tr,next_level);
    }

    // BOTTOM-LEFT
    {
        auto & bl = parent->btm_left;
        bl.reset(new VxTile);

        BuildTile(bl.get(),parent->minLon,parent->minLat,lla_cen.lon,lla_cen.lat,ecef_ml,parent->ecef_bl,ecef_bm,ecef_cen,next_level);
    }

    // BOTTOM-RIGHT
    {
        auto & br = parent->btm_right;
        br.reset(new VxTile);

        BuildTile(br.get(),lla_bm.lon,lla_bm.lat,lla_mr.lon,lla_mr.lat,ecef_cen,ecef_bm,parent->ecef_br,ecef_mr,next_level);
    }
}

osg::ref_ptr<osg::Geometry> QuickBuildTileGeometry(VxTile * tile, uint8_t quad)
{
    osg::ref_ptr<osg::Vec3dArray> list_vx = new osg::Vec3dArray(4);

    if(quad == 0) {

    }
    if(quad == 2) {

    }
    if(quad == 4) {

    }
    if(quad == 8) {

    }
    list_vx->at(0) = tile->ecef_tl;
    list_vx->at(1) = tile->ecef_bl;
    list_vx->at(2) = tile->ecef_br;
    list_vx->at(3) = tile->ecef_tr;

    osg::ref_ptr<osg::Vec4Array> list_cx =
            new osg::Vec4Array(1);
    list_cx->at(0) = osg::Vec4(1.0,1.0,1.0,1.0);

    osg::ref_ptr<osg::Geometry> gm = new osg::Geometry;
    gm->setVertexArray(list_vx);
    gm->setColorArray(list_cx,osg::Array::BIND_OVERALL);
    gm->addPrimitiveSet(new osg::DrawArrays(
        osg::PrimitiveSet::LINE_LOOP,0,list_vx->size()));

    return gm;
}

size_t g_tile_count=0;

void BuildViewExtentsGeometry(VxTile * tile, osg::ref_ptr<osg::Group> &gp_vx_tiles)
{

    //  Build the geometry for this tile
    if(tile->gm_dirty) {

        osg::ref_ptr<osg::Geode> gd = new osg::Geode;

        if(!(tile->top_left || tile->gm_tl)) {
            // build gm_tl
            tile->gm_tl = QuickBuildTileGeometry(tile,0);
            gd->addDrawable(tile->gm_tl);
        }

        if(!(tile->btm_left || tile->gm_bl)) {
            // build gm_bl
            tile->gm_bl = QuickBuildTileGeometry(tile,2);
            gd->addDrawable(tile->gm_bl);
        }

        if(!(tile->btm_right || tile->gm_br)) {
            // build gm_br
            tile->gm_br = QuickBuildTileGeometry(tile,4);
            gd->addDrawable(tile->gm_br);
        }

        if(!(tile->top_right || tile->gm_tr)) {
            // build gm_tr
            tile->gm_tr = QuickBuildTileGeometry(tile,8);
            gd->addDrawable(tile->gm_tr);
        }

        if(gd->getNumDrawables() > 0) {
            gp_vx_tiles->addChild(gd);
        }

        tile->gm_dirty = false;
    }


    if(tile->top_left) {
        BuildViewExtentsGeometry(tile->top_left.get(),gp_vx_tiles);
    }
    if(tile->btm_left) {
        BuildViewExtentsGeometry(tile->btm_left.get(),gp_vx_tiles);
    }
    if(tile->btm_right) {
        BuildViewExtentsGeometry(tile->btm_right.get(),gp_vx_tiles);
    }
    if(tile->top_right) {
        BuildViewExtentsGeometry(tile->top_right.get(),gp_vx_tiles);
    }
}

bool BuildViewExtents(osg::Vec3d const &eye,
                      VxTile * tile)
{
    g_tile_count++;

    // Check if this tile is visible with the
    // curent view frustum:

    // Note: we can combine the horizon test as
    // just another plane in the frustum test...

    bool visible = ((!CalcBBoxIsBehindPlane(g_horizon_plane_norm,
                                            g_horizon_plane_pt,
                                            tile->bbox_center,
                                            tile->bbox_ext)) &&

                      CalcFrustumIntersectsBox(tile->bbox_center,
                                               tile->bbox_ext));

    if(!visible) {
        return false;
    }

    // Check if this tile needs to be split
    // based on distance from the eye
    if(tile->level < (K_MAX_LOD-1)) {
        bool xsec = false;
        if(TriangleIntersectsSphere(tile->ecef_tl,
                                    tile->ecef_bl,
                                    tile->ecef_br,
                                    eye,
                                    g_list_lod_dist[tile->level]))
        {
            xsec = true;
        }
        else if(TriangleIntersectsSphere(tile->ecef_tl,
                                         tile->ecef_br,
                                         tile->ecef_tr,
                                         eye,
                                         g_list_lod_dist[tile->level]))
        {
            xsec = true;
        }

        if(xsec) {
            SplitTile(tile); // optimize by only calling when necessary
            uint8_t temp_clip = 0;

            if(BuildViewExtents(eye,tile->top_left.get())) {
                temp_clip |= 1;
            }
            else {
                tile->top_left = nullptr;
            }

            if(BuildViewExtents(eye,tile->btm_left.get())) {
                temp_clip |= 2;
            }
            else {
                tile->btm_left = nullptr;
            }

            if(BuildViewExtents(eye,tile->btm_right.get())) {
                temp_clip |= 4;
            }
            else {
                tile->btm_right = nullptr;
            }

            if(BuildViewExtents(eye,tile->top_right.get())) {
                temp_clip |= 8;
            }
            else {
                tile->top_right = nullptr;
            }

            if(!(temp_clip == tile->gm_clip)) {
                tile->gm_dirty = true;
            }
        }
    }

    return true;
}

std::vector<VxTile*> BuildBaseViewExtents()
{
    std::vector<VxTile*> list_vxtiles;

    // Start at level 2 [4 x 4]
    size_t const num_tiles_side = 4;
    double lon_step = 360.0/num_tiles_side;
    double lat_step = 180.0/num_tiles_side;

    for(size_t i=0; i < num_tiles_side; i++) { // lon
        for(size_t j=0; j < num_tiles_side; j++) { // lat
            VxTile * vxtile = new VxTile;

            vxtile->minLon = -180.0 + lon_step*i;
            vxtile->maxLon = vxtile->minLon + lon_step;

            vxtile->minLat = -90.0 + lat_step*j;
            vxtile->maxLat = vxtile->minLat + lat_step;

            BuildTile(vxtile,vxtile->minLon,vxtile->minLat,vxtile->maxLon,vxtile->maxLat,2);

            list_vxtiles.push_back(vxtile);
        }
    }

    return list_vxtiles;
}

osg::ref_ptr<osg::Group> BuildBaseViewExtentsGeometry(std::vector<VxTile*> const &list_vx_tiles)
{
    osg::ref_ptr<osg::Group> gp = new osg::Group;

    for(auto & vx_tile : list_vx_tiles)
    {
        // Create the bounding sphere geometry for this tile
//        osg::ref_ptr<osg::ShapeDrawable> bsphere =
//                new osg::ShapeDrawable(
//                    new osg::Sphere(vx_tile->bsphere_center,
//                                    vx_tile->bsphere_radius));

        osg::ref_ptr<osg::ShapeDrawable> bbox =
                new osg::ShapeDrawable(
                    new osg::Box(vx_tile->bbox_center,
                                 vx_tile->bbox_ext.x()*2.0,
                                 vx_tile->bbox_ext.y()*2.0,
                                 vx_tile->bbox_ext.z()*2.0));

        bbox->setColor(osg::Vec4(0.5,0.5,0.5,0.1));

        osg::ref_ptr<osg::Geode> gd = new osg::Geode;
        gd->addDrawable(bbox);
        gp->addChild(gd);
    }

    gp->getOrCreateStateSet()->setMode ( GL_DEPTH_TEST, osg::StateAttribute::OFF );
    gp->setName("bboxes");
    return gp;
}

void UpdateBaseViewExtentsGeometryColor(osg::ref_ptr<osg::Group> bboxes,
                                        size_t const index,
                                        osg::Vec4 const &color)
{
    osg::Geode * gd = static_cast<osg::Geode*>(
                bboxes->getChild(index));

    osg::ShapeDrawable * bbox = static_cast<osg::ShapeDrawable*>(
                gd->getDrawable(0));

    bbox->setColor(color);
}

class KeyboardEventHandler : public osgGA::GUIEventHandler
{
public:
    virtual bool handle(osgGA::GUIEventAdapter const &event_adapter,
                        osgGA::GUIActionAdapter &action_adapter)
    {
        (void)action_adapter;

        switch(event_adapter.getEventType())
        {
            case osgGA::GUIEventAdapter::KEYDOWN:
            {
                switch(event_adapter.getKey())
                {
                    case osgGA::GUIEventAdapter::KEY_Space:
                    {
                        g_calc_view_extents = true;
                        return true;
                    }
                    default: {
                        return false;
                    }
                }
                // fall through to default?
            }
            default: {
                return false;
            }
        }
        return false;
    }

    virtual void accept(osgGA::GUIEventHandlerVisitor &v)
    {
        v.visit(*this);
    }
};

int main(int argc, const char *argv[])
{
    (void)argc;
    (void)argv;

    // Build the base view extents. We always start finding
    // detailed view extents by testing the base extents against
    // the frustum.
    std::vector<VxTile*> list_base_vx_tiles = BuildBaseViewExtents();
    osg::ref_ptr<osg::Group> gp_vx_tiles = new osg::Group;

    // Celestial body surface mesh
    auto gp_celestial = BuildCelestialSurfaceNode();

    // Bounding spheres for the base vx tiles
    auto gp_bboxes = BuildBaseViewExtentsGeometry(list_base_vx_tiles);

    // View0 root
    osg::ref_ptr<osg::Group> gp_root0 = new osg::Group;
    gp_root0->addChild(gp_celestial);
    gp_root0->addChild(gp_bboxes);
    gp_root0->addChild(gp_vx_tiles);

    // View1 root
    osg::ref_ptr<osg::Group> gp_root1 = new osg::Group;
    gp_root1->addChild(gp_celestial);
    gp_root1->addChild(gp_bboxes);
    gp_root1->addChild(gp_vx_tiles);

    // disable lighting and enable blending
    gp_root0->getOrCreateStateSet()->setMode( GL_LIGHTING,osg::StateAttribute::OFF);
    gp_root0->getOrCreateStateSet()->setMode( GL_BLEND,osg::StateAttribute::ON);
    gp_root0->getOrCreateStateSet()->setMode( GL_DEPTH_TEST,osg::StateAttribute::OFF);

    gp_root1->getOrCreateStateSet()->setMode( GL_LIGHTING,osg::StateAttribute::OFF);
    gp_root1->getOrCreateStateSet()->setMode( GL_BLEND,osg::StateAttribute::ON);
    gp_root1->getOrCreateStateSet()->setMode( GL_DEPTH_TEST,osg::StateAttribute::OFF);


    osgViewer::CompositeViewer viewer;
    viewer.setThreadingModel(osgViewer::ViewerBase::SingleThreaded);

    // Create View 0
    {
        osgViewer::View* view = new osgViewer::View;
        viewer.addView( view );

        view->setUpViewInWindow( 10, 10, 640, 480 );
        view->setSceneData( gp_root0.get() );
        view->getCamera()->setClearColor(osg::Vec4(0.1,0.1,0.1,1.0));
        view->setCameraManipulator( new osgGA::TrackballManipulator );
    }

    // Create view 1
    {
        osgViewer::View* view = new osgViewer::View;
        viewer.addView( view );

        view->addEventHandler(new KeyboardEventHandler());
        view->setUpViewInWindow( 10, 510, 640, 480 );
        view->setSceneData( gp_root1.get() );
        view->getCamera()->setClearColor(osg::Vec4(0.1,0.1,0.1,1.0));
        view->setCameraManipulator( new osgGA::TrackballManipulator );
    }

    // Create the fixed base VxTiles we use to
    // calculate view extents

    while (!viewer.done())
    {
        osg::Camera * camera = viewer.getView(0)->getCamera();

        // Create a new camera frustum node
        auto new_frustum = BuildFrustumNode(camera);

        // Create a new horizon plane node
        auto new_horizonplane = BuildHorizonPlaneNode(camera);

        // Get camera eye
        osg::Vec3d cam_eye;
        {
            osg::Vec3d cam_up,cam_vpt;
            camera->getViewMatrixAsLookAt(cam_eye,cam_vpt,cam_up);
        }

        // Update the base view extent geometry
        {
            g_tile_count=0;
            std::chrono::time_point<std::chrono::system_clock> start,end;
            start = std::chrono::system_clock::now();

            gp_vx_tiles->removeChildren(0,gp_vx_tiles->getNumChildren());

            for(size_t i=0; i < list_base_vx_tiles.size(); i++) {
                if(BuildViewExtents(cam_eye,list_base_vx_tiles[i])) {
                    UpdateBaseViewExtentsGeometryColor(gp_bboxes,i,osg::Vec4(1.0,0.5,0.0,0.05));
                    BuildViewExtentsGeometry(list_base_vx_tiles[i],gp_vx_tiles);
                }
                else {
                    UpdateBaseViewExtentsGeometryColor(gp_bboxes,i,osg::Vec4(0.5,0.5,0.5,0.05));
                }
            }
            end = std::chrono::system_clock::now();
            std::chrono::duration<double> elapsed_seconds = end-start;
//            std::cout << "###: took: " << elapsed_seconds.count()*1000.0
//                      << " ms for " << g_tile_count << " tiles" << std::endl;
        }


        // Update gp_root0
        {
            for(size_t i=0; i < gp_root0->getNumChildren(); i++) {
                std::string const name = gp_root0->getChild(i)->getName();
                if(name == "horizonplane") {
                    gp_root0->removeChild(i);
                    i--;
                }
            }
            // Add new nodes
            gp_root0->addChild(new_horizonplane);
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
            }
            // Add new nodes
            gp_root1->addChild(new_frustum);
            gp_root1->addChild(new_horizonplane);
        }
        viewer.frame();
    }
    return 0;
}
