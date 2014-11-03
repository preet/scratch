/*
   Copyright (C) 2014 Preet Desai (preet.desai@gmail.com)

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/

#ifndef SCRATCH_GEOMETRY_UTILS_H
#define SCRATCH_GEOMETRY_UTILS_H

#include <set>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>

#include <osg/Vec3d>
#include <osg/Vec4>
#include <osg/Matrixd>
#include <osg/io_utils>
#include <osg/Camera>

// ============================================================= //
// ============================================================= //

#define K_PI 3.141592653589
#define K_DEG2RAD K_PI/180.0
#define K_RAD2DEG 180.0/K_PI

// epsilon
#define K_EPS 1E-11
#define K_NEPS -1E-11

// average radius
#define RAD_AV 6371000.0

double const RAD_AV_INV_EXP2 = (1.0/6371000.0)*(1.0/6371000.0);

std::vector<osg::Vec4> const K_COLOR_TABLE {
    {255/255., 255/255., 255/255., 1.},
    {202/255., 245/255., 29/255., 1.},
    {0/255., 191/255., 0/255., 1.},
    {100/255., 245/255., 174/255., 1.},
    {0/255., 235/255., 231/255., 1.},
    {66/255., 206/255., 252/255., 1.},
    {124/255., 160/255., 252/255., 1.},
    {173/255., 146/255., 252/255., 1.},
    {255/255., 120/255., 252/255., 1.},
    {255/255., 117/255., 172/255., 1.},
    {255/255., 142/255., 107/255., 1.},
    {252/255., 174/255., 91/255., 1.},
    {252/255., 194/255., 0/255., 1.},
    {202/255., 245/255., 29/255., 1.},
    {0/255., 191/255., 0/255., 1.},
    {100/255., 245/255., 174/255., 1.},
    {0/255., 235/255., 231/255., 1.},
    {255/255., 255/255., 255/255., 1.},

    {255/255., 255/255., 255/255., 1.}, // to be safe
    {255/255., 255/255., 255/255., 1.}, // to be safe
    {255/255., 255/255., 255/255., 1.}, // to be safe
    {255/255., 255/255., 255/255., 1.}, // to be safe
    {255/255., 255/255., 255/255., 1.}  // to be safe
};

static const uint64_t K_LIST_TWO_EXP[32] = {
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

osg::Vec3d const K_ZERO_VEC = osg::Vec3d(0.0,0.0,0.0);

// ============================================================= //
// ============================================================= //

struct LLA
{
    LLA() {}

    LLA(double lon,double lat) :
        lon(lon),
        lat(lat),
        alt(0.0)
    {}

    LLA(double lon,double lat, double alt) :
        lon(lon),
        lat(lat),
        alt(alt)
    {}

    double lon;
    double lat;
    double alt;
};

struct Circle
{
    osg::Vec3d center;
    osg::Vec3d normal;
//    osg::Vec3d u;
//    osg::Vec3d v;
    double radius;
};

struct Plane
{
    osg::Vec3d n;   // plane normal; no guarantee n is normalized!
    osg::Vec3d p;   // point on the plane
    double d;       // d = n*p
};

struct Edge
{
    osg::Vec3d a;
    osg::Vec3d dirn_ab; // (A -> B) == (B-A)
};

struct Frustum
{
    std::vector<Plane> list_planes;
    std::vector<Edge> list_edges;
    std::vector<osg::Vec3d> list_vx;
    std::vector<osg::Vec3d const *> list_pyr_vx;
    osg::Vec3d eye;

    Frustum()
    {
        list_planes.resize(6);
        list_edges.resize(12);
        list_vx.resize(8);
        list_pyr_vx.resize(5);
    }
};

struct GeoBounds
{
    GeoBounds() :
        minLon(0),maxLon(0),
        minLat(0),maxLat(0)
    {}

    GeoBounds(double min_lon,
              double max_lon,
              double min_lat,
              double max_lat) :
        minLon(min_lon),
        maxLon(max_lon),
        minLat(min_lat),
        maxLat(max_lat)
    {

    }

    bool operator==(GeoBounds const &b) const
    {
        return ((minLon == b.minLon) &&
                (maxLon == b.maxLon) &&
                (minLat == b.minLat) &&
                (maxLat == b.maxLat));
    }

    double minLon;
    double maxLon;
    double minLat;
    double maxLat;
};

enum class Intersection : uint8_t
{
    FALSE = 0,
    TRUE,
    COINCIDENT,
    PARALLEL,
    CONTAINED
};

enum class GeometryResult : uint8_t
{
    XSEC_FALSE = 0,
    XSEC_TRUE,
    XSEC_COINCIDENT,
    XSEC_PARALLEL,
    XSEC_CONTAINED,
    CLIP_INSIDE,
    CLIP_OUTSIDE,
    CLIP_XSEC
};

enum AngleRange
{
    DEG_0_360,
    DEG_180_180
};


// ============================================================= //
// ============================================================= //

LLA ConvECEFToLLA(const osg::Vec3d &pointECEF);

osg::Vec3d ConvLLAToECEF(const LLA &pointLLA);

std::vector<LLA> ConvListECEFToLLA(std::vector<osg::Vec3d> const &list_ecef);
std::vector<osg::Vec3d> ConvListLLAToECEF(std::vector<LLA> const &list_lla);

std::pair<bool,osg::Vec2d> ConvWorldToNDC(osg::Matrixd const &mvp,
                                          osg::Vec3d const &world);

bool ConvWorldToNDC(osg::Matrixd const &mvp,
                    osg::Vec3d const &world,
                    osg::Vec2d &ndc);

osg::Vec2d ConvWorldToNDC(osg::Matrixd const &mvp,
                          osg::Vec3d const &world,
                          bool &ok);

double CalcValidAngleDegs(double angle, AngleRange range);

double CalcTriangleArea(osg::Vec2d const &vxA,
                        osg::Vec2d const &vxB,
                        osg::Vec2d const &vxC);

bool CalcPolyIsInFront(osg::Vec2d const &polyA_axis_dirn,
                       osg::Vec2d const &polyA_axis_pt,
                       std::vector<osg::Vec2d> const &polyB_list_vx);

osg::Vec2d CalcPerpendicular(osg::Vec2d v);

double CalcGeoBoundsArea(GeoBounds const &bounds);

bool CalcGeoBoundsIntersection(GeoBounds const &a,
                               GeoBounds const &b);

bool CalcGeoBoundsIntersection(GeoBounds const &a,
                               GeoBounds const &b,
                               GeoBounds &xsec);

bool CalcWithinGeoBounds(GeoBounds const &bounds,
                         LLA const &lla);

bool CalcTriangleAARectIntersection(std::vector<osg::Vec2d> const &tri,
                                    std::vector<osg::Vec2d> const &rect);

bool CalcQuadAARectIntersection(std::vector<osg::Vec2d> const &quad,
                                std::vector<osg::Vec2d> const &rect);

osg::Vec3d CalcPointPlaneProjection(osg::Vec3d const &point,
                                    Plane const &plane);

Intersection CalcLinePlaneIntersection(osg::Vec3d const &a,
                                       osg::Vec3d const &b,
                                       Plane const &plane,
                                       osg::Vec3d &xsec);

Plane CalcLonPlane(double lon,bool normal_left=true,bool normalize=false);

Plane CalcLatPlane(double lat,bool normal_up=true);


size_t CalcPlanePolyIntersection(Plane const &plane,
                                 std::vector<osg::Vec3d> const &list_poly_vx,
                                 std::vector<osg::Vec3d> &list_xsec);


GeometryResult CalcTrianglePlaneClip(std::vector<osg::Vec3d> const &tri,
                                     Plane const &plane,
                                     std::vector<osg::Vec3d> &inside,
                                     std::vector<osg::Vec3d> &outside);

bool CalcCameraNearFarDist(osg::Vec3d const &eye,
                           osg::Vec3d const &view_dirn,
                           double const pin_surf_dist_m,
                           double &dist_near,
                           double &dist_far);

void CalcQuadraticEquationReal(double a, double b, double c,
                               std::vector<double> &listRoots);

bool CalcRayEarthIntersection(osg::Vec3d const &rayPoint,
                              osg::Vec3d const &rayDirn,
                              osg::Vec3d &xsecNear,
                              osg::Vec3d &xsecFar);

Plane CalcHorizonPlane(osg::Vec3d const &eye,
                       double const clamp_dist_m=500.0);

osg::Vec3d CalcTriangleClosestPoint(osg::Vec3d const &a,
                                    osg::Vec3d const &b,
                                    osg::Vec3d const &c,
                                    osg::Vec3d const &p);

bool CalcTriangleIntersectsSphere(osg::Vec3d const &a,
                                  osg::Vec3d const &b,
                                  osg::Vec3d const &c,
                                  osg::Vec3d const &sphere_center,
                                  double sphere_radius);

bool CalcSphereOutsideFrustumExact(Frustum const &frustum,
                                   osg::Vec3d const &sphere_center,
                                   double const sphere_radius);

bool CalcSphereOutsidePlane(Plane const &plane,
                            osg::Vec3d const &center,
                            double const radius);

void CalcProjFrustumPoly(Frustum const &frustum,
                         Plane const &horizon_plane,
                         std::vector<osg::Vec3d> &list_ecef);

std::vector<std::pair<double,double>>
CalcLonRange(std::vector<LLA> const &list_lla);

bool CalcMinGeoBoundsFromLLAPoly(LLA const &camLLA,
                                 std::vector<LLA> const &listPLLA,
                                 std::vector<GeoBounds> &listBounds);

osg::Vec3d ClosestPointCirclePoint(Circle const &c,
                                   osg::Vec3d const &p);

void BuildEarthSurface(double min_lon,
                       double max_lon,
                       double min_lat,
                       double max_lat,
                       uint16_t lon_segments,
                       uint16_t lat_segments,
                       std::vector<osg::Vec3d> &list_vx,
                       std::vector<osg::Vec2d> &list_tx,
                       std::vector<uint16_t> &list_ix);

void BuildEarthSurface(double min_lon,
                       double max_lon,
                       double min_lat,
                       double max_lat,
                       uint16_t lon_segments,
                       uint16_t lat_segments,
                       std::vector<osg::Vec3d> &list_vx,
                       std::vector<uint16_t> &list_ix);

void BuildEarthSurface(double min_lon,
                       double max_lon,
                       double min_lat,
                       double max_lat,
                       uint16_t lon_segments,
                       uint16_t lat_segments,
                       std::vector<LLA> &list_lla,
                       std::vector<osg::Vec3d> &list_vx,
                       std::vector<uint16_t> &list_ix);

// ============================================================= //
// ============================================================= //


#endif // SCRATCH_GEOMETRY_UTILS_H
