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
    {252/255., 194/255., 0/255., 1.},
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

// ============================================================= //
// ============================================================= //

struct LLA
{
    double lon;
    double lat;
    double alt;
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

// ============================================================= //
// ============================================================= //

LLA ConvECEFToLLA(const osg::Vec3d &pointECEF);

osg::Vec3d ConvLLAToECEF(const LLA &pointLLA);

std::pair<bool,osg::Vec2d> ConvWorldToNDC(osg::Matrixd const &mvp,
                                          osg::Vec3d const &world);

double CalcTriangleArea(osg::Vec2d const &vxA,
                        osg::Vec2d const &vxB,
                        osg::Vec2d const &vxC);

bool CalcPolyIsInFront(osg::Vec2d const &polyA_axis_dirn,
                       osg::Vec2d const &polyA_axis_pt,
                       std::vector<osg::Vec2d> const &polyB_list_vx);

osg::Vec2d CalcPerpendicular(osg::Vec2d v);

bool CalcTriangleAARectIntersection(std::vector<osg::Vec2d> const &tri,
                                    std::vector<osg::Vec2d> const &rect);

bool CalcQuadAARectIntersection(std::vector<osg::Vec2d> const &quad,
                                std::vector<osg::Vec2d> const &rect);

osg::Vec3d CalcPointPlaneProjection(osg::Vec3d const &point,
                                    Plane const &plane);

// CalcLinePlaneIntersection
// * computes the intersection point between the
//   line segment specified by @a and @b and @plane
// * returns false if no xsec point exists or if
//   the xsec point lies outside the segment
Intersection CalcLinePlaneIntersection(osg::Vec3d const &a,
                                       osg::Vec3d const &b,
                                       Plane const &plane,
                                       osg::Vec3d &xsec);

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

// ============================================================= //
// ============================================================= //


#endif // SCRATCH_GEOMETRY_UTILS_H
