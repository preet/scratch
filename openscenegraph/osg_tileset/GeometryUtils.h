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

#include <osg/Vec3d>
#include <osg/Matrixd>

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

osg::Vec3d CalcPointPlaneProjection(osg::Vec3d const &point,
                                    Plane const &plane);

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

void BuildEarthSurface(double minLon, double minLat,
                       double maxLon, double maxLat,
                       uint16_t lonSegments,
                       uint16_t latSegments,
                       std::vector<osg::Vec3d> &vertexArray,
                       std::vector<osg::Vec2d> &texCoords,
                       std::vector<uint16_t> &triIdx);

void BuildEarthSurface(double minLon, double minLat,
                       double maxLon, double maxLat,
                       uint16_t lonSegments,
                       uint16_t latSegments,
                       std::vector<osg::Vec3d> &vertexArray,
                       std::vector<uint16_t> &triIdx);

// ============================================================= //
// ============================================================= //


#endif // SCRATCH_GEOMETRY_UTILS_H
