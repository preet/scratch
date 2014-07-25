
//   Copyright (C) 2014 Preet Desai (preet.desai@gmail.com)

//   Licensed under the Apache License, Version 2.0 (the "License");
//   you may not use this file except in compliance with the License.
//   You may obtain a copy of the License at

//       http://www.apache.org/licenses/LICENSE-2.0

//   Unless required by applicable law or agreed to in writing, software
//   distributed under the License is distributed on an "AS IS" BASIS,
//   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//   See the License for the specific language governing permissions and
//   limitations under the License.

// Functions from MathGeoLib copyright Jukka Jylänki,
// available under the Apache 2.0 license:
// https://github.com/juj/MathGeoLib

#ifndef GM_UTIL_H
#define GM_UTIL_H

#include <memory>
#include <vector>
#include <iostream>
#include <cassert>

#include <osg/Vec3d>

//////////////////////////////////////////////////////

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

//////////////////////////////////////////////////////

size_t const K_MAX_LOD = 18;
double const K_MAX_LOD_AREA_REF_M = 150*150;
double FOVY_DEGS = 35.0;

double const K_MAX_POS_DBL = std::numeric_limits<double>::max();
double const K_MIN_NEG_DBL = -K_MAX_POS_DBL;

//////////////////////////////////////////////////////

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
    {255/255., 255/255., 255/255., 1.}
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

struct OBB
{
    // obb
    osg::Vec3d center;
    osg::Vec3d ori[3]; // local orientation axes
    osg::Vec3d ext;    // positive half-extents

    Plane faces[3];
};

// From Real-Time Collision Detection, Ericson
double CalcMinDist2PointOBB(osg::Vec3d const &p,
                            OBB const &obb)
{
    osg::Vec3d v = p - obb.center;
    double dist2 = 0;

    for(uint8_t i=0; i < 3; i++) {
        double d = v * obb.ori[i];
        double excess = 0;

        if(d < obb.ext[i]*-1.0) {
            excess = d + obb.ext[i];
        }
        else if(d > obb.ext[i]) {
            excess = d - obb.ext[i];
        }

        dist2 += (excess*excess);
    }
    return dist2;
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

//    for(auto dist : list_lod_dist) {
//        std::cout << "##: " << dist << std::endl;
//    }

    return list_lod_dist;
}

std::vector<double> K_LIST_LOD_DIST = CalcLodDistances();

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

osg::Vec3d CalcPointPlaneProjection(osg::Vec3d const &point,
                                    Plane const &plane)
{
    double t = ((plane.n*point)-plane.d)/(plane.n*plane.n);
    return (point-(plane.n*t));
}

osg::Vec3d CalcPointRayProjection(osg::Vec3d const &point,
                                  osg::Vec3d const &ray_point,
                                  osg::Vec3d const &ray_dirn)
{
    double t = ((point-ray_point)*ray_dirn)/(ray_dirn*ray_dirn);
    return (ray_point+(ray_dirn*t));
}

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

bool CalcHorizonPlane(osg::Vec3d const &eye,
                      Plane & horizon_plane,
                      bool flip_normal)
{
    // We need to clamp eye_length such that the
    // eye is outside of the celestial body surface

    osg::Vec3d xsecNear,xsecFar;
    if(!CalcRayEarthIntersection(eye,eye,xsecNear,xsecFar)) {
        return false;
    }

    horizon_plane.n = eye;
    horizon_plane.n.normalize();

    double const eye_length2 = eye.length2();
    double const xsec_length2 = xsecNear.length2();

    osg::Vec3d eye_clamped = eye;
    if((eye.length() - RAD_AV) < 5000.0) {
        eye_clamped = (horizon_plane.n*(5000.0+RAD_AV));
    }

    double eye_length = eye_clamped.length();
    double const inv_dist = 1.0/eye_length;

    // by similar triangles
    horizon_plane.p = horizon_plane.n * (RAD_AV*RAD_AV*inv_dist);

    std::cout << "###: " << (horizon_plane.p.length()) << std::endl;

    if(flip_normal) {
        horizon_plane.n *= -1.0;
    }
    horizon_plane.d = horizon_plane.n * horizon_plane.p;

    return true;
}

bool CalcRayPlaneIntersection(osg::Vec3d const &ray_pt,
                              osg::Vec3d const &ray_dirn,
                              Plane const &plane,
                              osg::Vec3d &xsec,
                              double &u)
{
    // ref: http://paulbourke.net/geometry/pointlineplane/
    double const u_den = plane.n * (ray_dirn);
    if(fabs(u_den) < 1E-4) {
        return false;
    }
    double const u_num = plane.n * (plane.p - ray_pt);
    u = u_num/u_den;
    xsec = ray_pt+(ray_dirn*u);

    return true;
}


osg::Vec3d CalcVectorRotation(osg::Vec3d const &input_vec,
                              osg::Vec3d const &axis_normalized,
                              double const angle_rads)
{
    double cos_angle_rads = cos(angle_rads);
    double sin_angle_rads = sin(angle_rads);


    osg::Vec3d rotated_vec =
            (input_vec * cos_angle_rads) + ((axis_normalized^input_vec) * sin_angle_rads) + (axis_normalized*(axis_normalized*input_vec)*(1-cos_angle_rads));

    return rotated_vec;
}

//////////////////////////////////////////////////////

// From MathGeoLib
void CalcOBBProjectionInterval(OBB const &obb,
                           osg::Vec3d const &axis,
                           double &min,
                           double &max)
{
    double x = fabs((axis * obb.ori[0]) * obb.ext.x());
    double y = fabs((axis * obb.ori[1]) * obb.ext.y());
    double z = fabs((axis * obb.ori[2]) * obb.ext.z());
    double pt = axis*obb.center;
    min = pt-x-y-z;
    max = pt+x+y+z;
}

void CalcPolyProjectionInterval(std::vector<osg::Vec3d> const &list_vx,
                                osg::Vec3d const &axis,
                                double &min,
                                double &max)
{
    // Project each vertex in @list_vx along @axis
    // and return the min and max of the projection
    // interval

    min = K_MAX_POS_DBL;
    max = K_MIN_NEG_DBL;

    for(auto const &vx : list_vx) {
        double proj = vx*axis;
        min = std::min(min,proj);
        max = std::max(max,proj);
    }
}

// From Eberly - The Method of Separating Axes

// WARN:
// When CalcWhichSide is used, *ALL* planes of the
// polyhedron must be tested (even if they're parallel)
int CalcWhichSide(osg::Vec3d const &polyA_axis_dirn,
                  osg::Vec3d const &polyA_axis_pt,
                  std::vector<osg::Vec3d> const &polyB_list_vx)
{
    int positive=0;
    int negative=0;

    for(auto const & vx : polyB_list_vx)
    {
        double t = polyA_axis_dirn * (vx - polyA_axis_pt);
        if(t > 0) {
            positive++;
        }
        else {
            negative++;
        }

        if(positive && negative) {
            // would it be cheaper to avoid all these ifs?
            return 0;
        }
    }

    return (positive ? 1 : -1);
}

bool CalcSphereOutsidePlane(Plane const &plane,
                            osg::Vec3d const &center,
                            double const radius)
{
    // min distance between sphere and plane
    double const dist = (plane.n * center) - plane.d;
    return (dist > radius);
}

bool CalcOBBOutsidePlane(Plane const &plane,
                         OBB const &obb)
{
    // min distance between bbox center and plane
    double const dist = (plane.n * obb.center) - plane.d;

    // radius
    double const r =
            obb.ext.x() * fabs(plane.n * obb.ori[0]) +
            obb.ext.y() * fabs(plane.n * obb.ori[1]) +
            obb.ext.z() * fabs(plane.n * obb.ori[2]);

    return (dist > r);
}

bool CalcFrustumOBBIntersect(Frustum const &frustum,
                             OBB const &obb)
{
//    if(CalcOBBOutsidePlane(frustum.list_planes[2],obb)) {
//        return false;
//    }

    for(auto const & plane : frustum.list_planes)
    {
        if(CalcOBBOutsidePlane(plane,obb)) {
            // bbox is outside this plane
            return false;
        }
    }
    return true;
}

bool CalcFrustumOBBIntersectSAT(Frustum const &frustum,
                                OBB const &obb)
{
    // Test the faces of the frustum
    for(auto const & plane : frustum.list_planes)
    {
        if(CalcOBBOutsidePlane(plane,obb)) {
            // bbox is outside this plane
            return false;
        }
    }

//    // Test the faces of the OBB
//    for(int i=0; i < 3; i++)
//    {
//        double obb_min,obb_max;
//        CalcOBBProjectionInterval(obb,
//                                  obb.faces[i].n,
//                                  obb_min,
//                                  obb_max);

//        double frustum_min,frustum_max;
//        CalcPolyProjectionInterval(frustum.list_vx,
//                                   obb.faces[i].n,
//                                   frustum_min,
//                                   frustum_max);

//        if((obb_min > frustum_max) || (frustum_min > obb_max)) {
//            // there's an axis with no overlap so the polys dont intersect
//            return false;
//        }
//    }

//    // Test edge cross products
//    std::vector<osg::Vec3d> list_obb_edges;
//    for(size_t i=0; i < 3; i++) {
//        list_obb_edges.push_back(obb.faces[i].p-obb.center);
//    }

//    std::vector<osg::Vec3d> list_frustum_edges;
//    for(size_t i=4; i < 8; i++) {
//        list_frustum_edges.push_back(frustum.list_vx[i] - frustum.eye);
//    }

//    for(size_t i=0; i < list_obb_edges.size(); i++) {
//        for(size_t j=0; j < list_frustum_edges.size(); j++)
//        {
//            osg::Vec3d axis = list_obb_edges[i]^list_frustum_edges[j];

//            double obb_min,obb_max;
//            CalcOBBProjectionInterval(obb,
//                                      axis,
//                                      obb_min,
//                                      obb_max);

//            double frustum_min,frustum_max;
//            CalcPolyProjectionInterval(frustum.list_vx,
//                                       axis,
//                                       frustum_min,
//                                       frustum_max);

//            if((obb_min > frustum_max) || (frustum_min > obb_max)) {
//                // there's an axis with no overlap so the polys dont intersect
//                return false;
//            }
//        }
//    }

    return true;
}

//////////////////////////////////////////////////////

// From MathGeoLib
osg::Vec3d CalcTriangleClosestPoint(osg::Vec3d const &a,
                                    osg::Vec3d const &b,
                                    osg::Vec3d const &c,
                                    osg::Vec3d const &p)
{
    /** The code for Triangle-float3 test is from
     *  Christer Ericson's Real-Time Collision Detection, pp. 141-142. */

    // Check if P is in vertex region outside A.
    osg::Vec3d ab = b - a;
    osg::Vec3d ac = c - a;
    osg::Vec3d ap = p - a;
    float d1 = (ab*ap);
    float d2 = (ac*ap);
    if (d1 <= 0.f && d2 <= 0.f)
    return a; // Barycentric coordinates are (1,0,0).

    // Check if P is in vertex region outside B.
    osg::Vec3d bp = p - b;
    float d3 = (ab*bp);
    float d4 = (ac*bp);
    if (d3 >= 0.f && d4 <= d3)
    return b; // Barycentric coordinates are (0,1,0).

    // Check if P is in edge region of AB, and if so, return the projection of P onto AB.
    float vc = d1*d4 - d3*d2;
    if (vc <= 0.f && d1 >= 0.f && d3 <= 0.f)
    {
    float v = d1 / (d1 - d3);
    return a + (ab*v); // The barycentric coordinates are (1-v, v, 0).
    }

    // Check if P is in vertex region outside C.
    osg::Vec3d cp = p - c;
    float d5 = (ab*cp);
    float d6 = (ac*cp);
    if (d6 >= 0.f && d5 <= d6)
    return c; // The barycentric coordinates are (0,0,1).

    // Check if P is in edge region of AC, and if so, return the projection of P onto AC.
    float vb = d5*d2 - d1*d6;
    if (vb <= 0.f && d2 >= 0.f && d6 <= 0.f)
    {
    float w = d2 / (d2 - d6);
    return a + (ac*w); // The barycentric coordinates are (1-w, 0, w).
    }

    // Check if P is in edge region of BC, and if so, return the projection of P onto BC.
    float va = d3*d6 - d5*d4;
    if (va <= 0.f && d4 - d3 >= 0.f && d5 - d6 >= 0.f)
    {
    float w = (d4 - d3) / (d4 - d3 + d5 - d6);
    return b + ((c - b)*w); // The barycentric coordinates are (0, 1-w, w).
    }

    // P must be inside the face region. Compute the closest point through its barycentric coordinates (u,v,w).
    float denom = 1.f / (va + vb + vc);
    float v = vb * denom;
    float w = vc * denom;
    return a + (ab * v) + (ac * w);
}

bool CalcTriangleIntersectsSphere(osg::Vec3d const &a,
                                  osg::Vec3d const &b,
                                  osg::Vec3d const &c,
                                  osg::Vec3d const &sphere_center,
                                  double sphere_radius)
{
    osg::Vec3d pt = CalcTriangleClosestPoint(a,b,c,sphere_center);

    if((pt-sphere_center).length2() <= (sphere_radius*sphere_radius)) {
        return true;
    }
    return false;
}

bool CalcSphereOutsideFrustumExact(Frustum const &frustum,
                                   osg::Vec3d const &sphere_center,
                                   double const sphere_radius)
{
    // For an exact test, first check if the sphere is
    // completely contained in the frustum side planes

    bool contained=true;

    // First four planes are the side planes
    for(size_t i=0; i < 4; i++) {
        Plane const &plane = frustum.list_planes[i];
        Plane plane_neg_n;
        plane_neg_n.n = plane.n*-1.0;
        plane_neg_n.p = plane.p;
        plane_neg_n.d = plane_neg_n.n*plane_neg_n.p;
        if(!CalcSphereOutsidePlane(plane_neg_n,
                                   sphere_center,
                                   sphere_radius))
        {
            contained = false;
            break;
        }
    }

    if(contained) {
        return false;
    }

    // If the sphere isn't completely contained,
    // treat the frustum as a mesh and check if any
    // of its triangles intersect with the sphere

    // Expect list_vx:
    // list_vx[0] = NBL
    // list_vx[1] = NBR
    // list_vx[2] = NTR
    // list_vx[3] = NTL
    // list_vx[4] = FBL
    // list_vx[5] = FBR
    // list_vx[6] = FTR
    // list_vx[7] = FTL

    // 6 faces, 12 triangles
    std::vector<osg::Vec3d const *> list_tris(12*3);

    list_tris[0] = &(frustum.list_vx[0]);
    list_tris[1] = &(frustum.list_vx[1]);
    list_tris[2] = &(frustum.list_vx[2]);

    list_tris[3] = &(frustum.list_vx[0]);
    list_tris[4] = &(frustum.list_vx[2]);
    list_tris[5] = &(frustum.list_vx[3]);

    list_tris[6] = &(frustum.list_vx[1]);
    list_tris[7] = &(frustum.list_vx[5]);
    list_tris[8] = &(frustum.list_vx[6]);

    list_tris[9] = &(frustum.list_vx[1]);
    list_tris[10] = &(frustum.list_vx[6]);
    list_tris[11] = &(frustum.list_vx[2]);

    list_tris[12] = &(frustum.list_vx[3]);
    list_tris[13] = &(frustum.list_vx[2]);
    list_tris[14] = &(frustum.list_vx[6]);

    list_tris[15] = &(frustum.list_vx[3]);
    list_tris[16] = &(frustum.list_vx[6]);
    list_tris[17] = &(frustum.list_vx[7]);

    list_tris[18] = &(frustum.list_vx[0]);
    list_tris[19] = &(frustum.list_vx[4]);
    list_tris[20] = &(frustum.list_vx[7]);

    list_tris[21] = &(frustum.list_vx[0]);
    list_tris[22] = &(frustum.list_vx[7]);
    list_tris[23] = &(frustum.list_vx[3]);

    list_tris[24] = &(frustum.list_vx[0]);
    list_tris[25] = &(frustum.list_vx[1]);
    list_tris[26] = &(frustum.list_vx[5]);

    list_tris[27] = &(frustum.list_vx[0]);
    list_tris[28] = &(frustum.list_vx[5]);
    list_tris[29] = &(frustum.list_vx[4]);

    list_tris[30] = &(frustum.list_vx[4]);
    list_tris[31] = &(frustum.list_vx[5]);
    list_tris[32] = &(frustum.list_vx[6]);

    list_tris[33] = &(frustum.list_vx[4]);
    list_tris[34] = &(frustum.list_vx[6]);
    list_tris[35] = &(frustum.list_vx[7]);

    // Check each triangle
    for(size_t i=0; i < list_tris.size(); i+=3) {
        if(CalcTriangleIntersectsSphere(*(list_tris[i+0]),
                                        *(list_tris[i+1]),
                                        *(list_tris[i+2]),
                                        sphere_center,
                                        sphere_radius))
        {
            return false;
        }
    }

    return true;
}

//////////////////////////////////////////////////////


#endif // GM_UTIL_H
