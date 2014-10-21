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

#include <GeometryUtils.h>
#include <cassert>

// ============================================================= //
// ============================================================= //

LLA ConvECEFToLLA(const osg::Vec3d &pointECEF)
{
    double radius = pointECEF.length();

    LLA pointLLA;
    pointLLA.lon = (atan2(pointECEF.y(),pointECEF.x()))*K_RAD2DEG;
    pointLLA.lat = (asin(pointECEF.z()/radius))*K_RAD2DEG;
    pointLLA.alt = radius-RAD_AV;

    return pointLLA;
}

osg::Vec3d ConvLLAToECEF(const LLA &pointLLA)
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

std::vector<LLA> ConvListECEFToLLA(std::vector<osg::Vec3d> const &list_ecef)
{
    std::vector<LLA> list_lla;
    list_lla.reserve(list_ecef.size());

    for(auto const &ecef : list_ecef) {
        list_lla.push_back(ConvECEFToLLA(ecef));
    }

    return list_lla;
}

std::vector<osg::Vec3d> ConvListLLAToECEF(std::vector<LLA> const &list_lla)
{
    std::vector<osg::Vec3d> list_ecef;
    list_ecef.reserve(list_lla.size());

    for(auto const &lla : list_lla) {
        list_ecef.push_back(ConvLLAToECEF(lla));
    }

    return list_ecef;
}

std::pair<bool,osg::Vec2d> ConvWorldToNDC(osg::Matrixd const &mvp,
                                          osg::Vec3d const &world)
{
    osg::Vec4d world4(world.x(),world.y(),world.z(),1.0);
    osg::Vec4d clip4 = world4 * mvp;
    if(clip4.w() < 1E-5) {
        std::cout << "bbaaaadddd" << std::endl;
    }

    if(fabs(clip4.w()) < 1E-5) {
        std::cout << "bad" << std::endl;
        return std::pair<bool,osg::Vec2d>(false,osg::Vec2d(0,0));
    }

    osg::Vec4d ndc4 = clip4/clip4.w();
    return std::pair<bool,osg::Vec2d>(true,osg::Vec2d(ndc4.x(),ndc4.y()));
}

osg::Vec2d ConvWorldToNDC(osg::Matrixd const &mvp,
                          osg::Vec3d const &world,
                          bool &ok)
{
    osg::Vec2d ndc2;
    osg::Vec4d world4(world.x(),world.y(),world.z(),1.0);
    osg::Vec4d clip4 = world4 * mvp;

    if(fabs(clip4.w()) < 1E-5) {
        ok = false;
        return ndc2;
    }

    osg::Vec4d ndc4 = clip4/clip4.w();

    ok = true;

    ndc2.x() = ndc4.x();
    ndc2.y() = ndc4.y();
    return ndc2;
}

double CalcValidAngleDegs(double angle, AngleRange range)
{
    if(range == DEG_0_360)   {
        angle = fmod(angle,360);
        if(angle < 0)
        {   angle += 360;   }

        return angle;
    }
    else   { // DEG_180_180
        angle = fmod(angle+180,360);
        if(angle < 0)
        {   angle += 360;   }

        return angle-180;
    }
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

bool CalcPolyIsInFront(osg::Vec2d const &polyA_axis_dirn,
                       osg::Vec2d const &polyA_axis_pt,
                       std::vector<osg::Vec2d> const &polyB_list_vx)
{
    for(auto const &vx : polyB_list_vx) {
        double t = (polyA_axis_dirn*(vx-polyA_axis_pt));
        if(!(t > 0)) {
            return false;
        }
    }
    return true;
}

// flip 90 degs right (cw)
osg::Vec2d CalcPerpendicularRight(osg::Vec2d v)
{
    std::swap(v.x(),v.y());
    v.y() *= -1.0;
    return v;
}

// flip 90 degs left (ccw)
osg::Vec2d CalcPerpendicularLeft(osg::Vec2d v)
{
    std::swap(v.x(),v.y());
    v.x() *= -1.0;
    return v;
}

bool CalcTriangleAARectIntersection(std::vector<osg::Vec2d> const &tri,
                                    std::vector<osg::Vec2d> const &rect)
{
    osg::Vec2d const &min = rect[0];
    osg::Vec2d const &max = rect[2];

    // First check if any vertices from the
    // triangle lie in the rectangle bounds
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

    if(CalcPolyIsInFront(CalcPerpendicularRight(rect[1]-rect[0]),rect[0],tri)) {
        return false;
    }
    if(CalcPolyIsInFront(CalcPerpendicularRight(rect[2]-rect[1]),rect[1],tri)) {
        return false;
    }
    if(CalcPolyIsInFront(CalcPerpendicularRight(rect[3]-rect[2]),rect[2],tri)) {
        return false;
    }
    if(CalcPolyIsInFront(CalcPerpendicularRight(rect[0]-rect[3]),rect[3],tri)) {
        return false;
    }

    // (test rectangle against triangle edge normals)
    if(CalcPolyIsInFront(CalcPerpendicularRight(tri[1]-tri[0]),tri[0],rect)) {
        return false;
    }
    if(CalcPolyIsInFront(CalcPerpendicularRight(tri[2]-tri[1]),tri[1],rect)) {
        return false;
    }
    if(CalcPolyIsInFront(CalcPerpendicularRight(tri[0]-tri[2]),tri[2],rect)) {
        return false;
    }

    return true;
}

bool CalcGeoBoundsIntersection(GeoBounds const &a,
                               GeoBounds const &b,
                               GeoBounds &xsec)
{
    bool outside =
            (a.minLon > b.maxLon) ||
            (a.maxLon < b.minLon) ||
            (a.minLat > b.maxLat) ||
            (a.maxLat < b.minLat);

    if(outside) {
        return false;
    }

    xsec.minLon = std::max(a.minLon,b.minLon);
    xsec.maxLon = std::min(a.maxLon,b.maxLon);
    xsec.minLat = std::max(a.minLat,b.minLat);
    xsec.maxLat = std::min(a.maxLat,b.maxLat);
    return true;
}

bool CalcWithinGeoBounds(GeoBounds const &bounds,
                         LLA const &lla)
{
    static const double k_eps = 1E-8;
    static const double k_neps = -1E-8;

    // Account for the +/- 180 discontinuity at the antemeridian
    if((180.0 - fabs(lla.lon)) < k_eps) {
        if((bounds.maxLon == 180.0) || (bounds.minLon == -180.0))  {
            bool outside =
                    (lla.lat-bounds.maxLat > k_eps) ||
                    (lla.lat-bounds.minLat < k_neps);
            return (!outside);
        }
    }

    bool outside =
            (lla.lon-bounds.maxLon > k_eps) ||
            (lla.lon-bounds.minLon < k_neps) ||
            (lla.lat-bounds.maxLat > k_eps) ||
            (lla.lat-bounds.minLat < k_neps);

//    if(outside) {

//        std::cout << ": " << (lla.lon > bounds.maxLon)
//                  << ", " << (lla.lon < bounds.minLon)
//                  << ", " << (lla.lat > bounds.maxLat)
//                  << ", " << (lla.lat < bounds.minLat)
//                  << std::endl;

//        if(lla.lat > bounds.maxLat) {
//            std::cout << "#: " << lla.lat << "," << bounds.maxLat << std::endl;
//        }

////        std::cout << "lla: lon: " << lla.lon << ", lat: " << lla.lat
////                  << ", b: min_lon: " << bounds.minLon << ", max_lon : " << bounds.maxLon
////                  << ", min_lat: " << bounds.minLat << ", max_lat: " << bounds.maxLat
////                  << std::endl;
//    }

    return (!outside);
}

double CalcGeoBoundsArea(GeoBounds const &b)
{
//    std::cout << ": min_lon: " << b.minLon
//              << ", max_lon: " << b.maxLon
//              << ", min_lat: " << b.minLat
//              << ", max_lat: " << b.maxLat
//              << std::endl;

    double const min_lon_rads = b.minLon*K_DEG2RAD;
    double const max_lon_rads = b.maxLon*K_DEG2RAD;
    double const min_lat_rads = b.minLat*K_DEG2RAD;
    double const max_lat_rads = b.maxLat*K_DEG2RAD;

    return ((sin(max_lat_rads) - sin(min_lat_rads)) *
            (max_lon_rads - min_lon_rads) * RAD_AV * RAD_AV);
}

bool CalcQuadAARectIntersection(std::vector<osg::Vec2d> const &quad,
                                std::vector<osg::Vec2d> const &rect)
{
    osg::Vec2d const &min = rect[0];
    osg::Vec2d const &max = rect[2];

    // First check if any vertices from the
    // triangle lie in the rectangle bounds
    for(size_t i=0; i < 4; i++) {
        bool outside =
                (quad[i].x() < min.x()) ||
                (quad[i].x() > max.x()) ||
                (quad[i].y() < min.y()) ||
                (quad[i].y() > max.y());

        if(!outside) {
            return true;
        }
    }

    // Next do SAT
    // (test quad against rectangle edge normals)
    if(CalcPolyIsInFront(CalcPerpendicularRight(rect[1]-rect[0]),rect[0],quad)) {
        return false;
    }
    if(CalcPolyIsInFront(CalcPerpendicularRight(rect[2]-rect[1]),rect[1],quad)) {
        return false;
    }
    if(CalcPolyIsInFront(CalcPerpendicularRight(rect[3]-rect[2]),rect[2],quad)) {
        return false;
    }
    if(CalcPolyIsInFront(CalcPerpendicularRight(rect[0]-rect[3]),rect[3],quad)) {
        return false;
    }

    // (test rectangle against quad edge normals)
    if(CalcPolyIsInFront(CalcPerpendicularRight(quad[1]-quad[0]),quad[0],rect)) {
        return false;
    }
    if(CalcPolyIsInFront(CalcPerpendicularRight(quad[2]-quad[1]),quad[1],rect)) {
        return false;
    }
    if(CalcPolyIsInFront(CalcPerpendicularRight(quad[3]-quad[2]),quad[2],rect)) {
        return false;
    }
    if(CalcPolyIsInFront(CalcPerpendicularRight(quad[0]-quad[3]),quad[3],rect)) {
        return false;
    }

    return true;
}

osg::Vec3d CalcPointPlaneProjection(osg::Vec3d const &point,
                                    Plane const &plane)
{
    double t = ((plane.n*point)-plane.d)/(plane.n*plane.n);
    return (point-(plane.n*t));
}

Intersection CalcLinePlaneIntersection(osg::Vec3d const &a,
                                       osg::Vec3d const &b,
                                       Plane const &plane,
                                       osg::Vec3d &xsec)
{
    // real-time collision detection

    osg::Vec3d const ab = b-a;
    double const u_den = plane.n*ab;

    if(fabs(u_den) < 1E-5) {
        // Line segment AB is parallel to the plane

        // Check if its coincident
        if(fabs(plane.n*a - plane.d) < 1E-5) {
            xsec = a;
            return Intersection::COINCIDENT;
        }

        return Intersection::PARALLEL;
    }

    double const u = (plane.d - (plane.n*a))/u_den;
    if(u >= 0.0 && u <= 1.0) {
        xsec = a + ab*u;
        return Intersection::TRUE;
    }

//    std::cout << "// u: " << u << std::endl;
//    std::cout << "// a: " << a << std::endl;
//    std::cout << "// b: " << b << std::endl;
//    std::cout << "// proj_a: " << (a-plane.p)*plane.n << std::endl;
//    std::cout << "// proj_b: " << (b-plane.p)*plane.n << std::endl;
//    std::cout << "// plane.n: " << plane.n << std::endl;
//    std::cout << "// plane.p: " << plane.p << std::endl;
//    std::cout << "// plane.d: " << plane.d << std::endl;
    return Intersection::FALSE;
}

//IntersectionType CalcLinePlaneIntersection(osg::Vec3d const &a,
//                                           osg::Vec3d const &b,
//                                           Plane const &plane,
//                                           osg::Vec3d &xsec_pt,
//                                           double &u)
//{
//    osg::Vec3d ab = b-a;
//    double const denom = plane.n * ab;

//    if(fabs(denom) < 1E-4) {
//        // The line is parallel to the plane
//        if(fabs(plane.n * (plane.p-a)) < 1E-4) {
//            // The line is coincident with the plane
//            return XSEC_COINCIDENT;
//        }
//        return XSEC_FALSE;
//    }

//    u = (plane.n*(plane.p-a))/denom;
//    xsec_pt = a + ab*u;
//    return XSEC_TRUE;
//}

//IntersectionType CalcLinePlaneIntersection(Edge const &line,
//                                           Plane const &plane,
//                                           double &u)
//{
//    double const denom = plane.n * line.dirn_ab;
//    if(fabs(denom) < 1E-4) {
//        // The line is parallel to the plane
//        if(fabs(plane.n * (plane.p-line.a)) < 1E-4) {
//            // The line is coincident with the plane
//            return XSEC_COINCIDENT;
//        }
//        return XSEC_FALSE;
//    }

//    u = (plane.n*(plane.p-line.a))/denom;
//    return XSEC_TRUE;
//}


GeometryResult CalcTrianglePlaneClip(std::vector<osg::Vec3d> const &tri,
                                     Plane const &plane,
                                     std::vector<osg::Vec3d> &inside,
                                     std::vector<osg::Vec3d> &outside)
{
    inside.clear();
    outside.clear();

    // tri wrap around indexes
    static const uint8_t tri_ix[4] {
        0,1,2,0
    };

    // tolerance
    static const double peps = 1E-5;
    static const double neps = peps*-1.0;

    double u[3];
    uint8_t out_count=0;
    uint8_t in_count=0;
    uint8_t zero_count=0; // vx on the plane
    for(uint8_t i=0; i < 3; i++) {
        u[i] = (tri[i]-plane.p)*plane.n;
        if(u[i] > peps) {
            out_count++;
        }
        else if(u[i] < neps) {
            in_count++;
        }
        else {
            zero_count++;
        }
    }

    // If the plane is touching any of the triangle's
    // vertices, the triangle must be fully inside or
    // outside the plane
    if(zero_count > 0) {
        if(out_count > 0) {
            return GeometryResult::CLIP_OUTSIDE;
        }
        else {
            return GeometryResult::CLIP_INSIDE;
        }
    }

    if(out_count == 0) {
        return GeometryResult::CLIP_INSIDE;
    }
    if(out_count == 3) {
        return GeometryResult::CLIP_OUTSIDE;
    }

    for(uint8_t n=1; n < 4; n++) {
        uint8_t const i_this = tri_ix[n];
        uint8_t const i_prev = tri_ix[n-1];

//        std::cout << "#: edge: " << int(n)
//                  << ", i_prev: " << int(i_prev)
//                  << ", i_this: " << int(i_this)
//                  << ", u[i_prev]: " << u[i_prev]
//                  << ", u[i_this]: " << u[i_this]
//                  << ", u[i_prev]*u[i_this]: " << (u[i_prev]*u[i_this])
//                  << std::endl;

        // Test the edges of the triangle to see if they cross
        // the plane. If the product of u for adjacent vertices
        // is <= zero it means those vertices lie on different
        // sides of the plane.
        if(u[i_prev] * u[i_this] < 0) {
            // get the intersection point between the edge and plane
            osg::Vec3d xsec;
            Intersection result = CalcLinePlaneIntersection(
                        tri[i_prev],tri[i_this],plane,xsec);
//            if(result != Intersection::TRUE) {
//                std::string xsec_type;
//                if(result == Intersection::COINCIDENT) {
//                    xsec_type = "COINCIDENT";
//                }
//                else if(result == Intersection::CONTAINED) {
//                    xsec_type = "CONTAINED";
//                }
//                else if(result == Intersection::FALSE) {
//                    xsec_type = "FALSE";
//                }
//                else if(result == Intersection::PARALLEL) {
//                    xsec_type = "PARALLEL";
//                }

//                std::cout << "#: edge: " << int(n)
//                          << ", i_prev: " << int(i_prev)
//                          << ", i_this: " << int(i_this)
//                          << ", u[i_prev]: " << u[i_prev]
//                          << ", u[i_this]: " << u[i_this]
//                          << ", u[i_prev]*u[i_this]: " << (u[i_prev]*u[i_this])
//                          << ", xsec_type: " << xsec_type
//                          << std::endl;
//            }
            assert(result == Intersection::TRUE);

            // build the triangle and quad that results from
            // clipping a triangle against the plane
            if(u[i_prev] > 0) { // outside
                outside.push_back(tri[i_prev]);
                outside.push_back(xsec);

                inside.push_back(xsec);
            }
            else {
                inside.push_back(tri[i_prev]);
                inside.push_back(xsec);

                outside.push_back(xsec);
            }
        }
        else {
            // edge vx are on the same side

            // build the triangle and quad that results from
            // clipping a triangle against the plane
            if(u[i_prev] > 0) { // outside
                outside.push_back(tri[i_prev]);
            }
            else {
                inside.push_back(tri[i_prev]);
            }
        }
    }

    return GeometryResult::CLIP_XSEC;
}

bool CalcCameraNearFarDist(osg::Vec3d const &eye,
                           osg::Vec3d const &view_dirn,
                           double const pin_surf_dist_m,
                           double &dist_near,
                           double &dist_far)
{
    double const radius = RAD_AV;
    double const eye_dist2 = eye.length2();

    if(eye_dist2 > (radius*radius)) {
        // The near distance is set to the length of the
        // the line segment of the eye projected onto
        // the plane with n==view_dirn and p==(0,0,0)
        // less the radius
        Plane plane;
        plane.n = (view_dirn*-1.0);
        plane.n.normalize();
        plane.p = osg::Vec3d(0,0,0);
        plane.d = (plane.n*plane.p);

        osg::Vec3d xsec = CalcPointPlaneProjection(eye,plane);
        dist_near = (eye-xsec).length() - (radius+pin_surf_dist_m);

        if(dist_near < 1.0) {
            dist_near = 1.0;
        }

        // The far distance is set to the tangential distance
        // from the eye to the horizon (see horizon plane)
        dist_far = sqrt(eye_dist2 - radius*radius);
        return true;
    }
    return false;
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

    // Earth is approximated as a sphere:
    // x^2+y^2+z^2 = RAD_AV^2

    double a = ((rayDirn.x()*rayDirn.x()) * RAD_AV_INV_EXP2) +
               ((rayDirn.y()*rayDirn.y()) * RAD_AV_INV_EXP2) +
               ((rayDirn.z()*rayDirn.z()) * RAD_AV_INV_EXP2);

    double b = (2*rayPoint.x()*rayDirn.x() * RAD_AV_INV_EXP2) +
               (2*rayPoint.y()*rayDirn.y() * RAD_AV_INV_EXP2) +
               (2*rayPoint.z()*rayDirn.z() * RAD_AV_INV_EXP2);

    double c = ((rayPoint.x()*rayPoint.x()) * RAD_AV_INV_EXP2) +
               ((rayPoint.y()*rayPoint.y()) * RAD_AV_INV_EXP2) +
               ((rayPoint.z()*rayPoint.z()) * RAD_AV_INV_EXP2) - 1;

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

            if((xsecNear-rayPoint).length2() > (xsecFar-rayPoint).length2())
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

Plane CalcHorizonPlane(osg::Vec3d const &eye,
                       double const clamp_dist_m)
{
    Plane horizon_plane;
    horizon_plane.n = eye;
    horizon_plane.n.normalize();

    osg::Vec3d eye_clamped = eye;
    double const min_dist = clamp_dist_m+RAD_AV;
    double const min_dist2 = min_dist*min_dist;

    if((eye_clamped.length2()) < min_dist2) {
        eye_clamped = (horizon_plane.n*min_dist);
    }

    double eye_length = (eye_clamped.length());
    double const inv_dist = 1.0/eye_length;

    // by similar triangles
    horizon_plane.p = horizon_plane.n * (RAD_AV*RAD_AV*inv_dist);
    horizon_plane.d = (horizon_plane.n*horizon_plane.p);

    return horizon_plane;
}

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

bool CalcSphereOutsidePlane(Plane const &plane,
                            osg::Vec3d const &center,
                            double const radius)
{
    // min distance between sphere and plane
    double const dist = (plane.n * center) - plane.d;
    return (dist > radius);
}

void CalcProjFrustumPoly(Frustum const &frustum,
                         Plane const &horizon_plane,
                         std::vector<osg::Vec3d> &list_ecef)
{
    // Check if the planet is in the view frustum
    if(CalcSphereOutsideFrustumExact(frustum,osg::Vec3d(0,0,0),RAD_AV)) {
        return;
    }

    // Use 8 points on the view frustum edges to find
    // the LLA polygon:
    std::vector<osg::Vec3d> list_ecef_xsec(8);
    list_ecef_xsec[0] = frustum.list_vx[4]; // BL
    list_ecef_xsec[1] = (frustum.list_vx[4]+frustum.list_vx[5])*0.5;
    list_ecef_xsec[2] = frustum.list_vx[5]; // BR
    list_ecef_xsec[3] = (frustum.list_vx[5]+frustum.list_vx[6])*0.5;
    list_ecef_xsec[4] = frustum.list_vx[6]; // TR
    list_ecef_xsec[5] = (frustum.list_vx[6]+frustum.list_vx[7])*0.5;
    list_ecef_xsec[6] = frustum.list_vx[7]; // TL
    list_ecef_xsec[7] = (frustum.list_vx[7]+frustum.list_vx[4])*0.5;

    list_ecef.clear();

    osg::Vec3d const far_center = (frustum.list_vx[4]+frustum.list_vx[6])*0.5;
    osg::Vec3d const view_dirn = (far_center-frustum.eye);

    for(size_t i=0; i < list_ecef_xsec.size(); i++)
    {
        osg::Vec3d const &ray_pt = frustum.eye;
        osg::Vec3d const ray_dirn = list_ecef_xsec[i]-frustum.eye;

        osg::Vec3d xsec_near,xsec_far;
        if(CalcRayEarthIntersection(ray_pt,
                                    ray_dirn,
                                    xsec_near,
                                    xsec_far))
        {
            double const u_near = (xsec_near-ray_pt)*ray_dirn;
            double const u_far = (xsec_far-ray_pt)*ray_dirn;
            if((u_near > 0) && (u_far > 0))
            {
                if((xsec_near-frustum.eye)*view_dirn > 0.0) {
                    list_ecef_xsec[i] = xsec_near;
                }
                else {
                    list_ecef_xsec[i] = xsec_far;
                }
                list_ecef.push_back(list_ecef_xsec[i]);
                continue;
            }
        }
        // Project rays that missed the planetary
        // body onto the horizon plane
        osg::Vec3d const ecef_horizon =
                CalcPointPlaneProjection(list_ecef_xsec[i],
                                         horizon_plane);

        if(!CalcRayEarthIntersection(ecef_horizon,
                                     horizon_plane.p - ecef_horizon,
                                     xsec_near,
                                     xsec_far))
        {
            // should never get here
            std::cout << "###: FATAL" << std::endl;
            return;
        }

        list_ecef_xsec[i] = xsec_near;
    }

    list_ecef = list_ecef_xsec;
}

std::vector<std::pair<double,double>> CalcLonRange(std::vector<LLA> const &list_lla)
{
    static const double k_eps = 1E-8;

    std::vector<std::pair<double,double>> list_ranges;

    // Walk along each point in list_lla keeping track
    // of the max/min lon angles traveled

    double travel = 0.0;
    double max_cw = 0.0;
    double max_ccw = 0.0;

    for(size_t i=1; i < list_lla.size(); i++) {
        // Get the angle traveled between this point and the next
        double delta = list_lla[i].lon - list_lla[i-1].lon;

        // Choose the shorter travel distance
        if(delta < -180.0) {
            delta += 360.0;
        }
        else if(delta > 180.0) {
            delta -= 360.0;
        }

        travel += delta;

        if(delta > 0.0) {
            max_ccw = std::max(max_ccw,travel);
        }
        else {
            max_cw  = std::min(max_cw,travel);
        }
    }

    max_cw = fabs(max_cw);

    double const check_360 = 360.0 - k_eps;
    if((max_ccw > check_360) || (max_cw > check_360)) {
        // The lon path has gone around 360 degrees
        list_ranges.emplace_back(-180.0,180.0);
    }
    else {
        // Save longitude ranges taking the antemeridian
        // discontinuity into account
        double start_lon = list_lla[0].lon;

        if((start_lon+max_ccw) > 180.0) {
            double max_lon = CalcValidAngleDegs(start_lon+max_ccw,DEG_180_180);
            list_ranges.emplace_back(-180.0,max_lon);           // a
            list_ranges.emplace_back(start_lon-max_cw,180.0);   // b
        }
        else if((start_lon-max_cw) < -180.0) {
            double min_lon = CalcValidAngleDegs(start_lon-max_cw,DEG_180_180);
            list_ranges.emplace_back(min_lon,180.0);            // a
            list_ranges.emplace_back(-180.0,start_lon+max_ccw); // b
        }
        else {
            // lon range doesn't cross the discontinuity
            list_ranges.emplace_back(start_lon-max_cw,start_lon+max_ccw);
        }
    }

    return list_ranges;
}

bool CalcMinGeoBoundsFromLLAPoly(LLA const &camLLA,
                                 std::vector<LLA> const &listPLLA,
                                 std::vector<GeoBounds> &listBounds)
{
    listBounds.clear();
    if(listPLLA.size() < 3)   {
        std::cout << "WARN: CalcGeoBounds: "
                    "Insufficient coords (min 3)\n";
        return false;
    }

    std::vector<double> listLonDegs(listPLLA.size());
    for(size_t i=0; i < listPLLA.size(); i++)   {
        listLonDegs[i] = listPLLA[i].lon;
    }
    listLonDegs.push_back(listLonDegs[0]); // wrap around

    // Walk along the polygon keeping track of
    // max/min angles traveled
    bool is360 = false;
    double travelDegs = 0;
    double maxTravelDegsCW = 0;
    double maxTravelDegsCCW = 0;
    for(size_t i=1; i < listLonDegs.size(); i++)   {
        double angleDelta = listLonDegs[i]-listLonDegs[i-1];

        // Get both CW and CCW angles for the delta
        double angleCW,angleCCW;
        if(angleDelta > 0.0)   {
            angleCCW = angleDelta;
            angleCW = angleCCW-360.0;
        }
        else if(angleDelta < 0.0)  {
            angleCW = angleDelta;
            angleCCW = 360.0+angleCW;
        }
        else   {
            continue;
        }

        // We track the shortest distance in degrees
        // between subsequent polygon points
        if(fabs(angleCCW) < fabs(angleCW))   {
            travelDegs += angleCCW;
            maxTravelDegsCCW = std::max(maxTravelDegsCCW,travelDegs);
        }
        else if(fabs(angleCW) < fabs(angleCCW))   {
            travelDegs += angleCW;
            maxTravelDegsCW  = std::min(maxTravelDegsCW,travelDegs);
        }
        else   {
            // This case indicates angleCW == angleCCW, which
            // means that the polygon segment passes through
            // the center of a circle of longitudes. We consider
            // this to mean the polygon segments travel the
            // full range from -180 to 180 degrees longitude.
            is360 = true;
            break;
        }
    }
    maxTravelDegsCW = fabs(maxTravelDegsCW);

    // Check if the polygon traveled 360 degrees.
    // Allow for an error in degrees.
    double checkError = 5.0;
    double check360 = 360.0 - checkError;

    if(is360 ||
       (maxTravelDegsCCW > check360) ||
       (maxTravelDegsCW  > check360))
    {
        GeoBounds b; b.minLon = -180.0; b.maxLon = 180.0;
        listBounds.push_back(b);
        is360 = true;
    }
    else   {
        // Save longitude bounds taking the antemeridian
        // discontinuity into account

        // Set startLon as one of the extreme lon values
        // by adding the furthest traveled CCW distance
        double startLon = listPLLA[0].lon;

        if(startLon+maxTravelDegsCCW > 180.0)   {
            double maxLon = startLon+maxTravelDegsCCW;
            maxLon = CalcValidAngleDegs(maxLon,DEG_180_180);

            GeoBounds a,b;
            a.minLon = -180.0; a.maxLon = maxLon;
            b.minLon = startLon-maxTravelDegsCW; b.maxLon = 180.0;
            listBounds.push_back(a);
            listBounds.push_back(b);

//            std::cout << "//: ping: " << a.minLon << ", " << a.maxLon << ",,, " << b.minLon << ", " << b.maxLon << std::endl;
        }
        else if(startLon-maxTravelDegsCW < -180.0)   {
            double minLon = startLon-maxTravelDegsCW;
            minLon = CalcValidAngleDegs(minLon,DEG_180_180);

            GeoBounds a,b;
            a.minLon = minLon; a.maxLon = 180.0;
            b.minLon = -180.0; b.maxLon = startLon+maxTravelDegsCCW;
            listBounds.push_back(a);
            listBounds.push_back(b);

//            std::cout << "//: pong: " << a.minLon << ", " << a.maxLon << ",,, " << b.minLon << ", " << b.maxLon << std::endl;
        }
        else   {
            GeoBounds b;
            b.minLon = startLon-maxTravelDegsCW;
            b.maxLon = startLon+maxTravelDegsCCW;
            listBounds.push_back(b);
        }
    }

    // Latitude Range

    // Its possible that the max and min latitude lie
    // on the surface between the given polygon points.

    // We very roughly try to account for this by
    // adding the latitude at the pole closest to
    // the camera as a critical latitude to compare
    // the existing latitudes with.
    double critLat = listPLLA[0].lat;
    if(is360)   {
        // check if the camera is above or below the 'equator'
        critLat = (camLLA.lat > 0) ? 90.0 : -90.0;
//        std::cout << "#: is360 and critLat @ " << critLat << std::endl;
    }

    // calc min/max for latitude
    double minLat = 90.0; double maxLat = -90.0;
    for(size_t i=0; i < listPLLA.size(); i++)   {
        minLat = std::min(minLat,listPLLA[i].lat);
        maxLat = std::max(maxLat,listPLLA[i].lat);
    }
    minLat = std::min(minLat,critLat);
    maxLat = std::max(maxLat,critLat);

    // save lat
    for(size_t i=0; i < listBounds.size(); i++)   {
        listBounds[i].minLat = minLat;
        listBounds[i].maxLat = maxLat;
    }

    return true;
}


void BuildEarthSurface(double min_lon,
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
            LLA lla;
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


void BuildEarthSurface(double min_lon,
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
            LLA lla;
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
            list_ix.push_back(v_idx);                   // 0
            list_ix.push_back(v_idx+1);                 // 1
            list_ix.push_back(v_idx+lon_segments+2);    // 2

            list_ix.push_back(v_idx);                   // 0
            list_ix.push_back(v_idx+lon_segments+2);    // 2
            list_ix.push_back(v_idx+lon_segments+1);    // 3

            v_idx++;
        }
        v_idx++;
    }
}

// ============================================================= //
// ============================================================= //
