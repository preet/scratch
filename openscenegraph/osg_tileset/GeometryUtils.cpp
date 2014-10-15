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

    double const u = plane.d - (plane.n*a)/u_den;
    if(u >= 0.0 && u <= 1.0) {
        xsec = a + ab*u;
        return Intersection::TRUE;
    }

    return Intersection::FALSE;
}

GeometryResult CalcTrianglePlaneClip(std::vector<osg::Vec3d> const &tri,
                                     Plane const &plane,
                                     std::vector<osg::Vec3d> &inside,
                                     std::vector<osg::Vec3d> &outside)
{
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
