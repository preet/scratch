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
    if(fabs(clip4.w()) < K_EPS) {
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

osg::Vec2d CalcPerpendicular(osg::Vec2d v)
{
    std::swap(v.x(),v.y());
    v.y() *= -1.0;
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

    if(CalcPolyIsInFront(CalcPerpendicular(rect[1]-rect[0]),rect[0],tri)) {
        return false;
    }
    if(CalcPolyIsInFront(CalcPerpendicular(rect[2]-rect[1]),rect[1],tri)) {
        return false;
    }
    if(CalcPolyIsInFront(CalcPerpendicular(rect[3]-rect[2]),rect[2],tri)) {
        return false;
    }
    if(CalcPolyIsInFront(CalcPerpendicular(rect[0]-rect[3]),rect[3],tri)) {
        return false;
    }

    // (test rectangle against triangle edge normals)
    if(CalcPolyIsInFront(CalcPerpendicular(tri[1]-tri[0]),tri[0],rect)) {
        return false;
    }
    if(CalcPolyIsInFront(CalcPerpendicular(tri[2]-tri[1]),tri[1],rect)) {
        return false;
    }
    if(CalcPolyIsInFront(CalcPerpendicular(tri[0]-tri[2]),tri[2],rect)) {
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

void BuildEarthSurface(double minLon, double minLat,
                       double maxLon, double maxLat,
                       uint16_t lonSegments,
                       uint16_t latSegments,
                       std::vector<osg::Vec3d> &vertexArray,
                       std::vector<osg::Vec2d> &texCoords,
                       std::vector<uint16_t> &triIdx)
{
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
            LLA lla;
            lla.lon = (j*lonStep)+minLon;
            lla.lat = (i*latStep)+minLat;
            lla.alt = 0.0;
            vertexArray.push_back(ConvLLAToECEF(lla));
            // surface tex coord
            texCoords.push_back(osg::Vec2d((double(j)/lonSegments),
                                           (double(i)/latSegments)));
        }
    }

    // stitch faces together
    // TODO: optimize push_back
//    triIdx.reserve(lonSegments*latSegments*6);
    uint16_t vIdx=0;
    for(uint16_t i=0; i < latSegments; i++)   {
        for(uint16_t j=0; j < lonSegments; j++)   {
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
}

void BuildEarthSurface(double minLon, double minLat,
                       double maxLon, double maxLat,
                       uint16_t lonSegments,
                       uint16_t latSegments,
                       std::vector<osg::Vec3d> &vertexArray,
                       std::vector<uint16_t> &triIdx)
{
    double lonStep = (maxLon-minLon)/lonSegments;
    double latStep = (maxLat-minLat)/latSegments;

    vertexArray.clear();
    triIdx.clear();

    // build vertex attributes
//    vertexArray.reserve((latSegments+1)*(lonSegments+1));
//    texCoords.reserve((latSegments+1)*(lonSegments+1));
    for(size_t i=0; i <= latSegments; i++)   {
        for(size_t j=0; j <= lonSegments; j++)   {
            // surface vertex
            LLA lla;
            lla.lon = (j*lonStep)+minLon;
            lla.lat = (i*latStep)+minLat;
            lla.alt = 0.0;
            vertexArray.push_back(ConvLLAToECEF(lla));
        }
    }

    // stitch faces together
    // TODO: optimize push_back
//    triIdx.reserve(lonSegments*latSegments*6);
    uint16_t vIdx=0;
    for(uint16_t i=0; i < latSegments; i++)   {
        for(uint16_t j=0; j < lonSegments; j++)   {
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
}

// ============================================================= //
// ============================================================= //
