/*
   This source is part of osmsrender

   Copyright 2012 Preet Desai

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

#ifndef SHPTK_HPP
#define SHPTK_HPP

#include <math.h>
#include <cstdlib>
#include <vector>

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

class Vec2
{
public:
    Vec2() :
        x(0),y(0) {}

    Vec2(double myX, double myY) :
        x(myX),y(myY) {}

    inline double Dot(Vec2 const & otherVec) const
    {
        return (x*otherVec.x)+
                (y*otherVec.y);
    }

    inline double PerpDot(Vec2 const &otherVec) const
    {   // perpendicular dot product operator
        // http://mathworld.wolfram.com/PerpDotProduct.html
        // http://johnblackburne.blogspot.ca/2012/02/...
        // ...perp-dot-product.html
        return (x*otherVec.y)-
                (y*otherVec.x);
    }

    inline double Magnitude() const
    {
        return sqrt(x*x + y*y);
    }

    inline double DistanceTo(Vec2 const &otherVec) const
    {
        return sqrt((x-otherVec.x)*(x-otherVec.x) +
                    (y-otherVec.y)*(y-otherVec.y));
    }

    inline double Distance2To(Vec2 const &otherVec) const
    {
        return ((x-otherVec.x)*(x-otherVec.x) +
                (y-otherVec.y)*(y-otherVec.y));
    }

    inline double AngleTo(Vec2 const &otherVec) const
    {   // returns the min angle between this
        // vector and otherVec (range is +/- pi)
        // http://johnblackburne.blogspot.ca/2012/01/...
        // ...angle-between-two-vectors.html

        // the angle is positive for CCW
        double dot = this->Dot(otherVec);
        double perpdot = this->PerpDot(otherVec);
        return atan2(perpdot,dot);
    }

    inline Vec2 Normalized() const
    {
        double vecMagnitude = sqrt(x*x + y*y);

        return Vec2(x/vecMagnitude,
                    y/vecMagnitude);
    }

    inline Vec2 ScaledBy(double scaleFactor) const
    {
        return Vec2(x*scaleFactor,
                    y*scaleFactor);
    }

    inline Vec2 operator+ (const Vec2 &otherVec) const
    {
        return Vec2(x+otherVec.x,
                    y+otherVec.y);
    }

    inline Vec2 operator- (const Vec2 &otherVec) const
    {
        return Vec2(x-otherVec.x,
                    y-otherVec.y);
    }

    inline Vec2 & operator = (const Vec2 &otherVec)
    {
        x = otherVec.x;
        y = otherVec.y;
        return *this;
    }

    inline bool operator == (Vec2 const &rhs) const
    {
        if(x == rhs.x)   {
            if(y == rhs.y)   {
                return true;
            }
        }
        return false;
    }

    double x;
    double y;
};

class Vec3
{
public:
    Vec3() :
        x(0),y(0),z(0) {}

    Vec3(double myX, double myY, double myZ) :
        x(myX),y(myY),z(myZ) {}

    inline double Dot(Vec3 const & otherVec) const
    {
        return (x*otherVec.x)+
                (y*otherVec.y)+
                (z*otherVec.z);
    }

    inline Vec3 Cross(Vec3 const & otherVec) const
    {
        return Vec3((y*otherVec.z - z*otherVec.y),
                    (z*otherVec.x - x*otherVec.z),
                    (x*otherVec.y - y*otherVec.x));
    }

    inline double Magnitude() const
    {
        return sqrt(x*x + y*y + z*z);
    }

    inline double DistanceTo(Vec3 const &otherVec) const
    {
        double dx = x-otherVec.x;
        double dy = y-otherVec.y;
        double dz = z-otherVec.z;
        return (sqrt(dx*dx + dy*dy + dz*dz));
    }

    inline double Distance2To(Vec3 const &otherVec) const
    {
        double dx = x-otherVec.x;
        double dy = y-otherVec.y;
        double dz = z-otherVec.z;
        return (dx*dx + dy*dy + dz*dz);
    }

    inline Vec3 Normalized() const
    {
        double vecMagnitude = sqrt(x*x + y*y + z*z);

        return Vec3(x/vecMagnitude,
                    y/vecMagnitude,
                    z/vecMagnitude);
    }

    inline Vec3 ScaledBy(double scaleFactor) const
    {
        return Vec3(x*scaleFactor,
                    y*scaleFactor,
                    z*scaleFactor);
    }

    inline Vec3 RotatedBy(Vec3 const &axisVec, double angleDegCCW)
    {
        if(!angleDegCCW)
        {   return Vec3(this->x,this->y,this->z);   }

        Vec3 rotatedVec;
        double angleRad = angleDegCCW*3.141592653589/180.0;
        rotatedVec = this->ScaledBy(cos(angleRad)) +
                (axisVec.Cross(*this)).ScaledBy(sin(angleRad)) +
                axisVec.ScaledBy(axisVec.Dot(*this)).ScaledBy(1-cos(angleRad));

        return rotatedVec;
    }

    inline Vec3 operator+ (const Vec3 &otherVec) const
    {
        return Vec3(x+otherVec.x,
                    y+otherVec.y,
                    z+otherVec.z);
    }

    inline Vec3 operator- (const Vec3 &otherVec) const
    {
        return Vec3(x-otherVec.x,
                    y-otherVec.y,
                    z-otherVec.z);
    }

    double x;
    double y;
    double z;
};

struct PointLLA
{
    PointLLA() :
        lon(0),lat(0),alt(0) {}

    PointLLA(double myLat, double myLon) :
        lat(myLat),lon(myLon),alt(0) {}

    PointLLA(double myLat, double myLon, double myAlt) :
        lon(myLon),lat(myLat),alt(myAlt) {}

    double lat;
    double lon;
    double alt;
};

Vec3 ConvLLAToECEF(const PointLLA &pointLLA)
{
    Vec3 pointECEF;

    // remember to convert deg->rad
    double sinLat = sin(pointLLA.lat * K_DEG2RAD);
    double sinLon = sin(pointLLA.lon * K_DEG2RAD);
    double cosLat = cos(pointLLA.lat * K_DEG2RAD);
    double cosLon = cos(pointLLA.lon * K_DEG2RAD);

    // v = radius of curvature (meters)
    double v = ELL_SEMI_MAJOR / (sqrt(1-(ELL_ECC_EXP2*sinLat*sinLat)));
    pointECEF.x = (v + pointLLA.alt) * cosLat * cosLon;
    pointECEF.y = (v + pointLLA.alt) * cosLat * sinLon;
    pointECEF.z = ((1-ELL_ECC_EXP2)*v + pointLLA.alt)*sinLat;

    return pointECEF;
}

PointLLA ConvECEFToLLA(const Vec3 &pointECEF)
{
    PointLLA pointLLA;

    double p = (sqrt(pow(pointECEF.x,2) + pow(pointECEF.y,2)));
    double th = atan2(pointECEF.z*ELL_SEMI_MAJOR, p*ELL_SEMI_MINOR);
    double sinTh = sin(th);
    double cosTh = cos(th);

    // calc longitude
    pointLLA.lon = atan2(pointECEF.y, pointECEF.x);

    // calc latitude
    pointLLA.lat = atan2(pointECEF.z + ELL_ECC2_EXP2*ELL_SEMI_MINOR*sinTh*sinTh*sinTh,
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

double CalcTriangleArea(const Vec3 &vxA,
                        const Vec3 &vxB,
                        const Vec3 &vxC)
{   // using half cross product magnitude
    return fabs(((vxB-vxA).Cross(vxB-vxC)).Magnitude()/2.0);
}

enum VWSimplifyType
{
    VW_AREA,
    VW_QUANTITY
};

// VWPoint32
// * 3d point struct for Visvalingam-Whyatt
//   polyline simplification
struct VWPoint3
{
    VWPoint3() : next_vx(NULL),prev_vx(NULL) {}

    Vec3 vx;
    VWPoint3 * prev_vx;
    VWPoint3 * next_vx;
    std::multimap<double,VWPoint3*>::iterator it;
};

void CalcPolylineSimplifyVW(std::vector<Vec3> const &listVx,
                            std::vector<Vec3> &listSimpleVx,
                            VWSimplifyType simplifyType,
                            double simplifyParam)
{
    // refs:
    // http://bost.ocks.org/mike/simplify/
    // http://www2.dcs.hull.ac.uk/CISRG/publications/DPs/DP10/DP10.html

    if(listVx.size() < 5)   {
        // no need to simplify
        listSimpleVx = listVx;
        return;
    }

    double areaTolerance = 1E20;    // ie very large
    size_t numVxFinal = 0;

    if(simplifyType == VW_AREA)   {
        // simplify by eliminating any points with an
        // effective area underneath the given limit
        areaTolerance = simplifyParam;
    }
    else if(simplifyType == VW_QUANTITY)   {
        // simplify by eliminating points until a specified
        // fraction of the original point set remains
        double retentionRatio = simplifyParam;
        if(retentionRatio < 0.0 || retentionRatio >= 1.0)   {
            std::cout << "CalcPolylineSimplifyVW: Invalid "
                        "retention ratio" << std::endl;
            listSimpleVx = listVx;
            return;
        }
        numVxFinal = (listVx.size() * retentionRatio);
        if(numVxFinal < 4)   {   numVxFinal = 4;   }    // clamp
    }

    VWPoint3 * vwpoint;
    std::vector<VWPoint3*> listVWPoints;
    listVWPoints.reserve(listVx.size());
    std::multimap<double,VWPoint3*> mapAreaToVx;
    std::multimap<double,VWPoint3*>::iterator itArea;
    for(size_t i=1; i < listVx.size()-1; i++)
    {
        double triArea =
            CalcTriangleArea(listVx[i-1],listVx[i],listVx[i+1]);

        if(triArea > 0)   {
            std::pair<double,VWPoint3*> insData;
            insData.first  = triArea;
            insData.second = NULL;
            itArea = mapAreaToVx.insert(insData);

            vwpoint = new VWPoint3;
            vwpoint->vx = listVx[i];
            vwpoint->it = itArea;

            itArea->second = vwpoint;
            listVWPoints.push_back(vwpoint);
        }
    }
    //
    vwpoint = new VWPoint3;
    vwpoint->vx = listVx[0];
    vwpoint->next_vx = listVWPoints[0];
    listVWPoints.insert(listVWPoints.begin(),vwpoint);

    vwpoint = new VWPoint3;
    vwpoint->vx = listVx[listVx.size()-1];
    vwpoint->prev_vx = listVWPoints.back();
    listVWPoints.push_back(vwpoint);

    //
    for(size_t i=1; i < listVWPoints.size()-1; i++)   {
        listVWPoints[i]->prev_vx = listVWPoints[i-1];
        listVWPoints[i]->next_vx = listVWPoints[i+1];
    }

    //
    while(mapAreaToVx.size() > numVxFinal)
    {
        // Lookup the vertex with the minimum area
        itArea = mapAreaToVx.begin();
        VWPoint3 * vw = itArea->second;
        bool isFirst = (vw->prev_vx->prev_vx == NULL);
        bool isLast  = (vw->next_vx->next_vx == NULL);

        // stop if all subsequent points have a greater
        // area than the specified tolerance
        if(itArea->first > areaTolerance)
        {   break;   }

        // Delete the minimum area vertex
        mapAreaToVx.erase(itArea);

        // adjust the adjacent vertex pointers
        vw->prev_vx->next_vx = vw->next_vx;
        vw->next_vx->prev_vx = vw->prev_vx;

        if(!isLast)   {
            // recalculate the adjacent triangle area for
            // the next vx (vx_prev,vx_next,vx_next_next)
            double nextArea = CalcTriangleArea(vw->prev_vx->vx,
                                               vw->next_vx->vx,
                                               vw->next_vx->next_vx->vx);
            mapAreaToVx.erase(vw->next_vx->it);
            vw->next_vx->it = mapAreaToVx.insert(std::make_pair(nextArea,vw->next_vx));
        }

        if(!isFirst)  {
            // recalculate the adjacent triangle area for
            // the prev vx (vx_prev_prev,vx_prev,vx_next)
            double prevArea = CalcTriangleArea(vw->prev_vx->prev_vx->vx,
                                               vw->prev_vx->vx,
                                               vw->next_vx->vx);
            mapAreaToVx.erase(vw->prev_vx->it);
            vw->prev_vx->it = mapAreaToVx.insert(std::make_pair(prevArea,vw->prev_vx));
        }
    }

    // save points
    listSimpleVx.clear();
    listSimpleVx.reserve(numVxFinal+2);
    vwpoint = listVWPoints[0];
    while(vwpoint != NULL)
    {
        listSimpleVx.push_back(vwpoint->vx);
        vwpoint = vwpoint->next_vx;
    }

    // clean up
    for(size_t i=0; i < listVWPoints.size(); i++)   {
        delete listVWPoints[i];
    }
}



#endif // SHPTK
