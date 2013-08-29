/*   

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
   * 
   */
   
#include <string>
#include <iostream>
#include <fstream>
#include <vector>

#include <osg/Program>
#include <osg/Uniform>
#include <osg/Geometry>
#include <osgViewer/Viewer>
#include <osg/AutoTransform>
#include <osgViewer/CompositeViewer>
#include <osgGA/TrackballManipulator>
#include <osg/MatrixTransform>
#include <osg/CullFace>

#include "../../UNTRACKED/Vec2.hpp"
#include "../../UNTRACKED/Vec3.hpp"

// geometric defines!
// PI!
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
#define CIR_AV_FOURTH 40041438.0/4.0

struct PointLLA
{
    PointLLA() :
        lon(0),lat(0),alt(0) {}

    PointLLA(double myLat, double myLon) :
        lon(myLon),lat(myLat),alt(0) {}

    PointLLA(double myLat, double myLon, double myAlt) :
        lon(myLon),lat(myLat),alt(myAlt) {}

    double lon;
    double lat;
    double alt;
};

struct LODRange
{
    LODRange() :
        min(0),max(0),level(0)
    {}

    double min;
    double max;
    size_t level;
    bool active;
};

struct VExtVertex
{
    VExtVertex() {}

    Vec3 ecef;
    PointLLA lla;
    bool visible;
    bool outsideFrustum;
    bool beyondHorizon;
};

struct GeoBounds
{
    GeoBounds() :
        minLat(0),maxLat(0),
        minLon(0),maxLon(0)
    {}

    double minLat; double maxLat;
    double minLon; double maxLon;
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

bool CalcPointBeyondHorizonPlane(Vec3 const &scaledSpaceEye,
                                 Vec3 const &scaledSpaceCenter)
{
    // ref: http://cesium.agi.com/2013/04/25/Horizon-culling/

    // camera in scaled space
    Vec3 cv = scaledSpaceEye;

    // object to be tested in scaled space
    Vec3 t = scaledSpaceCenter;

    // camera to object vector
    Vec3 vt = t - cv;

    double vh_mag_2 = (cv.x*cv.x + cv.y*cv.y + cv.z*cv.z) - 1.0;
    double vt_dot_vc = vt.Dot(cv.ScaledBy(-1.0));

    return (vt_dot_vc > vh_mag_2);
}

enum IntersectionType
{
    XSEC_FALSE,
    XSEC_TRUE,
    XSEC_COINCIDENT,
    XSEC_PARALLEL,
    XSEC_CONTAINED
};

IntersectionType CalcCircleCircleIntersection(Vec2 centerA, double radA,
                                              Vec2 centerB, double radB,
                                              std::vector<Vec2> &xsecPoints)
{
    // ref:
    // http://paulbourke.net/geometry/circlesphere/

    // let dist be the distance between the centers
    // of circles A and B
    double dist = centerA.DistanceTo(centerB);

    if(dist > (radA+radB))
    {   // The circles are too far away to intersect
        return XSEC_FALSE;
    }
    else if(dist < fabs(radA-radB))
    {   // One circle is contained within the other
        return XSEC_CONTAINED;
    }
    else if((dist == 0) && (radA==radB))
    {   // The circles are coincident
        return XSEC_COINCIDENT;
    }

    double a = (radA*radA - radB*radB + dist*dist) / (2.0*dist);
    double h = sqrt(radA*radA - a*a);
    Vec2 p2(centerA + (centerB-centerA).ScaledBy(a/dist));
    double rx = (centerB.y-centerA.y)*(h/dist);
    double ry = (centerB.x-centerA.x)*(h/dist);

    Vec2 xsec1;
    xsec1.x = p2.x + rx;
    xsec1.y = p2.y - ry;
    xsecPoints.push_back(xsec1);

    if(dist != (radA + radB))   {
        // (ie, if there's more than one POI)
        Vec2 xsec2;
        xsec2.x = p2.x - rx;
        xsec2.y = p2.y + ry;
        xsecPoints.push_back(xsec2);
    }

    return XSEC_TRUE;
}

std::string ReadFileAsString(std::string const &fileName)
{
    std::ifstream ifs(fileName.c_str());
    std::string content( (std::istreambuf_iterator<char>(ifs) ),
                         (std::istreambuf_iterator<char>()    ) );
    return content;
}

std::vector<LODRange> BuildLODRanges()
{
    LODRange range;
    std::vector<LODRange> listRanges;

    range.min = 0;
    range.max = 800;
    listRanges.push_back(range);

    range.min = 800;
    range.max = 2000;
    listRanges.push_back(range);

    range.min = 2000;
    range.max = 4000;
    listRanges.push_back(range);

    range.min = 4000;
    range.max = 8000;
    listRanges.push_back(range);

    range.min = 8000;
    range.max = 16000;
    listRanges.push_back(range);

    range.min = 16000;
    range.max = 50000;
    listRanges.push_back(range);

    range.min = 50000;
    range.max = 100000;
    listRanges.push_back(range);

    range.min = 100000;
    range.max = 200000;
    listRanges.push_back(range);

    range.min = 200000;
    range.max = 400000;
    listRanges.push_back(range);

    range.min = 400000;
    range.max = 800000;
    listRanges.push_back(range);

    range.min = 800000;
    range.max = 2000000;
    listRanges.push_back(range);

    range.min = 2000000;
    range.max = 5000000;
    listRanges.push_back(range);

    range.min = 5000000;
    range.max = 12500000;
    listRanges.push_back(range);

    range.min = 12500000;
    range.max = 25000000;
    listRanges.push_back(range);

    range.min = 25000000;
    range.max = 50000000;
    listRanges.push_back(range);

    for(size_t i=0; i < listRanges.size(); i++)   {
        double avRange = (listRanges[i].min+
                          listRanges[i].max)/2.0;
        double viewMag = (CIR_AV)/avRange;
        double level = log2(viewMag);
        size_t roundedLevel = size_t(round(level));
        listRanges[i].level = roundedLevel;
    }

    return listRanges;
}

std::vector<LODRange> g_LIST_LOD_RANGES;

bool BuildEarthSurfaceGeometry(double minLon, double minLat,
                               double maxLon, double maxLat,
                               size_t lonSegments,
                               size_t latSegments,
                               std::vector<VExtVertex> &listVx,
                               std::vector<size_t> &listIx,
                               double alt=0.0)
{
    double lonStep = (maxLon-minLon)/lonSegments;
    double latStep = (maxLat-minLat)/latSegments;

    listVx.clear();
    listIx.clear();

    // build vertex attributes
    for(size_t i=0; i <= latSegments; i++)   {
        for(size_t j=0; j <= lonSegments; j++)   {
            // surface vertex
            PointLLA lla((i*latStep)+minLat,(j*lonStep)+minLon,alt);
            Vec3 ecef = ConvLLAToECEF(lla);
            VExtVertex vx;
            vx.ecef = ecef;
            vx.lla = lla;
            listVx.push_back(vx);
        }
    }

    // stitch faces together
    size_t vIdx=0;
    for(size_t i=0; i < latSegments; i++)   {
        for(size_t j=0; j < lonSegments; j++)   {
            listIx.push_back(vIdx);
            listIx.push_back(vIdx+lonSegments+2);
            listIx.push_back(vIdx+lonSegments+1);

            listIx.push_back(vIdx);
            listIx.push_back(vIdx+1);
            listIx.push_back(vIdx+lonSegments+2);

            vIdx++;
        }
        vIdx++;
    }
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

bool CalcRayEarthIntersection(Vec3 const &rayPoint,
                              Vec3 const &rayDirn,
                              Vec3 &xsecNear,
                              Vec3 &xsecFar)
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

    double a = ((rayDirn.x*rayDirn.x) / ELL_SEMI_MAJOR_EXP2) +
               ((rayDirn.y*rayDirn.y) / ELL_SEMI_MAJOR_EXP2) +
               ((rayDirn.z*rayDirn.z) / ELL_SEMI_MINOR_EXP2);

    double b = (2*rayPoint.x*rayDirn.x/ELL_SEMI_MAJOR_EXP2) +
               (2*rayPoint.y*rayDirn.y/ELL_SEMI_MAJOR_EXP2) +
               (2*rayPoint.z*rayDirn.z/ELL_SEMI_MINOR_EXP2);

    double c = ((rayPoint.x*rayPoint.x) / ELL_SEMI_MAJOR_EXP2) +
               ((rayPoint.y*rayPoint.y) / ELL_SEMI_MAJOR_EXP2) +
               ((rayPoint.z*rayPoint.z) / ELL_SEMI_MINOR_EXP2) - 1;

    std::vector<double> listRoots;
    CalcQuadraticEquationReal(a,b,c,listRoots);
    if(!listRoots.empty())
    {
        if(listRoots.size() == 1)   {
            xsecNear.x = rayPoint.x + listRoots[0]*rayDirn.x;
            xsecNear.y = rayPoint.y + listRoots[0]*rayDirn.y;
            xsecNear.z = rayPoint.z + listRoots[0]*rayDirn.z;
            xsecFar = xsecNear;
            return true;
        }
        else   {
            xsecNear.x = rayPoint.x + listRoots[0]*rayDirn.x;
            xsecNear.y = rayPoint.y + listRoots[0]*rayDirn.y;
            xsecNear.z = rayPoint.z + listRoots[0]*rayDirn.z;

            xsecFar.x = rayPoint.x + listRoots[1]*rayDirn.x;
            xsecFar.y = rayPoint.y + listRoots[1]*rayDirn.y;
            xsecFar.z = rayPoint.z + listRoots[1]*rayDirn.z;

            if(rayPoint.Distance2To(xsecNear) > rayPoint.Distance2To(xsecFar))
            {
                Vec3 temp = xsecNear;
                xsecNear = xsecFar;
                xsecFar = temp;
            }
            return true;
        }
    }
    return false;
}

void CullPointsAgainstHorizon(Vec3 const &scaledSpaceEye,
                              std::vector<VExtVertex*> &listVx)
{
    // transform the vertex into scaled space
    for(size_t i=0; i < listVx.size(); i++)   {
        Vec3 scaledSpaceVx(listVx[i]->ecef.x/ELL_SEMI_MAJOR,
                           listVx[i]->ecef.y/ELL_SEMI_MAJOR,
                           listVx[i]->ecef.z/ELL_SEMI_MINOR);

        listVx[i]->beyondHorizon = false;
        if(CalcPointBeyondHorizonPlane(scaledSpaceEye,scaledSpaceVx))
        {   listVx[i]->beyondHorizon = true;   }
    }
}

void CullPointsAgainstFrustum(std::vector<Vec3> const &listFPlaneNorms,
                              std::vector<Vec3> const &listFPlanePoints,
                              std::vector<VExtVertex*> &listVx)
{
    for(size_t i=0; i < listVx.size(); i++)   {
        listVx[i]->outsideFrustum = false;
        for(size_t j=0; j < listFPlanePoints.size(); j++)   {
            // vector from point on plane to vertex
            Vec3 p_to_vx = listVx[i]->ecef-listFPlanePoints[j];
            if(p_to_vx.Dot(listFPlaneNorms[j]) > 0)   {
                // a positive dot product indicates the point is
                // outside the given plane wrt to the frustum
                listVx[i]->outsideFrustum = true;
                break;
            }
        }
    }
}

osg::Vec4 CalcRainbowGradient(double cVal)
{
    // clamp cVal between 0 and 1
    if(cVal < 0)   {
        cVal = 0.0;
    }
    if(cVal > 1)   {
        cVal = 1.0;
    }

    unsigned char R,G,B;
    size_t maxBars = 6;     // number of color bars

    double m = maxBars * cVal;
    size_t n = size_t(m);

    double fraction = m-n;
    unsigned char t = int(fraction*255);

    switch(n)   {
        case 0:   {
            R = 255;
            G = t;
            B = 0;
            break;
        }
        case 1:   {
            R = 255 - t;
            G = 255;
            B = 0;
            break;
        }
        case 2:   {
            R = 0;
            G = 255;
            B = t;
            break;
        }
        case 3:   {
            R = 0;
            G = 255 - t;
            B = 255;
            break;
        }
        case 4:   {
            R = t;
            G = 0;
            B = 255;
            break;
        }
        case 5:   {
            R = 255;
            G = 0;
            B = 255 - t;
            break;
        }
    }

    osg::Vec4 myColor(R/255.0,G/255.0,B/255.0,1.0);

    return myColor;
}

void CalcValidAngle(double &angle)
{
    // TODO: make valid for any angle
    //       not just 0-360

    if(angle > 180.0)   {
        angle -= 360.0;
    }
    else if(angle < -180.0)   {
        angle += 360.0;
    }
}

bool CalcGeoBoundsFromLLAPoly(Vec3 eye,
                   std::vector<PointLLA> const &listPLLA,
                   std::vector<GeoBounds> &listBounds)
{
    listBounds.clear();
    if(listPLLA.size() < 3) {
        std::cout << "WARN: CalcGeoBounds: "
                    "Insufficient coords ("
                 << listPLLA.size() << ")" << std::endl;
        return false;
    }

    // Longitude Range

    // Determine the longitude range of listPLLA by converting
    // longitudes to 2d cartesian (x,y) and 'walking' along the
    // polygon, keeping track of CW/CCW distance traveled

    std::vector<double> listLonRads(listPLLA.size());
    std::vector<Vec2> listVxLon(listPLLA.size());
    for(size_t i=0; i < listPLLA.size(); i++) {
        double angleRads = listPLLA[i].lon*K_PI/180.0;
        listLonRads[i] = angleRads;
        listVxLon[i].x = cos(angleRads);
        listVxLon[i].y = sin(angleRads);
    }

    // walk along the polygon keeping track of max/min
    // angles; direction is relative to (x,y) = (0,0)
    bool is360 = false;
    Vec3 vecCenter(0,0,0);
    listVxLon.push_back(listVxLon[0]); // wrap around
    listLonRads.push_back(listLonRads[0]); // wrap around

    double diffAngle = 0;
    double angleRads = 0;
    double travelRadsCW = 0;
    double travelRadsCCW = 0;

    for(size_t i=1; i < listVxLon.size(); i++)
    {
        diffAngle = listLonRads[i]-listLonRads[i-1];

        if(diffAngle == 0) {
            // adjacent points w/ same lon
            continue;
        }
        else if(diffAngle > K_PI) {
            diffAngle -= (2*K_PI);
        }
        else if(diffAngle < K_PI*-1) {
            diffAngle += (2*K_PI);
        }
        diffAngle = fabs(diffAngle);


        Vec3 vecLon(listVxLon[i-1].x,listVxLon[i-1].y,0);
        Vec3 vecDirn(listVxLon[i].x-vecLon.x,
                     listVxLon[i].y-vecLon.y,0);

        if(vecDirn.x == 0 && vecDirn.y == 0) {
            // adjacent points w/ same longitude
            continue;
        }

        // using the right-hand rule, upVec determines
        // whether we're traversing the polygon edge CW
        // or CCW wrt to the center
        Vec3 vecUp = (vecLon-vecCenter).Cross(vecDirn);
        if(vecUp.z > 0) { // CCW
            angleRads += diffAngle;
            travelRadsCCW = std::max(travelRadsCCW,angleRads);
        }
        else if(vecUp.z < 0) { // CW
            angleRads -= diffAngle;
            travelRadsCW = std::min(travelRadsCW,angleRads);
        }
        else { // passes through the center
            is360 = true;
            break;
        }
//        std::cout << "temp: " << angleRads << std::endl;
    }

    double degCCW = travelRadsCCW*180.0/K_PI;
    double degCW = travelRadsCW *180.0/K_PI;

//    std::cout << "degCCW: " << degCCW << ", degCW: " << degCW << std::endl;

    // TODO (comparison to 360 needs to be inexact +/- deg)
    if(is360 || fabs(degCCW)>=360.0 || fabs(degCW)>=360.0) {
        GeoBounds b; b.minLon = -180.0; b.maxLon = 180.0;
        listBounds.push_back(b);
        is360 = true;
    }
    else {
        double startLon = listPLLA[0].lon;

        if(startLon+degCCW > 180.0) {
            double maxLon = startLon+degCCW; CalcValidAngle(maxLon);
            GeoBounds a; a.minLon = -180.0; a.maxLon = maxLon;
            GeoBounds b; b.minLon = startLon+degCW; b.maxLon = 180.0;
            listBounds.push_back(a);
            listBounds.push_back(b);
        }
        else if(startLon+degCW < -180.0) {
            double minLon = startLon+degCW; CalcValidAngle(minLon);
            GeoBounds a; a.minLon = minLon; a.maxLon = 180.0;
            GeoBounds b; b.minLon = -180.0; b.maxLon = startLon+degCCW;
            listBounds.push_back(a);
            listBounds.push_back(b);
        }
        else {
            GeoBounds b;
            b.minLon = startLon+degCW;
            b.maxLon = startLon+degCCW;
            listBounds.push_back(b);
        }
    }

    // Latitude Range

    // We need to address the possibility that the max
    // and min latitude lie on the surface defined by
    // the vertices comprising the spherical polygon

    // formally finding the latitude range for an arbitrary
    // spherical polygon is complex, so we only look at the
    // case where the polygon spans the north or south pole
    // by doing a projected point in polygon test

    // we determine which pole to check by seeing whether
    // the observer (eye) is north or south of the equator

    double critLat = listPLLA[0].lat;
    if(is360)
    {
        // check if the camera is above or below the 'equator'
        critLat = (eye.Dot(Vec3(0,0,1)) >= 0) ? 90 : -90;
        std::cout << "### is360 and critLat @ " << critLat << std::endl;
    }

    // calc min/max for latitude
    double minLat = 185; double maxLat = -185;
    for(size_t i=0; i < listPLLA.size(); i++) {
        minLat = std::min(minLat,listPLLA[i].lat);
        maxLat = std::max(maxLat,listPLLA[i].lat);
    }
    minLat = std::min(minLat,critLat);
    maxLat = std::max(maxLat,critLat);

    // save lat
    for(size_t i=0; i < listBounds.size(); i++) {
        listBounds[i].minLat = minLat;
        listBounds[i].maxLat = maxLat;

// OSRDEBUG << "### Range: " << i;
// OSRDEBUG << "### minLon: " << listBounds[i].minLon
// << ", maxLon: " << listBounds[i].maxLon;
// OSRDEBUG << "### minLat: " << listBounds[i].minLat
// << ", maxLat: " << listBounds[i].maxLat;
    }

    return true;
}


// CalcMinGeoBoundsFromLLAPoly


bool CalcGeoBoundsFromLLAPoly(PointLLA const &camLLA,
                              std::vector<PointLLA> const &listPLLA,
                              std::vector<GeoBounds> &listBounds)
{
//    for(size_t i=0; i < listPLLA.size(); i++)   {
//        std::cout << "INPUT " << i
//                  << ", lon: " << listPLLA[i].lon
//                  << ", lat: " << listPLLA[i].lat
//                  << ", alt: " << listPLLA[i].alt
//                  << std::endl;
//    }

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

//        std::cout << "angleCW: " << angleCW
//                  << ", angleCCW: " << angleCCW << std::endl;

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

//        std::cout << "travelDegs: " << travelDegs
//                  << ", maxTravelCCW: " << maxTravelDegsCCW
//                  << ", maxTravelCW: " << maxTravelDegsCW
//                  << std::endl;
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
            CalcValidAngle(maxLon);

            GeoBounds a,b;
            a.minLon = -180.0; a.maxLon = maxLon;
            b.minLon = startLon-maxTravelDegsCW; b.maxLon = 180.0;
            listBounds.push_back(a);
            listBounds.push_back(b);
        }
        else if(startLon-maxTravelDegsCW < -180.0)   {
            double minLon = startLon-maxTravelDegsCW;
            CalcValidAngle(minLon);

            GeoBounds a,b;
            a.minLon = minLon; a.maxLon = 180.0;
            b.minLon = -180.0; b.maxLon = startLon+maxTravelDegsCCW;
            listBounds.push_back(a);
            listBounds.push_back(b);
        }
        else   {
            GeoBounds b;
            b.minLon = startLon-maxTravelDegsCW;
            b.maxLon = startLon+maxTravelDegsCCW;
            listBounds.push_back(b);
        }

//        // Determine if the camera lies within the LLA
//        // polygon's longitude range
//        bool camLonInRange = false;
//        for(size_t i=0; i < listBounds.size(); i++)   {
//            if(!(camLLA.lon < listBounds[i].minLon ||
//                 camLLA.lon > listBounds[i].maxLon))
//            {
//                camLonInRange = true;
//                break;
//            }
//        }

//        // If the calculated longitude range is on the
//        // opposite side of the camera eye, inverse it.
//        if(!camLonInRange)   {
//            if(listBounds.size() == 1)   {
//                GeoBounds a_inv,b_inv;
//                GeoBounds a = listBounds[0];
//                listBounds.clear();
//                if(a.minLon == -180.0)   {
//                    a_inv.minLon = a.maxLon;
//                    a_inv.maxLon = 180.0;
//                    listBounds.push_back(a_inv);
//                }
//                else if(a.maxLon == 180.0)   {
//                    a_inv.minLon = -180.0;
//                    a_inv.maxLon = a.minLon;
//                    listBounds.push_back(a_inv);
//                }
//                else   {
//                    a_inv.minLon = -180.0;
//                    a_inv.maxLon = a.minLon;
//                    b_inv.minLon = a.maxLon;
//                    b_inv.maxLon = 180.0;
//                    listBounds.push_back(a_inv);
//                    listBounds.push_back(b_inv);
//                }
//            }
//            else   { // listBounds.size() == 2
//                GeoBounds inv;
//                GeoBounds a = listBounds[0];
//                GeoBounds b = listBounds[1];
//                listBounds.clear();

//                if(a.minLon == -180.0)   { // b.maxLon = 180.0
//                    inv.minLon = a.maxLon;
//                    inv.maxLon = b.minLon;
//                }
//                else   { // b.minLon = -180.0, a.maxLon = 180.0
//                    inv.minLon = b.maxLon;
//                    inv.maxLon = a.minLon;
//                }
//                listBounds.push_back(inv);
//            }
//        }
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
        if(camLLA.lat > 0);

        critLat = (camLLA.lat > 0) ? 90 : -90;
        std::cout << "#: is360 and critLat @ " << critLat << std::endl;
    }

    // calc min/max for latitude
    double minLat = 90; double maxLat = -90;
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


//void BuildTilesFromCameraLOD(Vec3 const &eye,
//                             Vec3 const &vpt,
//                             Vec3 const &up,
//                             double const fovy,
//                             double const ar,
//                             double const znear,
//                             double const zfar)
//{
////    std::cout << "SZ LIST BASE TILES BEF/AFT: "
////              << CountBaseTiles();

//    std::vector<double> listMaxDist2(g_LIST_LOD_RANGES[0].level+1,-1.0);
//    for(size_t i=0; i < listMaxDist2.size(); i++)   {
//        for(size_t j=0; j < g_LIST_LOD_RANGES.size(); j++)   {
//            if(g_LIST_LOD_RANGES[j].level == i)   {
//                listMaxDist2[i] = g_LIST_LOD_RANGES[j].max*
//                                  g_LIST_LOD_RANGES[j].max;
//            }
//        }
////        std::cout << i << ": " << listMaxDist2[i] << std::endl;
//    }

//    for(size_t i=0; i < g_LIST_BASE_TILES.size(); i++)   {
//        BuildQuadTilesFromCam(eye,
//                              g_LIST_BASE_TILES[i],
//                              listMaxDist2,
//                              g_LIST_TEMP_TILES,
//                              g_LIST_TEMP_VERTICES);
//    }

//    // cull tile vertices against horizon
//    Vec3 scaledSpaceEye(eye.x / ELL_SEMI_MAJOR,
//                        eye.y / ELL_SEMI_MAJOR,
//                        eye.z / ELL_SEMI_MINOR);

//    CullPointsAgainstHorizon(scaledSpaceEye,g_LIST_BASE_VERTICES);
//    CullPointsAgainstHorizon(scaledSpaceEye,g_LIST_TEMP_VERTICES);

//    // build frustum

//    // calculate the four edge vectors of the view frustum
//    double fovy_rad_bi  = (fovy*K_PI/180.0)/2.0;
//    double dAlongViewPt = cos(fovy_rad_bi);
//    double dAlongUp     = sin(fovy_rad_bi);
//    double dAlongRight  = dAlongUp*ar;

//    Vec3 camRight     = (vpt-eye).Cross(up);
//    Vec3 vAlongUp     = up.Normalized().ScaledBy(dAlongUp);
//    Vec3 vAlongRight  = camRight.Normalized().ScaledBy(dAlongRight);
//    Vec3 vAlongViewPt = (vpt-eye).Normalized().ScaledBy(dAlongViewPt);

//    std::vector<Vec3> listProjVec(9);
//    listProjVec[0] = vAlongViewPt - vAlongUp - vAlongRight;     // bottom left
//    listProjVec[1] = vAlongViewPt - vAlongUp;                   // bottom
//    listProjVec[2] = vAlongViewPt - vAlongUp + vAlongRight;     // bottom right
//    listProjVec[3] = vAlongViewPt + vAlongRight;                // right
//    listProjVec[4] = vAlongViewPt + vAlongUp + vAlongRight;     // top right
//    listProjVec[5] = vAlongViewPt + vAlongUp;                   // top
//    listProjVec[6] = vAlongViewPt + vAlongUp - vAlongRight;     // top left
//    listProjVec[7] = vAlongViewPt - vAlongRight;                // left
//    listProjVec[8] = vAlongViewPt;                              // center

//    double nearDist = 10.0;
//    double farDist  = eye.Magnitude()*2.0;
//    Vec3 nearBL = eye + listProjVec[0].Normalized().ScaledBy(nearDist);
//    Vec3 nearTR = eye + listProjVec[4].Normalized().ScaledBy(nearDist);
//    Vec3 farBL  = eye + listProjVec[0].Normalized().ScaledBy(farDist);

//    // frustum plane normals (these point outside
//    // according to right hand rule)
//    std::vector<Vec3> listFPlaneNorms(6);
//    listFPlaneNorms[0] = listProjVec[0].Cross(listProjVec[2]);  // bottom
//    listFPlaneNorms[1] = listProjVec[2].Cross(listProjVec[4]);  // right
//    listFPlaneNorms[2] = listProjVec[4].Cross(listProjVec[6]);  // top
//    listFPlaneNorms[3] = listProjVec[6].Cross(listProjVec[0]);  // left
//    listFPlaneNorms[4] = vAlongViewPt.Normalized();             // far
//    listFPlaneNorms[5] = listFPlaneNorms[4].ScaledBy(-1.0);     // near

//    // frustum plane points
//    std::vector<Vec3> listFPlanePoints(6);
//    listFPlanePoints[0] = nearBL;           // bottom
//    listFPlanePoints[1] = nearTR;           // right
//    listFPlanePoints[2] = nearTR;           // top
//    listFPlanePoints[3] = nearBL;           // left
//    listFPlanePoints[4] = farBL;            // far
//    listFPlanePoints[5] = nearBL;           // near

//    CullPointsAgainstFrustum(listFPlaneNorms,listFPlanePoints,g_LIST_BASE_VERTICES);
//    CullPointsAgainstFrustum(listFPlaneNorms,listFPlanePoints,g_LIST_TEMP_VERTICES);

////    for(size_t i=0; i < g_LIST_BASE_TILES.size(); i++)   {
////        CullTilesAgainstFrustum(listFPlaneNorms,
////                                listFPlanePoints,
////                                g_LIST_BASE_TILES[i]);
////    }


////    std::cout << "/" << CountBaseTiles() << std::endl;
//}

osg::ref_ptr<osg::Program> g_VERTEX_ATTR_FLAT_SHADER;
osg::ref_ptr<osg::Program> g_VERTEX_ATTR_DIFF_SHADER;

void SetupShaders()
{
    std::string verStr,vShader,fShader;
    verStr = "#version 120\n";    // desktop opengl 2

    g_VERTEX_ATTR_FLAT_SHADER = new osg::Program;
    g_VERTEX_ATTR_FLAT_SHADER->setName("DefaultShader");

    vShader = ReadFileAsString("default_vert.glsl");
    g_VERTEX_ATTR_FLAT_SHADER->addShader(
                new osg::Shader(osg::Shader::VERTEX,verStr+vShader));

    fShader = ReadFileAsString("default_frag.glsl");
    g_VERTEX_ATTR_FLAT_SHADER->addShader(
                new osg::Shader(osg::Shader::FRAGMENT,verStr+fShader));


    g_VERTEX_ATTR_DIFF_SHADER = new osg::Program;
    g_VERTEX_ATTR_DIFF_SHADER->setName("DiffuseShader");

    vShader = ReadFileAsString("diffuse_vert.glsl");
    g_VERTEX_ATTR_DIFF_SHADER->addShader(
                new osg::Shader(osg::Shader::VERTEX,verStr+vShader));

    fShader = ReadFileAsString("diffuse_frag.glsl");
    g_VERTEX_ATTR_DIFF_SHADER->addShader(
                new osg::Shader(osg::Shader::FRAGMENT,verStr+fShader));
}

enum AngleRange
{
    DEG_0_360,
    DEG_180_180
};

double CalcValidAngleDegs(double angle, AngleRange range)
{
    if(range == DEG_0_360)   {
        angle = fmod(angle,360);
        if(angle < 0)
        {   angle += 360;   }

        return angle;
    }

    else   {    // DEG_180_180
        angle = fmod(angle+180,360);
        if(angle < 0)
        {   angle += 360;   }

        return angle-180;
    }
}

void CalcGeoDestination(const PointLLA &pointStart,
                        double bearingDegrees,  // note: CW from North
                        double distanceMeters,
                        PointLLA &pointDest)
{
    // This method was taken from [1] and is originally
    // Copyright 2002-2012 Chris Veness.

    // [1] - Latitude/longitude spherical geodesy formulae & scripts
    //       http://www.movable-type.co.uk/scripts/latlong.html

//    if(distanceMeters > CIR_AV/4)   {
//        // prevent 'wrapping around' by clipping the max distance
//        // to a quarter of the earth's av. circumference
//        distanceMeters = CIR_AV/4;
//    }

    double bearingRad = bearingDegrees * K_PI/180.0;
    double angularDist = distanceMeters / ELL_SEMI_MAJOR;
    double lat1 = pointStart.lat * K_PI/180.0;
    double lon1 = pointStart.lon * K_PI/180.0;
    double cosLat1 = cos(lat1);
    double sinLat1 = sin(lat1);
    double cosAngDist = cos(angularDist);
    double sinAngDist = sin(angularDist);

    pointDest.lat = asin(sinLat1*cosAngDist +
                         cosLat1*sinAngDist*cos(bearingRad));

    pointDest.lon = lon1 + atan2(sin(bearingRad)*sinAngDist*cosLat1,
                                 cosAngDist-sinLat1*sin(pointDest.lat));

    // convert back to degrees
    pointDest.lat *= (180.0/K_PI);
    pointDest.lon *= (180.0/K_PI);

    // normalize lon to +/- 180
    // TODO verify what happens to lon/lat when we
    //      start 'circling' around with extreme
    //      distance values
    pointDest.lon = CalcValidAngleDegs(pointDest.lon,DEG_180_180);
}

osg::Group * BuildGpEarthFromCamera(osg::Camera * camera,
                                    std::vector<LODRange> &listLodRanges)
{
    // [earth group]
    osg::ref_ptr<osg::Group> gpEarth = new osg::Group;
    gpEarth->setName("gpEarth");

    // [earth geode]
    osg::ref_ptr<osg::Geode> gdEarth = new osg::Geode;
    osg::StateSet * ss = gpEarth->getOrCreateStateSet();
    ss->setAttributeAndModes(g_VERTEX_ATTR_FLAT_SHADER);
//    ss->setRenderBinDetails(100,"RenderBin");
//    ss->setMode(GL_DEPTH_TEST,osg::StateAttribute::OFF);

    // [earth geometry]
    std::vector<VExtVertex> buildListVx;
    std::vector<size_t> buildListIx;
    BuildEarthSurfaceGeometry(-180,-90,180,90,16,16,
                              buildListVx,buildListIx);

    osg::ref_ptr<osg::Vec3dArray> listVx = new osg::Vec3dArray;
    osg::ref_ptr<osg::Vec3Array> listNx = new osg::Vec3Array;
    osg::ref_ptr<osg::Vec4Array>  listCx = new osg::Vec4Array;
    osg::ref_ptr<osg::DrawElementsUInt> listIx =
            new osg::DrawElementsUInt(GL_TRIANGLES);

    for(size_t i=0; i < buildListVx.size(); i++)   {
        osg::Vec3d vx(buildListVx[i].ecef.x,
                      buildListVx[i].ecef.y,
                      buildListVx[i].ecef.z);
        osg::Vec3 nx = vx; nx.normalize();
        listVx->push_back(vx);
        listNx->push_back(nx);
        listCx->push_back(osg::Vec4(0.16,0.16,0.16,1.0));
    }
    for(size_t i=0; i < buildListIx.size(); i++)   {
        listIx->push_back(buildListIx[i]);
    }

    osg::ref_ptr<osg::Geometry> gmEarth = new osg::Geometry;
    gmEarth->setVertexArray(listVx);
    gmEarth->setNormalArray(listNx);
    gmEarth->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);
    gmEarth->setColorArray(listCx);
    gmEarth->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
    gmEarth->addPrimitiveSet(listIx);

    gdEarth->addDrawable(gmEarth);
    gpEarth->addChild(gdEarth);

    if(camera)   {
        std::vector<std::vector<GeoBounds> >
                listListBounds(listLodRanges.size());

        std::vector<std::vector<GeoBounds> >
                listTempBounds(listLodRanges.size());

        // Check if the current LOD range intersects with
        // the surface of the Earth at all

        // We do a rough test by converting an LOD range
        // sphere and the Earth into circles and testing
        // for intersection.

        // The LOD circle center can be placed arbitrarily as
        // long as the distance between centers reflects the
        // distance between Earth's center and the camera eye.

        osg::Vec3d veye,vvpt,vup;
        camera->getViewMatrixAsLookAt(veye,vvpt,vup);

        Vec3 xsecNear,xsecFar,camOnSurf;
        Vec3 eye(veye.x(),veye.y(),veye.z());
        PointLLA camLLA = ConvECEFToLLA(eye);
        CalcRayEarthIntersection(eye,eye.ScaledBy(-1.0),xsecNear,xsecFar);
        camOnSurf = xsecNear;

        for(size_t i=0; i < listLodRanges.size(); i++)   {
            //
            listLodRanges[i].active = false;

            // Check if the current LOD range intersects with
            // the surface of the Earth at all

            // We do a rough test by converting an LOD range
            // sphere and the Earth into circles and testing
            // for intersection.

            // The LOD circle center can be placed arbitrarily as
            // long as the distance between centers reflects the
            // distance between Earth's center and the camera eye.

            // OSRDEBUG << "INFO: Active Extent: "
            // << listLODRanges[i].second;

            double d2_earthRad = camOnSurf.Magnitude();
            Vec2 d2_earthCenter(0,0);

            double d2_lodRad = listLodRanges[i].max;
            Vec2 d2_lodCenter(eye.Magnitude()*-1.0,0);

            IntersectionType xsecType;
            std::vector<Vec2> d2_xsecPoints;
            xsecType = CalcCircleCircleIntersection(d2_earthCenter,d2_earthRad,
                                                    d2_lodCenter,d2_lodRad,
                                                    d2_xsecPoints);

            if(d2_xsecPoints.size() < 2)   {
                // If there are less than two intersection points
                // between the LOD and Earth circles, check if the
                // LOD circle compasses the earth

                if(xsecType == XSEC_CONTAINED)   {
                    GeoBounds b;
                    b.minLon = -180; b.maxLon = 180;
                    b.minLat = -90;  b.maxLat = 90;
                    std::cout << i << ": XSEC_CONTAINED" << std::endl;
                    listTempBounds[i].push_back(b);
                    listListBounds[i].push_back(b);
                    listLodRanges[i].active = true;

                    break;  // since larger LOD ranges will also
                    // fully contain the Earth
                }
                else
                {   // the LOD sphere doesn't intersect the
                    // Earth, so ignore this range
                    //                    OSRDEBUG << "WARN: No overlap for LOD range: "
                    //                             << listLodRanges[i].max;
                    continue;
                }
            }

            // To determine an equivalent LOD distance on the Earth's
            // surface, we use half the arc length on the Earth circle
            // defined by the LOD / Earth overlap
            double halfArcLength = 0;

            // get the angle of both POIs on the Earth's surface
            double xsAngle1Deg = atan2(d2_xsecPoints[0].y - d2_earthCenter.y,
                    d2_xsecPoints[0].x - d2_earthCenter.x);

            double xsAngle2Deg = atan2(d2_xsecPoints[1].y - d2_earthCenter.y,
                    d2_xsecPoints[1].x - d2_earthCenter.x);

            // convert to degrees,0_360
            xsAngle1Deg = CalcValidAngleDegs(xsAngle1Deg*180.0/K_PI,DEG_0_360);
            xsAngle2Deg = CalcValidAngleDegs(xsAngle2Deg*180.0/K_PI,DEG_0_360);

            double xsAngleMidRad = (xsAngle1Deg+xsAngle2Deg)/2.0 * K_PI/180.0;
            Vec2 xsPointMid(d2_earthCenter.x + d2_earthRad*cos(xsAngleMidRad),
                            d2_earthCenter.y + d2_earthRad*sin(xsAngleMidRad));

            // ensure this is the arc length within the LOD circle
            if((xsPointMid-d2_lodCenter).Magnitude() < d2_lodRad)
            {   // arc is within LOD circle
                double arcAngleDegs = fabs(xsAngle2Deg-xsAngle1Deg);
                halfArcLength = 2*K_PI*d2_earthRad * (arcAngleDegs/360.0)/2.0;
            }
            else
            {   // arc is outside LOD circle, use the inverse arc
                double arcAngleDegs = 360.0 - fabs(xsAngle2Deg-xsAngle1Deg);
                halfArcLength = 2*K_PI*d2_earthRad * (arcAngleDegs/360.0)/2.0;
            }

            // Limit distance using a rough est of horizon
            halfArcLength = std::min(CIR_AV_FOURTH*0.8,halfArcLength);

            // Use the LOD distance to create an lla bounding box
            std::vector<PointLLA> listRangeLLA(4);
            CalcGeoDestination(camLLA,0,halfArcLength,listRangeLLA[0]);     // north
            CalcGeoDestination(camLLA,90,halfArcLength,listRangeLLA[1]);    // east
            CalcGeoDestination(camLLA,180,halfArcLength,listRangeLLA[2]);   // south
            CalcGeoDestination(camLLA,270,halfArcLength,listRangeLLA[3]);   // west

            CalcGeoBoundsFromLLAPoly(eye,listRangeLLA,listTempBounds[i]);
            CalcGeoBoundsFromLLAPoly(camLLA,listRangeLLA,listListBounds[i]);
            listLodRanges[i].active = true;
        }

        for(size_t i=0; i < listListBounds.size(); i++)   {
            for(size_t j=0; j < listTempBounds[i].size(); j++)   {
                GeoBounds const &a = listTempBounds[i][j];
                std::cout << "a: " << i
                          << ", minLon: " << a.minLon
                          << ", maxLon: " << a.maxLon
                          << ", minLat: " << a.minLat
                          << ", maxLat: " << a.maxLat
                          << std::endl;
            }
            for(size_t j=0; j < listListBounds[i].size(); j++)   {
                GeoBounds const &b = listListBounds[i][j];

                std::cout << "b: " << i
                          << ", minLon: " << b.minLon
                          << ", maxLon: " << b.maxLon
                          << ", minLat: " << b.minLat
                          << ", maxLat: " << b.maxLat
                          << std::endl;

                // [surface geometry] // IMPROVE NUMSEG BASED ON SURFACE RANGE
                size_t numseg = 16;
                buildListVx.clear(); buildListIx.clear();
                BuildEarthSurfaceGeometry(b.minLon,b.minLat,b.maxLon,b.maxLat,
                                          numseg,numseg,buildListVx,buildListIx,
                                          (listListBounds.size()-i)*30000);

//                BuildEarthSurfaceGeometry(-180,-90,180,90,8,8,
//                                          buildListVx,buildListIx);

                osg::ref_ptr<osg::Vec3dArray> gmVx = new osg::Vec3dArray;
                osg::ref_ptr<osg::Vec3Array> gmNx = new osg::Vec3Array;
                osg::ref_ptr<osg::Vec4Array> gmCx = new osg::Vec4Array;
                osg::ref_ptr<osg::DrawElementsUInt> gmIx =
                        new osg::DrawElementsUInt(GL_TRIANGLES);

                for(size_t k=0; k < buildListVx.size(); k++)   {
                    osg::Vec3d vx(buildListVx[k].ecef.x,
                                  buildListVx[k].ecef.y,
                                  buildListVx[k].ecef.z);
                    osg::Vec3 nx = vx;
                    nx.normalize();

                    osg::Vec4 cx =CalcRainbowGradient(
                                double(i)/listListBounds.size());

                    gmVx->push_back(vx);
                    gmNx->push_back(nx);
                    gmCx->push_back(cx);
                }
                for(size_t k=0; k < buildListIx.size(); k++)   {
                    gmIx->push_back(buildListIx[k]);
                }

                osg::ref_ptr<osg::Geometry> gmLodSurface = new osg::Geometry;
                gmLodSurface->setVertexArray(gmVx);
                gmLodSurface->setNormalArray(gmNx);
                gmLodSurface->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);
                gmLodSurface->setColorArray(gmCx);
                gmLodSurface->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
                gmLodSurface->addPrimitiveSet(gmIx);

                osg::ref_ptr<osg::Geode> gdLodSurface = new osg::Geode;
                ss = gdLodSurface->getOrCreateStateSet();
                ss->setAttributeAndModes(g_VERTEX_ATTR_FLAT_SHADER);
//                ss->setMode(GL_DEPTH_TEST,osg::StateAttribute::OFF);
//                ss->setRenderBinDetails(100-i,"RenderBin");
                gdLodSurface->addDrawable(gmLodSurface);
                gpEarth->addChild(gdLodSurface);
            }
        }
    }


    return gpEarth.release();
}

//osg::Geode * BuildGdEarthFromCamera(osg::Camera * camera)
//{

//    if(camera)   {
//        // generate the tiles
//        osg::Vec3d veye,vvpt,vup;
//        double fovy,ar,znear,zfar;
//        camera->getViewMatrixAsLookAt(veye,vvpt,vup);
//        camera->getProjectionMatrixAsPerspective(fovy,ar,znear,zfar);
//        BuildTilesFromCameraLOD(Vec3(veye.x(),veye.y(),veye.z()),
//                                Vec3(vvpt.x(),vvpt.y(),vvpt.z()),
//                                Vec3(vup.x(),vup.y(),vup.z()),
//                                fovy,ar,znear,zfar);
//    }

//    std::vector<osg::Vec3d> listTileLineVx;
//    for(size_t i=0; i < g_LIST_BASE_TILES.size(); i++)   {
//        BuildListVxFromTiles(g_LIST_BASE_TILES[i],listTileLineVx);
//    }


//    // [earth geometry]
//    osg::ref_ptr<osg::Vec3dArray> listVx = new osg::Vec3dArray;
//    osg::ref_ptr<osg::Vec4Array>  listCx = new osg::Vec4Array;
//    for(size_t i=0; i < listTileLineVx.size(); i++)   {
//        listVx->push_back(listTileLineVx[i]);
//        listCx->push_back(osg::Vec4(0.2,0.2,0.2,1.0));
//    }

//    osg::ref_ptr<osg::Geometry> gmEarth = new osg::Geometry;
//    gmEarth->setVertexArray(listVx);
//    gmEarth->setColorArray(listCx);
//    gmEarth->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
//    gmEarth->addPrimitiveSet(new osg::DrawArrays(GL_LINES,0,listVx->size()));

//    // [earth geode]
//    osg::ref_ptr<osg::Geode> gdEarth = new osg::Geode;
//    gdEarth->setName("gdEarth");
//    gdEarth->addDrawable(gmEarth);

//    osg::StateSet * ss = gdEarth->getOrCreateStateSet();
//    ss->setAttributeAndModes(g_VERTEX_ATTR_FLAT_SHADER);


//    // clean up
//    for(size_t i=0; i < g_LIST_TEMP_TILES.size(); i++)   {
//        delete g_LIST_TEMP_TILES[i];
//    } g_LIST_TEMP_TILES.clear();

//    for(size_t i=0; i < g_LIST_TEMP_VERTICES.size(); i++)   {
//        delete g_LIST_TEMP_VERTICES[i];
//    } g_LIST_TEMP_VERTICES.clear();

//    for(size_t i=0; i < g_LIST_BASE_TILES.size(); i++)   {
//        g_LIST_BASE_TILES[i]->subtileBL = NULL;
//        g_LIST_BASE_TILES[i]->subtileBR = NULL;
//        g_LIST_BASE_TILES[i]->subtileTR = NULL;
//        g_LIST_BASE_TILES[i]->subtileTL = NULL;
//    }

//    return gdEarth.release();
//}

osg::Group * BuildFrustumFromCamera(osg::Camera * camera,
                                    std::vector<LODRange> const &listLodRanges)
{

    if(!camera)   {
        osg::ref_ptr<osg::Group> gpCamera = new osg::Group;
        gpCamera->setName("gpCamera");
        return gpCamera.release();
    }

    osg::Matrixd proj = camera->getProjectionMatrix();
    osg::Matrixd mv = camera->getViewMatrix();

    osg::Vec3d eye,up,vpt;
    camera->getViewMatrixAsLookAt(eye,up,vpt);

    // Get near and far from the Projection matrix.
//    const double near = proj(3,2) / (proj(2,2)-1.0);
//    const double far = proj(3,2) / (1.0+proj(2,2));

    double near = 500;
    double far = ELL_SEMI_MAJOR*4;

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
    v->resize( 9 );
    (*v)[0].set( 0., 0., 0. );
    (*v)[1].set( nLeft, nBottom, -near );
    (*v)[2].set( nRight, nBottom, -near );
    (*v)[3].set( nRight, nTop, -near );
    (*v)[4].set( nLeft, nTop, -near );
    (*v)[5].set( fLeft, fBottom, -far );
    (*v)[6].set( fRight, fBottom, -far );
    (*v)[7].set( fRight, fTop, -far );
    (*v)[8].set( fLeft, fTop, -far );

    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
    geom->setUseDisplayList( false );
    geom->setVertexArray( v );

    osg::ref_ptr<osg::Vec4Array> c = new osg::Vec4Array;
    for(size_t i=0; i < v->size(); i++)   {
        c->push_back(osg::Vec4(0.35,0.35,0.35,1.0));
    }
    geom->setColorArray(c);
    geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

    GLushort idxLines[8] = {
        0, 5, 0, 6, 0, 7, 0, 8 };
    GLushort idxLoops0[4] = {
        1, 2, 3, 4 };
    GLushort idxLoops1[4] = {
        5, 6, 7, 8 };
    geom->addPrimitiveSet( new osg::DrawElementsUShort( osg::PrimitiveSet::LINES, 8, idxLines ) );
    geom->addPrimitiveSet( new osg::DrawElementsUShort( osg::PrimitiveSet::LINE_LOOP, 4, idxLoops0 ) );
    geom->addPrimitiveSet( new osg::DrawElementsUShort( osg::PrimitiveSet::LINE_LOOP, 4, idxLoops1 ) );

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->addDrawable(geom);

    osg::ref_ptr<osg::Geode> gdLodRanges = new osg::Geode;

    osg::Vec3d eye_normalized = eye;
    eye_normalized.normalize();

    osg::ref_ptr<osg::AutoTransform> axfLodRanges =
            new osg::AutoTransform;
    axfLodRanges->setAutoRotateMode(osg::AutoTransform::ROTATE_TO_SCREEN);
    axfLodRanges->setNormal(osg::Vec3d(0,0,1));
    axfLodRanges->setPosition(eye);
    axfLodRanges->addChild(gdLodRanges);

    // lod range circles
    for(size_t lod=0; lod < listLodRanges.size(); lod++)
    {
        double r = listLodRanges[lod].max;
        size_t numsides = 24;
        double segmentAngle = (2*K_PI)/numsides;

        osg::ref_ptr<osg::Vec3dArray> listVx =
                new osg::Vec3dArray;

        osg::ref_ptr<osg::Vec4Array> listCx =
                new osg::Vec4Array;

        for(size_t v=0; v < numsides; v++)   {
            double vxAngle = segmentAngle*v;
            osg::Vec3d vx(r*cos(vxAngle),
                          r*sin(vxAngle),
                          0.0);

            listVx->push_back(vx);
            listCx->push_back(CalcRainbowGradient(
                double(lod)/listLodRanges.size()));
        }

        osg::ref_ptr<osg::Geometry> gmLodRange =
                new osg::Geometry;

        gmLodRange->setVertexArray(listVx);
        gmLodRange->setColorArray(listCx);
        gmLodRange->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
        gmLodRange->addPrimitiveSet(new osg::DrawArrays(GL_LINE_LOOP,0,listVx->size()));

        gdLodRanges->addDrawable(gmLodRange);
    }

    // Create parent MatrixTransform to transform the view volume by
    // the inverse ModelView matrix.
    osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform;
    mt->setMatrix( osg::Matrixd::inverse( mv ) );
    mt->addChild( geode );

    osg::ref_ptr<osg::Group> gpCamera = new osg::Group;
    gpCamera->setName("gpCamera");
    gpCamera->addChild(mt);
    gpCamera->addChild(axfLodRanges);

    osg::StateSet * ss = gpCamera->getOrCreateStateSet();
    ss->setAttributeAndModes(g_VERTEX_ATTR_FLAT_SHADER);

    return gpCamera.release();
}


osg::MatrixTransform * BuildFrustumFromCamera(osg::Camera * camera)
{
    osg::Matrixd proj = camera->getProjectionMatrix();
    osg::Matrixd mv = camera->getViewMatrix();

    // Get near and far from the Projection matrix.
//    const double near = proj(3,2) / (proj(2,2)-1.0);
//    const double far = proj(3,2) / (1.0+proj(2,2));

    double near = 500;
    double far = ELL_SEMI_MAJOR*4;

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
    v->resize( 9 );
    (*v)[0].set( 0., 0., 0. );
    (*v)[1].set( nLeft, nBottom, -near );
    (*v)[2].set( nRight, nBottom, -near );
    (*v)[3].set( nRight, nTop, -near );
    (*v)[4].set( nLeft, nTop, -near );
    (*v)[5].set( fLeft, fBottom, -far );
    (*v)[6].set( fRight, fBottom, -far );
    (*v)[7].set( fRight, fTop, -far );
    (*v)[8].set( fLeft, fTop, -far );

    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
    geom->setUseDisplayList( false );
    geom->setVertexArray( v );

    osg::ref_ptr<osg::Vec4Array> c = new osg::Vec4Array;
    for(size_t i=0; i < v->size(); i++)   {
        c->push_back(osg::Vec4(0.6,0.6,0.6,1.0));
    }
    geom->setColorArray(c);
    geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

    GLushort idxLines[8] = {
        0, 5, 0, 6, 0, 7, 0, 8 };
    GLushort idxLoops0[4] = {
        1, 2, 3, 4 };
    GLushort idxLoops1[4] = {
        5, 6, 7, 8 };
    geom->addPrimitiveSet( new osg::DrawElementsUShort( osg::PrimitiveSet::LINES, 8, idxLines ) );
    geom->addPrimitiveSet( new osg::DrawElementsUShort( osg::PrimitiveSet::LINE_LOOP, 4, idxLoops0 ) );
    geom->addPrimitiveSet( new osg::DrawElementsUShort( osg::PrimitiveSet::LINE_LOOP, 4, idxLoops1 ) );

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->addDrawable(geom);

    osg::StateSet * ss = geode->getOrCreateStateSet();
    ss->setAttributeAndModes(g_VERTEX_ATTR_FLAT_SHADER);

    // Create parent MatrixTransform to transform the view volume by
    // the inverse ModelView matrix.
    osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform;
    mt->setMatrix( osg::Matrixd::inverse( mv ) );
    mt->addChild( geode );
    mt->setName("gpCamera");

    return mt.release();
}

osg::Geode * BuildOctahedron(osg::Vec3d const &center,
                             osg::Vec4 const &color,
                             double const size)
{
    double ppushsz = size/2;
    double npushsz = ppushsz*-1.0;
    osg::ref_ptr<osg::Vec3dArray> gmListVx = new osg::Vec3dArray;
    gmListVx->push_back(center + osg::Vec3(ppushsz,0,0));     // 0 +x
    gmListVx->push_back(center + osg::Vec3(0,ppushsz,0));     // 1 +y
    gmListVx->push_back(center + osg::Vec3(0,0,ppushsz));     // 2 +z
    gmListVx->push_back(center + osg::Vec3(npushsz,0,0));     // 3 -x
    gmListVx->push_back(center + osg::Vec3(0,npushsz,0));     // 4 -y
    gmListVx->push_back(center + osg::Vec3(0,0,npushsz));     // 5 -z

    osg::ref_ptr<osg::Vec4Array> gmListCx = new osg::Vec4Array;
    gmListCx->push_back(color);
    gmListCx->push_back(color);
    gmListCx->push_back(color);
    gmListCx->push_back(color);
    gmListCx->push_back(color);
    gmListCx->push_back(color);

    osg::ref_ptr<osg::DrawElementsUByte> gmListIx =
            new osg::DrawElementsUByte(GL_TRIANGLES);
    gmListIx->push_back(0);
    gmListIx->push_back(1);
    gmListIx->push_back(2);

    gmListIx->push_back(1);
    gmListIx->push_back(3);
    gmListIx->push_back(2);

    gmListIx->push_back(3);
    gmListIx->push_back(4);
    gmListIx->push_back(2);

    gmListIx->push_back(4);
    gmListIx->push_back(0);
    gmListIx->push_back(2);

    gmListIx->push_back(1);
    gmListIx->push_back(0);
    gmListIx->push_back(5);

    gmListIx->push_back(3);
    gmListIx->push_back(1);
    gmListIx->push_back(5);

    gmListIx->push_back(4);
    gmListIx->push_back(3);
    gmListIx->push_back(5);

    gmListIx->push_back(0);
    gmListIx->push_back(4);
    gmListIx->push_back(5);

    osg::ref_ptr<osg::Geometry> gmOct = new osg::Geometry;
    gmOct->setVertexArray(gmListVx);
    gmOct->setColorArray(gmListCx);
    gmOct->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
    gmOct->addPrimitiveSet(gmListIx);

    osg::ref_ptr<osg::Geode> gdOct = new osg::Geode;
    gdOct->addDrawable(gmOct);

    osg::StateSet * ss = gdOct->getOrCreateStateSet();
    ss->setAttributeAndModes(g_VERTEX_ATTR_FLAT_SHADER);

    return gdOct.release();
}

//bool CalcGeoBoundsFromLLAPoly(Vec3 eye,
//                   std::vector<PointLLA> const &listPLLA,
//                   std::vector<GeoBounds> &listBounds)
//{
//    listBounds.clear();
//    if(listPLLA.size() < 3)   {
//        std::cout << "WARN: CalcGeoBounds: "
//                    "Insufficient coords ("
//                 << listPLLA.size() << ")" << std::endl;
//        return false;
//    }

//    // Longitude Range

//    // Determine the longitude range of listPLLA by converting
//    // longitudes to 2d cartesian (x,y) and 'walking' along the
//    // polygon, keeping track of CW/CCW distance traveled

//    std::vector<double> listLonRads(listPLLA.size());
//    std::vector<Vec2> listVxLon(listPLLA.size());
//    for(size_t i=0; i < listPLLA.size(); i++)   {
//        double angleRads = listPLLA[i].lon*K_PI/180.0;
//        listLonRads[i] = angleRads;
//        listVxLon[i].x = cos(angleRads);
//        listVxLon[i].y = sin(angleRads);
//    }

//    // walk along the polygon keeping track of max/min
//    // angles; direction is relative to (x,y) = (0,0)
//    bool is360 = false;
//    Vec3 vecCenter(0,0,0);
//    listVxLon.push_back(listVxLon[0]);      // wrap around
//    listLonRads.push_back(listLonRads[0]);  // wrap around

//    double diffAngle = 0;
//    double angleRads = 0;
//    double travelRadsCW = 0;
//    double travelRadsCCW = 0;

//    for(size_t i=1; i < listVxLon.size(); i++)
//    {
//        diffAngle = listLonRads[i]-listLonRads[i-1];

//        if(diffAngle == 0)              {
//            // adjacent points w/ same lon
//            continue;
//        }
//        else if(diffAngle > K_PI)       {
//            diffAngle -= (2*K_PI);
//        }
//        else if(diffAngle < K_PI*-1)    {
//            diffAngle += (2*K_PI);
//        }
//        diffAngle = fabs(diffAngle);


//        Vec3 vecLon(listVxLon[i-1].x,listVxLon[i-1].y,0);
//        Vec3 vecDirn(listVxLon[i].x-vecLon.x,
//                     listVxLon[i].y-vecLon.y,0);

//        if(vecDirn.x == 0 && vecDirn.y == 0)   {
//            // adjacent points w/ same longitude
//            continue;
//        }

//        // using the right-hand rule, upVec determines
//        // whether we're traversing the polygon edge CW
//        // or CCW wrt to the center
//        Vec3 vecUp = (vecLon-vecCenter).Cross(vecDirn);
//        if(vecUp.z > 0)         {       // CCW
//            angleRads += diffAngle;
//            travelRadsCCW = std::max(travelRadsCCW,angleRads);
//        }
//        else if(vecUp.z < 0)    {       // CW
//            angleRads -= diffAngle;
//            travelRadsCW = std::min(travelRadsCW,angleRads);
//        }
//        else   {    // passes through the center
//            is360 = true;
//            break;
//        }
//        std::cout << "temp: " << angleRads << std::endl;
//    }

//    double degCCW = travelRadsCCW*180.0/K_PI;
//    double degCW  = travelRadsCW *180.0/K_PI;

//    std::cout << "degCCW: " << degCCW << ", degCW: " << degCW << std::endl;

//    // TODO (comparison to 360 needs to be inexact +/- deg)
//    if(is360 || fabs(degCCW)>=360.0 || fabs(degCW)>=360.0)   {
//        GeoBounds b; b.minLon = -180.0; b.maxLon = 180.0;
//        listBounds.push_back(b);
//        is360 = true;
//    }
//    else   {
//        double startLon = listPLLA[0].lon;

//        if(startLon+degCCW > 180.0)   {
//            double maxLon = startLon+degCCW; CalcValidAngle(maxLon);
//            GeoBounds a; a.minLon = -180.0; a.maxLon = maxLon;
//            GeoBounds b; b.minLon = startLon+degCW; b.maxLon = 180.0;
//            listBounds.push_back(a);
//            listBounds.push_back(b);
//        }
//        else if(startLon+degCW < -180.0)   {
//            double minLon = startLon+degCW; CalcValidAngle(minLon);
//            GeoBounds a; a.minLon = minLon; a.maxLon = 180.0;
//            GeoBounds b; b.minLon = -180.0; b.maxLon = startLon+degCCW;
//            listBounds.push_back(a);
//            listBounds.push_back(b);
//        }
//        else   {
//            GeoBounds b;
//            b.minLon = startLon+degCW;
//            b.maxLon = startLon+degCCW;
//            listBounds.push_back(b);
//        }
//    }

//    // Latitude Range

//    // We need to address the possibility that the max
//    // and min latitude lie on the surface defined by
//    // the vertices comprising the spherical polygon

//    // formally finding the latitude range for an arbitrary
//    // spherical polygon is complex, so we only look at the
//    // case where the polygon spans the north or south pole
//    // by doing a projected point in polygon test

//    // we determine which pole to check by seeing whether
//    // the observer (eye) is north or south of the equator

//    double critLat = listPLLA[0].lat;
//    if(is360)
//    {
//        // check if the camera is above or below the 'equator'
//        critLat = (eye.Dot(Vec3(0,0,1)) >= 0) ? 90 : -90;
//        std::cout << "### is360 and critLat @ " << critLat << std::endl;
//    }

//    // calc min/max for latitude
//    double minLat = 185; double maxLat = -185;
//    for(size_t i=0; i < listPLLA.size(); i++)   {
//        minLat = std::min(minLat,listPLLA[i].lat);
//        maxLat = std::max(maxLat,listPLLA[i].lat);
//    }
//    minLat = std::min(minLat,critLat);
//    maxLat = std::max(maxLat,critLat);

//    // save lat
//    for(size_t i=0; i < listBounds.size(); i++)   {
//        listBounds[i].minLat = minLat;
//        listBounds[i].maxLat = maxLat;

////        OSRDEBUG << "### Range: " << i;
////        OSRDEBUG << "### minLon: " << listBounds[i].minLon
////                 << ", maxLon: " << listBounds[i].maxLon;
////        OSRDEBUG << "### minLat: " << listBounds[i].minLat
////                 << ", maxLat: " << listBounds[i].maxLat;
//    }

//    return true;
//}
