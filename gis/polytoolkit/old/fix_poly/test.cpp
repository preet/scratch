// STL
#include <iostream>
#include <iomanip>
#include <cstring>
#include <fstream>
#include <stack>
#include <vector>
#include <set>

// geom defs
#include "Vec2.hpp"
#include "Vec3.hpp"

//
#include <ogrsf_frmts.h>

// PI!
#define K_PI 3.141592653589

// epsilon
#define K_EPS 1E-11

// WGS84 ellipsoid parameters
// (http://en.wikipedia.org/wiki/WGS_84)
#define ELL_SEMI_MAJOR 6378137.0            // meters
#define ELL_SEMI_MAJOR_EXP2 40680631590769

#define ELL_SEMI_MINOR 6356752.3142         // meters
#define ELL_SEMI_MINOR_EXP2 40408299984087.1

#define ELL_F 1/298.257223563
#define ELL_ECC_EXP2 6.69437999014e-3
#define ELL_ECC2_EXP2 6.73949674228e-3

struct Tri
{
    Vec2 A;
    Vec2 B;
    Vec2 C;

    double len_a;
    double len_b;
    double len_c;
};

struct TriangleMesh
{
    std::vector<Vec3> listVertices;
    std::vector<unsigned int> listIdxs;
};

class PointLLA
{
public:
    PointLLA() :
        lon(0),lat(0),alt(0) {}

    PointLLA(double myLat, double myLon) :
        lat(myLat),lon(myLon),alt(0) {}

    PointLLA(double myLat, double myLon, double myAlt) :
        lon(myLon),lat(myLat),alt(myAlt) {}

    double lon;
    double lat;
    double alt;
};

Vec3 convLLAToECEF(const PointLLA &pointLLA)
{
    Vec3 pointECEF;

    // remember to convert deg->rad
    double sinLat = sin(pointLLA.lat * K_PI/180.0f);
    double sinLon = sin(pointLLA.lon * K_PI/180.0f);
    double cosLat = cos(pointLLA.lat * K_PI/180.0f);
    double cosLon = cos(pointLLA.lon * K_PI/180.0f);

    // v = radius of curvature (meters)
    double v = ELL_SEMI_MAJOR / (sqrt(1-(ELL_ECC_EXP2*sinLat*sinLat)));
    pointECEF.x = (v + pointLLA.alt) * cosLat * cosLon;
    pointECEF.y = (v + pointLLA.alt) * cosLat * sinLon;
    pointECEF.z = ((1-ELL_ECC_EXP2)*v + pointLLA.alt)*sinLat;

    return pointECEF;
}

int calcWindingNumber(Vec2 checkPt,std::vector<Vec2> polyContour)
{
    // expect polyContour[first] == polyContour[last]
    // winding number = 0 if checkPt is not inside polyContour
    // winding number = n > 0 if polyContour winds around checkPt 'n' times CCW
    // winding number = n < 0 if polyContour winds around checkPt 'n' times CW
    double windingNum = 0;

    // see if the polygon crosses the antemeridian and
    // adjust for a latitude/longitude discontinuity if it does

    // get longitude extents
    double minLon =  200;
    double maxLon = -200;
    for(int i=0; i < polyContour.size(); i++)
    {
        minLon = std::min(minLon,polyContour[i].x);
        maxLon = std::max(maxLon,polyContour[i].x);
    }

    if(maxLon-minLon > 270)
    {   // assume poly spans antemeridian
        for(int i=0; i < polyContour.size(); i++)
        {
            if(polyContour[i].x < 0)
            {   polyContour[i].x += 360.0;   }
        }
        if(checkPt.x < 0)
        {   checkPt.x += 360.0;   }
    }

    for(int i=0; i < polyContour.size(); i++)
    {
        std::cout << "#" << polyContour[i].x << "," << polyContour[i].y << "\n";
    }

    // center checkPt at (0,0) for ease of use
    // and shift the polygon accordingly
    for(int i=0; i < polyContour.size(); i++)
    {   polyContour[i] = polyContour[i]-checkPt;   }

    // now we walk along the polygon's edges; every
    // time a polygon edge crosses the positive x
    // from below, increment the winding number;
    // every time the positive x axis is crossed
    // from above, decrement the winding number
    for(int i=1; i < polyContour.size(); i++)
    {
        double xPrev = polyContour[i-1].x;
        double yPrev = polyContour[i-1].y;
        double xNext = polyContour[i].x;
        double yNext = polyContour[i].y;

        if(yPrev*yNext < 0) // the edge crosses the x axis
        {
            // r is the x coord of intersection between edge and x axis
            double r = xPrev + (yPrev*(xNext-xPrev))/(yPrev-yNext);

            if(r > 0)   // edge crosses the positive x axis!
            {
                if(yPrev < 0)   // edge crosses from below
                {   windingNum += 1;   }
                else            // edge crosses from above
                {   windingNum -= 1;   }
            }
        }
        else if(yPrev == 0 && xPrev > 0)
        {   // edge crosses from +ve x axis to above or below
            if(yNext > 0)
            {   windingNum += 0.5;   }
            else
            {   windingNum -= 0.5;   }
        }
        else if(yNext == 0 && xNext > 0)
        {   // edge crosses from above or below to +ve x axis
            if(yPrev < 0)
            {   windingNum += 0.5;   }
            else
            {   windingNum -= 0.5;   }
        }
    }

    // I don't think that you can ever end up
    // with half winding numbers, but just incase
    if(windingNum == -0.5)
    {   return -1;   }
    else if(windingNum == 0.5)
    {   return 1;   }
    else
    {   return int(windingNum);   }
}


int main(int argc, const char *argv[])
{
    std::vector<Vec2> listPolyline;
//    listPolyline.push_back(Vec2(7.90419,54.1804));
//    listPolyline.push_back(Vec2(7.90065,54.1907));
//    listPolyline.push_back(Vec2(7.89454,54.1946));
//    listPolyline.push_back(Vec2(7.87989,54.1939));
//    listPolyline.push_back(Vec2(7.88587,54.1907));
//    listPolyline.push_back(Vec2(7.89039,54.1876));
//    listPolyline.push_back(Vec2(7.89357,54.1831));
//    listPolyline.push_back(Vec2(7.89772,54.1823));
//    listPolyline.push_back(Vec2(7.89918,54.1782));
//    listPolyline.push_back(Vec2(7.90419,54.1804));

//    Vec2 checkPt(7.89379,54.1864);


    char * inputWkt = "POLYGON ((-151.430500941964908 59.612197223123388,-151.425955649633494 59.610785223659207,-151.410513755005809 59.607733466026644,-151.430500941964908 59.612197223123388))";
    OGRGeometry *myGeom;
    OGRGeometryFactory::createFromWkt(&inputWkt, NULL, &myGeom);
    OGRPolygon *singlePoly = (OGRPolygon*)myGeom;
    OGRLinearRing* outerRing = singlePoly->getExteriorRing();

    std::vector<Vec2> listOuterRingPts(outerRing->getNumPoints());
    for(int j=0; j < listOuterRingPts.size(); j++)   {
        OGRPoint *myPt = new OGRPoint; outerRing->getPoint(j,myPt);
        listOuterRingPts[j] = Vec2(myPt->getX(),myPt->getY());
        delete myPt;
    }

    Vec2 checkPt(-150.723,60.1237);
    std::cout << "Winding Number of checkPt is " << calcWindingNumber(checkPt,listOuterRingPts) <<"\n";

    return 0;
}
