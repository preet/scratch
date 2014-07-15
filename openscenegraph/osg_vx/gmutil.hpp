#ifndef GM_UTIL_H
#define GM_UTIL_H

#include <memory>
#include <vector>
#include <iostream>

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
    {0., 0., 0., 1.},
    {41/255., 41/255., 41/255., 1.},
    {102/255., 102/255., 102/255., 1.},
    {140/255., 140/255., 140/255., 1.},
    {200/255., 200/255., 200/255., 1.},
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
        list_edges.resize(8);
        list_vx.resize(8);
    }
};

struct OBB
{
    // obb
    osg::Vec3d center;
    osg::Vec3d ori[3]; // local orientation axes
    osg::Vec3d ext;    // positive half-extents
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
                      Plane & horizon_plane)
{
    // We need to clamp eye_length such that the
    // eye is outside of the celestial body surface

    osg::Vec3d xsecNear,xsecFar;
    if(!CalcRayEarthIntersection(eye,eye*-1.0,xsecNear,xsecFar)) {
        return false;
    }

    double const xsecNear_length = xsecNear.length();
    double eye_length = eye.length();

    if((eye_length - xsecNear_length) < 500.0) { // clamp to 500m
        return false;
    }

    double const inv_dist = 1.0/eye_length;

    horizon_plane.n = eye*inv_dist;
    horizon_plane.p = horizon_plane.n * (RAD_AV*RAD_AV*inv_dist);

    horizon_plane.n *= -1.0;
    horizon_plane.d = horizon_plane.n * horizon_plane.p;
    return true;
}

//////////////////////////////////////////////////////

// From MathGeoLib
void CalcOBBAxisProjection(OBB const &obb,
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

// From Eberly - The Method of Separating Axes
int WhichSide(osg::Vec3d const &polyA_axis_dirn,
              osg::Vec3d const &polyA_axis_pt,
              std::vector<osg::Vec3d> const &polyB_list_vx)
{
    int positive=0;
//    int negative=0;

    for(auto const & vx : polyB_list_vx)
    {
        double t = polyA_axis_dirn * (vx - polyA_axis_pt);
        if(t > 0) {
            positive++;
        }
//        else {
//            negative++;
//        }
    }

    return (positive ? 1 : -1);
}

bool CalcFrustumQuadIntersectSAT(Frustum const &frustum,
                                 std::vector<osg::Vec3d const *> list_quad_vx)
{
    // Test the faces of the frustum
    for(auto const & plane : frustum.list_planes)
    {
//        if(WhichSide(plane.n,plane.p,list_quad_vx) > 0) {
//            return false;
//        }
    }

    // Test the faces of the
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


//////////////////////////////////////////////////////


#endif // GM_UTIL_H
