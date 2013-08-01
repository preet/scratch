
#include <string>
#include <iostream>
#include <fstream>
#include <vector>

#include <osg/Program>
#include <osg/Uniform>
#include <osg/Geometry>
#include <osgViewer/Viewer>
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

struct LODRange
{
    LODRange() :
        min(0),max(0),level(0)
    {}

    double min;
    double max;
    size_t level;
};

struct Vertex
{
    Vertex() :
        x(0),y(0),z(0),
        lon(0),lat(0),alt(0),
        distToEye(0),
        hasBeenTested(false),
        inFrustum(false)
    {}

    double x,y,z;
    double lon,lat,alt;
    double distToEye;
    bool hasBeenTested;
    bool inFrustum;
};

struct Tile
{
    Tile() :
        vxTL(NULL),
        vxBL(NULL),
        vxBR(NULL),
        vxTR(NULL),
        subtileTL(NULL),
        subtileBL(NULL),
        subtileBR(NULL),
        subtileTR(NULL),
        level(0),
        inFrustum(true),
        inFrontOfHorizon(true)
    {}

    Vertex  * vxTL;
    Vertex  * vxBL;
    Vertex  * vxBR;
    Vertex  * vxTR;

    Tile * subtileTL;
    Tile * subtileBL;
    Tile * subtileBR;
    Tile * subtileTR;

    size_t level;
    bool inFrustum;
    bool inFrontOfHorizon;

};

std::vector<Vertex*> g_LIST_BASE_VERTICES;
std::vector<Tile*>   g_LIST_BASE_TILES;

std::vector<Vertex*> g_LIST_TEMP_VERTICES;

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

void BuildBaseTileList(double minLon, double minLat,
                       double maxLon, double maxLat,
                       size_t lonSegments,
                       size_t latSegments,
                       std::vector<Vertex*> &listVx,
                       std::vector<Tile*> &listTiles)
{
    double lonStep = (maxLon-minLon)/lonSegments;
    double latStep = (maxLat-minLat)/latSegments;

    // build vertices
    for(size_t i=0; i <= latSegments; i++)   {
        for(size_t j=0; j <= lonSegments; j++)   {
            Vec3 temp = ConvLLAToECEF(PointLLA((i*latStep)+minLat,
                                               (j*lonStep)+minLon,0.0));
            Vertex * vx = new Vertex;
            vx->lon = (j*lonStep)+minLon;
            vx->lat = (i*latStep)+minLat;
            vx->alt = 0;
            vx->x = temp.x;
            vx->y = temp.y;
            vx->z = temp.z;

            listVx.push_back(vx);
        }
    }

    // build tiles
    size_t vIdx=0;
    for(size_t i=0; i < latSegments; i++)   {
        for(size_t j=0; j < lonSegments; j++)   {
            Tile * tile = new Tile;
            tile->vxBL = listVx[vIdx];
            tile->vxBR = listVx[vIdx+1];
            tile->vxTR = listVx[vIdx+lonSegments+2];
            tile->vxTL = listVx[vIdx+lonSegments+1];
            listTiles.push_back(tile);
            vIdx++;
        }
        vIdx++;
    }
}

void CountSubTiles(Tile * tile, size_t &count)   {
    if(tile)   {
        count++;
        CountSubTiles(tile->subtileBL,count);
        CountSubTiles(tile->subtileBR,count);
        CountSubTiles(tile->subtileTL,count);
        CountSubTiles(tile->subtileTR,count);
    }
}


size_t CountBaseTiles()
{
    size_t count=0;
    for(size_t i=0; i < g_LIST_BASE_TILES.size(); i++)   {
        CountSubTiles(g_LIST_BASE_TILES[i],count);
    }
    return count;
}

void GetTilesAboveLevel(Tile * root, double const maxLevel,
                        std::vector<Tile*> &listTiles)
{
    if(root)   {
        if(root->level > maxLevel)   {
            listTiles.push_back(root);
        }
        GetTilesAboveLevel(root->subtileTL,maxLevel,listTiles);
        GetTilesAboveLevel(root->subtileBL,maxLevel,listTiles);
        GetTilesAboveLevel(root->subtileBR,maxLevel,listTiles);
        GetTilesAboveLevel(root->subtileTR,maxLevel,listTiles);
    }
}

void BuildQuadtreeFromCamera(Vec3 const &eye,
                             std::vector<double> const &listDist2ByLevel,
                             Tile * tile)
{
    // generate tile center
    PointLLA cenLLA((tile->vxTL->lat + tile->vxBR->lat)/2.0,
                    (tile->vxTL->lon + tile->vxBR->lon)/2.0);

    Vec3 cenECEF = ConvLLAToECEF(cenLLA);

    if((listDist2ByLevel[tile->level] < 0) ||
       (cenECEF.Distance2To(eye) < listDist2ByLevel[tile->level]))   {
        // build subtiles

        // center vx
        Vertex * vxC = new Vertex;
        vxC->lon = cenLLA.lon;
        vxC->lat = cenLLA.lat;
        vxC->alt = cenLLA.alt;
        vxC->x = cenECEF.x;
        vxC->y = cenECEF.y;
        vxC->z = cenECEF.z;
        g_LIST_TEMP_VERTICES.push_back(vxC);

        // center-left vx
        PointLLA llaCL(cenLLA.lat,tile->vxTL->lon);
        Vec3 ecefCL = ConvLLAToECEF(llaCL);
        Vertex * vxCL = new Vertex;
        vxCL->lon = llaCL.lon;
        vxCL->lat = llaCL.lat;
        vxCL->alt = llaCL.alt;
        vxCL->x = ecefCL.x;
        vxCL->y = ecefCL.y;
        vxCL->z = ecefCL.z;
        g_LIST_TEMP_VERTICES.push_back(vxCL);

        // center-right vx
        PointLLA llaCR(cenLLA.lat,tile->vxTR->lon);
        Vec3 ecefCR = ConvLLAToECEF(llaCR);
        Vertex * vxCR = new Vertex;
        vxCR->lon = llaCR.lon;
        vxCR->lat = llaCR.lat;
        vxCR->alt = llaCR.alt;
        vxCR->x = ecefCR.x;
        vxCR->y = ecefCR.y;
        vxCR->z = ecefCR.z;
        g_LIST_TEMP_VERTICES.push_back(vxCR);

        // center-top vx
        PointLLA llaCT(tile->vxTL->lat,cenLLA.lon);
        Vec3 ecefCT = ConvLLAToECEF(llaCT);
        Vertex * vxCT = new Vertex;
        vxCT->lon = llaCT.lon;
        vxCT->lat = llaCT.lat;
        vxCT->alt = llaCT.alt;
        vxCT->x = ecefCT.x;
        vxCT->y = ecefCT.y;
        vxCT->z = ecefCT.z;
        g_LIST_TEMP_VERTICES.push_back(vxCT);

        // center-btm vx
        PointLLA llaCB(tile->vxBL->lat,cenLLA.lon);
        Vec3 ecefCB = ConvLLAToECEF(llaCB);
        Vertex * vxCB = new Vertex;
        vxCB->lon = llaCB.lon;
        vxCB->lat = llaCB.lat;
        vxCB->alt = llaCB.alt;
        vxCB->x = ecefCB.x;
        vxCB->y = ecefCB.y;
        vxCB->z = ecefCB.z;
        g_LIST_TEMP_VERTICES.push_back(vxCB);

        tile->subtileTL = new Tile;
        tile->subtileTL->vxTL = tile->vxTL;
        tile->subtileTL->vxBL = vxCL;
        tile->subtileTL->vxBR = vxC;
        tile->subtileTL->vxTR = vxCT;
        tile->subtileTL->level = tile->level+1;
        BuildQuadtreeFromCamera(eye,listDist2ByLevel,tile->subtileTL);

        tile->subtileBL = new Tile;
        tile->subtileBL->vxTL = vxCL;
        tile->subtileBL->vxBL = tile->vxBL;
        tile->subtileBL->vxBR = vxCB;
        tile->subtileBL->vxTR = vxC;
        tile->subtileBL->level = tile->level+1;
        BuildQuadtreeFromCamera(eye,listDist2ByLevel,tile->subtileBL);

        tile->subtileBR = new Tile;
        tile->subtileBR->vxTL = vxC;
        tile->subtileBR->vxBL = vxCB;
        tile->subtileBR->vxBR = tile->vxBR;
        tile->subtileBR->vxTR = vxCR;
        tile->subtileBR->level = tile->level+1;
        BuildQuadtreeFromCamera(eye,listDist2ByLevel,tile->subtileBR);

        tile->subtileTR = new Tile;
        tile->subtileTR->vxTL = vxCT;
        tile->subtileTR->vxBL = vxC;
        tile->subtileTR->vxBR = vxCR;
        tile->subtileTR->vxTR = tile->vxTR;
        tile->subtileTR->level = tile->level+1;
        BuildQuadtreeFromCamera(eye,listDist2ByLevel,tile->subtileTR);
    }
}

void CullTilesAgainstHorizon(Vec3 const &scaledSpaceEye,
                             Tile * tile)
{
    if(tile)   {
        // if any of the four corner vertices
        // of these tiles are in front of the
        // horizon plane, we save the tile

        Vec3 ssvx_bl,ssvx_br,ssvx_tr,ssvx_tl; // scaled ellipsoid space vertex

        // bottom left
        ssvx_bl = Vec3(tile->vxBL->x/ELL_SEMI_MAJOR,
                       tile->vxBL->y/ELL_SEMI_MAJOR,
                       tile->vxBL->z/ELL_SEMI_MINOR);

        ssvx_br = Vec3(tile->vxBR->x/ELL_SEMI_MAJOR,
                       tile->vxBR->y/ELL_SEMI_MAJOR,
                       tile->vxBR->z/ELL_SEMI_MINOR);

        ssvx_tr = Vec3(tile->vxTR->x/ELL_SEMI_MAJOR,
                       tile->vxTR->y/ELL_SEMI_MAJOR,
                       tile->vxTR->z/ELL_SEMI_MINOR);

        ssvx_tl = Vec3(tile->vxTL->x/ELL_SEMI_MAJOR,
                       tile->vxTL->y/ELL_SEMI_MAJOR,
                       tile->vxTL->z/ELL_SEMI_MINOR);


        if(CalcPointBeyondHorizonPlane(scaledSpaceEye,ssvx_bl) &&
           CalcPointBeyondHorizonPlane(scaledSpaceEye,ssvx_br) &&
           CalcPointBeyondHorizonPlane(scaledSpaceEye,ssvx_tr) &&
           CalcPointBeyondHorizonPlane(scaledSpaceEye,ssvx_tl))
        {
            tile->inFrontOfHorizon = false;
        }
        else   {
            tile->inFrontOfHorizon = true;
        }

        CullTilesAgainstHorizon(scaledSpaceEye,tile->subtileBL);
        CullTilesAgainstHorizon(scaledSpaceEye,tile->subtileBR);
        CullTilesAgainstHorizon(scaledSpaceEye,tile->subtileTR);
        CullTilesAgainstHorizon(scaledSpaceEye,tile->subtileTL);
    }
}

void CullTilesAgainstFrustum(std::vector<Vec3> const &listFPlaneNorms,
                             std::vector<Vec3> const &listFPlanePoints,
                             Tile * tile)
{
    // normal vectors are pointing outside the frustum

    if(tile)   {
        Vec3 vx_bl(tile->vxBL->x,tile->vxBL->y,tile->vxBL->z);
        Vec3 vx_br(tile->vxBR->x,tile->vxBR->y,tile->vxBR->z);
        Vec3 vx_tr(tile->vxTR->x,tile->vxTR->y,tile->vxTR->z);
        Vec3 vx_tl(tile->vxTL->x,tile->vxTL->y,tile->vxTL->z);

        std::vector<Vec3> listCornerVx;
        listCornerVx.push_back(vx_bl);
        listCornerVx.push_back(vx_br);
        listCornerVx.push_back(vx_tr);
        listCornerVx.push_back(vx_tl);

        std::vector<bool> listCornerInFrustum(4,true);

        // !! All four should have to be out of the frustum
        //    to completely discard the tile

        for(size_t i=0; i < listCornerVx.size(); i++)   {
            // if any of the four corner vertices of the tile are
            // within all six planes of the frustum, keep the tile

            for(size_t j=0; j < listFPlanePoints.size(); j++)   {
                // vector from point on plane to tile corner vx
                Vec3 p_to_vx = listCornerVx[i]-listFPlanePoints[j];
                if(p_to_vx.Dot(listFPlaneNorms[j]) > 0)   {
                    // a positive dot product indicates the point is
                    // outside the given plane wrt to the frustum
                    listCornerInFrustum[i] = false;
                    break;
                }
            }
        }

        tile->inFrustum =
                (listCornerInFrustum[0] ||
                 listCornerInFrustum[1] ||
                 listCornerInFrustum[2] ||
                 listCornerInFrustum[3]);

        CullTilesAgainstFrustum(listFPlaneNorms,listFPlanePoints,tile->subtileBL);
        CullTilesAgainstFrustum(listFPlaneNorms,listFPlanePoints,tile->subtileBR);
        CullTilesAgainstFrustum(listFPlaneNorms,listFPlanePoints,tile->subtileTR);
        CullTilesAgainstFrustum(listFPlaneNorms,listFPlanePoints,tile->subtileTL);
    }
}

void BuildTilesFromCameraLOD(Vec3 const &eye,
                             Vec3 const &vpt,
                             Vec3 const &up,
                             double const fovy,
                             double const ar,
                             double const znear,
                             double const zfar)
{
//    std::cout << "SZ LIST BASE TILES BEF: "
//              << CountBaseTiles() << std::endl;

    std::vector<double> listMaxDist2(g_LIST_LOD_RANGES[0].level+1,-1.0);
    for(size_t i=0; i < listMaxDist2.size(); i++)   {
        for(size_t j=0; j < g_LIST_LOD_RANGES.size(); j++)   {
            if(g_LIST_LOD_RANGES[j].level == i)   {
                listMaxDist2[i] = g_LIST_LOD_RANGES[j].max*g_LIST_LOD_RANGES[j].max;
            }
        }
    }

    for(size_t i=0; i < g_LIST_BASE_TILES.size(); i++)   {
        BuildQuadtreeFromCamera(eye,listMaxDist2,g_LIST_BASE_TILES[i]);
    }

    Vec3 scaledSpaceEye(eye.x / ELL_SEMI_MAJOR,
                        eye.y / ELL_SEMI_MAJOR,
                        eye.z / ELL_SEMI_MINOR);
    for(size_t i=0; i < g_LIST_BASE_TILES.size(); i++)   {
        CullTilesAgainstHorizon(scaledSpaceEye,g_LIST_BASE_TILES[i]);
    }

    // build frustum

    // calculate the four edge vectors of the view frustum
    double fovy_rad_bi  = (fovy*K_PI/180.0)/2.0;
    double dAlongViewPt = cos(fovy_rad_bi);
    double dAlongUp     = sin(fovy_rad_bi);
    double dAlongRight  = dAlongUp*ar;

    Vec3 camRight     = (vpt-eye).Cross(up);
    Vec3 vAlongUp     = up.Normalized().ScaledBy(dAlongUp);
    Vec3 vAlongRight  = camRight.Normalized().ScaledBy(dAlongRight);
    Vec3 vAlongViewPt = (vpt-eye).Normalized().ScaledBy(dAlongViewPt);

    std::vector<Vec3> listProjVec(9);
    listProjVec[0] = vAlongViewPt - vAlongUp - vAlongRight;     // bottom left
    listProjVec[1] = vAlongViewPt - vAlongUp;                   // bottom
    listProjVec[2] = vAlongViewPt - vAlongUp + vAlongRight;     // bottom right
    listProjVec[3] = vAlongViewPt + vAlongRight;                // right
    listProjVec[4] = vAlongViewPt + vAlongUp + vAlongRight;     // top right
    listProjVec[5] = vAlongViewPt + vAlongUp;                   // top
    listProjVec[6] = vAlongViewPt + vAlongUp - vAlongRight;     // top left
    listProjVec[7] = vAlongViewPt - vAlongRight;                // left
    listProjVec[8] = vAlongViewPt;                              // center

    double nearDist = 10.0;
    double farDist  = eye.Magnitude()*2.0;
    Vec3 nearBL = eye + listProjVec[0].Normalized().ScaledBy(nearDist);
    Vec3 nearTR = eye + listProjVec[4].Normalized().ScaledBy(nearDist);
    Vec3 farBL  = eye + listProjVec[0].Normalized().ScaledBy(farDist);

    // frustum plane normals (these point outside
    // according to right hand rule)
    std::vector<Vec3> listFPlaneNorms(6);
    listFPlaneNorms[0] = listProjVec[0].Cross(listProjVec[2]);  // bottom
    listFPlaneNorms[1] = listProjVec[2].Cross(listProjVec[4]);  // right
    listFPlaneNorms[2] = listProjVec[4].Cross(listProjVec[6]);  // top
    listFPlaneNorms[3] = listProjVec[6].Cross(listProjVec[0]);  // left
    listFPlaneNorms[4] = vAlongViewPt.Normalized();             // far
    listFPlaneNorms[5] = listFPlaneNorms[4].ScaledBy(-1.0);     // near

    // frustum plane points
    std::vector<Vec3> listFPlanePoints(6);
    listFPlanePoints[0] = nearBL;           // bottom
    listFPlanePoints[1] = nearTR;           // right
    listFPlanePoints[2] = nearTR;           // top
    listFPlanePoints[3] = nearBL;           // left
    listFPlanePoints[4] = farBL;            // far
    listFPlanePoints[5] = nearBL;           // near

    for(size_t i=0; i < g_LIST_BASE_TILES.size(); i++)   {
        CullTilesAgainstFrustum(listFPlaneNorms,
                                listFPlanePoints,
                                g_LIST_BASE_TILES[i]);
    }


//    std::cout << "SZ LIST BASE TILES AFT: "
//              << CountBaseTiles() << std::endl;
}

osg::Program * g_VERTEX_ATTR_FLAT_SHADER;

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
}

void BuildListVxFromTiles(Tile * root, std::vector<osg::Vec3d> &listVx)
{
    if(root)   {
        if(root->inFrustum && root->inFrontOfHorizon)   {
            osg::Vec3d vxBL(root->vxBL->x,root->vxBL->y,root->vxBL->z);
            osg::Vec3d vxBR(root->vxBR->x,root->vxBR->y,root->vxBR->z);
            osg::Vec3d vxTR(root->vxTR->x,root->vxTR->y,root->vxTR->z);
            osg::Vec3d vxTL(root->vxTL->x,root->vxTL->y,root->vxTL->z);

            // TL<->BL
            listVx.push_back(vxTL);
            listVx.push_back(vxBL);

            // BL<->BR
            listVx.push_back(vxBL);
            listVx.push_back(vxBR);

            // BR<->TR
            listVx.push_back(vxBR);
            listVx.push_back(vxTR);

            // TR<->TL
            listVx.push_back(vxTR);
            listVx.push_back(vxTL);
        }

        BuildListVxFromTiles(root->subtileTL,listVx);
        BuildListVxFromTiles(root->subtileBL,listVx);
        BuildListVxFromTiles(root->subtileBR,listVx);
        BuildListVxFromTiles(root->subtileTR,listVx);
    }
}

osg::Geode * BuildGdEarthFromCamera(osg::Camera * camera)
{

    if(camera)   {
        // generate the tiles
        osg::Vec3d veye,vvpt,vup;
        double fovy,ar,znear,zfar;
        camera->getViewMatrixAsLookAt(veye,vvpt,vup);
        camera->getProjectionMatrixAsPerspective(fovy,ar,znear,zfar);
        BuildTilesFromCameraLOD(Vec3(veye.x(),veye.y(),veye.z()),
                                Vec3(vvpt.x(),vvpt.y(),vvpt.z()),
                                Vec3(vup.x(),vup.y(),vup.z()),
                                fovy,ar,znear,zfar);
    }

    std::vector<osg::Vec3d> listTileLineVx;
    for(size_t i=0; i < g_LIST_BASE_TILES.size(); i++)   {
        BuildListVxFromTiles(g_LIST_BASE_TILES[i],listTileLineVx);
    }


    // [earth geometry]
    osg::ref_ptr<osg::Vec3dArray> listVx = new osg::Vec3dArray;
    osg::ref_ptr<osg::Vec4Array>  listCx = new osg::Vec4Array;
    for(size_t i=0; i < listTileLineVx.size(); i++)   {
        listVx->push_back(listTileLineVx[i]);
        listCx->push_back(osg::Vec4(1.0,1.0,1.0,1.0));
    }

    osg::ref_ptr<osg::Geometry> gmEarth = new osg::Geometry;
    gmEarth->setVertexArray(listVx);
    gmEarth->setColorArray(listCx);
    gmEarth->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
    gmEarth->addPrimitiveSet(new osg::DrawArrays(GL_LINES,0,listVx->size()));

    // [earth geode]
    osg::ref_ptr<osg::Geode> gdEarth = new osg::Geode;
    gdEarth->setName("gdEarth");
    gdEarth->addDrawable(gmEarth);

    osg::StateSet * ss = gdEarth->getOrCreateStateSet();
    ss->setAttributeAndModes(g_VERTEX_ATTR_FLAT_SHADER);


    // clean up
    std::vector<Tile*> listTiles;
    for(size_t i=0; i < g_LIST_BASE_TILES.size(); i++)   {
        GetTilesAboveLevel(g_LIST_BASE_TILES[i],4,listTiles);
        g_LIST_BASE_TILES[i]->subtileBL = NULL;
        g_LIST_BASE_TILES[i]->subtileBR = NULL;
        g_LIST_BASE_TILES[i]->subtileTR = NULL;
        g_LIST_BASE_TILES[i]->subtileTL = NULL;
    }
    for(size_t i=0; i < listTiles.size(); i++)   {
        delete listTiles[i];
    }
    for(size_t i=0; i < g_LIST_TEMP_VERTICES.size(); i++)   {
        delete g_LIST_TEMP_VERTICES[i];
    }
    g_LIST_TEMP_VERTICES.clear();

    return gdEarth.release();
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
    mt->setName("xfCameraFrustum");

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
