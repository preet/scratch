// sys includes
#include <math.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <sys/time.h>

// osg includes
#include <osg/Vec3>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/PrimitiveSet>
#include <osg/PolygonMode>
#include <osgViewer/Viewer>
#include <osg/Material>
#include <osg/MatrixTransform>
#include <osg/AutoTransform>
#include <osgViewer/CompositeViewer>
#include <osgGA/TrackballManipulator>

// obb
#include "obb.hpp"
#include "tri.hpp"

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

size_t const K_MAX_LOD = 18;
double const K_MAX_LOD_AREA_REF_M = 150*150;
double FOVY_DEGS = 35.0;
uint16_t const K_MAX_VX_LOD = 6;
uint16_t const K_MIN_VX_LOD = 2;

std::vector<osg::Vec4> const K_COLOR_TABLE {
    {0., 0., 0., 1.},
    {41/255., 41/255., 41/255., 0.},
    {102/255., 102/255., 102/255., 0.},
    {140/255., 140/255., 140/255., 0.},
    {200/255., 200/255., 200/255., 0.},
    {66/255., 206/255., 252/255., 0.},
    {124/255., 160/255., 252/255., 0.},
    {173/255., 146/255., 252/255., 0.},
    {255/255., 120/255., 252/255., 0.},
    {255/255., 117/255., 172/255., 0.},
    {255/255., 142/255., 107/255., 0.},
    {252/255., 174/255., 91/255., 0.},
    {252/255., 194/255., 0/255., 0.},
    {202/255., 245/255., 29/255., 0.},
    {0/255., 191/255., 0/255., 0.},
    {100/255., 245/255., 174/255., 0.},
    {0/255., 235/255., 231/255., 0.},
    {255/255., 255/255., 255/255., 0.}
};

struct VxTile;
std::vector<std::vector<VxTile*>> g_list_lod_vxtiles(K_MAX_VX_LOD);

std::vector<double> CalcLodDistances();
std::vector<double> g_list_lod_dist = CalcLodDistances();

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

struct GeoBounds
{
    GeoBounds() :
        minLat(0),maxLat(0),
        minLon(0),maxLon(0)
    {}

    double minLat; double maxLat;
    double minLon; double maxLon;
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

    // a, b and c are found by substituting the parametric
    // equation of a line into the equation for a ellipsoid
    // and solving in terms of the line's parameter

    // * http://en.wikipedia.org/wiki/Ellipsoid
    // * http://gis.stackexchange.com/questions/20780/...
    //   ...point-of-intersection-for-a-ray-and-earths-surface

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

bool CalcApproxBoundingSphere(std::vector<osg::Vec3d> const &list_vx,
                              osg::Vec3d & sphere_center,
                              double & sphere_radius)
{
    if(list_vx.size() < 3) {
        return false;
    }

    // From Real-Time Collision Detection by Ericson p 89

    // Get the bbox min and max
    osg::Vec3d min = list_vx[0];
    osg::Vec3d max = list_vx[0];

    for(auto const & vx : list_vx) {
        min.x() = std::min(min.x(),vx.x());
        min.y() = std::min(min.y(),vx.y());
        min.z() = std::min(min.z(),vx.z());

        max.x() = std::max(max.x(),vx.x());
        max.y() = std::max(max.y(),vx.y());
        max.z() = std::max(max.z(),vx.z());
    }

    // Let the center of the sphere be the bbox center
    sphere_center = (min+max)*0.5;

    // Set the radius as the furthest distance between
    // the center and any single point in list_vx
    sphere_radius = 0;
    for(auto const & vx : list_vx) {
        sphere_radius = std::max(sphere_radius,(vx-sphere_center).length2());
    }

    sphere_radius = sqrt(sphere_radius);
    return true;
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

    return list_lod_dist;
}

osg::ref_ptr<osg::AutoTransform> BuildLodRingsNode()
{
    osg::ref_ptr<osg::Geode> gd_rings = new osg::Geode;
    gd_rings->getOrCreateStateSet()->setMode( GL_LIGHTING,
                                              osg::StateAttribute::OFF |
                                              osg::StateAttribute::PROTECTED );

    // For each ring
    for(size_t i=0; i < K_MAX_LOD; i++) {
        // Get the distance for this LOD
        double dist = g_list_lod_dist[i];

        double cgrad = double(i+1)/K_MAX_LOD;
        osg::Vec4 color = K_COLOR_TABLE[i];

        // Create a ring of vertices
        osg::ref_ptr<osg::Vec3dArray> list_vx = new osg::Vec3dArray((i+1)*2 + 12);
        double const rotate_by_rads = (2.0*K_PI/list_vx->size());

        for(size_t j=0; j < list_vx->size(); j++){
            list_vx->at(j) = osg::Vec3d(
                        dist*cos(rotate_by_rads*j),
                        dist*sin(rotate_by_rads*j),
                        0);
        }

        osg::ref_ptr<osg::Vec4Array> list_cx = new osg::Vec4Array;
        list_cx->push_back(color);

        osg::ref_ptr<osg::Geometry> gm = new osg::Geometry;
        gm->setVertexArray(list_vx);
        gm->setColorArray(list_cx,osg::Array::BIND_OVERALL);
        gm->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_LOOP,0,list_vx->size()));

        gd_rings->addDrawable(gm.get());
    }

    osg::ref_ptr<osg::AutoTransform> xf_rings = new osg::AutoTransform;
    xf_rings->addChild(gd_rings);
    xf_rings->setAutoRotateMode(osg::AutoTransform::ROTATE_TO_SCREEN);

    return xf_rings;
}


// From osg/examples/osgthirdpersonview
osg::ref_ptr<osg::MatrixTransform> BuildFrustumNode(osg::Camera * camera)
{
    // Projection and ModelView matrices
    osg::Matrixd proj;
    osg::Matrixd mv;
    if (camera)
    {
        proj = camera->getProjectionMatrix();
        mv = camera->getViewMatrix();
    }
    else
    {
        // Create some kind of reasonable default Projection matrix.
        proj.makePerspective( 30., 1., 1., 10. );
        // leave mv as identity
    }

    // Get near and far from the Projection matrix.
    const double near = proj(3,2) / (proj(2,2)-1.0);
    const double far = proj(3,2) / (1.0+proj(2,2));

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

    v->at(0).set( 0., 0., 0. );
    v->at(1).set( nLeft, nBottom, -near );
    v->at(2).set( nRight, nBottom, -near );
    v->at(3).set( nRight, nTop, -near );
    v->at(4).set( nLeft, nTop, -near );
    v->at(5).set( fLeft, fBottom, -far );
    v->at(6).set( fRight, fBottom, -far );
    v->at(7).set( fRight, fTop, -far );
    v->at(8).set( fLeft, fTop, -far );

    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
    geom->setUseDisplayList( false );
    geom->setVertexArray( v );

    osg::ref_ptr<osg::Vec4Array> c = new osg::Vec4Array;
    c->push_back(osg::Vec4(1,1,1,1));
    geom->setColorArray( c, osg::Array::BIND_OVERALL );

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
    geode->addDrawable( geom );

    geode->getOrCreateStateSet()->setMode( GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED );


    // Create parent MatrixTransform to transform the view volume by
    // the inverse ModelView matrix.
    osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform;
    mt->setName("frustum");
    mt->setMatrix( osg::Matrixd::inverse( mv ) );
    mt->addChild( geode );

    return mt;
}

osg::ref_ptr<osg::Group> BuildCelestialSurfaceNode()
{
    std::vector<osg::Vec3d> list_vx;
    std::vector<osg::Vec2d> list_tx;
    std::vector<size_t> list_ix;

    BuildEarthSurfaceGeometry(-180,-90,
                              180,90,
                              32,16,
                              list_vx,
                              list_tx,
                              list_ix);

    osg::ref_ptr<osg::Vec3dArray> vx_array =
            new osg::Vec3dArray;
    for(auto const &vx : list_vx) {
        vx_array->push_back(vx);
    }

    osg::ref_ptr<osg::Vec4Array> cx_array =
            new osg::Vec4Array;
    cx_array->push_back(osg::Vec4(0.25,0.25,0.25,1.0));

    osg::ref_ptr<osg::DrawElementsUShort> ix_array =
            new osg::DrawElementsUShort(GL_TRIANGLES);
    for(auto const ix : list_ix) {
        ix_array->push_back(ix);
    }

    // geometry
    osg::ref_ptr<osg::Geometry> gm = new osg::Geometry;
    gm->setVertexArray(vx_array.get());
    gm->setColorArray(cx_array,osg::Array::Binding::BIND_OVERALL);
    gm->addPrimitiveSet(ix_array.get());

    // geode
    osg::ref_ptr<osg::Geode> gd = new osg::Geode;
    gd->addDrawable(gm.get());

    // group
    osg::ref_ptr<osg::Group> gp = new osg::Group;
    gp->addChild(gd.get());

    // enable wireframe and disable lighting
    osg::ref_ptr<osg::PolygonMode> polygonMode = new osg::PolygonMode;
    polygonMode->setMode(osg::PolygonMode::FRONT_AND_BACK,
                         osg::PolygonMode::LINE);

    gp->getOrCreateStateSet()->setAttribute(polygonMode.get());
    gp->getOrCreateStateSet()->setMode(GL_LIGHTING,osg::StateAttribute::OFF);


    return gp;
}

struct VxTile
{
    double minLon;
    double maxLon;
    double minLat;
    double maxLat;

    osg::Vec3d ecef_tl;
    osg::Vec3d ecef_bl;
    osg::Vec3d ecef_br;
    osg::Vec3d ecef_tr;

    uint16_t level;

    VxTile * top_left;
    VxTile * top_right;
    VxTile * btm_left;
    VxTile * btm_right;



    VxTile() :
        level(0),
        top_left(nullptr),
        top_right(nullptr),
        btm_left(nullptr),
        btm_right(nullptr)
    {
        // empty
    }
};

void SplitTile(VxTile * parent)
{
    // Split this tile into 4
    PointLLA lla_cen;
    lla_cen.lon = (parent->minLon + parent->maxLon)*0.5;
    lla_cen.lat = (parent->minLat + parent->maxLat)*0.5;
    lla_cen.alt = 0;

    PointLLA const lla_tm(lla_cen.lon,parent->maxLat);
    PointLLA const lla_bm(lla_cen.lon,parent->minLat);
    PointLLA const lla_ml(parent->minLon,lla_cen.lat);
    PointLLA const lla_mr(parent->maxLon,lla_cen.lat);

    osg::Vec3d const ecef_cen = ConvLLAToECEF(lla_cen);
    osg::Vec3d const ecef_tm = ConvLLAToECEF(lla_tm);
    osg::Vec3d const ecef_bm = ConvLLAToECEF(lla_bm);
    osg::Vec3d const ecef_ml = ConvLLAToECEF(lla_ml);
    osg::Vec3d const ecef_mr = ConvLLAToECEF(lla_mr);

    uint16_t const next_level = parent->level + 1;

    // TOP-LEFT
    {
        auto & tl = parent->top_left;
        tl = new VxTile;

        tl->minLon = lla_ml.lon;
        tl->minLat = lla_ml.lat;

        tl->maxLon = lla_tm.lon;
        tl->maxLat = lla_tm.lat;

        tl->ecef_tl = parent->ecef_tl;
        tl->ecef_bl = ecef_ml;
        tl->ecef_br = ecef_cen;
        tl->ecef_tr = ecef_tm;

        tl->level = next_level;

//        std::cout << "my_level: " << tl->level << ", ";
//        std::cout << "minLat: " << tl->minLat << ", "
//                  << "maxLat: " << tl->maxLat << ", "
//                  << "minLon: " << tl->minLon << ", "
//                  << "maxLon: " << tl->maxLon
//                  << std::endl;
    }

    // TOP-RIGHT
    {
        auto & tr = parent->top_right;
        tr = new VxTile;

        tr->minLon = lla_cen.lon;
        tr->minLat = lla_cen.lat;

        tr->maxLon = parent->maxLon;
        tr->maxLat = parent->maxLat;

        tr->ecef_tl = ecef_tm;
        tr->ecef_bl = ecef_cen;
        tr->ecef_br = ecef_mr;
        tr->ecef_tr = parent->ecef_tr;

        tr->level = next_level;
    }

    // BOTTOM-LEFT
    {
        auto & bl = parent->btm_left;
        bl = new VxTile;

        bl->minLon = parent->minLon;
        bl->minLat = parent->minLat;

        bl->maxLon = lla_cen.lon;
        bl->maxLat = lla_cen.lat;

        bl->ecef_tl = ecef_ml;
        bl->ecef_bl = parent->ecef_bl;
        bl->ecef_br = ecef_bm;
        bl->ecef_tr = ecef_cen;

        bl->level = next_level;
    }

    // BOTTOM-RIGHT
    {
        auto & br = parent->btm_right;
        br = new VxTile;

        br->minLon = lla_bm.lon;
        br->minLat = lla_bm.lat;

        br->maxLon = lla_mr.lon;
        br->maxLat = lla_mr.lat;

        br->ecef_tl = ecef_cen;
        br->ecef_bl = ecef_bm;
        br->ecef_br = parent->ecef_br;
        br->ecef_tr = ecef_mr;

        br->level = next_level;
    }
}

void BuildViewExtents(osg::Vec3d const &eye,
                      osg::Vec3d const &surf_xsec,
                      VxTile * tile)
{
    // Return if tile isn't visible within the camera frustum
    // if(!visible) { return; }

    // If the tile size is larger than [threshold], divide
    if(tile->level < K_MIN_VX_LOD) {

        g_list_lod_vxtiles[tile->level].push_back(tile);
        SplitTile(tile);
        BuildViewExtents(eye,surf_xsec,tile->top_left);
        BuildViewExtents(eye,surf_xsec,tile->btm_left);
        BuildViewExtents(eye,surf_xsec,tile->btm_right);
        BuildViewExtents(eye,surf_xsec,tile->top_right);
    }
    else {
        // Return if the tile doesn't intersect the lod distance
        // from the eye for its level

        // Approximate the tile with two triangles
        bool tri_xsec = TriangleIntersectsSphere(tile->ecef_tl,
                                                 tile->ecef_bl,
                                                 tile->ecef_br,
                                                 eye,
                                                 g_list_lod_dist[tile->level]);

        if(!tri_xsec) {
            tri_xsec = TriangleIntersectsSphere(tile->ecef_tl,
                                                tile->ecef_br,
                                                tile->ecef_tr,
                                                eye,
                                                g_list_lod_dist[tile->level]);
        }

        if(!tri_xsec) {
            return;
        }

        // If the tile intersects its lod sphere, subdivide it
        g_list_lod_vxtiles[tile->level].push_back(tile);

        // Return if the tile level exceeds the max
        if(tile->level == (K_MAX_VX_LOD-1)) {
            return;
        }

        SplitTile(tile);
        BuildViewExtents(eye,surf_xsec,tile->top_left);
        BuildViewExtents(eye,surf_xsec,tile->btm_left);
        BuildViewExtents(eye,surf_xsec,tile->btm_right);
        BuildViewExtents(eye,surf_xsec,tile->top_right);
    }
}

std::vector<VxTile*> BuildBaseViewExtents()
{
    std::vector<VxTile*> list_vxtiles;

    // Start at level 2

}

osg::ref_ptr<osg::Group> GetTileGeometry()
{
    osg::ref_ptr<osg::Geode> gd = new osg::Geode;

    for(size_t i=0; i < g_list_lod_vxtiles.size(); i++) {

        osg::Vec4 color = K_COLOR_TABLE[i];

//        std::cout << "###: " << g_list_lod_vxtiles[i].size() << std::endl;

        for(size_t j=0; j < g_list_lod_vxtiles[i].size(); j++) {
            VxTile * tile = g_list_lod_vxtiles[i][j];

            // Create a ring for the tile
            osg::ref_ptr<osg::Vec3dArray> list_vx = new osg::Vec3dArray;
            list_vx->push_back(tile->ecef_tl);
            list_vx->push_back(tile->ecef_bl);
            list_vx->push_back(tile->ecef_br);
            list_vx->push_back(tile->ecef_tr);

//            std::cout << "###: " << tile->minLon << ", " << tile->maxLon << ", " << tile->minLat << ", " << tile->maxLat << std::endl;

//            std::cout << "###: " << tile->ecef_tl.x() << ", " << tile->ecef_tl.y() << ", " << tile->ecef_tl.z() << std::endl;
//            std::cout << "###: " << tile->ecef_bl.x() << ", " << tile->ecef_bl.y() << ", " << tile->ecef_bl.z() << std::endl;
//            std::cout << "###: " << tile->ecef_br.x() << ", " << tile->ecef_br.y() << ", " << tile->ecef_br.z() << std::endl;
//            std::cout << "###: " << tile->ecef_tr.x() << ", " << tile->ecef_tr.y() << ", " << tile->ecef_tr.z() << std::endl;

            osg::ref_ptr<osg::Vec4Array> list_cx = new osg::Vec4Array;
            list_cx->push_back(color);

            osg::ref_ptr<osg::Geometry> gm = new osg::Geometry;
            gm->setVertexArray(list_vx);
            gm->setColorArray(list_cx,osg::Array::BIND_OVERALL);
            gm->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_LOOP,0,list_vx->size()));

            gd->addDrawable(gm.get());
        }
    }

    osg::ref_ptr<osg::Group> gp = new osg::Group;
    gp->setName("gpvxtiles");
    gp->addChild(gd.get());

    return gp;
}

volatile bool g_calc_view_extents = false;

class KeyboardEventHandler : public osgGA::GUIEventHandler
{
public:
    virtual bool handle(osgGA::GUIEventAdapter const &event_adapter,
                        osgGA::GUIActionAdapter &action_adapter)
    {
        (void)action_adapter;

        switch(event_adapter.getEventType())
        {
            case osgGA::GUIEventAdapter::KEYDOWN:
            {
                switch(event_adapter.getKey())
                {
                    case osgGA::GUIEventAdapter::KEY_Space:
                    {
                        g_calc_view_extents = true;
                        return true;
                    }
                    default: {
                        return false;
                    }
                }
                // fall through to default?
            }
            default: {
                return false;
            }
        }
        return false;
    }

    virtual void accept(osgGA::GUIEventHandlerVisitor &v)
    {
        v.visit(*this);
    }
};

int main(int argc, const char *argv[])
{
    (void)argc;
    (void)argv;

    // Camera frustum mesh
    auto xf_camera = BuildFrustumNode(NULL);
    xf_camera->setName("camera");

    // Lod rings
    auto xf_rings = BuildLodRingsNode();
    xf_rings->setName("rings");

    // Celestial body surface mesh
    auto gp_celestial = BuildCelestialSurfaceNode();
    gp_celestial->setName("celestial");

    // View0 root
    osg::ref_ptr<osg::Group> gp_root0 = new osg::Group;
    gp_root0->addChild(gp_celestial);

    // View1 root
    osg::ref_ptr<osg::Group> gp_root1 = new osg::Group;
    gp_root1->addChild(xf_camera);
    gp_root1->addChild(xf_rings);
    gp_root1->addChild(gp_celestial);


    osgViewer::CompositeViewer viewer;

    // Create View 0
    {
        osgViewer::View* view = new osgViewer::View;
        viewer.addView( view );

        view->setUpViewInWindow( 10, 10, 640, 480 );
        view->setSceneData( gp_root0.get() );
        view->getCamera()->setClearColor(osg::Vec4(0.1,0.1,0.1,1.0));
        view->setCameraManipulator( new osgGA::TrackballManipulator );
    }

    // Create view 1
    {
        osgViewer::View* view = new osgViewer::View;
        viewer.addView( view );

        view->addEventHandler(new KeyboardEventHandler());
        view->setUpViewInWindow( 10, 510, 640, 480 );
        view->setSceneData( gp_root1.get() );
        view->getCamera()->setClearColor(osg::Vec4(0.1,0.1,0.1,1.0));
        view->setCameraManipulator( new osgGA::TrackballManipulator );
    }

    // Create the fixed base VxTiles we use to
    // calculate view extents

    while (!viewer.done())
    {
        // Update the wireframe frustum
        size_t const num_children = gp_root->getNumChildren();
        for(size_t i=0; i < num_children; i++) {
            std::string const name = gp_root->getChild(i)->getName();
            if(name == "frustum") {
                gp_root->removeChild(i);
                break;
            }
        }
        gp_root->addChild(BuildFrustumNode(
            viewer.getView(0)->getCamera(),
            osg::Vec4(1,1,1,1)).get());

        // update the rings
        osg::Vec3d eye,vpt,up;
        viewer.getView(0)->getCamera()->getViewMatrixAsLookAt(eye,vpt,up);
        xf_rings->setPosition(eye);

        if(g_calc_view_extents) {
            std::cout << "###: CalcViewExtents" << std::endl;

            // Delete old view extents
            for(size_t i=0; i < g_list_lod_vxtiles.size(); i++) {
                for(size_t j=0; j < g_list_lod_vxtiles[i].size(); j++) {
                    delete g_list_lod_vxtiles[i][j];
                }
                g_list_lod_vxtiles[i].clear();
            }

            // Create the level 0 tile
            VxTile * vx_root = new VxTile();
            vx_root->minLon = -180.0;
            vx_root->maxLon = 180.0;
            vx_root->minLat = -90.0;
            vx_root->maxLat = 90.0;

            vx_root->ecef_tl = ConvLLAToECEF(PointLLA(vx_root->minLon,vx_root->maxLat,0));
            vx_root->ecef_bl = ConvLLAToECEF(PointLLA(vx_root->minLon,vx_root->minLat,0));
            vx_root->ecef_br = ConvLLAToECEF(PointLLA(vx_root->maxLon,vx_root->minLat,0));
            vx_root->ecef_tr = ConvLLAToECEF(PointLLA(vx_root->maxLon,vx_root->maxLat,0));

            vx_root->level = 0;

            // Calc surf xsec with eye vec
            osg::Vec3d xsecNear,xsecFar;
            if(CalcRayEarthIntersection(eye,vpt-eye,xsecNear,xsecFar)) {
                BuildViewExtents(eye,xsecNear,vx_root);
                osg::ref_ptr<osg::Group> gp_vxtiles = GetTileGeometry();
                gp_tiles->addChild(gp_vxtiles);
            }
            else {
                std::cout << "###: Warn: eye -> vpt doesnt not xsec earth";
            }

            g_calc_view_extents = false;
        }

        viewer.frame();
    }
    return 0;
}
