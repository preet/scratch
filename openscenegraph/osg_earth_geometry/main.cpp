 // sys includes
#include <math.h>
#include <vector>
#include <iostream>
#include <sys/time.h>

// osg includes
#include <osg/Geometry>
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>
#include <osgUtil/SmoothingVisitor>
#include <osg/Texture2D>
#include <osgDB/ReadFile>
#include <osg/PolygonMode>
#include <osg/Geode>
#include <osg/Vec3>
#include <osgViewer/Viewer>

// PI!
#define K_PI 3.141592653589

// WGS84 ellipsoid parameters (http://en.wikipedia.org/wiki/WGS_84)
#define ELL_SEMI_MAJOR 6378137.0            // meters
#define ELL_SEMI_MINOR 6356752.3142         // meters
#define ELL_F 1/298.257223563
#define ELL_ECC_EXP2 6.69437999014e-3
#define ELL_ECC2_EXP2 6.73949674228e-3


// longitude, latitude, altitude point class
class PointLLA
{
public:
    PointLLA(double myLat, double myLon, double myAlt) :
        lon(myLon),lat(myLat),alt(myAlt) {}

    PointLLA(double myLat, double myLon) :
        lat(myLat),lon(myLon) {}

    double lon;
    double lat;
    double alt;
};


// cartesian coordinate system point class
class Point3D
{
public:
    Point3D() :
        x(0),y(0),z(0) {}

    Point3D(double myX, double myY, double myZ) :
        x(myX),y(myY),z(myZ) {}

    double x;
    double y;
    double z;
};


class Point2D
{
public:
    Point2D() :
        x(0),y(0) {}

    Point2D(double myX, double myY) :
        x(myX), y(myY) {}

    double x;
    double y;
};

namespace scout

{

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

    inline double DistanceTo(Vec3 const &otherVec) const
    {
        return sqrt((x-otherVec.x)*(x-otherVec.x) +
                    (y-otherVec.y)*(y-otherVec.y) +
                    (z-otherVec.z)*(z-otherVec.z));
    }

    inline double Distance2To(Vec3 const &otherVec) const
    {
        return ((x-otherVec.x)*(x-otherVec.x) +
                (y-otherVec.y)*(y-otherVec.y) +
                (z-otherVec.z)*(z-otherVec.z));
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

    inline Vec3 RotatedBy(const Vec3 &axisVec, double angleDeg)
    {
        Vec3 rotatedVec;
        double angleRad = angleDeg*K_PI/180;
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

}


void convLLAToECEF(PointLLA const &pointLLA, Point3D &pointECEF);
void convECEFToLLA(Point3D const &pointECEF, PointLLA &pointLLA);

void createSphereSurfaceGeometry(int numRings, int sectorDivs,
                                 std::vector<Point3D> &myVertices,
                                 std::vector<Point3D> &myNormals,
                                 std::vector<Point2D> &myTexCoords,
                                 std::vector<unsigned int> &myIndices);

bool createEarthSurfaceGeometry(int latSegments, int lonSegments,
                                std::vector<Point3D> &myVertices,
                                std::vector<Point3D> &myNormals,
                                std::vector<Point2D> &myTexCoords,
                                std::vector<unsigned int> &myIndices);

int main()
{   
    bool renderAsPointCloud = false;

    // node: origin
    double sF = 1000000;  // scaleFactor
    osg::ref_ptr<osg::ShapeDrawable> myOriginX = new osg::ShapeDrawable;
    myOriginX->setShape(new osg::Box(osg::Vec3(5*sF,0,0),10*sF,0.1*sF,0.1*sF));
    myOriginX->setColor(osg::Vec4(1,0,0,1));

    osg::ref_ptr<osg::ShapeDrawable> myOriginY = new osg::ShapeDrawable;
    myOriginY->setShape(new osg::Box(osg::Vec3(0,5*sF,0),0.1*sF,10*sF,0.1*sF));
    myOriginY->setColor(osg::Vec4(0,1,0,1));

    osg::ref_ptr<osg::ShapeDrawable> myOriginZ = new osg::ShapeDrawable;
    myOriginZ->setShape(new osg::Box(osg::Vec3(0,0,5*sF),0.1*sF,0.1*sF,10*sF));
    myOriginZ->setColor(osg::Vec4(0,0,1,1));

    osg::ref_ptr<osg::ShapeDrawable> myOriginSphere = new osg::ShapeDrawable;
    myOriginSphere->setShape(new osg::Sphere(osg::Vec3(0,0,0),0.5*sF));
    myOriginSphere->setColor(osg::Vec4(0.7,0.7,0.7,1));

    osg::ref_ptr<osg::Geode> myOriginNode = new osg::Geode;
    myOriginNode->addDrawable(myOriginX.get());
    myOriginNode->addDrawable(myOriginY.get());
    myOriginNode->addDrawable(myOriginZ.get());
    myOriginNode->addDrawable(myOriginSphere.get());

    // node: frustum
    double fSF = ELL_SEMI_MAJOR;    // scale factor
    scout::Vec3 camEye(859270.45,-4541000.3,4381186.5);
    scout::Vec3 camViewpoint(-3739340.3,-5001226.7,0);
    scout::Vec3 camUp(-0.68697857,-0.068752435,0.72341797);

    double camFovY = 20.0;
    double camAspectRatio = 1.33;

    // calculate four edge vectors of the frustum
    double camFovY_rad_bi = (camFovY*K_PI/180.0)/2;
    double dAlongViewpoint = cos(camFovY_rad_bi);
    double dAlongUp = sin(camFovY_rad_bi);
    double dAlongRight = dAlongUp*camAspectRatio;
    double nearDist = 666666.67;
    double farDist = 8378137;

    scout::Vec3 camAlongViewpoint = camViewpoint-camEye;
    scout::Vec3 camRight = camAlongViewpoint.Cross(camUp);
    scout::Vec3 vAlongViewpoint = camAlongViewpoint.Normalized().ScaledBy(dAlongViewpoint);
    scout::Vec3 vAlongUp = camUp.Normalized().ScaledBy(dAlongUp);
    scout::Vec3 vAlongRight = camRight.Normalized().ScaledBy(dAlongRight);

    scout::Vec3 viewTL = camEye + (vAlongViewpoint + vAlongUp - vAlongRight).ScaledBy(4*fSF);
    scout::Vec3 viewTR = camEye + (vAlongViewpoint + vAlongUp + vAlongRight).ScaledBy(4*fSF);
    scout::Vec3 viewBL = camEye + (vAlongViewpoint - vAlongUp - vAlongRight).ScaledBy(4*fSF);
    scout::Vec3 viewBR = camEye + (vAlongViewpoint - vAlongUp + vAlongRight).ScaledBy(4*fSF);
    scout::Vec3 viewNP = camEye + vAlongViewpoint.Normalized().ScaledBy(nearDist);
    scout::Vec3 viewFP = camEye + vAlongViewpoint.Normalized().ScaledBy(farDist);

    osg::ref_ptr<osg::Vec3Array> frustumVertices = new osg::Vec3Array;
    frustumVertices->push_back(osg::Vec3(camEye.x,camEye.y,camEye.z));       // 0 cam eye
    frustumVertices->push_back(osg::Vec3(viewTL.x,viewTL.y,viewTL.z));       // 1 cam TL
    frustumVertices->push_back(osg::Vec3(viewTR.x,viewTR.y,viewTR.z));       // 2 cam TR
    frustumVertices->push_back(osg::Vec3(viewBL.x,viewBL.y,viewBL.z));       // 3 cam BL
    frustumVertices->push_back(osg::Vec3(viewBR.x,viewBR.y,viewBR.z));       // 4 cam BR

    osg::ref_ptr<osg::DrawElementsUInt> frustumIndices =
            new osg::DrawElementsUInt(GL_TRIANGLES);

    // left face
    frustumIndices->push_back(0);
    frustumIndices->push_back(3);
    frustumIndices->push_back(1);

    // right face
    frustumIndices->push_back(0);
    frustumIndices->push_back(2);
    frustumIndices->push_back(4);

    // bottom face
    frustumIndices->push_back(0);
    frustumIndices->push_back(4);
    frustumIndices->push_back(3);

    // top face
    frustumIndices->push_back(0);
    frustumIndices->push_back(1);
    frustumIndices->push_back(2);

    osg::ref_ptr<osg::Geometry> frustumGeom = new osg::Geometry;
    frustumGeom->setVertexArray(frustumVertices.get());
    frustumGeom->addPrimitiveSet(frustumIndices.get());

    // node: frustum
    osg::ref_ptr<osg::Geode> myFrustumNode = new osg::Geode;
    myFrustumNode->addDrawable(frustumGeom.get());

    // set frustum rendering mode to wireframe
    osg::PolygonMode* polygonMode = new osg::PolygonMode();
    polygonMode->setMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE);
    myFrustumNode->getOrCreateStateSet()->setAttributeAndModes(polygonMode,
                                                               osg::StateAttribute::ON);

    // draw view extents query info
    double minLon = -79.292803;
    double minLat = 43.659178;
    double maxLon = -79.286771;
    double maxLat = 43.661683;

    PointLLA veNW(maxLat,minLon);
    PointLLA veNE(maxLat,maxLon);
    PointLLA veSW(minLat,minLon);
    PointLLA veSE(minLat,maxLon);

    Point3D pointNW; convLLAToECEF(veNW,pointNW);
    Point3D pointNE; convLLAToECEF(veNE,pointNE);
    Point3D pointSW; convLLAToECEF(veSW,pointSW);
    Point3D pointSE; convLLAToECEF(veSE,pointSE);

    osg::ref_ptr<osg::Vec3Array> viewExtentsVertices = new osg::Vec3Array;
    viewExtentsVertices->push_back(osg::Vec3(pointNW.x,pointNW.y,pointNW.z));
    viewExtentsVertices->push_back(osg::Vec3(pointNE.x,pointNE.y,pointNE.z));
    viewExtentsVertices->push_back(osg::Vec3(pointSW.x,pointSW.y,pointSW.z));
    viewExtentsVertices->push_back(osg::Vec3(pointSE.x,pointSE.y,pointSE.z));

    // q0
    viewExtentsVertices->push_back(osg::Vec3(858764.55,-4541020.3,4380974.7));
    viewExtentsVertices->push_back(osg::Vec3(859108.31,-4540955.3,4380974.7));
    viewExtentsVertices->push_back(osg::Vec3(858800.26,-4541209.1,4380773.3));
    viewExtentsVertices->push_back(osg::Vec3(859144.03,-4541144.1,4380773.3));

    // q1
    viewExtentsVertices->push_back(osg::Vec3(858630.3,-4541045.7,4380974.7));
    viewExtentsVertices->push_back(osg::Vec3(859108.31,-4540955.3,4380974.7));
    viewExtentsVertices->push_back(osg::Vec3(858666,-4541234.5,4380773.3));
    viewExtentsVertices->push_back(osg::Vec3(859144.03,-4541144.1,4380773.3));

    osg::ref_ptr<osg::DrawElementsUInt> viewExtentsIndices =
            new osg::DrawElementsUInt(GL_QUADS);
    viewExtentsIndices->push_back(0);
    viewExtentsIndices->push_back(1);
    viewExtentsIndices->push_back(3);
    viewExtentsIndices->push_back(2);

    viewExtentsIndices->push_back(4);
    viewExtentsIndices->push_back(5);
    viewExtentsIndices->push_back(7);
    viewExtentsIndices->push_back(6);

    viewExtentsIndices->push_back(8);
    viewExtentsIndices->push_back(9);
    viewExtentsIndices->push_back(11);
    viewExtentsIndices->push_back(10);

    osg::ref_ptr<osg::Geometry> viewExtentsGeom = new osg::Geometry;
    viewExtentsGeom->setVertexArray(viewExtentsVertices.get());
    viewExtentsGeom->addPrimitiveSet(viewExtentsIndices.get());

    osg::ref_ptr<osg::Geode> viewExtentsNode = new osg::Geode;
    viewExtentsNode->addDrawable(viewExtentsGeom.get());
    viewExtentsNode->getOrCreateStateSet()->setAttributeAndModes(polygonMode,
                                                                 osg::StateAttribute::ON);


    // node: pointcloud
    std::vector<Point3D> myVertices;
    std::vector<Point3D> myNormals;
    std::vector<Point2D> myTexCoords;
    std::vector<unsigned int> myIndices;

    createEarthSurfaceGeometry(24,36,
                               myVertices,
                               myNormals,
                               myTexCoords,
                               myIndices);

    // save ECEF vertices, normals, texcoords
    osg::ref_ptr<osg::Vec3Array> osgVertices = new osg::Vec3Array;
    osg::ref_ptr<osg::Vec3Array> osgNormals = new osg::Vec3Array;
    osg::ref_ptr<osg::Vec2Array> osgTexCoords = new osg::Vec2Array;

    for(int i=0; i < myVertices.size(); i++)
    {
        osgVertices->push_back(osg::Vec3(myVertices.at(i).x,
                                         myVertices.at(i).y,
                                         myVertices.at(i).z));

        osgNormals->push_back(osg::Vec3(myNormals.at(i).x,
                                        myNormals.at(i).y,
                                        myNormals.at(i).z));

        osgTexCoords->push_back(osg::Vec2(myTexCoords.at(i).x,
                                          myTexCoords.at(i).y));
    }

    // save ECEF tex indices
    osg::ref_ptr<osg::DrawElementsUInt> osgIndices =
            new osg::DrawElementsUInt(GL_TRIANGLES);

    osg::ref_ptr<osg::DrawElementsUInt> osgIndicesPointCloud =
            new osg::DrawElementsUInt(GL_POINTS, myIndices.size());

    for(int i=0; i < myIndices.size(); i++)
    {
        osgIndices->push_back(myIndices.at(i));
        osgIndicesPointCloud->push_back(myIndices.at(i));
    }

    // geometry: earth
    osg::ref_ptr<osg::Geometry> earthGeom = new osg::Geometry;

    if(!renderAsPointCloud)
    {
        earthGeom->setVertexArray(osgVertices.get());
        earthGeom->setNormalArray(osgNormals.get());
        earthGeom->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);
        earthGeom->addPrimitiveSet(osgIndices.get());
        osgUtil::SmoothingVisitor::smooth(*earthGeom);
    }
    else
    {
        earthGeom->setVertexArray(osgVertices.get());
        earthGeom->addPrimitiveSet(osgIndicesPointCloud.get());
    }

    // texture: earth
    osg::ref_ptr<osg::Image> earthMap = osgDB::readImageFile("earth_map.jpg");
    osg::ref_ptr<osg::Texture2D> earthTexture = new osg::Texture2D;
    earthTexture->setImage(earthMap.get());

    // note -- must set texture wrap mode to "CLAMP_TO_EDGE"
    // to avoid texture borders at the edge!
    if(!renderAsPointCloud)
    {
        earthTexture->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
        earthTexture->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);

        earthGeom->setTexCoordArray(0,osgTexCoords.get());
        earthGeom->getOrCreateStateSet()->setTextureAttributeAndModes(0,earthTexture.get());
    }

    // node: earth
    osg::ref_ptr<osg::Geode> myEarthNode = new osg::Geode;
    myEarthNode->addDrawable(earthGeom.get());

    // node: root
    osg::ref_ptr<osg::Group> myRootNode = new osg::Group;
//    myRootNode->addChild(myOriginNode.get());
    myRootNode->addChild(myEarthNode.get());
    //myRootNode->addChild(myFrustumNode.get());
    //myRootNode->addChild(viewExtentsNode.get());

    if(renderAsPointCloud)
    {
        osg::ref_ptr<osg::StateSet> rootState = myRootNode->getOrCreateStateSet();
        rootState->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    }



    // setup viewer
    osgViewer::Viewer viewer;
    viewer.setThreadingModel(osgViewer::ViewerBase::SingleThreaded);
    viewer.setUpViewInWindow(100,100,800,480);
    viewer.setSceneData(myRootNode.get());
    return viewer.run();
}


void createSphereSurfaceGeometry(int numRings, int sectorDivs,
                                 std::vector<Point3D> &myVertices,
                                 std::vector<Point3D> &myNormals,
                                 std::vector<Point2D> &myTexCoords,
                                 std::vector<unsigned int> &myIndices)
{
    double thetaStep = 180.0f/(numRings+1);
    double phiStep = 360.0f/sectorDivs;
    double radMeters = 5;

    for(int ring=0; ring < (numRings+2); ring++)
    {
        for(int sector=0; sector < (sectorDivs+1); sector++)
        {
            double myTheta = thetaStep*ring-90;
            double myPhi = phiStep*sector;

            if(myTheta > -90)
            {
                Point3D point0(cos(myTheta*K_PI/180.0f)*cos(myPhi*K_PI/180.0f)*radMeters,
                               cos(myTheta*K_PI/180.0f)*sin(myPhi*K_PI/180.0f)*radMeters,
                               sin(myTheta*K_PI/180.0f)*radMeters);

                Point3D point1(cos(myTheta*K_PI/180.0f)*cos((myPhi+phiStep)*K_PI/180.0f)*radMeters,
                               cos(myTheta*K_PI/180.0f)*sin((myPhi+phiStep)*K_PI/180.0f)*radMeters,
                               sin(myTheta*K_PI/180.0f)*radMeters);

                Point3D point2(cos((myTheta-thetaStep)*K_PI/180.0f)*cos((myPhi)*K_PI/180.0f)*radMeters,
                               cos((myTheta-thetaStep)*K_PI/180.0f)*sin((myPhi)*K_PI/180.0f)*radMeters,
                               sin((myTheta-thetaStep)*K_PI/180.0f)*radMeters);

                myVertices.push_back(point2);
                myNormals.push_back(point2);
                myIndices.push_back(myVertices.size()-1);

                myVertices.push_back(point1);
                myNormals.push_back(point1);
                myIndices.push_back(myVertices.size()-1);

                myVertices.push_back(point0);
                myNormals.push_back(point0);
                myIndices.push_back(myVertices.size()-1);

                myTexCoords.push_back(Point2D(myPhi/360.0f,((myTheta-thetaStep)+90.0)/180.0));
                myTexCoords.push_back(Point2D((myPhi+phiStep)/360.0f,(myTheta+90.0)/180.0));
                myTexCoords.push_back(Point2D(myPhi/360.0f,(myTheta+90.0)/180.0));

            }
        }
    }
}


bool createEarthSurfaceGeometry(int latSegments, int lonSegments,
                                std::vector<Point3D> &myVertices,
                                std::vector<Point3D> &myNormals,
                                std::vector<Point2D> &myTexCoords,
                                std::vector<unsigned int> &myIndices)
{
    Point3D pointECEF;
    double latStepSize = 180.0f / double(latSegments);
    double lonStepSize = 360.0f / double(lonSegments);

    if(latStepSize < 4 || lonStepSize < 4)
    {   return false;   }

    for(int i=0; i <= latSegments; i++)
    {
        for(int j=0; j <= lonSegments; j++)
        {
            double myLat = 90.0 - (i*latStepSize);
            double myLon = -180.0 + (j*lonStepSize);

            convLLAToECEF(PointLLA(myLat,myLon),pointECEF);
            myVertices.push_back(pointECEF);
            myNormals.push_back(myVertices.back());
            myTexCoords.push_back(Point2D(((myLon+180.0)/360.0),
                                          (myLat+90.0)/180.0));
        }
    }

    for(int i=0; i < latSegments; i++)
    {
        int pOffset = (lonSegments+1)*i;

        if(i != latSegments-1)
        {
            for(int j=pOffset; j < pOffset+lonSegments+1; j++)
            {
                myIndices.push_back(j);
                myIndices.push_back(j+lonSegments);
                myIndices.push_back(j+lonSegments+1);

                if(i > 0)
                {
                    myIndices.push_back(j);
                    myIndices.push_back(j+lonSegments+1);
                    myIndices.push_back(j+1);
                }
            }
        }
        else
        {
            for(int j=pOffset; j < pOffset+lonSegments+1; j++)
            {
                myIndices.push_back(j+lonSegments+1);
                myIndices.push_back(j+1);
                myIndices.push_back(j);
            }
        }
    }

    return true;
}

void convLLAToECEF(PointLLA const &pointLLA, Point3D &pointECEF)
{
    // conversion formula from...
    // hxxp://www.microem.ru/pages/u_blox/tech/dataconvert/GPS.G1-X-00006.pdf

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
}


void convECEFToLLA(Point3D const &pointECEF, PointLLA &pointLLA)
{
    // conversion formula from...
    // hxxp://www.microem.ru/pages/u_blox/tech/dataconvert/GPS.G1-X-00006.pdf

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
    pointLLA.lon = pointLLA.lon * 180.0/K_PI;
    pointLLA.lat = pointLLA.lat * 180.0/K_PI;
}
