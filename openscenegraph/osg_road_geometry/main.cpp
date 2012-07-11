// sys includes
#include <math.h>
#include <vector>
#include <iostream>
#include <sys/time.h>

//#include <QDebug>

// osg includes
#include <osg/Geometry>
#include <osg/PolygonMode>
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>
#include <osg/Geode>
#include <osgViewer/Viewer>

// PI!
#define K_PI 3.141592653589

// WGS84 ellipsoid parameters (http://en.wikipedia.org/wiki/WGS_84)
#define ELL_SEMI_MAJOR 6378137.0            // meters
#define ELL_SEMI_MINOR 6356752.3142         // meters
#define ELL_INV_F 1/298.257223563

// WGS84 ellipse eccentricity
double g_ELL_ecc_exp2;

// geographic coordinate system
class PointLLA
{
public:

    PointLLA(double myLat, double myLon, double myAlt)
    {   lon = myLon;   lat = myLat;    alt = myLat;   }

    PointLLA(double myLat, double myLon)
    {   lon = myLon;   lat = myLat;    alt = 0;   }

    double lon;
    double lat;
    double alt;
};

// cartesian coordinate system
class Point3D
{
public:
    Point3D()
    {   x = 0;   y = 0;   z = 0;   }

    Point3D(double myX, double myY, double myZ)
    {   x = myX;   y = myY;   z = myZ;   }

    double x;
    double y;
    double z;
};

void convLLAToECEF(const PointLLA &pointGCS, Point3D &pointECEF);
void convWayPathToOffsets(osg::Vec3Array const * listWayPoints,
                          osg::Vec3Array * listOffsetPointsA,
                          osg::Vec3Array * listOffsetPointsB,
                          osg::Vec3 const &pointEarthCenter,
                          double const &lineWidth);
void convWayOffsetsTo_TRI_STRIP(const osg::Vec3Array *offsetArrayA,
                                const osg::Vec3Array *offsetArrayB,
                                osg::Vec3Array *wayTriStrip);

int main()
{   
    // define WGS84 eccentricity
    g_ELL_ecc_exp2 = (2*ELL_INV_F) - (ELL_INV_F*ELL_INV_F);

    // get some LLA coordinates
    std::vector<PointLLA> wayPointsLLA;

    // WAY ID 69844700
    wayPointsLLA.push_back(PointLLA(43.6690187, -79.3843227));
    wayPointsLLA.push_back(PointLLA(43.6689848, -79.3843065));
    wayPointsLLA.push_back(PointLLA(43.668562, -79.3841442));
    wayPointsLLA.push_back(PointLLA(43.6684823, -79.3841087));
    wayPointsLLA.push_back(PointLLA(43.6687109, -79.383001));
    wayPointsLLA.push_back(PointLLA(43.6692003, -79.3832128));
    wayPointsLLA.push_back(PointLLA(43.669247, -79.383233));

    // DENVER PLACE, SCARBOROUGH (LOOP)
//    wayPointsLLA.push_back(PointLLA(43.7682636, -79.2488975));
//    wayPointsLLA.push_back(PointLLA(43.7682264, -79.2490949));
//    wayPointsLLA.push_back(PointLLA(43.768183, -79.2491464));
//    wayPointsLLA.push_back(PointLLA(43.7681148, -79.2491722));
//    wayPointsLLA.push_back(PointLLA(43.7680529, -79.2491035));
//    wayPointsLLA.push_back(PointLLA(43.7680281, -79.2489834));
//    wayPointsLLA.push_back(PointLLA(43.7680529, -79.248889));
//    wayPointsLLA.push_back(PointLLA(43.7681768, -79.2488546));
//    wayPointsLLA.push_back(PointLLA(43.7682636, -79.2488975));

//    // SEGMENT OF STELVIO PASS ITALY
//    wayPointsLLA.push_back(PointLLA(46.5315501, 10.4564453));
//    wayPointsLLA.push_back(PointLLA(46.5315220, 10.4564494));
//    wayPointsLLA.push_back(PointLLA(46.5314896, 10.4564166));
//    wayPointsLLA.push_back(PointLLA(46.5313572, 10.4561587));
//    wayPointsLLA.push_back(PointLLA(46.5312671, 10.4559397));
//    wayPointsLLA.push_back(PointLLA(46.5311671, 10.4558128));
//    wayPointsLLA.push_back(PointLLA(46.5310362, 10.4557330));
//    wayPointsLLA.push_back(PointLLA(46.5304194, 10.4555181));
//    wayPointsLLA.push_back(PointLLA(46.5301335, 10.4552397));
//    wayPointsLLA.push_back(PointLLA(46.5300884, 10.4552233));
//    wayPointsLLA.push_back(PointLLA(46.5300068, 10.4552315));
//    wayPointsLLA.push_back(PointLLA(46.5298772, 10.4552622));
//    wayPointsLLA.push_back(PointLLA(46.5296026, 10.4553093));
//    wayPointsLLA.push_back(PointLLA(46.5294699, 10.4553312));
//    wayPointsLLA.push_back(PointLLA(46.5294350, 10.4553525));
//    wayPointsLLA.push_back(PointLLA(46.5294192, 10.4553885));
//    wayPointsLLA.push_back(PointLLA(46.5294192, 10.4554343));
//    wayPointsLLA.push_back(PointLLA(46.5294328, 10.4554622));
//    wayPointsLLA.push_back(PointLLA(46.5294598, 10.4554704));
//    wayPointsLLA.push_back(PointLLA(46.5299544, 10.4553508));
//    wayPointsLLA.push_back(PointLLA(46.5299746, 10.4553508));
//    wayPointsLLA.push_back(PointLLA(46.5299904, 10.4553705));
//    wayPointsLLA.push_back(PointLLA(46.5300006, 10.4554114));
//    wayPointsLLA.push_back(PointLLA(46.5299938, 10.4554458));
//    wayPointsLLA.push_back(PointLLA(46.5299713, 10.4554785));
//    wayPointsLLA.push_back(PointLLA(46.5299082, 10.4555015));
//    wayPointsLLA.push_back(PointLLA(46.5295071, 10.4555981));
//    wayPointsLLA.push_back(PointLLA(46.5294508, 10.4556325));
//    wayPointsLLA.push_back(PointLLA(46.5294226, 10.4556734));
//    wayPointsLLA.push_back(PointLLA(46.5294238, 10.4557127));
//    wayPointsLLA.push_back(PointLLA(46.5294395, 10.4557373));
//    wayPointsLLA.push_back(PointLLA(46.5294711, 10.4557422));
//    wayPointsLLA.push_back(PointLLA(46.5295758, 10.4557258));
//    wayPointsLLA.push_back(PointLLA(46.5298124, 10.4556587));
//    wayPointsLLA.push_back(PointLLA(46.5298823, 10.4556701));
//    wayPointsLLA.push_back(PointLLA(46.5299431, 10.4556865));
//    wayPointsLLA.push_back(PointLLA(46.5300006, 10.4557438));
//    wayPointsLLA.push_back(PointLLA(46.5301110, 10.4558928));
//    wayPointsLLA.push_back(PointLLA(46.5302901, 10.4561352));
//    wayPointsLLA.push_back(PointLLA(46.5306033, 10.4564070));

    // translate them (roughly) to the origin
    Point3D translatedCenter;
    convLLAToECEF(wayPointsLLA.at(5),translatedCenter);
    translatedCenter.x *= 0;
    translatedCenter.y *= 0;
    translatedCenter.z *= 0;

    osg::Vec3 pointEarthCenter(translatedCenter.x,
                               translatedCenter.y,
                               translatedCenter.z);

    // convert them to cartesian coordinates
    osg::ref_ptr<osg::Vec3Array> wayPoints3D = new osg::Vec3Array;
    for(int i=0; i < wayPointsLLA.size(); i++)
    {
        Point3D myWayPoint;
        convLLAToECEF(wayPointsLLA.at(i),myWayPoint);

        wayPoints3D->push_back(osg::Vec3(myWayPoint.x + translatedCenter.x,
                                         myWayPoint.y + translatedCenter.y,
                                         myWayPoint.z + translatedCenter.z));
    }



    // create origin
    osg::ref_ptr<osg::ShapeDrawable> myOriginX = new osg::ShapeDrawable;
    myOriginX->setShape(new osg::Box(osg::Vec3(5,0,0),10,0.5,0.5));
    myOriginX->setColor(osg::Vec4(1,0,0,1));

    osg::ref_ptr<osg::ShapeDrawable> myOriginY = new osg::ShapeDrawable;
    myOriginY->setShape(new osg::Box(osg::Vec3(0,5,0),0.5,10,0.5));
    myOriginY->setColor(osg::Vec4(0,1,0,1));

    osg::ref_ptr<osg::ShapeDrawable> myOriginZ = new osg::ShapeDrawable;
    myOriginZ->setShape(new osg::Box(osg::Vec3(0,0,5),0.5,0.5,10));
    myOriginZ->setColor(osg::Vec4(0,0,1,1));

    osg::ref_ptr<osg::Vec3Array> earthCenterDirn = new osg::Vec3Array;
    osg::Vec3 centerDirn = pointEarthCenter;
    centerDirn.normalize(); centerDirn *= 10;
    earthCenterDirn->push_back(osg::Vec3(0,0,0));
    earthCenterDirn->push_back(centerDirn);

    osg::ref_ptr<osg::Geometry> myEarthCenterDirn = new osg::Geometry;
    myEarthCenterDirn->setVertexArray(earthCenterDirn);
    myEarthCenterDirn->addPrimitiveSet(new osg::DrawArrays(GL_LINE_STRIP,0,earthCenterDirn->size()));

    osg::ref_ptr<osg::ShapeDrawable> myOriginSphere = new osg::ShapeDrawable;
    myOriginSphere->setShape(new osg::Sphere(osg::Vec3(0,0,0),0.5));
    myOriginSphere->setColor(osg::Vec4(0.7,0.7,0.7,1));

    osg::ref_ptr<osg::Geode> myOriginNode = new osg::Geode;
    myOriginNode->addDrawable(myOriginX.get());
    myOriginNode->addDrawable(myOriginY.get());
    myOriginNode->addDrawable(myOriginZ.get());
    myOriginNode->addDrawable(myOriginSphere.get());
    myOriginNode->addDrawable(myEarthCenterDirn.get());

    // create path
    double lineWidth = 4;
    osg::ref_ptr<osg::Vec3Array> listOffsetPointsA = new osg::Vec3Array;
    osg::ref_ptr<osg::Vec3Array> listOffsetPointsB = new osg::Vec3Array;
    osg::ref_ptr<osg::Vec3Array> listTriStripVerts = new osg::Vec3Array;

    convWayPathToOffsets(wayPoints3D.get(),
                         listOffsetPointsA.get(),
                         listOffsetPointsB.get(),
                         pointEarthCenter,lineWidth);

    convWayOffsetsTo_TRI_STRIP(listOffsetPointsA.get(),
                               listOffsetPointsB.get(),
                               listTriStripVerts.get());

    osg::ref_ptr<osg::Geometry> myWayPath = new osg::Geometry;
    myWayPath->setVertexArray(listTriStripVerts.get());
    myWayPath->addPrimitiveSet(new osg::DrawArrays(GL_TRIANGLE_STRIP,0,listTriStripVerts->size()));

    osg::ref_ptr<osg::Geode> myWaysNode = new osg::Geode;
    myWaysNode->addDrawable(myWayPath.get());

    // create root
    osg::ref_ptr<osg::Group> myRootNode = new osg::Group;
    //myRootNode->addChild(myOriginNode.get());
    myRootNode->addChild(myWaysNode.get());

    // render mode
    osg::PolygonMode *polygonMode = new osg::PolygonMode();
    polygonMode->setMode(osg::PolygonMode::FRONT_AND_BACK,osg::PolygonMode::LINE);
    myRootNode->getOrCreateStateSet()->setAttributeAndModes(polygonMode,osg::StateAttribute::ON);
    myRootNode->getOrCreateStateSet()->setMode(GL_LIGHTING,osg::StateAttribute::OFF);

    // start viewer
    osgViewer::Viewer viewer;
    viewer.setThreadingModel(osgViewer::ViewerBase::SingleThreaded);
    viewer.setUpViewInWindow(100,100,800,480);
    viewer.setSceneData(myRootNode.get());
    return viewer.run();
}

// convert a point in the current geographic coordinate system
// to its equivalent point in the Earth-Centered Earth-Fixed
// cartesian coordinate system
void convLLAToECEF(PointLLA const &pointGCS, Point3D &pointECEF)
{
    // http://www.ordnancesurvey.co.uk/oswebsite/gps/docs/...
    // ...A_Guide_to_Coordinate_Systems_in_Great_Britain.pdf

    // remember to convert deg->rad
    double sinLat = sin(pointGCS.lat * K_PI/180.0f);
    double sinLon = sin(pointGCS.lon * K_PI/180.0f);
    double cosLat = cos(pointGCS.lat * K_PI/180.0f);
    double cosLon = cos(pointGCS.lon * K_PI/180.0f);

    // v = radius of curvature (meters)
    double v = ELL_SEMI_MAJOR / (sqrt(1-(g_ELL_ecc_exp2*sinLat*sinLat)));

    pointECEF.x = (v + pointGCS.alt) * cosLat * cosLon;
    pointECEF.y = (v + pointGCS.alt) * cosLat * sinLon;
    pointECEF.z = ((1-g_ELL_ecc_exp2)*v + pointGCS.alt)*sinLat;
}

void convWayPathToOffsets(const osg::Vec3Array *listWayPoints,
                          osg::Vec3Array *listOffsetPointsA,
                          osg::Vec3Array *listOffsetPointsB,
                          const osg::Vec3 &pointEarthCenter,
                          const double &lineWidth)
{
    osg::Vec3 vecOffset;            // vector in the direction of the line segment's offset

    osg::Vec3 vecWaySurface;        // vector along the current line segment of the way

    osg::Vec3 vecEarthCenter;       // vector from a point on the current line segment to
                                    // earth's center -- note that we're in a different
                                    // reference frame since the geometry as shifted to
                                    // account for position issues

    std::cout << "listWayPoints size: " << listWayPoints->size()
              << std::endl;

    for(int i=0; i < listWayPoints->size(); i++)
    {
        std::cout << i << "  " << "(" << listWayPoints->at(i).x()
                 << "," << listWayPoints->at(i).y()
                 << "," << listWayPoints->at(i).z()
                 << ")" << std::endl;
    }


    int listSize = listWayPoints->size();

    // offset the first point in the wayPoint list
    // using the normal to the first line segment
    vecEarthCenter = pointEarthCenter-listWayPoints->at(0);
    vecWaySurface = listWayPoints->at(1)-listWayPoints->at(0);

    vecOffset = (vecWaySurface^vecEarthCenter);
    vecOffset.normalize();
    vecOffset *= lineWidth*0.5;

    listOffsetPointsA->push_back(listWayPoints->at(0) + vecOffset);
    listOffsetPointsB->push_back(listWayPoints->at(0) - vecOffset);

    // points in the middle of the wayPoint list have two
    // offsets -- one for each the preceding segment and
    // one for the following segment
    for(int i=1; i < listSize-1; i++)
    {
        // vecOffset remains the same for the preceding segment
        listOffsetPointsA->push_back(listWayPoints->at(i) + vecOffset);
        listOffsetPointsB->push_back(listWayPoints->at(i) - vecOffset);

        // vecOffset is different for the following segment
        vecEarthCenter = pointEarthCenter-listWayPoints->at(i);
        vecWaySurface = listWayPoints->at(i+1)-listWayPoints->at(i);

        vecOffset = (vecWaySurface^vecEarthCenter);
        vecOffset.normalize();
        vecOffset *= lineWidth*0.5;

        listOffsetPointsA->push_back(listWayPoints->at(i) + vecOffset);
        listOffsetPointsB->push_back(listWayPoints->at(i) - vecOffset);
    }

    // offset the last point in the wayPoint list
    // using the normal to the last line segment
    vecEarthCenter = pointEarthCenter-listWayPoints->at(listSize-1);
    vecWaySurface = listWayPoints->at(listSize-1) - listWayPoints->at(listSize-2);

    vecOffset = (vecWaySurface^vecEarthCenter);
    vecOffset.normalize();
    vecOffset *= lineWidth*0.5;

    listOffsetPointsA->push_back(listWayPoints->at(listSize-1) + vecOffset);
    listOffsetPointsB->push_back(listWayPoints->at(listSize-1) - vecOffset);
}

void convWayOffsetsTo_TRI_STRIP(osg::Vec3Array const * offsetArrayA,
                                osg::Vec3Array const * offsetArrayB,
                                osg::Vec3Array * wayTriStrip)
{
    std::cout << "offsetArray size: " << offsetArrayA->size()
              << std::endl;

    // define triangle strip
    for(int i=0; i < offsetArrayA->size(); i++)
    {
        wayTriStrip->push_back(offsetArrayA->at(i));
        wayTriStrip->push_back(offsetArrayB->at(i));
    }

    for(int i=0; i < wayTriStrip->size(); i++)
    {
        std::cout << i << "  " << "(" << wayTriStrip->at(i).x()
                 << "," << wayTriStrip->at(i).y()
                 << "," << wayTriStrip->at(i).z()
                 << ")" << std::endl;
    }
}
