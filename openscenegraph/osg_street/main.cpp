#include <QString>
#include <QFile>
#include <QTextStream>
#include <QDebug>

#include <osgViewer/Viewer>
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>
#include <osg/Program>
#include <osg/Shader>
#include <osg/Uniform>
#include <osg/CullFace>

#include <osmscout/Database.h>

#include <vector>

#include "Vec2.hpp"
#include "Vec3.hpp"

// PI!
#define K_PI 3.141592653589

// epsilon error
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


QString readFileAsQString(const QString &myFilePath)
{
    QFile myFile(myFilePath);
    myFile.open(QIODevice::ReadOnly);

    QTextStream textStream(&myFile);
    return textStream.readAll();
}


struct PointLLA
{
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

enum OutlineType
{
    OL_CENTER,
    OL_LEFT,
    OL_RIGHT
};

void GetStreetVx(std::vector<Vec3> &listVx);
void GetStreetVx(osmscout::Id,std::vector<Vec3> &listVx);

/*
void buildPolylineAsTriStrip(std::vector<Vec3> const &listPolylineVx,
                             double const polylineWidth,
                             std::vector<Vec3> &listVx,
                             std::vector<Vec2> &listTx)
{
    size_t numPts = listPolylineVx.size();
    size_t numOffsets = (numPts-1)*2;   // 2 for each edge
    std::vector<Vec3> listOffsetPtsL(numOffsets);
    std::vector<Vec3> listOffsetPtsR(numOffsets);
    std::vector<Vec2> listOffsetTxL(numOffsets);
    std::vector<Vec2> listOffsetTxR(numOffsets);

    Vec3 vecNormal;                     // vector originating at the center of the earth
                                        // (0,0,0) to a vertex on the way

    Vec3 vecDirn;                       // vector along a given segment on the way

    Vec3 vecOffset;                     // vector normal to both vecPlaneNormal and
                                        // vecAlongSegment (used to create offset)

    Vec3 vecOffsetL,vecOffsetR;         // offsets from way center

    // keep track of edge distances and total length
    double totalLength = 0;
    std::vector<double> listEdgeDists(numPts,0);

    // we offset the polyline on both sides of the
    // centerline and stitch the resulting shape together

    // to account for self intersections or gaps that occur in
    // the 'inner' or 'outer' offsets, we use the perpendicular
    // bisector of the adjoining edges to find the inner edge
    // intersection and straight lines to fill outer edge gaps
    // (this provides a bevel/boxed joint effect)

    // in the resulting triangle strip, each 'joint' created
    // from adjacent edges in the original polyline contains
    // one degenerate triangle

    // [TODO: in MapRenderer, discard duplicate adjacent vertex data]

    // for each segment, create dirn and offset vecs
    size_t k=0;
    double offsetLength = polylineWidth/2;
    for(size_t i=1; i < numPts; i++)   {
        // offset vertex coordinates
        vecNormal = listPolylineVx[i];
        vecDirn   = listPolylineVx[i]-listPolylineVx[i-1];
        vecOffset = vecDirn.Cross(vecNormal).Normalized();

        vecOffsetL = vecOffset.ScaledBy(offsetLength);
        vecOffsetR = vecOffsetL.ScaledBy(-1.0);

        listOffsetPtsL[k] = listPolylineVx[i-1]+vecOffsetL;
        listOffsetPtsR[k] = listPolylineVx[i-1]+vecOffsetR; k++;

        listOffsetPtsL[k] = listPolylineVx[i]+vecOffsetL;
        listOffsetPtsR[k] = listPolylineVx[i]+vecOffsetR;   k++;

        // edge length and total polyline length
        double edgeLength = vecDirn.Magnitude();
        listEdgeDists[i] = listEdgeDists[i-1]+edgeLength;
        totalLength += edgeLength;
    }

    k=0;
    for(size_t i=1; i < numPts; i++)   {
        // offset texture coordinates
        listOffsetTxL[k].x = 0.0;
        listOffsetTxR[k].x = 1.0;
        listOffsetTxL[k].y = listEdgeDists[i-1]/totalLength;
        listOffsetTxR[k].y = listOffsetTxL[k].y;  k++;

        listOffsetTxL[k].x = 0.0;
        listOffsetTxR[k].x = 1.0;
        listOffsetTxL[k].y = listEdgeDists[i]/totalLength;
        listOffsetTxR[k].y = listOffsetTxL[k].y;  k++;
    }

    if(numPts == 2)   {
        // TODO special case with single edge
        return;
    }
    else   {
        for(size_t i=1; i < numPts-1; i++)
        {
            size_t idx = (i*2)-2;   // first idx for prev edge offset

            // determine the angle between two adjacent edges
            Vec3 edgePrev = (listPolylineVx[i-1]-listPolylineVx[i]).Normalized();
            Vec3 edgeNext = (listPolylineVx[i+1]-listPolylineVx[i]).Normalized();
            Vec3 edgeBisect = edgePrev+edgeNext;
            double edgeBisectLength = edgeBisect.Magnitude();
//            std::cout << "edgeBisectLength: " << edgeBisectLength << "\n";

            // special case: collinear edges
            if(edgeBisectLength < 0.001)   {    // 1mm
                //wtf
                listOffsetPtsL[idx+1] = listOffsetPtsL[idx+2];//listOffsetPtsR[idx+1];
                listOffsetPtsR[idx+1] = listOffsetPtsR[idx+2];//listOffsetPtsR[idx+2];
                continue;
            }

            // |AxB| = |A|*|B|*sinTheta
            double sinTheta = (edgePrev.Cross(edgeBisect)).Magnitude()/
                    (edgePrev.Magnitude()*edgeBisectLength);

            // special case: edge doubles back on itself [bad data]
            if(sinTheta < K_EPS)   {
                listOffsetPtsL[idx+1] = listOffsetPtsR[idx+2];
                listOffsetPtsL[idx+2] = listOffsetPtsR[idx+1];
                continue;
            }

            qDebug() << "SINTHETA " << sinTheta << ", EDGEBISECTLENGTH " << edgeBisectLength;

            // get the vertex coincident with the xsec of adjacent inside edges
            Vec3 vecBisect = edgeBisect.Normalized().ScaledBy(offsetLength/sinTheta);
            vecBisect = listPolylineVx[i]+vecBisect;

            // determine which side (left or right) corresponds
            // to the inner and outer offsets and move vertices
            double distToXsecL = listOffsetPtsL[idx+1].Distance2To(vecBisect);
            double distToXsecR = listOffsetPtsR[idx+1].Distance2To(vecBisect);
            if(distToXsecL < distToXsecR)   {       // left is the inner offset
                listOffsetPtsL[idx+1] = vecBisect;
                listOffsetPtsL[idx+2] = vecBisect;
            }
            else   {                                // right is the inner offset
                listOffsetPtsR[idx+1] = vecBisect;
                listOffsetPtsR[idx+2] = vecBisect;
            }
        }
    }

    for(size_t i=0; i < listOffsetPtsL.size(); i++)
    {
        listVx.push_back(listOffsetPtsL[i]);
        listVx.push_back(listOffsetPtsR[i]);

        listTx.push_back(listOffsetTxL[i]);
        listTx.push_back(listOffsetTxR[i]);
    }
}
*/

void buildPolylineAsLineStrip(std::vector<Vec3> const &listPolylineVx,
                              std::vector<Vec3> &listVx,
                              std::vector<Vec2> &listTx)
{
    size_t numPts = listPolylineVx.size();
    double totalLength = 0;
    std::vector<double> listEdgeDists(numPts,0);

    for(size_t i=1; i < numPts; i++)   {
        // edge length and total polyline length
        double edgeLength = (listPolylineVx[i]-listPolylineVx[i-1]).Magnitude();
        listEdgeDists[i] = listEdgeDists[i-1]+edgeLength;
        totalLength += edgeLength;
    }

    listVx = listPolylineVx;
    size_t k=0;
    for(size_t i=0; i < numPts; i++)   {
        // offset texture coordinates
        listTx.push_back(Vec2(0.0,listEdgeDists[i]/totalLength));
    }
}

void buildPolylineAsTriStrip(std::vector<Vec3> const &listPolylineVx,
                             double const polylineWidth,
                             std::vector<Vec3> &listVx,
                             std::vector<Vec2> &listTx,
                             double &polylineLength)
{
    size_t numPts = listPolylineVx.size();
    size_t numOffsets = (numPts-1)*2;   // 2 for each edge
    std::vector<Vec3> listOffsetPtsL(numOffsets);
    std::vector<Vec3> listOffsetPtsR(numOffsets);
    std::vector<Vec2> listOffsetTxL(numOffsets);
    std::vector<Vec2> listOffsetTxR(numOffsets);

    Vec3 vecNormal;                     // vector originating at the center of the earth
                                        // (0,0,0) to a vertex on the way

    Vec3 vecDirn;                       // vector along a given segment on the way

    Vec3 vecOffset;                     // vector normal to both vecPlaneNormal and
                                        // vecAlongSegment (used to create offset)

    Vec3 vecOffsetL,vecOffsetR;         // offsets from way center

    // keep track of edge distances and total length
    double totalLength = 0;
    std::vector<double> listEdgeDists(numPts,0);

    // we offset the polyline on both sides of the
    // centerline and stitch the resulting shape together

    // to account for self intersections or gaps that occur in
    // the 'inner' or 'outer' offsets, we use the perpendicular
    // bisector of the adjoining edges to find the inner edge
    // intersection and straight lines to fill outer edge gaps
    // (this provides a bevel/boxed joint effect)

    // in the resulting triangle strip, each 'joint' created
    // from adjacent edges in the original polyline contains
    // one degenerate triangle

    // [TODO: in MapRenderer, discard duplicate adjacent vertex data]

    // for each segment, create dirn and offset vecs
    size_t k=0;
    double offsetLength = polylineWidth/2;
    for(size_t i=1; i < numPts; i++)   {
        // offset vertex coordinates
        vecNormal = listPolylineVx[i];
        vecDirn   = listPolylineVx[i]-listPolylineVx[i-1];
        vecOffset = vecDirn.Cross(vecNormal).Normalized();

        vecOffsetL = vecOffset.ScaledBy(offsetLength);
        vecOffsetR = vecOffsetL.ScaledBy(-1.0);

        listOffsetPtsL[k] = listPolylineVx[i-1]+vecOffsetL;
        listOffsetPtsR[k] = listPolylineVx[i-1]+vecOffsetR; k++;

        listOffsetPtsL[k] = listPolylineVx[i]+vecOffsetL;
        listOffsetPtsR[k] = listPolylineVx[i]+vecOffsetR;   k++;

        // edge length and total polyline length
        double edgeLength = vecDirn.Magnitude();
        listEdgeDists[i] = listEdgeDists[i-1]+edgeLength;
        totalLength += edgeLength;
    }

    k=0;
    for(size_t i=1; i < numPts; i++)   {
        // offset texture coordinates
        listOffsetTxL[k].x = 0.0;
        listOffsetTxR[k].x = 1.0;
        listOffsetTxL[k].y = listEdgeDists[i-1]/totalLength;
        listOffsetTxR[k].y = listOffsetTxL[k].y;  k++;

        listOffsetTxL[k].x = 0.0;
        listOffsetTxR[k].x = 1.0;
        listOffsetTxL[k].y = listEdgeDists[i]/totalLength;
        listOffsetTxR[k].y = listOffsetTxL[k].y;  k++;
    }

    if(numPts > 2)   {
        // we only adjust the join vertices
        // if there's more than one edge
        for(size_t i=1; i < numPts-1; i++)
        {
//            continue;
            size_t idx = (i*2)-2;   // first idx for prev edge offset

            // determine the angle between two adjacent edges
            Vec3 edgePrev = (listPolylineVx[i-1]-listPolylineVx[i]).Normalized();
            Vec3 edgeNext = (listPolylineVx[i+1]-listPolylineVx[i]).Normalized();
            Vec3 edgeBisect = edgePrev+edgeNext;
            double edgeBisectLength = edgeBisect.Magnitude();
            std::cout << "edgeBisectLength:" << edgeBisect.Magnitude() << ", ";

            // special case: collinear edges
            if(edgeBisectLength < 0.001)   {
                listOffsetPtsL[idx+1] = listOffsetPtsL[idx+2];//listOffsetPtsR[idx+1];
                listOffsetPtsR[idx+1] = listOffsetPtsR[idx+2];//listOffsetPtsR[idx+2];
                continue;
            }

            // |AxB| = |A|*|B|*sinTheta
            double sinTheta = (edgePrev.Cross(edgeBisect)).Magnitude()/
                    (edgePrev.Magnitude()*edgeBisectLength);

            std::cout << "sinTheta:" << sinTheta << ", ";

            // special case:
            // * extreme angle between segments
            // * edge doubles back on itself [bad data]
            if(sinTheta < 0.33)   {
                listOffsetPtsL[idx+1] = listOffsetPtsR[idx+2];
                listOffsetPtsL[idx+2] = listOffsetPtsR[idx+1];
                continue;
            }

            // get the vertex coincident with the xsec of adjacent inside edges
            Vec3 vecBisect = edgeBisect.Normalized().ScaledBy(offsetLength/sinTheta);
            std::cout << "vecBisect:" << vecBisect.Magnitude() << std::endl;
            vecBisect = listPolylineVx[i]+vecBisect;


            // determine which side (left or right) corresponds
            // to the inner and outer offsets and move vertices
            double distToXsecL = listOffsetPtsL[idx+1].Distance2To(vecBisect);
            double distToXsecR = listOffsetPtsR[idx+1].Distance2To(vecBisect);
            if(distToXsecL < distToXsecR)   {       // left is the inner offset
                listOffsetPtsL[idx+1] = vecBisect;
                listOffsetPtsL[idx+2] = vecBisect;
            }
            else   {                                // right is the inner offset
                listOffsetPtsR[idx+1] = vecBisect;
                listOffsetPtsR[idx+2] = vecBisect;
            }
        }
    }

    // save vertex data
    listVx.resize(listOffsetPtsL.size()*2);
    listTx.resize(listVx.size()); k=0;
    for(size_t i=0; i < listOffsetPtsL.size(); i++)
    {   // left
        listVx[k] = listOffsetPtsL[i];
        listTx[k] = listOffsetTxL[i];   k++;
        // right
        listVx[k] = listOffsetPtsR[i];
        listTx[k] = listOffsetTxR[i];   k++;
    }

    // save length
    polylineLength = totalLength;
}

void printVector(Vec3 const &myVector)
{
    qDebug() << "### > " << myVector.x
             << " " << myVector.y
             << " " << myVector.z;
}

int main(int argc, char *argv[])
{
    // set debug severity
    osg::setNotifyLevel(osg::ALWAYS);

    // add shaders to openscenegraph
    osg::ref_ptr<osg::Program> shProgram = new osg::Program;
    shProgram->setName("DefaultProgram");

    // vertex shader
    QString vShader = readFileAsQString("shaders/model_vert.glsl");
    shProgram->addShader(new osg::Shader(osg::Shader::VERTEX,vShader.toStdString()));

    // fragment shader
    QString fShader = readFileAsQString("shaders/model_frag.glsl");
    shProgram->addShader(new osg::Shader(osg::Shader::FRAGMENT,fShader.toStdString()));

    // [geometry]
    double polylineLength;
    std::vector<Vec3> vxArray,vxTriStrip;
    std::vector<Vec2> txTriStrip;
    GetStreetVx(8127449,vxArray);
    buildPolylineAsTriStrip(vxArray,120,vxTriStrip,txTriStrip,polylineLength);
//    buildPolylineAsLineStrip(vxArray,vxTriStrip,txTriStrip);

    for(size_t i=0; i < vxTriStrip.size(); i++)
    {
        printVector(vxTriStrip[i]);
    }

    osg::ref_ptr<osg::Vec3Array> listVx = new osg::Vec3Array(vxTriStrip.size());
    osg::ref_ptr<osg::Vec2Array> listTx = new osg::Vec2Array(txTriStrip.size());
    for(size_t i=0; i < listVx->size(); i++)   {
        listVx->at(i).x() = vxTriStrip[i].x - vxTriStrip[0].x;
        listVx->at(i).y() = vxTriStrip[i].y - vxTriStrip[0].y;
        listVx->at(i).z() = vxTriStrip[i].z - vxTriStrip[0].z;

        listTx->at(i).x() = txTriStrip[i].x;
        listTx->at(i).y() = txTriStrip[i].y;
    }

    osg::ref_ptr<osg::DrawElementsUInt> listIx = new osg::DrawElementsUInt(GL_LINE_STRIP);
    for(size_t i=0; i < listVx->size(); i++)   {
        listIx->push_back(i);
    }

    osg::ref_ptr<osg::Geometry> gmStreet = new osg::Geometry;
    gmStreet->setVertexArray(listVx);
    gmStreet->setNormalArray(listVx);
    gmStreet->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);
    gmStreet->setTexCoordArray(1,listTx);
    gmStreet->addPrimitiveSet(listIx);

    // [geode]
    osg::ref_ptr<osg::Geode> gdStreet = new osg::Geode;
    osg::StateSet *ss = gdStreet->getOrCreateStateSet();
    ss->setAttributeAndModes(shProgram,osg::StateAttribute::ON);
    gdStreet->addDrawable(gmStreet);

    // [root]
    osg::ref_ptr<osg::Group> groupRoot = new osg::Group;
    ss = groupRoot->getOrCreateStateSet();
    ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    groupRoot->addChild(gdStreet);

    // viewer
    osgViewer::Viewer viewer;
    viewer.setThreadingModel(osgViewer::ViewerBase::SingleThreaded);
    viewer.setUpViewInWindow(100,100,800,480);
    viewer.setSceneData(groupRoot.get());
//    viewer.getCamera()->setClearColor(osg::Vec4(0,0,0,1));

    osgViewer::Viewer::Windows windows;
    viewer.getWindows(windows);
    for(osgViewer::Viewer::Windows::iterator itr = windows.begin();
        itr != windows.end(); ++itr)   {
        (*itr)->getState()->setUseModelViewAndProjectionUniforms(true);
        (*itr)->getState()->setUseVertexAttributeAliasing(true);
    }

    return viewer.run();
}

void GetStreetVx(osmscout::Id wayId,
                 std::vector<Vec3> &listVx)
{
    bool opOk = false;
    osmscout::DatabaseParameter databaseParam;
    osmscout::Database database(databaseParam);

    opOk = database.Open("/home/preet/Documents/maps/openstreetmap/london_render");
    if(!opOk)   {   std::cout << "ERROR Opening database\n";   return; }

    osmscout::WayRef wayRef;
    opOk = database.GetWay(wayId,wayRef);
    if(!opOk)   {   std::cout << "ERROR Finding way " << size_t(wayId) <<"\n";   return; }

    listVx.clear();
    for(size_t i=0; i < wayRef->nodes.size(); i++)   {
        PointLLA pointLLA(wayRef->nodes[i].GetLat(),
                          wayRef->nodes[i].GetLon());
        listVx.push_back(convLLAToECEF(pointLLA));
    }

    database.Close();
}

void GetStreetVx(std::vector<Vec3> &listVx)
{
    std::vector<PointLLA> wayPointsLLA;

    // WAY ID 22688985
//    wayPointsLLA.push_back(PointLLA(43.6762198, -79.3998242));
//    wayPointsLLA.push_back(PointLLA(43.6763392, -79.3994667));
//    wayPointsLLA.push_back(PointLLA(43.6766179, -79.3981721));

    // ??
//    wayPointsLLA.push_back(PointLLA(43.67622, -79.399824));
//    wayPointsLLA.push_back(PointLLA(43.676339, -79.399467));
//    wayPointsLLA.push_back(PointLLA(43.676618, -79.398172));
    wayPointsLLA.push_back(PointLLA(43.676987, -79.398323));
    wayPointsLLA.push_back(PointLLA(43.677038, -79.398343));
    wayPointsLLA.push_back(PointLLA(43.676987, -79.398323));
//    wayPointsLLA.push_back(PointLLA(43.676618, -79.398172));
//    wayPointsLLA.push_back(PointLLA(43.676636, -79.397989));
//    wayPointsLLA.push_back(PointLLA(43.676556, -79.397963));
//    wayPointsLLA.push_back(PointLLA(43.676244, -79.397838));
//    wayPointsLLA.push_back(PointLLA(43.675483, -79.397543));
//    wayPointsLLA.push_back(PointLLA( 43.675437, -79.397525));

    // WAY ID 69844700
//    wayPointsLLA.push_back(PointLLA(43.6690187, -79.3843227));
//    wayPointsLLA.push_back(PointLLA(43.6689848, -79.3843065));
//    wayPointsLLA.push_back(PointLLA(43.668562, -79.3841442));
//    wayPointsLLA.push_back(PointLLA(43.6684823, -79.3841087));
//    wayPointsLLA.push_back(PointLLA(43.6687109, -79.383001));
//    wayPointsLLA.push_back(PointLLA(43.6692003, -79.3832128));
//    wayPointsLLA.push_back(PointLLA(43.669247, -79.383233));

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

    // STELVIO PASS ITALY
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

    // convert to ECEF
    listVx.resize(wayPointsLLA.size());
    Vec3 xlateVec = convLLAToECEF(wayPointsLLA[0]);
    for(size_t i=0; i < wayPointsLLA.size(); i++)   {
        listVx[i] = convLLAToECEF(wayPointsLLA[i]);     // convert from LLA
//        listVx[i] = listVx[i]-xlateVec;                 // xlate to center
    }
}

















