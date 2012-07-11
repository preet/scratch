// sys includes
#include <math.h>
#include <vector>
#include <iostream>
#include <sys/time.h>

// osg includes
#include <osg/Geometry>
#include <osg/MatrixTransform>
#include <osg/Drawable>
#include <osg/ShapeDrawable>
#include <osg/Geode>
#include <osgUtil/Tessellator>
#include <osg/PolygonMode>
#include <osg/Vec3>
#include <osgViewer/Viewer>
#include <osg/PrimitiveSet>

// libosmscout includes
#include <osmscout/import/Import.h>
#include <osmscout/Database.h>
#include <osmscout/StyleConfigLoader.h>
#include <osmscout/Util.h>

// other
#include "Vec2.hpp"
#include "Vec3.hpp"

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

bool calcPolyIsCCW(const std::vector<Vec2> &listPoints)
{
    // based on  hxxp://en.wikipedia.org/wiki/Curve_orientation
    // and       hxxp://local.wasp.uwa.edu.au/~pbourke/geometry/clockwise/

    // note: poly must be simple

    int ptIdx = 0;
    for(int i=1; i < listPoints.size(); i++)
    {
        // find the point with the smallest y value,
        if(listPoints[i].y < listPoints[ptIdx].y)
        {   ptIdx = i;   }

        // if y values are equal save the point with greatest x
        else if(listPoints[i].y == listPoints[ptIdx].y)
        {
            if(listPoints[i].x < listPoints[ptIdx].x)
            {   ptIdx = i;   }
        }
    }

    int prevIdx = (ptIdx == 0) ? listPoints.size()-1 : ptIdx-1;
    int nextIdx = (ptIdx == listPoints.size()-1) ? 0 : ptIdx+1;

    double signedArea = (listPoints[ptIdx].x-listPoints[prevIdx].x) *
                        (listPoints[nextIdx].y-listPoints[ptIdx].y) -
                        (listPoints[ptIdx].y-listPoints[prevIdx].y) *
                        (listPoints[nextIdx].x-listPoints[ptIdx].x);

    return (signedArea > 0.0);
}

osg::Vec3 convVec3ToOsgVec3(const Vec3 &myVector)
{   return osg::Vec3(myVector.x,myVector.y,myVector.z);   }

struct AreaRenderData
{
    std::vector<Vec3>                 listOuterPts;
    std::vector<std::vector<Vec3> >   listListInnerPts;
};

struct RelAreaRenderData
{
    std::vector<AreaRenderData>       listAreaData;
};

int main()
{   
    osmscout::DatabaseParameter osmDatabaseParam;
    osmscout::Database osmDatabase(osmDatabaseParam);
    if(osmDatabase.Open("/home/preet/Documents/Maps/render/multipolygon"))
    {   std::cout << "Opened Database Successfully!" << std::endl;   }

    osmscout::RelationRef relRef;
    osmDatabase.GetRelation(203,relRef);


    RelAreaRenderData renderData;

    // we expect each osmscout multipolygon relation to
    // have multiple outer and inner polygons defined
    // heirarchically

    // roles[0-N].ring: 0,1,0,1,1,2,0,1,2,3
    // {0,1} {0,1,1} {2} {0,1} {2,3} -> five polys
    // let 0,2,4,... represent filled polys
    // let 1,3,5,... represent corresponding
    // clips or boolean subtractions from 0,2,4

    // we can interpret this as
    // {0,1} {0,1,1} {1} {0,1} {0,1}
    // as long as we apply the same fill/related
    // attributes for the entire relation
    for(int i=0; i < relRef->roles.size(); i++)
    {   // look for outerRing
        if(relRef->roles[i].ring%2 == 0 &&
                relRef->roles[i].GetType() != osmscout::typeIgnore)
        {
            std::vector<Vec2>                 listOuterPts;
            std::vector<std::vector<Vec2> >   listListInnerPts;

            osmscout::TypeId roleType = relRef->roles[i].GetType();
            std::cout << "Outer Role Type: " << osmDatabase.GetTypeConfig()->GetTypeInfo(roleType).GetName() << std::endl;

            // save outerRing nodes
            for(int r=0; r < relRef->roles[i].nodes.size(); r++)
            {
                Vec2 myPt(relRef->roles[i].nodes[r].GetLon(),
                          relRef->roles[i].nodes[r].GetLat());
                listOuterPts.push_back(myPt);
            }
            i++;

            // look for innerRings belonging to outerRing
            while(i < relRef->roles.size())
            {
                osmscout::TypeId roleType = relRef->roles[i].GetType();
                std::cout << "Inner Role Type: " << osmDatabase.GetTypeConfig()->GetTypeInfo(roleType).GetName() << std::endl;

                // iterate until all innerRing data is saved
                if(relRef->roles[i].ring%2 == 1)
                {   // save innerRing nodes
                    std::vector<Vec2> listInnerPts;
                    for(int r=0; r < relRef->roles[i].nodes.size(); r++)
                    {
                        Vec2 myPt(relRef->roles[i].nodes[r].GetLon(),
                                  relRef->roles[i].nodes[r].GetLat());
                        listInnerPts.push_back(myPt);
                    }
                    listListInnerPts.push_back(listInnerPts);
                    i++;
                }
                else    // means ringId is outerRing
                {   i--;   break;   }
            }

            // build AreaRenderData
            AreaRenderData areaData;

            // outer points
            for(int i=0; i < listOuterPts.size(); i++)   {
                Vec3 ecefPt = convLLAToECEF(PointLLA(listOuterPts[i].y,
                                                     listOuterPts[i].x,0.0));
                areaData.listOuterPts.push_back(ecefPt);
            }

            // fix cw/ccw: expect outer pts to be CCW
            if(!calcPolyIsCCW(listOuterPts))   {
                std::reverse(areaData.listOuterPts.begin(),
                             areaData.listOuterPts.end());
            }

            // inner points
            areaData.listListInnerPts.resize(listListInnerPts.size());
            for(int i=0; i < listListInnerPts.size(); i++)   {
                for(int j=0; j < listListInnerPts[i].size(); j++)   {
                    Vec3 ecefPt = convLLAToECEF(PointLLA(listListInnerPts[i][j].y,
                                                         listListInnerPts[i][j].x,0.0));
                    areaData.listListInnerPts[i].push_back(ecefPt);
                }

                // fix cw/ccw: expect inner pts to be CW
                if(calcPolyIsCCW(listListInnerPts[i]))   {
                    std::reverse(areaData.listListInnerPts[i].begin(),
                                 areaData.listListInnerPts[i].end());
                }
            }

            // save
            renderData.listAreaData.push_back(areaData);
        }
    }

    //
    osg::ref_ptr<osg::Group> nodeRoot = new osg::Group;
    osg::Vec3 offsetVec = convVec3ToOsgVec3(renderData.listAreaData[0].listOuterPts[0]);
    osg::ref_ptr<osg::Vec3Array> areaVerts = new osg::Vec3Array;
    osg::ref_ptr<osg::Vec3Array> areaNorms = new osg::Vec3Array;

    // stores the vertex indxs for each simple poly
    // in the relation -- lsPolys[0] is the outer poly
    // and all subsequent entries are inner polys
    std::vector<unsigned int> lsPolys;

    //
    for(int i=0; i < renderData.listAreaData.size(); i++)
    {
        std::vector<Vec3> const &lsOuterPts =
                renderData.listAreaData[i].listOuterPts;

        std::cout << "outerpts " << lsOuterPts.size() << std::endl;

        std::vector<std::vector<Vec3> > const &lsLsInnerPts =
                renderData.listAreaData[i].listListInnerPts;

        for(int j=0; j < lsOuterPts.size(); j++)
        {   areaVerts->push_back(convVec3ToOsgVec3(lsOuterPts[j]));   }
        lsPolys.push_back(lsOuterPts.size());

        for(int j=0; j < lsLsInnerPts.size(); j++)   {
            for(int k=0; k < lsLsInnerPts[j].size(); k++)
            {   areaVerts->push_back(convVec3ToOsgVec3(lsLsInnerPts[j][k]));   }
            lsPolys.push_back(lsPolys.back() + lsLsInnerPts[j].size());
        }
    }

    // normals
    for(int i=0; i < areaVerts->size(); i++)   {
        osg::Vec3 areaNorm = areaVerts->at(i);
        areaNorm.normalize();
        areaNorms->push_back(areaNorm);
    }

    std::cerr << "we have " << areaVerts->size() << " vertices" << std::endl;

    // offsetFix
    for(int i=0; i < areaVerts->size(); i++)
    {   areaVerts->at(i) -= offsetVec;   }

    //
    osg::ref_ptr<osg::Geometry> geomArea = new osg::Geometry;
    geomArea->setVertexArray(areaVerts.get());
    geomArea->setNormalArray(areaNorms.get());
    geomArea->setNormalBinding(osg::Geometry::BIND_OVERALL);

    osgUtil::Tessellator areaTess;
    areaTess.setTessellationType(osgUtil::Tessellator::TESS_TYPE_GEOMETRY);
    areaTess.setWindingType(osgUtil::Tessellator::TESS_WINDING_POSITIVE);
    areaTess.setTessellationNormal(offsetVec);

    for(int i=0; i < lsPolys.size(); i++)   {
        unsigned int vStart,vNum;
        vStart = (i==0) ? 0 : lsPolys[i-1];
        vNum = (lsPolys[i]-1-vStart)+1;
        geomArea->addPrimitiveSet(new osg::DrawArrays(GL_TRIANGLE_FAN,vStart,vNum));
    }
    areaTess.retessellatePolygons(*geomArea);

    // unable to make tessellation work manually?
//    areaTess.beginTessellation();
//    for(int i=0; i < lsPolys.size(); i++)   {
//        unsigned int vStart,vEnd;
//        vStart = (i==0) ? 0 : lsPolys[i-1];
//        vEnd = lsPolys[i]-1;
//        areaTess.beginContour();
//        for(int j=vStart; j <= vEnd; j++)   {
//            osg::Vec3 * areaVertex = &(areaVerts->at(j));
//            areaTess.addVertex(areaVertex);
//        }
//        areaTess.endContour();
//    }
//    areaTess.endTessellation();

//    osg::Geometry::PrimitiveSetList areaPrimSetList = areaTess.getContours();
//    std::cout << "set list is " << geomArea->getPrimitiveSetList().size() << " sets" << std::endl;

    //
    osg::ref_ptr<osg::Geode> geodeArea = new osg::Geode;
    geodeArea->addDrawable(geomArea.get());

    // transform
    osg::ref_ptr<osg::MatrixTransform> nodeTransform = new osg::MatrixTransform;
    nodeTransform->setMatrix(osg::Matrix::translate(offsetVec));
    nodeTransform->addChild(geodeArea.get());

    //
    nodeRoot->addChild(geodeArea.get());


    // start viewer
    osgViewer::Viewer viewer;
    viewer.setThreadingModel(osgViewer::ViewerBase::SingleThreaded);
    viewer.setUpViewInWindow(100,100,600,360);
    viewer.setSceneData(nodeRoot.get());
    return viewer.run();
}
