// sys includes
#include <math.h>
#include <vector>
#include <iostream>
#include <sys/time.h>

// osg includes
#include <osg/Vec3>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/PrimitiveSet>
#include <osg/Texture2D>
#include <osgDB/ReadFile>
#include <osg/PolygonMode>
#include <osgViewer/Viewer>
#include <osg/Material>
#include <osg/MatrixTransform>

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

void calcQuadraticEquationReal(double a, double b, double c,
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

bool calcRayEarthIntersection(const Vec3 &linePoint,
                              const Vec3 &lineDirn,
                              Vec3 &nearXsecPoint)
{
    // the solution for intersection points between a ray
    // and the Earth's surface is a quadratic equation

    // first calculate the quadratic equation params:
    // a(x^2) + b(x) + c

    // a, b and c are found by substituting the parametric
    // equation of a line into the equation for a ellipsoid
    // and solving in terms of the line's parameter

    // * http://en.wikipedia.org/wiki/Ellipsoid
    // * http://gis.stackexchange.com/questions/20780/...
    //   ...point-of-intersection-for-a-ray-and-earths-surface

    std::vector<double> listRoots;

    double a = (pow(lineDirn.x,2) / ELL_SEMI_MAJOR_EXP2) +
            (pow(lineDirn.y,2) / ELL_SEMI_MAJOR_EXP2) +
            (pow(lineDirn.z,2) / ELL_SEMI_MINOR_EXP2);

    double b = (2*linePoint.x*lineDirn.x/ELL_SEMI_MAJOR_EXP2) +
            (2*linePoint.y*lineDirn.y/ELL_SEMI_MAJOR_EXP2) +
            (2*linePoint.z*lineDirn.z/ELL_SEMI_MINOR_EXP2);

    double c = (pow(linePoint.x,2) / ELL_SEMI_MAJOR_EXP2) +
            (pow(linePoint.y,2) / ELL_SEMI_MAJOR_EXP2) +
            (pow(linePoint.z,2) / ELL_SEMI_MINOR_EXP2) - 1;

    calcQuadraticEquationReal(a,b,c,listRoots);
    if(!listRoots.empty())
    {   // ensure poi lies along ray dirn
        if((listRoots[0] > 0) && (listRoots[1] > 0))
        {
            Vec3 point1;
            point1.x = linePoint.x + listRoots.at(0)*lineDirn.x;
            point1.y = linePoint.y + listRoots.at(0)*lineDirn.y;
            point1.z = linePoint.z + listRoots.at(0)*lineDirn.z;

            Vec3 point2;
            point2.x = linePoint.x + listRoots.at(1)*lineDirn.x;
            point2.y = linePoint.y + listRoots.at(1)*lineDirn.y;
            point2.z = linePoint.z + listRoots.at(1)*lineDirn.z;

            // save the point nearest to the ray's origin
            if(linePoint.DistanceTo(point1) > linePoint.DistanceTo(point2))
            {   nearXsecPoint = point2;   }
            else
            {   nearXsecPoint = point1;   }

            return true;
        }
    }
    return false;
}

void buildEarthSurfGeom(double minLon,double minLat,
                        double maxLon,double maxLat,
                        unsigned int lonSegments,
                        unsigned int latSegments,
                        std::vector<Vec3> &vertexArray,
                        std::vector<Vec2> &texCoords,
                        std::vector<unsigned int> &triIdx)
{
    double lonStep = (maxLon-minLon)/lonSegments;
    double latStep = (maxLat-minLat)/latSegments;

    vertexArray.clear();
    texCoords.clear();
    triIdx.clear();

    // build vertex attributes
    Vec3 sVert;
    for(int i=0; i <= latSegments; i++)   {
        for(int j=0; j <= lonSegments; j++)   {
            // surface vertex
            vertexArray.push_back(convLLAToECEF(PointLLA((i*latStep)+minLat,
                                                         (j*lonStep)+minLon)));
            // surface tex coord
            texCoords.push_back(Vec2((j*lonStep)/(maxLon-minLon),
                                     (i*latStep)/(maxLat-minLat)));
        }
    }

    // stitch faces together
    unsigned int vIdx=0;
    for(int i=0; i < latSegments; i++)   {
        for(int j=0; j < lonSegments; j++)   {
            triIdx.push_back(vIdx);
            triIdx.push_back(vIdx+lonSegments+1);
            triIdx.push_back(vIdx+lonSegments+2);

            triIdx.push_back(vIdx);
            triIdx.push_back(vIdx+lonSegments+2);
            triIdx.push_back(vIdx+1);

            vIdx++;
        }
        vIdx++;
    }
}

void buildPlaneGeometry(Vec3 const &vecCenter,
                        Vec3 const &vecNormal,
                        Vec3 const &vecUp,
                        double width, double height,
                        unsigned int widthSegments,
                        unsigned int heightSegments,
                        std::vector<Vec3> &vertexArray,
                        std::vector<Vec2> &texCoords,
                        std::vector<unsigned int> &triIdx)
{
    double segmentWidth = width/widthSegments;
    double segmentHeight = height/heightSegments;

    // build vertex attributes
    for(int i=0; i <= widthSegments; i++)   {   // rows
        for(int j=0; j <= heightSegments; j++)   {  // cols
            vertexArray.push_back(Vec3(i*segmentWidth,j*segmentHeight,0.0));
            texCoords.push_back(Vec2(i*segmentWidth/width,j*segmentHeight/height));
        }
    }

    // stitch faces together
    unsigned int vIdx = 0;
    for(int i=0; i < widthSegments; i++)   {    // rows
        for(int j=0; j < heightSegments; j++)   {   // cols
            triIdx.push_back(vIdx);
            triIdx.push_back(vIdx+heightSegments+1);
            triIdx.push_back(vIdx+heightSegments+2);

            triIdx.push_back(vIdx);
            triIdx.push_back(vIdx+heightSegments+2);
            triIdx.push_back(vIdx+1);

            vIdx++;
        }
        vIdx++;
    }

    // translate to origin, then rotate, then translate to center
    Vec3 vecTranslate(-1*width/2,-1*height/2,0);

    // transformed x,y,z
    Vec3 xformedx = vecUp.Cross(vecNormal).Normalized();
    Vec3 xformedy = vecUp.Normalized();
    Vec3 xformedz = vecNormal.Normalized();

    for(int i=0; i < vertexArray.size(); i++)   {
        // translate relative to (0,0,0)
        vertexArray[i] = vertexArray[i] + vecTranslate;

        // xform matrix multiplication (col. major mult.)
        double vx = vertexArray[i].x;
        double vy = vertexArray[i].y;
        double vz = vertexArray[i].z;

        vx = xformedx.x*vertexArray[i].x +
             xformedy.x*vertexArray[i].y +
             xformedz.x*vertexArray[i].z;

        vy = xformedx.y*vertexArray[i].x +
             xformedy.y*vertexArray[i].y +
             xformedz.y*vertexArray[i].z;

        vz = xformedx.z*vertexArray[i].x +
             xformedy.z*vertexArray[i].y +
             xformedz.z*vertexArray[i].z;

        vertexArray[i].x = vx + vecCenter.x;
        vertexArray[i].y = vy + vecCenter.y;
        vertexArray[i].z = vz + vecCenter.z;

        //
        Vec3 earthSurfPt;
        Vec3 rayOrigin = vertexArray[i].Normalized().ScaledBy(ELL_SEMI_MAJOR*1.1);  // so its closer to the side we want
        Vec3 rayDirn = vertexArray[i].Normalized().ScaledBy(-1); // invert direction
        calcRayEarthIntersection(rayOrigin,rayDirn,earthSurfPt);
        vertexArray[i] = earthSurfPt;
    }
}

void buildPlaneGeometry(osg::Geometry *geomPlane,
                        osg::Vec3 const &vecCenter,
                        osg::Vec3 const &vecNormal,
                        osg::Vec3 const &vecUp,
                        double width, double height,
                        unsigned int widthSegments,
                        unsigned int heightSegments)
{
    double segmentWidth = width/widthSegments;
    double segmentHeight = height/heightSegments;

    osg::ref_ptr<osg::Vec3Array> vertexArray = new osg::Vec3Array;
    osg::ref_ptr<osg::Vec3Array> normalArray = new osg::Vec3Array;
    osg::ref_ptr<osg::Vec2Array> texCoords = new osg::Vec2Array;

    // build vertex attributes
    for(int i=0; i <= widthSegments; i++)   {   // rows
        for(int j=0; j <= heightSegments; j++)   {  // cols
            vertexArray->push_back(osg::Vec3(i*segmentWidth,j*segmentHeight,0.0));
            texCoords->push_back(osg::Vec2(i*segmentWidth/width,j*segmentHeight/height));
        }
    }
    normalArray->push_back(vecNormal);

    // stitch faces together
    osg::ref_ptr<osg::DrawElementsUInt> triIdx =
            new osg::DrawElementsUInt(GL_TRIANGLES);

    unsigned int vIdx = 0;
    for(int i=0; i < widthSegments; i++)   {    // rows
        for(int j=0; j < heightSegments; j++)   {   // cols
//            if(vIdx != heightSegments)
            triIdx->push_back(vIdx);
            triIdx->push_back(vIdx+heightSegments+1);
            triIdx->push_back(vIdx+heightSegments+2);

            triIdx->push_back(vIdx);
            triIdx->push_back(vIdx+heightSegments+2);
            triIdx->push_back(vIdx+1);

            vIdx++;
        }
        vIdx++;
    }

    // translate to origin, then rotate, then translate to center
    osg::Vec3 vecTranslate(-1*width/2,-1*height/2,0);
    osg::Vec3 vecNorm = vecNormal; vecNorm.normalize();
    osg::Matrix xformMatrix;

    xformMatrix(1,0) = vecUp.x();
    xformMatrix(1,1) = vecUp.y();
    xformMatrix(1,2) = vecUp.z();
    xformMatrix(1,3) = 0;

    xformMatrix(2,0) = vecNorm.x();
    xformMatrix(2,1) = vecNorm.y();
    xformMatrix(2,2) = vecNorm.z();
    xformMatrix(2,3) = 0;

    xformMatrix(0,0) = (vecUp^vecNorm).x();
    xformMatrix(0,1) = (vecUp^vecNorm).y();
    xformMatrix(0,2) = (vecUp^vecNorm).z();
    xformMatrix(0,3) = 0;

    xformMatrix(3,0) = vecCenter.x();
    xformMatrix(3,1) = vecCenter.y();
    xformMatrix(3,2) = vecCenter.z();
    xformMatrix(3,3) = 1;

    for(int i=0; i < vertexArray->size(); i++)   {
        vertexArray->at(i) = vertexArray->at(i)+vecTranslate;
        vertexArray->at(i) = xformMatrix.preMult(vertexArray->at(i));
    }

    // project vertices against sphere

    // save geometry
    geomPlane->setVertexArray(vertexArray.get());
    geomPlane->setNormalArray(normalArray.get());
    geomPlane->setNormalBinding(osg::Geometry::BIND_OVERALL);
    geomPlane->setTexCoordArray(0,texCoords.get());
    geomPlane->addPrimitiveSet(triIdx.get());
}

int main()
{
    // root node
    osg::ref_ptr<osg::Group> nodeRoot = new osg::Group;

    double axisLength = 1.25*ELL_SEMI_MAJOR;

    // generate axes
    osg::ref_ptr<osg::Vec3Array> axesVertices = new osg::Vec3Array;
    osg::ref_ptr<osg::Vec4dArray> axesColors = new osg::Vec4dArray;
    osg::ref_ptr<osg::DrawElementsUInt> axesIndices = new osg::DrawElementsUInt(GL_LINES);
    axesVertices->push_back(osg::Vec3(-1*axisLength,0,0));
    axesVertices->push_back(osg::Vec3(axisLength,0,0));
    axesColors->push_back(osg::Vec4(1,0,0,1));
    axesColors->push_back(osg::Vec4(1,0,0,1));
    axesVertices->push_back(osg::Vec3(0,-1*axisLength,0));
    axesVertices->push_back(osg::Vec3(0,axisLength,0));
    axesColors->push_back(osg::Vec4(0,1,0,1));
    axesColors->push_back(osg::Vec4(0,1,0,1));
    axesVertices->push_back(osg::Vec3(0,0,-1*axisLength));
    axesVertices->push_back(osg::Vec3(0,0,axisLength));
    axesColors->push_back(osg::Vec4(0,0,1,1));
    axesColors->push_back(osg::Vec4(0,0,1,1));
    axesIndices->push_back(0); axesIndices->push_back(1);
    axesIndices->push_back(2); axesIndices->push_back(3);
    axesIndices->push_back(4); axesIndices->push_back(5);

    osg::ref_ptr<osg::Geometry> myAxes = new osg::Geometry;
    myAxes->setVertexArray(axesVertices);
    myAxes->addPrimitiveSet(axesIndices);
    myAxes->setColorArray(axesColors);
    myAxes->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

    osg::ref_ptr<osg::Geode> geodeAxes = new osg::Geode;
    geodeAxes->addDrawable(myAxes.get());
    nodeRoot->addChild(geodeAxes.get());

    std::vector<Vec3> surfVertexArray;
    std::vector<Vec2> surfTexCoords;
    std::vector<unsigned int> surfTriIdxs;

    // front face

    buildPlaneGeometry(Vec3(50,0,0),
                       Vec3(1,0,0),
                       Vec3(0,0,1),
                       100,
                       100,
                       8,8,
                       surfVertexArray,
                       surfTexCoords,
                       surfTriIdxs);

//    buildEarthSurfGeom(-90,-45,0,45,8,8,
//                       surfVertexArray,
//                       surfTexCoords,
//                       surfTriIdxs);

//    buildEarthSurfGeom(-180,45,180,80,8,8,
//                       surfVertexArray,
//                       surfTexCoords,
//                       surfTriIdxs);

    osg::ref_ptr<osg::Vec3Array> geomPxVertexArray =
            new osg::Vec3Array(surfVertexArray.size());
    for(int i=0; i < geomPxVertexArray->size(); i++)   {
        geomPxVertexArray->at(i) = osg::Vec3(surfVertexArray[i].x,
                                             surfVertexArray[i].y,
                                             surfVertexArray[i].z);
    }

    osg::ref_ptr<osg::DrawElementsUInt> geomPxTriIdx =
            new osg::DrawElementsUInt(GL_TRIANGLES,surfTriIdxs.size());
    for(int i=0; i < geomPxTriIdx->size(); i++)   {
        geomPxTriIdx->at(i) = surfTriIdxs[i];
    }

    osg::ref_ptr<osg::Vec2Array> geomPxTexCoords =
            new osg::Vec2Array(surfTexCoords.size());
    for(int i=0; i < geomPxTexCoords->size(); i++)   {
        geomPxTexCoords->at(i) = osg::Vec2(surfTexCoords[i].x,
                                           surfTexCoords[i].y);
    }

    osg::ref_ptr<osg::Vec4Array> geomPxColors =
            new osg::Vec4Array;
    geomPxColors->push_back(osg::Vec4(0,0,0,0));

    osg::ref_ptr<osg::Geometry> geomPx = new osg::Geometry;
    geomPx->setVertexArray(geomPxVertexArray.get());
    geomPx->setNormalArray(geomPxVertexArray.get());
    geomPx->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);
    geomPx->setTexCoordArray(0,geomPxTexCoords.get());
//    geomPx->setColorArray(geomPxColors.get());
//    geomPx->setColorBinding(osg::Geometry::BIND_OVERALL);
    geomPx->addPrimitiveSet(geomPxTriIdx.get());

    osg::ref_ptr<osg::Image> imgPx = osgDB::readImageFile("textures/p_x.jpg");
    osg::ref_ptr<osg::Texture2D> texPx = new osg::Texture2D;
    texPx->setImage(imgPx.get());
//    texPx->setFilter(osg::Texture::MAG_FILTER,osg::Texture::NEAREST);
    texPx->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
    texPx->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
    geomPx->getOrCreateStateSet()->setTextureAttributeAndModes(0,texPx.get());

    //sdf
    osg::ref_ptr<osg::Geode> geodeCube = new osg::Geode;
    geodeCube->addDrawable(geomPx.get());

    osg::ref_ptr<osg::Geode> geodeOverlay =
            dynamic_cast<osg::Geode*>(geodeCube->clone(osg::CopyOp::DEEP_COPY_ALL));
    for(int i=0; i < geodeOverlay->getDrawableList().size(); i++)
    {
        osg::Geometry * geom = geodeOverlay->getDrawableList().at(i)->asGeometry();
        geom->getOrCreateStateSet()->getTextureAttributeList().clear();
        geom->getOrCreateStateSet()->getTextureModeList().clear();
    }

    osg::ref_ptr<osg::MatrixTransform> nodeXform = new osg::MatrixTransform;
    nodeXform->setMatrix(osg::Matrix::scale(1.01,1.01,1.01));
//    nodeXform->addChild(geodeOverlay.get());

    osg::ref_ptr<osg::PolygonMode> polygonMode = new osg::PolygonMode;
    polygonMode->setMode(osg::PolygonMode::FRONT_AND_BACK,
                         osg::PolygonMode::LINE);

    osg::ref_ptr<osg::Material> meshMaterial = new osg::Material;
    meshMaterial->setColorMode(osg::Material::OFF);
    meshMaterial->setDiffuse(osg::Material::FRONT_AND_BACK,
                             osg::Vec4(1,1,1,0.25));

    geodeOverlay->getOrCreateStateSet()->setAttribute(polygonMode.get());
    geodeOverlay->getOrCreateStateSet()->setAttribute(meshMaterial.get());

    nodeRoot->getOrCreateStateSet()->setMode(GL_BLEND,osg::StateAttribute::ON);
    nodeRoot->addChild(geodeCube.get());
    nodeRoot->addChild(nodeXform.get());

    // start viewer
    osgViewer::Viewer viewer;
    viewer.setThreadingModel(osgViewer::ViewerBase::SingleThreaded);
    viewer.setUpViewInWindow(100,100,800,480);
    viewer.setSceneData(nodeRoot.get());
    return viewer.run();
}
