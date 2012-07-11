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
            if(vIdx != heightSegments)
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

    // generate axes
//    osg::ref_ptr<osg::Vec3Array> axesVertices = new osg::Vec3Array;
//    osg::ref_ptr<osg::Vec4dArray> axesColors = new osg::Vec4dArray;
//    osg::ref_ptr<osg::DrawElementsUInt> axesIndices = new osg::DrawElementsUInt(GL_LINES);
//    axesVertices->push_back(osg::Vec3(-2,0,0));
//    axesVertices->push_back(osg::Vec3(2,0,0));
//    axesColors->push_back(osg::Vec4(1,0,0,1));
//    axesColors->push_back(osg::Vec4(1,0,0,1));
//    axesVertices->push_back(osg::Vec3(0,-2,0));
//    axesVertices->push_back(osg::Vec3(0,2,0));
//    axesColors->push_back(osg::Vec4(0,1,0,1));
//    axesColors->push_back(osg::Vec4(0,1,0,1));
//    axesVertices->push_back(osg::Vec3(0,0,-2));
//    axesVertices->push_back(osg::Vec3(0,0,2));
//    axesColors->push_back(osg::Vec4(0,0,1,1));
//    axesColors->push_back(osg::Vec4(0,0,1,1));
//    axesIndices->push_back(0); axesIndices->push_back(1);
//    axesIndices->push_back(2); axesIndices->push_back(3);
//    axesIndices->push_back(4); axesIndices->push_back(5);

//    osg::ref_ptr<osg::Geometry> myAxes = new osg::Geometry;
//    myAxes->setVertexArray(axesVertices);
//    myAxes->addPrimitiveSet(axesIndices);
//    myAxes->setColorArray(axesColors);
//    myAxes->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

//    osg::ref_ptr<osg::Geode> geodeAxes = new osg::Geode;
//    geodeAxes->addDrawable(myAxes.get());
//    nodeRoot->addChild(geodeAxes.get());

    double radius = ELL_SEMI_MAJOR;

    // front face
    osg::ref_ptr<osg::Geometry> geomPosX = new osg::Geometry;
    buildPlaneGeometry(geomPosX,
                       osg::Vec3(radius,0,0),
                       osg::Vec3(1,0,0),
                       osg::Vec3(0,0,1),
                       radius*2,radius*2,4,4);

    osg::ref_ptr<osg::Image> imgPosX = osgDB::readImageFile("textures/earth_front.jpg");
    osg::ref_ptr<osg::Texture2D> texPosX = new osg::Texture2D;
    texPosX->setImage(imgPosX.get());
    texPosX->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
    texPosX->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
    geomPosX->getOrCreateStateSet()->setTextureAttributeAndModes(0,texPosX.get());

    // back face
    osg::ref_ptr<osg::Geometry> geomNegX = new osg::Geometry;
    buildPlaneGeometry(geomNegX,
                       osg::Vec3(-1*radius,0,0),
                       osg::Vec3(-1,0,0),
                       osg::Vec3(0,0,1),
                       radius*2,radius*2,4,4);

    osg::ref_ptr<osg::Image> imgNegX = osgDB::readImageFile("textures/earth_back.jpg");
    osg::ref_ptr<osg::Texture2D> texNegX = new osg::Texture2D;
    texNegX->setImage(imgNegX.get());
    texNegX->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
    texNegX->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
    geomNegX->getOrCreateStateSet()->setTextureAttributeAndModes(0,texNegX.get());

    // right face
    osg::ref_ptr<osg::Geometry> geomPosY = new osg::Geometry;
    buildPlaneGeometry(geomPosY,
                       osg::Vec3(0,radius,0),
                       osg::Vec3(0,1,0),
                       osg::Vec3(0,0,1),
                       radius*2,radius*2,4,4);

    osg::ref_ptr<osg::Image> imgPosY = osgDB::readImageFile("textures/earth_right.jpg");
    osg::ref_ptr<osg::Texture2D> texPosY = new osg::Texture2D;
    texPosY->setImage(imgPosY.get());
    texPosY->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
    texPosY->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
    geomPosY->getOrCreateStateSet()->setTextureAttributeAndModes(0,texPosY.get());

    // left face
    osg::ref_ptr<osg::Geometry> geomNegY = new osg::Geometry;
    buildPlaneGeometry(geomNegY,
                       osg::Vec3(0,-1*radius,0),
                       osg::Vec3(0,-1,0),
                       osg::Vec3(0,0,1),
                       radius*2,radius*2,4,4);

    osg::ref_ptr<osg::Image> imgNegY = osgDB::readImageFile("textures/earth_left.jpg");
    osg::ref_ptr<osg::Texture2D> texNegY = new osg::Texture2D;
    texNegY->setImage(imgNegY.get());
    texNegY->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
    texNegY->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
    geomNegY->getOrCreateStateSet()->setTextureAttributeAndModes(0,texNegY.get());

    // top face
    osg::ref_ptr<osg::Geometry> geomPosZ = new osg::Geometry;
    buildPlaneGeometry(geomPosZ,
                       osg::Vec3(0,0,radius),
                       osg::Vec3(0,0,1),
                       osg::Vec3(-1,0,0),
                       radius*2,radius*2,4,4);

    osg::ref_ptr<osg::Image> imgPosZ = osgDB::readImageFile("textures/earth_top.jpg");
    osg::ref_ptr<osg::Texture2D> texPosZ = new osg::Texture2D;
    texPosZ->setImage(imgPosZ.get());
    texPosZ->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
    texPosZ->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
    geomPosZ->getOrCreateStateSet()->setTextureAttributeAndModes(0,texPosZ.get());

    // bottom face
    osg::ref_ptr<osg::Geometry> geomNegZ = new osg::Geometry;
    buildPlaneGeometry(geomNegZ,
                       osg::Vec3(0,0,-1*radius),
                       osg::Vec3(0,0,-1),
                       osg::Vec3(1,0,0),
                       radius*2,radius*2,4,4);

    osg::ref_ptr<osg::Image> imgNegZ = osgDB::readImageFile("textures/earth_btm.jpg");
    osg::ref_ptr<osg::Texture2D> texNegZ = new osg::Texture2D;
    texNegZ->setImage(imgNegZ.get());
    texNegZ->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
    texNegZ->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
    geomNegZ->getOrCreateStateSet()->setTextureAttributeAndModes(0,texNegZ.get());


    //
    osg::ref_ptr<osg::Geode> geodeCube = new osg::Geode;
    geodeCube->addDrawable(geomPosX.get());
    geodeCube->addDrawable(geomNegX.get());
    geodeCube->addDrawable(geomPosY.get());
    geodeCube->addDrawable(geomNegY.get());
    geodeCube->addDrawable(geomPosZ.get());
    geodeCube->addDrawable(geomNegZ.get());

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
    nodeXform->addChild(geodeOverlay.get());

    osg::ref_ptr<osg::PolygonMode> polygonMode = new osg::PolygonMode;
    polygonMode->setMode(osg::PolygonMode::FRONT_AND_BACK,
                         osg::PolygonMode::LINE);

    osg::ref_ptr<osg::Material> meshMaterial = new osg::Material;
    meshMaterial->setColorMode(osg::Material::OFF);
    meshMaterial->setDiffuse(osg::Material::FRONT_AND_BACK,
                             osg::Vec4(1,1,1,0.5));

    geodeOverlay->getOrCreateStateSet()->setAttribute(polygonMode.get());
    geodeOverlay->getOrCreateStateSet()->setAttribute(meshMaterial.get());

//    nodeRoot->getOrCreateStateSet()->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
    nodeRoot->getOrCreateStateSet()->setMode(GL_BLEND,osg::StateAttribute::ON);
    nodeRoot->addChild(geodeCube.get());
    nodeRoot->addChild(nodeXform.get());

    // start viewer
    osgViewer::Viewer viewer;
    viewer.setThreadingModel(osgViewer::ViewerBase::SingleThreaded);
    viewer.setUpViewInWindow(100,100,600,360);
    viewer.setSceneData(nodeRoot.get());
    return viewer.run();
}
