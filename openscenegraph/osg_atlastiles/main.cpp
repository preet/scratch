#include <osgViewer/Viewer>
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>
#include <osg/AutoTransform>
#include <osg/PolygonOffset>
#include <osg/Texture2D>
#include <osg/Program>
#include <osg/Shader>
#include <osgDB/ReadFile>
#include <osg/Uniform>
#include <osg/CullFace>

#include <vector>
#include <fstream>

// geometric defines!
// PI!
#define K_PI 3.141592653589

// epsilon error
#define K_EPS 1E-11
#define K_NEPS -1E-11

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

#include "/home/preet/Dev/projects/osmsrender/osmsrender/util/Vec2.hpp";
#include "/home/preet/Dev/projects/osmsrender/osmsrender/util/Vec3.hpp";

using namespace osmsrender;

struct PointLLA
{
    PointLLA() :
        lon(0),lat(0),alt(0) {}

    PointLLA(double myLat, double myLon) :
        lat(myLat),lon(myLon),alt(0) {}

    PointLLA(double myLat, double myLon, double myAlt) :
        lon(myLon),lat(myLat),alt(myAlt) {}

    double lat;
    double lon;
    double alt;
};

void ConvLLAToECEF(const PointLLA &pointLLA, Vec3 &pointECEF);
Vec3 ConvLLAToECEF(const PointLLA &pointLLA);
void ConvECEFToLLA(const Vec3 &pointECEF, PointLLA &pointLLA);
PointLLA ConvECEFToLLA(const Vec3 &pointECEF);
bool BuildEarthSurfaceGeometry(double minLon, double minLat,
                               double maxLon, double maxLat,
                               size_t lonSegments,
                               size_t latSegments,
                               std::vector<Vec3> &vertexArray,
                               std::vector<Vec2> &texCoords,
                               std::vector<size_t> &triIdx);

std::string readFileAsString(std::string const &fileName)
{
    std::ifstream ifs(fileName.c_str());
    std::string content( (std::istreambuf_iterator<char>(ifs) ),
                         (std::istreambuf_iterator<char>()    ) );
    return content;
}

struct TileTxCoords
{
    Vec2 topLeft;
    Vec2 btmRight;
};

struct TextureAtlas
{
    osg::ref_ptr<osg::Texture2D>    texture;
    osg::ref_ptr<osg::Image>        image;
    std::vector<std::string>        listTileSpaces;
    size_t numTiles;
};

typedef std::vector<TextureAtlas> ListAtlases;
typedef std::vector<TileTxCoords> ListAtlasTileCoords;

typedef std::vector<osg::ref_ptr<osg::Texture2D> >  AtlasListTextures;
typedef std::vector<osg::ref_ptr<osg::Image> >      AtlasListImages;
typedef std::vector<std::vector<std::string> >      AtlasListTileSpaces;

// List of tile coordinates in the atlas
// (arranged in row major order) for quick lookup
typedef std::vector<TileTxCoords>                   AtlasListTileCoords;



// ========================================================================== //

class TileAtlasOSG
{
public:
    TileAtlasOSG(size_t imageSize,size_t tileSize);

    bool AddTile(std::string const &tilePath,
                 osg::StateSet * stateSet,
                 TileTxCoords &tileCoords);

    bool RemTile(std::string const &tilePath,
                 osg::StateSet * stateSet);

private:
    size_t calcNextPowerOfTwo(size_t x) const;

    //
    size_t m_imageSize;
    size_t m_tileSize;
    size_t m_numSpaces;
    size_t m_numBytes;

    osg::Texture2D::FilterMode m_minMode;
    osg::Texture2D::FilterMode m_magMode;
    osg::Texture2D::WrapMode m_wrapMode_s;
    osg::Texture2D::WrapMode m_wrapMode_t;

    ListAtlases             m_listAtlases;
    ListAtlasTileCoords     m_listAtlasTileCoords;
};

int main(int argc, char *argv[])
{
    // shaders
    std::string vShader,fShader;
    osg::ref_ptr<osg::Program> coastShader = new osg::Program;
    coastShader->setName("CoastShader");
    vShader = readFileAsString("shaders/coast_vert.glsl");
    fShader = readFileAsString("shaders/coast_frag.glsl");
    coastShader->addShader(new osg::Shader(osg::Shader::VERTEX,vShader));
    coastShader->addShader(new osg::Shader(osg::Shader::FRAGMENT,fShader));

    // atlas
    TileAtlasOSG * tileAtlas = new TileAtlasOSG(512,256);

    std::string pathTile1 = "/home/preet/Dev/scratch/gis/mapnik_coastlines/sph_tiles/tiles/west/1/0_0.png";
    std::string pathTile2 = "/home/preet/Dev/scratch/gis/mapnik_coastlines/sph_tiles/tiles/west/1/0_1.png";
    std::string pathTile3 = "/home/preet/Dev/scratch/gis/mapnik_coastlines/sph_tiles/tiles/west/1/1_0.png";
    std::string pathTile4 = "/home/preet/Dev/scratch/gis/mapnik_coastlines/sph_tiles/tiles/west/1/1_1.png";

    std::vector<Vec3>   listVx1,listVx2,listVx3,listVx4;
    std::vector<Vec2>   listTx1,listTx2,listTx3,listTx4;
    std::vector<size_t> listIx1,listIx2,listIx3,listIx4;
    BuildEarthSurfaceGeometry(-180,-90,    0,90,    16,16,listVx1,listTx1,listIx1);  // NW
    BuildEarthSurfaceGeometry(-180,-90,    -90,0,   16,16,listVx2,listTx2,listIx2);  // SW
    BuildEarthSurfaceGeometry(-90 ,  0,    0,90,    16,16,listVx3,listTx3,listIx3);  // NE
    BuildEarthSurfaceGeometry(-90 ,  0,    -90,0,   16,16,listVx4,listTx4,listIx4);  // SE

    osg::ref_ptr<osg::Vec3dArray> gmListVx1 = new osg::Vec3dArray;
    osg::ref_ptr<osg::Vec3dArray> gmListVx2 = new osg::Vec3dArray;
    osg::ref_ptr<osg::Vec3dArray> gmListVx3 = new osg::Vec3dArray;
    osg::ref_ptr<osg::Vec3dArray> gmListVx4 = new osg::Vec3dArray;

    osg::ref_ptr<osg::Vec2Array>  gmListTx1 = new osg::Vec2Array;
    osg::ref_ptr<osg::Vec2Array>  gmListTx2 = new osg::Vec2Array;
    osg::ref_ptr<osg::Vec2Array>  gmListTx3 = new osg::Vec2Array;
    osg::ref_ptr<osg::Vec2Array>  gmListTx4 = new osg::Vec2Array;

    for(size_t i=0; i < listVx1.size(); i++)   {
        gmListVx1->push_back(osg::Vec3d(listVx1[i].x,listVx1[i].y,listVx1[i].z));
    }

    for(size_t i=0; i < listVx2.size(); i++)   {
        gmListVx2->push_back(osg::Vec3d(listVx2[i].x,listVx2[i].y,listVx2[i].z));
    }

    for(size_t i=0; i < listVx3.size(); i++)   {
        gmListVx3->push_back(osg::Vec3d(listVx3[i].x,listVx3[i].y,listVx3[i].z));
    }

    for(size_t i=0; i < listVx4.size(); i++)   {
        gmListVx4->push_back(osg::Vec3d(listVx4[i].x,listVx4[i].y,listVx4[i].z));
    }

    osg::ref_ptr<osg::DrawElementsUInt> gmListIx1 =
            new osg::DrawElementsUInt(GL_TRIANGLES);

    osg::ref_ptr<osg::DrawElementsUInt> gmListIx2 =
            new osg::DrawElementsUInt(GL_TRIANGLES);

    osg::ref_ptr<osg::DrawElementsUInt> gmListIx3 =
            new osg::DrawElementsUInt(GL_TRIANGLES);

    osg::ref_ptr<osg::DrawElementsUInt> gmListIx4 =
            new osg::DrawElementsUInt(GL_TRIANGLES);

    for(size_t i=0; i < listIx1.size(); i++)    {
        gmListIx1->push_back(listIx1[i]);
    }

    for(size_t i=0; i < listIx2.size(); i++)    {
        gmListIx2->push_back(listIx2[i]);
    }

    for(size_t i=0; i < listIx3.size(); i++)    {
        gmListIx3->push_back(listIx3[i]);
    }

    for(size_t i=0; i < listIx4.size(); i++)    {
        gmListIx4->push_back(listIx4[i]);
    }

    osg::ref_ptr<osg::Geometry> gmTile1 = new osg::Geometry;
    gmTile1->setVertexArray(gmListVx1);
    gmTile1->setTexCoordArray(0,gmListTx1);
    gmTile1->addPrimitiveSet(gmListIx1);

    osg::ref_ptr<osg::Geometry> gmTile2 = new osg::Geometry;
    gmTile2->setVertexArray(gmListVx2);
    gmTile2->setTexCoordArray(0,gmListTx2);
    gmTile2->addPrimitiveSet(gmListIx2);

    osg::ref_ptr<osg::Geometry> gmTile3 = new osg::Geometry;
    gmTile3->setVertexArray(gmListVx3);
    gmTile3->setTexCoordArray(0,gmListTx3);
    gmTile3->addPrimitiveSet(gmListIx3);

    osg::ref_ptr<osg::Geometry> gmTile4 = new osg::Geometry;
    gmTile4->setVertexArray(gmListVx4);
    gmTile4->setTexCoordArray(0,gmListTx4);
    gmTile4->addPrimitiveSet(gmListIx4);

    osg::ref_ptr<osg::Geode> gdTile1 = new osg::Geode;
    osg::StateSet * ss = gdTile1->getOrCreateStateSet();
    ss->setAttributeAndModes(coastShader,osg::StateAttribute::ON);
    gdTile1->addDrawable(gmTile1);

    TileTxCoords txCoords1;
    tileAtlas->AddTile(pathTile1,ss,txCoords1);
    std::cout << " ## 1: TL: "
              << txCoords1.topLeft.x << ","
              << txCoords1.topLeft.y << " , BR: "
              << txCoords1.btmRight.x << ","
              << txCoords1.btmRight.y << std::endl;

    for(size_t i=0; i < listTx1.size(); i++)   {
        double kTemp = fabs(txCoords1.topLeft.x-txCoords1.btmRight.x);
        Vec2 tileTx = listTx1[i].ScaledBy(kTemp);
        tileTx = tileTx + Vec2(txCoords1.topLeft.x,txCoords1.topLeft.y);
        gmListTx1->push_back(osg::Vec2(tileTx.x,tileTx.y));
    }

    osg::ref_ptr<osg::Geode> gdTile2 = new osg::Geode;
    ss = gdTile2->getOrCreateStateSet();
    ss->setAttributeAndModes(coastShader,osg::StateAttribute::ON);
    gdTile2->addDrawable(gmTile2);

    TileTxCoords txCoords2;
    tileAtlas->AddTile(pathTile2,ss,txCoords2);
    std::cout << " ## 2: TL: "
              << txCoords2.topLeft.x << ","
              << txCoords2.topLeft.y << " , BR: "
              << txCoords2.btmRight.x << ","
              << txCoords2.btmRight.y << std::endl;

    for(size_t i=0; i < listTx2.size(); i++)   {
        double kTemp = fabs(txCoords2.topLeft.x-txCoords2.btmRight.x);
        Vec2 tileTx = listTx2[i].ScaledBy(kTemp);
        tileTx = tileTx + Vec2(txCoords2.topLeft.x,txCoords2.topLeft.y);
        gmListTx2->push_back(osg::Vec2(tileTx.x,tileTx.y));
    }

    osg::ref_ptr<osg::Geode> gdTile3 = new osg::Geode;
    ss = gdTile3->getOrCreateStateSet();
    ss->setAttributeAndModes(coastShader,osg::StateAttribute::ON);
    gdTile3->addDrawable(gmTile3);

    TileTxCoords txCoords3;
    tileAtlas->AddTile(pathTile3,ss,txCoords3);
    std::cout << " ## 3: TL: "
              << txCoords3.topLeft.x << ","
              << txCoords3.topLeft.y << " , BR: "
              << txCoords3.btmRight.x << ","
              << txCoords3.btmRight.y << std::endl;

    for(size_t i=0; i < listTx3.size(); i++)   {
        double kTemp = fabs(txCoords3.topLeft.x-txCoords3.btmRight.x);
        Vec2 tileTx = listTx3[i].ScaledBy(kTemp);
        tileTx = tileTx + Vec2(txCoords3.topLeft.x,txCoords3.topLeft.y);
        gmListTx3->push_back(osg::Vec2(tileTx.x,tileTx.y));
    }

    osg::ref_ptr<osg::Geode> gdTile4 = new osg::Geode;
    ss = gdTile4->getOrCreateStateSet();
    ss->setAttributeAndModes(coastShader,osg::StateAttribute::ON);
    gdTile4->addDrawable(gmTile4);

    TileTxCoords txCoords4;
    tileAtlas->AddTile(pathTile4,ss,txCoords4);
    std::cout << " ## 4: TL: "
              << txCoords4.topLeft.x << ","
              << txCoords4.topLeft.y << " , BR: "
              << txCoords4.btmRight.x << ","
              << txCoords4.btmRight.y << std::endl;

    for(size_t i=0; i < listTx4.size(); i++)   {
        double kTemp = fabs(txCoords4.topLeft.x-txCoords4.btmRight.x);
        Vec2 tileTx = listTx4[i].ScaledBy(kTemp);
        tileTx = tileTx + Vec2(txCoords4.topLeft.x,txCoords4.topLeft.y);
        gmListTx4->push_back(osg::Vec2(tileTx.x,tileTx.y));
    }

    // [root]
    osg::ref_ptr<osg::Group> groupRoot = new osg::Group;
    ss = groupRoot->getOrCreateStateSet();
    ss->setMode(GL_BLEND,osg::StateAttribute::ON);
    groupRoot->addChild(gdTile1);
    groupRoot->addChild(gdTile2);
    groupRoot->addChild(gdTile3);
    groupRoot->addChild(gdTile4);

    // viewer
    osgViewer::Viewer viewer;
    viewer.setThreadingModel(osgViewer::ViewerBase::SingleThreaded);
    viewer.setUpViewInWindow(100,100,800,480);
    viewer.setSceneData(groupRoot.get());

    osgViewer::Viewer::Windows windows;
    viewer.getWindows(windows);
    for(osgViewer::Viewer::Windows::iterator itr = windows.begin();
        itr != windows.end(); ++itr)   {
        (*itr)->getState()->setUseModelViewAndProjectionUniforms(true);
        (*itr)->getState()->setUseVertexAttributeAliasing(true);
    }

    return viewer.run();

}

void ConvLLAToECEF(const PointLLA &pointLLA, Vec3 &pointECEF)
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

Vec3 ConvLLAToECEF(const PointLLA &pointLLA)
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

void ConvECEFToLLA(const Vec3 &pointECEF, PointLLA &pointLLA)
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
    pointLLA.lon = pointLLA.lon * 180.0/K_PI;
    pointLLA.lat = pointLLA.lat * 180.0/K_PI;

    return pointLLA;
}

bool BuildEarthSurfaceGeometry(double minLon, double maxLon,
                               double minLat, double maxLat,
                               size_t lonSegments,
                               size_t latSegments,
                               std::vector<Vec3> &vertexArray,
                               std::vector<Vec2> &texCoords,
                               std::vector<size_t> &triIdx)
{
    if((!(minLon < maxLon)) || (!(minLat < maxLat)))   {
        std::cout << "ERROR: EarthSurfaceGeometry: Invalid bounds\n";
        return false;
    }

    if(latSegments < 4 || lonSegments < 4)   {
        std::cout << "ERROR: EarthSurfaceGeometry: Insufficient segments\n";
        return false;
    }

    double lonStep = (maxLon-minLon)/lonSegments;
    double latStep = (maxLat-minLat)/latSegments;

    vertexArray.clear();
    texCoords.clear();
    triIdx.clear();

    // build vertex attributes
    for(size_t i=0; i <= latSegments; i++)   {
        for(size_t j=0; j <= lonSegments; j++)   {
            // surface vertex
            vertexArray.push_back(ConvLLAToECEF(PointLLA((i*latStep)+minLat,
                                                         (j*lonStep)+minLon)));
            // surface tex coord
            texCoords.push_back(Vec2((j*lonStep)/(maxLon-minLon),
                                     (i*latStep)/(maxLat-minLat)));
        }
    }

    // stitch faces together
    size_t vIdx=0;
    for(size_t i=0; i < latSegments; i++)   {
        for(size_t j=0; j < lonSegments; j++)   {
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

    return true;
}

TileAtlasOSG::TileAtlasOSG(size_t imageSize,
                           size_t tileSize)
{
    // Force tile textures to be power of two
    m_imageSize = calcNextPowerOfTwo(imageSize);
    m_tileSize  = calcNextPowerOfTwo(tileSize);

    // Check if invalid sizes; set to sane defaults
    if(m_tileSize >= m_imageSize)   {
        m_imageSize = 1024;
        m_tileSize  = 256;
    }
    m_numBytes  = imageSize*imageSize*4;
    m_numSpaces = (m_imageSize*m_imageSize)/
                  (m_tileSize*m_tileSize);

    // Calculate tile coordinates
    size_t sideLength = m_imageSize/m_tileSize;
    double tileFrac   = 1.0/sideLength;
    for(size_t i=0; i < sideLength; i++)  // cols
    {
        for(size_t j=0; j < sideLength; j++)  // rows
        {
            TileTxCoords tileTxCoords;
            tileTxCoords.topLeft.x  = tileFrac*j;
            tileTxCoords.topLeft.y  = tileFrac*i;
            tileTxCoords.btmRight.x = tileFrac*(j+1);
            tileTxCoords.btmRight.y = tileFrac*(i+1);

            // save
            m_listAtlasTileCoords.push_back(tileTxCoords);
        }
    }

    //
    m_minMode = osg::Texture2D::NEAREST;
    m_magMode = osg::Texture2D::NEAREST;
    m_wrapMode_s = osg::Texture2D::CLAMP_TO_EDGE;
    m_wrapMode_t = osg::Texture2D::CLAMP_TO_EDGE;

    std::cout << "TileAtlasOSG: " << m_imageSize << " x " << m_imageSize << std::endl;

    // NOTE
    // GL_UNSIGNED_BYTE [4 bytes per pixel]
    // GL_UNSIGNED_SHORT_4_4_4_4 [2 bytes per pixel]

    // specifying osg image pixels manually:
    // http://www.jotschi.de/Technik/2009/10/18/
    // openscenegraph-osgimage-getting-a-subimage-from-an-image.html
}

bool TileAtlasOSG::AddTile(std::string const &tilePath,
                           osg::StateSet * stateSet,
                           TileTxCoords &tileCoords)
{
    osg::ref_ptr<osg::Image> tileImg =
            osgDB::readImageFile(tilePath);

    bool opOk = tileImg.valid() &&      // check if readImage was successful
                tileImg->valid();       // check if the image is valid

    if(!opOk)   {
        std::cout << "TileAtlas: AddTile: path/image invalid!" << std::endl;
        return false;
    }

    // scale image to the correct size
    tileImg->scaleImage(m_tileSize,m_tileSize,1);

    // Check each atlas to see if there is any
    // empty space available
    for(size_t i=0; i < m_listAtlases.size(); i++)
    {
        std::vector<std::string> &listTileSpaces =
                m_listAtlases[i].listTileSpaces;

        for(size_t j=0; j < listTileSpaces.size(); j++)
        {
            if(listTileSpaces[j].empty())
            {
                // copy the tile image into the atlas image
                tileCoords = m_listAtlasTileCoords[j];
                size_t xBeg = tileCoords.topLeft.x * m_imageSize;
                size_t yBeg = tileCoords.topLeft.y * m_imageSize;
                std::cout << "xBeg is " << xBeg << std::endl;
                std::cout << "yBeg is " << yBeg << std::endl;

                // save
                m_listAtlases[i].listTileSpaces[j] = tilePath;
                m_listAtlases[i].image->copySubImage(xBeg,yBeg,0,tileImg);
                m_listAtlases[i].numTiles++;

                std::cout << "!! : " << m_listAtlases[i].numTiles << std::endl;

                // update state set
                stateSet->setTextureAttributeAndModes(0,m_listAtlases[i].texture);

                return true;
            }
        }
    }

    // If there is no space available create
    // a new texture atlas
    TextureAtlas texAtlas;
    texAtlas.numTiles = 0;
    texAtlas.listTileSpaces.resize(m_numSpaces);
    texAtlas.image = new osg::Image;
    texAtlas.image->allocateImage(m_imageSize,m_imageSize,1,
                                  GL_RGBA,GL_UNSIGNED_BYTE);
    // (temp pixel color data)
    unsigned char * initData = texAtlas.image->data();
    for(size_t i=0; i < m_numBytes; i+=4)   {
        initData[i  ] = 155;
        initData[i+1] = 155;
        initData[i+2] = 155;
        initData[i+3] = 0;
    }
    // copy the tile image into the first
    // atlas tile space
    tileCoords = m_listAtlasTileCoords[0];
    size_t xBeg = tileCoords.topLeft.x * m_imageSize;
    size_t yBeg = tileCoords.topLeft.y * m_imageSize;
    std::cout << "xBeg is " << xBeg << std::endl;
    std::cout << "yBeg is " << yBeg << std::endl;
    texAtlas.image->copySubImage(0,0,0,tileImg);
    texAtlas.listTileSpaces[0] = tilePath;
    texAtlas.numTiles++;

    // setup texture
    texAtlas.texture = new osg::Texture2D;
    texAtlas.texture->setFilter(osg::Texture2D::MIN_FILTER,m_minMode);
    texAtlas.texture->setFilter(osg::Texture2D::MAG_FILTER,m_magMode);
    texAtlas.texture->setWrap(osg::Texture2D::WRAP_S,m_wrapMode_s);
    texAtlas.texture->setWrap(osg::Texture2D::WRAP_T,m_wrapMode_t);
    texAtlas.texture->setImage(texAtlas.image);

    // save atlas
    m_listAtlases.push_back(texAtlas);

    // update state set
    osg::ref_ptr<osg::Uniform> uSampler = new osg::Uniform("Texture",0);
    stateSet->addUniform(uSampler);
    stateSet->setTextureAttributeAndModes(0,texAtlas.texture);

    return true;
}

//bool TileAtlasOSG::AddTile(std::string const &tilePath,
//                           osg::StateSet * stateSet,
//                           TileTxCoords &tileCoords)
//{
//    osg::ref_ptr<osg::Image> tileImg =
//            osgDB::readImageFile(tilePath);

//    // WARN
//    // tileImg will be null if readImageFile failed, so the
//    // following check is useless

//    if(!(tileImg->valid()))   {
//        std::cout << "TileAtlas: AddTile: path/image invalid!" << std::endl;
//        return false;
//    }

//    // Check for empty spaces
////    for(size_t i=0; i < m_listAtlases.size(); i++)
////    {
////        std::vector<std::string> &listTileSpaces =
////                m_listAtlases[i].listTileSpaces;

////        for(size_t j=0; j < listTileSpaces.size(); j++)
////        {
////            if(listTileSpaces[j].empty())
////            {   // save path
////                listTileSpaces[j] = tilePath;

////                // copy the tile image into the atlas image
////                tileCoords = m_listAtlasTileCoords[j];
////                size_t xBeg = tileCoords.topLeft.x * m_tileSize;
////                size_t yBeg = tileCoords.topLeft.y * m_tileSize;
////                m_listAtlases[i].image->copySubImage(xBeg,yBeg,0,tileImg);
////                m_listAtlases[i].numTiles++;

////                OSRDEBUG << "!! : " << m_listAtlases[i].numTiles;

////                return true;
////            }
////        }
////    }

//    if(m_listAtlases.empty())
//    {
//        TextureAtlas texAtlas;
//        texAtlas.numTiles = 0;
//        texAtlas.listTileSpaces.resize(m_numSpaces);
//        texAtlas.image = new osg::Image;
//        texAtlas.image->allocateImage(m_imageSize,m_imageSize,1,
//                                      GL_RGBA,GL_UNSIGNED_BYTE);
//        // (temp pixel color data)
//        unsigned char * initData = texAtlas.image->data();
//        for(size_t i=0; i < m_numBytes; i+=4)   {
//            initData[i  ] = 155;
//            initData[i+1] = 155;
//            initData[i+2] = 155;
//            initData[i+3] = 0;
//        }

//        // copy the tile image into the atlas image
//        tileCoords = m_listAtlasTileCoords[0];
//        size_t xBeg = tileCoords.topLeft.x * m_tileSize;
//        size_t yBeg = tileCoords.topLeft.y * m_tileSize;
//        texAtlas.image->copySubImage(0,0,0,tileImg);
//        texAtlas.listTileSpaces[0] = tilePath;
//        texAtlas.numTiles++;

//        // setup texture
//        texAtlas.texture = new osg::Texture2D;
//        texAtlas.texture->setFilter(osg::Texture2D::MIN_FILTER,m_minMode);
//        texAtlas.texture->setFilter(osg::Texture2D::MAG_FILTER,m_magMode);
//        texAtlas.texture->setWrap(osg::Texture2D::WRAP_S,m_wrapMode_s);
//        texAtlas.texture->setWrap(osg::Texture2D::WRAP_T,m_wrapMode_t);
//        texAtlas.texture->setImage(texAtlas.image);

//        // save atlas
//        m_listAtlases.push_back(texAtlas);
//    }

//    // If there are no empty spaces available in the
//    // current list of atlas images, create a new atlas



//    // add to state set
////    if(stateSet->getUniform("Texture") == NULL)   {
////        osg::ref_ptr<osg::Uniform> uSampler = new osg::Uniform("Texture",0);
////        stateSet->addUniform(uSampler);
////    }
//    stateSet->setTextureAttributeAndModes(0,m_listAtlases[0].texture);

//    return true;
//}

bool TileAtlasOSG::RemTile(std::string const &tilePath,
                           osg::StateSet * stateSet)
{
    size_t i = 0;
    bool foundPath = false;

    // Find the tile to remove
    for(i=0; i < m_listAtlases.size(); i++)
    {
        std::vector<std::string> &listTileSpaces =
                m_listAtlases[i].listTileSpaces;

        for(size_t j=0; j < listTileSpaces.size(); j++)
        {
            if(listTileSpaces[j].compare(tilePath) == 0)
            {   // just clear the tile path string -- we don't
                // explicitly overwrite the pixel data here
                listTileSpaces[j].clear();
                foundPath = true;
                break;
            }
        }
    }

    if(!foundPath)   {
        std::cout << "TileAtlas: RemTile: path not found!" << std::endl;
        return false;
    }

    // check if the atlas is empty
    std::vector<std::string> &listTileSpaces =
            m_listAtlases[i].listTileSpaces;

    for(size_t j=0; j < listTileSpaces.size(); j++)   {
        if(!listTileSpaces[j].empty())
        {   // the atlas isn't empty, just return
            return true;
        }
    }

    // remove the empty atlas
//    stateSet->removeTextureAttribute(0,m_listAtlases[i].texture);
//    if(stateSet->getUniform("Texture") != NULL)   {
//        stateSet->removeUniform("Texture");
//    }

    m_listAtlases.erase(m_listAtlases.begin()+i);
    return true;
}

size_t TileAtlasOSG::calcNextPowerOfTwo(size_t x) const
{
    size_t val = 1;

    while(val < x)
    {   val *= 2;   }

    return val;
}
