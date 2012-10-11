// STL
#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>

// OGR
#include <ogrsf_frmts.h>

// OpenCTM
#include <openctm/openctm.h>

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

// longitude, latitude, altitude point class
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

struct Vx
{
    double x;
    double y;
    double z;
};

Vx ConvLLAToECEF(const PointLLA &pointLLA)
{
    Vx pointECEF;

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

int main(int argc, const char *argv[])
{
    if(argc != 3)   {
        std::cout << "Usage: #> ./pcl_wkt_to_ctm inputfile.csv outputfile.ctm\n";
        std::cout << " * Expect each line of input to contain a single WKT def\n";
        std::cout << " * The output file is an OpenCTM file containing point data\n";
        std::cout << " * You can create the input file from a shapefile with gdal's ogr tools:\n";
        std::cout << "   ogr2ogr -f CSV file.csv shape.shp -lco -nlt POLYGON -select \"\"\n";
    }

    std::ifstream inputWktFile;
    inputWktFile.open(argv[1]);

    Vx myVx;
    std::vector<Vx> listVx;

    if(inputWktFile.is_open())
    {
        while(!inputWktFile.eof())
        {
            std::string wktLine;
            std::getline(inputWktFile,wktLine);

            // remove any quotes around wktLine
            if(wktLine[0] == '\"' || wktLine[0] == '\'')
            {   wktLine.erase(0,1);   }

            if(wktLine[wktLine.size()-1] == '\"' || wktLine[wktLine.size()-1] == '\'')
            {   wktLine.erase(wktLine.size()-1,1);   }

            // create geometry from wkt
            char *inputWKT = new char[wktLine.size()+1];
            char *inputWKTRef = inputWKT;
            strcpy(inputWKT,wktLine.c_str());

            OGRGeometry * inputGeometry;
            OGRGeometryFactory::createFromWkt(&inputWKT, NULL, &inputGeometry);

            if (inputGeometry == NULL)   {
                std::cout << "Warn: WKT is not valid (ignoring)" << std::endl;
                std::cout << "-> " << wktLine << std::endl;
                delete[] inputWKTRef;
                continue;
            }
            delete[] inputWKTRef;

            if(!(inputGeometry->getGeometryType() == wkbPolygon))   {
                std::cout << "Warn: WKT is not POLYGON (ignoring)" << std::endl;
                std::cout << "-> " << wktLine << std::endl;
                continue;
            }

            // WKT_POLYGON // one outer ring, multiple inner rings

            OGRPolygon *singlePoly = (OGRPolygon*)inputGeometry;

            // outer ring
            OGRLinearRing *outerRing = singlePoly->getExteriorRing();

            // (0,0,0) signals a new ring (inner or outer)
            myVx.x = 0; myVx.y = 0; myVx.z = 0; listVx.push_back(myVx);
            for(size_t i=0; i < outerRing->getNumPoints(); i++)   {
                PointLLA pointLLA(outerRing->getY(i),outerRing->getX(i));
                myVx = ConvLLAToECEF(pointLLA);
                listVx.push_back(myVx);
            }

            // inner rings
            for(size_t n=0; n < singlePoly->getNumInteriorRings(); n++)   {
                OGRLinearRing * innerRing = singlePoly->getInteriorRing(n);

                myVx.x = 0; myVx.y = 0; myVx.z = 0; listVx.push_back(myVx);
                for(size_t i=0; i < innerRing->getNumPoints(); i++)    {
                    PointLLA pointLLA(innerRing->getY(i),innerRing->getX(i));
                    myVx = ConvLLAToECEF(pointLLA); listVx.push_back(myVx);
                    listVx.push_back(myVx);
                }
            }
            delete inputGeometry;
        }
        inputWktFile.close();

        // write OpenCTM file
        CTMcontext  ctmContext;
        CTMuint     ctmVxCount,ctmTriCount;
        CTMuint   * ctmListIx;
        CTMfloat  * ctmListVx;

        ctmContext = ctmNewContext(CTM_EXPORT);
        ctmCompressionMethod(ctmContext,CTM_METHOD_MG1);
        ctmCompressionLevel(ctmContext,5);

        // create vertex list
        ctmVxCount = listVx.size();
        ctmListVx = (CTMfloat*)malloc(3 * sizeof(CTMfloat)*ctmVxCount);

        unsigned int vIdx = 0;
        for(size_t i=0; i < ctmVxCount; i++)   {
            ctmListVx[vIdx] = listVx[i].x; vIdx++;
            ctmListVx[vIdx] = listVx[i].y; vIdx++;
            ctmListVx[vIdx] = listVx[i].z; vIdx++;
        }

        // create null mesh connectivity data as a workaround
        // to get OpenCTM to compress just the point data
        ctmTriCount = 1;
        ctmListIx = (CTMuint*) malloc(3 * sizeof(CTMuint)*ctmTriCount);

        for(int i=0; i < ctmTriCount*3; i++)   {
            ctmListIx[i] = 0;
        }


        ctmDefineMesh(ctmContext,ctmListVx,ctmVxCount,
                      ctmListIx,ctmTriCount,NULL);

        ctmSave(ctmContext,argv[2]);
        free(ctmListIx);
        free(ctmListVx);
    }

    std::cout << "Info: Saved " << listVx.size() << " points\n";

    return 0;
}
