// STL
#include <iostream>
#include <iomanip>
#include <cstring>
#include <sstream>
#include <fstream>
#include <stack>
#include <set>
#include <sys/time.h>

// OGR
#include <ogrsf_frmts.h>
#include <ogr_spatialref.h>

// timing vars
timeval t1,t2;
std::string timingDesc;

void StartTiming(std::string const &desc)
{
    timingDesc = desc;
    gettimeofday(&t1,NULL);
}

void EndTiming()
{
    gettimeofday(&t2,NULL);
    double timeTaken = 0;
    timeTaken += (t2.tv_sec - t1.tv_sec) * 1000.0 * 1000.0;
    timeTaken += (t2.tv_usec - t1.tv_usec);
    std::cout << "INFO: " << timingDesc << ": \t\t"
              << timeTaken/1000 << " milliseconds" << std::endl;
}

template <typename T>
std::string NumberToString ( T Number )
{
    std::stringstream ss;
    ss << Number;
    return ss.str();
}

int main(int argc, const char *argv[])
{
    if(argc != 3) {
        std::cout << "Usage: #> ./ptk_xform_wkt myinputfile myoutputfile\n";
        std::cout << "* Expect each line of the input file to contain a single WKT POLYGON() def\n";
        std::cout << "* The output file is in the same format as the input file\n";
        return 0;
    }

    std::cout << "Was GDAL built against GEOS?\n ";
    if(OGRGeometryFactory::haveGEOS())   {
        std::cout << "-> Yeah, WOOOOO!" << std::endl;
    }
    else   {
        std::cout << "-> Naw :(" << std::endl;
        std::cout << "-> GDAL needs to be built against GEOS to use this tool" << std::endl;
        return -1;
    }

    StartTiming("[Transform from EPSG 3785 (Mercator) to EPSG 4326 (WGS84 Lat/Lon)]");

    std::ifstream inputWktFile;
    inputWktFile.open(argv[1]);

    std::ofstream outputWktFile;
    outputWktFile.open(argv[2]);

    // get number of input lines
    unsigned int numInputLines=0;
    if(inputWktFile.is_open())   {
        while(!inputWktFile.eof())   {
            std::string wktLine;
            std::getline(inputWktFile,wktLine);
            numInputLines++;
        }
    }
    inputWktFile.close();
    inputWktFile.open(argv[1]);

    // do stuff
    if(inputWktFile.is_open() && outputWktFile.is_open())
    {
        int linesProcessed = 0;
        outputWktFile << "WKT\n";

        // setup coordinate transform (from EPSG:3785
        // [Google Mercator]) to EPSG:4326 [WGS84 lat/lon]
        OGRSpatialReference sourceSRS, targetSRS;
        sourceSRS.importFromEPSG(3785);
        targetSRS.importFromEPSG(4326);

        OGRCoordinateTransformation * coordXform;
        coordXform = OGRCreateCoordinateTransformation(&sourceSRS,&targetSRS);

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

            OGRGeometry *inputGeometry;
            OGRGeometryFactory::createFromWkt(&inputWKT, NULL, &inputGeometry);

            if (inputGeometry == NULL)   {
                std::cout << "Error: WKT is not valid (ignoring)" << std::endl;
                std::cout << "-> " << wktLine << std::endl;
                delete[] inputWKTRef;
                continue;
            }
            delete[] inputWKTRef;

            if(inputGeometry->getGeometryType() == wkbPolygon)   {
                // outer ring
                OGRPolygon *singlePoly = (OGRPolygon*)inputGeometry;
                OGRLinearRing* outerRing = singlePoly->getExteriorRing();
                for(int j=0; j < outerRing->getNumPoints(); j++)   {
                    double px = outerRing->getX(j);
                    double py = outerRing->getY(j);
                    coordXform->Transform(1,&px,&py);
                    outerRing->setPoint(j,px,py,0);
                }
                // inner rings
                for(int j=0; j < singlePoly->getNumInteriorRings(); j++)   {
                    OGRLinearRing *innerRing = singlePoly->getInteriorRing(j);
                    for(int k=0; k < innerRing->getNumPoints(); k++)   {
                        double px = innerRing->getX(k);
                        double py = innerRing->getY(k);
                        coordXform->Transform(1,&px,&py);
                        innerRing->setPoint(k,px,py,0);
                    }
                }

                // write output
                char *outputWKT;
                inputGeometry->exportToWkt(&outputWKT);
                outputWktFile << outputWKT << std::endl;
                delete[] outputWKT;

                // clean up
                delete inputGeometry;
            }
            else   {
                std::cout << "Error: Could not xform geometry, "
                             "WKT type is not a POLYGON()\n";
                std::cout << "-> WKT: " << wktLine << "\n";
            }

            linesProcessed++;
            std::cout << "ptk_xform_wkt: Lines Processed: "
                      << linesProcessed << "/" << numInputLines <<std::endl;
        }
        inputWktFile.close();
        outputWktFile.close();
    }
    EndTiming();

    return 0;
}
