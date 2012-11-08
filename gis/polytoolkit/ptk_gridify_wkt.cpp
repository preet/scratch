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

// clipper
#include "clipper/clipper.hpp"

// defs
// AUS_NZ
//#define MINLON 110
//#define MAXLON 180
//#define MINLAT -56
//#define MAXLAT -8
//#define LONSTEP 2
//#define LATSTEP 2
//#define DBLMT 1E10

// AMERICAS
//#define MINLON -180
//#define MAXLON -26
//#define MINLAT -58
//#define MAXLAT 84
//#define LONSTEP 2
//#define LATSTEP 2
//#define DBLMT 1E10

// split AMERICAS into 4 [expanded, should cull to above bbox later]
//#define MINLON -180
//#define MAXLON -20
//#define MINLAT -58
//#define MAXLAT 86
//#define LONSTEP 40
//#define LATSTEP 36
//#define DBLMT 1E10

#define MINLON -180
#define MAXLON 180
#define MINLAT -90
#define MAXLAT 90
#define LONSTEP 180
#define LATSTEP 180
#define DBLMT 1E10

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
              << timeTaken << " microseconds" << std::endl;
}

template <typename T>
std::string NumberToString ( T Number )
{
    std::stringstream ss;
    ss << Number;
    return ss.str();
}

double StringToNumber ( const std::string &Text )
{
    std::stringstream ss(Text);
    double result;
    return ss >> result ? result : 0;
}

bool calcAreaRectOverlap(double r1_bl_x, double r1_bl_y,
                         double r1_tr_x, double r1_tr_y,
                         double r2_bl_x, double r2_bl_y,
                         double r2_tr_x, double r2_tr_y)
{
    // check if rectangles intersect
    if((r1_tr_x < r2_bl_x) || (r1_bl_x > r2_tr_x) ||
            (r1_tr_y < r2_bl_y) || (r1_bl_y > r2_tr_y))
    {   return false;   }

    return true;
}

int main(int argc, const char *argv[])
{
    if(argc != 3) {
        std::cout << "Usage: #> ./ptk_gridify_wkt myinputfile.dat myoutputfile.ply\n";
        std::cout << "* Expect each line of the input file to contain a single WKT def\n";
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

    StartTiming("[Gridification]");

    std::ifstream inputWktFile;
    inputWktFile.open(argv[1]);

    std::ofstream outputWktFileWEST;
    outputWktFileWEST.open("OUTPUT_WEST.csv");
    outputWktFileWEST << "WKT\n";

    std::ofstream outputWktFileEAST;
    outputWktFileEAST.open("OUTPUT_EAST.csv");
    outputWktFileEAST << "WKT\n";

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
    unsigned int latExtents = MAXLAT-MINLAT;
    unsigned int lonExtents = MAXLON-MINLON;
    if(inputWktFile.is_open())
    {
        int linesProcessed = 0;
//        outputWktFile << "WKT\n";

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

            unsigned int numOuterRingPts = 0;

            // get bounding box
            OGREnvelope boundingBox;
            inputGeometry->getEnvelope(&boundingBox);

            ClipperLib::Polygons inputPolys;
            if(inputGeometry->getGeometryType() == wkbPolygon)   {
                // outer ring
                OGRPolygon *singlePoly = (OGRPolygon*)inputGeometry;
                OGRLinearRing* outerRing = singlePoly->getExteriorRing();
                ClipperLib::Polygon outerPoly;
                numOuterRingPts = outerRing->getNumPoints();
                for(int j=0; j < outerRing->getNumPoints()-1; j++)   {
                    ClipperLib::IntPoint intPt;
                    intPt.X = ClipperLib::long64(outerRing->getX(j)*DBLMT);
                    intPt.Y = ClipperLib::long64(outerRing->getY(j)*DBLMT);
                    outerPoly.push_back(intPt);
                }
                inputPolys.push_back(outerPoly);

                // inner rings
                for(int j=0; j < singlePoly->getNumInteriorRings(); j++)   {
                    OGRLinearRing *innerRing = singlePoly->getInteriorRing(j);
                    ClipperLib::Polygon innerPoly;
                    for(int k=0; k < innerRing->getNumPoints()-1; k++)   {
                        ClipperLib::IntPoint intPt;
                        intPt.X = ClipperLib::long64(innerRing->getX(k)*DBLMT);
                        intPt.Y = ClipperLib::long64(innerRing->getY(k)*DBLMT);
                        innerPoly.push_back(intPt);
                    }
                    inputPolys.push_back(innerPoly);
                }
            }
            else   {
                std::cout << "Error: Could not clip geometry, "
                             "WKT type is not a POLYGON()\n";
                std::cout << "-> WKT: " << wktLine << "\n";
                linesProcessed++;
                delete inputGeometry;
                continue;
            }

            // only do a full intersection if the bounding boxes of
            // the grid extents and geometry in question overlap
            if(!calcAreaRectOverlap(MINLON,MINLAT,MAXLON,MAXLAT,
                                    boundingBox.MinX,boundingBox.MinY,
                                    boundingBox.MaxX,boundingBox.MaxY))   {
                std::cout << "Info: Failed Bounding Box Test,ignoring poly" << std::endl;
                linesProcessed++;
                delete inputGeometry;
                continue;
            }

            delete inputGeometry;

            // gridify geometry (ClipperLib)
            unsigned int cellCount = 1;
            unsigned int numCells = ((MAXLAT-MINLAT)/LATSTEP) * ((MAXLON-MINLON)/LONSTEP);
            for(int i=0; i < (MAXLAT-MINLAT)/LATSTEP; i++)   {
                for(int j=0; j < (MAXLON-MINLON)/LONSTEP; j++)   {

                    int btm = i*LATSTEP + MINLAT;
                    int top = btm + LATSTEP;
                    int left = j*LONSTEP + MINLON;
                    int right = left+LONSTEP;

                    std::cout << "Line: " << linesProcessed << "/" << numInputLines
                              << "   Current Grid Cell: " << cellCount
                              << "/" << numCells << ", (" << numOuterRingPts
                              << " outer ring points)\n";
                    cellCount++;

                    // only do a full intersection if the bounding boxes of
                    // the grid extents and geometry in question overlap
//                    if(!calcAreaRectOverlap(left,btm,right,top,
//                                            boundingBox.MinX,boundingBox.MinY,
//                                            boundingBox.MaxX,boundingBox.MaxY))   {
////                        std::cout << "-> Failed Bounding Box Test,ignoring cell" << std::endl;
//                        continue;
//                    }

                    ClipperLib::Polygon gridCell;
                    gridCell.push_back(ClipperLib::IntPoint(ClipperLib::long64(left*DBLMT),
                                                            ClipperLib::long64(top*DBLMT)));

                    gridCell.push_back(ClipperLib::IntPoint(ClipperLib::long64(left*DBLMT),
                                                            ClipperLib::long64(btm*DBLMT)));

                    gridCell.push_back(ClipperLib::IntPoint(ClipperLib::long64(right*DBLMT),
                                                            ClipperLib::long64(btm*DBLMT)));

                    gridCell.push_back(ClipperLib::IntPoint(ClipperLib::long64(right*DBLMT),
                                                            ClipperLib::long64(top*DBLMT)));

                    ClipperLib::Clipper clipperObj;
                    clipperObj.AddPolygons(inputPolys,ClipperLib::ptSubject);
                    clipperObj.AddPolygon(gridCell,ClipperLib::ptClip);

                    ClipperLib::ExPolygons xsecPolys;
                    if(clipperObj.Execute(ClipperLib::ctIntersection,xsecPolys) && (xsecPolys.size() > 0))   {
                        // save
//                        std::cout << "      Num Intersecting Polys w/ Cell: " << xsecPolys.size() << "\n";
                        for(int x=0; x < xsecPolys.size(); x++)   {
                            OGRPolygon savePolygon;
                            ClipperLib::ExPolygon const &xsecPoly = xsecPolys[x];

                            // outer
                            OGRLinearRing outerRing;
                            for(int y=0; y < xsecPoly.outer.size(); y++)   {
                                outerRing.addPoint(double(xsecPoly.outer[y].X)/DBLMT,
                                                   double(xsecPoly.outer[y].Y)/DBLMT);
                            }
                            outerRing.addPoint(double(xsecPoly.outer[0].X)/DBLMT,
                                               double(xsecPoly.outer[0].Y)/DBLMT);
                            savePolygon.addRing(&outerRing);

                            // inner
                            for(int y=0; y < xsecPoly.holes.size(); y++)   {
                                OGRLinearRing innerRing;
                                for(int z=0; z < xsecPoly.holes[y].size(); z++)   {
                                    innerRing.addPoint(double(xsecPoly.holes[y][z].X)/DBLMT,
                                                       double(xsecPoly.holes[y][z].Y)/DBLMT);
                                }
                                innerRing.addPoint(double(xsecPoly.holes[y][0].X)/DBLMT,
                                                   double(xsecPoly.holes[y][0].Y)/DBLMT);
                                savePolygon.addRing(&innerRing);
                            }

                            // save as wkt
                            char *outputWKT;
                            savePolygon.exportToWkt(&outputWKT);

                            if(j==0)   {
                                outputWktFileWEST << outputWKT << std::endl;
                            }
                            else   {
                                outputWktFileEAST << outputWKT << std::endl;
                            }

                            delete[] outputWKT;

                        }
                    }
                }
            }

            linesProcessed++;
            std::cout << "ptk_gridify_wkt: Lines Processed: "
                      << linesProcessed << "/" << numInputLines <<std::endl;
        }
        inputWktFile.close();
        outputWktFileWEST.close();
        outputWktFileEAST.close();
    }
    EndTiming();

    return 0;
}
