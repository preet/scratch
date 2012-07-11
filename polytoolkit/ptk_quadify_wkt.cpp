// STL
#include <iostream>
#include <iomanip>
#include <cstring>
#include <sstream>
#include <fstream>
#include <stack>
#include <set>
#include <dirent.h>
#include <sys/time.h>

// OGR
#include <ogrsf_frmts.h>

// clipper
#include "clipper/clipper.hpp"

#define MINLON -180
#define MAXLON -20
#define MINLAT -58
#define MAXLAT 86
#define LONSTEP 20
#define LATSTEP 18
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

struct BoundingBox
{
    double minLat;
    double minLon;
    double maxLat;
    double maxLon;
};


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

BoundingBox getQuadKeyExtents(BoundingBox const &rootExtents,
                              std::string const &quadKey)
{
    BoundingBox myExtents;
    myExtents = rootExtents;

    if(quadKey.size() < 2)   {
        return myExtents;
    }

    if(quadKey.size() % 2 != 0)   {
        return myExtents;
    }

    for(int i=0; i < quadKey.size(); i+=2)
    {
        std::string quadrant = quadKey.substr(i,2);
        double halfLonStep = (myExtents.maxLon-myExtents.minLon)/2;
        double halfLatStep = (myExtents.maxLat-myExtents.minLat)/2;

        // top left quadrant
        if(quadrant.compare("00") == 0)   {
            myExtents.maxLon -= halfLonStep;
            myExtents.minLat += halfLatStep;
        }
        // top right quadrant
        else if(quadrant.compare("01") == 0)   {
            myExtents.minLon += halfLonStep;
            myExtents.minLat += halfLatStep;
        }
        // bottom left quadrant
        else if(quadrant.compare("10") == 0)   {
            myExtents.maxLon -= halfLonStep;
            myExtents.maxLat -= halfLatStep;
        }
        // bottom right quadrant
        else if(quadrant.compare("11") == 0)   {
            myExtents.minLon += halfLonStep;
            myExtents.maxLat -= halfLatStep;
        }
    }

//    // debug
//    std::cout << "New Extents: "
//              << myExtents.minLon << "," << myExtents.maxLon << "|"
//              << myExtents.minLat << "," << myExtents.maxLat
//              << std::endl;

    return myExtents;
}

int main(int argc, const char *argv[])
{
    if(argc != 8) {
        std::cout << "Usage: #> ./ptk_quadify_wkt MINLON MAXLON MINLAT MAXLAT LEVELS inputfile.dat outputdir\n";
        std::cout << "* Expect each line of the input file to contain a single WKT def\n";
        std::cout << "* The output file is in the same format as the input file\n";
        return 0;
    }

    std::cout << "Was GDAL built against GEOS?\n ";
    if(OGRGeometryFactory::haveGEOS())   {
        std::cout << "-> Yeah, Good to go!" << std::endl;
    }
    else   {
        std::cout << "-> Naw :(" << std::endl;
        std::cout << "-> GDAL needs to be built against GEOS to use this tool" << std::endl;
        return -1;
    }

    StartTiming("[Quadification]");

    std::string inputStr;
    BoundingBox rootExtents;

    inputStr = std::string(argv[1]);
    rootExtents.minLon = StringToNumber(inputStr);

    inputStr = std::string(argv[2]);
    rootExtents.maxLon = StringToNumber(inputStr);

    inputStr = std::string(argv[3]);
    rootExtents.minLat = StringToNumber(inputStr);

    inputStr = std::string(argv[4]);
    rootExtents.maxLat = StringToNumber(inputStr);

    inputStr = std::string(argv[5]);
    unsigned int numLevels = StringToNumber(inputStr);

    // create output dir
    std::string makeDir("mkdir ");
    makeDir.append(argv[7]);
    system(makeDir.c_str());

    // create output prefix
    std::string outputDir = std::string(argv[7]) + std::string("/");
    std::string outputPrefix = std::string(argv[7]);
    outputPrefix.append("/TILE_");

    std::string str00("00");
    std::string str01("01");
    std::string str10("10");
    std::string str11("11");

    for(int n=0; n < numLevels; n++)   {

        std::cout << "ptk_quadify_wkt: Level " << n << "\n";

        // get a list of files in the folder
        std::vector<std::string> listFiles;
        DIR *dir; struct dirent *ent;
        dir = opendir(argv[7]);
        if(dir != NULL)   {
            while((ent=readdir(dir)) != NULL)   {
                std::string fileName(ent->d_name);
                if(!(fileName.compare(".") == 0) &&
                        !(fileName.compare("..") == 0))   {
                    listFiles.push_back(fileName);
                }
            }
        }

        bool keepPrev = false;

        // if this is the first iteration,
        // use the inputfile as source
        if(listFiles.size() == 0)   {
            keepPrev = true;
            std::string inputFile(argv[6]);
            listFiles.push_back(inputFile);
        }

        // split each file in the folder
        // into four new files and remove the
        // original file when done
        for(int m=0; m < listFiles.size(); m++)   {
            std::ifstream inputWktFile;
            std::ofstream tile00;
            std::ofstream tile01;
            std::ofstream tile10;
            std::ofstream tile11;
            unsigned int numInputLines=0;
            unsigned int linesProcessed=0;
            BoundingBox pExtents;

            if(keepPrev)   {    // ie first iteration
                tile00.open(outputPrefix+"00");
                tile01.open(outputPrefix+"01");
                tile10.open(outputPrefix+"10");
                tile11.open(outputPrefix+"11");
                inputWktFile.open(listFiles[m].c_str());

                // bounding box
                pExtents = rootExtents;

                // line count
                while(!inputWktFile.eof())   {
                    std::string wktLine;
                    std::getline(inputWktFile,wktLine);
                    numInputLines++;
                }
                inputWktFile.close();
                inputWktFile.open(listFiles[m].c_str());

//                std::cout << "ptk_quadify_wkt: Input File " << listFiles[m]
//                          << " has " << numInputLines << " lines\n";
            }
            else   {
                tile00.open(outputDir+listFiles[m]+"00");
                tile01.open(outputDir+listFiles[m]+"01");
                tile10.open(outputDir+listFiles[m]+"10");
                tile11.open(outputDir+listFiles[m]+"11");
                std::string tileFile = outputDir+listFiles[m];
                inputWktFile.open(tileFile.c_str());

                // bounding box
                std::string qKey = listFiles[m].substr(listFiles[m].find("_")+1);
                pExtents = getQuadKeyExtents(rootExtents,qKey);

                // line count
                while(!inputWktFile.eof())   {
                    std::string wktLine;
                    std::getline(inputWktFile,wktLine);
                    numInputLines++;
                }
                inputWktFile.close();
                inputWktFile.open(tileFile.c_str());

//                std::cout << "ptk_quadify_wkt: Input File " << listFiles[m]
//                          << " has " << numInputLines << " lines\n";
            }

            // csv wkt 'header'
//            tile00 << "WKT\n";
//            tile01 << "WKT\n";
//            tile10 << "WKT\n";
//            tile11 << "WKT\n";

            // read in geometry from input file line by line
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
//                    std::cout << "Error: WKT is not valid (ignoring)" << std::endl;
//                    std::cout << "-> " << wktLine << std::endl;
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
                    delete inputGeometry;
                }
                else   {
//                    std::cout << "Error: Could not clip geometry, "
//                                 "WKT type is not a POLYGON()\n";
//                    std::cout << "-> WKT: " << wktLine << "\n";
                    linesProcessed++;
                    delete inputGeometry;
                    continue;
                }

                // intersection with tiles
                double halfLonStep = (pExtents.maxLon-pExtents.minLon)/2;
                double halfLatStep = (pExtents.maxLat-pExtents.minLat)/2;
                double top,btm,left,right;

                ClipperLib::Polygon gridCell;
                gridCell.push_back(ClipperLib::IntPoint(0,0));
                gridCell.push_back(ClipperLib::IntPoint(0,0));
                gridCell.push_back(ClipperLib::IntPoint(0,0));
                gridCell.push_back(ClipperLib::IntPoint(0,0));

                // [tile00]
                std::cout << "ptk_quadify_wkt: Input File " << listFiles[m] << ": Level " << n
                          << ", Line:" << linesProcessed << "/" << numInputLines << ", Quadrant 00\n";

                top = pExtents.maxLat; btm = pExtents.minLat+halfLatStep;
                left = pExtents.minLon; right = pExtents.minLon+halfLonStep;

                if(calcAreaRectOverlap(left,btm,right,top,
                                        boundingBox.MinX,boundingBox.MinY,
                                        boundingBox.MaxX,boundingBox.MaxY))
                {
                    ClipperLib::Clipper clipperObj;
                    ClipperLib::ExPolygons xsecPolys;

                    gridCell[0].X = ClipperLib::long64(left*DBLMT);
                    gridCell[0].Y = ClipperLib::long64(top*DBLMT);
                    gridCell[1].X = ClipperLib::long64(left*DBLMT);
                    gridCell[1].Y = ClipperLib::long64(btm*DBLMT);
                    gridCell[2].X = ClipperLib::long64(right*DBLMT);
                    gridCell[2].Y = ClipperLib::long64(btm*DBLMT);
                    gridCell[3].X = ClipperLib::long64(right*DBLMT);
                    gridCell[3].Y = ClipperLib::long64(top*DBLMT);

                    clipperObj.AddPolygons(inputPolys,ClipperLib::ptSubject);
                    clipperObj.AddPolygon(gridCell,ClipperLib::ptClip);

                    if(clipperObj.Execute(ClipperLib::ctIntersection,xsecPolys) && (xsecPolys.size() > 0))   {
                        for(int x=0; x < xsecPolys.size(); x++)   {
                            OGRPolygon savePolygon;
                            ClipperLib::ExPolygon const &xsecPoly = xsecPolys[x];

                            OGRLinearRing outerRing;
                            for(int y=0; y < xsecPoly.outer.size(); y++)
                            {    outerRing.addPoint(double(xsecPoly.outer[y].X)/DBLMT,double(xsecPoly.outer[y].Y)/DBLMT);   }
                            outerRing.addPoint(double(xsecPoly.outer[0].X)/DBLMT,double(xsecPoly.outer[0].Y)/DBLMT);
                            savePolygon.addRing(&outerRing);

                            for(int y=0; y < xsecPoly.holes.size(); y++)   {
                                OGRLinearRing innerRing;
                                for(int z=0; z < xsecPoly.holes[y].size(); z++)
                                {   innerRing.addPoint(double(xsecPoly.holes[y][z].X)/DBLMT,double(xsecPoly.holes[y][z].Y)/DBLMT);   }
                                innerRing.addPoint(double(xsecPoly.holes[y][0].X)/DBLMT,double(xsecPoly.holes[y][0].Y)/DBLMT);
                                savePolygon.addRing(&innerRing);
                            }
                            // save as wkt
                            char *outputWKT;
                            savePolygon.exportToWkt(&outputWKT);
                            tile00 << outputWKT << std::endl;
                            delete[] outputWKT;
                        }
                    }
                }

                // [tile01]
                std::cout << "ptk_quadify_wkt: Input File " << listFiles[m] << ": Level " << n
                          << ", Line:" << linesProcessed << "/" << numInputLines << ", Quadrant 01\n";

                top = pExtents.maxLat; btm = pExtents.minLat+halfLatStep;
                left = pExtents.minLon+halfLonStep; right = pExtents.maxLon;

                if(calcAreaRectOverlap(left,btm,right,top,
                                        boundingBox.MinX,boundingBox.MinY,
                                        boundingBox.MaxX,boundingBox.MaxY))
                {
                    ClipperLib::Clipper clipperObj;
                    ClipperLib::ExPolygons xsecPolys;

                    gridCell[0].X = ClipperLib::long64(left*DBLMT);
                    gridCell[0].Y = ClipperLib::long64(top*DBLMT);
                    gridCell[1].X = ClipperLib::long64(left*DBLMT);
                    gridCell[1].Y = ClipperLib::long64(btm*DBLMT);
                    gridCell[2].X = ClipperLib::long64(right*DBLMT);
                    gridCell[2].Y = ClipperLib::long64(btm*DBLMT);
                    gridCell[3].X = ClipperLib::long64(right*DBLMT);
                    gridCell[3].Y = ClipperLib::long64(top*DBLMT);

                    clipperObj.AddPolygons(inputPolys,ClipperLib::ptSubject);
                    clipperObj.AddPolygon(gridCell,ClipperLib::ptClip);

                    if(clipperObj.Execute(ClipperLib::ctIntersection,xsecPolys) && (xsecPolys.size() > 0))   {
                        for(int x=0; x < xsecPolys.size(); x++)   {
                            OGRPolygon savePolygon;
                            ClipperLib::ExPolygon const &xsecPoly = xsecPolys[x];

                            OGRLinearRing outerRing;
                            for(int y=0; y < xsecPoly.outer.size(); y++)
                            {    outerRing.addPoint(double(xsecPoly.outer[y].X)/DBLMT,double(xsecPoly.outer[y].Y)/DBLMT);   }
                            outerRing.addPoint(double(xsecPoly.outer[0].X)/DBLMT,double(xsecPoly.outer[0].Y)/DBLMT);
                            savePolygon.addRing(&outerRing);

                            for(int y=0; y < xsecPoly.holes.size(); y++)   {
                                OGRLinearRing innerRing;
                                for(int z=0; z < xsecPoly.holes[y].size(); z++)
                                {   innerRing.addPoint(double(xsecPoly.holes[y][z].X)/DBLMT,double(xsecPoly.holes[y][z].Y)/DBLMT);   }
                                innerRing.addPoint(double(xsecPoly.holes[y][0].X)/DBLMT,double(xsecPoly.holes[y][0].Y)/DBLMT);
                                savePolygon.addRing(&innerRing);
                            }
                            // save as wkt
                            char *outputWKT;
                            savePolygon.exportToWkt(&outputWKT);
                            tile01 << outputWKT << std::endl;
                            delete[] outputWKT;
                        }
                    }
                }

                // [tile10]
                std::cout << "ptk_quadify_wkt: Input File " << listFiles[m] << ": Level " << n
                          << ", Line:" << linesProcessed << "/" << numInputLines << ", Quadrant 10\n";

                top = pExtents.minLat+halfLatStep; btm = pExtents.minLat;
                left = pExtents.minLon; right = pExtents.minLon+halfLonStep;

                if(calcAreaRectOverlap(left,btm,right,top,
                                        boundingBox.MinX,boundingBox.MinY,
                                        boundingBox.MaxX,boundingBox.MaxY))
                {
                    ClipperLib::Clipper clipperObj;
                    ClipperLib::ExPolygons xsecPolys;

                    gridCell[0].X = ClipperLib::long64(left*DBLMT);
                    gridCell[0].Y = ClipperLib::long64(top*DBLMT);
                    gridCell[1].X = ClipperLib::long64(left*DBLMT);
                    gridCell[1].Y = ClipperLib::long64(btm*DBLMT);
                    gridCell[2].X = ClipperLib::long64(right*DBLMT);
                    gridCell[2].Y = ClipperLib::long64(btm*DBLMT);
                    gridCell[3].X = ClipperLib::long64(right*DBLMT);
                    gridCell[3].Y = ClipperLib::long64(top*DBLMT);

                    clipperObj.AddPolygons(inputPolys,ClipperLib::ptSubject);
                    clipperObj.AddPolygon(gridCell,ClipperLib::ptClip);

                    if(clipperObj.Execute(ClipperLib::ctIntersection,xsecPolys) && (xsecPolys.size() > 0))   {
                        for(int x=0; x < xsecPolys.size(); x++)   {
                            OGRPolygon savePolygon;
                            ClipperLib::ExPolygon const &xsecPoly = xsecPolys[x];

                            OGRLinearRing outerRing;
                            for(int y=0; y < xsecPoly.outer.size(); y++)
                            {    outerRing.addPoint(double(xsecPoly.outer[y].X)/DBLMT,double(xsecPoly.outer[y].Y)/DBLMT);   }
                            outerRing.addPoint(double(xsecPoly.outer[0].X)/DBLMT,double(xsecPoly.outer[0].Y)/DBLMT);
                            savePolygon.addRing(&outerRing);

                            for(int y=0; y < xsecPoly.holes.size(); y++)   {
                                OGRLinearRing innerRing;
                                for(int z=0; z < xsecPoly.holes[y].size(); z++)
                                {   innerRing.addPoint(double(xsecPoly.holes[y][z].X)/DBLMT,double(xsecPoly.holes[y][z].Y)/DBLMT);   }
                                innerRing.addPoint(double(xsecPoly.holes[y][0].X)/DBLMT,double(xsecPoly.holes[y][0].Y)/DBLMT);
                                savePolygon.addRing(&innerRing);
                            }
                            // save as wkt
                            char *outputWKT;
                            savePolygon.exportToWkt(&outputWKT);
                            tile10 << outputWKT << std::endl;
                            delete[] outputWKT;
                        }
                    }
                }

                // [tile11]
                std::cout << "ptk_quadify_wkt: Input File " << listFiles[m] << ": Level " << n
                          << ", Line:" << linesProcessed << "/" << numInputLines << ", Quadrant 11\n";

                top = pExtents.minLat+halfLatStep; btm = pExtents.minLat;
                left = pExtents.minLon+halfLonStep; right = pExtents.maxLon;

                if(calcAreaRectOverlap(left,btm,right,top,
                                        boundingBox.MinX,boundingBox.MinY,
                                        boundingBox.MaxX,boundingBox.MaxY))
                {
                    ClipperLib::Clipper clipperObj;
                    ClipperLib::ExPolygons xsecPolys;

                    gridCell[0].X = ClipperLib::long64(left*DBLMT);
                    gridCell[0].Y = ClipperLib::long64(top*DBLMT);
                    gridCell[1].X = ClipperLib::long64(left*DBLMT);
                    gridCell[1].Y = ClipperLib::long64(btm*DBLMT);
                    gridCell[2].X = ClipperLib::long64(right*DBLMT);
                    gridCell[2].Y = ClipperLib::long64(btm*DBLMT);
                    gridCell[3].X = ClipperLib::long64(right*DBLMT);
                    gridCell[3].Y = ClipperLib::long64(top*DBLMT);

                    clipperObj.AddPolygons(inputPolys,ClipperLib::ptSubject);
                    clipperObj.AddPolygon(gridCell,ClipperLib::ptClip);

                    if(clipperObj.Execute(ClipperLib::ctIntersection,xsecPolys) && (xsecPolys.size() > 0))   {
                        for(int x=0; x < xsecPolys.size(); x++)   {
                            OGRPolygon savePolygon;
                            ClipperLib::ExPolygon const &xsecPoly = xsecPolys[x];

                            OGRLinearRing outerRing;
                            for(int y=0; y < xsecPoly.outer.size(); y++)
                            {    outerRing.addPoint(double(xsecPoly.outer[y].X)/DBLMT,double(xsecPoly.outer[y].Y)/DBLMT);   }
                            outerRing.addPoint(double(xsecPoly.outer[0].X)/DBLMT,double(xsecPoly.outer[0].Y)/DBLMT);
                            savePolygon.addRing(&outerRing);

                            for(int y=0; y < xsecPoly.holes.size(); y++)   {
                                OGRLinearRing innerRing;
                                for(int z=0; z < xsecPoly.holes[y].size(); z++)
                                {   innerRing.addPoint(double(xsecPoly.holes[y][z].X)/DBLMT,double(xsecPoly.holes[y][z].Y)/DBLMT);   }
                                innerRing.addPoint(double(xsecPoly.holes[y][0].X)/DBLMT,double(xsecPoly.holes[y][0].Y)/DBLMT);
                                savePolygon.addRing(&innerRing);
                            }
                            // save as wkt
                            char *outputWKT;
                            savePolygon.exportToWkt(&outputWKT);
                            tile11 << outputWKT << std::endl;
                            delete[] outputWKT;
                        }
                    }
                }
                linesProcessed++;
            }

            // close input file
            inputWktFile.close();

            // close newly completed tiles
            tile00.close();
            tile01.close();
            tile10.close();
            tile11.close();

            // remove parent tile if necessary
            if(!keepPrev)   {
                std::string rmFile("rm ");
                rmFile.append(outputDir+listFiles[m]);
                system(rmFile.c_str());
            }
        }
    }
    EndTiming();

    return 0;
}
