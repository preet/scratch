// sys
#include <dirent.h>
#include <sys/time.h>

// stl
#include <iostream>
#include <iomanip>
#include <string>
#include <cstring>
#include <sstream>
#include <fstream>
#include <stack>
#include <set>

// osg
#include <ogrsf_frmts.h>

// clipper
#include <clipper.hpp>

// ========================================================================== //
// ========================================================================== //

#define DBLMT 1E10

double stringToNumber ( const std::string &Text )
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
    if((r1_tr_x < r2_bl_x) || (r1_bl_x > r2_tr_x) ||
       (r1_tr_y < r2_bl_y) || (r1_bl_y > r2_tr_y))
    {   return false;   }

    return true;
}

struct BoundingBox
{
    double minLat;
    double minLon;
    double maxLat;
    double maxLon;
};

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

    for(size_t i=0; i < quadKey.size(); i+=2)
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

namespace ClipperLib
{
    struct ExPolygon {
      Polygon outer;
      Polygons holes;
    };
    typedef std::vector< ExPolygon > ExPolygons;

    void AddOuterPolyNodeToExPolygons(PolyNode& polynode, ExPolygons& expolygons)
    {
      size_t cnt = expolygons.size();
      expolygons.resize(cnt + 1);
      expolygons[cnt].outer = polynode.Contour;
      expolygons[cnt].holes.resize(polynode.ChildCount());
      for (int i = 0; i < polynode.ChildCount(); ++i)
      {
        expolygons[cnt].holes[i] = polynode.Childs[i]->Contour;
        //Add outer polygons contained by (nested within) holes ...
        for (int j = 0; j < polynode.Childs[i]->ChildCount(); ++j)
          AddOuterPolyNodeToExPolygons(*polynode.Childs[i]->Childs[j], expolygons);
      }
    }

    void PolyTreeToExPolygons(PolyTree& polytree, ExPolygons& expolygons)
    {
      expolygons.clear();
      for (int i = 0; i < polytree.ChildCount(); ++i)
        AddOuterPolyNodeToExPolygons(*polytree.Childs[i], expolygons);
    }
}

// ========================================================================== //
// ========================================================================== //

int main(int argc, const char *argv[])
{
    if(argc != 8)   {
        std::cout << "Usage: #> ./shptk_quadify MINLON MAXLON MINLAT MAXLAT LEVELS inputfile.shp outputdir\n";
        std::cout << "* Expect all geometry in shapefile to be of type POLYGON only\n";
        std::cout << "* The output files are in the same format as the input file\n";
        return 0;
    }

    std::cout << "Was GDAL built against GEOS?\n ";
    if(OGRGeometryFactory::haveGEOS())   {
        std::cout << "-> Yes" << std::endl;

        // register OGR drivers
        OGRRegisterAll();
    }
    else   {
        std::cout << "-> No" << std::endl;
        std::cout << "-> GDAL needs to be built against GEOS to use this tool" << std::endl;
        return -1;
    }


    std::string inputStr;
    BoundingBox rootExtents;

    inputStr = std::string(argv[1]);
    rootExtents.minLon = stringToNumber(inputStr);

    inputStr = std::string(argv[2]);
    rootExtents.maxLon = stringToNumber(inputStr);

    inputStr = std::string(argv[3]);
    rootExtents.minLat = stringToNumber(inputStr);

    inputStr = std::string(argv[4]);
    rootExtents.maxLat = stringToNumber(inputStr);

    inputStr = std::string(argv[5]);
    unsigned int numLevels = stringToNumber(inputStr);

    // create output dir
    std::string makeDir("mkdir ");
    makeDir.append(argv[7]);
    system(makeDir.c_str());

    // create output prefix
    std::string outputDir = std::string(argv[7]) + std::string("/");
    std::string outputPrefix = std::string(argv[7]);
    outputPrefix.append("/TILE_");

    // get output driver
    std::string outpDriverName ="ESRI Shapefile";
    OGRSFDriver * opDriver;
    opDriver = OGRSFDriverRegistrar::GetRegistrar()->
            GetDriverByName(outpDriverName.c_str());
    if(opDriver == NULL)   {
        std::cout << "ERROR: " << outpDriverName
                  << " Driver not available " << std::endl;
        return -1;
    }

    std::string str00("00");
    std::string str01("01");
    std::string str10("10");
    std::string str11("11");
    std::string strExt(".shp");

    for(size_t n=0; n < numLevels; n++)   {

        std::cout << "INFO: Level " << n << "\n";

        // get a list of files in the folder
        std::vector<std::string> listFiles;
        DIR *dir; struct dirent *ent;
        dir = opendir(argv[7]);
        if(dir != NULL)   {
            while((ent=readdir(dir)) != NULL)   {
                std::string fileName(ent->d_name);
                if(!(fileName.compare(".") == 0) &&
                   !(fileName.compare("..") == 0))   {
                    std::string fileExt =
                            fileName.substr(fileName.size()-4,
                                            fileName.size()-1);
                    if(fileExt.compare(".shp") == 0)   {
                        listFiles.push_back(fileName);
                        std::cout << "INFO: Input File: "
                                  << listFiles.back() << std::endl;
                    }
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

        // split each file in the folder into four new files
        // and remove the original file when done
        for(size_t m=0; m < listFiles.size(); m++)   {

            // open input file
            OGRDataSource * ipShpFile;
            OGRLayer * ipLayer;
            size_t ixFeature = 0;
            size_t numFeatures = 0;
            std::string ipFilePath;

            // create output files for each quadrant
            OGRDataSource * opShp00;
            OGRDataSource * opShp01;
            OGRDataSource * opShp10;
            OGRDataSource * opShp11;

            std::string opFile00;
            std::string opFile01;
            std::string opFile10;
            std::string opFile11;

            BoundingBox pExtents;

            std::string fileBaseName =
                    listFiles[m].substr(0,listFiles[m].size()-4);

            if(keepPrev)   {    // ie first iteration
                ipFilePath = listFiles[m];
                opFile00 = outputPrefix + str00 + strExt;
                opFile01 = outputPrefix + str01 + strExt;
                opFile10 = outputPrefix + str10 + strExt;
                opFile11 = outputPrefix + str11 + strExt;

                // bounding box
                pExtents = rootExtents;
            }
            else   {
                ipFilePath = outputDir + listFiles[m];
                opFile00 = outputDir + fileBaseName + str00 + strExt;
                opFile01 = outputDir + fileBaseName + str01 + strExt;
                opFile10 = outputDir + fileBaseName + str10 + strExt;
                opFile11 = outputDir + fileBaseName + str11 + strExt;

                // bounding box
                std::string qKey = listFiles[m].substr(listFiles[m].find("_")+1);
                pExtents = getQuadKeyExtents(rootExtents,qKey);
            }

            // open input file
            ipShpFile = OGRSFDriverRegistrar::Open(ipFilePath.c_str(),FALSE);
            if(ipShpFile == NULL)   {
                std::cout << "ERROR: Could not open shape file "
                          << listFiles[m] << "\n";
                return -1;
            }

            // open input layer
            ipLayer = ipShpFile->GetLayer(0);
            numFeatures = ipLayer->GetFeatureCount();
//            std::cout << "INFO: Using Layer: "
//                      << ipLayer->GetName() << std::endl;


            // open up output files
            opShp00 = opDriver->CreateDataSource(opFile00.c_str(),NULL);
            if(opShp00 == NULL)   {
                std::cout << "ERROR: Creating Output File 00 Failed: "
                          << opFile00 << std::endl;
                return -1;
            }

            opShp01 = opDriver->CreateDataSource(opFile01.c_str(),NULL);
            if(opShp01 == NULL)   {
                std::cout << "ERROR: Creating Output File 01 Failed: "
                          << opFile01 << std::endl;
                return -1;
            }

            opShp10 = opDriver->CreateDataSource(opFile10.c_str(),NULL);
            if(opShp10 == NULL)   {
                std::cout << "ERROR: Creating Output File 10 Failed: "
                          << opFile10 << std::endl;
                return -1;
            }

            opShp11 = opDriver->CreateDataSource(opFile11.c_str(),NULL);
            if(opShp11 == NULL)   {
                std::cout << "ERROR: Creating Output File 11 Failed: "
                          << opFile11 << std::endl;
                return -1;
            }

            OGRLayer * opShp00Layer =
                    opShp00->CreateLayer(ipLayer->GetName(),NULL,wkbPolygon,NULL);

            OGRLayer * opShp01Layer =
                    opShp01->CreateLayer(ipLayer->GetName(),NULL,wkbPolygon,NULL);

            OGRLayer * opShp10Layer =
                    opShp10->CreateLayer(ipLayer->GetName(),NULL,wkbPolygon,NULL);

            OGRLayer * opShp11Layer =
                    opShp11->CreateLayer(ipLayer->GetName(),NULL,wkbPolygon,NULL);


            // read in geometry from the input shape file
            // feature by feature

            ipLayer->ResetReading();
            OGRFeature * ipFeature;
            while( (ipFeature = ipLayer->GetNextFeature()) != NULL )
            {
//                std::cout << "INFO: Feature " << ixFeature
//                          << "/" << numFeatures << std::endl;

                OGRGeometry * ipGeometry;
                ipGeometry = ipFeature->GetGeometryRef();

                if(ipGeometry == NULL)   {
                    std::cout << "WARN: Ignoring NULL feature: "
                              << ixFeature << std::endl;
                    continue;
                }

                if(ipGeometry->getGeometryType() != wkbPolygon)   {
                    std::cout << "WARN: Feature type is not POLYGON "
                              << "(ignoring): " << ixFeature << std::endl;
                    continue;
                }

                // get bounding box
                OGREnvelope boundingBox;
                ipGeometry->getEnvelope(&boundingBox);

                // convert input geometry to clipper format
                ClipperLib::Polygons inputPolys;

                // outer ring
                OGRPolygon *singlePoly = (OGRPolygon*)ipGeometry;
                OGRLinearRing* outerRing = singlePoly->getExteriorRing();
                ClipperLib::Polygon outerPoly;
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

                // intersection with tiles
                double halfLonStep = (pExtents.maxLon-pExtents.minLon)/2;
                double halfLatStep = (pExtents.maxLat-pExtents.minLat)/2;
                double top,btm,left,right;

                ClipperLib::Polygon gridCell;
                gridCell.push_back(ClipperLib::IntPoint(0,0));
                gridCell.push_back(ClipperLib::IntPoint(0,0));
                gridCell.push_back(ClipperLib::IntPoint(0,0));
                gridCell.push_back(ClipperLib::IntPoint(0,0));

                // TILE 00
//                std::cout << "shptk_quadify: Input File " << listFiles[m] << ": Level " << n
//                          << ", Line:" << ixFeature << "/" << numFeatures << ", Quadrant 00\n";

                top = pExtents.maxLat; btm = pExtents.minLat+halfLatStep;
                left = pExtents.minLon; right = pExtents.minLon+halfLonStep;

                if(calcAreaRectOverlap(left,btm,right,top,
                                        boundingBox.MinX,boundingBox.MinY,
                                        boundingBox.MaxX,boundingBox.MaxY))
                {
                    ClipperLib::Clipper clipperObj;
                    ClipperLib::PolyTree xsecResult;

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

                    if(clipperObj.Execute(ClipperLib::ctIntersection,xsecResult))
                    {
                        ClipperLib::ExPolygons xsecPolys;
                        ClipperLib::PolyTreeToExPolygons(xsecResult,xsecPolys);

                        for(size_t x=0; x < xsecPolys.size(); x++)   {
                            OGRPolygon savePolygon;
                            ClipperLib::ExPolygon const &xsecPoly = xsecPolys[x];

                            OGRLinearRing outerRing;
                            for(size_t y=0; y < xsecPoly.outer.size(); y++)
                            {    outerRing.addPoint(double(xsecPoly.outer[y].X)/DBLMT,double(xsecPoly.outer[y].Y)/DBLMT);   }
                            outerRing.addPoint(double(xsecPoly.outer[0].X)/DBLMT,double(xsecPoly.outer[0].Y)/DBLMT);
                            savePolygon.addRing(&outerRing);

                            for(size_t y=0; y < xsecPoly.holes.size(); y++)   {
                                OGRLinearRing innerRing;
                                for(size_t z=0; z < xsecPoly.holes[y].size(); z++)
                                {   innerRing.addPoint(double(xsecPoly.holes[y][z].X)/DBLMT,double(xsecPoly.holes[y][z].Y)/DBLMT);   }
                                innerRing.addPoint(double(xsecPoly.holes[y][0].X)/DBLMT,double(xsecPoly.holes[y][0].Y)/DBLMT);
                                savePolygon.addRing(&innerRing);
                            }

                            // save to output file
                            OGRFeature * opFeature;
                            opFeature = OGRFeature::CreateFeature(opShp00Layer->GetLayerDefn());
                            opFeature->SetGeometry(&savePolygon);

                            if(opShp00Layer->CreateFeature(opFeature) != OGRERR_NONE)   {
                                std::cout << "ERROR: Failed to create output feature\n";
                                return -1;
                            }
                            OGRFeature::DestroyFeature(opFeature);
                        }
                    }
                }

                // TILE 01
//                std::cout << "shptk_quadify: Input File " << listFiles[m] << ": Level " << n
//                          << ", Line:" << ixFeature << "/" << numFeatures << ", Quadrant 01\n";

                top = pExtents.maxLat; btm = pExtents.minLat+halfLatStep;
                left = pExtents.minLon+halfLonStep; right = pExtents.maxLon;

                if(calcAreaRectOverlap(left,btm,right,top,
                                       boundingBox.MinX,boundingBox.MinY,
                                       boundingBox.MaxX,boundingBox.MaxY))
                {
                    ClipperLib::Clipper clipperObj;
                    ClipperLib::PolyTree xsecResult;

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

                    if(clipperObj.Execute(ClipperLib::ctIntersection,xsecResult))
                    {
                        ClipperLib::ExPolygons xsecPolys;
                        ClipperLib::PolyTreeToExPolygons(xsecResult,xsecPolys);

                        for(size_t x=0; x < xsecPolys.size(); x++)   {
                            OGRPolygon savePolygon;
                            ClipperLib::ExPolygon const &xsecPoly = xsecPolys[x];

                            OGRLinearRing outerRing;
                            for(size_t y=0; y < xsecPoly.outer.size(); y++)
                            {    outerRing.addPoint(double(xsecPoly.outer[y].X)/DBLMT,double(xsecPoly.outer[y].Y)/DBLMT);   }
                            outerRing.addPoint(double(xsecPoly.outer[0].X)/DBLMT,double(xsecPoly.outer[0].Y)/DBLMT);
                            savePolygon.addRing(&outerRing);

                            for(size_t y=0; y < xsecPoly.holes.size(); y++)   {
                                OGRLinearRing innerRing;
                                for(size_t z=0; z < xsecPoly.holes[y].size(); z++)
                                {   innerRing.addPoint(double(xsecPoly.holes[y][z].X)/DBLMT,double(xsecPoly.holes[y][z].Y)/DBLMT);   }
                                innerRing.addPoint(double(xsecPoly.holes[y][0].X)/DBLMT,double(xsecPoly.holes[y][0].Y)/DBLMT);
                                savePolygon.addRing(&innerRing);
                            }

                            // save to output file
                            OGRFeature * opFeature;
                            opFeature = OGRFeature::CreateFeature(opShp01Layer->GetLayerDefn());
                            opFeature->SetGeometry(&savePolygon);

                            if(opShp01Layer->CreateFeature(opFeature) != OGRERR_NONE)   {
                                std::cout << "ERROR: Failed to create output feature\n";
                                return -1;
                            }
                            OGRFeature::DestroyFeature(opFeature);
                        }
                    }
                }

                // TILE 10
//                std::cout << "shptk_quadify: Input File " << listFiles[m] << ": Level " << n
//                          << ", Line:" << ixFeature << "/" << numFeatures << ", Quadrant 10\n";

                top = pExtents.minLat+halfLatStep; btm = pExtents.minLat;
                left = pExtents.minLon; right = pExtents.minLon+halfLonStep;

                if(calcAreaRectOverlap(left,btm,right,top,
                                       boundingBox.MinX,boundingBox.MinY,
                                       boundingBox.MaxX,boundingBox.MaxY))
                {
                    ClipperLib::Clipper clipperObj;
                    ClipperLib::PolyTree xsecResult;

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

                    if(clipperObj.Execute(ClipperLib::ctIntersection,xsecResult))
                    {
                        ClipperLib::ExPolygons xsecPolys;
                        ClipperLib::PolyTreeToExPolygons(xsecResult,xsecPolys);

                        for(size_t x=0; x < xsecPolys.size(); x++)   {
                            OGRPolygon savePolygon;
                            ClipperLib::ExPolygon const &xsecPoly = xsecPolys[x];

                            OGRLinearRing outerRing;
                            for(size_t y=0; y < xsecPoly.outer.size(); y++)
                            {    outerRing.addPoint(double(xsecPoly.outer[y].X)/DBLMT,double(xsecPoly.outer[y].Y)/DBLMT);   }
                            outerRing.addPoint(double(xsecPoly.outer[0].X)/DBLMT,double(xsecPoly.outer[0].Y)/DBLMT);
                            savePolygon.addRing(&outerRing);

                            for(size_t y=0; y < xsecPoly.holes.size(); y++)   {
                                OGRLinearRing innerRing;
                                for(size_t z=0; z < xsecPoly.holes[y].size(); z++)
                                {   innerRing.addPoint(double(xsecPoly.holes[y][z].X)/DBLMT,double(xsecPoly.holes[y][z].Y)/DBLMT);   }
                                innerRing.addPoint(double(xsecPoly.holes[y][0].X)/DBLMT,double(xsecPoly.holes[y][0].Y)/DBLMT);
                                savePolygon.addRing(&innerRing);
                            }

                            // save to output file
                            OGRFeature * opFeature;
                            opFeature = OGRFeature::CreateFeature(opShp10Layer->GetLayerDefn());
                            opFeature->SetGeometry(&savePolygon);

                            if(opShp10Layer->CreateFeature(opFeature) != OGRERR_NONE)   {
                                std::cout << "ERROR: Failed to create output feature\n";
                                return -1;
                            }
                            OGRFeature::DestroyFeature(opFeature);
                        }
                    }
                }

                // TILE 11
//                std::cout << "shptk_quadify: Input File " << listFiles[m] << ": Level " << n
//                          << ", Line:" << ixFeature << "/" << numFeatures << ", Quadrant 10\n";

                top = pExtents.minLat+halfLatStep; btm = pExtents.minLat;
                left = pExtents.minLon+halfLonStep; right = pExtents.maxLon;

                if(calcAreaRectOverlap(left,btm,right,top,
                                       boundingBox.MinX,boundingBox.MinY,
                                       boundingBox.MaxX,boundingBox.MaxY))
                {
                    ClipperLib::Clipper clipperObj;
                    ClipperLib::PolyTree xsecResult;

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

                    if(clipperObj.Execute(ClipperLib::ctIntersection,xsecResult))
                    {
                        ClipperLib::ExPolygons xsecPolys;
                        ClipperLib::PolyTreeToExPolygons(xsecResult,xsecPolys);

                        for(size_t x=0; x < xsecPolys.size(); x++)   {
                            OGRPolygon savePolygon;
                            ClipperLib::ExPolygon const &xsecPoly = xsecPolys[x];

                            OGRLinearRing outerRing;
                            for(size_t y=0; y < xsecPoly.outer.size(); y++)
                            {    outerRing.addPoint(double(xsecPoly.outer[y].X)/DBLMT,double(xsecPoly.outer[y].Y)/DBLMT);   }
                            outerRing.addPoint(double(xsecPoly.outer[0].X)/DBLMT,double(xsecPoly.outer[0].Y)/DBLMT);
                            savePolygon.addRing(&outerRing);

                            for(size_t y=0; y < xsecPoly.holes.size(); y++)   {
                                OGRLinearRing innerRing;
                                for(size_t z=0; z < xsecPoly.holes[y].size(); z++)
                                {   innerRing.addPoint(double(xsecPoly.holes[y][z].X)/DBLMT,double(xsecPoly.holes[y][z].Y)/DBLMT);   }
                                innerRing.addPoint(double(xsecPoly.holes[y][0].X)/DBLMT,double(xsecPoly.holes[y][0].Y)/DBLMT);
                                savePolygon.addRing(&innerRing);
                            }

                            // save to output file
                            OGRFeature * opFeature;
                            opFeature = OGRFeature::CreateFeature(opShp11Layer->GetLayerDefn());
                            opFeature->SetGeometry(&savePolygon);

                            if(opShp11Layer->CreateFeature(opFeature) != OGRERR_NONE)   {
                                std::cout << "ERROR: Failed to create output feature\n";
                                return -1;
                            }
                            OGRFeature::DestroyFeature(opFeature);
                        }
                    }
                }
                OGRFeature::DestroyFeature(ipFeature);
                ixFeature++;
            }

            // clean up all inputs/outputs
            OGRDataSource::DestroyDataSource(ipShpFile);
            OGRDataSource::DestroyDataSource(opShp00);
            OGRDataSource::DestroyDataSource(opShp01);
            OGRDataSource::DestroyDataSource(opShp10);
            OGRDataSource::DestroyDataSource(opShp11);

            // remove parent tile if necessary
            if(!keepPrev)   {
                // get file name (no ext)
                std::string fBaseName = listFiles[m].substr(0,listFiles[m].size()-4);
                std::string rmFile("rm ");
                std::string sysrm;

                sysrm = rmFile + outputDir + fBaseName + ".shp";
//                std::cout << "INFO: syscall: " << sysrm << std::endl;
                system(sysrm.c_str());

                sysrm = rmFile + outputDir + fBaseName + ".shx";
//                std::cout << "INFO: syscall: " << sysrm << std::endl;
                system(sysrm.c_str());

                sysrm = rmFile + outputDir + fBaseName + ".dbf";
//                std::cout << "INFO: syscall: " << sysrm << std::endl;
                system(sysrm.c_str());
            }
        }
    }

    return 0;
}
