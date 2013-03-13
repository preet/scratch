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

struct BoundingBox
{
    double minLat;
    double minLon;
    double maxLat;
    double maxLon;
};

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
    // check if rectangles intersect
    if((r1_tr_x < r2_bl_x) || (r1_bl_x > r2_tr_x) ||
            (r1_tr_y < r2_bl_y) || (r1_bl_y > r2_tr_y))
    {   return false;   }

    return true;
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
    if(argc != 7)   {
        std::cout << "Usage: #> ./shptk_crop MINLON MAXLON MINLAT MAXLAT inputfile.shp outputfile.shp\n";
        std::cout << "* Expect all geometry in shapefile to be of type POLYGON only\n";
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

    // parse input args
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

    std::string inputFile(argv[5]);
    std::string outputFile(argv[6]);

    // open input file
    OGRDataSource * poDS;
    poDS = OGRSFDriverRegistrar::Open(inputFile.c_str(),FALSE);
    if(poDS == NULL)   {
        std::cout << "ERROR: Could not open input file: "
                  << inputFile << std::endl;
        return -1;
    }

    // open output driver
    std::string outpDriverName ="ESRI Shapefile";
    OGRSFDriver * opDriver;
    opDriver = OGRSFDriverRegistrar::GetRegistrar()->
            GetDriverByName(outpDriverName.c_str());
    if(opDriver == NULL)   {
        std::cout << "ERROR: " << outpDriverName
                  << " Driver not available " << std::endl;
        return -1;
    }

    // define the intersection box
    ClipperLib::Polygon xsecBox;
    xsecBox.push_back(ClipperLib::IntPoint(0,0));
    xsecBox.push_back(ClipperLib::IntPoint(0,0));
    xsecBox.push_back(ClipperLib::IntPoint(0,0));
    xsecBox.push_back(ClipperLib::IntPoint(0,0));
    // top left
    xsecBox[0].X = ClipperLib::long64(rootExtents.minLon * DBLMT);
    xsecBox[0].Y = ClipperLib::long64(rootExtents.maxLat * DBLMT);
    // bottom left
    xsecBox[1].X = ClipperLib::long64(rootExtents.minLon * DBLMT);
    xsecBox[1].Y = ClipperLib::long64(rootExtents.minLat * DBLMT);
    // bottom right
    xsecBox[2].X = ClipperLib::long64(rootExtents.maxLon * DBLMT);
    xsecBox[2].Y = ClipperLib::long64(rootExtents.minLat * DBLMT);
    // top right
    xsecBox[3].X = ClipperLib::long64(rootExtents.maxLon * DBLMT);
    xsecBox[3].Y = ClipperLib::long64(rootExtents.maxLat * DBLMT);

    // open input layer
    OGRLayer * poLayer;
    poLayer = poDS->GetLayer(0);
    std::cout << "INFO: Using Layer: " << poLayer->GetName()
              << std::endl;

    // prepare output
    OGRDataSource * opDS;
    opDS = opDriver->CreateDataSource(outputFile.c_str(),NULL);
    if(opDS == NULL)   {
        std::cout << "ERROR: Could not create file for output\n";
        return -1;
    }

    OGRLayer * opLayer;
    opLayer = opDS->CreateLayer(poLayer->GetName(),NULL,wkbPolygon,NULL);
    if(opLayer == NULL)   {
        std::cout << "ERROR: Could not create output layer\n";
        return -1;
    }

    // iterate through input features
    poLayer->ResetReading();

    size_t ixFeature = 0;
    size_t numFeatures = poLayer->GetFeatureCount();

    OGRFeature * poFeature;
    while( (poFeature = poLayer->GetNextFeature()) != NULL )
    {
        std::cout << "INFO: Feature " << ixFeature
                  << "/" << numFeatures << std::endl;

        OGRGeometry * poGeometry;
        poGeometry = poFeature->GetGeometryRef();

        if(poGeometry == NULL)   {
            std::cout << "WARN: Ignoring NULL feature: "
                      << ixFeature << std::endl;
            continue;
        }

        if(poGeometry->getGeometryType() != wkbPolygon)   {
            std::cout << "WARN: Feature type is not POLYGON "
                      << "(ignoring): " << ixFeature << std::endl;
            continue;
        }

        // get the polygon geometry for this feature
        ClipperLib::Polygons inputPolys;
        OGRPolygon * poPoly = (OGRPolygon*)poGeometry;

        // bounding box
        OGREnvelope boundingBox;
        poGeometry->getEnvelope(&boundingBox);

        // outer ring
        ClipperLib::Polygon outerPoly;
        OGRLinearRing * outerRing = poPoly->getExteriorRing();
        for(int j=0; j < outerRing->getNumPoints()-1; j++)   {
            ClipperLib::IntPoint intPt;
            intPt.X = ClipperLib::long64(outerRing->getX(j)*DBLMT);
            intPt.Y = ClipperLib::long64(outerRing->getY(j)*DBLMT);
            outerPoly.push_back(intPt);
        }
        inputPolys.push_back(outerPoly);

        // inner rings
        for(int j=0; j < poPoly->getNumInteriorRings(); j++)   {
            OGRLinearRing *innerRing = poPoly->getInteriorRing(j);
            ClipperLib::Polygon innerPoly;
            for(int k=0; k < innerRing->getNumPoints()-1; k++)   {
                ClipperLib::IntPoint intPt;
                intPt.X = ClipperLib::long64(innerRing->getX(k)*DBLMT);
                intPt.Y = ClipperLib::long64(innerRing->getY(k)*DBLMT);
                innerPoly.push_back(intPt);
            }
            inputPolys.push_back(innerPoly);
        }

        // quick bbox test before full intersection test
        if(calcAreaRectOverlap(rootExtents.minLon,rootExtents.minLat,
                               rootExtents.maxLon,rootExtents.maxLat,
                               boundingBox.MinX,boundingBox.MinY,
                               boundingBox.MaxX,boundingBox.MaxY))
        {
            ClipperLib::Clipper clipper;
            ClipperLib::PolyTree xsecResult;
            clipper.AddPolygons(inputPolys,ClipperLib::ptSubject);
            clipper.AddPolygon(xsecBox,ClipperLib::ptClip);

            if(clipper.Execute(ClipperLib::ctIntersection,xsecResult))
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

                    // save in output file
                    OGRFeature * opFeature;
                    opFeature = OGRFeature::CreateFeature(opLayer->GetLayerDefn());
                    opFeature->SetGeometry(&savePolygon);

                    if(opLayer->CreateFeature(opFeature) != OGRERR_NONE)   {
                        std::cout << "ERROR: Failed to create output feature\n";
                        return -1;
                    }
                    OGRFeature::DestroyFeature(opFeature);
                }
            }
        }
        OGRFeature::DestroyFeature(poFeature);
        ixFeature++;
    }

    // clean up input,output datasource
    OGRDataSource::DestroyDataSource(poDS);
    OGRDataSource::DestroyDataSource(opDS);


    return 0;
}
