// STL
#include <iostream>
#include <iomanip>
#include <cstring>
#include <sstream>
#include <fstream>
#include <stack>
#include <vector>
#include <set>
#include <map>
#include <algorithm>
#include <sys/time.h>

// OGR
#include <ogrsf_frmts.h>

#include "Vec2.hpp"

// defs
#define SIMPLIFY_MODE 1
#define DP_DIST 500
#define VW_AREA 4000    // 2250-4000 seems decent

enum SimplifyMode
{
    DOUGLAS_PEUCKER = 0,
    VISVALINGAM_WHYATT = 1
};

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

double calcAreaTriangle(double a_x, double a_y,
                        double b_x, double b_y,
                        double c_x, double c_y)
{   // http://www.mathopenref.com/coordtrianglearea.html
    return fabs((a_x*(b_y-c_y) + b_x*(c_y-a_y) + c_x*(a_y-b_y))/2);
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

void simplifyWithVW(OGRLinearRing *polyLine)
{
    // * compute the effective area of each point and store
    //   it in a list with the corresponding points
    //   (note, first last pts don't have an eff. area)
    std::multimap<double,unsigned int> pointsByArea;
    std::map<unsigned int,double> areasByPoint;

    unsigned int numPts = polyLine->getNumPoints()-1;   // ignore the last point since
                                                        // the last point == first point
    for(unsigned int i=1; i < numPts-1; i++)   {
        double triArea = calcAreaTriangle(polyLine->getX(i-1),polyLine->getY(i-1),
                                          polyLine->getX(i+0),polyLine->getY(i+0),
                                          polyLine->getX(i+1),polyLine->getY(i+1));

        // * delete all the points that have an effective
        //   area of zero (this indicates a colinear set of pts)
        if(triArea > 0)   {
            std::pair<double,unsigned int> insArea(triArea,i);
            pointsByArea.insert(insArea);

            std::pair<unsigned int,double> insPt(i,triArea);
            areasByPoint.insert(insPt);
        }
    }

    // * expect pointsByArea and areasByPoint to be in sync
    unsigned int sIdx = 0;
    unsigned int eIdx = numPts-1;

    std::multimap<double,unsigned int>::iterator aListIt;
    std::map<unsigned int,double>::iterator pListIt,tempIt,lastIt;
    std::pair<std::multimap<double,unsigned int>::iterator,
              std::multimap<double,unsigned int>::iterator> itRange;

//    for(pListIt = areasByPoint.begin();
//        pListIt != areasByPoint.end(); ++pListIt)   {
////        std::cout << "PIdx: " << pListIt->first << ", " << pListIt->second << "\n";
//    }

//    for(aListIt = pointsByArea.begin();
//        aListIt != pointsByArea.end(); ++aListIt)   {
////        std::cout << "Area: " << aListIt->first << " " << aListIt->second << "\n";
//    }

    while(1) {

        if(pointsByArea.size() < 3)   {
            break;
        }

        // find the point with the least effective area
        // and call it the current point
        aListIt = pointsByArea.begin();
        pListIt = areasByPoint.find(aListIt->second);
        lastIt = areasByPoint.end(); --lastIt;

        unsigned int cIdx = pListIt->first; // current point
        unsigned int pIdx,pIdx_prev,nIdx,nIdx_next;

        // indicates if the current point is at either end;
        // note that a point cant be first AND last because
        // we have a minimum of four points per poly
        bool isFirst,isLast;

        if(pListIt == areasByPoint.begin())   {             // is first
            isFirst = true;
            tempIt = pListIt;
                                    pIdx = sIdx;
            std::advance(tempIt,1); nIdx = tempIt->first;
            std::advance(tempIt,1); nIdx_next = tempIt->first;

//            std::cout << "First Tri: " << pIdx <<","<<cIdx<<","<<nIdx<<","<<aListIt->first<<"\n";
        }
        else if(pListIt == lastIt)   {        // is last
            isLast = true;
            tempIt = pListIt;

            std::advance(tempIt,-2); pIdx_prev = tempIt->first;
            std::advance(tempIt,1);  pIdx = tempIt->first;
                                     nIdx = eIdx;
//            std::cout << "Last Tri: " << pIdx <<","<<cIdx<<","<<nIdx<<","<<aListIt->first<<"\n";
        }
        else   {                                            // is inbetween
            isFirst = false;
            isLast = false;

            // set pIdx and pIdx_prev
            tempIt = pListIt;
            std::advance(tempIt,-1); pIdx=tempIt->first;
            if(tempIt == areasByPoint.begin())
            {   pIdx_prev=sIdx;   }
            else
            {   std::advance(tempIt,-1); pIdx_prev=tempIt->first;   }

            // set nIdx and nIdx_next
            tempIt = pListIt;
            std::advance(tempIt,1);  nIdx = tempIt->first;
            if(tempIt == lastIt)
            {   nIdx_next=eIdx;   }
            else
            {   std::advance(tempIt,1);  nIdx_next=tempIt->first;   }

//            std::cout << "Middle Tri: "
//                      << pIdx <<","<<cIdx<<","<<nIdx<<","<<aListIt->first
//                      << " | ";
//            std::cout << "\n";
        }

        // if the area of the current point is below the
        // threshold, delete it from the polyline
        if(aListIt->first < VW_AREA)   {
            // delete the point from our maps
            pointsByArea.erase(aListIt);
            areasByPoint.erase(pListIt);

            if(!isLast)   {
                // recalculate the adjacent triangle
                // area for the next index (pIdx,nIdx,nIdx_next)

                // * find the elements we have to update in each map
                pListIt = areasByPoint.find(nIdx);
                itRange = pointsByArea.equal_range(pListIt->second);
                for(aListIt = itRange.first; aListIt != itRange.second; ++aListIt)   {
                    if(aListIt->second == nIdx)   {
                        break;
                    }
                }
                // * recalculate area
                double nIdxArea = calcAreaTriangle(polyLine->getX(pIdx),
                                                   polyLine->getY(pIdx),
                                                   polyLine->getX(nIdx),
                                                   polyLine->getY(nIdx),
                                                   polyLine->getX(nIdx_next),
                                                   polyLine->getY(nIdx_next));

                // * replace old entries in both maps for nIdx
                pointsByArea.erase(aListIt);
                areasByPoint.erase(pListIt);

                std::pair<double,unsigned int> insArea(nIdxArea,nIdx);
                pointsByArea.insert(insArea);

                std::pair<unsigned int,double> insPt(nIdx,nIdxArea);
                areasByPoint.insert(insPt);
            }

            if(!isFirst)   {
                // recalculate the adjacent triangle
                // area for the prev index (nIdx,pIdx,pIdx_prev)

                // * find the elements we have to update in each map
                pListIt = areasByPoint.find(pIdx);
                itRange = pointsByArea.equal_range(pListIt->second);
                for(aListIt = itRange.first; aListIt != itRange.second; ++aListIt)   {
                    if(aListIt->second == pIdx)   {
                        break;
                    }
                }
                // * recalculate area
                double pIdxArea = calcAreaTriangle(polyLine->getX(nIdx),
                                                   polyLine->getY(nIdx),
                                                   polyLine->getX(pIdx),
                                                   polyLine->getY(pIdx),
                                                   polyLine->getX(pIdx_prev),
                                                   polyLine->getY(pIdx_prev));

                // * replace old entries in both maps for pIdx
                pointsByArea.erase(aListIt);
                areasByPoint.erase(pListIt);

                std::pair<double,unsigned int> insArea(pIdxArea,pIdx);
                pointsByArea.insert(insArea);

                std::pair<unsigned int,double> insPt(pIdx,pIdxArea);
                areasByPoint.insert(insPt);
            }
        }
        else   {
            // since point areas are ordered by increasing area,
            // if the current point area is greater than the threshold,
            // all other points are as well, and we're done
            break;
        }
    }

    // save
    std::vector<Vec2> listRingPts;
    listRingPts.push_back(Vec2(polyLine->getX(sIdx),polyLine->getY(sIdx)));               // first

    for(pListIt = areasByPoint.begin();                                                   // in betweens
        pListIt != areasByPoint.end(); ++pListIt)   {
        listRingPts.push_back(Vec2(polyLine->getX(pListIt->first),
                                   polyLine->getY(pListIt->first)));
    }
    listRingPts.push_back(Vec2(polyLine->getX(eIdx),polyLine->getY(eIdx)));               // last
    listRingPts.push_back(listRingPts[0]);       // wrap == first

    polyLine->empty();
    for(unsigned int i=0; i < listRingPts.size(); i++)   {
        polyLine->addPoint(listRingPts[i].x,listRingPts[i].y,0);
    }
}

int main(int argc, const char *argv[])
{
    if(argc != 3) {
        std::cout << "Usage: #> ./ptk_simplify_wkt myinputfile.dat myoutputfile.ply\n";
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

    StartTiming("[Simplification]");

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

            if(!(inputGeometry->getGeometryType() == wkbMultiPolygon ||
                 inputGeometry->getGeometryType() == wkbPolygon))   {
                std::cout << "Error: WKT is not POLYGON/MULTIPOLYGON (ignoring)" << std::endl;
                std::cout << "-> " << wktLine << std::endl;
                continue;
            }

            OGRGeometry *simplerGeometry;

            if(SIMPLIFY_MODE == DOUGLAS_PEUCKER)   {
                simplerGeometry = inputGeometry->SimplifyPreserveTopology(DP_DIST);

                if(!(simplerGeometry == NULL))   {
                    char *outputWKT;
                    simplerGeometry->exportToWkt(&outputWKT);
                    outputWktFile << outputWKT << std::endl;
                    delete[] outputWKT;
                    delete simplerGeometry;
                }
                else   {
                    std::cout << "Simplified Geom was NULL\n" << std::endl;
                }
            }

            else if (SIMPLIFY_MODE == VISVALINGAM_WHYATT)   {
                // http://www2.dcs.hull.ac.uk/CISRG/publications/DPs/DP10/DP10.html
                // this algorithm works on polylines -- so we operate
                // on the constituent rings of the polygons
                if(inputGeometry->getGeometryType() == wkbMultiPolygon)   {
                    OGRMultiPolygon *multiPoly = (OGRMultiPolygon*)inputGeometry;
                    for(int i=0; i < multiPoly->getNumGeometries(); i++)   {
                        // outer ring
                        OGRPolygon *singlePoly = (OGRPolygon*)(multiPoly->getGeometryRef(i));
                        OGRLinearRing* outerRing = singlePoly->getExteriorRing();
                        simplifyWithVW(outerRing);

                        // inner rings
                        for(int j=0; j < singlePoly->getNumInteriorRings(); j++)   {
                            OGRLinearRing *innerRing = singlePoly->getInteriorRing(j);
                            simplifyWithVW(innerRing);
                        }
                    }
                }
                else if(inputGeometry->getGeometryType() == wkbPolygon)   {
                    // outer ring
                    OGRPolygon *singlePoly = (OGRPolygon*)inputGeometry;
                    OGRLinearRing* outerRing = singlePoly->getExteriorRing();
                    simplifyWithVW(outerRing);

                    // inner rings
                    for(int j=0; j < singlePoly->getNumInteriorRings(); j++)   {
                        OGRLinearRing *innerRing = singlePoly->getInteriorRing(j);
                        simplifyWithVW(innerRing);
                    }
                }

                char *outputWKT;
                inputGeometry->exportToWkt(&outputWKT);
                outputWktFile << outputWKT << std::endl;
                delete[] outputWKT;
            }

            delete inputGeometry;

            linesProcessed++;
            std::cout << "Lines Processed: "
                      << linesProcessed << "/" << numInputLines <<std::endl;
        }
        inputWktFile.close();
        outputWktFile.close();
    }
    EndTiming();

    return 0;
}
