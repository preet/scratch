/*
 Copyright (c) 2009-2012, 
 Gustavo Adolfo Ken Arroyo Ohori    g.a.k.arroyoohori@tudelft.nl
 Hugo Ledoux                        h.ledoux@tudelft.nl
 Martijn Meijers                    b.m.meijers@tudelft.nl
 All rights reserved.
 
 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met: 
 
 1. Redistributions of source code must retain the above copyright notice, this
 list of conditions and the following disclaimer. 
 2. Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution. 
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
  Minor additions to incorporate import of multiple polygons in a single
  file, triangulation of said polygons, and export to a *.ply file by

  Preet Desai       prismatic.project@gmail.com

  This file, including all changes from the original (available at
  https://github.com/tudelft-gist/prepair) is made available under the
  same terms as the original license, shown above
*/

// STL
#include <iostream>
#include <iomanip>
#include <cstring>
#include <fstream>
#include <stack>
#include <set>
#include <unordered_set>
#include <sys/time.h>

// OGR
#include <ogrsf_frmts.h>

// CGAL
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Constrained_triangulation_plus_2.h>
#include <CGAL/Triangle_2.h>

// openctm
#include "openctm/openctm.h"

// geom defs
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

#define USE_ECEF false

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Triangulation_vertex_base_2<K> VB;
typedef CGAL::Constrained_triangulation_face_base_2<K> FB;
typedef CGAL::Triangulation_face_base_with_info_2<void *, K, FB> FBWI;
typedef CGAL::Triangulation_data_structure_2<VB, FBWI> TDS;
typedef CGAL::Exact_predicates_tag PT;
typedef CGAL::Exact_intersections_tag IT;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, TDS, PT> CDT;

typedef CGAL::Constrained_triangulation_plus_2<CDT> Triangulation;
typedef Triangulation::Point Point;


OGRMultiPolygon* repair(OGRGeometry* geometry, Triangulation &triangulation);
void tag(Triangulation &triangulation, void *interiorHandle, void *exteriorHandle);
std::list<Triangulation::Vertex_handle> *getBoundary(Triangulation::Face_handle face, int edge);

// timing var
timeval t1,t2;
std::string timingDesc;

struct Tri
{
    Vec2 A;
    Vec2 B;
    Vec2 C;

    double len_a;
    double len_b;
    double len_c;
};

struct TriangleMesh
{
    std::vector<Vec3> listVertices;
    std::vector<unsigned int> listIdxs;
};

bool compareVec3(Vec3 const &first, Vec3 const &second)
{
    Vec3 refPt;
    if(USE_ECEF == true)
    {}
    else
    {   refPt.x = -180; refPt.y = 0; refPt.z = 0;   }

    if(first.Distance2To(refPt) < second.Distance2To(refPt))
    {   return true;   }

    return false;
}

std::string Vec3ToString (Vec3 const &myVec)
{
    std::stringstream ss;
    ss.precision(12);
    ss << myVec.x << "," << myVec.y << "," << myVec.z << std::endl;
    return ss.str();
}

Vec3 StringToVec3(const std::string &myText)
{
    unsigned int xEnd = myText.find(",");
    unsigned int yEnd = myText.find_last_of(",");
    std::string xStr = myText.substr(0,xEnd);
    std::string yStr = myText.substr(xEnd+1,yEnd-xEnd-1);
    std::string zStr = myText.substr(yEnd+1);

    Vec3 myVec;
    std::stringstream ssX(xStr);
    ssX >> myVec.x;

    std::stringstream ssY(yStr);
    ssY >> myVec.y;

    std::stringstream ssZ(zStr);
    ssZ >> myVec.z;

    return myVec;
}

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

inline int isLeft( Vec2 P0, Vec2 P1, Vec2 P2 )
{
    return ( (P1.x - P0.x) * (P2.y - P0.y)
            - (P2.x - P0.x) * (P1.y - P0.y) );
}

int calcCrossingNumber(Vec2 P,std::vector<Vec2> V)
{
    // Copyright 2001, softSurfer (www.softsurfer.com)
    // This code may be freely used and modified for any purpose
    // providing that this copyright notice is included with it.
    // SoftSurfer makes no warranty for this code, and cannot be held
    // liable for any real or imagined damage resulting from its use.
    // Users of this code must verify correctness for their application.
    int    cn = 0;    // the crossing number counter

    // loop through all edges of the polygon
    for (int i=0; i<V.size()-1; i++) {    // edge from V[i] to V[i+1]
       if (((V[i].y <= P.y) && (V[i+1].y > P.y))    // an upward crossing
        || ((V[i].y > P.y) && (V[i+1].y <= P.y))) { // a downward crossing
            // compute the actual edge-ray intersect x-coordinate
            float vt = (float)(P.y - V[i].y) / (V[i+1].y - V[i].y);
            if (P.x < V[i].x + vt * (V[i+1].x - V[i].x)) // P.x < intersect
                ++cn;   // a valid crossing of y=P.y right of P.x
        }
    }
    return (cn&1);    // 0 if even (out), and 1 if odd (in)
}


int calcWindingNumber(Vec2 checkPt,std::vector<Vec2> polyContour)
{
    // easy to follow but naive/slow implementation
    // expect polyContour[first] == polyContour[last]
    // winding number = 0 if checkPt is not inside polyContour
    // winding number = n > 0 if polyContour winds around checkPt 'n' times CCW
    // winding number = n < 0 if polyContour winds around checkPt 'n' times CW
    double windingNum = 0;

    // see if the polygon crosses the antemeridian and
    // adjust for a latitude/longitude discontinuity if it does

    // get longitude extents
    double minLon =  200;
    double maxLon = -200;
    for(int i=0; i < polyContour.size(); i++)
    {
        minLon = std::min(minLon,polyContour[i].x);
        maxLon = std::max(maxLon,polyContour[i].x);
    }

    if(maxLon-minLon > 270)
    {   // assume poly spans antemeridian
        for(int i=0; i < polyContour.size(); i++)
        {
            if(polyContour[i].x < 0)
            {   polyContour[i].x += 360.0;   }
        }
        if(checkPt.x < 0)
        {   checkPt.x += 360.0;   }
    }

    // center checkPt at (0,0) for ease of use
    // and shift the polygon accordingly
    for(int i=0; i < polyContour.size(); i++)
    {   polyContour[i] = polyContour[i]-checkPt;   }

    // now we walk along the polygon's edges; every
    // time a polygon edge crosses the positive x
    // from below, increment the winding number;
    // every time the positive x axis is crossed
    // from above, decrement the winding number
    for(int i=1; i < polyContour.size(); i++)
    {
        double xPrev = polyContour[i-1].x;
        double yPrev = polyContour[i-1].y;
        double xNext = polyContour[i].x;
        double yNext = polyContour[i].y;

        if(yPrev*yNext < 0) // the edge crosses the x axis
        {
            // r is the x coord of intersection between edge and x axis
            double r = xPrev + (yPrev*(xNext-xPrev))/(yPrev-yNext);

            if(r > 0)   // edge crosses the positive x axis!
            {
                if(yPrev < 0)   // edge crosses from below
                {   windingNum += 1;   }
                else            // edge crosses from above
                {   windingNum -= 1;   }
            }
        }
        else if(yPrev == 0 && xPrev > 0)
        {   // edge crosses from +ve x axis to above or below
            if(yNext > 0)
            {   windingNum += 0.5;   }
            else
            {   windingNum -= 0.5;   }
        }
        else if(yNext == 0 && xNext > 0)
        {   // edge crosses from above or below to +ve x axis
            if(yPrev < 0)
            {   windingNum += 0.5;   }
            else
            {   windingNum -= 0.5;   }
        }
    }

    // I don't think that you can ever end up
    // with half winding numbers, but just incase
    if(windingNum == -0.5)
    {   return -1;   }
    else if(windingNum == 0.5)
    {   return 1;   }
    else
    {   return int(windingNum);   }
}


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


int main(int argc, const char *argv[])
{
    if(argc != 3) {
        std::cout << "Usage: #> ./ptk_wkt_to_ctm inputfile myoutputfile\n";
        std::cout << "* Expect each line of the input file to contain a single WKT POLYGON() def\n";
        std::cout << "* The output file is a mesh in OpenCTM format\n";
        return 0;
    }

    StartTiming("[Triangulate Data]");

    std::ifstream wktFile;
    std::string cFileName(argv[1]);
    wktFile.open(argv[1]);

    // get number of input lines
    unsigned int numInputLines=0;
    if(wktFile.is_open())   {
        while(!wktFile.eof())   {
            std::string wktLine;
            std::getline(wktFile,wktLine);
            numInputLines++;
        }
    }
    wktFile.close();
    wktFile.open(argv[1]);

    if(wktFile.is_open())
    {
        TriangleMesh triMesh;
        int linesProcessed = 0;

        while(!wktFile.eof())
        {
            std::string wktLine;
            std::getline(wktFile,wktLine);

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

            // process / fix geometry
            Triangulation myTriangulation;
            OGRMultiPolygon* outputPolygons = repair(inputGeometry,myTriangulation);

            // get list of cdt triangles
            std::vector<Tri> listCDTTriangles;
            std::vector<bool> listTrisToKeep;
            Triangulation::Finite_faces_iterator fIt;
            for(fIt = myTriangulation.finite_faces_begin();
                fIt != myTriangulation.finite_faces_end(); ++fIt)
            {
                Triangulation::Triangle cdtTri = myTriangulation.triangle(fIt);

                Tri myTri;
                myTri.A.x = cdtTri[0].x();
                myTri.A.y = cdtTri[0].y();

                myTri.B.x = cdtTri[1].x();
                myTri.B.y = cdtTri[1].y();

                myTri.C.x = cdtTri[2].x();
                myTri.C.y = cdtTri[2].y();

                myTri.len_a = sqrt( pow(myTri.C.x-myTri.B.x,2) + pow(myTri.C.y-myTri.B.y,2));
                myTri.len_b = sqrt( pow(myTri.C.x-myTri.A.x,2) + pow(myTri.C.y-myTri.A.y,2));
                myTri.len_c = sqrt( pow(myTri.A.x-myTri.B.x,2) + pow(myTri.A.y-myTri.B.y,2));

                listCDTTriangles.push_back(myTri);
                listTrisToKeep.push_back(false);
            }

            if(!(outputPolygons == NULL))
            {
                // discard triangles using the repaired multipolygon as a ref
                for(int i=0; i < outputPolygons->getNumGeometries(); i++)
                {
                    OGRGeometry *polyGeometry = outputPolygons->getGeometryRef(i);
                    OGRPolygon *singlePoly = (OGRPolygon*)polyGeometry;

                    // filter triangles outside of outer ring
                    OGRLinearRing* outerRing = singlePoly->getExteriorRing();
                    std::vector<Vec2> listOuterRingPts(outerRing->getNumPoints());
                    for(int j=0; j < listOuterRingPts.size(); j++)   {
                        OGRPoint *myPt = new OGRPoint; outerRing->getPoint(j,myPt);
                        listOuterRingPts[j] = Vec2(myPt->getX(),myPt->getY());
                        delete myPt;
                    }

                    std::vector<Tri>::iterator triIt;
                    for(triIt = listCDTTriangles.begin();
                        triIt != listCDTTriangles.end(); ++triIt)
                    {                      
                        // create triangle incenter
                        Vec2 inCenter;
                        inCenter.x = ((triIt->len_a*triIt->A.x + triIt->len_b*triIt->B.x + triIt->len_c*triIt->C.x) /
                                      (triIt->len_a+triIt->len_b+triIt->len_c));

                        inCenter.y = ((triIt->len_a*triIt->A.y + triIt->len_b*triIt->B.y + triIt->len_c*triIt->C.y) /
                                      (triIt->len_a+triIt->len_b+triIt->len_c));

                        int crossingNum = calcCrossingNumber(inCenter,listOuterRingPts);

                        // we want to keep all triangles within outer rings
                        if(crossingNum != 0)    // if cross num != 0, point is in poly
                        {   listTrisToKeep[triIt-listCDTTriangles.begin()] = true;   }
                    }

                    // filter triangles inside inner rings
                    for(int j=0; j < singlePoly->getNumInteriorRings(); j++)
                    {
                        OGRLinearRing* innerRing = singlePoly->getInteriorRing(j);
                        std::vector<Vec2> listInnerRingPts(innerRing->getNumPoints());
                        for(int k=0; k < listInnerRingPts.size(); k++)   {
                            OGRPoint *myPt = new OGRPoint; innerRing->getPoint(k,myPt);
                            listInnerRingPts[k] = Vec2(myPt->getX(),myPt->getY());
                            delete myPt;
                        }

                        for(triIt = listCDTTriangles.begin();
                            triIt != listCDTTriangles.end(); ++triIt)
                        {
                            if(listTrisToKeep[triIt-listCDTTriangles.begin()] == true)
                            {
                                // create triangle incenter
                                Vec2 inCenter;
                                inCenter.x = ((triIt->len_a*triIt->A.x + triIt->len_b*triIt->B.x + triIt->len_c*triIt->C.x) /
                                              (triIt->len_a+triIt->len_b+triIt->len_c));

                                inCenter.y = ((triIt->len_a*triIt->A.y + triIt->len_b*triIt->B.y + triIt->len_c*triIt->C.y) /
                                              (triIt->len_a+triIt->len_b+triIt->len_c));

                                int crossingNum = calcCrossingNumber(inCenter,listInnerRingPts);

                                // we want to remove all triangles within inner rings
                                if(crossingNum != 0)    // if cross num != 0, point is in poly
                                {   listTrisToKeep[triIt-listCDTTriangles.begin()] = false;   }
                            }
                        }
                    }

                    // save triangles
                    for(triIt = listCDTTriangles.begin();
                        triIt != listCDTTriangles.end(); ++triIt)
                    {
                        if(listTrisToKeep[triIt-listCDTTriangles.begin()])
                        {
                            Vec3 pt0(triIt->A.x,triIt->A.y,0);
                            Vec3 pt1(triIt->B.x,triIt->B.y,0);
                            Vec3 pt2(triIt->C.x,triIt->C.y,0);

                            triMesh.listVertices.push_back(pt0);
                            triMesh.listIdxs.push_back(triMesh.listVertices.size()-1);

                            triMesh.listVertices.push_back(pt1);
                            triMesh.listIdxs.push_back(triMesh.listVertices.size()-1);

                            triMesh.listVertices.push_back(pt2);
                            triMesh.listIdxs.push_back(triMesh.listVertices.size()-1);
                        }
                    }
                }

                delete inputGeometry;
                delete outputPolygons;

                linesProcessed++;
                std::cout << "ptk_wkt_to_ctm: " << cFileName << ": Lines Processed: "
                          << linesProcessed << "/" << numInputLines <<std::endl;
            }
        }
        wktFile.close();
        EndTiming();

        StartTiming("[Clean Mesh]");

        // clean mesh to remove duplicate verts
        std::vector<std::string>::iterator vecStrIt;
        std::vector<std::string> listUniqueVertexStrings(triMesh.listVertices.size());
        for(int i=0; i < triMesh.listVertices.size(); i++)   {
            listUniqueVertexStrings[i] = Vec3ToString(triMesh.listVertices[i]);
        }
        std::sort(listUniqueVertexStrings.begin(),listUniqueVertexStrings.end());
        vecStrIt = std::unique(listUniqueVertexStrings.begin(),listUniqueVertexStrings.end());
        listUniqueVertexStrings.resize(vecStrIt-listUniqueVertexStrings.begin());

        std::vector<unsigned int> idxMap;
        for(int i=0; i < triMesh.listVertices.size(); i++)   {
            vecStrIt = std::lower_bound(listUniqueVertexStrings.begin(),
                                        listUniqueVertexStrings.end(),
                                        Vec3ToString(triMesh.listVertices[i]));
            idxMap.push_back(vecStrIt-listUniqueVertexStrings.begin());
        }

        triMesh.listVertices.clear();
        for(int i=0; i < listUniqueVertexStrings.size(); i++)   {
            triMesh.listVertices.push_back(StringToVec3(listUniqueVertexStrings[i]));
        }

        for(int i=0; i < triMesh.listIdxs.size(); i++)   {
            unsigned int oldIdx = triMesh.listIdxs[i];
            triMesh.listIdxs[i] = idxMap[oldIdx];
        }
        EndTiming();

//        StartTiming("[Convert LLA to ECEF]");
//        for(int i=0; i < triMesh.listVertices.size(); i++)   {
//            Vec3 ecefVx = convLLAToECEF(PointLLA(triMesh.listVertices[i].y,
//                                                 triMesh.listVertices[i].x));
//            triMesh.listVertices[i] = ecefVx;
//        }
//        EndTiming();

        StartTiming("[Write Mesh as CTM file]");
        // write CTM file
        CTMcontext context;
        CTMuint vertCount, triCount, *indices;
        CTMfloat *vertices;

        // create context
        context = ctmNewContext(CTM_EXPORT);
        ctmCompressionMethod(context,CTM_METHOD_MG1);
        ctmCompressionLevel(context,5);

        // create mesh in memory
        vertCount = triMesh.listVertices.size();
        triCount = triMesh.listIdxs.size()/3;
        vertices = (CTMfloat *) malloc(3 * sizeof(CTMfloat) * vertCount);
        indices = (CTMuint *) malloc(3 * sizeof(CTMuint) * triCount);

        // build mesh
        unsigned int vIdx=0;
        for(int i=0; i < vertCount; i++)   {
            vertices[vIdx] = triMesh.listVertices[i].x; vIdx++;
            vertices[vIdx] = triMesh.listVertices[i].y; vIdx++;
            vertices[vIdx] = triMesh.listVertices[i].z; vIdx++;
        }

        for(int i=0; i < triCount*3; i++)   {
            indices[i] = triMesh.listIdxs[i];
        }

        // define mesh
        ctmDefineMesh(context,vertices,vertCount,indices,triCount,NULL);

        // save
        ctmSave(context,argv[2]);

        // free context/mesh
        ctmFreeContext(context);
        free(indices);
        free(vertices);

        EndTiming();
    }

    return 0;
}

void tag(Triangulation &triangulation, void *interiorHandle, void *exteriorHandle) {
	
    // Clean tags
    for (Triangulation::Face_handle currentFace = triangulation.all_faces_begin(); currentFace != triangulation.all_faces_end(); ++currentFace)
        currentFace->info() = NULL;
    
    // Initialise tagging
    std::stack<Triangulation::Face_handle> interiorStack, exteriorStack;
    exteriorStack.push(triangulation.infinite_face());
    std::stack<Triangulation::Face_handle> *currentStack = &exteriorStack;
    std::stack<Triangulation::Face_handle> *dualStack = &interiorStack;
    void *currentHandle = exteriorHandle;
    void *dualHandle = interiorHandle;
    
    // Until we finish
    while (!interiorStack.empty() || !exteriorStack.empty()) {
        
        // Give preference to whatever we're already doing
        while (!currentStack->empty()) {
            Triangulation::Face_handle currentFace = currentStack->top();
			currentStack->pop();
            if (currentFace->info() != NULL) continue;
			currentFace->info() = currentHandle;
            for (int currentEdge = 0; currentEdge < 3; ++currentEdge) {
                if (currentFace->neighbor(currentEdge)->info() == NULL)
                    if (currentFace->is_constrained(currentEdge)) dualStack->push(currentFace->neighbor(currentEdge));
                else currentStack->push(currentFace->neighbor(currentEdge));
            }
        }
			
        // Flip
        if (currentHandle == exteriorHandle) {
            currentHandle = interiorHandle;
            dualHandle = exteriorHandle;
            currentStack = &interiorStack;
            dualStack = &exteriorStack;
        } else {
            currentHandle = exteriorHandle;
            dualHandle = interiorHandle;
            currentStack = &exteriorStack;
            dualStack = &interiorStack;
        }
	}
}




std::list<Triangulation::Vertex_handle> *getBoundary(Triangulation::Face_handle face, int edge) {
    
    std::list<Triangulation::Vertex_handle> *vertices = new std::list<Triangulation::Vertex_handle>();
    
    // Check clockwise edge
    if (!face->is_constrained(face->cw(edge)) && face->neighbor(face->cw(edge))->info() != NULL) {
		face->neighbor(face->cw(edge))->info() = NULL;
		std::list<Triangulation::Vertex_handle> *v1 = getBoundary(face->neighbor(face->cw(edge)), face->neighbor(face->cw(edge))->index(face));
		vertices->splice(vertices->end(), *v1);
		delete v1;
	}
	
	// Add central vertex
	vertices->push_back(face->vertex(edge));
	
	// Check counterclockwise edge
    if (!face->is_constrained(face->ccw(edge)) && face->neighbor(face->ccw(edge))->info() != NULL) {
		face->neighbor(face->ccw(edge))->info() = NULL;
		std::list<Triangulation::Vertex_handle> *v2 = getBoundary(face->neighbor(face->ccw(edge)), face->neighbor(face->ccw(edge))->index(face));
		vertices->splice(vertices->end(), *v2);
		delete v2;
	}
	
    return vertices;
}




OGRMultiPolygon* repair(OGRGeometry* geometry, Triangulation &triangulation) {
  
  // Triangulation
//  Triangulation triangulation;
  switch (geometry->getGeometryType()) {
      
    case wkbPolygon: {
      
      OGRPolygon *polygon = (OGRPolygon *)geometry;
      for (int currentPoint = 0; currentPoint < polygon->getExteriorRing()->getNumPoints(); ++currentPoint) {
        triangulation.insert_constraint(Point(polygon->getExteriorRing()->getX(currentPoint), 
                                              polygon->getExteriorRing()->getY(currentPoint)),
                                        Point(polygon->getExteriorRing()->getX((currentPoint+1)%polygon->getExteriorRing()->getNumPoints()), 
                                              polygon->getExteriorRing()->getY((currentPoint+1)%polygon->getExteriorRing()->getNumPoints())));
      } for (int currentRing = 0; currentRing < polygon->getNumInteriorRings(); ++currentRing) {
        for (int currentPoint = 0; currentPoint < polygon->getInteriorRing(currentRing)->getNumPoints(); ++currentPoint)
          triangulation.insert_constraint(Point(polygon->getInteriorRing(currentRing)->getX(currentPoint), 
                                                polygon->getInteriorRing(currentRing)->getY(currentPoint)),
                                          Point(polygon->getInteriorRing(currentRing)->getX((currentPoint+1)%polygon->getInteriorRing(currentRing)->getNumPoints()), 
                                                polygon->getInteriorRing(currentRing)->getY((currentPoint+1)%polygon->getInteriorRing(currentRing)->getNumPoints())));
      } break;
      
    } default:
      std::cout << "Error: Cannot understand input. Only polygons are supported." << std::endl;
      break;
  } 
  
//  std::cout << "Triangulation: " << triangulation.number_of_faces() << " faces, " << triangulation.number_of_vertices() << " vertices." << std::endl;
  if (triangulation.number_of_faces() < 1) {
    return NULL;
  }
  
  // Tag
  void *interior = malloc(sizeof(void *));
  void *exterior = malloc(sizeof(void *));
  tag(triangulation, interior, exterior);
  
  // Reconstruct
  //    OGRMultiPolygon outputPolygons;
  OGRMultiPolygon* outputPolygons = new OGRMultiPolygon();
  for (Triangulation::Finite_faces_iterator seedingFace = triangulation.finite_faces_begin(); seedingFace != triangulation.finite_faces_end(); ++seedingFace) {
    
    if (seedingFace->info() != interior) continue;
    
    // Get boundary
    std::list<Triangulation::Vertex_handle> *vertices = new std::list<Triangulation::Vertex_handle>();
    seedingFace->info() = NULL;
    if (seedingFace->neighbor(2)->info() == interior) {
      seedingFace->neighbor(2)->info() = NULL;
      std::list<Triangulation::Vertex_handle> *l2 = getBoundary(seedingFace->neighbor(2), seedingFace->neighbor(2)->index(seedingFace));
      vertices->splice(vertices->end(), *l2);
      delete l2;
    } vertices->push_back(seedingFace->vertex(0));
    if (seedingFace->neighbor(1)->info() == interior) {
      seedingFace->neighbor(1)->info() = NULL;
      std::list<Triangulation::Vertex_handle> *l1 = getBoundary(seedingFace->neighbor(1), seedingFace->neighbor(1)->index(seedingFace));
      vertices->splice(vertices->end(), *l1);
      delete l1;
    } vertices->push_back(seedingFace->vertex(2));
    if (seedingFace->neighbor(0)->info() == interior) {
      seedingFace->neighbor(0)->info() = NULL;
      std::list<Triangulation::Vertex_handle> *l0 = getBoundary(seedingFace->neighbor(0), seedingFace->neighbor(0)->index(seedingFace));
      vertices->splice(vertices->end(), *l0);
      delete l0;
    } vertices->push_back(seedingFace->vertex(1));
    
    // Find cutting vertices
    std::set<Triangulation::Vertex_handle> visitedVertices;
    std::set<Triangulation::Vertex_handle> repeatedVertices;
    for (std::list<Triangulation::Vertex_handle>::iterator currentVertex = vertices->begin(); currentVertex != vertices->end(); ++currentVertex) {
      if (!visitedVertices.insert(*currentVertex).second) repeatedVertices.insert(*currentVertex);
    } visitedVertices.clear();
    
    // Cut and join rings in the correct order
    std::list<std::list<Triangulation::Vertex_handle> *> rings;
    std::stack<std::list<Triangulation::Vertex_handle> *> chainsStack;
    std::map<Triangulation::Vertex_handle, std::list<Triangulation::Vertex_handle> *> vertexChainMap;
    std::list<Triangulation::Vertex_handle> *newChain = new std::list<Triangulation::Vertex_handle>();
    for (std::list<Triangulation::Vertex_handle>::iterator currentVertex = vertices->begin(); currentVertex != vertices->end(); ++currentVertex) {
      
      // New chain
      if (repeatedVertices.count(*currentVertex) > 0) {
        // Closed by itself
        if (newChain->front() == *currentVertex) {
          // Degenerate (insufficient vertices to be valid)
          if (newChain->size() < 3) delete newChain;
          else {
            std::list<Triangulation::Vertex_handle>::iterator secondElement = newChain->begin();
            ++secondElement;
            // Degenerate (zero area)
            if (newChain->back() == *secondElement) delete newChain;
            // Valid
            else rings.push_back(newChain);
          }
        }
        // Open by itself
        else {
          // Closed with others in stack
          if (vertexChainMap.count(*currentVertex)) {
            
            while (chainsStack.top() != vertexChainMap[*currentVertex]) {
              newChain->splice(newChain->begin(), *chainsStack.top());
              chainsStack.pop();
            } newChain->splice(newChain->begin(), *chainsStack.top());
            chainsStack.pop();
            vertexChainMap.erase(*currentVertex);
            // Degenerate (insufficient vertices to be valid)
            if (newChain->size() < 3) delete newChain;
            else {
              std::list<Triangulation::Vertex_handle>::iterator secondElement = newChain->begin();
              ++secondElement;
              // Degenerate (zero area)
              if (newChain->back() == *secondElement) delete newChain;
              // Valid
              else rings.push_back(newChain);
            }
          }
          // Open
          else {
            // Not first chain
            if (repeatedVertices.count(newChain->front()) > 0) vertexChainMap[newChain->front()] = newChain;
            chainsStack.push(newChain);
          }
        } newChain = new std::list<Triangulation::Vertex_handle>();
      } newChain->push_back(*currentVertex);
    }
    // Final ring
    while (chainsStack.size() > 0) {
      newChain->splice(newChain->begin(), *chainsStack.top());
      chainsStack.pop();
    }
    // Degenerate (insufficient vertices to be valid)
    if (newChain->size() < 3) delete newChain;
    else {
      std::list<Triangulation::Vertex_handle>::iterator secondElement = newChain->begin();
      ++secondElement;
      // Degenerate (zero area)
      if (newChain->back() == *secondElement) delete newChain;
      // Valid
      else rings.push_back(newChain);
    }
    // Make rings
    std::list<OGRLinearRing *> ringsForPolygon;
    for (std::list<std::list<Triangulation::Vertex_handle> *>::iterator currentRing = rings.begin(); currentRing != rings.end(); ++currentRing) {
      OGRLinearRing *newRing = new OGRLinearRing();
      for (std::list<Triangulation::Vertex_handle>::reverse_iterator currentVertex = (*currentRing)->rbegin(); currentVertex != (*currentRing)->rend(); ++currentVertex) {
        newRing->addPoint((*currentVertex)->point().x(), (*currentVertex)->point().y());
      } newRing->addPoint((*currentRing)->back()->point().x(), (*currentRing)->back()->point().y());
      ringsForPolygon.push_back(newRing);
    } OGRPolygon *newPolygon = new OGRPolygon();
    for (std::list<OGRLinearRing *>::iterator currentRing = ringsForPolygon.begin(); currentRing != ringsForPolygon.end(); ++currentRing) {
      if (!(*currentRing)->isClockwise()) {
        newPolygon->addRingDirectly(*currentRing);
        break;
      }
    } for (std::list<OGRLinearRing *>::iterator currentRing = ringsForPolygon.begin(); currentRing != ringsForPolygon.end(); ++currentRing)
      if ((*currentRing)->isClockwise()) newPolygon->addRingDirectly(*currentRing);
    outputPolygons->addGeometryDirectly(newPolygon);
  }
  return outputPolygons;
}

