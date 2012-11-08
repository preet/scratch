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
  file, and export to a file by

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
#include <sys/time.h>

// OGR
#include <ogrsf_frmts.h>

// CGAL
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Constrained_triangulation_plus_2.h>
#include <CGAL/Triangle_2.h>

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


int main(int argc, const char *argv[])
{
    if(argc != 3) {
        std::cout << "Usage: #> ./ptk_repair_wkt myinputfile myoutputfile\n";
        std::cout << "* Expect each line of the input file to contain a single WKT POLYGON() def\n";
        std::cout << "* The output file is in the same format as the input file\n";
        return 0;
    }

    StartTiming("[Repair Polygons]");

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
                std::cout << "-> WKT: " << wktLine << std::endl;
                delete[] inputWKTRef;
                continue;
            }
            delete[] inputWKTRef;

            if(inputGeometry->getGeometryType() == wkbPolygon)   {
                Triangulation cTri;
                OGRMultiPolygon* outputPolygons = repair(inputGeometry,cTri);

                if(!(outputPolygons == NULL))   {
                    // output everything as POLYGONS()
                    for(int i=0; i < outputPolygons->getNumGeometries(); i++)   {
                        char *outputWKT;
                        outputPolygons->getGeometryRef(i)->exportToWkt(&outputWKT);
                        outputWktFile << outputWKT << std::endl;
                        delete[] outputWKT;
                    }
                }
                else   {
                    std::cout << "Error: Could not repair geometry,: "
                                 "input points are collinear (no area)\n ";
                    std::cout << "-> WKT: " << wktLine << "\n";
                }
            }
            else   {
                std::cout << "Error: Could not repair geometry, "
                             "WKT type is not a POLYGON()\n";
                std::cout << "-> WKT: " << wktLine << "\n";
            }

            linesProcessed++;
            std::cout << "Lines Processed: "
                      << linesProcessed << "/"
                      << numInputLines << std::endl;

            // clean up
            delete inputGeometry;
        }
        inputWktFile.close();
        outputWktFile.close();
    }
    EndTiming();

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

