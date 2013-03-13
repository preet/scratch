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
  Additions to incorporate import of multiple polygons in a single
  file, triangulation of said polygons, and export to a mesh file by

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

// OGR
#include <ogrsf_frmts.h>

// CGAL
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Constrained_triangulation_plus_2.h>
#include <CGAL/Triangle_2.h>

// shptk
#include "shptk.hpp"

// OpenSceneGraph (debug only)
#include <osg/Geometry>
#include <osg/PolygonMode>
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>
#include <osg/Geode>
#include <osgViewer/Viewer>

typedef CGAL::Exact_predicates_inexact_constructions_kernel             K;
typedef CGAL::Triangulation_vertex_base_2<K>                            VB;
typedef CGAL::Constrained_triangulation_face_base_2<K>                  FB;
typedef CGAL::Triangulation_face_base_with_info_2<void *, K, FB>        FBWI;
typedef CGAL::Triangulation_data_structure_2<VB, FBWI>                  TDS;
typedef CGAL::Exact_predicates_tag                                      PT;
typedef CGAL::Exact_intersections_tag                                   IT;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, TDS, PT>          CDT;

typedef CGAL::Constrained_triangulation_plus_2<CDT>                     Triangulation;
typedef Triangulation::Vertex_handle                                    Vertex_handle;
typedef Triangulation::Point                                            Point;

OGRMultiPolygon* repair(OGRGeometry* geometry, Triangulation &triangulation, void * &interior, void * &exterior);
void tag(Triangulation &triangulation, void *interiorHandle, void *exteriorHandle);
std::list<Triangulation::Vertex_handle> *getBoundary(Triangulation::Face_handle face, int edge);


bool vxExistsInList(Vec2 const &vx,
                    std::vector<Vec2> const &listVx,
                    int &idx)
{
    bool exists = false;
    for(size_t i=0; i < listVx.size(); i++)   {
        if(vx == listVx[i])   {
            exists = true;
            idx = i;
            break;
        }
    }
    return exists;
}


int main(int argc, const char *argv[])
{
    if(argc != 3) {
        std::cout << "Usage: #> ./shptk_meshgen inputfile myoutputfile\n";
        std::cout << "* Expect inputfile to be shapefile with POLYGON data only\n";
        std::cout << "* The output file is a mesh in OpenCTM format\n";
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

    std::string ipFilePath(argv[1]);
    std::string opFilePath(argv[2]);

    // open input file and feature layer
    OGRDataSource * ipShpFile = OGRSFDriverRegistrar::Open(ipFilePath.c_str(),FALSE);
    if(ipShpFile == NULL)   {
        std::cout << "ERROR: Could not open input file: " << ipFilePath << std::endl;
        return -1;
    }

    OGRLayer * ipLayer = ipShpFile->GetLayer(0);
    size_t numFeatures = ipLayer->GetFeatureCount();
    ipLayer->ResetReading();

    // iterate through input features and build mesh
    std::vector<Vec2> listMeshVx;
    std::vector<size_t> listMeshIx;
    size_t ixFeature = 0;
    OGRFeature * ipFeature;
    while( (ipFeature = ipLayer->GetNextFeature()) != NULL )
    {
        std::cout << "INFO: Feature " << ixFeature
                  << "/" << numFeatures << std::endl;

        OGRGeometry * ipGeometry;
        ipGeometry = ipFeature->GetGeometryRef();

        if(ipGeometry == NULL)   {
            std::cout << "WARN: Ignoring NULL feature: "
                      << ixFeature << std::endl;
            continue;
        }

        if(ipGeometry->getGeometryType() != wkbPolygon)   {
            std::cout << "WARN: Feature type is not POLYGON "
                      << "(its " << ipGeometry->getGeometryName()
                      << ") " << std::endl;
            continue;
        }

        // simplify geometry
        // note: we temp. convert to ecef for simplifcation
        OGRPolygon * ipPoly = (OGRPolygon *)ipGeometry;
        OGRPolygon * sPolygon = new OGRPolygon;

        // outer ring
        std::vector<Vec3> listOuterVx, listOuterVxSimp;
        OGRLinearRing * ipOuterRing = ipPoly->getExteriorRing();
        for(int i=0; i < ipOuterRing->getNumPoints(); i++)   {
            PointLLA lla(ipOuterRing->getY(i),ipOuterRing->getX(i));
            listOuterVx.push_back(ConvLLAToECEF(lla));
        }
        CalcPolylineSimplifyVW(listOuterVx,listOuterVxSimp,VW_AREA,100000.0);
        OGRLinearRing * sOuterRing = new OGRLinearRing;
        for(size_t i=0; i < listOuterVxSimp.size(); i++)   {
            PointLLA lla = ConvECEFToLLA(listOuterVxSimp[i]);
            sOuterRing->addPoint(lla.lon,lla.lat,0);
        }
        sPolygon->addRingDirectly(sOuterRing);

        // inner rings
        for(int i=0; i < ipPoly->getNumInteriorRings(); i++)   {
            std::vector<Vec3> listInnerVx,listInnerVxSimp;
            OGRLinearRing * ipInnerRing = ipPoly->getInteriorRing(i);
            for(int j=0; j < ipInnerRing->getNumPoints(); j++)   {
                PointLLA lla(ipInnerRing->getY(j),ipInnerRing->getX(j));
                listInnerVx.push_back(ConvLLAToECEF(lla));
            }
            CalcPolylineSimplifyVW(listInnerVx,listInnerVxSimp,VW_AREA,100000.0);
            OGRLinearRing * sInnerRing = new OGRLinearRing;
            for(size_t j=0; j < listInnerVxSimp.size(); j++)   {
                PointLLA lla = ConvECEFToLLA(listInnerVxSimp[j]);
                sInnerRing->addPoint(lla.lon,lla.lat,0);
            }
            sPolygon->addRingDirectly(sInnerRing);
        }
        sPolygon->flattenTo2D();    // required to convert to wkbPolygon

        //
        int ix;
        Vec2 vx;
        std::vector<Vec2> listVx;
        std::vector<size_t> listIx;

        // simultaneously repair ipGeometry and retrieve
        // the triangulation used in the repair process
        void * interior;
        void * exterior;
        Triangulation opTriangulation;
        OGRMultiPolygon * opPolygons = repair(sPolygon,
                                              opTriangulation,
                                              interior,
                                              exterior);

        if(opPolygons == NULL)   {
            std::cout << "WARN: Triangulation failed for "
                      << "feature " << ixFeature << std::endl;
            continue;
        }

        Triangulation::Finite_faces_iterator fIt;
        for(fIt  = opTriangulation.finite_faces_begin();
            fIt != opTriangulation.finite_faces_end(); ++fIt)
        {
//            std::cout << "sf->info(): " << fIt->info() << ", "
//                      << "interior: " << (interior) << ", "
//                      << "exterior: " << (exterior) << std::endl;

            if(fIt->info() == NULL)   {
                // get triangle
                Triangulation::Triangle cdtTri =
                        opTriangulation.triangle(fIt);

                // point A
                vx.x = cdtTri[0].x();
                vx.y = cdtTri[0].y();
                if(vxExistsInList(vx,listVx,ix))
                {   listIx.push_back(ix);   }
                else   {
                    listVx.push_back(vx);
                    listIx.push_back(listVx.size()-1);
                }

                // point B
                vx.x = cdtTri[1].x();
                vx.y = cdtTri[1].y();
                if(vxExistsInList(vx,listVx,ix))
                {   listIx.push_back(ix);   }
                else   {
                    listVx.push_back(vx);
                    listIx.push_back(listVx.size()-1);
                }

                // point C
                vx.x = cdtTri[2].x();
                vx.y = cdtTri[2].y();
                if(vxExistsInList(vx,listVx,ix))
                {   listIx.push_back(ix);   }
                else   {
                    listVx.push_back(vx);
                    listIx.push_back(listVx.size()-1);
                }
            }
        }
        // adj indices
        for(size_t i=0; i < listIx.size(); i++)   {
            listIx[i] += listMeshVx.size();
        }

        // copy data to mesh
        listMeshVx.insert(listMeshVx.end(),listVx.begin(),listVx.end());
        listMeshIx.insert(listMeshIx.end(),listIx.begin(),listIx.end());

        // clean up
        delete opPolygons;
        OGRFeature::DestroyFeature(ipFeature);
        ixFeature++;
    }

    std::cout << "szMeshVx: " << listMeshVx.size() << std::endl;
    std::cout << "szMeshIx: " << listMeshIx.size() << std::endl;


    // debug with openscenegraph
    osg::ref_ptr<osg::Vec3dArray> gmVxArray = new osg::Vec3dArray;
    for(size_t i=0; i < listMeshVx.size(); i++)
    {   gmVxArray->push_back(osg::Vec3d(listMeshVx[i].x,listMeshVx[i].y,0));   }

    osg::ref_ptr<osg::Vec4Array> gmCxArray = new osg::Vec4Array;
    gmCxArray->push_back(osg::Vec4(1,1,0,1));

    osg::ref_ptr<osg::DrawElementsUInt> gmIxArray =
            new osg::DrawElementsUInt(GL_TRIANGLES);
    for(size_t i=0; i < listMeshIx.size(); i++)
    {   gmIxArray->push_back(listMeshIx[i]);   }

    osg::ref_ptr<osg::Geometry> gmMesh = new osg::Geometry;
    gmMesh->setVertexArray(gmVxArray);
    gmMesh->setColorArray(gmCxArray);
    gmMesh->setColorBinding(osg::Geometry::BIND_OVERALL);
    gmMesh->addPrimitiveSet(gmIxArray);

    osg::ref_ptr<osg::Geode> gdMesh = new osg::Geode;
    gdMesh->addDrawable(gmMesh);

    osg::ref_ptr<osg::Group> gpRoot = new osg::Group;
    gpRoot->addChild(gdMesh);

    osg::StateSet * ss = gpRoot->getOrCreateStateSet();
    ss->setMode(GL_BLEND,osg::StateAttribute::ON);
    ss->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
    //ss->setMode(GL_CULL_FACE,osg::StateAttribute::ON |
    //                         osg::StateAttribute::OVERRIDE);


    // setup viewer
    osgViewer::Viewer viewer;
    viewer.setThreadingModel(osgViewer::ViewerBase::SingleThreaded);
    viewer.setUpViewInWindow(100,100,800,480);
    viewer.setSceneData(gpRoot.get());
    return viewer.run();
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

OGRMultiPolygon* repair(OGRGeometry* geometry, Triangulation &triangulation, void * &interior, void * &exterior) {

    // Triangulation
    //  Triangulation triangulation;

//    std::cout << "Input Geometry Type: " << geometry->getGeometryType()
//              << ", Expected Type: " << wkbPolygon << std::endl;


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
    //void *interior = malloc(sizeof(void *));
    //void *exterior = malloc(sizeof(void *));
    interior = malloc(sizeof(void *));
    exterior = malloc(sizeof(void *));
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
