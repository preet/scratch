// STL
#include <iostream>
#include <iomanip>
#include <cstring>
#include <fstream>
#include <stack>
#include <set>

// openctm
#include <openctm.h>

// OpenSceneGraph (debug only)
#include <osg/Geometry>
#include <osg/PolygonMode>
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>
#include <osg/Geode>
#include <osgViewer/Viewer>

// shptk
#include "shptk.hpp"
#include "shptk_prepair.hpp"

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

    std::cout << "Feature Count: " << numFeatures << std::endl;

    // iterate through input features and build mesh
    std::vector<Vec2> listMeshVx;
    std::vector<size_t> listMeshIx;
    size_t ixFeature = 0;
    OGRFeature * ipFeature;
    while( (ipFeature = ipLayer->GetNextFeature()) != NULL )
    {
//        std::cout << "INFO: Feature " << ixFeature
//                  << "/" << numFeatures << std::endl;

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
        CalcPolylineSimplifyVW(listOuterVx,listOuterVxSimp,VW_AREA,1500.0);
        OGRLinearRing * sOuterRing = new OGRLinearRing;
        for(size_t i=0; i < listOuterVxSimp.size(); i++)   {
            PointLLA lla = ConvECEFToLLA(listOuterVxSimp[i]);
            sOuterRing->addPoint(lla.lon,lla.lat,0);
        }
        if(sOuterRing->getNumPoints() < 3)   {
            delete sOuterRing;
            continue;
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
            CalcPolylineSimplifyVW(listInnerVx,listInnerVxSimp,VW_AREA,1500.0);
            OGRLinearRing * sInnerRing = new OGRLinearRing;
            for(size_t j=0; j < listInnerVxSimp.size(); j++)   {
                PointLLA lla = ConvECEFToLLA(listInnerVxSimp[j]);
                sInnerRing->addPoint(lla.lon,lla.lat,0);
            }
            if(sInnerRing->getNumPoints() < 3)   {
                delete sInnerRing;
                continue;
            }
            sPolygon->addRingDirectly(sInnerRing);
        }
        sPolygon->flattenTo2D();    // required to convert to wkbPolygon

        // geometry mesh
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

    if(listMeshIx.size() < 3)   {
        std::cout << "INFO: " << ipShpFile << " has no mesh data ";
        return 0;
    }

    // convert mesh to CTM format
    CTMcontext context;
    CTMuint vertCount, triCount, *indices;
    CTMfloat *vertices;

    // create context
    context = ctmNewContext(CTM_EXPORT);
    ctmCompressionMethod(context,CTM_METHOD_MG1);
    ctmCompressionLevel(context,5);

    // create mesh
    vertCount   = listMeshVx.size();
    triCount    = listMeshIx.size()/3;
    vertices    = (CTMfloat *) malloc(3 * sizeof(CTMfloat) * vertCount);
    indices     = (CTMuint *) malloc(3 * sizeof(CTMuint) * triCount);

    unsigned int vIdx=0;
    for(int i=0; i < vertCount; i++)   {
        vertices[vIdx] = listMeshVx[i].x; vIdx++;
        vertices[vIdx] = listMeshVx[i].y; vIdx++;
        vertices[vIdx] = 0.0; vIdx++;
    }

    for(int i=0; i < triCount*3; i++)   {
        indices[i] = listMeshIx[i];
    }

    // define mesh
    ctmDefineMesh(context,vertices,vertCount,indices,triCount,NULL);

    // save
    ctmSave(context,argv[2]);

    // free context/mesh
    ctmFreeContext(context);
    free(indices);
    free(vertices);

    return 0;
}

//// debug with openscenegraph
//osg::ref_ptr<osg::Vec3dArray> gmVxArray = new osg::Vec3dArray;
//for(size_t i=0; i < listMeshVx.size(); i++)
//{   gmVxArray->push_back(osg::Vec3d(listMeshVx[i].x,listMeshVx[i].y,0));   }

//osg::ref_ptr<osg::Vec4Array> gmCxArray = new osg::Vec4Array;
//gmCxArray->push_back(osg::Vec4(1,1,0,1));

//osg::ref_ptr<osg::DrawElementsUInt> gmIxArray =
//        new osg::DrawElementsUInt(GL_TRIANGLES);
//for(size_t i=0; i < listMeshIx.size(); i++)
//{   gmIxArray->push_back(listMeshIx[i]);   }

//osg::ref_ptr<osg::Geometry> gmMesh = new osg::Geometry;
//gmMesh->setVertexArray(gmVxArray);
//gmMesh->setColorArray(gmCxArray);
//gmMesh->setColorBinding(osg::Geometry::BIND_OVERALL);
//gmMesh->addPrimitiveSet(gmIxArray);

//osg::ref_ptr<osg::Geode> gdMesh = new osg::Geode;
//gdMesh->addDrawable(gmMesh);

//osg::ref_ptr<osg::Group> gpRoot = new osg::Group;
//gpRoot->addChild(gdMesh);

//osg::StateSet * ss = gpRoot->getOrCreateStateSet();
//ss->setMode(GL_BLEND,osg::StateAttribute::ON);
//ss->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
////ss->setMode(GL_CULL_FACE,osg::StateAttribute::ON |
////                         osg::StateAttribute::OVERRIDE);


//// setup viewer
//osgViewer::Viewer viewer;
//viewer.setThreadingModel(osgViewer::ViewerBase::SingleThreaded);
//viewer.setUpViewInWindow(100,100,800,480);
//viewer.setSceneData(gpRoot.get());
//return viewer.run();
