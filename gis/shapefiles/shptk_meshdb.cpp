// sys
#include <dirent.h>

// stl
#include <iostream>
#include <iomanip>
#include <cstring>
#include <fstream>
#include <stack>
#include <set>

// openctm
#include <openctm.h>

// kompex
#include <KompexSQLitePrerequisites.h>
#include <KompexSQLiteDatabase.h>
#include <KompexSQLiteStatement.h>
#include <KompexSQLiteException.h>
#include <KompexSQLiteStreamRedirection.h>
#include <KompexSQLiteBlob.h>

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

// custom write function to store ctm mesh as an sqlite blob
//unsigned int g_pos;
//static CTMuint CTMCALL ctmReadBlob(void * aBuf,CTMuint aCount,void * aUserData)
//{
//    // aBuf - target buffer
//    // aUserData - source buffer
//    // aCount - num of bytes to read

//    char * sourceBuffer = (char *) aUserData;
//    char * targetBuffer = (char *) aBuf;
//    unsigned int bytesWritten = g_pos;
//    for(unsigned int i=0; i < aCount; i++)   {
//        targetBuffer[i] = sourceBuffer[g_pos];
//        g_pos++;
//    }
//    bytesWritten = g_pos-bytesWritten;
//    return bytesWritten;
//}

size_t g_bytesWritten;
static CTMuint CTMCALL ctmWriteBlob(const void * aBuf,
                                    CTMuint aCount,
                                    void * aUserData)
{
    // copy data over
    const char * sourceBuffer = (const char *) aBuf;
    std::vector<char> * targetBuffer = (std::vector<char>*)aUserData;
    targetBuffer->reserve(aCount);
    for(size_t i=0; i < aCount; i++)   {
        targetBuffer->push_back(sourceBuffer[i]);
        g_bytesWritten++;
    }
    return g_bytesWritten;
}



int main(int argc, const char *argv[])
{
    if(argc != 3) {
        std::cout << "Usage: #> ./shptk_meshgen inputdir outputfile.sqlite\n";
        std::cout << "* This util will convert the shapefiles in inputdir to \n";
        std::cout << "  meshes in OpenCTM format and save them in an sqlite db\n";
        std::cout << "* Expect inputdir to contain shapefiles with POLYGON data only\n";
        std::cout << "* Expect shapefiles to be named TILE_(quadkey), ie TILE_00110011\n";
        return 0;
    }

    // check if GDAL was built against GEOS
    if(OGRGeometryFactory::haveGEOS())   {
        // register OGR drivers
        OGRRegisterAll();
    }
    else   {
        std::cout << "ERROR: GDAL needs to be built against "
                     "GEOS to use this tool" << std::endl;
        return -1;
    }

    std::string ipPath(argv[1]);
    std::string opPath(argv[2]);

    //
    GeoBounds rootBounds;
    rootBounds.minLon = -180.0;
    rootBounds.maxLon = 0.0;
    rootBounds.minLat = -90.0;
    rootBounds.maxLat = 90.0;
    size_t magLevel = 7;
    double numDivs = pow(2,double(magLevel));
    double lonWidth = (rootBounds.maxLon-rootBounds.minLon)/numDivs;
    double latWidth = (rootBounds.maxLat-rootBounds.minLat)/numDivs;

    // get a list of files in the input path
    std::vector<std::string> listFiles;
    DIR *dir; struct dirent *ent;
    dir = opendir(argv[1]);
    if(dir != NULL)   {
        while((ent=readdir(dir)) != NULL)   {
            std::string fileName(ent->d_name);
            if(!(fileName.compare(".") == 0) &&
               !(fileName.compare("..") == 0))   {

                // push back files with a 'shp' extension
                std::string fileExt =
                        fileName.substr(fileName.size()-4,
                                        fileName.size()-1);
                if(fileExt.compare(".shp") == 0)   {
                    listFiles.push_back(fileName);
                }
            }
        }
    }
    size_t numFiles = listFiles.size();
    if(numFiles == 0)   {
        std::cout << "INFO: No files found" << std::endl;
    }
    else   {
        std::cout << "INFO: Found " << numFiles
                  << " input files" << std::endl;
    }

    // open/create database
    // Schema: We have one table:
    // TILES:  ROWID, MAG, X, Y, MESH

    Kompex::SQLiteDatabase  * pDatabase =
            new Kompex::SQLiteDatabase(opPath,SQLITE_OPEN_READWRITE |
                                              SQLITE_OPEN_CREATE,0);

    Kompex::SQLiteStatement * pStmt =
            new Kompex::SQLiteStatement(pDatabase);

    pStmt->SqlStatement("CREATE TABLE IF NOT EXISTS TILES("
                        "MAG INTEGER NOT NULL, "
                        "X INTEGER NOT NULL, "
                        "Y INTEGER NOT NULL, "
                        "MESH BLOB)");

    // process file by file
    for(size_t f=0; f < numFiles; f++)
    {
        std::cout << "File " << f << "/" << numFiles << std::endl;

        // get quadkey and bounds
        std::string quadKey  = listFiles[f].substr(5,listFiles[f].size()-4);
        GeoBounds tileBounds;
        CalcBoundsFromQuadKey(quadKey,rootBounds,tileBounds);

        double adjLon = (tileBounds.minLon + (lonWidth/2.0)) + 180.0;       // 0-360 [W->E]
        double adjLat = 90.0 - (tileBounds.minLat + (latWidth/2.0));        // 0-180 [N->S]

        size_t lonIx = adjLon/lonWidth;
        size_t latIx = adjLat/latWidth;

        // std::cout << "LONIX: " << lonIx << " , LATIX: " << latIx << std::endl;

        // open input file and feature layer
        std::string fullFilePath = ipPath + listFiles[f];
        OGRDataSource * ipShpFile = OGRSFDriverRegistrar::Open(fullFilePath.c_str(),FALSE);
        if(ipShpFile == NULL)   {
            std::cout << "ERROR: Could not open input file: "
                      << listFiles[f] << std::endl;
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
            // std::cout << "INFO: Feature " << ixFeature
            //           << "/" << numFeatures << std::endl;

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
                // std::cout << "sf->info(): " << fIt->info() << ", "
                //           << "interior: " << (interior) << ", "
                //           << "exterior: " << (exterior) << std::endl;

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

        if(listMeshIx.size() < 3)   {
            std::cout << "INFO: " << listFiles[f] << " has no mesh data ";
            continue;
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

        // save as a blob
        std::vector<char> listData;
        g_bytesWritten = 0;
        ctmSaveCustom(context,ctmWriteBlob,&listData);

        char * ctmBlob = new char[listData.size()];
        for(size_t b=0; b < g_bytesWritten; b++)   {
            ctmBlob[b] = listData[b];
        }

        try   {
            pStmt->Sql("INSERT INTO TILES(MAG,X,Y,MESH) VALUES(?,?,?,?);");
            pStmt->BindInt(1,int(magLevel));
            pStmt->BindInt(2,int(lonIx));
            pStmt->BindInt(3,int(latIx));
            pStmt->BindBlob(4,ctmBlob,g_bytesWritten);
            pStmt->ExecuteAndFree();
        }
        catch(Kompex::SQLiteException &exception)   {
            std::cout << "sqlite exception: "
                      << exception.GetString();
        }
        delete ctmBlob;
    }

    delete pStmt;
    delete pDatabase;

    return 0;
}


































