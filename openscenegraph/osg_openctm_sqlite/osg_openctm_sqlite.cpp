#include <iostream>
#include <exception>
#include <iomanip>
#include <cstring>
#include <fstream>
#include <stack>
#include <vector>
#include <dirent.h>
#include <sys/time.h>

// kompex
#include "kompex/KompexSQLitePrerequisites.h"
#include "kompex/KompexSQLiteDatabase.h"
#include "kompex/KompexSQLiteStatement.h"
#include "kompex/KompexSQLiteException.h"
#include "kompex/KompexSQLiteStreamRedirection.h"
#include "kompex/KompexSQLiteBlob.h"

// openctm
#include "openctm/openctm.h"

// openscenegraph
#include <osg/Geometry>
#include <osg/PolygonMode>
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>
#include <osg/Geode>
#include <osgViewer/Viewer>

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
              << timeTaken << " microseconds" << std::endl;
}

std::string NumberToString ( double number )
{
    std::stringstream ss;
    ss << number;
    return ss.str();
}

double StringToNumber ( const std::string &Text )
{
    std::stringstream ss(Text);
    double result;
    return ss >> result ? result : 0;
}

std::string ParseFileName(std::string fileName)
{
    std::string fileKey;

    if(fileName.substr(0,1).compare("W") == 0)
    {   fileKey += "0";   }
    else
    {   fileKey += "1";   }

    fileKey += fileName.substr(fileName.find_last_of("_")+1);
    return fileKey;
}

unsigned int g_pos;

static CTMuint CTMCALL ctmReadBlob(void * aBuf,CTMuint aCount,void * aUserData)
{
    // aBuf - target buffer
    // aUserData - source buffer
    // aCount - num of bytes to read

    char * sourceBuffer = (char *) aUserData;
    char * targetBuffer = (char *) aBuf;
    unsigned int bytesWritten = g_pos;
    for(unsigned int i=0; i < aCount; i++)   {
        targetBuffer[i] = sourceBuffer[g_pos];
        g_pos++;
    }
    bytesWritten = g_pos-bytesWritten;
    return bytesWritten;

//  return (CTMuint) fread(aBuf, 1, (size_t) aCount, (FILE *) aUserData);
}

std::ifstream::pos_type size;
char * memblock;

int main(int argc, const char *argv[])
{
    if(argc != 3) {
        std::cout << "Usage: #> ./osg_openctm_sqlite inputdb row\n";
        std::cout << "* draw an openctm mesh stored as a row in an sqlite db with osg\n";
        return 0;
    }

    Kompex::SQLiteDatabase *pDatabase = new Kompex::SQLiteDatabase(argv[1],SQLITE_OPEN_READWRITE,0);
    Kompex::SQLiteStatement *pStmt = new Kompex::SQLiteStatement(pDatabase);

    int cRow = 0;
    int rowId = 0;
    int dbRow = 2;
    bool isEmpty = false;

    std::string rowFileKey;
    pStmt->Sql("SELECT rowid,filekey,empty FROM grid_mesh;");
    while(pStmt->FetchRow())   {
        if(cRow == dbRow)   {
            // row file key
            rowId = pStmt->GetColumnInt(0);
            rowFileKey = pStmt->GetColumnString(1);
            isEmpty = pStmt->GetColumnBool(2);
        }
        cRow++;
    }
    pStmt->FreeQuery();
    Kompex::SQLiteBlob *pKompexBlob;

    if(!isEmpty)   {
        // read in ctm blob to memory
        pKompexBlob=new Kompex::SQLiteBlob(pDatabase,"main","grid_mesh","mesh",1);
        size = pKompexBlob->GetBlobSize();
        memblock = new char[size];
        pKompexBlob->ReadBlob(memblock,size);
        delete pKompexBlob;
        std::cout << "# Read in " << rowFileKey << " (" << size << " bytes)\n";

        // uncompress ctm
        CTMcontext context;
        CTMuint    vertCount, triCount;
        CTMuint const* indices;
        CTMfloat   const * vertices;

        context = ctmNewContext(CTM_IMPORT);

        g_pos = 0;
        ctmLoadCustom(context,ctmReadBlob,(void*)memblock);
        if(ctmGetError(context) == CTM_NONE)
        {
            // access the mesh data
            vertCount = ctmGetInteger(context, CTM_VERTEX_COUNT);
            vertices = ctmGetFloatArray(context, CTM_VERTICES);
            triCount = ctmGetInteger(context, CTM_TRIANGLE_COUNT);
            indices = ctmGetIntegerArray(context, CTM_INDICES);

            std::cout << "# Mesh has " << vertCount << " vertices\n";
            std::cout << "# Mesh has " << triCount << " triangles\n";

            // build up openscenegraph geometry
            osg::ref_ptr<osg::Vec3Array> listVxArray = new osg::Vec3Array(vertCount);
            unsigned int vxIdx=0;
            for(int i=0; i < listVxArray->size(); i++)   {
                osg::Vec3 vertex;
                vertex.x() = vertices[vxIdx]; vxIdx++;
                vertex.y() = vertices[vxIdx]; vxIdx++;
                vertex.z() = vertices[vxIdx]; vxIdx++;
                listVxArray->at(i) = vertex;
            }

            osg::ref_ptr<osg::DrawElementsUInt> listIdxs =
                    new osg::DrawElementsUInt(GL_TRIANGLES,triCount*3);
            for(int i=0; i < listIdxs->size(); i++)   {
                listIdxs->at(i) = indices[i];
            }

            osg::ref_ptr<osg::Geometry> geomMesh = new osg::Geometry;
            geomMesh->setVertexArray(listVxArray.get());
            geomMesh->setNormalArray(listVxArray.get());
            geomMesh->addPrimitiveSet(listIdxs.get());

            osg::ref_ptr<osg::Geode> geodeMesh = new osg::Geode;
            geodeMesh->addDrawable(geomMesh.get());

            osg::ref_ptr<osg::Group> groupRoot = new osg::Group;
            groupRoot->addChild(geodeMesh.get());

            // free ctm memory
            ctmFreeContext(context);

            // start viewer
            osgViewer::Viewer viewer;
            viewer.setThreadingModel(osgViewer::ViewerBase::SingleThreaded);
            viewer.setUpViewInWindow(100,100,800,480);
            viewer.setSceneData(groupRoot.get());
            return viewer.run();
        }
        else
        {
            std::cout << "# Fatal error reading in mesh file! Exiting...\n";
            ctmFreeContext(context);
            return 0;
        }



//        // stuff
        delete[] memblock;
//        delete pStmt;
//        delete pDatabase;
        return 0;
    }
    else   {
        std::cout << "# Row " << rowId << " was empty! Exiting...\n";
        delete pStmt;
        delete pDatabase;
        return 0;
    }
}

