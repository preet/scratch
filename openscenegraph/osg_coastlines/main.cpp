// sys includes
#include <math.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <sys/time.h>

// osg includes
#include <osg/Geometry>
#include <osg/PolygonMode>
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>
#include <osg/Geode>
#include <osgViewer/Viewer>


// OpenCTM Includes
#include "openctm.h"

int main(int argc, const char *argv[])
{   
    if(argc != 2)   {
        std::cout << "Usage: #> ./osg_coastlines coastline.ctm\n";
        std::cout << "* Input OpenCTM coastline data.\n";
    }

    osg::ref_ptr<osg::Vec3dArray> listVx = new osg::Vec3dArray;

    // open input file
    CTMcontext       ctmContext;
    CTMuint          ctmVxCount;
    CTMfloat const * ctmListVx;

    ctmContext = ctmNewContext(CTM_IMPORT);
    ctmLoad(ctmContext,argv[1]);
    if(ctmGetError(ctmContext) == CTM_NONE)
    {
        ctmVxCount = ctmGetInteger(ctmContext,CTM_VERTEX_COUNT);
        ctmListVx  = ctmGetFloatArray(ctmContext,CTM_VERTICES);

        size_t k=0;
        for(size_t i=0; i < ctmVxCount; i++)   {
            osg::Vec3d mVx;
            mVx.x() = ctmListVx[k]; k++;
            mVx.y() = ctmListVx[k]; k++;
            mVx.z() = ctmListVx[k]; k++;
            listVx->push_back(mVx);
        }
    }
    else
    {   std::cout << "Error: Could not read input file\n"; return -1;   }

    std::cout << "Read in " << ctmVxCount << " vertices";
    ctmFreeContext(ctmContext);

    osg::ref_ptr<osg::DrawElementsUInt> listIx =
            new osg::DrawElementsUInt(GL_LINES);

    for(size_t i=0; i < listVx->size(); i++)   {

        if(listVx->at(i) == osg::Vec3d(0,0,0))   {

            if(listIx->size() > 0)   {
                listIx->pop_back();
            }

            i++;
            listIx->push_back(i);
            continue;
        }

        listIx->push_back(i);
        listIx->push_back(i);
    }

    osg::ref_ptr<osg::Geometry> geomCoastlines = new osg::Geometry;
    geomCoastlines->getOrCreateStateSet()->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
    geomCoastlines->setVertexArray(listVx);
    geomCoastlines->addPrimitiveSet(listIx);

    osg::ref_ptr<osg::Geode> geodeCoastlines = new osg::Geode;
    geodeCoastlines->addDrawable(geomCoastlines);

    osg::ref_ptr<osg::Group> groupRoot = new osg::Group;
    groupRoot->addChild(geodeCoastlines);

    // start viewer
    osgViewer::Viewer viewer;
    viewer.setThreadingModel(osgViewer::ViewerBase::SingleThreaded);
    viewer.setUpViewInWindow(100,100,800,480);
    viewer.setSceneData(groupRoot);
    return viewer.run();
}
