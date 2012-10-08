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

int main(int argc, const char *argv[])
{   
    if(argc != 2)   {
        std::cout << "Usage: #> ./osg_coastlines inputfile.bin\n";
        std::cout << "* Expect input file to be a list of doubles "
                     "that represents a bunch of line loops in ECEF.\n";
        std::cout << "* Input file uses (0,0,0) points to indicate "
                     "the start of a loop\n";
        std::cout << "* Use wkt2bin (scratch/shapefiles) to generate "
                     "input file if it isn't present\n";
    }

    // read in file
    std::vector<double> listInpDims; // ordered x,y,z,x,y,z...
    std::ifstream inputFile(argv[1],std::ios::in | std::ios::binary);
    if(!inputFile.is_open())   {
        std::cout << "Error: Couldn't open input file: " << argv[1];
        std::cout << "\nExiting...\n"; return -1;
    }

    double mVal;
    while(inputFile)   {
        inputFile.read(reinterpret_cast<char*>(&mVal),sizeof(double));
        listInpDims.push_back(mVal);
    }
    inputFile.close();

    std::cout << "Read in " << listInpDims.size() << " values\n";

    // convert the data into an osg array
    osg::ref_ptr<osg::Vec3dArray> listVx = new osg::Vec3dArray;
    for(size_t i=0; i < listInpDims.size()-1; i++)    {
        osg::Vec3d mVx;
        mVx.x() = listInpDims[i]; i++;
        mVx.y() = listInpDims[i]; i++;
        mVx.z() = listInpDims[i];
        listVx->push_back(mVx);
    }

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
