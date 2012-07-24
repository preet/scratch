 // sys includes
#include <math.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <sys/time.h>

// geometry
#include "Vec3.hpp"

// osg includes
#include <osg/GL>
#include <osg/GLU>
#include <osg/PolygonMode>
#include <osg/Drawable>
#include <osg/Geometry>
#include <osgViewer/Viewer>



// define self-intersecting star shape (with color)
    //      0
    //     / \
    //3---+---+---2
    //  \ |   | /
    //   \|   |/
    //    +   +
    //    |\ /|
    //    | + |
    //    |/ \|
    //    1   4
    GLdouble star[5][6] = { { 0.0, 3.0, 0,  1, 0, 0},       // 0: x,y,z,r,g,b
                            {-1.0, 0.0, 0,  0, 1, 0},       // 1:
                            { 1.6, 1.9, 0,  1, 0, 1},       // 2:
                            {-1.6, 1.9, 0,  1, 1, 0},       // 3:
                            { 1.0, 0.0, 0,  0, 0, 1} };     // 4:

    // define concave quad data (vertices only)
    //  0    2
    //  \ \/ /
    //   \3 /
    //    \/
    //    1
    GLdouble quad1[4][3] = { {-5.0789754,-23.433885,-23.156778},
                             {-15.99901,-7.8413014,-4.9914378},
                             {10.231934,3.1406977,1.2619247},
                             {21.144129,-12.45333,-16.903392} };

    // define concave quad with a hole
    //  0--------3
    //  | 4----7 |
    //  | |    | |
    //  | 5----6 |
    //  1--------2
    GLdouble quad2[8][3] = { {-2,3,10}, {-2,0,10}, {2,0,10}, { 2,3,10},
                             {-1,2,10}, {-1,1,10}, {1,1,10}, { 1,2,10} };

    GLdouble quad3[8][3] = { {-2,3,10}, {-2,0,10}, {2,0,10}, { 2,3,10},
                             {-1,2,10}, {-1,1,10}, {1,1,10}, { 1,2,10} };

int main()
{
    std::vector<Vec3> listOuterVx;
    listOuterVx.push_back(Vec3(0,3,0));
    listOuterVx.push_back(Vec3(-1,0,0));
    listOuterVx.push_back(Vec3(1.6,1.9,0));
    listOuterVx.push_back(Vec3(-1.6,1.9,0));
    listOuterVx.push_back(Vec3(1,0,0));

    Vec3 offsetVec(0,0,8);

    std::vector<Vec3> const &listBtmVx = listOuterVx;
    std::vector<Vec3> listTopVx(listBtmVx.size());
    for(size_t i=0; i < listTopVx.size(); i++)
    {   listTopVx[i] = listBtmVx[i] + offsetVec;   }

    size_t v=0;
    Vec3 alongLeft,alongUp,triNx;
    std::vector<Vec3> listTriVx;
    std::vector<Vec3> listTriNx;
    for(v=0; v < listBtmVx.size()-1; v++)
    {
        // triangle 1
        listTriVx.push_back(listBtmVx[v]);
        listTriVx.push_back(listBtmVx[v+1]);
        listTriVx.push_back(listTopVx[v+1]);

        // triangle 2
        listTriVx.push_back(listBtmVx[v]);
        listTriVx.push_back(listTopVx[v+1]);
        listTriVx.push_back(listTopVx[v]);

        // normal
        alongUp = (listTopVx[v]-listBtmVx[v]);
        alongLeft = (listBtmVx[v]-listBtmVx[v+1]);

        triNx = alongUp.Cross(alongLeft).Normalized();
        listTriNx.insert(listTriNx.end(),6,triNx);
    }

    // v is now pointing to the last vertex
    // in the contour so add the last face

    // triangle 1
    listTriVx.push_back(listBtmVx[v]);
    listTriVx.push_back(listBtmVx[0]);
    listTriVx.push_back(listTopVx[0]);

    // triangle 2
    listTriVx.push_back(listBtmVx[v]);
    listTriVx.push_back(listTopVx[0]);
    listTriVx.push_back(listTopVx[v]);

    // normal
    alongUp = (listTopVx[0]-listBtmVx[0]);
    alongLeft = (listBtmVx[v]-listBtmVx[0]);
    triNx = alongUp.Cross(alongLeft).Normalized();
    listTriNx.insert(listTriNx.end(),6,triNx);

    std::cout << "Added " << listTriVx.size() << " vertex positions";
    std::cout << "Added " << listTriNx.size() << " normal positions";

    osg::ref_ptr<osg::Vec3Array> listVx = new osg::Vec3Array;
    osg::ref_ptr<osg::Vec3Array> listNx = new osg::Vec3Array;
    for(size_t i=0; i < listTriVx.size(); i++)   {
        // save vertex
        listVx->push_back(osg::Vec3(listTriVx[i].x,
                                    listTriVx[i].y,
                                    listTriVx[i].z));
        // save normal
        listNx->push_back(osg::Vec3(listTriNx[i].x,
                                    listTriNx[i].y,
                                    listTriNx[i].z));
    }

    osg::ref_ptr<osg::Geometry> myGeom = new osg::Geometry;
    myGeom->setVertexArray(listVx);
    myGeom->setNormalArray(listNx);
    myGeom->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);
    myGeom->addPrimitiveSet(new osg::DrawArrays(GL_TRIANGLES,0,listVx->size()));

    osg::ref_ptr<osg::Geode> myGeode = new osg::Geode;
    myGeode->addDrawable(myGeom);

    osg::ref_ptr<osg::Group> nodeRoot = new osg::Group;
    osg::StateSet * ss = nodeRoot->getOrCreateStateSet();
    nodeRoot->addChild(myGeode);

    // wireframe
//    osg::PolygonMode *polyModeObj = new osg::PolygonMode;
//    polyModeObj->setMode(osg::PolygonMode::FRONT_AND_BACK,
//                         osg::PolygonMode::LINE);
//    ss->setAttribute(polyModeObj);

    // setup viewer
    osgViewer::Viewer viewer;
    viewer.setThreadingModel(osgViewer::ViewerBase::SingleThreaded);
    viewer.setUpViewInWindow(100,100,800,480);
    viewer.setSceneData(nodeRoot.get());
    return viewer.run();
}
