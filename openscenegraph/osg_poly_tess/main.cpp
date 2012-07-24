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


void TessBeginCallback(GLenum type)
{
    std::cout << "Draw Mode: " << int(type) << std::endl;
}

void TessVertexCallback(void * vertex_data, void * finalGeometry)
{
    GLdouble const * vPtr;
    vPtr = (GLdouble *) vertex_data;

    std::vector<Vec3> * myGeometry = (std::vector<Vec3> *)finalGeometry;
    myGeometry->push_back(Vec3(vPtr[0],vPtr[1],vPtr[2]));
}

void TessCombineCallback(GLdouble coords[3],
                         void * vertex_data[4],
                         GLfloat weight[4],
                         void **dataOut,
                         void * finalGeometry)
{
    GLdouble * vPtr;
    vPtr = (GLdouble *) malloc(3*sizeof(GLdouble));
    vPtr[0] = coords[0];
    vPtr[1] = coords[1];
    vPtr[2] = coords[2];

    std::cout << "Combine!" << std::endl;

//    std::vector<Vec3> * myGeometry = (std::vector<Vec3> *) finalGeometry;
//    myGeometry->push_back(Vec3(vPtr[0],vPtr[1],vPtr[2]));

    *dataOut = vPtr;

    // dont i have to free this memory (vPtr)?
}

void TessEdgeCallback()
{}

void TessEndCallback()
{}

void TessErrorCallback(GLenum errorCode)
{
    const GLubyte *errString;
    errString = osg::gluErrorString(errorCode);

    std::cout << "Tessellation Error Code: "
              << int(errorCode);
}

int main()
{
    // geometry
//    std::vector<Vec3> star;
//    star.push_back(Vec3(250,50,0));
//    star.push_back(Vec3(1,0,1));
//    star.push_back(Vec3(325,200,0));
//    star.push_back(Vec3(1,1,0));
//    star.push_back(Vec3(400,50,0));
//    star.push_back(Vec3(0,1,1));
//    star.push_back(Vec3(250,150,0));
//    star.push_back(Vec3(1,0,0));
//    star.push_back(Vec3(400,150,0));
//    star.push_back(Vec3(0,1,0));

    bool drawStar=false;
    bool drawQuad1=true;
    bool drawQuad2=false;

    std::vector<Vec3> tessellatedGeometry;

    osg::GLUtesselator * m_tessobj;
    m_tessobj = osg::gluNewTess();

    osg::gluTessCallback(m_tessobj,GLU_TESS_VERTEX_DATA,        (void(*)())TessVertexCallback);
    osg::gluTessCallback(m_tessobj,GLU_TESS_COMBINE_DATA,       (void(*)())TessCombineCallback);
    osg::gluTessCallback(m_tessobj,GLU_TESS_EDGE_FLAG_DATA,     (void(*)())TessEdgeCallback);
    osg::gluTessCallback(m_tessobj,GLU_TESS_BEGIN,              (void(*)())TessBeginCallback);
    osg::gluTessCallback(m_tessobj,GLU_TESS_END,                (void(*)())TessEndCallback);
    osg::gluTessCallback(m_tessobj,GLU_TESS_ERROR,              (void(*)())TessErrorCallback);

    osg::gluTessProperty(m_tessobj,GLU_TESS_WINDING_RULE,GLU_TESS_WINDING_ODD);
    osg::gluTessBeginPolygon(m_tessobj,&tessellatedGeometry);

    if(drawStar)   {
        osg::gluTessBeginContour(m_tessobj);
        osg::gluTessVertex(m_tessobj, star[0], star[0]);
        osg::gluTessVertex(m_tessobj, star[1], star[1]);
        osg::gluTessVertex(m_tessobj, star[2], star[2]);
        osg::gluTessVertex(m_tessobj, star[3], star[3]);
        osg::gluTessVertex(m_tessobj, star[4], star[4]);
        osg::gluTessEndContour(m_tessobj);
    }
    else if(drawQuad1)   {
        osg::gluTessBeginContour(m_tessobj);
        osg::gluTessVertex(m_tessobj, quad1[0], quad1[0]);
        osg::gluTessVertex(m_tessobj, quad1[1], quad1[1]);
        osg::gluTessVertex(m_tessobj, quad1[2], quad1[2]);
        osg::gluTessVertex(m_tessobj, quad1[3], quad1[3]);
        osg::gluTessEndContour(m_tessobj);
    }
    else if(drawQuad2)   {
        osg::gluTessBeginContour(m_tessobj);
        osg::gluTessVertex(m_tessobj, quad2[0], quad2[0]);
        osg::gluTessVertex(m_tessobj, quad2[1], quad2[1]);
        osg::gluTessVertex(m_tessobj, quad2[2], quad2[2]);
        osg::gluTessVertex(m_tessobj, quad2[3], quad2[3]);
        osg::gluTessEndContour(m_tessobj);

        osg::gluTessBeginContour(m_tessobj);
        osg::gluTessVertex(m_tessobj, quad3[4], quad3[4]);
        osg::gluTessVertex(m_tessobj, quad3[5], quad3[5]);
        osg::gluTessVertex(m_tessobj, quad3[6], quad3[6]);
        osg::gluTessVertex(m_tessobj, quad3[7], quad3[7]);
        osg::gluTessEndContour(m_tessobj);
    }


    osg::gluTessEndPolygon(m_tessobj);
    osg::gluDeleteTess(m_tessobj);

    std::cout << "TessellatedGeometry size: " << tessellatedGeometry.size() << std::endl;

    osg::ref_ptr<osg::Vec3Array> listVx = new osg::Vec3Array;
    osg::ref_ptr<osg::Vec3Array> listNx = new osg::Vec3Array;

    for(int i=0; i < tessellatedGeometry.size(); i++)   {
        listVx->push_back(osg::Vec3(tessellatedGeometry[i].x,
                                    tessellatedGeometry[i].y,
                                    tessellatedGeometry[i].z));
    }

//    listVx->push_back(osg::Vec3(0.5,-0.5,0));
//    listVx->push_back(osg::Vec3(-0.5,-0.5,0));
//    listVx->push_back(osg::Vec3(0,1,0));

    listNx->push_back(osg::Vec3(0,0,1));

    osg::ref_ptr<osg::Geometry> myGeom = new osg::Geometry;
    myGeom->setVertexArray(listVx);
    myGeom->setNormalArray(listNx);
    myGeom->setNormalBinding(osg::Geometry::BIND_OVERALL);
    myGeom->addPrimitiveSet(new osg::DrawArrays(GL_TRIANGLES,0,listVx->size()));

    osg::ref_ptr<osg::Geode> myGeode = new osg::Geode;
    myGeode->addDrawable(myGeom);

    osg::ref_ptr<osg::Group> nodeRoot = new osg::Group;
    nodeRoot->addChild(myGeode);

    // setup viewer
    osgViewer::Viewer viewer;
    viewer.setThreadingModel(osgViewer::ViewerBase::SingleThreaded);
    viewer.setUpViewInWindow(100,100,800,480);
    viewer.setSceneData(nodeRoot.get());
    return viewer.run();
}
