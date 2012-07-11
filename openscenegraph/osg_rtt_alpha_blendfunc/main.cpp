#include <osg/Geode>
#include <osg/Group>
#include <osg/Material>
#include <osg/Texture2D>
#include <osg/BlendFunc>
#include <osgViewer/Viewer>

int main( int argc, char **argv )
{
    // example shows how to render an alpha 'mask' to
    // an fbo which is then applied to an object
    // in the main scene

    // root
    osg::ref_ptr<osg::Group> nodeRoot = new osg::Group;
    nodeRoot->getOrCreateStateSet()->setMode(GL_BLEND,osg::StateAttribute::ON);

    // generate axes
    osg::ref_ptr<osg::Vec3Array> axesVertices = new osg::Vec3Array;
    osg::ref_ptr<osg::Vec4dArray> axesColors = new osg::Vec4dArray;
    osg::ref_ptr<osg::DrawElementsUInt> axesIndices = new osg::DrawElementsUInt(GL_LINES);
    axesVertices->push_back(osg::Vec3(-300,0,0));
    axesVertices->push_back(osg::Vec3(300,0,0));
    axesColors->push_back(osg::Vec4(0.25,0,0,1));
    axesColors->push_back(osg::Vec4(1,0,0,1));
    axesVertices->push_back(osg::Vec3(0,-200,0));
    axesVertices->push_back(osg::Vec3(0,200,0));
    axesColors->push_back(osg::Vec4(0,0.25,0,1));
    axesColors->push_back(osg::Vec4(0,1,0,1));
    axesVertices->push_back(osg::Vec3(0,0,-200));
    axesVertices->push_back(osg::Vec3(0,0,200));
    axesColors->push_back(osg::Vec4(0,0,0.25,1));
    axesColors->push_back(osg::Vec4(0,0,1,1));
    axesIndices->push_back(0); axesIndices->push_back(1);
    axesIndices->push_back(2); axesIndices->push_back(3);
    axesIndices->push_back(4); axesIndices->push_back(5);

    osg::ref_ptr<osg::Geometry> myAxes = new osg::Geometry;
    myAxes->setVertexArray(axesVertices);
    myAxes->addPrimitiveSet(axesIndices);
    myAxes->setColorArray(axesColors);
    myAxes->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

    osg::ref_ptr<osg::Geode> geodeAxes = new osg::Geode;
    geodeAxes->addDrawable(myAxes.get());
    nodeRoot->addChild(geodeAxes.get());

    // colors
    osg::ref_ptr<osg::Vec4Array> blue = new osg::Vec4Array;
    blue->push_back(osg::Vec4(0.5,0.5,0,1));

    osg::ref_ptr<osg::Vec4Array> yellow = new osg::Vec4Array;
    yellow->push_back(osg::Vec4(1,1,0,1));

    // bled func (important!)
    // http://www.machwerx.com/2009/02/11/glblendfunc/
    // http://www.andersriggelsen.dk/glblendfunc.php  <-- great site
    osg::ref_ptr<osg::BlendFunc> blendFunc = new osg::BlendFunc;
    blendFunc->setFunction(GL_ZERO,GL_ONE_MINUS_SRC_ALPHA);

    // square
    osg::ref_ptr<osg::Vec3dArray> sVerts = new osg::Vec3dArray(4);
    sVerts->at(0) = osg::Vec3d(-2,-2,0);
    sVerts->at(1) = osg::Vec3d(2,-2,0);
    sVerts->at(2) = osg::Vec3d(2,2,0);
    sVerts->at(3) = osg::Vec3d(-2,2,0);
    osg::ref_ptr<osg::Vec3dArray> sNorms = new osg::Vec3dArray(1);
    sNorms->at(0) = osg::Vec3d(0,0,1);
    osg::ref_ptr<osg::DrawElementsUInt> sIdxs =
            new osg::DrawElementsUInt(GL_TRIANGLES,6);
    sIdxs->at(0) = 0; sIdxs->at(1) = 1; sIdxs->at(2) = 2;
    sIdxs->at(3) = 0; sIdxs->at(4) = 2; sIdxs->at(5) = 3;
    osg::ref_ptr<osg::Geometry> geomSquare = new osg::Geometry;
    geomSquare->setVertexArray(sVerts.get());
    geomSquare->setNormalArray(sNorms.get());
    geomSquare->setNormalBinding(osg::Geometry::BIND_OVERALL);
    geomSquare->setColorArray(blue.get());
    geomSquare->setColorBinding(osg::Geometry::BIND_OVERALL);
    geomSquare->addPrimitiveSet(sIdxs.get());
    osg::ref_ptr<osg::Geode> geodeSquare = new osg::Geode;
    geodeSquare->addDrawable(geomSquare.get());

    // triangle
    osg::ref_ptr<osg::Vec3dArray> tVerts = new osg::Vec3dArray(3);
    tVerts->at(0) = osg::Vec3d(-1.5,-1.5,2);
    tVerts->at(1) = osg::Vec3d(1.5,-1.5,2);
    tVerts->at(2) = osg::Vec3d(0,1.5,2);
    osg::ref_ptr<osg::Vec3dArray> tNorms = new osg::Vec3dArray(1);
    tNorms->at(0) = osg::Vec3d(0,0,1);
    osg::ref_ptr<osg::DrawElementsUInt> tIdxs =
            new osg::DrawElementsUInt(GL_TRIANGLES,3);
    tIdxs->at(0) = 0; tIdxs->at(1) = 1; tIdxs->at(2) = 2;
    osg::ref_ptr<osg::Geometry> geomTri = new osg::Geometry;
    geomTri->setVertexArray(tVerts.get());
    geomTri->setNormalArray(tNorms.get());
    geomTri->setNormalBinding(osg::Geometry::BIND_OVERALL);
    geomTri->setColorArray(yellow.get());
    geomTri->setColorBinding(osg::Geometry::BIND_OVERALL);
    geomTri->addPrimitiveSet(tIdxs.get());
    osg::ref_ptr<osg::Geode> geodeTri = new osg::Geode;
    geodeTri->addDrawable(geomTri.get());
    geodeTri->getOrCreateStateSet()->setAttribute(blendFunc.get());

    // group node
    osg::ref_ptr<osg::Group> groupStuff = new osg::Group;
    groupStuff->addChild(geodeTri.get());
    groupStuff->addChild(geodeSquare.get());
    groupStuff->getOrCreateStateSet()->setMode(GL_BLEND,osg::StateAttribute::ON);
    groupStuff->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

    // texture quad
    osg::ref_ptr<osg::Texture2D> texRender = new osg::Texture2D;
    texRender->setTextureSize(500,500);
    texRender->setInternalFormat(GL_RGBA);
    texRender->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR);
    texRender->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);

    osg::ref_ptr<osg::Geometry> geomScreenQuad;
    geomScreenQuad = osg::createTexturedQuadGeometry(osg::Vec3(-250,-250,0),
                                                     osg::Vec3(500,0,0),
                                                     osg::Vec3(0,500,0));
    osg::ref_ptr<osg::Geode> geodeScreenQuad = new osg::Geode;
    geodeScreenQuad->addDrawable(geomScreenQuad.get());
    geodeScreenQuad->getOrCreateStateSet()->
            setTextureAttributeAndModes(0,texRender,osg::StateAttribute::ON);
    nodeRoot->addChild(geodeScreenQuad.get());

    // texture camera
    osg::ref_ptr<osg::Camera> texCamera = new osg::Camera;
    texCamera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    texCamera->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    texCamera->setViewport(0,0,500,500);
    texCamera->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
    texCamera->setRenderOrder(osg::Camera::PRE_RENDER);
    texCamera->attach(osg::Camera::COLOR_BUFFER,texRender);
    texCamera->setViewMatrixAsLookAt(osg::Vec3d(0,0,20),
                                     osg::Vec3d(0,0,0),
                                     osg::Vec3d(-1,0,0));
    texCamera->setProjectionMatrixAsPerspective(30,1,2,150);
    texCamera->setClearColor(osg::Vec4(0,0,0,0));
    texCamera->addChild(groupStuff.get());
    nodeRoot->addChild(texCamera.get());

    // add a viewport to the viewer and attach the scene graph.
    osgViewer::Viewer viewer;
    viewer.setThreadingModel(osgViewer::ViewerBase::SingleThreaded);
    viewer.getCamera()->setClearColor(osg::Vec4(0.1,0.1,0.1,1));
    viewer.setUpViewInWindow(100,100,640,360);
    viewer.setSceneData(nodeRoot.get());

    return viewer.run();
}
