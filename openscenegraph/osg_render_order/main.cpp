// sys includes
#include <math.h>
#include <vector>
#include <iostream>
#include <sys/time.h>

// osg includes
#include <osgText/Text>
#include <osgText/TextBase>
#include <osg/BoundingBox>
#include <osg/Drawable>
#include <osg/Geometry>
#include <osg/Material>
#include <osg/Node>
#include <osg/Matrixd>
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>
#include <osgViewer/Viewer>

#define K_PI 3.14159265


int main()
{
   // generate axes
   osg::ref_ptr<osg::Vec3Array> axesVertices = new osg::Vec3Array;
   osg::ref_ptr<osg::Vec4dArray> axesColors = new osg::Vec4dArray;
   osg::ref_ptr<osg::DrawElementsUInt> axesIndices = new osg::DrawElementsUInt(GL_LINES);
   axesVertices->push_back(osg::Vec3(-300,0,30));
   axesVertices->push_back(osg::Vec3(300,0,30));
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
   geodeAxes->getOrCreateStateSet()->setRenderBinDetails(19,"RenderBin");
//   geodeAxes->getOrCreateStateSet()->setMode(GL_LIGHTING,osg::StateAttribute::OFF);

   // simple ribbon
   osg::ref_ptr<osg::Vec3Array> ribbonVs = new osg::Vec3Array;
   ribbonVs->push_back(osg::Vec3(-100,-10,0));
   ribbonVs->push_back(osg::Vec3(100,-10,0));
   ribbonVs->push_back(osg::Vec3(100,10,0));
   ribbonVs->push_back(osg::Vec3(-100,10,0));

   osg::ref_ptr<osg::Vec3Array> ribbonNs = new osg::Vec3Array;
   ribbonNs->push_back(osg::Vec3(0,0,1));

   osg::ref_ptr<osg::DrawElementsUInt> ribbonIs = new osg::DrawElementsUInt(GL_TRIANGLES);
   ribbonIs->push_back(0); ribbonIs->push_back(1); ribbonIs->push_back(2);
   ribbonIs->push_back(0); ribbonIs->push_back(2); ribbonIs->push_back(3);

   osg::ref_ptr<osg::Geometry> ribbonGeom = new osg::Geometry;
   ribbonGeom->setVertexArray(ribbonVs.get());
   ribbonGeom->setNormalArray(ribbonNs.get());
   ribbonGeom->setNormalBinding(osg::Geometry::BIND_OVERALL);
   ribbonGeom->addPrimitiveSet(ribbonIs.get());

   // mat 1
   unsigned int layer1 = 11;
   osg::ref_ptr<osg::Material> ribbonMat1 = new osg::Material;
   ribbonMat1->setDiffuse(osg::Material::FRONT_AND_BACK,osg::Vec4(0,1,1,0.5));

   // mat 2
   unsigned int layer2 = 12;
   osg::ref_ptr<osg::Material> ribbonMat2 = new osg::Material;
   ribbonMat2->setDiffuse(osg::Material::FRONT_AND_BACK,osg::Vec4(1,0,1,0.5));

   // mat 3
   unsigned int layer3 = 13;
   osg::ref_ptr<osg::Material> ribbonMat3 = new osg::Material;
   ribbonMat3->setDiffuse(osg::Material::FRONT_AND_BACK,osg::Vec4(1,1,0,0.5));

   osg::ref_ptr<osg::Geode> geodeRibbon = new osg::Geode;
   geodeRibbon->addDrawable(ribbonGeom.get());

   osg::Matrixd myXform;

   // to avoid some rendering issues, we want to force building geometry
   // to render *after* surface level area geometry

   // setRenderingHint(osg::StateSet::TRANSPARENT_BIN) is equivalent to
   // mode: USE_RENDERBIN_DETAILS
   // num: 10 // (higher number renders AFTER lower number)
   // name: "DepthSortedBin"  // for geometry with transparency

   // setRenderingHint(osg::StateSet::OPAQUE_BIN) is equivalent to
   // mode: USE_RENDERBIN_DETAILS
   // num: 0 // (higher number renders AFTER lower number)
   // name: "RenderBin" // bin used for opaque geometry

   osg::ref_ptr<osg::MatrixTransform> xformRibbon1 = new osg::MatrixTransform;
   myXform = osg::Matrix::rotate(0*K_PI/180.0,osg::Vec3(0,0,1));
   myXform.setTrans(osg::Vec3(0,0,10));
   xformRibbon1->setMatrix(myXform);
   xformRibbon1->getOrCreateStateSet()->setAttribute(ribbonMat1.get());
   xformRibbon1->getOrCreateStateSet()->setRenderBinDetails(layer3,"RenderBin");
   xformRibbon1->addChild(geodeRibbon.get());

   osg::ref_ptr<osg::MatrixTransform> xformRibbon2 = new osg::MatrixTransform;
   myXform = osg::Matrix::rotate(120*K_PI/180.0,osg::Vec3(0,0,1));
   myXform.setTrans(osg::Vec3(0,0,0));
   xformRibbon2->setMatrix(myXform);
   xformRibbon2->getOrCreateStateSet()->setAttribute(ribbonMat2.get());
   xformRibbon2->getOrCreateStateSet()->setRenderBinDetails(layer2,"RenderBin");
   xformRibbon2->addChild(geodeRibbon.get());

   osg::ref_ptr<osg::MatrixTransform> xformRibbon3 = new osg::MatrixTransform;
   myXform = osg::Matrix::rotate(-120*K_PI/180.0,osg::Vec3(0,0,1));
   myXform.setTrans(osg::Vec3(0,0,-10));
   xformRibbon3->setMatrix(myXform);
   xformRibbon3->getOrCreateStateSet()->setAttribute(ribbonMat3.get());
   xformRibbon3->getOrCreateStateSet()->setRenderBinDetails(layer1,"RenderBin");
   xformRibbon3->addChild(geodeRibbon.get());

   osg::ref_ptr<osg::MatrixTransform> xformRibbon4 =
      dynamic_cast<osg::MatrixTransform*>(xformRibbon1->clone(osg::CopyOp::DEEP_COPY_ALL));
   myXform = xformRibbon4->getMatrix();
   myXform.setTrans(220,0,10);
   xformRibbon4->setMatrix(myXform);
   xformRibbon4->getOrCreateStateSet()->setRenderBinDetails(layer1,"RenderBin");
   xformRibbon4->getOrCreateStateSet()->setMode(GL_DEPTH_TEST,osg::StateAttribute::OFF);

   osg::ref_ptr<osg::MatrixTransform> xformRibbon5 =
      dynamic_cast<osg::MatrixTransform*>(xformRibbon2->clone(osg::CopyOp::DEEP_COPY_ALL));
   myXform = xformRibbon5->getMatrix();
   myXform.setTrans(220,0,0);
   xformRibbon5->setMatrix(myXform);
   xformRibbon5->getOrCreateStateSet()->setRenderBinDetails(layer2,"RenderBin");
   xformRibbon5->getOrCreateStateSet()->setMode(GL_DEPTH_TEST,osg::StateAttribute::OFF);

   osg::ref_ptr<osg::MatrixTransform> xformRibbon6 =
      dynamic_cast<osg::MatrixTransform*>(xformRibbon3->clone(osg::CopyOp::DEEP_COPY_ALL));
   myXform = xformRibbon6->getMatrix();
   myXform.setTrans(220,0,-10);
   xformRibbon6->setMatrix(myXform);
   xformRibbon6->getOrCreateStateSet()->setRenderBinDetails(layer3,"RenderBin");
   xformRibbon6->getOrCreateStateSet()->setMode(GL_DEPTH_TEST,osg::StateAttribute::OFF);

   // root
   osg::ref_ptr<osg::Group> nodeRoot = new osg::Group;
   nodeRoot->getOrCreateStateSet()->setMode(GL_BLEND,osg::StateAttribute::ON);
   nodeRoot->addChild(geodeAxes.get());

   nodeRoot->addChild(xformRibbon1.get());
   nodeRoot->addChild(xformRibbon2.get());
   nodeRoot->addChild(xformRibbon3.get());

   nodeRoot->addChild(xformRibbon4.get());
   nodeRoot->addChild(xformRibbon5.get());
   nodeRoot->addChild(xformRibbon6.get());

   // setup viewer
   osgViewer::Viewer viewer;
   viewer.setThreadingModel(osgViewer::ViewerBase::SingleThreaded);
   viewer.getCamera()->setClearColor(osg::Vec4(0.1,0.1,0.1,1));
   viewer.setUpViewInWindow(100,100,800,480);
   viewer.setSceneData(nodeRoot.get());
   return viewer.run();
}
