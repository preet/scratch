#include <osgViewer/Viewer>
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>
#include <osg/AutoTransform>
#include <osg/PolygonOffset>
#include <osg/Program>
#include <osg/Shader>
#include <osg/Uniform>
#include <osgText/Text>
#include <osg/CullFace>

#include <vector>
#include <iostream>
#include <fstream>

osg::Camera const * g_cam;
osg::Matrixf g_xfBillboard;

std::string simple_vert =
"#version 120\n"
"varying vec4 VertexColor;\n"
"void main()\n"
"{\n"
"    VertexColor = gl_Color;\n"
"    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;\n"
"}\n";

std::string simple_frag =
"#version 120\n"
"varying vec4 VertexColor;\n"
"void main()\n"
"{\n"
"    gl_FragColor = VertexColor;\n"
"}\n";

std::string bb_vert =
"#version 120\n"
"attribute vec3 ModelPosition;\n"
"uniform mat4 BillboardRotate;\n"
"varying vec4 VertexColor;\n"
"void main()\n"
"{\n"
"    vec4 v_modelPosition = vec4(ModelPosition.xyz,1.0);\n"
"    vec4 v_xlateModelToWorld = gl_Vertex - v_modelPosition;\n"
"    vec4 v_alignedPosition = BillboardRotate * v_modelPosition;\n"
"    vec4 v_worldPosition = v_alignedPosition + v_xlateModelToWorld;\n"
"    VertexColor = gl_Color;\n"
"    gl_Position = gl_ModelViewProjectionMatrix * v_worldPosition;\n"
"}\n";

std::string bb_frag =
"#version 120\n"
"varying vec4 VertexColor;\n"
"void main()\n"
"{\n"
"    gl_FragColor = VertexColor;\n"
"}\n";



class BillboardXFCallback : public osg::NodeCallback
{
public:
    virtual void operator()
    (osg::Node* node, osg::NodeVisitor* nv)
    {
        osg::Vec3d camEye,camVPt,camUp;
        osg::Vec3d camLookAt,camSide;
        g_cam->getViewMatrixAsLookAt(camEye,camVPt,camUp,1.0f);

        camUp.normalize();
        camLookAt = camEye-camVPt;
        camLookAt.normalize();
        camSide = camUp^camLookAt;
        camSide.normalize();

        // x axis
        g_xfBillboard(0,0) = camSide.x();
        g_xfBillboard(0,1) = camSide.y();
        g_xfBillboard(0,2) = camSide.z();
        g_xfBillboard(0,3) = 0;

        // y axis
        g_xfBillboard(1,0) = camUp.x();
        g_xfBillboard(1,1) = camUp.y();
        g_xfBillboard(1,2) = camUp.z();
        g_xfBillboard(1,3) = 0;

        // z axis
        g_xfBillboard(2,0) = camLookAt.x();
        g_xfBillboard(2,1) = camLookAt.y();
        g_xfBillboard(2,2) = camLookAt.z();
        g_xfBillboard(2,3) = 0;

        // continue traversing so osg processes
        // other nodes with callbacks as well
        traverse(node,nv);
    }
};

class BillboardUniformCallback : public osg::Uniform::Callback
{
public:
    virtual void operator()
    (osg::Uniform * uniform, osg::NodeVisitor * nv)
    {   // update uniform
        uniform->set(g_xfBillboard);
    }
};

std::string readFileAsString(std::string const &fileName)
{
    std::ifstream ifs(fileName.c_str());
    std::string content( (std::istreambuf_iterator<char>(ifs) ),
                         (std::istreambuf_iterator<char>()    ) );
    return content;
}

int main(int argc, char *argv[])
{
    // set debug severity
//    osg::setNotifyLevel(osg::DEBUG_INFO);

    // shaders
    std::string vShader,fShader;
    osg::ref_ptr<osg::Program> simpleShader = new osg::Program;
    simpleShader->setName("SimpleShader");
    vShader = simple_vert;
    fShader = simple_frag;
    simpleShader->addShader(new osg::Shader(osg::Shader::VERTEX,vShader));
    simpleShader->addShader(new osg::Shader(osg::Shader::FRAGMENT,fShader));

    osg::ref_ptr<osg::Program> bbShader = new osg::Program;
    bbShader->setName("BillboardShader");
    vShader = readFileAsString("shaders/bb_vert.glsl");
    fShader = readFileAsString("shaders/bb_frag.glsl");
    bbShader->addShader(new osg::Shader(osg::Shader::VERTEX,vShader));
    bbShader->addShader(new osg::Shader(osg::Shader::FRAGMENT,fShader));
    bbShader->addBindAttribLocation("ModelPosition",6);

    osg::ref_ptr<osg::Program> textShader = new osg::Program;
    textShader->setName("TextShader");
    vShader = readFileAsString("shaders/text_vert.glsl");
    fShader = readFileAsString("shaders/text_frag.glsl");
    textShader->addShader(new osg::Shader(osg::Shader::VERTEX,vShader));
    textShader->addShader(new osg::Shader(osg::Shader::FRAGMENT,fShader));

    osg::StateSet * ss;
    osg::Vec3d frontShift(1.0,0,0);
    osg::Vec3d bl(-0.1,-0.1,0);
    osg::Vec3d br(0.1,-0.1,0);
    osg::Vec3d tr(0.1,0.1,0);
    osg::Vec3d tl(-0.1,0.1,0);

    // setup
    // + grRoot
    //   + gdBack       (uses billboard shader)
    //     + gmBack
    //   + xfFront      (uses autotransform)
    //     + gdFront
    //       + gmFront


    // back geometry
    osg::ref_ptr<osg::Vec3Array> listBxBack = new osg::Vec3Array(4);
    listBxBack->at(0) = bl;// - osg::Vec3d(0,0,0.01);
    listBxBack->at(1) = br;// - osg::Vec3d(0,0,0.01);
    listBxBack->at(2) = tr;// - osg::Vec3d(0,0,0.01);
    listBxBack->at(3) = tl;// - osg::Vec3d(0,0,0.01);

    osg::ref_ptr<osg::Vec3dArray> listVxBack = new osg::Vec3dArray(4);
    listVxBack->at(0) = bl + frontShift*0.5;
    listVxBack->at(1) = br + frontShift*0.5;
    listVxBack->at(2) = tr + frontShift*0.5;
    listVxBack->at(3) = tl + frontShift*0.5;

    osg::ref_ptr<osg::Vec4Array> listCxBack = new osg::Vec4Array(4);
    osg::Vec4 backColor(0,0,0,1);
    listCxBack->at(0) = backColor;
    listCxBack->at(1) = backColor;
    listCxBack->at(2) = backColor;
    listCxBack->at(3) = backColor;

    osg::ref_ptr<osg::DrawElementsUInt> listIx =
            new osg::DrawElementsUInt(GL_TRIANGLES,6);
    listIx->at(0) = 0;
    listIx->at(1) = 1;
    listIx->at(2) = 2;
    listIx->at(3) = 0;
    listIx->at(4) = 2;
    listIx->at(5) = 3;

    osg::ref_ptr<osg::Geometry> gmBack = new osg::Geometry;
    gmBack->setVertexArray(listVxBack);
    gmBack->setColorArray(listCxBack);
    gmBack->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
    gmBack->setVertexAttribArray(6,listBxBack);
    gmBack->setVertexAttribBinding(6,osg::Geometry::BIND_PER_VERTEX);
    gmBack->addPrimitiveSet(listIx);

    // back geode
    osg::ref_ptr<osg::Geode> gdBack = new osg::Geode;
    ss = gdBack->getOrCreateStateSet();
    ss->setAttributeAndModes(bbShader,osg::StateAttribute::ON);

    osg::ref_ptr<osg::Uniform> uBillboard =
        new osg::Uniform("BillboardRotate",osg::Matrixd::identity());

    osg::ref_ptr<BillboardUniformCallback> bbUniCallback =
            new BillboardUniformCallback();

    uBillboard->setUpdateCallback(bbUniCallback);
    ss->addUniform(uBillboard);
    gdBack->addDrawable(gmBack);

    osg::ref_ptr<osg::PolygonOffset> polyOffset = new osg::PolygonOffset;
    polyOffset->setFactor(1.0f);
    polyOffset->setUnits(1.0f);
    ss = gmBack->getOrCreateStateSet();
    ss->setAttributeAndModes(polyOffset);

    // text geometry
    osg::ref_ptr<osgText::Text> gmText = new osgText::Text;
    gmText->setFont("/home/preet/Dev/scratch/openscenegraph/osg_text_geometry/DroidSans-Bold.ttf");
    gmText->setAlignment(osgText::Text::CENTER_CENTER);
    gmText->setCharacterSize(0.1);
    gmText->setText("Hello World");
    gmText->setPosition(osg::Vec3(0,0,0));

    osg::Vec4 fontColor(1,0,1,1);
    osg::ref_ptr<osg::Uniform> uFontColor = new osg::Uniform("Color",fontColor);

    // text geode
    osg::ref_ptr<osg::Geode> gdText = new osg::Geode;
    ss = gdText->getOrCreateStateSet();
    ss->addUniform(uFontColor);
    ss->setAttributeAndModes(textShader);
    gdText->addDrawable(gmText);

    // fake text geometry
    osg::ref_ptr<osg::Vec3dArray> listVxText = new osg::Vec3dArray;
    osg::ref_ptr<osg::Vec4Array> listCxText = new osg::Vec4Array;
    osg::ref_ptr<osg::Vec3Array> listBxText = new osg::Vec3Array;
    osg::ref_ptr<osg::Vec2Array> listTxText = new osg::Vec2Array;
    osg::ref_ptr<osg::DrawElementsUInt> listIxText = new osg::DrawElementsUInt(GL_TRIANGLES);
    osgText::Text::TextureGlyphQuadMap mapGlyphsByTexture =
            gmText->getTextureGlyphQuadMap();

    // expect only one texture!
    osgText::Text::GlyphQuads glyphQuads;
    glyphQuads = mapGlyphsByTexture.begin()->second;
    std::cout << "num glyphs " << glyphQuads.getGlyphs().size() << std::endl;
    std::cout << "num coords " << glyphQuads.getCoords().size() << std::endl;
    for(size_t i=0; i < glyphQuads.getCoords().size(); i++)   {
        listVxText->push_back(osg::Vec3d(glyphQuads.getCoords()[i].x(),
                                         glyphQuads.getCoords()[i].y(),0) +
                              frontShift*0.5);

        listBxText->push_back(osg::Vec3(glyphQuads.getCoords()[i].x(),
                                         glyphQuads.getCoords()[i].y(),0));

        listTxText->push_back(osg::Vec2(glyphQuads.getTexCoords()[i].x(),
                                        glyphQuads.getTexCoords()[i].y()));

        std::cout << "glyphc " << glyphQuads.getCoords()[i].x() << std::endl;
        listCxText->push_back(fontColor);
    }
    for(size_t i=0; i < glyphQuads.getGlyphs().size(); i++)   {
        size_t numIx = listIxText->size();

        listIxText->push_back(0 + numIx);
        listIxText->push_back(1 + numIx);
        listIxText->push_back(2 + numIx);

        listIxText->push_back(0 + numIx);
        listIxText->push_back(2 + numIx);
        listIxText->push_back(3 + numIx);
    }

    std::cout << "listvx sz " << listVxText->size() << std::endl;
    std::cout << "listix sz " << listIxText->size() << std::endl;

    osg::ref_ptr<osg::Geometry> gmShText = new osg::Geometry;
    gmShText->setVertexArray(listVxText);
    gmShText->setColorArray(listCxText);
    gmShText->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
    gmShText->setTexCoordArray(0,listTxText);
    gmShText->setVertexAttribArray(6,listBxText);
    gmShText->setVertexAttribBinding(6,osg::Geometry::BIND_PER_VERTEX);
    gmShText->addPrimitiveSet(new osg::DrawArrays(GL_QUADS,0,listVxText->size()));

    // fake text geode
    osg::ref_ptr<osg::Geode> gdShText = new osg::Geode;
    ss = gdShText->getOrCreateStateSet();
    ss->setAttributeAndModes(bbShader,osg::StateAttribute::ON);
    ss->addUniform(uBillboard);
    ss->setTextureAttributeAndModes(0,mapGlyphsByTexture.begin()->first);
    gdShText->addDrawable(gmShText);

    // front geometry
    osg::ref_ptr<osg::Vec3dArray> listVxFront = new osg::Vec3dArray(4);
    listVxFront->at(0) = bl*0.5 + frontShift*0.5;
    listVxFront->at(1) = br*0.5 + frontShift*0.5;
    listVxFront->at(2) = tr*0.5 + frontShift*0.5;
    listVxFront->at(3) = tl*0.5 + frontShift*0.5;

    osg::ref_ptr<osg::Vec3Array> listBxFront = new osg::Vec3Array(4);
    listBxFront->at(0) = bl*0.5;
    listBxFront->at(1) = br*0.5;
    listBxFront->at(2) = tr*0.5;
    listBxFront->at(3) = tl*0.5;

    osg::ref_ptr<osg::Vec4Array> listCxFront = new osg::Vec4Array(4);
    osg::Vec4 frontColor(0.8,0.5,0,1);
    listCxFront->at(0) = frontColor;
    listCxFront->at(1) = frontColor;
    listCxFront->at(2) = frontColor;
    listCxFront->at(3) = frontColor;

    osg::ref_ptr<osg::Geometry> gmFront = new osg::Geometry;
    gmFront->setVertexArray(listVxFront);
    gmFront->setColorArray(listCxFront);
    gmFront->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
    gmFront->setVertexAttribArray(6,listBxFront);
    gmFront->setVertexAttribBinding(6,osg::Geometry::BIND_PER_VERTEX);
    gmFront->addPrimitiveSet(listIx);

    // front geode
    osg::ref_ptr<osg::Geode> gdFront = new osg::Geode;
    ss = gdFront->getOrCreateStateSet();
    ss->setAttributeAndModes(simpleShader,osg::StateAttribute::ON);
    gdFront->addDrawable(gmFront);

    // front transform
    osg::ref_ptr<osg::AutoTransform> xfFront = new osg::AutoTransform;
    xfFront->setAutoRotateMode(osg::AutoTransform::ROTATE_TO_CAMERA);
    xfFront->setPosition(frontShift*0.5);
    xfFront->addChild(gdFront);


    // cube for reference
    osg::ref_ptr<osg::ShapeDrawable> gmCubeO =
            new osg::ShapeDrawable(new osg::Box(osg::Vec3(0,0,0),0.1f));
    gmCubeO->setColor(osg::Vec4(1,1,1,1));

    osg::ref_ptr<osg::ShapeDrawable> gmCubeX =
            new osg::ShapeDrawable(new osg::Box(osg::Vec3(1,0,0),0.1f));
    gmCubeX->setColor(osg::Vec4(1,0,0,1));

    osg::ref_ptr<osg::ShapeDrawable> gmCubeY =
            new osg::ShapeDrawable(new osg::Box(osg::Vec3(0,1,0),0.1f));
    gmCubeY->setColor(osg::Vec4(0,1,0,1));

    osg::ref_ptr<osg::ShapeDrawable> gmCubeZ =
            new osg::ShapeDrawable(new osg::Box(osg::Vec3(0,0,1),0.1f));
    gmCubeZ->setColor(osg::Vec4(0,0,1,1));

    // cube geode
    osg::ref_ptr<osg::Geode> gdCube = new osg::Geode;
    ss = gdCube->getOrCreateStateSet();
    ss->setAttributeAndModes(simpleShader,osg::StateAttribute::ON);
    gdCube->addDrawable(gmCubeO);
    gdCube->addDrawable(gmCubeX);
    gdCube->addDrawable(gmCubeY);
    gdCube->addDrawable(gmCubeZ);

    gdBack->addDrawable(gmFront);

    // [root]
    osg::ref_ptr<osg::Group> groupRoot = new osg::Group;
    ss = groupRoot->getOrCreateStateSet();
    ss->setMode(GL_BLEND,osg::StateAttribute::ON);
//    groupRoot->addChild(gdCube);
    groupRoot->addChild(gdBack);
//    groupRoot->addChild(gdText);
    groupRoot->addChild(gdShText);
//    groupRoot->addChild(xfFront);

    osg::ref_ptr<BillboardXFCallback> bbXFCallback = new BillboardXFCallback;
    groupRoot->setUpdateCallback(bbXFCallback);

    // viewer
    osgViewer::Viewer viewer;
    viewer.setThreadingModel(osgViewer::ViewerBase::SingleThreaded);
    viewer.setUpViewInWindow(100,100,800,480);
    viewer.setSceneData(groupRoot.get());
//    viewer.setRunMaxFrameRate(120.0);
    g_cam = viewer.getCamera();

    osgViewer::Viewer::Windows windows;
    viewer.getWindows(windows);
    for(osgViewer::Viewer::Windows::iterator itr = windows.begin();
        itr != windows.end(); ++itr)   {
        (*itr)->getState()->setUseModelViewAndProjectionUniforms(true);
        (*itr)->getState()->setUseVertexAttributeAliasing(true);
    }

    return viewer.run();
}
