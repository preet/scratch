 // sys includes
#include <math.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <sys/time.h>

// osg includes
#include <osgText/Text>
#include <osgText/TextBase>
#include <osg/BoundingBox>
#include <osg/Drawable>
#include <osg/Geometry>
#include <osg/Node>
#include <osg/Matrixd>
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>
#include <osgViewer/Viewer>

std::string readFileAsString(std::string const &fileName)
{
    std::ifstream ifs(fileName.c_str());
    std::string content( (std::istreambuf_iterator<char>(ifs) ),
                         (std::istreambuf_iterator<char>()    ) );
    return content;
}

int main()
{
    osg::setNotifyLevel(osg::INFO);

    // text geometry
    std::string textStr("Hello World");
    osg::ref_ptr<osgText::Text> myText = new osgText::Text;
    myText->setFont("DroidSans-Bold.ttf");
    myText->setCharacterSize(20.0f);
    myText->setText(textStr);

    // text node
    osg::ref_ptr<osg::Geode> textGeode = new osg::Geode;
    textGeode->addDrawable(myText);

    // text shaders
    osg::ref_ptr<osg::Program> shProgram = new osg::Program;
    shProgram->setName("TextShader");

    std::string vShader = std::string("#version 120\n") +
            readFileAsString("shaders/NoShading_vert.glsl");
    shProgram->addShader(new osg::Shader(osg::Shader::VERTEX,vShader));

    std::string fShader = std::string("#version 120\n") +
            readFileAsString("shaders/NoShading_frag.glsl");
    shProgram->addShader(new osg::Shader(osg::Shader::FRAGMENT,fShader));

    osg::ref_ptr<osg::Uniform> textColor = new osg::Uniform("MaterialColor",osg::Vec4(0,1,1,0.5));
    osg::ref_ptr<osg::Uniform> textTexture = new osg::Uniform("GlyphTexture",0);

    osg::StateSet *ss = textGeode->getOrCreateStateSet();
    ss->addUniform(textColor);
    ss->addUniform(textTexture);
    ss->setAttributeAndModes(shProgram,osg::StateAttribute::ON);

    osg::ref_ptr<osg::Group> nodeRoot = new osg::Group;
    nodeRoot->addChild(textGeode);

    // setup viewer
    osgViewer::Viewer viewer;
    viewer.setThreadingModel(osgViewer::ViewerBase::SingleThreaded);
    viewer.setUpViewInWindow(100,100,800,480);
    viewer.setSceneData(nodeRoot.get());
    return viewer.run();
}
