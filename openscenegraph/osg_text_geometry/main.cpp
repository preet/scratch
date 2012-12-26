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
//    osg::setNotifyLevel(osg::INFO);

    // text shaders
    osg::ref_ptr<osg::Program> shProgram = new osg::Program;
    shProgram->setName("TextShader");

    std::string vShader = std::string("#version 120\n") +
            readFileAsString("NoShading_vert.glsl");
    shProgram->addShader(new osg::Shader(osg::Shader::VERTEX,vShader));

    std::string fShader = std::string("#version 120\n") +
            readFileAsString("NoShading_frag.glsl");
    shProgram->addShader(new osg::Shader(osg::Shader::FRAGMENT,fShader));


    // stuff
    std::string strmessage("Castles built on sand");
    osgText::String message("Castles built on sand");

    osg::ref_ptr<osgText::Font> font = osgText::readFontFile("DroidSans-Bold.ttf");
    osgText::FontResolution fontResolution(32,32);

    osg::ref_ptr<osg::Geode> gdText = new osg::Geode;

    osgText::Glyph * glyph;
    for(size_t i=0; i < message.size(); i++)
    {
        glyph = font->getGlyph(fontResolution,message.at(i));
        osg::Vec2 minTexCoord = glyph->getMinTexCoord();        // bl
        osg::Vec2 maxTexCoord = glyph->getMaxTexCoord();        // tr
        std::cout << " "  << strmessage[i] << ": "
                  << "Min Coords: " << minTexCoord.x() << "," << minTexCoord.y() << " | "
                  << "Max Coords: " << maxTexCoord.x() << "," << maxTexCoord.y() << "\n";

        osg::ref_ptr<osg::Vec3Array> listVx = new osg::Vec3Array;
        osg::Vec3 vecCenter(i*2.0,i*2.0,0);
        listVx->push_back(osg::Vec3(-1,-1,0)+vecCenter);        // bl
        listVx->push_back(osg::Vec3(1,-1,0)+vecCenter);         // br
        listVx->push_back(osg::Vec3(1,1,0)+vecCenter);          // tr
        listVx->push_back(osg::Vec3(-1,1,0)+vecCenter);         // tl

        osg::ref_ptr<osg::Vec3Array> listNx = new osg::Vec3Array;
        listNx->push_back(osg::Vec3(0,0,1));
        listNx->push_back(osg::Vec3(0,0,1));
        listNx->push_back(osg::Vec3(0,0,1));
        listNx->push_back(osg::Vec3(0,0,1));

        osg::ref_ptr<osg::Vec2Array> listTx = new osg::Vec2Array;
        listTx->push_back(minTexCoord);                                     // bl
        listTx->push_back(osg::Vec2(maxTexCoord.x(),minTexCoord.y()));      // br
        listTx->push_back(maxTexCoord);                                     // tr
        listTx->push_back(osg::Vec2(minTexCoord.x(),maxTexCoord.y()));      // tl

        osg::ref_ptr<osg::DrawElementsUByte> listIx =
                new osg::DrawElementsUByte(GL_TRIANGLES);
        listIx->push_back(0);
        listIx->push_back(1);
        listIx->push_back(2);
        listIx->push_back(0);
        listIx->push_back(2);
        listIx->push_back(3);

        osg::ref_ptr<osg::Geometry> gmChar = new osg::Geometry;
        gmChar->setVertexArray(listVx);
        gmChar->setNormalArray(listNx);
        gmChar->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);
        gmChar->setTexCoordArray(0,listTx);
        gmChar->addPrimitiveSet(listIx);

        gdText->addDrawable(gmChar);
    }

    osg::ref_ptr<osg::Uniform> textColor = new osg::Uniform("MaterialColor",osg::Vec4(0,1,0,1));
    osg::ref_ptr<osg::Uniform> textTexture = new osg::Uniform("GlyphTexture",1);
    osg::StateSet * ss = gdText->getOrCreateStateSet();
    ss->addUniform(textColor);
    ss->addUniform(textTexture);

    std::cout << "num listGlyphTextures: "
              << font->getGlyphTextureList().size() << "\n";

    if(font->getGlyphTextureList().size() > 0)   {
        osg::ref_ptr<osgText::GlyphTexture> glyphTex = font->getGlyphTextureList().at(0);
        ss->setTextureAttributeAndModes(1,glyphTex);
    }

    ss->setAttributeAndModes(shProgram,osg::StateAttribute::ON);

    osg::ref_ptr<osg::Group> nodeRoot = new osg::Group;
    nodeRoot->addChild(gdText);

    // setup viewer
    osgViewer::Viewer viewer;
    viewer.setThreadingModel(osgViewer::ViewerBase::SingleThreaded);
    viewer.setUpViewInWindow(100,100,800,480);
    viewer.setSceneData(nodeRoot.get());
    return viewer.run();
}
