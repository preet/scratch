#include <QString>
#include <QFile>
#include <QTextStream>
#include <QDebug>

#include <osgViewer/Viewer>
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>
#include <osg/Program>
#include <osg/Shader>
#include <osg/Uniform>
#include <osg/CullFace>

#include <vector>

#define K_PI 3.141592653589


QString readFileAsQString(const QString &myFilePath)
{
    QFile myFile(myFilePath);
    myFile.open(QIODevice::ReadOnly);

    QTextStream textStream(&myFile);
    return textStream.readAll();
}

int main(int argc, char *argv[])
{
    // set debug severity
    //osg::setNotifyLevel(osg::DEBUG_INFO);

    // add shaders to openscenegraph
    osg::ref_ptr<osg::Program> shProgram = new osg::Program;
    shProgram->setName("DefaultProgram");

    // vertex shader
    QString vShader = readFileAsQString("shaders/model_vert.glsl");
    shProgram->addShader(new osg::Shader(osg::Shader::VERTEX,vShader.toStdString()));

    // fragment shader
    QString fShader = readFileAsQString("shaders/model_frag.glsl");
    shProgram->addShader(new osg::Shader(osg::Shader::FRAGMENT,fShader.toStdString()));

    // vertices,tex coords
    double numEdges = 7;
    osg::ref_ptr<osg::Vec3Array> listVx = new osg::Vec3Array;
    osg::ref_ptr<osg::Vec2Array> listTx = new osg::Vec2Array;
    listVx->push_back(osg::Vec3(0,0,0));        // center
    listTx->push_back(osg::Vec2(0,0));
    for(size_t i=0; i <= numEdges; i++)   {
        osg::Vec3d vx;
        vx.x() = cos((2*K_PI/numEdges)*i);
        vx.y() = sin((2*K_PI/numEdges)*i);
        qDebug() << K_PI/numEdges << "*" << i;
        vx.z() = 0;
        listVx->push_back(vx);
        osg::Vec2d tx;
        if(i%2 == 0)   {
            tx.x() = 1;
            tx.y() = 0;
        }
        else   {
            tx.x() = 0;
            tx.y() = 1;
        }
        listTx->push_back(tx);
    }
    // indices
    osg::ref_ptr<osg::DrawElementsUInt> listIx =
            new osg::DrawElementsUInt(GL_TRIANGLES);
    for(size_t i=2; i < listVx->size(); i++)   {
        listIx->push_back(0);
        listIx->push_back(i-1);
        listIx->push_back(i);
    }
    // [geometry]
    osg::ref_ptr<osg::Geometry> gmPoly = new osg::Geometry;
    gmPoly->setVertexArray(listVx);
    gmPoly->setTexCoordArray(0,listTx);
    gmPoly->addPrimitiveSet(listIx);

    // [geode]
    osg::ref_ptr<osg::Geode> gdPoly = new osg::Geode;
    osg::StateSet *ss = gdPoly->getOrCreateStateSet();
    ss->setAttributeAndModes(shProgram,osg::StateAttribute::ON);
    gdPoly->addDrawable(gmPoly);

    // [root]
    osg::ref_ptr<osg::Group> groupRoot = new osg::Group;
    ss = groupRoot->getOrCreateStateSet();
    ss->setMode(GL_BLEND,osg::StateAttribute::ON);
    groupRoot->addChild(gdPoly);

    // viewer
    osgViewer::Viewer viewer;
    viewer.setThreadingModel(osgViewer::ViewerBase::SingleThreaded);
    viewer.setUpViewInWindow(100,100,800,480);
    viewer.setSceneData(groupRoot.get());

    osgViewer::Viewer::Windows windows;
    viewer.getWindows(windows);
    for(osgViewer::Viewer::Windows::iterator itr = windows.begin();
        itr != windows.end(); ++itr)   {
        (*itr)->getState()->setUseModelViewAndProjectionUniforms(true);
        (*itr)->getState()->setUseVertexAttributeAliasing(true);
    }

    return viewer.run();
}
