#include <QString>
#include <QFile>
#include <QTextStream>
#include <QDebug>

#include "openctm/openctm.h"

#include <osgViewer/Viewer>
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>
#include <osg/Program>
#include <osg/Shader>
#include <osg/Uniform>
#include <osg/CullFace>

QString readFileAsQString(const QString &myFilePath)
{
    QFile myFile(myFilePath);
    myFile.open(QIODevice::ReadOnly);

    QTextStream textStream(&myFile);
    return textStream.readAll();
}

float minX = 1e5; float maxX = -1e5;
float minY = 1e5; float maxY = -1e5;
float minZ = 1e5; float maxZ = -1e5;

class MyCallback : public osg::Drawable::ComputeBoundingBoxCallback
{
public:
   MyCallback() {}
    virtual osg::BoundingBox computeBound(const osg::Drawable& drawable) const
   {
      osg::BoundingBox box(minX, minY, minZ, maxX, maxY, maxZ);
      return box;
   }
};

int main(int argc, char *argv[])
{
    // set debug severity
//    osg::setNotifyLevel(osg::DEBUG_INFO);

    // build openctm mesh
    CTMcontext meshContext;
    CTMuint meshVertCount,meshTriCount;
    CTMuint const *meshIndices;
    CTMfloat const *meshVertices;
    CTMfloat const *meshNormals;
    meshContext = ctmNewContext(CTM_IMPORT);

    ctmLoad(meshContext,"models/dragon_100k.ctm");

    if(ctmGetError(meshContext) == CTM_NONE)   {
        // access the mesh data
        meshVertCount = ctmGetInteger(meshContext,CTM_VERTEX_COUNT);
        meshIndices = ctmGetIntegerArray(meshContext,CTM_INDICES);
        meshVertices = ctmGetFloatArray(meshContext,CTM_VERTICES);
        meshNormals = ctmGetFloatArray(meshContext,CTM_NORMALS);
        meshTriCount = ctmGetInteger(meshContext,CTM_TRIANGLE_COUNT);

        qDebug() << "Model has" << meshVertCount << "vertices";
    }
    else  {
        return -1;
    }

    // build openscenegraph geometry
    unsigned int vxIdx = 0;
    unsigned int nxIdx = 0;
    osg::ref_ptr<osg::Vec3Array> listVxArray = new osg::Vec3Array(meshVertCount);
    osg::ref_ptr<osg::Vec3Array> listNxArray = new osg::Vec3Array(meshVertCount);
    for(int i=0; i < listVxArray->size(); i++)  {
        osg::Vec3 vertex;
        vertex.x() = meshVertices[vxIdx]; vxIdx++;
        vertex.y() = meshVertices[vxIdx]; vxIdx++;
        vertex.z() = meshVertices[vxIdx]; vxIdx++;
        listVxArray->at(i) = vertex;

        osg::Vec3 normal;
        normal.x() = meshNormals[nxIdx]; nxIdx++;
        normal.y() = meshNormals[nxIdx]; nxIdx++;
        normal.z() = meshNormals[nxIdx]; nxIdx++;
        listNxArray->at(i) = normal;

        minX = std::min(minX,vertex.x());
        maxX = std::max(maxX,vertex.x());

        minY = std::min(minY,vertex.y());
        maxY = std::max(maxY,vertex.y());

        minZ = std::min(minZ,vertex.z());
        maxZ = std::max(maxZ,vertex.z());
    }

    osg::ref_ptr<osg::DrawElementsUInt> listIdxs =
            new osg::DrawElementsUInt(GL_TRIANGLES,meshTriCount*3);
    for(int i=0; i < listIdxs->size(); i++)  {
        listIdxs->at(i) = meshIndices[i];
    }

    osg::ref_ptr<osg::Geometry> geomMesh = new osg::Geometry;
    geomMesh->setVertexArray(listVxArray);
    geomMesh->setNormalArray(listNxArray);
    geomMesh->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);
    geomMesh->addPrimitiveSet(listIdxs.get());

    osg::ref_ptr<osg::Geode> geodeMesh = new osg::Geode;
    geodeMesh->addDrawable(geomMesh.get());


    osg::ref_ptr<osg::Group> groupRoot = new osg::Group;
    groupRoot->addChild(geodeMesh.get());

    // free openctm context
    ctmFreeContext(meshContext);

    // add shaders to openscenegraph
    osg::ref_ptr<osg::Program> shProgram = new osg::Program;
    shProgram->setName("DefaultProgram");

    // vertex shader
    QString vShader = readFileAsQString("shaders/model_vert.glsl");
    shProgram->addShader(new osg::Shader(osg::Shader::VERTEX,vShader.toStdString()));

    // fragment shader
    QString fShader = readFileAsQString("shaders/model_frag.glsl");
    shProgram->addShader(new osg::Shader(osg::Shader::FRAGMENT,fShader.toStdString()));

    // enable shader program
    osg::StateSet *ss = geodeMesh->getOrCreateStateSet();
    ss->setAttributeAndModes(shProgram,osg::StateAttribute::ON);
    ss->setMode(GL_BLEND, osg::StateAttribute::ON);
    ss->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    osg::ref_ptr<osg::CullFace> cullFace = new osg::CullFace();
    cullFace->setMode(osg::CullFace::FRONT_AND_BACK);
    ss->setAttributeAndModes(cullFace,osg::StateAttribute::OFF);

    // model bounds
    osg::BoundingBox modelBounds(minX,minY,minZ,maxX,maxY,maxZ);

    // viewer
    osgViewer::Viewer viewer;
    viewer.setThreadingModel(osgViewer::ViewerBase::SingleThreaded);
    viewer.setUpViewInWindow(100,100,800,480);
    viewer.setSceneData(groupRoot.get());

    osgViewer::Viewer::Windows windows;
    viewer.getWindows(windows);
    for(osgViewer::Viewer::Windows::iterator itr = windows.begin();
        itr != windows.end();
        ++itr)
    {
        (*itr)->getState()->setUseModelViewAndProjectionUniforms(true);
        (*itr)->getState()->setUseVertexAttributeAliasing(true);
    }

    return viewer.run();
}
