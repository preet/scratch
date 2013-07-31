#include "helpers.hpp"

int main(int argc, char *argv[])
{
    // set debug severity
    //osg::setNotifyLevel(osg::DEBUG_INFO);

    // [base tiles]
    BuildBaseTileList(-180,-86,0,86,8,16,
                      g_LIST_BASE_VERTICES,
                      g_LIST_BASE_TILES);
    for(size_t i=0; i < g_LIST_BASE_TILES.size(); i++)   {
        g_LIST_BASE_TILES[i]->level = 4;
    }

    std::cout << "SZ LIST BASE TILES: "
              << CountBaseTiles() << std::endl;


    // [shaders]
    SetupShaders();

    // [lods]
    g_LIST_LOD_RANGES = BuildLODRanges();

    // [earth node]
    osg::ref_ptr<osg::Geode> gdEarth = BuildGdEarthFromCamera(NULL);

    // [camera node]
    osg::ref_ptr<osg::MatrixTransform> xfCameraFrustum
            = new osg::MatrixTransform; // empty
    xfCameraFrustum->setName("xfCameraFrustum");

    // [root group]
    osg::ref_ptr<osg::Group> gpRoot = new osg::Group;
    gpRoot->getOrCreateStateSet()->setMode(GL_BLEND,osg::StateAttribute::ON);

    gpRoot->addChild(gdEarth);
    gpRoot->addChild(xfCameraFrustum);


    // composite viewer
    osgViewer::CompositeViewer cviewer;
    cviewer.setThreadingModel(osgViewer::Viewer::SingleThreaded);

    {   // add view 0
        osgViewer::View * view = new osgViewer::View;
        cviewer.addView(view);

        view->setUpViewInWindow(1000,0,640,480);
        view->setSceneData(gpRoot.get());
        view->setCameraManipulator(new osgGA::TrackballManipulator);
    }

    {   // add view 1
        osgViewer::View * view = new osgViewer::View;
        cviewer.addView(view);

        view->setUpViewInWindow(1000,500,640,480);
        view->setSceneData(gpRoot.get());
        view->setCameraManipulator(new osgGA::TrackballManipulator);
    }

    osgViewer::Viewer::Windows windows;
    cviewer.getWindows(windows);
    for(osgViewer::Viewer::Windows::iterator itr = windows.begin();
        itr != windows.end();
        ++itr)
    {
        (*itr)->getState()->setUseModelViewAndProjectionUniforms(true);
        (*itr)->getState()->setUseVertexAttributeAliasing(true);
    }

    size_t nf = 30;
    size_t fc = 0;

    while(!cviewer.done())   {

        size_t numChildren =
                gpRoot->getNumChildren();

        // update earth node
        if(fc%nf == 0)   {
            for(size_t i=0; i < numChildren; i++)   {
                osg::Node * child = gpRoot->getChild(i);
                if(child->getName() == "gdEarth")   {
                    gpRoot->removeChild(child);
                    osg::ref_ptr<osg::Geode> earth =
                        BuildGdEarthFromCamera(cviewer.getView(0)->getCamera());
                    gpRoot->addChild(earth);
                    break;
                }
            }
        }



        // update camera frustum
        for(size_t i=0; i < numChildren; i++)   {
            osg::Node * child = gpRoot->getChild(i);
            if(child->getName() == "xfCameraFrustum")   {
                gpRoot->removeChild(child);
                osg::ref_ptr<osg::MatrixTransform> cameraFrustum =
                        BuildFrustumFromCamera(cviewer.getView(0)->getCamera());
                gpRoot->addChild(cameraFrustum);
                break;
            }
        }

        fc++;
        if(fc > 30)
        { fc = 1; }

//        std::cout << fc << "," << fc%nf << std::endl;

        cviewer.frame();
    }

    return 0;
}

//PointLLA lla(43,-79,10000);
//Vec3 eye = ConvLLAToECEF(lla);

//    // [poi node]
//    osg::Vec3d c(eye.x,eye.y,eye.z);
//    osg::ref_ptr<osg::Geode> gdPoi =
//            BuildOctahedron(osg::Vec3d(eye.x,eye.y,eye.z),
//                            osg::Vec4(1,0,0,0.5),
//                            1000);

    //    gpRoot->addChild(gdPoi);

