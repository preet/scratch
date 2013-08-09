/*   

   Copyright 2012 Preet Desai

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
   * 
   */

#include "helpers3.hpp"

int main(int argc, char *argv[])
{
    // set debug severity
    //osg::setNotifyLevel(osg::DEBUG_INFO);

    std::vector<PointLLA> listPLLA;
    listPLLA.push_back(PointLLA(75,-135));
    listPLLA.push_back(PointLLA(65,-90));
    listPLLA.push_back(PointLLA(-43,45));
    listPLLA.push_back(PointLLA(-63,90));

    PointLLA camLLA(PointLLA(77,90,25000));
    Vec3 ceye = ConvLLAToECEF(camLLA);

    std::vector<GeoBounds> listBounds;
    CalcLonRangeFromLLAPoly(camLLA,listPLLA,listBounds);

    for(size_t i=0; i < listBounds.size(); i++)   {
        std::cout << i
                  << ": minLat: " << listBounds[i].minLat
                  << ", maxLat: " << listBounds[i].maxLat
                  << ", minLon: " << listBounds[i].minLon
                  << ", maxLon: " << listBounds[i].maxLon
                  << std::endl;
    }

    return 0;

    // [base tiles]
    BuildBaseTileList(-180,-86,180,86,16,16,
                      g_LIST_BASE_VERTICES,
                      g_LIST_BASE_TILES);
    for(size_t i=0; i < g_LIST_BASE_TILES.size(); i++)   {
        g_LIST_BASE_TILES[i]->level = 4;
    }

    // [shaders]
    SetupShaders();

    // [lods]
    g_LIST_LOD_RANGES = BuildLODRanges();

    // [earth node]
    osg::ref_ptr<osg::Geode> gdEarth = BuildGdEarthFromCamera(NULL);

    // [camera node]
    osg::ref_ptr<osg::MatrixTransform> xfCameraFrustum
            = new osg::MatrixTransform; // empty
    xfCameraFrustum->setName("gpCamera");

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

        view->setUpViewInWindow(0,0,640,360);
        view->setSceneData(gpRoot.get());
        view->setCameraManipulator(new osgGA::TrackballManipulator);
        view->getCamera()->setClearColor(osg::Vec4(0.1,0.1,0.1,1.0));
    }

    {   // add view 1
        osgViewer::View * view = new osgViewer::View;
        cviewer.addView(view);

        view->setUpViewInWindow(0,425,640,360);
        view->setSceneData(gpRoot.get());
        view->setCameraManipulator(new osgGA::TrackballManipulator);
        view->getCamera()->setClearColor(osg::Vec4(0.1,0.1,0.1,1.0));
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

    while(!cviewer.done())   {

        size_t numChildren =
                gpRoot->getNumChildren();

//        // update earth node
//        if(fc%nf == 0)   {
//            for(size_t i=0; i < numChildren; i++)   {
//                osg::Node * child = gpRoot->getChild(i);
//                if(child->getName() == "gdEarth")   {
//                    gpRoot->removeChild(child);
//                    osg::ref_ptr<osg::Geode> earth =
//                        BuildGdEarthFromCamera(cviewer.getView(0)->getCamera());
//                    gpRoot->addChild(earth);
//                    break;
//                }
//            }
//        }


        // update camera frustum
        for(size_t i=0; i < numChildren; i++)   {
            osg::Node * child = gpRoot->getChild(i);
            if(child->getName() == "gpCamera")   {
                gpRoot->removeChild(child);
                osg::ref_ptr<osg::Group> cameraFrustum =
                        BuildFrustumFromCamera(cviewer.getView(0)->getCamera(),
                                               g_LIST_LOD_RANGES);
                gpRoot->addChild(cameraFrustum);
                break;
            }
        }

        cviewer.frame();
    }

    return 0;
}

//int main(int argc, char *argv[])
//{
//    // set debug severity
//    //osg::setNotifyLevel(osg::DEBUG_INFO);

//    // [base tiles]
//    BuildBaseTileList(-180,-86,180,86,16,16,
//                      g_LIST_BASE_VERTICES,
//                      g_LIST_BASE_TILES);
//    for(size_t i=0; i < g_LIST_BASE_TILES.size(); i++)   {
//        g_LIST_BASE_TILES[i]->level = 4;
//    }

//    std::cout << "SZ LIST BASE TILES: "
//              << CountBaseTiles() << std::endl;


//    // [shaders]
//    SetupShaders();

//    // [lods]
//    g_LIST_LOD_RANGES = BuildLODRanges();

//    // [earth node]
//    osg::ref_ptr<osg::Geode> gdEarth = BuildGdEarthFromCamera(NULL);

//    // [camera node]
//    osg::ref_ptr<osg::MatrixTransform> xfCameraFrustum
//            = new osg::MatrixTransform; // empty
//    xfCameraFrustum->setName("xfCameraFrustum");

//    // [root group]
//    osg::ref_ptr<osg::Group> gpRoot = new osg::Group;
//    gpRoot->getOrCreateStateSet()->setMode(GL_BLEND,osg::StateAttribute::ON);

//    gpRoot->addChild(gdEarth);
//    gpRoot->addChild(xfCameraFrustum);


//    // composite viewer
//    osgViewer::CompositeViewer cviewer;
//    cviewer.setThreadingModel(osgViewer::Viewer::SingleThreaded);

//    {   // add view 0
//        osgViewer::View * view = new osgViewer::View;
//        cviewer.addView(view);

//        view->setUpViewInWindow(0,0,640,360);
//        view->setSceneData(gpRoot.get());
//        view->setCameraManipulator(new osgGA::TrackballManipulator);
//        view->getCamera()->setClearColor(osg::Vec4(0.1,0.1,0.1,1.0));
//    }

//    {   // add view 1
//        osgViewer::View * view = new osgViewer::View;
//        cviewer.addView(view);

//        view->setUpViewInWindow(0,425,640,360);
//        view->setSceneData(gpRoot.get());
//        view->setCameraManipulator(new osgGA::TrackballManipulator);
//        view->getCamera()->setClearColor(osg::Vec4(0.1,0.1,0.1,1.0));
//    }

//    osgViewer::Viewer::Windows windows;
//    cviewer.getWindows(windows);
//    for(osgViewer::Viewer::Windows::iterator itr = windows.begin();
//        itr != windows.end();
//        ++itr)
//    {
//        (*itr)->getState()->setUseModelViewAndProjectionUniforms(true);
//        (*itr)->getState()->setUseVertexAttributeAliasing(true);
//    }

//    size_t nf = 30;
//    size_t fc = 0;

//    while(!cviewer.done())   {

//        size_t numChildren =
//                gpRoot->getNumChildren();

//        // update earth node
//        if(fc%nf == 0)   {
//            for(size_t i=0; i < numChildren; i++)   {
//                osg::Node * child = gpRoot->getChild(i);
//                if(child->getName() == "gdEarth")   {
//                    gpRoot->removeChild(child);
//                    osg::ref_ptr<osg::Geode> earth =
//                        BuildGdEarthFromCamera(cviewer.getView(0)->getCamera());
//                    gpRoot->addChild(earth);
//                    break;
//                }
//            }
//        }



//        // update camera frustum
//        for(size_t i=0; i < numChildren; i++)   {
//            osg::Node * child = gpRoot->getChild(i);
//            if(child->getName() == "xfCameraFrustum")   {
//                gpRoot->removeChild(child);
//                osg::ref_ptr<osg::MatrixTransform> cameraFrustum =
//                        BuildFrustumFromCamera(cviewer.getView(0)->getCamera());
//                gpRoot->addChild(cameraFrustum);
//                break;
//            }
//        }

//        fc++;
//        if(fc > 30)
//        { fc = 1; }

////        std::cout << fc << "," << fc%nf << std::endl;

//        cviewer.frame();
//    }

//    return 0;
//}

//PointLLA lla(43,-79,10000);
//Vec3 eye = ConvLLAToECEF(lla);

//    // [poi node]
//    osg::Vec3d c(eye.x,eye.y,eye.z);
//    osg::ref_ptr<osg::Geode> gdPoi =
//            BuildOctahedron(osg::Vec3d(eye.x,eye.y,eye.z),
//                            osg::Vec4(1,0,0,0.5),
//                            1000);

    //    gpRoot->addChild(gdPoi);

