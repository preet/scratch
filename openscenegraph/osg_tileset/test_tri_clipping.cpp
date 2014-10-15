#include <iostream>

#include <ViewController.hpp>
#include <OSGUtils.h>
#include <TileSetLLByPixelArea.h>
#include <DataSetTilesLL.h>

osg::ref_ptr<osg::Group> BuildTriangleGeometry(std::vector<osg::Vec3d> const &list_tri_vx,
                                               osg::Vec4 const &color)
{
    osg::ref_ptr<osg::Vec3dArray> list_vx = new osg::Vec3dArray;
    for(size_t i=0; i < list_tri_vx.size(); i++) {
        list_vx->push_back(list_tri_vx[i]);
    }

    osg::ref_ptr<osg::Vec4Array> list_cx = new osg::Vec4Array;
    list_cx->push_back(color);

    osg::ref_ptr<osg::Geometry> gm = new osg::Geometry;
    gm->setVertexArray(list_vx);
    gm->setColorArray(list_cx,osg::Array::BIND_OVERALL);
    gm->addPrimitiveSet(new osg::DrawArrays(GL_TRIANGLES,0,list_vx->size()));

    osg::ref_ptr<osg::Geode> gd = new osg::Geode;
    gd->addDrawable(gm);
    gd->getOrCreateStateSet()->setMode(
                GL_CULL_FACE,
                osg::StateAttribute::ON |
                osg::StateAttribute::OVERRIDE);

    osg::ref_ptr<osg::Group> gp_tri = new osg::Group;
    gp_tri->addChild(gd);
    return gp_tri;
}

osg::ref_ptr<osg::Group> BuildAxesGeometry(double length=3.0)
{
    osg::ref_ptr<osg::Vec3dArray> list_vx = new osg::Vec3dArray;
    list_vx->push_back(osg::Vec3d(0,0,0));
    list_vx->push_back(osg::Vec3d(length,0,0));

    list_vx->push_back(osg::Vec3d(0,0,0));
    list_vx->push_back(osg::Vec3d(0,length,0));

    list_vx->push_back(osg::Vec3d(0,0,0));
    list_vx->push_back(osg::Vec3d(0,0,length));

    osg::ref_ptr<osg::Vec4Array> list_cx = new osg::Vec4Array;
    list_cx->push_back(osg::Vec4(1,0,0,1));
    list_cx->push_back(osg::Vec4(1,0,0,1));

    list_cx->push_back(osg::Vec4(0,1,0,1));
    list_cx->push_back(osg::Vec4(0,1,0,1));

    list_cx->push_back(osg::Vec4(0,0,1,1));
    list_cx->push_back(osg::Vec4(0,0,1,1));

    osg::ref_ptr<osg::Geometry> gm = new osg::Geometry;
    gm->setVertexArray(list_vx);
    gm->setColorArray(list_cx,osg::Array::BIND_PER_VERTEX);
    gm->addPrimitiveSet(new osg::DrawArrays(GL_LINES,0,list_vx->size()));

    osg::ref_ptr<osg::Geode> gd = new osg::Geode;
    gd->addDrawable(gm);

    osg::ref_ptr<osg::Group> gp = new osg::Group;
    gp->addChild(gd);
    return gp;
}

bool RemoveChildByName(osg::Group * gp_parent,
                       std::string const &name)
{
    for(size_t i=0; i < gp_parent->getNumChildren(); i++)
    {
        std::string const node_name =
                gp_parent->getChild(i)->getName();

        if(node_name == name) {
            gp_parent->removeChild(i);
            return true;
        }
    }

    return false;
}

osg::Vec4 const K_COLOR_INSIDE = osg::Vec4(0.98,0.5,0.5,1); // pink
osg::Vec4 const K_COLOR_OUTSIDE = osg::Vec4(0.37,0.77,0.35,1); // green

int main()
{
    // view root
    osg::ref_ptr<osg::Group> gp_root = new osg::Group;

    // disable lighting and enable blending
    gp_root->getOrCreateStateSet()->setMode( GL_LIGHTING,osg::StateAttribute::OFF);
    gp_root->getOrCreateStateSet()->setMode( GL_BLEND,osg::StateAttribute::ON);
    gp_root->getOrCreateStateSet()->setMode( GL_DEPTH_TEST,osg::StateAttribute::OFF);

    // base triangle
    std::vector<osg::Vec3d> const list_tri_vx = {
        osg::Vec3d(1,0,0),
        osg::Vec3d(0,1,0),
        osg::Vec3d(0,0,1)
    };

    // gp_tri
    {
        osg::ref_ptr<osg::Group> gp_tri =
                BuildTriangleGeometry(list_tri_vx,
                                      osg::Vec4(0.3,0.3,0.3,1.0));
        gp_root->addChild(gp_tri);
    }

    // gp_axes
    {
        osg::ref_ptr<osg::Group> gp_axes = BuildAxesGeometry(3.0);
        gp_root->addChild(gp_axes);
    }

    // setup view
    double const view_width  = 600;
    double const view_height = 360;
    osgViewer::Viewer viewer;
    viewer.setThreadingModel(osgViewer::ViewerBase::SingleThreaded);
    viewer.setUpViewInWindow(100,100,view_width,view_height);
    viewer.setSceneData(gp_root);
    viewer.getCamera()->setClearColor(osg::Vec4(0.1,0.1,0.1,1.0));

    osg::ref_ptr<osgGA::TrackballManipulator> view_manip =
            new osgGA::TrackballManipulator;
    view_manip->setMinimumDistance(100);
    viewer.setCameraManipulator(view_manip);

    while(!viewer.done())
    {
        osg::Camera const * cam = viewer.getCamera();
        osg::Vec3d eye,vpt,up;
        cam->getViewMatrixAsLookAt(eye,vpt,up);

        // create the clip plane
        Plane plane;
        plane.n = (vpt-eye); // view dirn
//        plane.n = osg::Vec3d(1,0,0);
        plane.p = osg::Vec3d(0,0,0);
        plane.d = plane.n*plane.p;

        std::vector<osg::Vec3d> list_inside_vx;
        std::vector<osg::Vec3d> list_outside_vx;
        GeometryResult result = CalcTrianglePlaneClip(list_tri_vx,
                                                      plane,
                                                      list_inside_vx,
                                                      list_outside_vx);
        // rem old
        RemoveChildByName(gp_root.get(),"gp_inside");
        RemoveChildByName(gp_root.get(),"gp_outside");

        if(result == GeometryResult::CLIP_INSIDE) {
            // add new gp_inside
            osg::ref_ptr<osg::Group> gp_inside =
                    BuildTriangleGeometry(list_tri_vx,K_COLOR_INSIDE);
            gp_inside->setName("gp_inside");
            gp_root->addChild(gp_inside);
        }
        else if(result == GeometryResult::CLIP_OUTSIDE) {
            // add new gp_outside
            osg::ref_ptr<osg::Group> gp_outside =
                    BuildTriangleGeometry(list_tri_vx,K_COLOR_OUTSIDE);
            gp_outside->setName("gp_outside");
            gp_root->addChild(gp_outside);
        }
        else if(result == GeometryResult::CLIP_XSEC) {
            assert(list_inside_vx.size() >= 3);
            assert(list_outside_vx.size() >= 3);

            // add new gp_inside
            if(list_inside_vx.size() > 3) {
                osg::Vec3d temp_vx = list_inside_vx.back();
                list_inside_vx.pop_back();
                list_inside_vx.push_back(list_inside_vx[0]);
                list_inside_vx.push_back(list_inside_vx[2]);
                list_inside_vx.push_back(temp_vx);

                osg::ref_ptr<osg::Group> gp_inside =
                        BuildTriangleGeometry(list_inside_vx,K_COLOR_INSIDE);
                gp_inside->setName("gp_inside");
                gp_root->addChild(gp_inside);
            }
            else {
                osg::ref_ptr<osg::Group> gp_inside =
                        BuildTriangleGeometry(list_inside_vx,K_COLOR_INSIDE);
                gp_inside->setName("gp_inside");
                gp_root->addChild(gp_inside);
            }

            // add new gp_outside
            if(list_outside_vx.size() > 3) {
                osg::Vec3d temp_vx = list_outside_vx.back();
                list_outside_vx.pop_back();
                list_outside_vx.push_back(list_outside_vx[0]);
                list_outside_vx.push_back(list_outside_vx[2]);
                list_outside_vx.push_back(temp_vx);

                osg::ref_ptr<osg::Group> gp_outside =
                        BuildTriangleGeometry(list_outside_vx,K_COLOR_OUTSIDE);
                gp_outside->setName("gp_outside");
                gp_root->addChild(gp_outside);
            }
            else {
                osg::ref_ptr<osg::Group> gp_outside =
                        BuildTriangleGeometry(list_outside_vx,K_COLOR_OUTSIDE);
                gp_outside->setName("gp_outside");
                gp_root->addChild(gp_outside);
            }
        }

        viewer.frame();
    }

    std::cout << "[exit...]" << std::endl;
    return 0;
}
