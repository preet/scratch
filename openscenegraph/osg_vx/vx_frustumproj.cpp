// sys includes
#include <math.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <sys/time.h>
#include <cassert>
#include <memory>
#include <numeric>
#include <chrono>

//
#include <osgnodes.hpp>



void CalcProjFrustumLLAPoly(Frustum const &frustum,
                            Plane const &horizon_plane,
                            std::vector<osg::Vec3d> &list_ecef_xsec)
{   
    // Check none of the frustum edges are perpendicular
    // to the horizon plane normal to ensure they intersect
    // the plane.
    osg::Vec3d eye_to_center = (frustum.eye)*-1.0;
    eye_to_center.normalize();

    for(size_t i=4; i < 8; i++) {
        osg::Vec3d vx_to_eye = frustum.list_vx[i]-frustum.eye;
        vx_to_eye.normalize();
        // Sets the max angle between eye_to_center and
        // a frustum edge to approximately 75 degrees
        if(vx_to_eye*eye_to_center < 0.25) {
            std::cout << "###: ERROR: degree threshold exceeded" << std::endl;
            return;
        }
    }

    // Check if the planet is in the view frustum
    if(CalcSphereOutsideFrustumExact(frustum,osg::Vec3d(0,0,0),RAD_AV)) {
        return;
    }

    osg::Vec3d const zero_vec(0,0,0);

    // Use 8 points on the view frustum edges to find
    // the LLA polygon:
    list_ecef_xsec.resize(8);
//    std::vector<osg::Vec3d> list_ecef_xsec(8);
    list_ecef_xsec[0] = frustum.list_vx[4]; // BL
    list_ecef_xsec[1] = (frustum.list_vx[4]+frustum.list_vx[5])*0.5;
    list_ecef_xsec[2] = frustum.list_vx[5]; // BR
    list_ecef_xsec[3] = (frustum.list_vx[5]+frustum.list_vx[6])*0.5;
    list_ecef_xsec[4] = frustum.list_vx[6]; // TR
    list_ecef_xsec[5] = (frustum.list_vx[6]+frustum.list_vx[7])*0.5;
    list_ecef_xsec[6] = frustum.list_vx[7]; // TL
    list_ecef_xsec[7] = (frustum.list_vx[7]+frustum.list_vx[4])*0.5;


    for(size_t i=0; i < list_ecef_xsec.size(); i++)
    {
        osg::Vec3d const &ray_pt = frustum.eye;
        osg::Vec3d const ray_dirn = list_ecef_xsec[i]-frustum.eye;

        osg::Vec3d xsec_near,xsec_far;
        if(CalcRayEarthIntersection(ray_pt,
                                    ray_dirn,
                                    xsec_near,
                                    xsec_far))
        {
            list_ecef_xsec[i] = xsec_near;
        }
        else {
            // Intersect rays that missed the planetary
            // body onto the horizon plane
            double u = -1;
            osg::Vec3d xsec_horizon;
            if(!(CalcRayPlaneIntersection(ray_pt,
                                          ray_dirn,
                                          horizon_plane,
                                          xsec_horizon,
                                          u) &&
                 (u > 0)))
            {
                std::cout << "###: ERROR: ray,horizon_plane xsec failed" << std::endl;
                return;
            }

            // Project the horizon plane intersection points
            // back onto the surface of the planet
            if(!CalcRayEarthIntersection(zero_vec,
                                         xsec_horizon,
                                         xsec_near,
                                         xsec_far))
            {
                // should never get here
                std::cout << "###: ERROR: RE xsec horizon to surface failed" << std::endl;
                return;
            }

//            // Project points that did not intersect the
//            // planetary body using the direction from the
//            // center of the planet to the point
//            osg::Vec3d const center_to_point =
//                    list_ecef_xsec[i]-zero_vec;

//            if(!CalcRayEarthIntersection(zero_vec,
//                                         center_to_point,
//                                         xsec_near,
//                                         xsec_far))
//            {
//                std::cout << "###: ERROR: RE xsec center_to_point failed" << std::endl;
//                return;
//            }

//            std::cout << "###: ping" << std::endl;

            // Use the intersection point that lies along
            // the positive projection (ie same direction)
            // as the direction vector
            osg::Vec3d xsec_surf0;
            if(xsec_horizon*xsec_near >= 0) {
                xsec_surf0 = xsec_near;
            }
            else {
                xsec_surf0 = xsec_far;
            }
            list_ecef_xsec[i] = xsec_surf0;
        }
    }
}

// Ensure that inside out projection works
void Test()
{
    osg::Vec3d xsec_near,xsec_far;
    if(CalcRayEarthIntersection(osg::Vec3d(0,0,0),
                                osg::Vec3d(1,0,0),
                                xsec_near,
                                xsec_far))
    {
        if(xsec_near*osg::Vec3d(1,0,0) >= 0) {
            std::cout << "###: xsec_near is +ve t along the ray" << std::endl;
        }
        else {
            std::cout << "###: xsec_near is -ve t along the ray" << std::endl;
        }
    }
    else {
        std::cout << "###: FATAL " << std::endl;
    }
}


//
int main()
{
    Frustum frustum;
    Plane horizon_plane;

    // Celestial body geometry
    auto gp_celestial = BuildCelestialSurfaceNode();

    osgViewer::CompositeViewer viewer;
    viewer.setThreadingModel(osgViewer::ViewerBase::SingleThreaded);

    // View0 root
    osg::ref_ptr<osg::Group> gp_root0 = new osg::Group;
    gp_root0->addChild(gp_celestial);

    // View1 root
    osg::ref_ptr<osg::Group> gp_root1 = new osg::Group;
    gp_root1->addChild(gp_celestial);

    // disable lighting and enable blending
    gp_root0->getOrCreateStateSet()->setMode( GL_LIGHTING,osg::StateAttribute::OFF);
    gp_root0->getOrCreateStateSet()->setMode( GL_BLEND,osg::StateAttribute::ON);
    gp_root0->getOrCreateStateSet()->setMode( GL_DEPTH_TEST,osg::StateAttribute::OFF);

    gp_root1->getOrCreateStateSet()->setMode( GL_LIGHTING,osg::StateAttribute::OFF);
    gp_root1->getOrCreateStateSet()->setMode( GL_BLEND,osg::StateAttribute::ON);
    gp_root1->getOrCreateStateSet()->setMode( GL_DEPTH_TEST,osg::StateAttribute::OFF);

    // Create View 0
    {
        osgViewer::View* view = new osgViewer::View;
        viewer.addView( view );

        view->setUpViewInWindow( 10, 10, 640, 480 );
        view->setSceneData( gp_root0.get() );
        view->getCamera()->setClearColor(osg::Vec4(0.1,0.1,0.1,1.0));
        view->setCameraManipulator( new osgGA::TrackballManipulator );
    }

    // Create view 1 (this view shows View0's frustum)
    {
        osgViewer::View* view = new osgViewer::View;
        viewer.addView( view );

        view->setUpViewInWindow( 10, 510, 640, 480 );
        view->setSceneData( gp_root1.get() );
        view->getCamera()->setClearColor(osg::Vec4(0.1,0.1,0.1,1.0));
        view->setCameraManipulator( new osgGA::TrackballManipulator );
    }

    while(!viewer.done())
    {
        osg::Camera * camera = viewer.getView(0)->getCamera();

        osg::Vec3d eye,vpt,up;
        camera->getViewMatrixAsLookAt(eye,vpt,up);

        // new horizon plane node
        auto new_horizon = BuildHorizonPlaneNode(camera,horizon_plane);

        // new camera frustum node
        double far_dist = 0.0;
        {
            double const eye_dist2 = eye.length2();
             double const rad_length2 = RAD_AV*RAD_AV;
            if(eye_dist2 > rad_length2) {
               far_dist = sqrt(eye.length2() - (RAD_AV*RAD_AV));
            }
        }
        auto new_frustum = BuildFrustumNode(camera,frustum,far_dist);

        std::vector<osg::Vec3d> list_ecef;
        CalcProjFrustumLLAPoly(frustum,horizon_plane,list_ecef);

        // new surf poly
        auto new_surfacepoly = BuildSurfacePoly(list_ecef);

//        // new lod rings
//        auto new_lodrings = BuildLodRingsNode(eye);

        // new min cam dist line node
        auto new_mincamdistline = BuildMinCamDistLineNode(eye);

        // Update gp_root0
        {
            for(size_t i=0; i < gp_root0->getNumChildren(); i++) {
                std::string const name = gp_root0->getChild(i)->getName();
                if(name == "horizonplane") {
                    gp_root0->removeChild(i);
                    i--;
                }
                else if(name == "surfacepoly") {
                    gp_root0->removeChild(i);
                    i--;
                }
            }
            gp_root0->addChild(new_horizon);
            gp_root0->addChild(new_surfacepoly);
        }

        // Update gp_root1
        {
            for(size_t i=0; i < gp_root1->getNumChildren(); i++) {
                std::string const name = gp_root1->getChild(i)->getName();
                // Remove prev camera node
                if(name == "frustum") {
                    gp_root1->removeChild(i);
                    i--;
                }
                else if(name == "mincamdistline") {
                    gp_root1->removeChild(i);
                    i--;
                }
//                else if(name == "lodrings") {
//                    gp_root1->removeChild(i);
//                    i--;
//                }
                else if(name == "horizonplane") {
                    gp_root1->removeChild(i);
                    i--;
                }
                else if(name == "surfacepoly") {
                    gp_root1->removeChild(i);
                    i--;
                }
            }
            // Add new nodes
            gp_root1->addChild(new_frustum);
            gp_root1->addChild(new_mincamdistline);
//            gp_root1->addChild(new_lodrings);
            gp_root1->addChild(new_horizon);
            gp_root1->addChild(new_surfacepoly);
        }

        viewer.frame();
    }

    return 0;
}
