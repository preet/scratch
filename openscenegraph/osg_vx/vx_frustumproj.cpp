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


enum IntersectionType: uint8_t
{
    XSEC_TRUE,
    XSEC_FALSE,
    XSEC_CONTAINED,
    XSEC_COINCIDENT
};

IntersectionType CalcSphereSphereIntersection(osg::Vec3d const &centerA,
                                              osg::Vec3d const &centerB,
                                              double const radiusA,
                                              double const radiusB,
                                              Plane &xsec_plane,
                                              double &xsec_radius)
{
    // ref:
    // http://paulbourke.net/geometry/circlesphere/

    double dist = (centerB-centerA).length();

    if(dist > (radiusA+radiusB)) {
        // The circles are too far away to intersect
        return XSEC_FALSE;
    }
    else if(dist < fabs(radiusA-radiusB)) {
        // One circle is contained within the other
        return XSEC_CONTAINED;
    }
    else if((dist==0) && (radiusA==radiusB)) {
        // The circles are coincident
        return XSEC_COINCIDENT;
    }

    double const a = (radiusA*radiusA -
                      radiusB*radiusB +
                      dist*dist)/(2*dist);

    xsec_plane.p = centerA + (centerB-centerA)*(a/dist);
    xsec_plane.n = centerB-centerA;
    xsec_plane.n.normalize();

    xsec_radius = sqrt(radiusA*radiusA - a*a);

    return XSEC_TRUE;
}

void CalcProjSphereLLAPoly(Plane const &horizon_plane,
                           osg::Vec3d const &sphere_center,
                           double const sphere_radius,
                           std::vector<osg::Vec3d> &list_ecef)
{
    double const dist = sphere_center.length();

    double const horizon_radius =
            sqrt(RAD_AV*RAD_AV - horizon_plane.p*horizon_plane.p);

    Plane xsec_plane;
    double xsec_radius;

    IntersectionType xsec_type =
            CalcSphereSphereIntersection(osg::Vec3d(0,0,0),
                                         sphere_center,
                                         RAD_AV,
                                         sphere_radius,
                                         xsec_plane,
                                         xsec_radius);

    if(xsec_type == XSEC_FALSE) {
        // std::cout << "###: no sphere/sphere xsec" << std::endl;
        return;
    }
    else if(xsec_type == XSEC_COINCIDENT) {
        // should never happen
        return;
    }
    else if(xsec_type == XSEC_CONTAINED) {
        if((dist+sphere_radius) < RAD_AV) {
            // The planetary sphere contains the distal sphere;
            // we should never get here
            return;
        }
        // The distal sphere contains the planetary sphere
        xsec_plane = horizon_plane;
        xsec_radius = horizon_radius;
    }
    else {
        double const dist2_horizon = (sphere_center-horizon_plane.p).length2();
        double const dist2_xsec = (sphere_center-xsec_plane.p).length2();
        if(dist2_horizon < dist2_xsec) {
            xsec_plane = horizon_plane;
            xsec_radius = horizon_radius;
        }
    }

    // Create a ring of 8 points on the XY plane
    double const xrd = 1.0;
    double const irt = 1.0/sqrt(2.0);
    osg::Vec3d const z_axis(0,0,1);
    std::vector<osg::Vec3d> list_xsec_vx(8);
    list_xsec_vx[0] = osg::Vec3d(xrd,0,0);
    list_xsec_vx[1] = osg::Vec3d(irt,irt,0);
    list_xsec_vx[2] = osg::Vec3d(0,xrd,0);
    list_xsec_vx[3] = osg::Vec3d(-irt,irt,0);
    list_xsec_vx[4] = osg::Vec3d(-xrd,0,0);
    list_xsec_vx[5] = osg::Vec3d(-irt,-irt,0);
    list_xsec_vx[6] = osg::Vec3d(0,-xrd,0);
    list_xsec_vx[7] = osg::Vec3d(irt,-irt,0);

    // Rotate the ring so that its aligned to the
    // xsec plane's normal vector
    osg::Vec3d axis = z_axis^xsec_plane.n;
    axis.normalize();

    if(axis.length2() > 1E-4) {
        double angle_rads = acos(z_axis*xsec_plane.n);
        for(auto &xsec_vx : list_xsec_vx) {
            xsec_vx = CalcVectorRotation(xsec_vx,axis,angle_rads);
        }
    }

    // Translate the ring using the xsec plane's 'center'
    for(auto &xsec_vx : list_xsec_vx) {
        xsec_vx = (xsec_vx*xsec_radius)+xsec_plane.p;
    }

    list_ecef = list_xsec_vx;
}

void CalcProjFrustumLLAPoly(Frustum const &frustum,
                            Plane const &horizon_plane,
                            std::vector<osg::Vec3d> &list_ecef)
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
    std::vector<osg::Vec3d> list_ecef_xsec(8);
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

    list_ecef = list_ecef_xsec;
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

        // new surfacepoly
        CalcProjSphereLLAPoly(horizon_plane,eye,K_LIST_LOD_DIST[2],list_ecef);
        auto new_lodsurfacepoly = BuildSurfacePoly(list_ecef,
                                                   K_COLOR_TABLE[2],
                                                   eye.length()/100.0);
        new_lodsurfacepoly->setName("lodsurfacepoly");

        CalcProjFrustumLLAPoly(frustum,horizon_plane,list_ecef);
        auto new_frustumsurfacepoly = BuildSurfacePoly(list_ecef,
                                                       osg::Vec4(0.25,0.25,0.25,1.0),
                                                       eye.length()/100.0);
        new_frustumsurfacepoly->setName("frustumsurfacepoly");


        // new lod rings
        auto new_lodrings = BuildLodRingsNode(eye);

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
                else if(name == "frustumsurfacepoly") {
                    gp_root0->removeChild(i);
                    i--;
                }
                else if(name == "lodsurfacepoly") {
                    gp_root0->removeChild(i);
                    i--;
                }
            }
            gp_root0->addChild(new_horizon);
            gp_root0->addChild(new_frustumsurfacepoly);
            gp_root0->addChild(new_lodsurfacepoly);
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
                else if(name == "lodrings") {
                    gp_root1->removeChild(i);
                    i--;
                }
                else if(name == "horizonplane") {
                    gp_root1->removeChild(i);
                    i--;
                }
                else if(name == "frustumsurfacepoly") {
                    gp_root1->removeChild(i);
                    i--;
                }
                else if(name == "lodsurfacepoly") {
                    gp_root1->removeChild(i);
                    i--;
                }
            }
            // Add new nodes
            gp_root1->addChild(new_frustum);
            gp_root1->addChild(new_mincamdistline);
            gp_root1->addChild(new_lodrings);
            gp_root1->addChild(new_horizon);
            gp_root1->addChild(new_frustumsurfacepoly);
            gp_root1->addChild(new_lodsurfacepoly);
        }

        viewer.frame();
    }

    return 0;
}
