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

#include <clipper.hpp>


enum IntersectionType: uint8_t
{
    XSEC_TRUE,
    XSEC_FALSE,
    XSEC_CONTAINED,
    XSEC_COINCIDENT
};

bool CalcCameraNearFarDist(osg::Vec3d const &eye,
                           osg::Vec3d const &view_dirn,
                           double const pin_surf_dist_m,
                           double &dist_near,
                           double &dist_far)
{
    double const radius = RAD_AV;
    double const eye_dist2 = eye.length2();

    if(eye_dist2 > (radius*radius)) {
        // The near distance is set to the length of the
        // the line segment of the eye projected onto
        // the plane with n==view_dirn and p==(0,0,0)
        // less the radius
        Plane plane;
        plane.n = (view_dirn*-1.0);
        plane.n.normalize();
        plane.p = osg::Vec3d(0,0,0);
        plane.d = (plane.n*plane.p);

        osg::Vec3d xsec = CalcPointPlaneProjection(eye,plane);
        dist_near = (eye-xsec).length() - (radius+pin_surf_dist_m);

        if(dist_near < 1.0) {
            dist_near = 1.0;
        }

        // The far distance is set to the tangential distance
        // from the eye to the horizon (see horizon plane)
        dist_far = sqrt(eye_dist2 - radius*radius);
        return true;
    }
    return false;
}

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

bool CalcGnomonicProjPoly(std::vector<osg::Vec3d> const &list_ecef,
                          osg::Vec3d const &proj_center,
                          Plane const &plane,
                          std::vector<osg::Vec3d> &list_proj_vx)
{
    list_proj_vx.resize(list_ecef.size());
    for(size_t i=0; i < list_ecef.size(); i++) {
        double u;
        if(!CalcRayPlaneIntersection(proj_center,
                                     list_ecef[i]-proj_center,
                                     plane,
                                     list_proj_vx[i],
                                     u)) {
            return false;
        }
    }
    return true;
}

bool CalcGnomonicProjIntersectionPoly(osg::Vec3d const &proj_center,
                                      Plane const &tangent_plane,
                                      std::vector<osg::Vec3d> const &poly_frustum,
                                      std::vector<osg::Vec3d> const &poly_lod_sphere,
                                      std::vector<osg::Vec3d> & poly_xsec,
                                      std::vector<osg::Vec3d> & poly_tangent)
{
    if(poly_lod_sphere.empty()) {
        return false;
    }

    // Calculate the rotation matrix to rotate the
    // tangent plane to the xy plane and its inverse
    osg::Vec3d const &from = tangent_plane.n;
    osg::Vec3d to(0,0,1);

    osg::Matrixd xf_tangent_to_xy;
    xf_tangent_to_xy.makeRotate(from,to);

    osg::Matrixd xf_xy_to_tangent =
            osg::Matrixd::inverse(xf_tangent_to_xy);

    // Project poly_frustum onto tangent_plane using proj_center,
    // align it to the xy plane, and save it to a clipper poly
    ClipperLib::Path poly1;
    std::vector<osg::Vec3d> proj_poly_frustum(poly_frustum.size());
    for(size_t i=0; i < poly_frustum.size(); i++) {
        // surface -> tangent_plane
        double u;
        if(!CalcRayPlaneIntersection(proj_center,
                                     poly_frustum[i]-proj_center,
                                     tangent_plane,
                                     proj_poly_frustum[i],
                                     u)) {
            return false;
        }
        // tangent_plane -> xy plane
        proj_poly_frustum[i] = proj_poly_frustum[i] * xf_tangent_to_xy;

        ClipperLib::cInt x = proj_poly_frustum[i].x()*100.0;
        ClipperLib::cInt y = proj_poly_frustum[i].y()*100.0;
        poly1.push_back(ClipperLib::IntPoint(x,y));
    }

    double const z_xy_plane = proj_poly_frustum[0].z();

    // Project poly_lod_sphere onto tangent_plane using proj_center,
    // align it to the xy plane, and save it to a clipper poly
    ClipperLib::Path poly2;
    std::vector<osg::Vec3d> proj_poly_lod_sphere(poly_lod_sphere.size());
    for(size_t i=0; i < poly_lod_sphere.size(); i++) {
        // surface -> tangent_plane
        double u;
        if(!CalcRayPlaneIntersection(proj_center,
                                     poly_lod_sphere[i]-proj_center,
                                     tangent_plane,
                                     proj_poly_lod_sphere[i],
                                     u)) {
            return false;
        }
        // tangent_plane -> xy plane
        proj_poly_lod_sphere[i] = proj_poly_lod_sphere[i] * xf_tangent_to_xy;

        ClipperLib::cInt x = proj_poly_lod_sphere[i].x()*100.0;
        ClipperLib::cInt y = proj_poly_lod_sphere[i].y()*100.0;
        poly2.push_back(ClipperLib::IntPoint(x,y));
    }

    poly_tangent = proj_poly_lod_sphere;
    for(size_t i=0; i < poly_tangent.size(); i++) {
        poly_tangent[i] = poly_tangent[i] * xf_xy_to_tangent;
    }

    // intersection using clipper
    ClipperLib::Clipper clipper;
    ClipperLib::Paths result;
    clipper.AddPath(poly1,ClipperLib::ptSubject,true);
    clipper.AddPath(poly2,ClipperLib::ptClip,true);
    if(!clipper.Execute(ClipperLib::ctIntersection,result)) {
        std::cout << "ERROR: CalcGnomonicProjIntersection: "
                      "Could not calc xsec region" << std::endl;
        return false;
    }
    if(result.empty()) {
//        std::cout << "WARN: CalcGnomonicProjIntersection: "
//                      "zero intersections" << std::endl;
        poly_xsec.clear();
        return true;
    }
    if(result.size() > 1) {
        std::cout << "ERROR: CalcGnomonicProjIntersection: "
                     "multiple intersections: "
                   << result.size() << std::endl;
        return false;
    }
    poly_xsec.resize(result[0].size());


    for(size_t i=0; i < poly_xsec.size(); i++) {
        // xy plane -> tangent_plane
        poly_xsec[i] =
                osg::Vec3d(double(result[0][i].X)*0.01,
                           double(result[0][i].Y)*0.01,
                           z_xy_plane) * xf_xy_to_tangent;

        // tangent_plane -> planet surface
        osg::Vec3d xsec_near,xsec_far;
        if(!CalcRayEarthIntersection(poly_xsec[i],
                                     proj_center-poly_xsec[i],
                                     xsec_near,
                                     xsec_far))
        {
            std::cout << "ERROR: CalcGnomonicProjIntersection: "
                          "could not reproject xsec area" << std::endl;
            return false;
        }
        poly_xsec[i] = xsec_near;
    }

    return true;
}

void CalcProjSpherePoly(Plane const &horizon_plane,
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

void CalcProjFrustumPoly_Direct(Frustum const &frustum,
                                   Plane const &horizon_plane,
                                   std::vector<osg::Vec3d> &list_ecef)
{
    (void)horizon_plane;

    osg::Vec3d const far_center = (frustum.list_vx[4]+frustum.list_vx[6])*0.5;
    osg::Vec3d const view_dirn = (far_center-frustum.eye);

    // Check if the planet is in the view frustum
    if(CalcSphereOutsideFrustumExact(frustum,osg::Vec3d(0,0,0),RAD_AV)) {
        return;
    }

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
            // should get these from the function?
            double const u_near = (xsec_near-ray_pt)*ray_dirn;
            double const u_far = (xsec_far-ray_pt)*ray_dirn;
            if((u_near > 0) && (u_far > 0))
            {
                if((xsec_near-frustum.eye)*view_dirn > 0.0) {
                    list_ecef_xsec[i] = xsec_near;
                }
                else {
                    list_ecef_xsec[i] = xsec_far;
                }

                continue;
            }
        }

        // Project the rays that missed the planet
        // back onto the surface of the planet
        if(!CalcRayEarthIntersection(list_ecef_xsec[i],
                                     list_ecef_xsec[i],
                                     xsec_near,
                                     xsec_far))
        {
            // should never get here
            std::cout << "###: ERROR: 2RE xsec to surface failed" << std::endl;
            return;
        }

        list_ecef_xsec[i] = xsec_near;
    }

    list_ecef = list_ecef_xsec;
}

void CalcProjFrustumPoly(Frustum const &frustum,
                            Plane const &horizon_plane,
                            std::vector<osg::Vec3d> &list_ecef)
{   
    // Check if the planet is in the view frustum
    if(CalcSphereOutsideFrustumExact(frustum,osg::Vec3d(0,0,0),RAD_AV)) {
        return;
    }

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

    list_ecef.clear();

    osg::Vec3d const far_center = (frustum.list_vx[4]+frustum.list_vx[6])*0.5;
    osg::Vec3d const view_dirn = (far_center-frustum.eye);

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
            double const u_near = (xsec_near-ray_pt)*ray_dirn;
            double const u_far = (xsec_far-ray_pt)*ray_dirn;
            if((u_near > 0) && (u_far > 0))
            {
                if((xsec_near-frustum.eye)*view_dirn > 0.0) {
                    list_ecef_xsec[i] = xsec_near;
                }
                else {
                    list_ecef_xsec[i] = xsec_far;
                }
                list_ecef.push_back(list_ecef_xsec[i]);
                continue;
            }
        }
        // Project rays that missed the planetary
        // body onto the horizon plane
        osg::Vec3d const ecef_horizon =
                CalcPointPlaneProjection(list_ecef_xsec[i],
                                         horizon_plane);

        if(!CalcRayEarthIntersection(ecef_horizon,
                                     horizon_plane.p - ecef_horizon,
                                     xsec_near,
                                     xsec_far))
        {
            // should never get here
            std::cout << "###: FATAL" << std::endl;
            return;
        }

        list_ecef_xsec[i] = xsec_near;
//        list_ecef.push_back(list_ecef_xsec[i]);
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

        osg::ref_ptr<osgGA::TrackballManipulator> view_manip =
                new osgGA::TrackballManipulator;
        view_manip->setMinimumDistance(100);

        view->setCameraManipulator(view_manip);

    }

    // Create view 1 (this view shows View0's frustum)
    {
        osgViewer::View* view = new osgViewer::View;
        viewer.addView( view );

        view->setUpViewInWindow( 10, 510, 640, 480 );
        view->setSceneData( gp_root1.get() );
        view->getCamera()->setClearColor(osg::Vec4(0.1,0.1,0.1,1.0));

        osg::ref_ptr<osgGA::TrackballManipulator> view_manip =
                new osgGA::TrackballManipulator;
        view_manip->setMinimumDistance(100);

        view->setCameraManipulator(view_manip);
    }

    while(!viewer.done())
    {
        osg::Camera * camera = viewer.getView(0)->getCamera();

        osg::Vec3d eye,vpt,up;
        camera->getViewMatrixAsLookAt(eye,vpt,up);

        // new horizon plane node
        auto new_horizon = BuildHorizonPlaneNode(camera,horizon_plane);

        // new camera frustum node

        double far_dist,near_dist;
        if(!CalcCameraNearFarDist(eye,vpt-eye,20000.0,near_dist,far_dist)) {
            far_dist=0.0;
            near_dist=0.0;
        }

        auto new_frustum = BuildFrustumNode(camera,frustum,near_dist,far_dist);

        osg::ref_ptr<osg::Group> new_lodsurfacepoly;
        osg::ref_ptr<osg::Group> new_frustumsurfacepoly;
        osg::ref_ptr<osg::Group> new_projcenter;

        if(eye.length() > RAD_AV) {

            osg::Vec3d proj_center = CalcGnomonicProjOrigin(horizon_plane);
            Plane tangent_plane = horizon_plane;
            tangent_plane.p = horizon_plane.n * RAD_AV;
            tangent_plane.d = tangent_plane.n * tangent_plane.p;

            std::vector<osg::Vec3d> poly_frustum_surf;
            CalcProjFrustumPoly(frustum,horizon_plane,poly_frustum_surf);

            std::vector<osg::Vec3d> poly_frustum_tangent;
            if(!CalcGnomonicProjPoly(poly_frustum_surf,
                                     proj_center,
                                     tangent_plane,
                                     poly_frustum_tangent)) {
                poly_frustum_tangent.clear();
            }

            std::vector<std::vector<osg::Vec3d>> list_polys_xsec(K_LIST_LOD_DIST.size());
            std::vector<std::vector<osg::Vec3d>> list_polys_tangent(K_LIST_LOD_DIST.size());
            for(size_t i=0; i < K_LIST_LOD_DIST.size(); i++) {
                std::vector<osg::Vec3d> poly_lod_sphere;
                CalcProjSpherePoly(horizon_plane,eye,K_LIST_LOD_DIST[i],poly_lod_sphere);

                // xsec
                if(!CalcGnomonicProjIntersectionPoly(proj_center,
                                                     tangent_plane,
                                                     poly_frustum_surf,
                                                     poly_lod_sphere,
                                                     list_polys_xsec[i],
                                                     list_polys_tangent[i]))
                {
                    list_polys_xsec[i].clear();
                    //list_polys_tangent[i].clear();
                }
            }




            // lodsurfacepoly
            new_lodsurfacepoly = new osg::Group;
            for(size_t i=0; i < list_polys_xsec.size(); i++) {
                new_lodsurfacepoly->addChild(BuildSurfacePoly(list_polys_tangent[i],
                                                              K_COLOR_TABLE[i],
                                                              eye.length()/1200.0));
            }

            // frustumsurfacepoly
            {
                new_frustumsurfacepoly = BuildSurfacePoly(poly_frustum_tangent,
                                                          osg::Vec4(0.75,0.5,1.0,1.0),
                                                          eye.length()/1200.0);
            }

            new_projcenter = new osg::Group;
//            new_projcenter = BuildRingNode(proj_center,
//                                           osg::Vec4(1.0,0.0,0.0,1.0),
//                                           eye.length()/300.0);
        }
        else {
            new_lodsurfacepoly = new osg::Group;
            new_frustumsurfacepoly = new osg::Group;
            new_projcenter = new osg::Group;
//            std::cout << "###: _in" << std::endl;
        }

        new_lodsurfacepoly->setName("lodsurfacepoly");
        new_frustumsurfacepoly->setName("frustumsurfacepoly");
        new_projcenter->setName("projcenter");





        // new lod rings
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
                else if(name == "frustumsurfacepoly") {
                    gp_root0->removeChild(i);
                    i--;
                }
                else if(name == "lodsurfacepoly") {
                    gp_root0->removeChild(i);
                    i--;
                }
                else if(name == "projcenter") {
                    gp_root0->removeChild(i);
                    i--;
                }
            }
            gp_root0->addChild(new_horizon);
            gp_root0->addChild(new_frustumsurfacepoly);
            gp_root0->addChild(new_lodsurfacepoly);
            gp_root0->addChild(new_projcenter);
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
                else if(name == "projcenter") {
                    gp_root1->removeChild(i);
                    i--;
                }
            }
            // Add new nodes
            gp_root1->addChild(new_frustum);
            gp_root1->addChild(new_mincamdistline);
//            gp_root1->addChild(new_lodrings);
            gp_root1->addChild(new_horizon);
            gp_root1->addChild(new_frustumsurfacepoly);
            gp_root1->addChild(new_lodsurfacepoly);
            gp_root1->addChild(new_projcenter);
        }

        viewer.frame();

    }
    return 0;
}

