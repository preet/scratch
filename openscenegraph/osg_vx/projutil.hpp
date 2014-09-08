#ifndef PROJ_UTIL_H
#define PROJ_UTIL_H

#include <gmutil.hpp>
#include <clipper.hpp>

osg::Vec3d CalcGnomonicProjOrigin(Plane const &horizon_plane,
                                  double const min_angle_degs=10.0)
{
    // For a true gnomonic projection, the projection
    // origin should be at the center of the planet.

    // However, if the horizon plane is too close to the
    // center (or contains it), points on the horizon plane
    // can't be projected to another plane with the same
    // normal as the projection rays will be parallel to
    // said plane.

    // The projection origin has to be moved back behind
    // the center of the planet in this case. How far its
    // moved back is determined by min_angle_degs.

    double const min_angle_rads =
            fabs(min_angle_degs*K_DEG2RAD);

    double const p_length = horizon_plane.p.length();

    double const angle_rads =
            atan2(p_length,RAD_AV);

//    std::cout << "###: " << angle_rads*K_RAD2DEG << std::endl;

    if(fabs(angle_rads) > min_angle_rads) {
        // We can use the true center
        return osg::Vec3d(0,0,0);
    }

    double min_p_length = tan(min_angle_rads)*RAD_AV;
    osg::Vec3d proj_center = horizon_plane.p;
    proj_center.normalize();
    proj_center = proj_center * (min_p_length-p_length) * -1.0;

    return proj_center;
}

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

bool CalcGnomonicProjPolyXY(std::vector<osg::Vec3d> const &list_ecef,
                            osg::Vec3d const &proj_center,
                            Plane const &tangent_plane,
                            std::vector<osg::Vec2d> &list_proj_vx_xy)
{
    osg::Matrixd xf_tangent_to_xy;
    xf_tangent_to_xy.makeRotate(tangent_plane.n,osg::Vec3d(0,0,1));

    // Close-to-gnomonic projection of list_ecef from proj_center
    list_proj_vx_xy.clear();
    list_proj_vx_xy.reserve(list_ecef.size());
    for(auto const &ecef : list_ecef) {
        double u;
        osg::Vec3d xsec;
        if(!CalcRayPlaneIntersection(proj_center,
                                     ecef-proj_center,
                                     tangent_plane,
                                     xsec,
                                     u))
        {
            return false;
        }

        xsec = xsec * xf_tangent_to_xy;
        list_proj_vx_xy.push_back(osg::Vec2d(xsec.x(),xsec.y()));
    }
    return true;
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
    }

    list_ecef = list_ecef_xsec;
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


#endif // PROJ_UTIL_H
