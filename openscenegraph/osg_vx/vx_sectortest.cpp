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

osg::Vec4 CalcRainbowGradient(double cVal)
{
    // clamp cVal between 0 and 1
    if(cVal < 0)   {
        cVal = 0.0;
    }
    if(cVal > 1)   {
        cVal = 1.0;
    }

    unsigned char R,G,B;
    size_t maxBars = 5;     // number of color bars

    double m = maxBars * cVal;
    size_t n = size_t(m);

    double fraction = m-n;
    unsigned char t = int(fraction*255);

    switch(n)   {
        case 0:   {
            R = 255;
            G = t;
            B = 0;
            break;
        }
        case 1:   {
            R = 255 - t;
            G = 255;
            B = 0;
            break;
        }
        case 2:   {
            R = 0;
            G = 255;
            B = t;
            break;
        }
        case 3:   {
            R = 0;
            G = 255 - t;
            B = 255;
            break;
        }
        case 4:   {
            R = t;
            G = 0;
            B = 255;
            break;
        }
        case 5:   {
            R = 255;
            G = 0;
            B = 255 - t;
            break;
        }
    }

    osg::Vec4 myColor(R/255.0,G/255.0,B/255.0,0.5);

    return myColor;
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

inline double EvalProjVectorSphere(osg::Vec3d const &proj_n,
                                   double const sin_lon,
                                   double const cos_lon,
                                   double const sin_lat,
                                   double const cos_lat)
{
     return ((proj_n.x()*RAD_AV*cos_lat*cos_lon) +
             (proj_n.y()*RAD_AV*cos_lat*sin_lon) +
             (proj_n.z()*RAD_AV*sin_lat));
}

void CalcCandidatesForSectorLon(osg::Vec3d const &proj_n,
                                double const lon_degs,
                                double const sin_lon,
                                double const cos_lon,
                                double const min_lat_degs,
                                double const max_lat_degs,
                                std::vector<PointLLA> &list_cand_lla,
                                std::vector<double> &list_cand_f)
{
    double lat_c0_rads = atan2(proj_n.z(),proj_n.x()*cos_lon + proj_n.y()*sin_lon);
    double lat_c1_rads = lat_c0_rads > 0 ? lat_c0_rads-K_PI : lat_c0_rads+K_PI;

    double lat_c0_degs = lat_c0_rads*K_RAD2DEG;
    double lat_c1_degs = lat_c1_rads*K_RAD2DEG;

    // If either crit latitude lies inside the sector, eval
    // f(lon,lat) for that latitude as well
    if(!(lat_c0_degs < min_lat_degs || lat_c0_degs > max_lat_degs)) {
        double sin_lat_c0 = sin(lat_c0_rads);
        double cos_lat_c0 = cos(lat_c0_rads);

        // f(min_lon,lat_c0)
        list_cand_lla.emplace_back(lon_degs,lat_c0_degs);
        list_cand_f.push_back(EvalProjVectorSphere(proj_n,
                                                   sin_lon,
                                                   cos_lon,
                                                   sin_lat_c0,
                                                   cos_lat_c0));
    }
    else if(!(lat_c1_degs < min_lat_degs || lat_c1_degs > max_lat_degs)) {
        double sin_lat_c1 = sin(lat_c1_rads);
        double cos_lat_c1 = cos(lat_c1_rads);

        // f(min_lon,lat_c1)
        list_cand_lla.emplace_back(lon_degs,lat_c1_degs);
        list_cand_f.push_back(EvalProjVectorSphere(proj_n,
                                                   sin_lon,
                                                   cos_lon,
                                                   sin_lat_c1,
                                                   cos_lat_c1));
    }
}

bool CalcProjMinMaxForSector(double const min_lon_degs,
                             double const min_lat_degs,
                             double const max_lon_degs,
                             double const max_lat_degs,
                             osg::Vec3d proj_n,
                             double &proj_min,
                             double &proj_max,
                             PointLLA &lla_min,
                             PointLLA &lla_max)
{
    proj_n.normalize();

    // Calculate the xsec of proj_n and the planet to find
    // the points where proj_min and proj_max occur for the
    // entire planet

    // TODO CalcRayEarthIntersection should be done once
    // outside of this function, not per tile!
    osg::Vec3d xsec_max,xsec_min;
    if(!CalcRayEarthIntersection(osg::Vec3d(0,0,0),
                                 proj_n,
                                 xsec_max,
                                 xsec_min))
    {
        std::cout << "###: CalcProjIntervalsForSector: "
                     "xsec earth failed" << std::endl;
        return false;
    }

    // Swap xsec_max and min if required
    {
        double proj_max_full = xsec_max*proj_n;
        double proj_min_full = xsec_min*proj_n;
        if(proj_max_full < proj_min_full) {
            std::swap(proj_max_full,proj_min_full);
            std::swap(xsec_max,xsec_min);
        }
    }

    double const sin_min_lon = sin(min_lon_degs*K_DEG2RAD);
    double const cos_min_lon = cos(min_lon_degs*K_DEG2RAD);

    double const sin_max_lon = sin(max_lon_degs*K_DEG2RAD);
    double const cos_max_lon = cos(max_lon_degs*K_DEG2RAD);

    double const sin_min_lat = sin(min_lat_degs*K_DEG2RAD);
    double const cos_min_lat = cos(min_lat_degs*K_DEG2RAD);

    double const sin_max_lat = sin(max_lat_degs*K_DEG2RAD);
    double const cos_max_lat = cos(max_lat_degs*K_DEG2RAD);

    // list of candidate lla and proj values
    std::vector<PointLLA> list_cand_lla;
    list_cand_lla.reserve(8);

    std::vector<double> list_cand_f;
    list_cand_f.reserve(8);


    //
    PointLLA lla_xsec = ConvECEFToLLA(xsec_max);


    if(lla_xsec.lon < min_lon_degs || lla_xsec.lon > max_lon_degs) {
        // If the crit lon is outside the sector range, the
        // corner points of the sector are candidates:

        // f(min_lon,min_lat)
        list_cand_lla.emplace_back(min_lon_degs,min_lat_degs);
        list_cand_f.push_back(EvalProjVectorSphere(proj_n,
                                                   sin_min_lon,
                                                   cos_min_lon,
                                                   sin_min_lat,
                                                   cos_min_lat));
        // f(min_lon,max_lat)
        list_cand_lla.emplace_back(min_lon_degs,max_lat_degs);
        list_cand_f.push_back(EvalProjVectorSphere(proj_n,
                                                   sin_min_lon,
                                                   cos_min_lon,
                                                   sin_max_lat,
                                                   cos_max_lat));
        // f(max_lon,min_lat)
        list_cand_lla.emplace_back(max_lon_degs,min_lat_degs);
        list_cand_f.push_back(EvalProjVectorSphere(proj_n,
                                                   sin_max_lon,
                                                   cos_max_lon,
                                                   sin_min_lat,
                                                   cos_min_lat));
        // f(max_lon,max_lat)
        list_cand_lla.emplace_back(max_lon_degs,max_lat_degs);
        list_cand_f.push_back(EvalProjVectorSphere(proj_n,
                                                   sin_max_lon,
                                                   cos_max_lon,
                                                   sin_max_lat,
                                                   cos_max_lat));

        // Additional candidates are the critical latitudes
        // along the min and max longitude edges:

        // Min longitude edge
        CalcCandidatesForSectorLon(proj_n,
                                   min_lon_degs,
                                   sin_min_lon,
                                   cos_min_lon,
                                   min_lat_degs,
                                   max_lat_degs,
                                   list_cand_lla,
                                   list_cand_f);

        // Max longitude edge
        CalcCandidatesForSectorLon(proj_n,
                                   max_lon_degs,
                                   sin_max_lon,
                                   cos_max_lon,
                                   min_lat_degs,
                                   max_lat_degs,
                                   list_cand_lla,
                                   list_cand_f);
    }
    else {
        // If the xsec lon is within the sector range,
        // get candidates for all possible latitudes
        // along xsec lon (min, max, crit)

        double const sin_crit_lon = sin(lla_xsec.lon*K_DEG2RAD);
        double const cos_crit_lon = cos(lla_xsec.lon*K_DEG2RAD);

        // f(crit_lon,min_lat)
        list_cand_lla.emplace_back(lla_xsec.lon,min_lat_degs);
        list_cand_f.push_back(EvalProjVectorSphere(proj_n,
                                                   sin_crit_lon,
                                                   cos_crit_lon,
                                                   sin_min_lat,
                                                   cos_min_lat));

        // f(crit_lon,max_lat)
        list_cand_lla.emplace_back(lla_xsec.lon,max_lat_degs);
        list_cand_f.push_back(EvalProjVectorSphere(proj_n,
                                                   sin_crit_lon,
                                                   cos_crit_lon,
                                                   sin_max_lat,
                                                   cos_max_lat));

        // Additional candidates are crit latitudes along crit lon
        CalcCandidatesForSectorLon(proj_n,
                                   lla_xsec.lon,
                                   sin_crit_lon,
                                   cos_crit_lon,
                                   min_lat_degs,
                                   max_lat_degs,
                                   list_cand_lla,
                                   list_cand_f);
    }

    proj_max = K_MIN_NEG_DBL;
    for(size_t i=0; i < list_cand_lla.size(); i++) {
        if(list_cand_f[i] > proj_max) {
            proj_max = list_cand_f[i];
            lla_max = list_cand_lla[i];
        }
    }

    return false;
}

void UpdateProjIntervalSectorMap(osg::Group * gp,
                                 osg::Vec3d proj_n)
{
    proj_n.normalize();

    osg::Geode * gd = static_cast<osg::Geode*>(gp->getChild(0));
    osg::Geometry * gm = static_cast<osg::Geometry*>(gd->getDrawable(0));
    osg::Vec3dArray * list_vx = static_cast<osg::Vec3dArray*>(gm->getVertexArray());

    double proj_max = K_MIN_NEG_DBL;
    double proj_min = K_MAX_POS_DBL;
    std::vector<double> list_p;
    list_p.reserve(list_vx->size());
    for(size_t i=0; i < list_vx->size(); i++) {
        double p=proj_n*list_vx->at(i);
        list_p.push_back(p);
        proj_max = std::max(proj_max,p);
        proj_min = std::min(proj_min,p);
    }
    double const delta = fabs(proj_min);
    proj_max += fabs(delta);


    osg::ref_ptr<osg::Vec4Array> list_cx = new osg::Vec4Array;
    list_cx->reserve(list_p.size());

    for(size_t i=0; i < list_p.size(); i++) {
        osg::Vec4 cx = CalcRainbowGradient((list_p[i]+delta)/proj_max);
        list_cx->push_back(cx);
    }

    gm->setColorArray(list_cx,osg::Array::Binding::BIND_PER_VERTEX);
}

osg::ref_ptr<osg::Group> BuildProjIntervalSectorMap(double min_lon,
                                                    double min_lat,
                                                    double max_lon,
                                                    double max_lat,
                                                    size_t lon_segments,
                                                    size_t lat_segments,
                                                    osg::Vec3d proj_n)
{
    osg::ref_ptr<osg::Group> gp = new osg::Group;

    proj_n.normalize();

    std::vector<osg::Vec3d> list_vx;
    std::vector<osg::Vec2d> list_tx;
    std::vector<size_t> list_ix;
    if(!BuildEarthSurfaceGeometry(min_lon,
                                  min_lat,
                                  max_lon,
                                  max_lat,
                                  lon_segments,
                                  lat_segments,
                                  list_vx,
                                  list_tx,
                                  list_ix))
    {
        std::cout << "BuildProjIntervalSectorMap: bad input range" << std::endl;
        return gp;
    }

    osg::ref_ptr<osg::Vec3dArray> vx_array = new osg::Vec3dArray;
    vx_array->reserve(list_vx.size());

    osg::ref_ptr<osg::Vec4Array> cx_array = new osg::Vec4Array;
    cx_array->reserve(list_vx.size());

    for(auto const &vx : list_vx) {
        vx_array->push_back(vx);
        cx_array->push_back(osg::Vec4(1,1,1,1));
    }

    osg::ref_ptr<osg::DrawElementsUShort> ix_array =
            new osg::DrawElementsUShort(GL_TRIANGLES);
    for(auto const ix : list_ix) {
        ix_array->push_back(ix);
    }

    // geometry
    osg::ref_ptr<osg::Geometry> gm = new osg::Geometry;
    gm->setVertexArray(vx_array.get());
    gm->setColorArray(cx_array.get(),osg::Array::Binding::BIND_PER_VERTEX);
    gm->addPrimitiveSet(ix_array.get());

    // geode
    osg::ref_ptr<osg::Geode> gd = new osg::Geode;
    gd->addDrawable(gm);

    // save
    gp->addChild(gd);
    gp->setName("sectorprojmap");

    return gp;
}

//
int main()
{
    Frustum frustum;
    Plane horizon_plane;

    // Celestial body geometry
    auto gp_celestial = BuildCelestialSurfaceNode();

    // Sector geometry
    auto gp_sectorprojmap = BuildProjIntervalSectorMap(-90,0,0,90,12,12,osg::Vec3d(0,0,1));
    gp_sectorprojmap->getOrCreateStateSet()->setMode( GL_DEPTH_TEST,osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);

    auto gp_sectormax = BuildFacingCircle(osg::Vec3d(0,0,0),1000.0,8,osg::Vec4(1,1,1,1));
    gp_sectormax->setName("sectormax");
    gp_sectorprojmap->getOrCreateStateSet()->setMode( GL_DEPTH_TEST,osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
    gp_sectormax->getOrCreateStateSet()->setRenderBinDetails(10,"RenderBin");

//    auto gp_sectormin = BuildFacingCircle(osg::Vec3d(0,0,0),1000.0,8,osg::Vec4(1,1,1,1));
//    gp_sectormin->setName("sectormin");

    osgViewer::CompositeViewer viewer;
    viewer.setThreadingModel(osgViewer::ViewerBase::SingleThreaded);

    // View0 root
    osg::ref_ptr<osg::Group> gp_root0 = new osg::Group;
    gp_root0->addChild(gp_celestial);
    gp_root0->addChild(gp_sectorprojmap);
    gp_root0->addChild(gp_sectormax);

    // View1 root
    osg::ref_ptr<osg::Group> gp_root1 = new osg::Group;
    gp_root1->addChild(gp_celestial);
    gp_root1->addChild(gp_sectorprojmap);
    gp_root1->addChild(gp_sectormax);

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

        // new mindcamdistline
        auto new_mincamdistline = BuildMinCamDistLineNode(eye);
        new_mincamdistline->getOrCreateStateSet()->setMode( GL_DEPTH_TEST,osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);

        // update gp_sectorprojmap
        UpdateProjIntervalSectorMap(gp_sectorprojmap.get(),eye);

        {
            PointLLA lla_min,lla_max;
            double proj_min,proj_max;
            CalcProjMinMaxForSector(-90,0,0,90,eye,proj_min,proj_max,lla_min,lla_max);

//            lla_max.alt = 0;
            std::cout << "###: lla: " << lla_max.lon << ", " << lla_max.lat << std::endl;
            UpdateFacingCircle(ConvLLAToECEF(lla_max),eye.length()/250.0,gp_sectormax);
        }

        // Update gp_root0
        {
            for(size_t i=0; i < gp_root0->getNumChildren(); i++) {
                std::string const name = gp_root0->getChild(i)->getName();
                if(name == "horizonplane") {
                    gp_root0->removeChild(i);
                    i--;
                }
            }
            gp_root0->addChild(new_horizon);
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
                else if(name == "horizonplane") {
                    gp_root1->removeChild(i);
                    i--;
                }
                else if(name == "mincamdistline") {
                    gp_root1->removeChild(i);
                    i--;
                }
            }
            // Add new nodes
            gp_root1->addChild(new_frustum);
            gp_root1->addChild(new_horizon);
            gp_root1->addChild(new_mincamdistline);
        }

        viewer.frame();

    }
    return 0;
}

