#ifndef TILE_UTIL_H
#define TILE_UTIL_H

#include <gmutil.hpp>
#include <osg/ref_ptr>
#include <osg/Group>

struct VxTile
{
    uint64_t id;

    double minLon;
    double maxLon;

    double minLat;
    double maxLat;

    double midLat;
    double midLon;

    osg::Vec3d ecef_LM;
    osg::Vec3d ecef_MB;
    osg::Vec3d ecef_RM;
    osg::Vec3d ecef_MT;
    osg::Vec3d ecef_MM;

    osg::Vec3d * p_ecef_LT;
    osg::Vec3d * p_ecef_LB;
    osg::Vec3d * p_ecef_RB;
    osg::Vec3d * p_ecef_RT;

    std::unique_ptr<VxTile> tile_LT;
    std::unique_ptr<VxTile> tile_LB;
    std::unique_ptr<VxTile> tile_RB;
    std::unique_ptr<VxTile> tile_RT;

    uint8_t level;

    OBB obb;

    bool _fvis;
    bool _hvis;

    bool _s;

    osg::ref_ptr<osg::Group> _gp;

    VxTile() : _s(true) {}
};

void CalcTileOBB(VxTile * tile)
{
    // 1. Determine the orthonormal basis

    // At the poles the distance in 'x' (lon) might
    // be zero or too small so choose the vec that
    // has a greater magnitude:
    osg::Vec3d const x_top = (*(tile->p_ecef_LT))-(*(tile->p_ecef_RT));
    osg::Vec3d const x_btm = (*(tile->p_ecef_LB))-(*(tile->p_ecef_RB));

    tile->obb.ori[0] = (x_top.length2() > x_btm.length2()) ? x_top : x_btm;
    tile->obb.ori[2] = tile->ecef_MM;
    tile->obb.ori[1] = tile->obb.ori[2]^tile->obb.ori[0];

    tile->obb.ori[0].normalize();
    tile->obb.ori[1].normalize();
    tile->obb.ori[2].normalize();

    // Get the bbox min and max
//    std::vector<osg::Vec3d const *> tile_vx(5);
//    tile_vx[0] = tile->p_ecef_LT;
//    tile_vx[1] = tile->p_ecef_LB;
//    tile_vx[2] = tile->p_ecef_RB;
//    tile_vx[3] = tile->p_ecef_RT;
//    tile_vx[4] = &(tile->ecef_MM);

    std::vector<osg::Vec3d const *> tile_vx {
        tile->p_ecef_LT,
        tile->p_ecef_LB,
        tile->p_ecef_RB,
        tile->p_ecef_RT,
        &(tile->ecef_MM)
    };

    // 2. Get the min and max corners
    osg::Vec3d min_rst(K_MAX_POS_DBL,K_MAX_POS_DBL,K_MAX_POS_DBL);
    osg::Vec3d max_rst(K_MIN_NEG_DBL,K_MIN_NEG_DBL,K_MIN_NEG_DBL);

    // Project all the vertices of the model onto the
    // orthonormal vectors and get min/max along them
    for(auto vx_ptr : tile_vx) {
        osg::Vec3d const &vx = *vx_ptr;
        double const dist_r = tile->obb.ori[0]*vx;
        double const dist_s = tile->obb.ori[1]*vx;
        double const dist_t = tile->obb.ori[2]*vx;

        min_rst.x() = std::min(min_rst.x(),dist_r);
        min_rst.y() = std::min(min_rst.y(),dist_s);
        min_rst.z() = std::min(min_rst.z(),dist_t);

        max_rst.x() = std::max(max_rst.x(),dist_r);
        max_rst.y() = std::max(max_rst.y(),dist_s);
        max_rst.z() = std::max(max_rst.z(),dist_t);
    }

    // 3. Calc center and extents
    tile->obb.center =
            (tile->obb.ori[0] * (min_rst.x()+max_rst.x())*0.5) +
            (tile->obb.ori[1] * (min_rst.y()+max_rst.y())*0.5) +
            (tile->obb.ori[2] * (min_rst.z()+max_rst.z())*0.5);

    tile->obb.ext = (max_rst-min_rst)*0.5;

    // 4. Save the unique face planes
    for(int i=0; i < 3; i++) {
        Plane &face = tile->obb.faces[i];
        face.n = tile->obb.ori[i];
        face.p = (face.n * tile->obb.ext[i]) + tile->obb.center;
        face.d = face.n * face.p;
    }
}

VxTile * BuildRootTile(uint8_t const level,
                       double const minLon,
                       double const minLat,
                       double const maxLon,
                       double const maxLat)
{
    VxTile * t = new VxTile;
    t->level = level;
    t->minLon = minLon;
    t->minLat = minLat;
    t->maxLon = maxLon;
    t->maxLat = maxLat;

    t->midLon = (minLon+maxLon)*0.5;
    t->midLat = (minLat+maxLat)*0.5;

    t->p_ecef_LT = new osg::Vec3d;
    *(t->p_ecef_LT) = ConvLLAToECEF(PointLLA(minLon,maxLat));

    t->p_ecef_LB = new osg::Vec3d;
    *(t->p_ecef_LB) = ConvLLAToECEF(PointLLA(minLon,minLat));

    t->p_ecef_RB = new osg::Vec3d;
    *(t->p_ecef_RB) = ConvLLAToECEF(PointLLA(maxLon,minLat));

    t->p_ecef_RT = new osg::Vec3d;
    *(t->p_ecef_RT) = ConvLLAToECEF(PointLLA(maxLon,maxLat));

    t->ecef_LM = ConvLLAToECEF(PointLLA(minLon,t->midLat));
    t->ecef_MB = ConvLLAToECEF(PointLLA(t->midLon,minLat));
    t->ecef_RM = ConvLLAToECEF(PointLLA(maxLon,t->midLat));
    t->ecef_MT = ConvLLAToECEF(PointLLA(t->midLon,maxLat));
    t->ecef_MM = ConvLLAToECEF(PointLLA(t->midLon,t->midLat));

    CalcTileOBB(t);

    return t;
}

std::unique_ptr<VxTile> BuildChildTile(VxTile * parent,
                                       uint8_t const quadrant,
                                       bool calc_obb=true)
{
    std::unique_ptr<VxTile> t(new VxTile);

    if(quadrant == 0) {
        // LT
        t->minLon = parent->minLon;
        t->maxLon = parent->midLon;
        t->minLat = parent->midLat;
        t->maxLat = parent->maxLat;

        t->p_ecef_LT = parent->p_ecef_LT;
        t->p_ecef_LB = &(parent->ecef_LM);
        t->p_ecef_RB = &(parent->ecef_MM);
        t->p_ecef_RT = &(parent->ecef_MT);
    }
    else if(quadrant == 1) {
        // LB
        t->minLon = parent->minLon;
        t->maxLon = parent->midLon;
        t->minLat = parent->minLat;
        t->maxLat = parent->midLat;

        t->p_ecef_LT = &(parent->ecef_LM);
        t->p_ecef_LB = parent->p_ecef_LB;
        t->p_ecef_RB = &(parent->ecef_MB);
        t->p_ecef_RT = &(parent->ecef_MM);
    }
    else if(quadrant == 2) {
        // RB
        t->minLon = parent->midLon;
        t->maxLon = parent->maxLon;
        t->minLat = parent->minLat;
        t->maxLat = parent->midLat;

        t->p_ecef_LT = &(parent->ecef_MM);
        t->p_ecef_LB = &(parent->ecef_MB);
        t->p_ecef_RB = parent->p_ecef_RB;
        t->p_ecef_RT = &(parent->ecef_RM);
    }
    else {
        // RT
        t->minLon = parent->midLon;
        t->maxLon = parent->maxLon;
        t->minLat = parent->midLat;
        t->maxLat = parent->maxLat;

        t->p_ecef_LT = &(parent->ecef_MT);
        t->p_ecef_LB = &(parent->ecef_MM);
        t->p_ecef_RB = &(parent->ecef_RM);
        t->p_ecef_RT = parent->p_ecef_RT;
    }

    t->level = parent->level + 1;
    t->midLon = (t->minLon+t->maxLon)*0.5;
    t->midLat = (t->minLat+t->maxLat)*0.5;

    t->ecef_LM = ConvLLAToECEF(PointLLA(t->minLon,t->midLat));
    t->ecef_MB = ConvLLAToECEF(PointLLA(t->midLon,t->minLat));
    t->ecef_RM = ConvLLAToECEF(PointLLA(t->maxLon,t->midLat));
    t->ecef_MT = ConvLLAToECEF(PointLLA(t->midLon,t->maxLat));
    t->ecef_MM = ConvLLAToECEF(PointLLA(t->midLon,t->midLat));

    if(calc_obb) {
        CalcTileOBB(t.get());
    }

    return t;
}

std::vector<VxTile*> BuildBaseViewExtents(uint8_t const level)
{
    std::vector<VxTile*> list_vxtiles;

    size_t const num_tiles_side = pow(2,level);
    double lon_step = 360.0/num_tiles_side;
    double lat_step = 180.0/num_tiles_side;

    for(size_t i=0; i < num_tiles_side; i++) { // lon
        for(size_t j=0; j < num_tiles_side; j++) { // lat

//    for(size_t i=2; i < 3; i++) { // lon
//        for(size_t j=2; j < 3; j++) { // lat

            double const min_lon = -180.0 + lon_step*i;
            double const max_lon = min_lon + lon_step;
            double const min_lat = -90.0 + lat_step*j;
            double const max_lat = min_lat + lat_step;

            VxTile * tile = BuildRootTile(level,
                                          min_lon,
                                          min_lat,
                                          max_lon,
                                          max_lat);

            list_vxtiles.push_back(tile);
        }
    }

    return list_vxtiles;
}


#endif // TILE_UTIL_H
