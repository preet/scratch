/*
   Copyright (C) 2014 Preet Desai (preet.desai@gmail.com)

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/

#include <iostream>

#include <cassert>
#include <TileSetLLByPixelRes.h>
#include <GeometryUtils.h>
#include <OSGUtils.h>

// ============================================================= //

// TODO move these to GeometryUtils
static Circle CalcCircleForLatPlane(Plane const &plane_lat)
{
    Circle c;
    c.normal = plane_lat.n;
    c.center = CalcPointLineProjection(plane_lat.p,
                                       K_ZERO_VEC,
                                       osg::Vec3d(0,0,1),
                                       false);
    c.radius = (c.center-plane_lat.p).length();
    return c;
}

static Circle CalcCircleForLonPlane(Plane const &plane_lon)
{
    Circle c;
    c.normal = plane_lon.n;
    c.center = K_ZERO_VEC;
    c.radius = RAD_AV;

    return c;
}

TileSetLLByPixelRes::Eval::Eval(GeoBounds const &bounds)
{
    // corners
    c_min_min = ConvLLAToECEF(LLA(bounds.minLon,bounds.minLat));
    c_max_min = ConvLLAToECEF(LLA(bounds.maxLon,bounds.minLat));
    c_max_max = ConvLLAToECEF(LLA(bounds.maxLon,bounds.maxLat));
    c_min_max = ConvLLAToECEF(LLA(bounds.minLon,bounds.maxLat));

    // lon and lat planes
    plane_min_lon = CalcLonPlane(bounds.minLon,true,true);
    plane_max_lon = CalcLonPlane(bounds.maxLon,false,true);
    plane_min_lat = CalcLatPlane(bounds.minLat,false);
    plane_max_lat = CalcLatPlane(bounds.maxLat,true);

    // circle arc edges
    circle_min_lon = CalcCircleForLonPlane(plane_min_lon);
    circle_max_lon = CalcCircleForLonPlane(plane_max_lon);
    circle_min_lat = CalcCircleForLatPlane(plane_min_lat);
    circle_max_lat = CalcCircleForLatPlane(plane_max_lat);
}


// ============================================================= //

TileSetLLByPixelRes::TileSetLLByPixelLength(double view_width,
                                            double view_height,
                                            Options const &opts,
                                            std::vector<RootTileDesc> const &list_root_tiles) :
    m_opts(opts),
    m_view_width(view_width),
    m_view_height(view_height)
{
    // Generate tiles from list_root_tiles
    for(auto const &root_tile : list_root_tiles) {
        std::unique_ptr<Tile> tile(new Tile(root_tile));
        m_list_root_tiles.push_back(std::move(tile));
    }
    m_tile_count = m_list_root_tiles.size();
}

TileSetLLByPixelRes::~TileSetLLByPixelRes()
{

}

void TileSetLLByPixelRes::UpdateTileSet(osg::Camera const * cam,
                                        std::vector<uint64_t> &list_tiles_add,
                                        std::vector<uint64_t> &list_tiles_upd,
                                        std::vector<uint64_t> &list_tiles_rem)
{
    // update view data
    m_cam = cam;
    m_mvp = cam->getViewMatrix()*
            cam->getProjectionMatrix();

    osg::Vec3d eye,vpt,up;
    cam->getViewMatrixAsLookAt(eye,vpt,up);
    m_view_dirn = vpt-eye;
    m_view_dirn.normalize();

    m_near_plane.n = m_view_dirn;
    m_near_plane.n.normalize();

    m_near_plane.p = eye + m_near_plane.n*100.0;
    m_near_plane.d = m_near_plane.n*m_near_plane.p;

    double ar,fovy,z_near,z_far;
    cam->getProjectionMatrixAsPerspective(fovy,ar,z_near,z_far);

    Frustum frustum;
    {
        auto osg_frustum = BuildFrustumNode(
                    "frustum",cam,frustum,z_near,z_far);
    }

    Plane horizon_plane = CalcHorizonPlane(eye);

    // TODO check calcprojfrustumpoly is successful
    CalcProjFrustumPoly(frustum,horizon_plane,m_list_frustum_ecef);

    m_list_frustum_bounds.clear();
    m_list_frustum_lla = ConvListECEFToLLA(m_list_frustum_ecef);
    CalcMinGeoBoundsFromLLAPoly(ConvECEFToLLA(eye),
                                m_list_frustum_lla,
                                m_list_frustum_bounds);

    if(m_list_frustum_bounds.empty()) {
        return;
    }

    m_frustum_pole = 0;
    for(auto const &bounds : m_list_frustum_bounds) {
        if(bounds.maxLat == 90.0) {
            m_frustum_pole = 1;
            break;
        }

        if(bounds.minLat == -90.0) {
            m_frustum_pole = 2;
            break;
        }
    }

    m_list_frustum_tri_planes =
            calcFrustumPolyTriPlanes(m_list_frustum_ecef,true);


    // rebuild the tileset with the new view data
    m_tile_count = m_list_root_tiles.size();
    std::map<uint64_t,Tile const *> list_tileset_new;
    for(auto & root_tile : m_list_root_tiles) {
        buildTileSet(root_tile,m_tile_count);
        buildTileSetList(root_tile,list_tileset_new);
    }

    // get tileset lists as sorted lists of tile ids
    std::vector<uint64_t> list_tiles_new;
    list_tiles_new.reserve(list_tileset_new.size());
    for(auto it = list_tileset_new.begin();
        it != list_tileset_new.end(); ++it)
    {
        list_tiles_new.push_back(it->first);
    }

    std::vector<uint64_t> list_tiles_old;
    list_tiles_old.reserve(m_list_tileset.size());
    for(auto it = m_list_tileset.begin();
        it != m_list_tileset.end(); ++it)
    {
        list_tiles_old.push_back(it->first);
    }

    // split the new and old tile sets into
    // tiles added, removed and common
    SplitSets(list_tiles_new,
              list_tiles_old,
              list_tiles_add,  // tiles added
              list_tiles_rem,  // tiles removed
              list_tiles_upd); // tiles common

    // TEMP TODO
    list_tiles_upd.clear();

    m_list_tileset = list_tileset_new;
}

TileSetLL::Tile const * TileSetLLByPixelRes::GetTile(uint64_t tile_id) const
{
    auto it = m_list_tileset.find(tile_id);
    if(it == m_list_tileset.end()) {
        return nullptr;
    }
    else {
        return it->second;
    }
}

void TileSetLLByPixelRes::buildTileSet(std::unique_ptr<Tile> &tile,
                                       size_t &tile_count)
{
    if( (tile->level < m_opts.max_level) &&
        (tile_count < (m_opts.max_tiles - 2)) &&
        tilePxResExceedsLevel(tile.get()) ) {
        //
        if(tile->clip == Tile::k_clip_NONE) {
            uint32_t const x = tile->x*2;
            uint32_t const y = tile->y*2;
            tile->tile_LT.reset(new Tile(tile.get(),x,y+1));
            tile->tile_LB.reset(new Tile(tile.get(),x,y));
            tile->tile_RB.reset(new Tile(tile.get(),x+1,y));
            tile->tile_RT.reset(new Tile(tile.get(),x+1,y+1));
            tile->clip = Tile::k_clip_ALL;
        }

        tile_count += 4; // add children
        tile_count -= 1; // rem parent

        buildTileSet(tile->tile_LT,tile_count);
        buildTileSet(tile->tile_LB,tile_count);
        buildTileSet(tile->tile_RB,tile_count);
        buildTileSet(tile->tile_RT,tile_count);
    }
    else {
        if(tile->clip == Tile::k_clip_ALL) {
            tile->tile_LT = nullptr;
            tile->tile_LB = nullptr;
            tile->tile_RB = nullptr;
            tile->tile_RT = nullptr;
            tile->clip = Tile::k_clip_NONE;
        }
    }
}

void TileSetLLByPixelRes::buildTileSetList(std::unique_ptr<Tile> const &tile,
                                           std::map<uint64_t,Tile const *> &list_tiles)
{
    if(tile->clip == Tile::k_clip_NONE) { // visible w no children
        list_tiles.insert(std::pair<uint64_t,Tile const *>(tile->id,tile.get()));
    }
    else {
        buildTileSetList(tile->tile_LT,list_tiles);
        buildTileSetList(tile->tile_LB,list_tiles);
        buildTileSetList(tile->tile_RB,list_tiles);
        buildTileSetList(tile->tile_RT,list_tiles);
    }
}

bool TileSetLLByPixelRes::tilePxResExceedsLevel(Tile const * tile)
{
    double const k_ndc_to_px =
            (m_view_width*m_view_height*0.25);

    GeoBounds const tile_bounds(tile->min_lon,
                                tile->max_lon,
                                tile->min_lat,
                                tile->max_lat);

    // Create the evalution geometry if required
    if(m_list_tile_eval.size() > m_opts.max_level*2) {
        // TODO proper metric for clearing this list
        m_list_tile_eval.clear();
    }

    auto it = m_list_tile_eval.find(tile->id);
    if(it == m_list_tile_eval.end()) {
        it = m_list_tile_eval.emplace(
                    std::pair<uint64_t,Eval>(
                        tile->id,
                        Eval(tile_bounds,
                             m_mvp,
                             m_opts.min_eval_angle_degs))).first;
    }

    if(!calcFrustumTileIntersection(it->second,
                                    m_list_frustum_ecef,
                                    m_list_frustum_bounds,
                                    m_list_frustum_tri_planes,
                                    tile_bounds))
    {
        // The tile isn't visible with the current frustum
        return false;
    }

    //
}

bool TileSetLLByPixelRes::calcFrustumTileIntersection(Eval const &eval,
                                                      std::vector<osg::Vec3d> const &list_frustum_vx,
                                                      std::vector<GeoBounds> const &list_frustum_bounds,
                                                      std::vector<Plane> const &list_frustum_tri_planes,
                                                      GeoBounds const &tile_bounds) const
{
    // TODO determine a good tolerance (this is in meters)
    static const double k_eps = 0.0;

    if(list_frustum_vx.size() != 8) {
        return false;
    }

    // Three tests:
    // 0. GeoBounds intersection
    //  * if the geobounds of the frustum poly and tile
    //    do not intersect, there is no intersection

    bool xsec = false;
    GeoBounds xsec_bounds;
    for(auto const &frustum_bounds : list_frustum_bounds) {
        if(CalcGeoBoundsIntersection(frustum_bounds,
                                     tile_bounds,
                                     xsec_bounds))
        {
            if(xsec_bounds == frustum_bounds) {
                // Frustum poly is within the tile
//                std::cout << "#: XSEC TYPE 1" << std::endl;
                return true;
            }

            xsec = true;
            break;
        }
    }

    if(!xsec) {
        return false;
    }

    // Check to see if the frustum poly edges intersect
    // with any tile planes

    size_t xsec_count;
    std::vector<osg::Vec3d> list_xsec;
    list_xsec.reserve(list_frustum_vx.size());

    // TODO:
    // Would it be faster to compute intersections
    // against all the tile edge planes everytime?

    // min_lon
    {
        xsec_count = CalcPlanePolyIntersection(eval.plane_min_lon,
                                               list_frustum_vx,
                                               list_xsec);
        for(size_t i=0; i < xsec_count; i++) {
            if(calcPointWithinTilePlanes(list_xsec[list_xsec.size()-1-i],
                                         eval.plane_min_lon,
                                         eval.plane_max_lon,
                                         eval.plane_min_lat,
                                         eval.plane_max_lat)) {
//                std::cout << "#: XSEC TYPE 2" << std::endl;
                return true;
            }
        }
    }

    // max_lon
    {
        xsec_count = CalcPlanePolyIntersection(eval.plane_max_lon,
                                               list_frustum_vx,
                                               list_xsec);

        for(size_t i=0; i < xsec_count; i++) {
            if(calcPointWithinTilePlanes(list_xsec[list_xsec.size()-1-i],
                                         eval.plane_min_lon,
                                         eval.plane_max_lon,
                                         eval.plane_min_lat,
                                         eval.plane_max_lat)) {
//                std::cout << "#: XSEC TYPE 2" << std::endl;
                return true;
            }
        }
    }

    // max_lat
    {
        xsec_count = CalcPlanePolyIntersection(eval.plane_min_lat,
                                               list_frustum_vx,
                                               list_xsec);

        for(size_t i=0; i < xsec_count; i++) {
            if(calcPointWithinTilePlanes(list_xsec[list_xsec.size()-1-i],
                                         eval.plane_min_lon,
                                         eval.plane_max_lon,
                                         eval.plane_min_lat,
                                         eval.plane_max_lat)) {
//                std::cout << "#: XSEC TYPE 2" << std::endl;
                return true;
            }
        }
    }

    // min_lat
    {
        xsec_count = CalcPlanePolyIntersection(eval.plane_max_lat,
                                               list_frustum_vx,
                                               list_xsec);

        for(size_t i=0; i < xsec_count; i++) {
            if(calcPointWithinTilePlanes(list_xsec[list_xsec.size()-1-i],
                                         eval.plane_min_lon,
                                         eval.plane_max_lon,
                                         eval.plane_min_lat,
                                         eval.plane_max_lat)) {
//                std::cout << "#: XSEC TYPE 2" << std::endl;
                return true;
            }
        }
    }

    // Check to see if the frustum poly completely
    // contains the tile (the previous test verified
    // the the tile isn't partially contained) by
    // finding if any of the triangle regions of
    // the frustum poly contains a point from the tile
    {
        // Test point from tile // TODO replace with Eval
        osg::Vec3d const &ecef = eval.ecef_mid;

        for(size_t i=0; i < list_frustum_tri_planes.size(); i+=3) {
            Plane const &plane0 = list_frustum_tri_planes[i+0];
            Plane const &plane1 = list_frustum_tri_planes[i+1];
            Plane const &plane2 = list_frustum_tri_planes[i+2];

            bool outside =
                    ((ecef-plane0.p)*plane0.n > k_eps) ||
                    ((ecef-plane1.p)*plane1.n > k_eps) ||
                    ((ecef-plane2.p)*plane2.n > k_eps);

            if(!outside) {
//                std::cout << "#: XSEC_TYPE 3_" << i/3 << std::endl;
                return true;
            }
        }
    }
}

std::vector<Plane>
TileSetLLByPixelRes::calcFrustumPolyTriPlanes(std::vector<osg::Vec3d> const &list_frustum_vx,
                                              bool normalize) const
{
    static const std::vector<uint16_t> list_ix = {
        0,1,7,
        1,2,3,
        3,4,5,
        5,6,7,
        5,7,1,
        5,1,3
    };

    std::vector<Plane> list_tri_planes;
    list_tri_planes.reserve(18);

    for(size_t i=0; i < list_ix.size(); i+=3) {
        // Each plane is the plane that goes through
        // the points of a triangle edge, and (0,0,0)
        // since each edge represents a great arc

        osg::Vec3d const &v0 = list_frustum_vx[list_ix[i+0]];
        osg::Vec3d const &v1 = list_frustum_vx[list_ix[i+1]];
        osg::Vec3d const &v2 = list_frustum_vx[list_ix[i+2]];

        Plane plane0;
        plane0.n = (v1-v0)^v0;
        if(normalize) {
            plane0.n.normalize();
        }
        plane0.p = v0;
        plane0.d = plane0.n*plane0.p;

        Plane plane1;
        plane1.n = (v2-v1)^v1;
        if(normalize) {
            plane1.n.normalize();
        }
        plane1.p = v1;
        plane1.d = plane1.n*plane1.p;

        Plane plane2;
        plane2.n = (v0-v2)^v2;
        if(normalize) {
            plane2.n.normalize();
        }
        plane2.p = v2;
        plane2.d = plane2.n*plane2.p;

        list_tri_planes.push_back(plane0);
        list_tri_planes.push_back(plane1);
        list_tri_planes.push_back(plane2);
    }

    return list_tri_planes;
}
