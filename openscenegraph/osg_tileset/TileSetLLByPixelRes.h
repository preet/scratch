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

#ifndef SCRATCH_TILESET_LL_BYPIXELLRES_H
#define SCRATCH_TILESET_LL_BYPIXELLRES_H

#include <TileSetLL.h>
#include <GeometryUtils.h>
#include <chrono>

class TileSetLLByPixelRes : public TileSetLL
{
public:

    struct Options
    {
        // Initialize to sane defaults
        Options() :
            tile_sz_px(350),
            min_level(0),
            max_level(18),
            fix_level(1),
            max_eval_sz(320),
            max_adj_offset_dist2_m(1000000.0*1000000.0)
        {}

        size_t tile_sz_px;
        size_t min_level;
        size_t max_level;
        size_t fix_level;
        size_t max_eval_sz;
        double max_adj_offset_dist2_m;
    };

    TileSetLLByPixelRes(double view_width,
                        double view_height,
                        Options const &opts,
                        std::vector<RootTileDesc> const &list_root_tiles);

    ~TileSetLLByPixelRes();

    void UpdateTileSet(osg::Camera const * cam,
                       std::vector<uint64_t> &list_tiles_add,
                       std::vector<uint64_t> &list_tiles_upd,
                       std::vector<uint64_t> &list_tiles_rem);

    Tile const * GetTile(uint64_t tile_id) const;

private:
    struct Eval
    {
        Eval(TileId id,
             GeoBounds const &bounds);

        TileId const id;

        // surface area (m^2)
        double surf_area_m2;

        // mid
        osg::Vec3d ecef_mid;

        // corner points
        // (min/max)lon_(min,max)lat
        osg::Vec3d c_min_min;
        osg::Vec3d c_max_min;
        osg::Vec3d c_max_max;
        osg::Vec3d c_min_max;

        // planes
        Plane plane_min_lon;
        Plane plane_max_lon;
        Plane plane_min_lat;
        Plane plane_max_lat;

        // circle arc edges
        Circle circle_min_lon;
        Circle circle_max_lon;
        Circle circle_min_lat;
        Circle circle_max_lat;
    };

    void buildTileSet(std::unique_ptr<Tile> &tile);

    void buildTileSetList(std::unique_ptr<Tile> const &tile,
                          std::map<uint64_t,Tile const *> &list_tiles);

    Eval const * getEvalData(Tile const * tile,
                             GeoBounds const &tile_bounds);

    bool tilePxResExceedsLevel(Tile * tile);

    // * checks whether or not the projected frustum poly
    //   as specified by @list_frustum_vx,)_bounds,_tri_planes
    //   intersects the tile given by @tile_bounds
    bool calcFrustumTileIntersection(Eval const &eval,
                                     std::vector<osg::Vec3d> const &list_frustum_vx,
                                     std::vector<GeoBounds> const &list_frustum_bounds,
                                     std::vector<Plane> const &list_frustum_tri_planes,
                                     GeoBounds const &tile_bounds) const;

    inline bool calcPointWithinTilePlanes(osg::Vec3d const &point,
                                          Plane const &plane_min_lon,
                                          Plane const &plane_max_lon,
                                          Plane const &plane_min_lat,
                                          Plane const &plane_max_lat) const
    {
        static const double k_eps = 1E-5; // meters

        bool outside =
                (((point-plane_min_lon.p)*plane_min_lon.n) > k_eps) ||
                (((point-plane_max_lon.p)*plane_max_lon.n) > k_eps) ||
                (((point-plane_min_lat.p)*plane_min_lat.n) > k_eps) ||
                (((point-plane_max_lat.p)*plane_max_lat.n) > k_eps);

        return (!outside);
    }

    // * calculates the edge planes of the triangles that
    //   make up a triangulated frustum proj poly
    // * plane normals face outward
    // * triangulation is pre-determined; expects:
    //   - list_frustum_vx must have 8 vertices in CCW order
    std::vector<Plane>
    calcFrustumPolyTriPlanes(std::vector<osg::Vec3d> const &list_frustum_vx,
                             bool normalize) const;

    //
    osg::Vec3d calcTileClosestPoint(LLA const &lla_distal,
                                    osg::Vec3d const &ecef_distal,
                                    GeoBounds const &bounds,
                                    Eval const &eval) const;

    //
    double calcPixelsPerMeterForDist(double dist_m,
                                     double screen_height_px,
                                     osg::Camera const * cam) const;

    // options
    Options const m_opts;

    //
    double const m_tex_px2_m2;

    // view data
    osg::Camera const * m_cam;
    double const m_view_width;
    double const m_view_height;

    osg::Vec3d m_eye;
    LLA m_lla_eye;

    std::vector<osg::Vec3d> m_list_frustum_ecef;
    std::vector<LLA>        m_list_frustum_lla;
    std::vector<GeoBounds>  m_list_frustum_bounds;
    std::vector<Plane>      m_list_frustum_tri_planes;

    // tiles
    std::vector<std::unique_ptr<Tile>> m_list_root_tiles;
    std::map<uint64_t,Tile const *> m_list_tileset;

    std::list<std::unique_ptr<Eval>> m_lru_eval;
    std::map<TileId,std::list<std::unique_ptr<Eval>>::iterator> m_lkup_eval;


    std::map<uint64_t,Eval> m_list_tile_eval;

    // debug
    std::chrono::time_point<std::chrono::system_clock> m_start;

};

#endif // SCRATCH_TILESET_LL_BYPIXELLRES_H
