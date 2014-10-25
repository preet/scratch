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

#ifndef SCRATCH_TILESET_LL_BYPIXELAREA_H
#define SCRATCH_TILESET_LL_BYPIXELAREA_H

#include <TileSetLL.h>
#include <GeometryUtils.h>

class TileSetLLByPixelArea : public TileSetLL
{
public:

    struct Options
    {
        // Initialize to sane defaults
        Options() :
            max_tiles(96),
            tile_sz_px(256),
            min_level(0),
            max_level(18),
            fix_level(1),
            preload_eval_level(2),
            max_adj_offset_dist2_m(1000000.0*1000000.0),
            min_eval_angle_degs(360.0/32.0)

        {}

        size_t max_tiles;
        size_t tile_sz_px;
        size_t min_level;
        size_t max_level;
        size_t fix_level;
        size_t preload_eval_level;
        double max_adj_offset_dist2_m;
        double min_eval_angle_degs;
    };

    TileSetLLByPixelArea(double view_width,
                         double view_height,
                         Options const &opts,
                         std::vector<RootTileDesc> const &list_root_tiles);

    ~TileSetLLByPixelArea();

    void UpdateTileSet(osg::Camera const * cam,
                       std::vector<uint64_t> &list_tiles_add,
                       std::vector<uint64_t> &list_tiles_upd,
                       std::vector<uint64_t> &list_tiles_rem);

    Tile const * GetTile(uint64_t tile_id) const;

    // debug
    GeoBounds m_debug0;
    GeoBounds m_debug1;

private:

    struct Eval
    {
        Eval(GeoBounds const &bounds,
             osg::Matrixd const &mvp,
             double min_angle_degs);

        // Evaluation geometry
        // * to approximate pixel area taken up by tile
        std::vector<osg::Vec3d> list_vx;
        std::vector<LLA>        list_lla;
        std::vector<osg::Vec2d> list_ndc;
        std::vector<uint16_t>   list_ix;
        std::vector<osg::Vec3d> list_quad_nx;
    };

    void buildTileSet(std::unique_ptr<Tile> &tile,
                      size_t &tile_count);

    void buildTileSetList(std::unique_ptr<Tile> const &tile,
                          std::map<uint64_t,Tile const *> &list_tiles);

    bool tilePxlAreaExceedsRes(Tile const * tile);

    // * calculates the NDC area of a tile for quads
    //   that are visible and in front of the near plane
    // * uses quads (ie, two triangles at once) to avoid
    //   asymmetry issues
    void calcTileNDCArea(Eval const &eval,
                         double &ndc_area,
                         double &surf_area_m2) const;

    // UNUSED
    // * calculate the NDC area of every quad in the tile
    double calcTileNDCAreaQuad(Eval const &eval,
                               Plane const &clip_plane,
                               bool cull_hidden_tris=true) const;

    // UNUSED
    // * calculate the NDC area of a single triangle
    // * tiles outside the NDC rectangle will be ignored
    // * tris will be clipped against clip_plane
    double calcTriangleNDCAreaVisible(Plane const &clip_plane,
                                      osg::Matrixd const &mvp,
                                      std::vector<osg::Vec3d> const &tri,
                                      std::vector<osg::Vec2d> const &tri_ndc) const;

    // UNUSED
    // * calculate the NDC area of a single triangle
    // * tiles outside the NDC rectangle will be included
    // * will be clipped against clip_plane
    double calcTriangleNDCAreaFull(Plane const &clip_plane,
                                   osg::Matrixd const &mvp,
                                   std::vector<osg::Vec3d> const &tri,
                                   std::vector<osg::Vec2d> const &tri_ndc) const;

    // options
    Options const m_opts;
    double const m_tile_px_area;

    // view data
    osg::Camera const * m_cam;
    double const m_view_width;
    double const m_view_height;
    osg::Matrixd m_mvp;
    osg::Vec3d m_view_dirn;
    Plane m_near_plane;

    std::vector<osg::Vec3d> m_list_frustum_ecef;
    std::vector<LLA> m_list_frustum_lla;
    std::vector<GeoBounds> m_list_frustum_bounds;
    uint8_t m_frustum_pole;

    // tiles
    std::vector<std::unique_ptr<Tile>> m_list_root_tiles;
    std::map<uint64_t,Tile const *> m_list_tileset;
    size_t m_tile_count;

    // clear list when full
    std::map<uint64_t,Eval> m_list_tile_eval;

};

#endif // SCRATCH_TILESET_LL_BYPIXELAREA_H
