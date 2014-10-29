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

#ifndef SCRATCH_TILESET_LL_H
#define SCRATCH_TILESET_LL_H

#include <memory>
#include <algorithm>

#include <osg/Camera>

template <typename T>
void SplitSets(std::vector<T> const &sorted_list_a,
               std::vector<T> const &sorted_list_b,
               std::vector<T> &list_diff_a,
               std::vector<T> &list_diff_b,
               std::vector<T> &list_xsec)
{
    list_diff_a.resize(sorted_list_a.size());
    list_diff_b.resize(sorted_list_b.size());
    list_xsec.resize(std::min(sorted_list_a.size(),
                              sorted_list_b.size()));

    typename std::vector<T>::iterator it;

    it = std::set_difference(sorted_list_a.begin(),
                             sorted_list_a.end(),
                             sorted_list_b.begin(),
                             sorted_list_b.end(),
                             list_diff_a.begin());
    list_diff_a.resize(it-list_diff_a.begin());

    it = std::set_difference(sorted_list_b.begin(),
                             sorted_list_b.end(),
                             sorted_list_a.begin(),
                             sorted_list_a.end(),
                             list_diff_b.begin());
    list_diff_b.resize(it-list_diff_b.begin());

    it = std::set_intersection(sorted_list_a.begin(),
                               sorted_list_a.end(),
                               sorted_list_b.begin(),
                               sorted_list_b.end(),
                               list_xsec.begin());
    list_xsec.resize(it-list_xsec.begin());
}

class TileSetLL
{
public:

    // ============================================================= //

    struct RootTileDesc
    {
        RootTileDesc(uint8_t root_id,
                     double min_lon,
                     double max_lon,
                     double min_lat,
                     double max_lat) :
            id(root_id),
            min_lon(min_lon),
            max_lon(max_lon),
            min_lat(min_lat),
            max_lat(max_lat)
        {}

        uint8_t const id;
        double const min_lon;
        double const max_lon;
        double const min_lat;
        double const max_lat;
    };

    // ============================================================= //

    class Tile
    {
    public:
        Tile(RootTileDesc const &r) :
            id(static_cast<uint64_t>(r.id) << 56),
            level(0),
            x(0),
            y(0),
            min_lon(r.min_lon),
            max_lon(r.max_lon),
            min_lat(r.min_lat),
            max_lat(r.max_lat),
            parent(nullptr),
            clip(k_clip_NONE)

        {
            // empty
            tile_px_res=-10;
        }

        Tile(Tile * parent,
             uint32_t x,
             uint32_t y) :
            id(getIdFromParentXY(parent,x,y)),
            level(parent->level + 1),
            x(x),
            y(y),
            min_lon(getLon(parent,x)),
            max_lon(getLon(parent,x+1)),
            min_lat(getLat(parent,y)),
            max_lat(getLat(parent,y+1)),
            parent(parent),
            clip(k_clip_NONE)
        {
            // empty
            tile_px_res=-10;
        }


        // unique id:
        // w  z  x      y
        // FF FF FFFFFF FFFFFF
        // w: root id (8 bits)
        // z: tile level (8 bits)
        // x: tile x (24 bits)
        // y: tile y (24 bits)
        uint64_t const id;

        // convenience
        uint8_t  const level;
        uint32_t const x;
        uint32_t const y;

        double const min_lon;
        double const max_lon;
        double const min_lat;
        double const max_lat;

        // quadtree relationships
        Tile * parent;
        std::unique_ptr<Tile> tile_LT;
        std::unique_ptr<Tile> tile_LB;
        std::unique_ptr<Tile> tile_RB;
        std::unique_ptr<Tile> tile_RT;

        // clip
        static const uint8_t k_clip_LT = 1 << 0;
        static const uint8_t k_clip_LB = 1 << 1;
        static const uint8_t k_clip_RB = 1 << 2;
        static const uint8_t k_clip_RT = 1 << 3;
        static const uint8_t k_clip_NONE = 0;
        static const uint8_t k_clip_ALL = 15;

        uint8_t clip;

        int64_t tile_px_res;

    private:
        static uint64_t getIdFromParentXY(Tile const * parent,
                                          uint32_t x,
                                          uint32_t y)
        {
            uint64_t root64 = parent->id;
            root64 = root64 >> 56;

            uint64_t level64 = parent->level + 1;
            uint64_t x64 = x;
            uint64_t y64 = y;
            uint64_t tile_id = 0;
            tile_id |= (root64 << 56);
            tile_id |= (level64 << 48);
            tile_id |= (x64 << 24);
            tile_id |= y64;

            return tile_id;
        }

        static double getLon(Tile const * p,
                             uint32_t x)
        {
            return p->min_lon + (p->max_lon-p->min_lon)*(x-p->x*2)*0.5;
        }

        static double getLat(Tile const * p,
                             uint32_t y)
        {
            return p->min_lat + (p->max_lat-p->min_lat)*(y-p->y*2)*0.5;
        }
    };

    // ============================================================= //

    virtual ~TileSetLL() {}

    virtual void UpdateTileSet(osg::Camera const * cam,
                               std::vector<uint64_t> &list_tiles_add,
                               std::vector<uint64_t> &list_tiles_upd,
                               std::vector<uint64_t> &list_tiles_rem) = 0;

    virtual Tile const * GetTile(uint64_t tile_id) const = 0;

    // ============================================================= //
};

#endif // SCRATCH_TILESET_LL_H
