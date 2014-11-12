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

#include <unordered_map>
#include <MiscUtils.h>
#include <TileDataSourceLL.h>
#include <TileVisibilityLL.h>

namespace scratch
{
    class TileSetLL
    {
    public:
        struct TileItem
        {
            TileLL const * tile;
            std::shared_ptr<TileDataSourceLL::Data> data;

            // sample data (s_start,s_delta,t_start,t_delta)
            // also needs to go here
        };

        struct Options
        {
            // sane defaults
            Options() :
                min_level(0),
                max_level(18),
                max_data(std::numeric_limits<uint64_t>::max()/2),
                cache_size_hint(128),
                list_preload_levels({0,1}),
                upsample_hint(false)
            {
                // empty
            }

            uint8_t min_level;
            uint8_t max_level;

            // The max number of data associated to a tile
            // (textures, geometry, etc) from TileDataSource
            // thats allowed.
            uint64_t max_data;

            // Hint for amount of TileDataSource::Data that is cached.
            // The amount may be exceeded if the number of visible tiles
            // during a given update is greater than the size hint.
            uint64_t cache_size_hint;

            // List of levels to preload tile data from.
            // max_data includes preloaded data.
            std::vector<uint8_t> list_preload_levels;

            // Upsample data for individual TileItems from parent
            // tiles if its own data isn't available yet. The
            // TileDataSource implementation must allow sampling.
            bool upsample_hint;
        };

        TileSetLL(std::unique_ptr<TileDataSourceLL> tile_data_source,
                  std::unique_ptr<TileVisibilityLL> tile_visibility,
                  Options options);

        ~TileSetLL();

        GeoBounds const & GetBounds() const;

        uint8_t GetMinLevel() const;

        uint8_t GetMaxLevel() const;

        uint8_t GetNumRootTilesX() const;

        uint8_t GetNumRootTilesY() const;

        void UpdateTileSet(osg::Camera const * cam,
                           std::vector<TileItem> &list_tiles_add,
                           std::vector<TileItem> &list_tiles_upd,
                           std::vector<TileItem> &list_tiles_rem);

    private:
        void buildTileSetBFS();
        void buildTileSetRanked();

        TileDataSourceLL::Request const *
        getOrCreateDataRequest(TileLL const * tile);

        static Options validateOptions(TileDataSourceLL const * source,
                                       Options opts);



        //
        std::unique_ptr<TileDataSourceLL> m_tile_data_source;
        std::unique_ptr<TileVisibilityLL> m_tile_visibility;
        Options const m_opts;


        //
        std::vector<std::unique_ptr<TileLL>> m_list_root_tiles;

        // 0 = view, 1 = base
        std::vector<uint8_t> m_list_level_is_preloaded;

        // lkup_preloaded_data
        std::map<
            TileLL::Id,
            std::shared_ptr<TileDataSourceLL::Request>
            > m_lkup_preloaded_data;

        // lru_view_data
        LRUCacheMap<
            TileLL::Id,
            std::shared_ptr<TileDataSourceLL::Request>,
            std::map
            > m_lru_view_data;

        bool m_preloaded_data_ready;
    };
}

#endif // SCRATCH_TILESET_LL_H
