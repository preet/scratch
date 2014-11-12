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

#include <TileSetLL.h>

namespace scratch
{
    TileSetLL::TileSetLL(std::unique_ptr<TileDataSourceLL> tile_data_source,
                         std::unique_ptr<TileVisibilityLL> tile_visibility,
                         Options options) :
        m_tile_data_source(std::move(tile_data_source)),
        m_tile_visibility(std::move(tile_visibility)),
        m_opts(validateOptions(m_tile_data_source.get(),options))
    {


        // Generate root tiles from the number of
        // root tiles in x and y
        auto const bounds = this->GetBounds();
        auto const num_root_tiles_x = this->GetNumRootTilesX();
        auto const num_root_tiles_y = this->GetNumRootTilesY();

        double const lon_width =
                (bounds.maxLon-bounds.minLon)/num_root_tiles_x;

        double const lat_width =
                (bounds.maxLat-bounds.minLat)/num_root_tiles_y;

        for(size_t y=0; y < num_root_tiles_y; y++) {
            for(size_t x=0; x < num_root_tiles_x; x++) {
                // Create bounds for this tile
                GeoBounds b(bounds.minLon,
                            bounds.minLon+(lon_width*(x+1)),
                            bounds.minLat,
                            bounds.minLat+(lat_width*(y+1)));

                // save
                m_list_root_tiles.emplace_back(new TileLL(b,x,y));
            }
        }


        // Preload the base textures
        m_preloaded_data_ready = false;

        m_list_level_is_preloaded.resize(
                    m_opts.list_preload_levels.size(),0);

        for(auto level : m_opts.list_preload_levels) {
            // mark as a preload level
            m_list_level_is_preloaded[level] = 1;

            // request data for all tiles in this level
            auto const tiles_in_x = ipow(2,level)*num_root_tiles_x;
            auto const tiles_in_y = ipow(2,level)*num_root_tiles_y;

            for(int64_t y=0; y < tiles_in_y; y++) {
                for(int64_t x=0; x < tiles_in_x; x++) {
                    // save request
                    auto const tile_id =
                            TileLL::GetIdFromLevelXY(level,x,y);

                    m_lkup_preloaded_data.emplace(
                                tile_id,
                                m_tile_data_source->RequestData(
                                    tile_id));
                }
            }
        }       
    }

    TileSetLL::~TileSetLL()
    {
        // empty
    }

    GeoBounds const & TileSetLL::GetBounds() const
    {
        return m_tile_data_source->GetBounds();
    }

    uint8_t TileSetLL::GetMinLevel() const
    {
        return m_opts.min_level;
    }

    uint8_t TileSetLL::GetMaxLevel() const
    {
        return m_opts.max_level;
    }

    uint8_t TileSetLL::GetNumRootTilesX() const
    {
        return m_tile_data_source->GetNumRootTilesX();
    }

    uint8_t TileSetLL::GetNumRootTilesY() const
    {
        return m_tile_data_source->GetNumRootTilesY();
    }

    void TileSetLL::UpdateTileSet(osg::Camera const * cam,
                                  std::vector<TileItem> &list_tiles_add,
                                  std::vector<TileItem> &list_tiles_upd,
                                  std::vector<TileItem> &list_tiles_rem)
    {
        // Ensure the base data has been loaded
        if(!m_preloaded_data_ready) {
            for(auto const &id_req : m_lkup_preloaded_data) {
                if(!id_req.second->IsFinished()) {
                    // We don't do anything until all of
                    // the base data is finished loading
                    return;
                }
            }
            m_preloaded_data_ready = true;
        }


        //
    }

    void TileSetLL::buildTileSetBFS()
    {
        // Build the tileset by doing a breadth first search
        // on all of the root tiles
        std::vector<TileLL *> list_tiles;

        for(auto & tile : m_list_root_tiles) {

            if(list_tiles.size() == m_opts.max_data) {
                return;
            }

            // Get visibility
            bool is_visible,exceeds_err;
            m_tile_visibility->GetVisibility(
                        tile.get(),is_visible,exceeds_err);

            // Save if visible
            if(is_visible) {

            }
        }

        for(size_t i=0; i < list_tiles.size(); i++) {
            //

        }
    }

    TileDataSourceLL::Request const *
    TileSetLL::getOrCreateDataRequest(TileLL const * tile)
    {
        // Check if this tile data has been preloaded
        if(m_list_level_is_preloaded[tile->level]) {
            auto it = m_lkup_preloaded_data.find(tile->id);
            return it->second.get();
        }

        // Create the request if it doesn't exist in cache
        bool exists;
        TileDataSourceLL::Request * data_req =
                m_lru_view_data.get(tile->id,true,exists).get();

        if(!exists) {
            m_lru_view_data.insert(
                        tile->id,
                        m_tile_data_source->RequestData(tile->id),
                        false);

            data_req = m_lru_view_data.get(
                        tile->id,false,exists).get();
        }

        return data_req;
    }

    TileSetLL::Options
    TileSetLL::validateOptions(TileDataSourceLL const * source,
                               Options opts)
    {
        // max_data must be less than the max possible
        // value of its data type to allow for safe
        // comparisons (ie, if(x > max_data))
        assert(m_opts.max_data < std::numeric_limits<uint64_t>::max());

        // Ensure min and max levels are between
        // corresponding source levels
        if(opts.min_level > opts.max_level) {
            std::swap(opts.min_level,opts.max_level);
        }
        else if(opts.min_level == opts.max_level) {
            opts.max_level++;
        }

        opts.min_level = clamp(opts.min_level,
                               source->GetMinLevel(),
                               source->GetMaxLevel());

        opts.max_level = clamp(opts.max_level,
                               source->GetMinLevel(),
                               source->GetMaxLevel());

        // The preload level list must be sorted
        // in increasing order
        std::sort(opts.list_preload_levels.begin(),
                  opts.list_preload_levels.end());

        // max_data must be equal to or larger than
        // the total number of preload tiles if a valid
        // limit is set
        uint64_t num_base_data = 0;
        for(auto level : opts.list_preload_levels) {
            num_base_data += ipow(4,level);
        }

        if(num_base_data > opts.max_data) {
            opts.max_data = num_base_data;
        }

        // TODO check upsampling hint

        return opts;
    }


} // scratch
