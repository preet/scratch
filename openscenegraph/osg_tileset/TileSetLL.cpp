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

    TileSetLL::Options
    TileSetLL::validateOptions(TileDataSourceLL const * source,
                               Options opts)
    {
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
        if(opts.max_data > -1) {
            uint64_t preload_tiles = 0;
            for(auto level : opts.list_preload_levels) {
                preload_tiles += ipow(4,level);
            }
            if(preload_tiles > opts.max_data) {
                opts.max_data = preload_tiles;
            }
        }

        // TODO check upsampling hint

        return opts;
    }

} // scratch
