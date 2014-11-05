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

#ifndef SCRATCH_DATASET_TILES_LL_H
#define SCRATCH_DATASET_TILES_LL_H

#include <DataSetTiles.h>
#include <TileSetLL.h>
#include <TileDataSource.h>

#include <osg/PolygonMode>
#include <osg/Texture2D>

class DataSetTilesLL : public DataSetTiles
{
public:
    struct Options
    {
        // sane defaults
        Options() :
            max_textures(64),
            list_base_levels({0,1})
        {}

        size_t max_textures;
        std::vector<uint8_t> list_base_levels;
    };

    DataSetTilesLL(Options const &opts,
                   std::unique_ptr<TileSetLL> tileset,
                   osg::Group * gp_tiles);

    ~DataSetTilesLL();

    void Update(osg::Camera const * cam);

private:
    osg::ref_ptr<osg::Group> createTileGm(TileLL::Id tile_id);
    osg::ref_ptr<osg::Group> createTileTextGm(TileLL const * tile,
                                              std::string const &text);


    Options const m_opts;
    std::unique_ptr<TileSetLL> m_tileset;
    osg::Group * m_gp_tiles;
    std::unique_ptr<TileImageSourceLLLocal> m_tile_image_source;


    // max_base_textures
    // * The number of preloaded static textures to use
    //   as a 'fallback' when a texture for a given tile
    //   isn't available (ie. if its loading)
    // * Generally contains textures close to the root
    // * Must be
    size_t m_num_base_textures;
    size_t m_max_view_textures;
    bool m_base_textures_loaded;



    std::vector<std::shared_ptr<TileImageSourceLL::Task>> m_list_req_images;

    //
    std::vector<osg::ref_ptr<osg::Texture2D>> m_list_base_textures;
    std::vector<osg::ref_ptr<osg::Texture2D>> m_list_view_textures;


    std::map<TileLL::Id,osg::Group*> m_list_sg_tiles;

    std::vector<osg::ref_ptr<osg::Texture2D>> m_list_tile_level_tex;

    osg::PolygonMode * m_poly_mode;

    osg::ref_ptr<osg::Group> m_gp_debug;
};

#endif // SCRATCH_DATASET_TILES_LL_H
