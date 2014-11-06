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
            list_base_levels({0,1}) // TODO: MUST BE SORTED
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

    osg::ref_ptr<osg::Group> createTileGm(TileLL const * tile,
                                          osg::ref_ptr<osg::Texture2D> const &tex,
                                          double const s_start,
                                          double const s_delta,
                                          double const t_start,
                                          double const t_delta);


    osg::ref_ptr<osg::Group> createTileTextGm(TileLL const * tile,
                                              std::string const &text);

    void moveReadyImagesToBaseTextures();
    void moveReadyImagesToViewTextures();

    void getTileTexture(osg::ref_ptr<osg::Texture2D> &texture,
                        double &s_start,
                        double &s_delta,
                        double &t_start,
                        double &t_delta);

    void applyDefaultTextureSettings(osg::Texture2D * texture) const;


    Options const m_opts;
    std::unique_ptr<TileSetLL> m_tileset;
    osg::Group * m_gp_tiles;
    std::unique_ptr<TileImageSourceLLLocal> m_tile_image_source;


    std::vector<uint8_t> m_list_level_is_base; // 0 = view, 1 = base

    // num_base_textures
    // * The number of preloaded static textures to use
    //   as a 'fallback' when a texture for a given tile
    //   isn't available (ie. if its loading)
    // * Generally contains textures close to the root
    // * Must be

    size_t m_num_base_textures;
    size_t m_max_view_textures;
    bool m_base_textures_loaded;

    // list_req_images
    // TODO want to rename to lkup_image_requests
    // * List of pending async requested images
    std::map<TileLL::Id,std::shared_ptr<TileImageSourceLL::Task>> m_lkup_req_images;

    // lkup_base_textures
    // * container for base textures with id based lookup
    std::map<TileLL::Id,osg::ref_ptr<osg::Texture2D>> m_lkup_base_textures;

    // lru_view_textures
    // * container for view textures with id based lookup
    // * container is a cache with a max capacity of
    //   (max_view_textures-num_base_textures)
    scratch::LRUCacheMap<TileLL::Id,osg::ref_ptr<osg::Texture2D>> m_lru_view_textures;

    // list_sg_tiles
    // * list of scenegraph tile data for all tiles
    std::map<TileLL::Id,osg::Group*> m_list_sg_tiles;

    std::vector<osg::ref_ptr<osg::Texture2D>> m_list_tile_level_tex;

    osg::PolygonMode * m_poly_mode;

    osg::ref_ptr<osg::Group> m_gp_debug;
};

#endif // SCRATCH_DATASET_TILES_LL_H
