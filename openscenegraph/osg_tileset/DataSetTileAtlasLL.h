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

#include <TileSetLL.h>
#include <osg/PolygonMode>

#include <osg/Texture2D>

namespace scratch
{
    class TileAtlas;

    class DataSetTilesLL
    {
    public:
        DataSetTilesLL(osg::Group * gp_tiles);

        ~DataSetTilesLL();

        void Update(osg::Camera const * cam);

        void UpdateOld(osg::Camera const * cam);

    private:

        // scene graph
        struct SGData
        {
            osg::ref_ptr<osg::Group> gp;
            TileLL::Id sample_id;
        };

        //
        osg::ref_ptr<osg::Group>
        createTileGm(TileSetLL::TileItem const * tile_item);

        void applyTileTx(TileSetLL::TileItem const * tile_item,
                         osg::Group * gp);

        void applyAtlasTx(osg::Group * gp,
                          osg::Texture2D * atlas_texture,
                          osg::Vec4 const &atlas_region);

        //
        std::unique_ptr<TileSetLL> m_tileset;

        std::unique_ptr<TileAtlas> m_tile_atlas;

        osg::Group * m_gp_tiles;
        osg::ref_ptr<osg::PolygonMode> m_poly_mode;
        std::map<TileLL::Id,SGData> m_lkup_sg_tiles;

        size_t m_tile_count;
    };
}

#endif // SCRATCH_DATASET_TILES_LL_H
