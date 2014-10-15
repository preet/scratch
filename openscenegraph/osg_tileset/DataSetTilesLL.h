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

#include <osg/PolygonMode>
#include <osg/Texture2D>

class DataSetTilesLL : public DataSetTiles
{
public:
    DataSetTilesLL(osg::Group * gp_tiles,
                   std::unique_ptr<TileSetLL> tileset);

    ~DataSetTilesLL();

    void Update(osg::Camera const * cam);

private:
    osg::ref_ptr<osg::Group> createTileGm(uint64_t tile_id);

    osg::Group * m_gp_tiles;
    std::unique_ptr<TileSetLL> m_tileset;
    std::map<uint64_t,osg::Group*> m_list_sg_tiles;

    std::vector<osg::ref_ptr<osg::Texture2D>> m_list_tile_level_tex;

    osg::PolygonMode * m_poly_mode;
};

#endif // SCRATCH_DATASET_TILES_LL_H
