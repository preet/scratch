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

namespace scratch
{
    class DataSetTilesLL
    {
    public:
        DataSetTilesLL(std::unique_ptr<TileSetLL> tileset,
                       osg::Group * gp_tiles);

        ~DataSetTilesLL();

        void Update(osg::Camera const * cam);

    private:
        std::unique_ptr<TileSetLL> m_tileset;
        osg::Group * m_gp_tiles;
    };
}

#endif // SCRATCH_DATASET_TILES_LL_H
