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

#ifndef SCRATCH_TILE_VISIBILITY_LL_H
#define SCRATCH_TILE_VISIBILITY_LL_H

#include <osg/Camera>

#include <TileLL.h>

namespace scratch
{
    class TileVisibilityLL
    {
    public:
        virtual ~TileVisibilityLL()
        {
            // empty
        }

        virtual void Update(osg::Camera const * cam) = 0;

        virtual void GetVisibility(TileLL const * tile,
                                   bool & is_visible,
                                   bool & exceeds_err_threshold) = 0;
    };


} // scratch

#endif // SCRATCH_TILE_VISIBILITY_LL_H
