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

#include <TileDataSourceLL.h>

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

        // TODO desc
        // @tile:
        // * pointer to tile, *must* be valid
        //
        // @data:
        // * pointer to specific tile data (may be terrain geometry
        //   or height data) that can be used by the TileVisibility
        //   implementation.
        // * @data may be null, and the implementation must handle
        //   that case (ie use only @tile to provide meaningful results)
        //
        // @is_visible:
        // * if the tile is visible wrt the current camera
        //
        // @norm_error:
        // * normalized error, exceeds threshold if > 1.0
        // * invalid if is_visible==false
        //
        // @closest_point:
        // * the approximate closest point on the tile with respect
        //   to the camera eye
        virtual void GetVisibility(TileLL const * tile,
                                   TileDataSourceLL::Data const * data,
                                   bool & is_visible,
                                   double & norm_error,
                                   osg::Vec3d & closest_point) = 0;
    };


} // scratch

#endif // SCRATCH_TILE_VISIBILITY_LL_H
