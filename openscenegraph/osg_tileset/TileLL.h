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

#ifndef SCRATCH_TILE_LL_H
#define SCRATCH_TILE_LL_H

#include <memory>
#include <algorithm>
#include <limits>
#include <cassert>
#include <functional>

//
#include <GeometryUtils.h>

//
namespace scratch
{
    struct TileLL
    {
        typedef uint64_t Id;

        // Root tile constructor
        TileLL(GeoBounds const &bounds,
               uint32_t x,
               uint32_t y) :
            id(GetIdFromLevelXY(0,x,y)),
            level(0),
            x(x),
            y(y),
            bounds(bounds),
            parent(nullptr),
            clip(k_clip_NONE)

        {
            // empty
        }

        // Child tile constructor
        TileLL(TileLL * parent,
               uint32_t x,
               uint32_t y) :
            id(GetIdFromParentXY(parent,x,y)),
            level(parent->level + 1),
            x(x),
            y(y),
            bounds(GetBounds(parent,x,y)),
            parent(parent),
            clip(k_clip_NONE)
        {
            // empty
        }

        // unique id:
        // w  z  x      y
        // 00 FF FFFFFF FFFFFF
        // w: placeholder (not yet used)
        // z: tile level (8 bits)
        // x: tile x (24 bits)
        // y: tile y (24 bits)
        Id const id;

        // geography
        uint8_t  const level;
        uint32_t const x;
        uint32_t const y;
        GeoBounds const bounds;

        // clip states
        static const uint8_t k_clip_LT = 1 << 0;
        static const uint8_t k_clip_LB = 1 << 1;
        static const uint8_t k_clip_RB = 1 << 2;
        static const uint8_t k_clip_RT = 1 << 3;
        static const uint8_t k_clip_NONE = 0;
        static const uint8_t k_clip_ALL = 15;

        // quadtree relationships
        TileLL * parent;
        std::unique_ptr<TileLL> tile_LT;
        std::unique_ptr<TileLL> tile_LB;
        std::unique_ptr<TileLL> tile_RB;
        std::unique_ptr<TileLL> tile_RT;
        uint8_t clip;

        // generic data store that
        // must be implemented
        class Data
        {
        public:
            virtual ~Data() = 0;
        };
        std::unique_ptr<Data> data;

        // ============================================================= //

        // TODO Check if inlining these functions is worth it
        static Id GetIdFromLevelXY(uint8_t level,
                                   uint32_t x,
                                   uint32_t y);

        static Id GetIdFromParentXY(TileLL const * parent,
                                    uint32_t x,
                                    uint32_t y);

        static void GetLevelXYFromId(Id const id,
                                     uint8_t &level,
                                     uint32_t &x,
                                     uint32_t &y);

        static GeoBounds GetBounds(TileLL const * p,
                                   uint32_t x,
                                   uint32_t y);

//        // Comparators for sorting tiles
//        static bool CompareLevelIncreasing(TileLL const * a,
//                                           TileLL const * b)
//        {
//            return (a->level < b->level);
//        }

//        static bool CompareIdIncreasing(TileLL const * a,
//                                        TileLL const * b)
//        {
//            return (a->id < b->id);
//        }
    };
}

#endif // SCRATCH_TILE_LL_H
