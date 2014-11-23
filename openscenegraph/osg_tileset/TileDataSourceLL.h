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

#ifndef SCRATCH_TILE_DATA_SOURCE_LL_H
#define SCRATCH_TILE_DATA_SOURCE_LL_H

#include <ThreadPool.h>
#include <TileLL.h>

namespace scratch
{
    class TileDataSourceLL
    {
    public:

        // ============================================================= //

        struct Data
        {
            virtual ~Data() {}
        };

        // ============================================================= //

        class Request : public ThreadPool::Task
        {
        public:
            Request(TileLL::Id id) :
                Task(id)
            {
                // empty
            }

            virtual ~Request()
            {
                // empty
            }

            TileLL::Id GetTileId() const
            {
                return this->GetId();
            }

            // TODO
            // should GetData() return a shared pointer?
            // why not just a raw pointer?
            virtual std::shared_ptr<Data> GetData() const=0;
        };

        // ============================================================= //

        TileDataSourceLL(GeoBounds const &bounds,
                         uint8_t max_level,
                         uint8_t num_root_tiles_x,
                         uint8_t num_root_tiles_y) :
            m_bounds(bounds),
            m_max_level(max_level),
            m_num_root_tiles_x(num_root_tiles_x),
            m_num_root_tiles_y(num_root_tiles_y)
        {
            // empty
        }

        virtual ~TileDataSourceLL()
        {
            // empty
        }

        GeoBounds const & GetBounds() const
        {
            return m_bounds;
        }

        uint8_t GetMaxLevel() const
        {
            return m_max_level;
        }

        uint8_t GetNumRootTilesX() const
        {
            return m_num_root_tiles_x;
        }

        uint8_t GetNumRootTilesY() const
        {
            return m_num_root_tiles_y;
        }

        virtual bool CanBeSampled() const = 0;

        virtual void StartRequestBlock() = 0;

        virtual void EndRequestBlock() = 0;

        // calls to RequestData must happen in between
        // a request block:
        // tile_data_source.StartRequestBlock();
        // auto req0 = tile_data_source.RequestData(...);
        // auto req1 = tile_data_source.RequestData(...);
        // auto req2 = tile_data_source.RequestData(...);
        // auto req3 = tile_data_source.RequestData(...);
        // tile_data_source.EndRequestBlock(); // sends the requests
        virtual std::shared_ptr<Request>
        RequestData(TileLL::Id id) = 0;

    private:
        GeoBounds const m_bounds;
        uint8_t const m_max_level;
        uint8_t const m_num_root_tiles_x;
        uint8_t const m_num_root_tiles_y;
    };

    // ============================================================= //

} // scratch

#endif // SCRATCH_TILE_DATA_SOURCE_LL_H
