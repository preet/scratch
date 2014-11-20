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

#ifndef SCRATCH_TILE_IMAGE_SOURCE_LL_H
#define SCRATCH_TILE_IMAGE_SOURCE_LL_H

#include <functional>

#include <TileDataSourceLL.h>

namespace scratch
{
    //
    class TileImageSourceLL : public TileDataSourceLL
    {
        //
        struct ImageData : public Data
        {
            ~ImageData();
            osg::ref_ptr<osg::Image> image;
        };

        //
        class ImageRequest : public Request
        {
        public:
            ImageRequest(TileLL::Id id,std::string path);
            ~ImageRequest();

            std::shared_ptr<Data> GetData() const;

            void Cancel();

        private:
            void process();

            std::string const m_path;
            std::shared_ptr<ImageData> m_data;
        };

    public:
        TileImageSourceLL(GeoBounds const &bounds,
                          uint8_t max_level,
                          uint8_t num_root_tiles_x,
                          uint8_t num_root_tiles_y,
                          std::function<std::string(TileLL::Id)> path_gen,
                          uint8_t num_threads=2);
        ~TileImageSourceLL();

        std::shared_ptr<Request> RequestData(TileLL::Id id);

    private:
        std::function<std::string(TileLL::Id)> m_path_gen;
        ThreadPool m_thread_pool;

    };

} // scratch


#endif // SCRATCH_TILE_IMAGE_SOURCE_LL_H
