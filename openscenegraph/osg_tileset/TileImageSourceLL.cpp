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

#include <thread>

#include <TileImageSourceLL.h>

#include <osgDB/ReadFile>

namespace scratch
{
    TileImageSourceLL::ImageData::~ImageData()
    {
        // empty
    }

    // ============================================================= //

    TileImageSourceLL::ImageRequest::ImageRequest(TileLL::Id id, std::string path) :
        TileDataSourceLL::Request(id),
        m_path(path)
    {
        // empty
    }

    TileImageSourceLL::ImageRequest::~ImageRequest()
    {
        // empty
        if(!this->IsFinished()) {
            Cancel();
        }
    }

    void TileImageSourceLL::ImageRequest::Cancel()
    {
        this->onCanceled();
    }

    void TileImageSourceLL::ImageRequest::process()
    {
        if(this->IsCanceled()) {
            return;
        }

        this->onStarted();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        m_data = std::make_shared<ImageData>();
        //m_data->image = osgDB::readImageFile(m_path);
        this->onFinished();
    }

    bool TileImageSourceLL::CanBeSampled() const
    {
        return true;
    }

    std::shared_ptr<TileDataSourceLL::Data>
    TileImageSourceLL::ImageRequest::GetData() const
    {
        return m_data;
    }

    // ============================================================= //

    TileImageSourceLL::TileImageSourceLL(GeoBounds const &bounds,
                                         uint8_t max_level,
                                         uint8_t num_root_tiles_x,
                                         uint8_t num_root_tiles_y,
                                         std::function<std::string(TileLL::Id)> path_gen,
                                         uint8_t num_threads) :
        TileDataSourceLL(bounds,
                         max_level,
                         num_root_tiles_x,
                         num_root_tiles_y),
        m_path_gen(std::move(path_gen)),
        m_thread_pool(num_threads)
    {
        // empty
    }

    TileImageSourceLL::~TileImageSourceLL()
    {
        // empty
    }

    std::shared_ptr<TileDataSourceLL::Request>
    TileImageSourceLL::RequestData(TileLL::Id id)
    {
//        std::cout << "###: task_count " << m_thread_pool.GetTaskCount() << std::endl;

        std::shared_ptr<ImageRequest> request =
                std::make_shared<ImageRequest>(
                    id,m_path_gen(id));

        m_thread_pool.PushFront(request);
        return request;
    }

} // scratch
