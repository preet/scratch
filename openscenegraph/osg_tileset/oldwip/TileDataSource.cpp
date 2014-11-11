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

#include <TileDataSource.h>

#include <osgDB/ReadFile>

// ============================================================= //

TileImageSourceLL::Task::Task(TileLL::Id id) :
    m_id(id)
{

}

TileImageSourceLL::Task::~Task()
{
    // empty
}

TileLL::Id TileImageSourceLL::Task::GetTileId() const
{
    return m_id;
}

// ============================================================= //

TileImageSourceLLLocal::TaskLocal::TaskLocal(TileLL::Id id,
                                             std::string path) :
    Task(id),
    m_path(std::move(path)),
    m_image(nullptr)
{
    // empty
}

TileImageSourceLLLocal::TaskLocal::~TaskLocal()
{
    // empty
}

osg::ref_ptr<osg::Image> TileImageSourceLLLocal::TaskLocal::GetImage() const
{
    if(this->IsFinished()) {
        return m_image;
    }

    return nullptr;
}

void TileImageSourceLLLocal::TaskLocal::Cancel()
{
    this->onCanceled();
}

void TileImageSourceLLLocal::TaskLocal::process()
{
    if(this->IsCanceled()) {
        return;
    }

    this->onStarted();
    m_image = osgDB::readImageFile(m_path);
    this->onFinished();
}

// ============================================================= //

TileImageSourceLLLocal::TileImageSourceLLLocal(std::function<std::string(TileLL::Id)> path_gen,
                                               uint8_t num_threads) :
    m_path_gen(std::move(path_gen)),
    m_thread_pool(num_threads)
{

}

TileImageSourceLLLocal::~TileImageSourceLLLocal()
{

}

std::shared_ptr<TileImageSourceLL::Task>
TileImageSourceLLLocal::RequestImage(TileLL::Id id)
{
    //
    std::shared_ptr<TaskLocal> task =
            std::make_shared<TaskLocal>(
                id,m_path_gen(id));

    m_thread_pool.Push(task);
    return task;
}
