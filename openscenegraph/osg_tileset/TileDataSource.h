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

#ifndef SCRATCH_TILE_DATA_SOURCE_H
#define SCRATCH_TILE_DATA_SOURCE_H

#include <functional>

#include <ThreadPool.h>
#include <TileSetLL.h>

// ============================================================= //

class TileImageSourceLL
{
public:
    //
    class Task : public scratch::ThreadPool::Task
    {
    public:
        Task(TileLL::Id id);
        virtual ~Task();

        TileLL::Id GetTileId() const;
        virtual osg::ref_ptr<osg::Image> GetImage() const=0;

    private:
        TileLL::Id const m_id;
    };

    virtual ~TileImageSourceLL() {}

    virtual std::shared_ptr<Task> RequestImage(TileLL::Id id) = 0;
};

// ============================================================= //

class TileImageSourceLLLocal : public TileImageSourceLL
{
    //
    class TaskLocal : public Task
    {
    public:
        TaskLocal(TileLL::Id id,
                  std::string path);
        ~TaskLocal();

        osg::ref_ptr<osg::Image> GetImage() const;

        void Cancel();

    private:
        void process();

        std::string const m_path;
        osg::ref_ptr<osg::Image> m_image;
    };

public:
    TileImageSourceLLLocal(std::function<std::string(TileLL::Id)> path_gen,
                           uint8_t num_threads=2);
    ~TileImageSourceLLLocal();

    std::shared_ptr<Task> RequestImage(TileLL::Id id);

private:
    std::function<std::string(TileLL::Id)> m_path_gen;
    scratch::ThreadPool m_thread_pool;

};

// ============================================================= //

#endif // SCRATCH_TILE_DATA_SOURCE_H
