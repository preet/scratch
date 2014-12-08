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

#include <ThreadPool.h>

#include <iostream>

namespace scratch
{
    // ============================================================= //

    ThreadPool::Task::Task() :
        m_started(false),
        m_running(false),
        m_canceled(false),
        m_finished(false),
        m_future(m_promise.get_future())
    {
        // empty
    }

    ThreadPool::Task::~Task()
    {
        // empty
    }

    bool ThreadPool::Task::IsStarted() const
    {
        return m_started;
    }

    bool ThreadPool::Task::IsRunning() const
    {
        return m_running;
    }

    bool ThreadPool::Task::IsCanceled() const
    {
        return m_canceled;
    }

    bool ThreadPool::Task::IsFinished() const
    {
        return m_finished;
    }

    void ThreadPool::Task::Wait() // TODO add duration wait
    {
        m_future.wait();
    }

    void ThreadPool::Task::onStarted()
    {
        m_started = true;
        m_running = true;
    }

    void ThreadPool::Task::onFinished()
    {
        m_running = false;
        m_finished = true;
        m_promise.set_value();
    }

    void ThreadPool::Task::onCanceled()
    {
        m_running = false;
        m_canceled = true;
        m_promise.set_value();
    }

    // ============================================================= //

    ThreadPool::ThreadPool(size_t thread_count) :
        m_thread_count(thread_count),
        m_running(false)
    {
        this->Resume();
    }

    ThreadPool::~ThreadPool()
    {
        this->Stop();
    }

    size_t ThreadPool::GetTaskCount() const
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_queue_tasks.size();
    }

    void ThreadPool::Push(std::shared_ptr<Task> const &task)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        // Add work to shared queue
        m_queue_tasks.push_back(task);

        // Wake one thread from the pool
        m_wait_cond.notify_one();
    }

    void ThreadPool::Stop()
    {
        if(m_running) {
            m_running = false;
            m_wait_cond.notify_all();

            for(auto & thread : m_list_threads) {
                thread.join();
            }
            m_list_threads.clear();
        }
    }

    void ThreadPool::Resume()
    {
        if(!m_running) {
            m_running = true;
            for(size_t i=0; i < m_thread_count; i++) {
                m_list_threads.emplace_back(&ThreadPool::loop,this);
            }
        }
    }

    void ThreadPool::loop()
    {
        while(m_running)
        {
            // acquire lock
            std::unique_lock<std::mutex> lock(m_mutex);

            while(m_running && m_queue_tasks.empty()) {
                // wait while there are no tasks to process
                m_wait_cond.wait(lock);
            }
            // wake-up automatically reacquires lock

            if(!m_running) {
                return;
            }

            // Take a task to process
            std::shared_ptr<Task> task = std::move(m_queue_tasks.front());
            m_queue_tasks.pop_front();

            lock.unlock(); // release lock

            // Process task
            task->Process();
        }
    }

    // ============================================================= //

} // scratch
