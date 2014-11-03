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

#ifndef SCRATCH_THREAD_POOL_H
#define SCRATCH_THREAD_POOL_H

#include <vector>
#include <queue>
#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <iostream>
#include <future>

namespace scratch
{
	class ThreadPool
	{
    public:

        class Task
        {
        public:
            Task() :
                m_started(false),
                m_running(false),
                m_canceled(false),
                m_finished(false),
                m_future(m_promise.get_future())
            {
                // empty
            }

            virtual ~Task()
            {
                // empty
            }

            // No copying allowed
            Task(Task const &)              = delete;
            Task & operator=(Task const &)  = delete;

            bool IsStarted() const
            {
                return m_started;
            }

            bool IsRunning() const
            {
                return m_running;
            }

            bool IsCanceled() const
            {
                return m_canceled;
            }

            bool IsFinished() const
            {
                return m_finished;
            }

            virtual void Wait() // TODO add duration wait
            {
                m_future.wait();
            }

            virtual void Process() = 0;
            virtual void Cancel() = 0;

        protected:
            void onStarted()
            {
                m_started = true;
                m_running = true;
            }

            void onFinished()
            {
                m_running = false;
                m_finished = true;
                m_promise.set_value();
            }

            void onCanceled()
            {
                m_running = false;
                m_canceled = true;
                m_promise.set_value();
            }

        private:
            std::atomic<bool> m_started;
            std::atomic<bool> m_running;
            std::atomic<bool> m_canceled;
            std::atomic<bool> m_finished;

            std::promise<void> m_promise;
            std::future<void>  m_future;
        };

        ThreadPool(size_t thread_count) :
            m_thread_count(thread_count),
            m_running(false)
        {
            this->Resume();
        }

        ~ThreadPool()
        {
            this->Stop();
        }

        void Push(std::shared_ptr<Task> const &task)
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            // Add work to shared queue
            m_queue_tasks.push_back(task);

            // Wake one thread from the pool
            m_wait_cond.notify_one();
        }

        void Stop()
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

        void Resume()
        {
            if(!m_running) {
                m_running = true;
                for(size_t i=0; i < m_thread_count; i++) {
                    m_list_threads.emplace_back(&ThreadPool::loop,this);
                }
            }
        }

        // No copying allowed
        ThreadPool(ThreadPool const &)              = delete;
        ThreadPool & operator=(ThreadPool const &)  = delete;
		
    private:
        void loop()
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

        size_t m_thread_count;
        std::vector<std::thread> m_list_threads;
        std::deque<std::shared_ptr<Task>> m_queue_tasks;

        std::atomic<bool> m_running;
        std::mutex m_mutex;
        std::condition_variable m_wait_cond;
	};

} // scratch

#endif // SCRATCH_THREAD_POOL_H
