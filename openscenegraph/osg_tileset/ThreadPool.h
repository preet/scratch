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
#include <list>
#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <future>
#include <set>

namespace scratch
{
	class ThreadPool
	{
    public:

        // ============================================================= //

        class Task
        {
            friend class ThreadPool;

        public:
            typedef uint64_t Id;

            Task(Task::Id id);
            virtual ~Task();

            // No copying allowed
            Task(Task const &)              = delete;
            Task & operator=(Task const &)  = delete;

            Id GetId() const
            {
                return m_id;
            }

            bool IsStarted() const;
            bool IsRunning() const;
            bool IsCanceled() const;
            bool IsFinished() const;

            // TODO add duration wait
            virtual void Cancel() = 0;
            void Wait();

        protected:
            void onStarted();
            void onFinished();
            void onCanceled();
            void onEnded();

        private:
            virtual void process() = 0;

            Id const m_id;

            std::atomic<bool> m_started;
            std::atomic<bool> m_running;
            std::atomic<bool> m_canceled;
            std::atomic<bool> m_finished;
            std::promise<void> m_promise;
            std::future<void>  m_future;

            std::mutex m_mutex_promise;
        };

        // ============================================================= //

        ThreadPool(size_t thread_count);
        ~ThreadPool();

        // No copying allowed
        ThreadPool(ThreadPool const &)              = delete;
        ThreadPool & operator=(ThreadPool const &)  = delete;

        size_t GetTaskCount() const;
        std::vector<Task::Id> GetTaskIdList() const;
        void PushFront(std::shared_ptr<Task> const &task);
        void PushBack(std::shared_ptr<Task> const &task);
        void PushFront(std::vector<std::shared_ptr<Task>> const &list_tasks);
        void PushBack(std::vector<std::shared_ptr<Task>> const &list_tasks);

//        // TODO maybe make this into a template function that
//        // accepts any kind of container?
//        template<typename ForwardIterator>
//        void PushFront(ForwardIterator tasks_begin,
//                       ForwardIterator tasks_end)
//        {
//            // use static assert to ensure iterator type
//            // is for a container that contains something that
//            // is shared_ptr<DerivedFromTask>
//        }

//        template<typename ForwardIterator>
//        void PushBack(ForwardIterator tasks_begin,
//                      ForwardIterator tasks_end)
//        {
//            // use static assert to ensure iterator type
//            // is for a container that contains something that
//            // is shared_ptr<DerivedFromTask>
//        }

        void Stop();
        void Resume();

    private:
        void loop();

        size_t m_thread_count;
        std::vector<std::thread> m_list_threads;
        std::list<std::shared_ptr<Task>> m_queue_tasks;

        std::atomic<bool> m_running;
        mutable std::mutex m_mutex;
        std::condition_variable m_wait_cond;

        // ============================================================= //
    };

} // scratch

#endif // SCRATCH_THREAD_POOL_H
