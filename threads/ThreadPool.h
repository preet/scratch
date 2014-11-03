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
#include <deque>
#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <future>

namespace scratch
{
	class ThreadPool
	{
    public:

        // ============================================================= //

        class Task
        {
        public:
            Task();
            virtual ~Task();

            // No copying allowed
            Task(Task const &)              = delete;
            Task & operator=(Task const &)  = delete;

            bool IsStarted() const;
            bool IsRunning() const;
            bool IsCanceled() const;
            bool IsFinished() const;

            // TODO add duration wait
            void Wait();

            virtual void Process() = 0;
            virtual void Cancel() = 0;

        protected:
            void onStarted();
            void onFinished();
            void onCanceled();

        private:
            std::atomic<bool> m_started;
            std::atomic<bool> m_running;
            std::atomic<bool> m_canceled;
            std::atomic<bool> m_finished;

            std::promise<void> m_promise;
            std::future<void>  m_future;
        };

        // ============================================================= //

        ThreadPool(size_t thread_count);
        ~ThreadPool();

        // No copying allowed
        ThreadPool(ThreadPool const &)              = delete;
        ThreadPool & operator=(ThreadPool const &)  = delete;

        size_t GetTaskCount() const;
        void Push(std::shared_ptr<Task> const &task);
        void Stop();
        void Resume();
		
    private:
        void loop();

        size_t m_thread_count;
        std::vector<std::thread> m_list_threads;
        std::deque<std::shared_ptr<Task>> m_queue_tasks;

        std::atomic<bool> m_running;
        mutable std::mutex m_mutex;
        std::condition_variable m_wait_cond;

        // ============================================================= //
	};

} // scratch

#endif // SCRATCH_THREAD_POOL_H
