#include <iostream>
#include <chrono>

#include<ThreadPool.h>

namespace scratch
{
    // TaskIsPrime
    // * checks if a number is prime
    class TaskIsPrime : public ThreadPool::Task
    {
    public:
        TaskIsPrime(size_t num) :
            m_cancel(false),
            m_num(num)
        {
            // empty
        }

        ~TaskIsPrime()
        {
            // empty
        }

        void Process()
        {
            if(this->IsCanceled()) {
                return;
            }

            this->onStarted();
            m_result = true;
            for(size_t i=2; i < m_num; i++) {
                if(m_cancel) {
                    this->onCanceled();
                    return;
                }
                if(m_num%i == 0) {
                    m_result = false;
                    break;
                }
            }
            this->onFinished();
        }

        void Cancel()
        {
            // set cancel flag to break Process()
            m_cancel = true;
        }

        bool IsPrime(bool * valid=nullptr) const
        {
            if(valid) {
                *valid = this->IsFinished();
            }

            return m_result;
        }

        size_t GetNumber() const
        {
            return m_num;
        }

    private:
        std::atomic<bool> m_cancel;
        std::atomic<bool> m_result;
        size_t const m_num;
    };

    // TaskTimeSlice
    class TaskTimeSlice : public ThreadPool::Task
    {
    public:
        TaskTimeSlice(uint64_t ms) :
            m_ms(clamp(ms))
        {
            // empty
        }

        ~TaskTimeSlice()
        {

        }

        void Process()
        {
            if(this->IsCanceled()) {
                return;
            }

            this->onStarted();
            std::chrono::milliseconds duration(50);
            for(size_t i=0; i < m_ms/50; i++) {
                if(m_cancel) {
                    this->onCanceled();
                    return;
                }
                std::this_thread::sleep_for(duration);
            }
            this->onFinished();
        }

        void Cancel()
        {
            // set cancel flag to break Process()
            m_cancel = true;
        }

    private:
        size_t clamp(uint64_t ms) const
        {
            if(ms < 100) {
                ms = 100;
            }
            if(ms > 500) {
                ms = 500;
            }
            return ms;
        }

        std::atomic<bool> m_cancel;
        size_t const m_ms;
    };
}

void Test_PushTasksAndWait()
{
    std::cout << "Test_PushTasksAndWait... " << std::endl;

    scratch::ThreadPool thread_pool(4);
    std::vector<std::shared_ptr<scratch::TaskIsPrime>> list_tasks;

    for(size_t i=0; i < 100; i++) {
        list_tasks.emplace_back(
                    std::make_shared<scratch::TaskIsPrime>(
                        i+1));

        // add task to pool
        thread_pool.Push(list_tasks.back());
    }

    // wait until all tasks are complete
    size_t num_finished_tasks=0;
    for(auto & task : list_tasks) {
        task->Wait();
        if(task->IsFinished()) {
            num_finished_tasks++;
        }
    }

    if((thread_pool.GetTaskCount() == 0) &&
       (num_finished_tasks == 100)) {
        std::cout << ": [OK]" << std::endl;
    }
    else {
        std::cout << ": [ERR]" << std::endl;
    }
    std::cout << std::endl;
}

void Test_PushTasksAndCancel()
{
    std::cout << "Test_PushTasksAndCancel... " << std::endl;

    scratch::ThreadPool thread_pool(4);
    std::vector<std::shared_ptr<scratch::TaskTimeSlice>> list_tasks;

    for(size_t i=0; i < 100; i++) {
        list_tasks.emplace_back(
                    std::make_shared<scratch::TaskTimeSlice>(
                        500));

        // add task to pool
        thread_pool.Push(list_tasks.back());
    }

    // cancel all tasks
    for(auto & task : list_tasks) {
        task->Cancel();
    }

    // stop the thread pool
    thread_pool.Stop();

    size_t num_canceled_tasks=0;
    for(auto & task : list_tasks) {
        if(task->IsCanceled()) {
            num_canceled_tasks++;
        }
    }

    std::cout << ": task queue size: " << thread_pool.GetTaskCount() << std::endl;
    std::cout << ": canceled " << num_canceled_tasks << " tasks" << std::endl;

    if(num_canceled_tasks > 0) {
        std::cout << ": [OK]" << std::endl;
    }
    else {
        std::cout << ": [ERR]" << std::endl;
    }
    std::cout << std::endl;
}

void Test_PushTasksStopAndResume()
{
    std::cout << "Test_PushTasksStopAndResume... " << std::endl;

    scratch::ThreadPool thread_pool(4);
    std::vector<std::shared_ptr<scratch::TaskTimeSlice>> list_tasks;

    for(size_t i=0; i < 100; i++) {
        list_tasks.emplace_back(
                    std::make_shared<scratch::TaskTimeSlice>(
                        100));

        // add task to pool
        thread_pool.Push(list_tasks.back());
    }

    size_t const before_count = thread_pool.GetTaskCount();
    std::cout << ": task queue size before stop and resume: "
              << before_count << std::endl;

    // stop the thread pool
    // expect currently active tasks will complete
    thread_pool.Stop();

    // resume and wait for a few more tasks
    thread_pool.Resume();
    for(size_t i=0; i < 50; i++) {
        list_tasks[i]->Wait();
    }

    size_t const after_count = thread_pool.GetTaskCount();
    std::cout << ": task queue size after stop and resume: "
              << after_count << std::endl;

    if(after_count < before_count) {
        std::cout << ": [OK]" << std::endl;
    }
    else {
        std::cout << ": [ERR]" << std::endl;
    }
    std::cout << std::endl;
}

int main()
{
    Test_PushTasksAndWait();
    Test_PushTasksAndCancel();
    Test_PushTasksStopAndResume();

    std::cout << "exiting..." << std::endl;

	return 0;
}
