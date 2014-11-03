#include<ThreadPool.h>

namespace scratch
{
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
}

int main()
{
    scratch::ThreadPool thread_pool(4);
    std::vector<std::shared_ptr<scratch::TaskIsPrime>> list_tasks;

    for(size_t i=0; i < 25; i++) {
        list_tasks.emplace_back(
                    std::make_shared<scratch::TaskIsPrime>(
                        i+1));

        // add task to pool
        thread_pool.Push(list_tasks.back());
    }

    // wait until all tasks are complete
    for(auto & task : list_tasks) {
        task->Wait();
        std::cout << "#: " << task->GetNumber() << " ";
        if(task->IsPrime()) {
            std::cout << "is prime" << std::endl;
        }
        else {
            std::cout << "is not prime" << std::endl;
        }
    }

    thread_pool.Stop();

    std::cout << "exiting..." << std::endl;

	return 0;
}
