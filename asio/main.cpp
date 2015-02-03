#include <chrono>
#include <thread>
#include <mutex>
#include <iostream>
#include <memory>
#include <thirdparty/asio/asio.hpp>

class Thread;

struct CountHandler
{
    CountHandler(int * count,
                 std::mutex * mutex,
                 std::chrono::milliseconds sleep_ms) :
        count(count),
        mutex(mutex),
        sleep_ms(sleep_ms)
    {
        // empty
    }

    CountHandler(int * count,
                 std::mutex * mutex,
                 std::chrono::milliseconds sleep_ms,
                 std::shared_ptr<Thread> thread) :
        count(count),
        mutex(mutex),
        sleep_ms(sleep_ms),
        thread(thread)
    {
        // empty
    }

    void operator()()
    {
        std::lock_guard<std::mutex> lock(*mutex);
        (*count) += 1;

        std::this_thread::sleep_for(sleep_ms);
    }

    int * count;
    std::mutex * mutex;
    std::chrono::milliseconds sleep_ms;
    std::shared_ptr<Thread> thread;
};

struct DefaultHandler
{
    DefaultHandler(std::string name) :
        name(name)
    {
        // empty
    }

    void operator()()
    {
        std::cout << " { ";
        std::cout << name;
        std::cout << " } ";
    }

    std::string name;
};

struct CleanupHandler
{
    CleanupHandler(std::string name,
                   asio::io_service * service) :
        name(name),
        service(service)
    {

    }

    void operator()()
    {
        std::cout << " { ";
        std::cout << name;

        service->stop();

        std::cout << " } ";
    }

    std::string name;
    asio::io_service * service;
};


//

class EventLoop
{
public:
    EventLoop() :
        m_running(false)
    {
        // empty
    }

    bool IsRunning()
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_running;
    }

    void BeforeRun()
    {
        // Check if the event loop is already running
        // and create work if necessary
        std::lock_guard<std::mutex> lock(m_mutex);

        if(m_running || m_asio_work) {
            return;
        }

        m_asio_work.reset(
                    new asio::io_service::work(
                        m_asio_service));

        m_running = true;
    }

    void Run()
    {
        // Enter the event loop
        m_asio_service.run();

        // Reset when leaving so we can call
        // io_service::run() again
        m_asio_service.reset();
    }

    void Stop()
    {
        std::lock_guard<std::mutex> lock(m_mutex);

        if(!(m_running && m_asio_work)) {
            return;
        }

        m_asio_work.reset(nullptr);
        m_asio_service.stop();
        m_running = false;
    }

    void PostHandler(CountHandler handler)
    {
        m_asio_service.post(handler);
    }

    void PostHandler(DefaultHandler handler)
    {
        m_asio_service.post(handler);
    }

    void PostHandler(CleanupHandler handler)
    {
        m_asio_service.post(handler);
    }

    void PostStopEvent()
    {
        m_asio_service.post(std::bind(&EventLoop::Stop,this));
    }

private:
    bool m_running;
    std::mutex m_mutex;
    asio::io_service m_asio_service;
    std::unique_ptr<asio::io_service::work> m_asio_work;
};

//

class Thread
{
public:
    Thread() : m_event_loop(new EventLoop)
    {

    }

    ~Thread()
    {
        // Its always safe to call Stop, regardless
        // of whether or not m_thread exists or if
        // ~Thread is called from m_thread's context
        this->Stop();

        bool destroying_self;
        {
            std::lock_guard<std::mutex> lock(m_thread_mutex);
            if(!m_thread) {
                return;
            }
            destroying_self = (m_thread->get_id() == std::this_thread::get_id());
        }

        if(!destroying_self) {
            this->Wait();
        }
        else {
            std::lock_guard<std::mutex> lock(m_thread_mutex);
            m_thread->detach();
            m_thread.reset(nullptr);
        }
    }

    void Start()
    {
        std::lock_guard<std::mutex> lock(m_thread_mutex);

        if(m_thread) {
            if(m_event_loop->IsRunning()) {
                std::cout << "warn: thread already running" << std::endl;
                return;
            }
            else {
                this->Wait();
            }
        }

        if(!m_thread) {
            m_event_loop->BeforeRun();
            m_thread.reset(new std::thread(&Thread::run,this));
        }
    }

    void Stop(bool process_waiting=false)
    {
        if(process_waiting) {
            // Post a stop event so that the events
            // currently on the queue are processed
            m_event_loop->PostStopEvent();
        }
        else {
            m_event_loop->Stop();
        }
    }

    void Wait()
    {
        std::lock_guard<std::mutex> lock(m_thread_mutex);

        if(!m_thread) {
            return;
        }

        // If wait is being called from the event loop thread
        if(m_thread->get_id() == std::this_thread::get_id()) {
            std::cout << "warn: thread called wait on itself" << std::endl;
            return; // should i fail catastrophically?
        }

        m_thread->join();
        m_thread.reset(nullptr);
    }

    void PostHandler(CountHandler handler)
    {
        m_event_loop->PostHandler(handler);
    }

    void PostHandler(DefaultHandler handler)
    {
        m_event_loop->PostHandler(handler);
    }

    void PostHandler(CleanupHandler handler)
    {
        m_event_loop->PostHandler(handler);
    }

private:
    void run()
    {
        std::shared_ptr<EventLoop> event_loop = m_event_loop;
        event_loop->Run();
    }

    std::unique_ptr<std::thread> m_thread;
    std::mutex m_thread_mutex;

    std::shared_ptr<EventLoop> m_event_loop;
};

void TrivialConstructionAndDestruction()
{
    std::shared_ptr<Thread> thread =
            std::make_shared<Thread>();
}

void PostHandlersWhenInactive()
{
    int count=0;
    std::mutex m;

    {
        std::shared_ptr<Thread> thread =
                std::make_shared<Thread>();

        std::chrono::milliseconds sleep_ms(0);
        thread->PostHandler(CountHandler(&count,&m,sleep_ms));
        thread->PostHandler(CountHandler(&count,&m,sleep_ms));
        thread->PostHandler(CountHandler(&count,&m,sleep_ms));
    }

    // Expect that none of the handlers get invoked
    // event when wait() is called since the event
    // loop is inactive
    assert(count==0);
}

void TestStop()
{
    int count=0;
    std::mutex m;

    {
        std::shared_ptr<Thread> thread =
                std::make_shared<Thread>();

        std::chrono::milliseconds sleep_ms(10);
        thread->PostHandler(CountHandler(&count,&m,sleep_ms));
        thread->PostHandler(CountHandler(&count,&m,sleep_ms));
        thread->PostHandler(CountHandler(&count,&m,sleep_ms));
        thread->Start();
    }
    // ~Thread calls Thread::Stop() and Thread::Wait()

    // Expect that most of the handlers get dropped
    // since Stop() should cancel them
    assert(count==0 || count==1);
}

void TestWait()
{
    int count=0;
    std::mutex m;

    {
        std::shared_ptr<Thread> thread =
                std::make_shared<Thread>();

        std::chrono::milliseconds sleep_ms(10);
        thread->PostHandler(CountHandler(&count,&m,sleep_ms));
        thread->PostHandler(CountHandler(&count,&m,sleep_ms));
        thread->PostHandler(CountHandler(&count,&m,sleep_ms));
        thread->Start();
        thread->Stop(true);
        thread->Wait();
    }

    // Expect that most of the handlers get dropped
    // since Stop() should cancel them
    assert(count==3);
}

void TestStartStartStartStopStopStop()
{
    int count=0;
    std::mutex m;

    {
        std::shared_ptr<Thread> thread =
                std::make_shared<Thread>();

        std::chrono::milliseconds sleep_ms(10);
        thread->PostHandler(CountHandler(&count,&m,sleep_ms));
        thread->PostHandler(CountHandler(&count,&m,sleep_ms));
        thread->PostHandler(CountHandler(&count,&m,sleep_ms));
        thread->Start();
        thread->Start();
        thread->Start();
        thread->Stop();
        thread->Stop();
        thread->Stop();
    }
}

void TestExtendedLifetime()
{
    int count=0;
    std::mutex m;

    {
        std::shared_ptr<Thread> thread =
                std::make_shared<Thread>();

        std::chrono::milliseconds sleep_ms(10);
        // By passing the thread shared_ptr to a posted
        // handler, we extend the lifetime of the thread
        // beyond this scope. We expect ~Thread to detach
        // the thread instead of joining as a result.
        thread->PostHandler(CountHandler(&count,&m,sleep_ms,thread));
        thread->PostHandler(CountHandler(&count,&m,sleep_ms));
        thread->PostHandler(CountHandler(&count,&m,sleep_ms));
        thread->Start();
    }

    std::this_thread::sleep_for(
                std::chrono::milliseconds(20));

    // Expect that only the first posted handler is
    // invoked since the thread is destroyed within
    // that handler (it holds the last shared_ptr)
    assert(count == 1);
}

int main()
{

    TrivialConstructionAndDestruction();

    PostHandlersWhenInactive();

    TestStop();

    TestWait();

    // TestStartStartStartStopStopStop(); // silly

    TestExtendedLifetime();

    std::cout << "All tests passed" << std::endl;

    return 0;
}
