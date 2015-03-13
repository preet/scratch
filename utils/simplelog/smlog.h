/*
   Copyright (C) 2015 Preet Desai (preet.desai@gmail.com)

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


#ifndef SCRATCH_INLINE_LOG_H
#define SCRATCH_INLINE_LOG_H

#include <string>
#include <memory>
#include <vector>
#include <array>
#include <ctime>
#include <mutex>
#include <bitset>
#include <iostream>
#include <sstream>

namespace smlog
{
    // ============================================================= //

    // Sink
    // * abstract class that represents logging output
    // * concrete classes must implement the log() method
    class Sink
    {
    public:
        virtual ~Sink() = default;
        virtual void log(std::string const &line)=0;
    };

    // SinkToStdOut
    // * simple sink that outputs to stdout
    class SinkToStdOut : public Sink
    {
    public:
        void log(std::string const &line)
        {
            m_mutex.lock();
            std::cout << line << std::endl;
            m_mutex.unlock();
        }

    private:
        std::mutex m_mutex;
    };

    // ============================================================= //

    // FormatBlock
    // * abstract class that represents a specific token
    //   of formatting that is prefixed to logging output
    class FormatBlock
    {
    public:
        virtual ~FormatBlock() = default;
        virtual std::string Get() = 0;
    };

    // FBRunTimeMs
    // * format block that provides elapsed time since
    //   its creation in the format (00:00:00.000)
    class FBRunTimeMs : public FormatBlock
    {
    public:
        FBRunTimeMs();
        ~FBRunTimeMs();

        std::string Get();

    private:
        std::string m_time_str;
        std::chrono::system_clock::time_point const m_start;
        std::array<char,10> const m_list_num_chars;
    };

    // FBCustomStr
    // * format block that wraps a string
    class FBCustomStr : public FormatBlock
    {
    public:
        FBCustomStr(std::string const &s);
        ~FBCustomStr();

        std::string Get();

    private:
        std::string const m_s;
    };

    // ============================================================= //

    // Logger
    // * simple logging class with optional thread safety
    class Logger
    {
    private:
        // Mutex
        // * abstract implementation of a mutex that
        //   only exposes a lock() and unlock() method
        // * allows us to provide a dummy implementation
        //   if thread safety isn't wanted
        struct Mutex
        {
            virtual ~Mutex() {}
            virtual void lock()=0;
            virtual void unlock()=0;
        };

        struct MutexSTL : public Mutex
        {
            ~MutexSTL() {}

            void lock() {
                m_mutex.lock();
            }

            void unlock() {
                m_mutex.unlock();
            }

            std::mutex m_mutex;
        };

        struct MutexDummy : public Mutex
        {
            ~MutexDummy() {}
            void lock() {}
            void unlock() {}
        };

        // Line
        // * class that wraps logging a line with RAII
        // * line is commited to log on destruction
        class Line
        {
        public:
            Line(std::vector<std::shared_ptr<Sink>> const * list_sinks,
                 std::vector<std::unique_ptr<FormatBlock>> const * list_fb,
                 Mutex * mutex,
                 bool line_valid) :
                m_list_sinks(list_sinks),
                m_list_fb(list_fb),
                m_mutex(mutex),
                m_line_valid(line_valid)
            {
                if(m_line_valid) {
                    // create the prefix
                    for(auto & fb : (*m_list_fb)) {
                        m_line.append(fb->Get());
                    }
                }
            }

            ~Line()
            {
                if(m_line_valid) {
                    for(auto &sink : (*m_list_sinks)) {
                        sink->log(m_line);
                    }
                }
                m_mutex->unlock();
            }

            template<typename T>
            Line & operator << (T const &msg)
            {
                if(m_line_valid) {
                    m_line.append(to_string(msg));
                }
                return *this;
            }

            Line & operator << (std::string const &msg)
            {
                if(m_line_valid) {
                    m_line.append(msg);
                }
                return *this;
            }

            Line & operator << (const char * msg)
            {
                if(m_line_valid) {
                    m_line.append(msg);
                }
                return *this;
            }

        private:
            std::vector<std::shared_ptr<Sink>> const * m_list_sinks;
            std::vector<std::unique_ptr<FormatBlock>> const * m_list_fb;
            Mutex * m_mutex;
            bool const m_line_valid;

            std::string m_line;
        };

    public:
        enum class Level : uint8_t
        {
            TRACE   = 0,
            DEBUG   = 1,
            INFO    = 2,
            WARN    = 3,
            ERROR   = 4,
            FATAL   = 5
        };

        // default constructor assumes thread safety is wanted
        Logger();

        // NOTE:
        // It is expected that FormatBlock ownership is
        // transfered to this Log instance and that the
        // transfered pointers are the only valid refs.

        // ie, create list_fbs like: {{ // start array
        //    { new FBImpl(...), new FBImpl(...) ... },
        //    { new FBImpl(...), new FBImpl(...) ... },
        //    { ... }
        // }} // end array; note that std::array requires
        //    // double braces for its init list

        // This is a work around since std::unique_ptrs can't be
        // constructed directly with initializer lists
        Logger(bool thread_safe,
               std::shared_ptr<Sink> const &sink,
               std::array<std::vector<FormatBlock*>,6> && list_fbs);

        bool AddSink(std::shared_ptr<Sink> const &new_sink);
        bool RemoveSink(std::shared_ptr<Sink> const &sink);
        void SetLevel(Level level);
        void UnsetLevel(Level level);
        void AddFormatBlock(std::unique_ptr<FormatBlock> fb,
                            Level level);

        // logging methods
        Line Trace();
        Line Debug();
        Line Info();
        Line Warn();
        Line Error();
        Line Fatal();

    private:
        template<typename T>
        static std::string to_string(T const &val)
        {
            std::ostringstream oss;
            oss << val;
            return oss.str();
        }

        std::unique_ptr<Mutex> m_mutex;
        std::vector<std::shared_ptr<Sink>> m_list_sinks;
        std::bitset<6> m_filter;
        std::array<std::vector<std::unique_ptr<FormatBlock>>,6> m_list_fb;
    };

} // Log


#endif // SCRATCH_INLINE_LOG_H
