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


//filters [run-time]:
//trace
//debug
//info
//warn
//error
//fatal

// mode [compile-time]:
// output
// null (should be no-op ish)

//sinks:
//stdout
//stderr
//file

//__file__
//__line__


#ifndef SCRATCH_INLINE_LOG_H
#define SCRATCH_INLINE_LOG_H

#include <string>
#include <memory>
#include <vector>
#include <array>
#include <ctime>
#include <mutex>
#include <bitset>

namespace illog
{
    class Sink
    {
    public:
        virtual void log(std::string const &line)=0;
    };

    class Log
    {
    public:
        class FormatBlock
        {
        public:
            virtual ~FormatBlock() {}
            virtual std::string get() = 0;
        };

        // 00:00:00.000
        class FBRunTimeMs : public FormatBlock
        {
        public:
            FBRunTimeMs() :
                m_time_str("00:00:00.000"),
                m_start(std::chrono::system_clock::now()),
                m_list_num_chars({'0','1','2','3','4','5','6','7','8','9'})
            {
                // empty
            }

            ~FBRunTimeMs()
            {
                // empty
            }

            std::string get()
            {
                auto now = std::chrono::system_clock::now();
                std::chrono::system_clock::duration elapsed = now-m_start;

                std::chrono::hours hours =
                        std::chrono::duration_cast<std::chrono::hours>(
                            elapsed);
                elapsed -= hours;

                std::chrono::minutes mins =
                        std::chrono::duration_cast<std::chrono::minutes>(
                            elapsed);
                elapsed -= mins;

                std::chrono::seconds secs =
                        std::chrono::duration_cast<std::chrono::seconds>(
                            elapsed);
                elapsed -= secs;

                std::chrono::milliseconds ms =
                        std::chrono::duration_cast<std::chrono::milliseconds>(
                            elapsed);

                uint_fast8_t const hours_count = hours.count();
                uint_fast8_t const mins_count = mins.count();
                uint_fast8_t const secs_count = secs.count();
                uint_fast16_t const ms_count = ms.count();

                m_time_str[0] = m_list_num_chars[hours_count/10];
                m_time_str[1] = m_list_num_chars[hours_count%10];

                m_time_str[3] = m_list_num_chars[mins_count/10];
                m_time_str[4] = m_list_num_chars[mins_count%10];

                m_time_str[6] = m_list_num_chars[secs_count/10];
                m_time_str[7] = m_list_num_chars[secs_count%10];

                m_time_str[9] = m_list_num_chars[ms_count/100];
                m_time_str[10] = m_list_num_chars[(ms_count%100)/10];
                m_time_str[11] = m_list_num_chars[(ms_count%100)%10];

                return m_time_str;
            }

        private:
            std::string m_time_str;
            std::chrono::system_clock::time_point const m_start;
            std::array<char,10> const m_list_num_chars;
        };

        class FBCustomStr : public FormatBlock
        {
        public:
            FBCustomStr(std::string const &s) : m_s(s)
            {
                // empty
            }

            ~FBCustomStr()
            {
                // empty
            }

            std::string get()
            {
                return m_s;
            }

        private:
            std::string const m_s;
        };


    private:
        class Line
        {
        public:
            Line(std::vector<std::shared_ptr<Sink>> const * list_sinks,
                 std::vector<std::unique_ptr<FormatBlock>> const * list_fb,
                 std::mutex * mutex,
                 bool line_valid) :
                m_list_sinks(list_sinks),
                m_list_fb(list_fb),
                m_mutex(mutex),
                m_line_valid(line_valid)
            {
                if(m_line_valid) {
                    // create the prefix
                    for(auto & fb : (*m_list_fb)) {
                        m_line.append(fb->get());
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
                    m_line.append(std::to_string(msg));
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
            std::mutex * m_mutex;
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


        Log()
        {
            // default filter is all on
            m_filter.set(static_cast<size_t>(Level::TRACE));
            m_filter.set(static_cast<size_t>(Level::DEBUG));
            m_filter.set(static_cast<size_t>(Level::INFO));
            m_filter.set(static_cast<size_t>(Level::WARN));
            m_filter.set(static_cast<size_t>(Level::ERROR));
            m_filter.set(static_cast<size_t>(Level::FATAL));
        }

        bool add_sink(std::shared_ptr<Sink> const &new_sink)
        {
            std::lock_guard<std::mutex> lock(m_mutex);

            for(size_t i=0; i < m_list_sinks.size(); i++) {
                std::shared_ptr<Sink> const &sink =
                        m_list_sinks[i];

                if(sink == new_sink) {
                    return false;
                }
            }

            m_list_sinks.push_back(new_sink);

            return true;
        }

        bool remove_sink(std::shared_ptr<Sink> const &sink)
        {
            std::lock_guard<std::mutex> lock(m_mutex);

            for(auto sink_it = m_list_sinks.begin();
                sink_it != m_list_sinks.end();
                ++sink_it)
            {
                if((*sink_it) == sink) {
                    m_list_sinks.erase(sink_it);
                    return true;
                }
            }
            return false;
        }

        void set_level(Level level)
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_filter.set(static_cast<size_t>(level));
        }

        void unset_level(Level level)
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_filter.reset(static_cast<size_t>(level));
        }

        void add_format_block(std::unique_ptr<FormatBlock> fb,
                              Level level)
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_list_fb[static_cast<size_t>(level)].push_back(
                        std::move(fb));
        }

        Line info()
        {
            m_mutex.lock();

            size_t const level = static_cast<size_t>(Level::INFO);

            return Line(&m_list_sinks,
                        &(m_list_fb[level]),
                        &m_mutex,
                        m_filter[level]);
        }

    private:
        std::mutex m_mutex;
        std::vector<std::shared_ptr<Sink>> m_list_sinks;

        std::bitset<6> m_filter;

        std::array<std::vector<std::unique_ptr<FormatBlock>>,6> m_list_fb;
    };
}

#endif // SCRATCH_INLINE_LOG_H
