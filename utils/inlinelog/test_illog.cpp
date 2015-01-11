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

// sys
#include <cstdint>

// stl
#include <string>
#include <vector>
#include <memory>
#include <iostream>
#include <chrono>
#include <mutex>
#include <sstream>
#include <thread>

// ilim
#include <illog.hpp>

class SinkToStdOut : public illog::Sink
{
public:
    void log(std::string const &line)
    {
        m_mutex.lock();
        std::stringstream s;
        s << line;
//        std::cout << line << std::endl;
        m_mutex.unlock();
    }

private:
    std::mutex m_mutex;
};


void CalcTime()
{
    // duration<Rep,Period>
    // Rep: integer format that counts periods
    // Period: std::ratio that holds num. seconds
    //         ie. hours: std::ratio<3600,1>
    //             milliseconds: std::ratio<1,1000>
    typedef std::chrono::duration<
            int,
            std::ratio_multiply<
                std::chrono::hours::period,
                std::ratio<24>
                >::type
            > std_chrono_days;

    std::chrono::system_clock::time_point const now =
            std::chrono::system_clock::now();

    std::chrono::system_clock::duration since_epoch =
            now.time_since_epoch();


    std::cout << "#: in nano_s: " << (since_epoch).count()
              << ", " << std::chrono::system_clock::duration::period::den
              << std::endl;

    // take out days since epoch
    std_chrono_days days =
            std::chrono::duration_cast<std_chrono_days>(
                since_epoch);

    since_epoch -= days;

    // take out remaining hours
    std::chrono::hours hours =
            std::chrono::duration_cast<std::chrono::hours>(
                since_epoch);

    since_epoch -= hours;

    // take out remaining minutes
    std::chrono::minutes mins =
            std::chrono::duration_cast<std::chrono::minutes>(
                since_epoch);

    since_epoch -= mins;

    // take out remaining seconds
    std::chrono::seconds secs =
            std::chrono::duration_cast<std::chrono::seconds>(
                since_epoch);

    since_epoch -= secs;

    //
    std::chrono::milliseconds ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                since_epoch);

    std::cout << days.count() << "d "
              << hours.count() << ':'
              << mins.count() << ':' << secs.count()
              << std::endl;

    std::cout << "  days: " << days.count()
              << ", hours: " << hours.count()
              << ", mins: " << mins.count()
              << ", sec: " << secs.count()
              << ", ms: " << ms.count()
              << std::endl;

    // as local time
    std::time_t tt = std::chrono::system_clock::to_time_t(now);
    std::tm local_tm = *std::localtime(&tt);

    std::cout << local_tm.tm_year + 1900 << '-';
    std::cout << local_tm.tm_mon + 1 << '-';
    std::cout << local_tm.tm_mday << ' ';
    std::cout << local_tm.tm_hour << ':';
    std::cout << local_tm.tm_min << ':';
    std::cout << local_tm.tm_sec << '\n';
}


int main()
{
    std::shared_ptr<illog::Sink> sink =
            std::make_shared<SinkToStdOut>();

    illog::Log log;
    log.add_sink(sink);

    std::unique_ptr<illog::Log::FBCustomStr> fb_info_name(
            new illog::Log::FBCustomStr(": INFO: MAPVIRE: " ));

    std::unique_ptr<illog::Log::FBRunTimeMs> fb_info_time(
            new illog::Log::FBRunTimeMs());

    log.add_format_block(std::move(fb_info_time),
                         illog::Log::Level::INFO);

    log.add_format_block(std::move(fb_info_name),
                         illog::Log::Level::INFO);

    std::chrono::milliseconds duration(321);



    std::chrono::time_point<std::chrono::system_clock> start,end;
    start = std::chrono::system_clock::now();
    for(size_t i=0; i < 10000; i++) {
        log.info() << "This is a typical debug message, here are some values {"
                   << 1 << "," << true << "," << 1.2345 << "}";

//        std::this_thread::sleep_for(duration);

//        log.info() << "This is a typical debug message, here are some values {"
//                   << 1 << "," << true << "," << 1.2345 << "}";

//        std::this_thread::sleep_for(duration);

//        log.info() << "This is a typical debug message, here are some values {"
//                   << 1 << "," << true << "," << 1.2345 << "}";

//        std::this_thread::sleep_for(duration);

//        log.info() << "This is a typical debug message, here are some values {"
//                   << 1 << "," << true << "," << 1.2345 << "}";

//        std::this_thread::sleep_for(duration);
    }

    log.unset_level(illog::Log::Level::INFO);

    for(size_t i=0; i < 10000; i++) {
        log.info() << "This is a typical debug message, here are some values {"
                   << 1 << "," << true << "," << 1.2345 << "}";
    }

    end = std::chrono::system_clock::now();
    std::cout << "took: " << std::chrono::duration<double>(end-start).count() << " sec" << std::endl;


//    CalcTime();

    return 0;
}
