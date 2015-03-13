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
#include <smlog.h>

// android
#ifdef ANDROID_LOGCAT
#include <android/log.h>
#endif


// SinkToStdOut
// * simple sink that outputs to stdout
class SinkToStdOut : public smlog::Sink
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


// SinkToLogCat
// * simple sink that outputs to android's logcat
#ifdef ANDROID_LOGCAT
class SinkToLogCat : public smlog::Sink
{
public:
    void log(std::string const &line)
    {
        m_mutex.lock();
        __android_log_print(ANDROID_LOG_VERBOSE,
                            "libname",
                            line.c_str());
        m_mutex.unlock();
    }

private:
    std::mutex m_mutex;
};
#endif


void testLog(smlog::Logger &log)
{
    log.Trace() << "This is a typical trace message, here are some values {"
                  << 1 << "," << true << "," << 1.2345 << "}";

    log.Debug() << "This is a typical debug message, here are some values {"
                  << 1 << "," << true << "," << 1.2345 << "}";

    log.Info() << "This is a typical info message, here are some values {"
                  << 1 << "," << true << "," << 1.2345 << "}";

    log.Warn() << "This is a typical warn message, here are some values {"
                  << 1 << "," << true << "," << 1.2345 << "}";

    log.Error() << "This is a typical error message, here are some values {"
                  << 1 << "," << true << "," << 1.2345 << "}";

    log.Fatal() << "This is a typical fatal message, here are some values {"
                  << 1 << "," << true << "," << 1.2345 << "}";
}


int main()
{
    // Create a sink for output
    std::shared_ptr<smlog::Sink> sink =
            std::make_shared<SinkToStdOut>();

    // Create the log
    smlog::Logger log;
    log.AddSink(sink);

    // Add some prefix tokens for each log message
    std::unique_ptr<smlog::FBCustomStr> fb0(
            new smlog::FBCustomStr("TRACE: LIBNAME: " ));

    std::unique_ptr<smlog::FBCustomStr> fb1(
            new smlog::FBCustomStr("DEBUG: LIBNAME: " ));

    std::unique_ptr<smlog::FBCustomStr> fb2(
            new smlog::FBCustomStr("INFO:  LIBNAME: " ));

    std::unique_ptr<smlog::FBCustomStr> fb3(
            new smlog::FBCustomStr("WARN:  LIBNAME: " ));

    std::unique_ptr<smlog::FBCustomStr> fb4(
            new smlog::FBCustomStr("ERROR: LIBNAME: " ));

    std::unique_ptr<smlog::FBCustomStr> fb5(
            new smlog::FBCustomStr("FATAL: LIBNAME: " ));

    log.AddFormatBlock(std::move(fb0),smlog::Logger::Level::TRACE);
    log.AddFormatBlock(std::move(fb1),smlog::Logger::Level::DEBUG);
    log.AddFormatBlock(std::move(fb2),smlog::Logger::Level::INFO);
    log.AddFormatBlock(std::move(fb3),smlog::Logger::Level::WARN);
    log.AddFormatBlock(std::move(fb4),smlog::Logger::Level::ERROR);
    log.AddFormatBlock(std::move(fb5),smlog::Logger::Level::FATAL);

    testLog(log);

    // Test setting / unsetting log levels. All levels
    // are initially enabled by default
    log.UnsetLevel(smlog::Logger::Level::TRACE);
    log.UnsetLevel(smlog::Logger::Level::DEBUG);
    log.UnsetLevel(smlog::Logger::Level::WARN);

    std::cout << std::endl;
    testLog(log);

    log.SetLevel(smlog::Logger::Level::DEBUG);
    std::cout << std::endl;
    testLog(log);

    return 0;
}

