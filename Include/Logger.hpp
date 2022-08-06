#pragma once

#include <mutex>
#include <string>

#include <log4cpp/Category.hh>
#include <log4cpp/Priority.hh>
#include <log4cpp/PatternLayout.hh>
#include <log4cpp/OstreamAppender.hh>
#include <log4cpp/RollingFileAppender.hh>

#include <boost/noncopyable.hpp>
#include <boost/filesystem.hpp>

#include "Def.hpp"

#define GET_LOGGER() Logger::instance(__FILE__)

class Logger: boost::noncopyable
{
    log4cpp::Category& category_;
    std::vector<std::shared_ptr<log4cpp::LayoutAppender>> appenders_;
    const std::string format_ = "[%d] [%t] [%x] [%p] [%c]: %m %n";  // Format : [time] [thread_id] [NDC] [priority] [category]: message newline

#if _DEBUG
    const std::string filename = "debug.log";
#else
    const std::string filename = "release.log";
#endif
    
    Logger(const std::string& name): category_(log4cpp::Category::getInstance(name))
    {
        namespace fs=boost::filesystem;
        
        fs::path app_log_dir_path = fs::path(Def::getAppDir());

        auto log_path = (app_log_dir_path / filename).string();
        auto file_appender = std::make_shared<log4cpp::RollingFileAppender>(name, log_path);
        appenders_.push_back(file_appender);

        // The appender will take over the resource management of layout, thus we don't need to dispose it.
        auto layout_1 = new log4cpp::PatternLayout();   
        layout_1->setConversionPattern(format_);
        file_appender->setLayout(layout_1);
        category_.addAppender(*file_appender);

#if _DEBUG
        auto console_appender = std::make_shared<log4cpp::OstreamAppender>(name, &std::cout);

        auto layout_2 = new log4cpp::PatternLayout();
        layout_2->setConversionPattern(format_);
        console_appender->setLayout(layout_2);
        category_.addAppender(*console_appender);
        appenders_.push_back(console_appender);
#endif
        
        category_.setPriority(log4cpp::Priority::DEBUG);
    }

    ~Logger() = default;
    
public:

    static Logger& instance(const std::string& name)
    {
        static std::mutex mutex;
        static std::map<std::string, std::shared_ptr<Logger>> loggers;

        {
	        std::lock_guard lock(mutex);
            if(loggers.find(name) != loggers.end())
                return *loggers[name];

            auto ptr =
                std::shared_ptr<Logger>(new Logger(name), [](Logger* logger) { delete logger; });
            loggers[name] = ptr;
            return *ptr;
        }
    }

    void debug(const std::string& message)
    {
	    category_.debug(message);
    }

    void info(const std::string& message)
    {
	    category_.info(message);
    }

    void error(const std::string& message)
    {
	    category_.error(message);
    }
};
