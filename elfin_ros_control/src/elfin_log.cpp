#include "elfin_ros_control/elfin_log.h"
#include "elfin_ros_control/elfin_default_log_handler.h"
#include <cstdarg>
#include <cstdio>

namespace elfin_ros_control
{
    class elfin_logger
    {
        public:
            elfin_logger()
            {
                log_level = LogLevel::WARN;
                log_handler.reset(new ElfinDefaultLogHandler());
            }
            ~elfin_logger()
            {
                if(log_handler)
                {
                    log_handler.reset();
                }
            }

            // 创建日志句柄
            void createNewLogHandler(std::unique_ptr<Elfin_LogHandler> elfinLogHandler)
            {
                log_handler = std::move(elfinLogHandler);
            }

            // 注销日志句柄
            void unregisterLogHandler()
            {
                log_handler.reset(new ElfinDefaultLogHandler());
            }

            // 设置日志等级
            void setLogLevel(LogLevel level)
            {
                log_level = level;
            }

            // 打印日志
            void log(const char* LogFile, int LogLine, LogLevel LogLevel, const char* log_out, ...)
            {
                if(!log_handler)
                {
                    log_handler.reset(new ElfinDefaultLogHandler());
                }
                log_handler->log(LogFile, LogLine, LogLevel, log_out);
            }

            // 获取日志等级
            LogLevel getLogLevel()
            {
                return log_level;
            }

        private:
            std::unique_ptr<Elfin_LogHandler> log_handler;
            LogLevel log_level;
    };
    elfin_logger logger;

    // 创建日志句柄
    void createNewLogHandler(std::unique_ptr<Elfin_LogHandler> elfinLogHandler)
    {
        logger.createNewLogHandler(std::move(elfinLogHandler));
    }

    // 注销日志句柄
    void unregisterLogHandler()
    {
        logger.unregisterLogHandler();
    }

    // 设置日志等级
    void setLogLevel(LogLevel level)
    {
        logger.setLogLevel(level);
    }

    // 打印日志
    void log(const char* LogFile, int LogLine, LogLevel LogLevel, const char* log_out, ...)
    {
        if(LogLevel >= logger.getLogLevel())
        {
            size_t log_size = 1024;
            std::unique_ptr<char[]> log_buffer;
            log_buffer.reset(new char[log_size]);

            va_list args;
            va_start(args, log_out);
            va_list args_copy;
            va_start(args_copy, args);

            size_t charsize = 1 + std::vsnprintf(log_buffer.get(), log_size, log_out, args);
            if(charsize >= log_size)
            {
                log_size = charsize + 1;
                log_buffer.reset(new char[log_size]);
                std::vsnprintf(log_buffer.get(), log_size, log_out, args_copy);
            }
            va_end(args);
            va_end(args_copy);
            
            logger.log(LogFile, LogLine, LogLevel, log_buffer.get());
        }
    } 
};