# pragma once
# include <inttypes.h>
# include <memory>

// 日志输出宏定义 - 文件名 行数 日志等级 日志内容
# define HR_LOG_DEBUG(...) elfin_ros_control::log(__FILE__, __LINE__, elfin_ros_control::LogLevel::DEBUG, __VA_ARGS__)
# define HR_LOG_WARN(...) elfin_ros_control::log(__FILE__, __LINE__, elfin_ros_control::LogLevel::WARN, __VA_ARGS__)
# define HR_LOG_INFO(...) elfin_ros_control::log(__FILE__, __LINE__, elfin_ros_control::LogLevel::INFO, __VA_ARGS__)
# define HR_LOG_ERROR(...) elfin_ros_control::log(__FILE__, __LINE__, elfin_ros_control::LogLevel::ERROR, __VA_ARGS__)
# define HR_LOG_FATAL(...) elfin_ros_control::log(__FILE__, __LINE__, elfin_ros_control::LogLevel::FATAL, __VA_ARGS__)

namespace elfin_ros_control
{
    enum class LogLevel
    {
        DEBUG = 0,
        INFO,
        WARN,
        ERROR,
        FATAL,
        NONE
    };

    class Elfin_LogHandler
    {
        public:
            virtual ~Elfin_LogHandler() = default;

            /*
                param file: 日志输出的文件
                param line: 文件中的代码行数
                param loglevel:日志等级
                param log_out: 日志内容
            */
            virtual void log(const char* file, int line, LogLevel loglevel, const char* log_out) = 0;
    };

    void createNewLogHandler(std::unique_ptr<Elfin_LogHandler> elfinLogHandler);

    void unregisterLogHandler();

    void setLogLevel(LogLevel level);

    void log(const char* LogFile, int LogLine, LogLevel LogLevel, const char* log_out, ...);
}