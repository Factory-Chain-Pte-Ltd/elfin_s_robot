#include <ros/console.h>
#include <elfin_ros_control/elfin_data_log_handler.h>

namespace elfin_ros_control
{
    bool logState = false;
    std::unique_ptr<ElfinDataLogHandler> elfin_log_handler(new ElfinDataLogHandler);

    ElfinDataLogHandler::ElfinDataLogHandler() : log_name(std::string(ROSCONSOLE_NAME_PREFIX)  +".elfin_log")
    {
    }

    void ElfinDataLogHandler::log(const char* LogFile, int LogLine, elfin_ros_control::LogLevel loglevel, const char* logOut)
    {   
        switch (loglevel)
        {
        case elfin_ros_control::LogLevel::DEBUG:
            logMessage(LogFile, LogLine, ros::console::levels::Debug, logOut);
            break;
        case elfin_ros_control::LogLevel::INFO:
            logMessage(LogFile, LogLine, ros::console::levels::Info, logOut);
            break;
        case elfin_ros_control::LogLevel::WARN:
            logMessage(LogFile, LogLine, ros::console::levels::Warn, logOut);
            break;
        case elfin_ros_control::LogLevel::ERROR:
            logMessage(LogFile, LogLine, ros::console::levels::Error, logOut);
            break;
        case elfin_ros_control::LogLevel::FATAL:
            logMessage(LogFile, LogLine, ros::console::levels::Fatal, logOut);
            break;
        default:
            break;
        }
    }

    void ElfinDataLogHandler::logMessage(const char* file, int line, ros::console::Level level, const char* message)
    {
        ROSCONSOLE_DEFINE_LOCATION(true, level, log_name);
        if(ROS_UNLIKELY(__rosconsole_define_location__enabled))
        {
            ros::console::print(NULL,__rosconsole_define_location__loc.logger_, level, file, line, "", "%s", message);
        }
    }
    
    void startElfinLogHandler()
    {
        if(logState == false)
        {
            elfin_ros_control::setLogLevel(elfin_ros_control::LogLevel::DEBUG);
            elfin_ros_control::createNewLogHandler(std::move(elfin_log_handler));
            logState = true;
        }
    }

    void stopElfinLogHandler()
    {
        if(logState == true)
        {
            elfin_ros_control::unregisterLogHandler();
            logState = false;
        }
    }
}