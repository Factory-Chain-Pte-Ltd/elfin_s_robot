#ifndef ELFIN_DATA_LOG_HANDLER_H
#define ELFIN_DATA_LOG_HANDLER_H

#include "elfin_ros_control/elfin_log.h"
#include <ros/ros.h>
#include <ros/console.h>

namespace elfin_ros_control
{
    class ElfinDataLogHandler : public elfin_ros_control::Elfin_LogHandler
    {
        public:
            ElfinDataLogHandler();
            
            void log(const char* LogFile, int LogLine, elfin_ros_control::LogLevel loglevel, const char* logOut) override;
        private:
    
            std::string log_name;

            void logMessage(const char* file, int line, ros::console::Level level, const char* message);
    };

    void startElfinLogHandler();
    void stopElfinLogHandler();
}

#endif