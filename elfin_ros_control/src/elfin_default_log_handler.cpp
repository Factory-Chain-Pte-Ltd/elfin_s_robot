#include "elfin_ros_control/elfin_default_log_handler.h"
#include <stdio.h>
#include <iostream>

namespace elfin_ros_control

{
    ElfinDefaultLogHandler::ElfinDefaultLogHandler() = default;

    void ElfinDefaultLogHandler::log(const char* file, int line, LogLevel loglevel, const char* log_out)
    {   
        // 根据日志等级输出日志
        switch (loglevel)
        {
        case LogLevel::INFO:
            // printf("%s %s %i: %s\n", "INFO", file, line, log_out);
            std::cout<<"info"<<std::endl;
            break;
        case LogLevel::DEBUG:
            // printf("%s %s %i: %s\n", "DEBUG", file, line, log_out);
            std::cout<<"info"<<std::endl;
            break;
        case LogLevel::WARN:
            // printf("%s %s %i: %s\n", "WARN", file, line, log_out);
            std::cout<<"info"<<std::endl;
            break;
        case LogLevel::ERROR:
            // printf("%s %s %i: %s\n", "ERROR", file, line, log_out);
            std::cout<<"info"<<std::endl;
            break;
        case LogLevel::FATAL:
            // printf("%s %s %i: %s\n", "FATAL", file, line, log_out);
            std::cout<<"info"<<std::endl;
            break;
        default:
            break;
        }   
    }
}