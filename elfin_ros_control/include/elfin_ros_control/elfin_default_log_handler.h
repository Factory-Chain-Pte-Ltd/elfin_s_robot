#pragma once

#include "elfin_ros_control/elfin_log.h"

namespace elfin_ros_control
{
    class ElfinDefaultLogHandler : public Elfin_LogHandler
    {
        public:
            ElfinDefaultLogHandler();

            /*
                param file: 日志输出的文件
                param line: 文件中的代码行数
                param loglevel:日志等级
                param log_out: 日志内容
            */
            void log(const char* file, int line, LogLevel loglevel, const char* log_out) override;
    };
}