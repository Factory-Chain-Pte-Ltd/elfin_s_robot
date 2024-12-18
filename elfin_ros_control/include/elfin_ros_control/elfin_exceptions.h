#ifndef HR_CLIENT_EXCEPTIONS_INC
#define HR_CLIENT_EXCEPTIONS_INC

#include <chrono>
#include <stdexcept>
#include <sstream>

namespace elfin_ros_control
{
    class HrException : virtual public std::runtime_error
    {
        public:
            explicit HrException() : std::runtime_error("")
            {
            }
            explicit HrException(const std::string& what_arg) : std::runtime_error(what_arg)
            {
            }
            explicit HrException(const char* what_arg) : std::runtime_error(what_arg)
            {
            }
            virtual ~HrException() = default;
        private:
    };
    class TimeoutException : public HrException
    {
        public:
            explicit TimeoutException() = delete;
            explicit TimeoutException(const std::string& text, timeval timeout) : std::runtime_error(text)
            {
                std::stringstream ss;
                ss<<text<<"(Configured timeout: "<<timeout.tv_sec + timeout.tv_usec * 1e-6 <<" sec)";
                text_ = ss.str();
            }
            virtual const char* what() const noexcept override
            {
                return text_.c_str();
            }
        private:
            std::string text_;
    };
    
}
#endif