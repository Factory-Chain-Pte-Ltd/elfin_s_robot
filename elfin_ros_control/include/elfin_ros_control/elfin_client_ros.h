#ifndef ELFIN_ROBOT_ROS_CLIENT_H
#define ELFIN_ROBOT_ROS_CLIENT_H
#include <elfin_industrial_driver/elfin/elfin_client.h>

#include <regex>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>

namespace elfin_ros_control
{
    class ElfinClientROS
    {
        public:
            ElfinClientROS(const ros::NodeHandle& nh, const std::string& robot_ip);
            ElfinClientROS() = delete;
            virtual ~ElfinClientROS() = default;
        private:

            ros::NodeHandle nh_;
            elfin::ElfinClient elfin_client_;
            bool connect();

            // 命令服务
            ros::ServiceServer electrify_service_;
            ros::ServiceServer blackout_service_;
            ros::ServiceServer enable_service_;
            ros::ServiceServer disenable_service_;
            ros::ServiceServer stop_move_service_;
            ros::ServiceServer reset_error_service_;
            ros::ServiceServer startProgram_service_;
            ros::ServiceServer pauseProgram_service_;
            ros::ServiceServer continueProgram_service_;
            ros::ServiceServer stopProgram_service_;

            // 其他服务
            ros::ServiceServer reconnect_service_;

        inline ros::ServiceServer create_elfin_command_server(const std::string& topic_name,
                                                              const std::string& elfin_command,
                                                              const std::string& command_req)
        {
            return nh_.advertiseService<std_srvs::SetBool::Request, std_srvs::SetBool::Response>(
                topic_name, [&, elfin_command, command_req](std_srvs::SetBool::Request& req,std_srvs::SetBool::Response& resp){
                    resp.message = this->elfin_client_.sendReceive(elfin_command);
                    resp.success = std::regex_match(resp.message, std::regex(command_req));
                    return true;
                });
        }
    };
}

#endif