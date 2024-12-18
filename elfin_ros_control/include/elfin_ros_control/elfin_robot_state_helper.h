#ifndef ELFIN_ROBOT_STATE_INC
#define ELFIN_ROBOT_STATE_INC

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

// #include <elfin_industrial_driver/elfin/

namespace elfin_ros_control
{
    class ElfinRobotStateHandler
    {
        public:
            ElfinRobotStateHandler(const ros::NodeHandle &h);
            ElfinRobotStateHandler() = delete;
            virtual ~ElfinRobotStateHandler() = default;
        
        private:
            ros::NodeHandle nh_;
            bool robot_started;

            void robotStateCB(const int robot_state);
            void updateRobotState();

            void startActionServer();

            ros::Subscriber robot_state_sub_;

            ros::ServiceClient robot_electrify_srv;
            ros::ServiceClient robot_blackout_srv;
            ros::ServiceClient robot_enable_srv;
            ros::ServiceClient robot_disenable_srv;
            ros::ServiceClient robot_stop_srv;
            ros::ServiceClient robot_reset_srv;

            ros::ServiceClient robot_startProgram_srv;
            ros::ServiceClient robot_pauseProgram_srv;
            ros::ServiceClient robot_continueProgram_srv;
            ros::ServiceClient robot_stopProgram_srv;

    };
}

#endif