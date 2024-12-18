#ifndef ELFIN_CONTROLLER_STOP_INC
#define ELFIN_CONTROLLER_STOP_INC

#include <ios>
#include <ros.h>
#include <std_msgs/Bool.h>

#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/ListControllers.h>

class elfin_controller_manager
{
    private:

        ros::NodeHandle nh_;
        ros::NodeHandle sole_nh_;
        ros::Subscriber robot_state_sub_;
        ros::ServiceClient elfin_controller_manager_srv_;
        ros::ServiceClient elfin_controller_list_srv_;

        void robotStateCallback(const std_msgs::BoolConstPtr& msg);
        void findControllerStop();

        std::vector<std::string> smae_controllers_list_;
        std::vector<std::string> stop_controllers_list_;

        bool elfin_controller_running;

    public:
        elfin_controller_manager() = delete;
        elfin_controller_manager(const ros::NodeHandle& nh);
        virtual ~elfin_controller_manager() = default;
};

#endif