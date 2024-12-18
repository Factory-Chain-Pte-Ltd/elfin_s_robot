#include <elfin_ros_control/elfin_controller_manager.h>

elfin_controller_manager::elfin_controller_manager(const ros::NodeHandle& nh) : nh_(nh), sole_nh_("~"), elfin_controller_running(true)
{
    robot_state_sub_ = nh_.subscribe("robot_state", 1, &elfin_controller_manager::robotStateCallback, this);
    
    elfin_controller_manager_srv_ = nh_.serviceClient<controller_manager_msgs::SwitchController>("controller_manager/switch_controller");

    elfin_controller_list_srv_ = nh_.serviceClient<controller_manager_msgs::ListControllers>("controller_manager/list_controllers");

    ROS_INFO_STREAM("Wait for controller manager come up, "<< nh_.resolveName("controller_manager/switch_controller"));

    elfin_controller_manager_srv_.waitForExistence();
    ROS_INFO_STREAM("Controller manager service activated.");
    
    ROS_INFO_STREAM("Wait for controller list come up" << nh_.resolveName("controller_manager/list_controllers"));

    elfin_controller_list_srv_.waitForExistence();
    ROS_INFO_STREAM("Controller list service activated.");

    if(!sole_nh_.getParam("smae_controllers_name", smae_controllers_list_))
    {
        smae_controllers_list_.push_back("joint_state_controller");
    }
    ROS_DEBUG("Waiting for controller running");

    while(stop_controllers_list_.empty())
    {
        findControllerStop();
        ros::Duration(1).sleep();
    }
    ROS_INFO("Elfin controller manager init finish");
}

void elfin_controller_manager::findControllerStop()
{
    controller_manager_msgs::ListControllers list_srv_;
    elfin_controller_list_srv_.call(list_srv_);
    stop_controllers_list_.clear();
    for(auto& controller : list_srv_.response.controller)
    {
        if(controller.state == "running")
        {
            auto it = std::find(smae_controllers_list_.begin(), 
                                smae_controllers_list_.end(), controller.name);
            if(it == smae_controllers_list_.end())
            {
                stop_controllers_list_.push_back(controller.name);
            }
        }
    }
}
;
void elfin_controller_manager::robotStateCallback(const std_msgs::BoolConstPtr& msg)
{
    ROS_INFO_STREAM("robot state call back data:" << std::boolalpha<<msg->data);
    if(msg->data && !elfin_controller_running)
    {
        ROS_INFO_STREAM("Start Controller now");
        controller_manager_msgs::SwitchController srv;
        srv.request.strictness = srv.request.STRICT;
        srv.request.start_controllers = stop_controllers_list_;
        if(!elfin_controller_manager_srv_.call(srv))
        {
            ROS_ERROR_STREAM("Cann't start the elfin controllers");
        }
    }else if(!msg->data && elfin_controller_running)
    {
        ROS_INFO_STREAM("Stop controller now");
        findControllerStop();
        controller_manager_msgs::SwitchController srv;
        srv.request.strictness = srv.request.STRICT;
        srv.request.stop_controllers = stop_controllers_list_;
        if(!elfin_controller_manager_srv_.call(srv))
        {
            ROS_ERROR_STREAM("Cann't stop the elfin controllers");
        }
    }
    elfin_controller_running = msg->data;
}