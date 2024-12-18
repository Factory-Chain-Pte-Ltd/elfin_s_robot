#include<ros/ros.h>
#include<elfin_ros_control/elfin_controller_manager.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "elfin_controller_node");
    ros::NodeHandle nh;
    ros::NodeHandle sole_nh_("");

    elfin_controller_manager elfin_controller_manager(nh);
    ros::spin();
    return 0;
}