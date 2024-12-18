#include <ros/ros.h>
#include <elfin_ros_control/elfin_client_ros.h>
#include <elfin_ros_control/elfin_data_log_handler.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "elfin_client_ros");
    ros::NodeHandle client_nh_("~");
    elfin_ros_control::startElfinLogHandler();

    std::string elfin_robot_ip = client_nh_.param<std::string>("robot_ip", "192.168.0.10");

    elfin_ros_control::ElfinClientROS client(client_nh_, elfin_robot_ip);
    ros::spin();
    return 0;
}