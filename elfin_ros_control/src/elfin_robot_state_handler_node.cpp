#include <elfin_ros_control/elfin_robot_state_helper.h>

using namespace elfin_ros_control;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "elfin_state_helper");
    ros::NodeHandle nh;

    ElfinRobotStateHandler state_helper(nh);
    ros::spin();
    return 0;
}