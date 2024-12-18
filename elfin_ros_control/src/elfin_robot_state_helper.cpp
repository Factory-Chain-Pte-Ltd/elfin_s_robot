#include <elfin_ros_control/elfin_robot_state_helper.h>
#include <std_srvs/SetBool.h>

namespace elfin_ros_control
{
    ElfinRobotStateHandler::ElfinRobotStateHandler(const ros::NodeHandle& nh) :
        nh_(nh), robot_started(false)
    {
        robot_state_sub_ = nh_.subscribe("elfin/elfin_fsm_state", 1, &ElfinRobotStateHandler::robotStateCB, this);

        robot_electrify_srv = nh_.serviceClient<std_srvs::SetBool>("elfin/robot_electrify");
        robot_blackout_srv = nh_.serviceClient<std_srvs::SetBool>("elfin/elfin_blackout");

        robot_enable_srv = nh_.serviceClient<std_srvs::SetBool>("elfin/elfin_enable");
        robot_disenable_srv = nh_.serviceClient<std_srvs::SetBool>("elfin/elfin_disenable");
        robot_stop_srv = nh_.serviceClient<std_srvs::SetBool>("elfin/elfin_stop");
        robot_reset_srv = nh_.serviceClient<std_srvs::SetBool>("elfin/elfin_reset");

        robot_startProgram_srv = nh_.serviceClient<std_srvs::SetBool>("elfin/elfin_startProgram");
        robot_pauseProgram_srv = nh_.serviceClient<std_srvs::SetBool>("elfin/elfin_pauseProgram");
        robot_continueProgram_srv = nh_.serviceClient<std_srvs::SetBool>("elfin/elfin_continueProgram");
        robot_stopProgram_srv = nh_.serviceClient<std_srvs::SetBool>("elfin/elfin_stopProgram");
    }

    void ElfinRobotStateHandler::robotStateCB(const int robot_state)
    {

    }

    void ElfinRobotStateHandler::updateRobotState()
    {

    }

    void ElfinRobotStateHandler::startActionServer()
    {

    }
}