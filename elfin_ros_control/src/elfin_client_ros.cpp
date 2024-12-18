#include <elfin_ros_control/elfin_client_ros.h>

namespace elfin_ros_control
{
    ElfinClientROS::ElfinClientROS(const ros::NodeHandle& nh, 
                                    const std::string& robot_ip) : nh_(nh), elfin_client_(robot_ip)
    {
        connect();

        electrify_service_ = create_elfin_command_server("elfin_electrify", "Electrify,;", "Electrify,OK,;");
        blackout_service_ = create_elfin_command_server("elfin_blackout", "BlackOut,;", "BlackOut,OK,;");
        enable_service_ = create_elfin_command_server("elfin_enable", "GrpEnable,0,;", "GrpEnable,OK,;");
        disenable_service_ = create_elfin_command_server("elfin_disenable", "GrpDisable,0,;", "GrpDisable,OK,;");
        stop_move_service_ = create_elfin_command_server("elfin_stop", "GrpStop,0,;", "GrpStop,OK,;");
        reset_error_service_ = create_elfin_command_server("elfin_reset", "GrpReset,0,;","GrpReset,OK,;");
        startProgram_service_ = create_elfin_command_server("start_program", "StartScript,;", "StartScript,OK,;");
        pauseProgram_service_ = create_elfin_command_server("pause_program", "PauseScript,;", "PauseScript,OK,;");
        continueProgram_service_ = create_elfin_command_server("continue_program", "ContinueScript,;", "ContinueScript,OK,;");
        stopProgram_service_ = create_elfin_command_server("stop_program", "StopScript,;", "StopScript,OK,;");

        reconnect_service_ = nh_.advertiseService<std_srvs::SetBool::Request, std_srvs::SetBool::Response>(
                        "elfin_connect", [&](std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp)
                        {
                            resp.success = connect();
                            return true;
                        });
    } 

    bool ElfinClientROS::connect()
    {
        timeval curTV;
        curTV.tv_sec = nh_.param("receive_timeout", 1);
        curTV.tv_usec = 0;
        return elfin_client_.connect();
    }
}