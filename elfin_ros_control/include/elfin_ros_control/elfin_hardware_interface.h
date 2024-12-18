/*
Created on Tue Sep 25 10:16 2018

@author: Cong Liu

 Software License Agreement (BSD License)

 Copyright (c) 2018, Han's Robot Co., Ltd.
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.
  * Neither the name of the copyright holders nor the names of its
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
*/
// author: Cong Liu

#ifndef ELFIN_ROS_CONTROL_HW_INTERFACE
#define ELFIN_ROS_CONTROL_HW_INTERFACE

#include <ros/ros.h>
#include <urdf/model.h>
#include <pthread.h>
#include <time.h>
#include <stdlib.h>
#include <math.h>
#include <string>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>

#include <sensor_msgs/JointState.h>
#include <realtime_tools/realtime_publisher.h>
#include <tf2_msgs/TFMessage.h>
#include <tf/tf.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/posvel_command_interface.h>
#include <elfin_hardware_interface/postrq_command_interface.h>
#include <elfin_hardware_interface/posveltrq_command_interface.h>
#include <hardware_interface/robot_hw.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <controller_manager/controller_manager.h>
#include <controller_manager_msgs/ListControllers.h>
#include <pass_through_controllers/trajectory_interface.h>

#include <elfin_ros_control/elfin_data_log_handler.h>
// #include <elfin_industrial_driver/elfin_command/elfin_trajectory_interface.h>
// #include <elfin_industrial_driver/elfin/elfin_driver.h>
// #include <elfin_ros_control/elfin_client_ros.h>
#include <industrial_robot_status_interface/industrial_robot_status_interface.h>

// #include "elfin_industrial_driver/elfin_tcp/elfin_tcp_control.h"
// #include "elfin_industrial_driver/elfin_tcp/elfin_tcp_data.h"
// #include "elfin_industrial_driver/elfin_tcp/elfin_tcp_state.h"
#include "elfin_tcp/elfin_tcp_data.h"
#include "elfin_tcp/elfin_tcp_control.h"
#include "elfin_tcp/elfin_tcp_state.h"

#include "elfin_robot_msgs/SetInt16.h"
#include "elfin_robot_msgs/SetFloat64.h"
#include "elfin_robot_msgs/ElfinDOCmd.h"

#include <iostream>
#include <fstream>

namespace elfin_ros_control {

typedef struct{
    std::string name;
    double position;
    double velocity;
    double effort;

    double position_cmd;
    double velocity_cmd;
    double vel_ff_cmd;
    double effort_cmd;

    double last_position;
}AxisInfo;

typedef struct{
    AxisInfo axis1;
    // AxisInfo axis2;
}ModuleInfo;

enum class RobotState
{
    PAUSED,
    Running,
    Stop
};

class ElfinHWInterface : public hardware_interface::RobotHW
{
public:
    ElfinHWInterface();
    ~ElfinHWInterface();

    bool init(ros::NodeHandle &nh, ros::NodeHandle &root_nh);
    bool prepareSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
                       const std::list<hardware_interface::ControllerInfo> &stop_list);

    void doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
                  const std::list<hardware_interface::ControllerInfo> &stop_list);

    void read_init();
    void read_update(const ros::Time &time_now);
    void write_update();
    bool freedriverState = false;

    bool GetRobotProgramState();
    bool handleRobotProgramState();
    bool ResetElfinControllers();

private:
    std::vector<ModuleInfo> module_infos_;

    hardware_interface::JointStateInterface jnt_state_interface_;
    hardware_interface::PositionJointInterface jnt_position_cmd_interface_;
    // hardware_interface::EffortJointInterface jnt_effort_cmd_interface_;
    // elfin_hardware_interface::PosTrqJointInterface jnt_postrq_cmd_interface_;
    // hardware_interface::PosVelJointInterface jnt_posvel_cmd_interface_;
    // elfin_hardware_interface::PosVelTrqJointInterface jnt_posveltrq_cmd_interface_;
    // hardware_interface::JointTrajectoryInterface jnt_traj_interface_;

    bool joint_position_cmd_interface_is_running = false;
    // bool joint_trajectory_interface_is_running = false;

    // ros::NodeHandle n_;
    // ros::NodeHandle root_n;
    HRDataClient HRClient;
    HRControlClient HRCmdClient;
    HRStateClient HRStateClient_;

    std::string robot_ip;
    int elfin_servo_port;
    int elfin_cds_port;
    int elfin_tcp_cmd_port;
    int elfin_tcp_state_port;
    
    bool start_ServoJ_State = false;
    int stop_send_time = 0;

    ros::Time read_update_time_;
    ros::Duration read_update_dur_;
    std::vector<int> slave_no_;
    std::vector<std::string> joint_names_;
    std::vector<bool> pre_switch_flags_;
    geometry_msgs::TransformStamped tcp_transform_;
    double act_posPCS[6];
    double cur_override;
    bool enable_state;
    bool moving_state;
    bool stop_pushServo;
    int curFSM_state;
    int error_code;
    double servoTime;
    double lookaheadTime;
    std::vector<int> box_do_state;
    std::vector<int>  box_di_state;
    std::vector<int>  box_co_state;
    std::vector<int> box_ci_state;
    std::vector<int> end_do_state;
    std::vector<int> end_di_state;

    std::vector<boost::shared_ptr<boost::mutex> > pre_switch_mutex_ptrs_;
    std::string elfin_controller_name_;
    std::vector<std::string> controller_joint_names_;

    bool isModuleMoving(int module_num);
    double motion_threshold_;

    // 控制器切换
    bool control_without_ros = true;
    
    // 末端力矩转换
    // void transForceTorque();
    // 获取当前控制器TCP工具姿态
    // void getTcpToolPose();
    // 发布TCP工具姿态
    // void publishTcpData();
    // 发布安全模式等级
    // void publishSafetyMode();
    // 获取机械臂当前状态
    // void ElfinRobotState(); 

    // ROS-I状态更新
    industrial_robot_status_interface::RobotStatus robot_status_resource_{};
    industrial_robot_status_interface::IndustrialRobotStatusInterface robot_status_interface_{};

    // bool setGroupPosMode(const std::vector<int>& module_no);
    // bool setGroupTrqMode(const std::vector<int>& module_no);
    void extractToolPose(const ros::Time& timestamp);
    // 状态/IO/位置等数据发布
    void publishPose();
    void publishOverride();
    void publishBoxDO();
    void publishBoxDI();
    void publishBoxCO();
    void publishBoxCI();
    void publishEndDO();
    void publishEndDI();
    void publishEnable();
    void publishCurFSM();
    void publishMoving();
    void publishErrorCode();
    void publishControlState();
    void update_robotState();
    // 相关服务回调
    bool enableRobot(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp);
    bool disableRobot(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp);
    bool resetRobot(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp);
    bool openFreeDriver(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp);
    bool closeFreeDriver(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp);
    bool setOverride(elfin_robot_msgs::SetFloat64::Request& req, elfin_robot_msgs::SetFloat64::Response& resp);
    bool setSecurityLevel(elfin_robot_msgs::SetInt16::Request& req, elfin_robot_msgs::SetInt16::Response& resp);
    bool setBoxDO(elfin_robot_msgs::ElfinDOCmd::Request& req, elfin_robot_msgs::ElfinDOCmd::Response& resp);
    bool setBoxCO(elfin_robot_msgs::ElfinDOCmd::Request& req, elfin_robot_msgs::ElfinDOCmd::Response& resp);
    bool setEndDO(elfin_robot_msgs::ElfinDOCmd::Request& req, elfin_robot_msgs::ElfinDOCmd::Response& resp);
    bool openRosControl(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp);
    bool closeRosControl(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp);
    bool stopActCtrlrs(std_srvs::SetBool::Response &resp);
    bool startElfinCtrlr(std_srvs::SetBool::Response &resp);
    bool getRosControlState(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp);
    bool StopRobotMove(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp);
    ros::ServiceClient switch_controller_client_;
    ros::ServiceClient list_controllers_client_;

    // 实时话题
    std::unique_ptr<realtime_tools::RealtimePublisher<tf2_msgs::TFMessage>> tcp_pose_pub_;
    std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64>> override_pub_;
    
    ros::Publisher box_do_pub_;
    ros::Publisher box_di_pub_;
    ros::Publisher box_co_pub_;
    ros::Publisher box_ci_pub_;
    ros::Publisher end_do_pub_;
    ros::Publisher end_di_pub_;
    ros::Publisher enable_state_pub_;
    ros::Publisher robot_moving_state_pub_;
    ros::Publisher robot_error_code_pub_;
    ros::Publisher robot_cur_FSM_pub_;
    ros::Publisher control_state_pub_;

    ros::ServiceServer enable_robot_;
    ros::ServiceServer disable_robot_;
    ros::ServiceServer reset_robot_;
    ros::ServiceServer stop_move_;
    ros::ServiceServer open_freeDriver_;
    ros::ServiceServer close_freeDriver_;
    ros::ServiceServer set_override_;
    ros::ServiceServer set_security_level_;
    ros::ServiceServer set_box_do_;
    ros::ServiceServer set_end_do_;
    ros::ServiceServer set_box_co_;
    ros::ServiceServer get_control_state_;
    ros::ServiceServer open_ros_controller_;
    ros::ServiceServer close_ros_controller_;
};
}
#endif
