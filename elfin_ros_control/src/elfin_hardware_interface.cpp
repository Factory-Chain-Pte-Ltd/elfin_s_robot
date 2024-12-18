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
// update the include file
#include "elfin_ros_control/elfin_hardware_interface.h"

#include <pluginlib/class_list_macros.hpp>
// #include <elfin_industrial_driver/elfin_command/elfin_trajectory_interface.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <cartesian_control_msgs/FollowCartesianTrajectoryAction.h>
#include <eigen3/Eigen/Geometry>
#include <stdexcept>

using industrial_robot_status_interface::RobotMode;
using industrial_robot_status_interface::TriState;

namespace elfin_ros_control {

ElfinHWInterface::ElfinHWInterface()
{
}

ElfinHWInterface::~ElfinHWInterface()
{

}

bool ElfinHWInterface::init(ros::NodeHandle &n_, ros::NodeHandle &root_n)
{
    int elfin_data_port[3] = {10004,10005,10006};
    n_.param<std::string>("elfin_robot_ip", robot_ip, "192.168.0.10");
    elfin_cds_port = n_.param("elfin_cds_data_port", 8893);
    if(elfin_cds_port != 8893)
    {
        ROS_ERROR_STREAM("Elfin data base prot error.");
        return false;
    }
    elfin_tcp_cmd_port = n_.param("elfin_tcp_cmd_port", 10003);
    if(elfin_tcp_cmd_port != 10003)
    {
        ROS_ERROR_STREAM("Elfin tcp cmd prot error.");
        return false;
    }
    elfin_tcp_state_port = n_.param("elfin_tcp_state_port", 10004);
    auto it = std::find(std::begin(elfin_data_port), std::end(elfin_data_port), elfin_tcp_state_port);
    if(it == std::end(elfin_data_port))
    {
        ROS_ERROR_STREAM("elfin tcp state port error.");
        return false;
    }

    servoTime = n_.param("servoj_time",0.01);
    if((servoTime>0.2) || (servoTime<0.001))
    {
        ROS_ERROR_STREAM("servoj time overstep the limit, must be in range[0.001s,0.2s]");
        return false;
    }
    lookaheadTime = n_.param("servoj_lookahread_time",0.01);
    if((lookaheadTime>0.2) || (lookaheadTime<0.001))
    {
        ROS_ERROR_STREAM("lookahead time overstep the limit, must be in range[0.001s,0.2s]");
        return false;
    }

    elfin_ros_control::startElfinLogHandler();
    int connect_cds = HRClient.Connect(robot_ip, elfin_cds_port);
    int connect_cmd = HRCmdClient.connectToRobot(robot_ip, elfin_tcp_cmd_port);
    int connect_state = HRStateClient_.Connect(robot_ip, elfin_tcp_state_port);
    if(connect_cds == -1 || connect_cmd == -1 || connect_state == -1)
    {
        HR_LOG_ERROR("Cann't not connect to elfin robot server, please check the ip and port.");
        return false;
    }else{
        HR_LOG_INFO("Connect Elfin Robot server, start elfin ros control init...");
    }
    int slave_no_array_default[6]={1, 2, 3, 4, 5, 6};
    std::vector<int> slave_no_default;
    slave_no_default.clear();
    slave_no_default.reserve(6);
    for(int i=0; i<6; i++)
    {
        slave_no_default.push_back(slave_no_array_default[i]);
    }
    n_.param<std::vector<int> >("slave_no", slave_no_, slave_no_default);

    std::vector<std::string> joint_names_default;
    joint_names_default.clear();
    joint_names_default.reserve(slave_no_.size());

    for(int i=0; i<slave_no_.size(); i++)
    {
        std::string num_1=boost::lexical_cast<std::string>(i+1);
        // std::string num_2=boost::lexical_cast<std::string>(2*(i+1));
        std::string name_1="elfin_joint";
        // std::string name_2="elfin_joint";
        name_1.append(num_1);
        // name_2.append(num_2);

        joint_names_default.push_back(name_1);
        // joint_names_default.push_back(name_2);
    }
    n_.param<std::vector<std::string> >("joint_names", joint_names_, joint_names_default);

    // // Initialize module_infos_
    module_infos_.clear();
    for(size_t j=0; j<slave_no_.size(); j++)
    {
        ModuleInfo module_info_tmp;
        
        module_info_tmp.axis1.name=joint_names_default[j];

    // module_info_tmp.axis2.name=joint_names_default[2*j+1];

        module_infos_.push_back(module_info_tmp);
        std::cout<<"module joint name: "<<module_info_tmp.axis1.name<<std::endl;
    }
    
    elfin_controller_name_=n_.param<std::string>("controller_name", "elfin_arm_controller");

    switch_controller_client_=root_n.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");
    list_controllers_client_=root_n.serviceClient<controller_manager_msgs::ListControllers>("/controller_manager/list_controllers");


    tcp_pose_pub_.reset(new realtime_tools::RealtimePublisher<tf2_msgs::TFMessage>(n_, "/elfin_pose", 1, true));
    override_pub_.reset(new realtime_tools::RealtimePublisher<std_msgs::Float64>(n_, "/elfin_override", 1, true));

    box_do_pub_ = n_.advertise<std_msgs::Int32MultiArray>("elfin_boxDO", 1);
    box_di_pub_ = n_.advertise<std_msgs::Int32MultiArray>("elfin_boxDI",1);
    box_co_pub_ = n_.advertise<std_msgs::Int32MultiArray>("elfin_boxCO",1);
    box_ci_pub_ = n_.advertise<std_msgs::Int32MultiArray>("elfin_boxCI",1);
    end_do_pub_ = n_.advertise<std_msgs::Int32MultiArray>("elfin_endDO",1);
    end_di_pub_ = n_.advertise<std_msgs::Int32MultiArray>("elfin_endDI",1);
    enable_state_pub_ = n_.advertise<std_msgs::Bool>("elfin_enable",1);
    robot_moving_state_pub_ = n_.advertise<std_msgs::Bool>("elfin_moving",1);
    robot_error_code_pub_ = n_.advertise<std_msgs::Int32>("elfin_errorCode",1);
    robot_cur_FSM_pub_ = n_.advertise<std_msgs::Int32>("elfin_curFSM",1);
    control_state_pub_ = n_.advertise<std_msgs::Bool>("controller_state",1);

    enable_robot_ = n_.advertiseService("enable_robot", &ElfinHWInterface::enableRobot, this);
    disable_robot_ = n_.advertiseService("disable_robot", &ElfinHWInterface::disableRobot, this);
    reset_robot_ = n_.advertiseService("reset_robot", &ElfinHWInterface::resetRobot, this);
    open_freeDriver_ = n_.advertiseService("open_freedriver", &ElfinHWInterface::openFreeDriver, this);
    close_freeDriver_ = n_.advertiseService("close_freedriver", &ElfinHWInterface::closeFreeDriver, this);
    set_override_ = n_.advertiseService("set_override", &ElfinHWInterface::setOverride, this);
    set_security_level_ = n_.advertiseService("set_security_level", &ElfinHWInterface::setSecurityLevel, this);
    set_box_do_ = n_.advertiseService("set_boxDO", &ElfinHWInterface::setBoxDO, this);
    set_end_do_ = n_.advertiseService("set_endDO", &ElfinHWInterface::setEndDO, this);
    set_box_co_ = n_.advertiseService("set_boxCO", &ElfinHWInterface::setBoxCO, this);
    get_control_state_ = n_.advertiseService("get_ros_control_state", &ElfinHWInterface::getRosControlState, this);
    open_ros_controller_ = n_.advertiseService("open_ros_control",&ElfinHWInterface::openRosControl,this);
    close_ros_controller_ = n_.advertiseService("close_ros_control", &ElfinHWInterface::closeRosControl,this);
    stop_move_ = n_.advertiseService("stop_robotMove", &ElfinHWInterface::StopRobotMove, this);
    // Initialize pre_switch_flags_ and pre_switch_mutex_ptrs_
    pre_switch_flags_.resize(6);
    for(int i=0; i<pre_switch_flags_.size(); i++)
    {
        pre_switch_flags_[i]=false;
    }

    pre_switch_mutex_ptrs_.resize(6);
    for(int i=0; i<pre_switch_mutex_ptrs_.size(); i++)
    {
        pre_switch_mutex_ptrs_[i]=boost::shared_ptr<boost::mutex>(new boost::mutex);
    }

    // Initialize the state and command interface
    for(size_t i=0; i<module_infos_.size(); i++)
    {
        hardware_interface::JointStateHandle jnt_state_handle_tmp1(module_infos_[i].axis1.name,
                                                                  &module_infos_[i].axis1.position,
                                                                  &module_infos_[i].axis1.velocity,
                                                                  &module_infos_[i].axis1.effort);
        jnt_state_interface_.registerHandle(jnt_state_handle_tmp1);
    }
    registerInterface(&jnt_state_interface_);

    for(size_t i=0; i<module_infos_.size(); i++)
    {
        hardware_interface::JointHandle jnt_handle_tmp1(jnt_state_interface_.getHandle(module_infos_[i].axis1.name),
                                                       &module_infos_[i].axis1.position_cmd);
        jnt_position_cmd_interface_.registerHandle(jnt_handle_tmp1);
    }
    registerInterface(&jnt_position_cmd_interface_);

    robot_status_interface_.registerHandle(industrial_robot_status_interface::IndustrialRobotStatusHandle(
                "industrial_robot_status_handle", robot_status_resource_));
    registerInterface(&robot_status_interface_);

    return true;
}

bool ElfinHWInterface::stopActCtrlrs(std_srvs::SetBool::Response &resp)
{
    // Check list controllers service
    if(!list_controllers_client_.exists())
    {
        resp.message="there is no controller manager";
        resp.success=false;
        return false;
    }

    // Find controllers to stop
    controller_manager_msgs::ListControllers::Request list_controllers_request;
    controller_manager_msgs::ListControllers::Response list_controllers_response;
    list_controllers_client_.call(list_controllers_request, list_controllers_response);
    std::vector<std::string> controllers_to_stop;
    controllers_to_stop.clear();

    controller_joint_names_.clear();
    for(int i=0; i<list_controllers_response.controller.size(); i++)
    {
        std::string name_tmp=list_controllers_response.controller[i].name;
        std::vector<controller_manager_msgs::HardwareInterfaceResources> resrc_tmp=list_controllers_response.controller[i].claimed_resources;
        if(strcmp(name_tmp.c_str(), elfin_controller_name_.c_str())==0)
        {
            for(int j=0; j<resrc_tmp.size(); j++)
            {
                controller_joint_names_.insert(controller_joint_names_.end(), resrc_tmp[j].resources.begin(),
                                               resrc_tmp[j].resources.end());
            }
            break;
        }
    }

    for(int i=0; i<list_controllers_response.controller.size(); i++)
    {
        std::string state_tmp=list_controllers_response.controller[i].state;
        std::string name_tmp=list_controllers_response.controller[i].name;
        std::vector<controller_manager_msgs::HardwareInterfaceResources> resrc_tmp=list_controllers_response.controller[i].claimed_resources;
        if(strcmp(state_tmp.c_str(), "running")==0)
        {
            bool break_flag=false;
            for(int j=0; j<resrc_tmp.size(); j++)
            {
                for(int k=0; k<controller_joint_names_.size(); k++)
                {
                if(std::find(resrc_tmp[j].resources.begin(), resrc_tmp[j].resources.end(),
                                 controller_joint_names_[k])!=resrc_tmp[j].resources.end())
                    {
                        break_flag=true;
                        controllers_to_stop.push_back(name_tmp);
                    }
                    if(break_flag)
                    {
                        break;
                    }
                }
                if(break_flag)
                {
                    break;
                }
            }
        }
    }

    // Stop active controllers
    if(controllers_to_stop.size()>0)
    {
        // Check switch controller service
        if(!switch_controller_client_.exists())
        {
            resp.message="there is no controller manager";
            resp.success=false;
            return false;
        }

        // Stop active controllers
        controller_manager_msgs::SwitchController::Request switch_controller_request;
        controller_manager_msgs::SwitchController::Response switch_controller_response;
        switch_controller_request.start_controllers.clear();
        switch_controller_request.stop_controllers=controllers_to_stop;
        switch_controller_request.strictness=switch_controller_request.STRICT;

        switch_controller_client_.call(switch_controller_request, switch_controller_response);
        if(!switch_controller_response.ok)
        {
            resp.message="Failed to stop active controllers";
            resp.success=false;
            return false;
        }
    }
    control_without_ros = true;
    return true;
}

bool ElfinHWInterface::startElfinCtrlr(std_srvs::SetBool::Response &resp)
{
    // Check switch controller service
    if(!switch_controller_client_.exists())
    {
        resp.message="there is no controller manager";
        resp.success=false;
        return false;
    }
    controller_manager_msgs::SwitchController::Request switch_controller_request;
    controller_manager_msgs::SwitchController::Response switch_controller_response;
    controller_manager_msgs::ListControllers::Request list_controllers_request;
    controller_manager_msgs::ListControllers::Response list_controllers_response;
    list_controllers_client_.call(list_controllers_request, list_controllers_response);
    // Start active controllers
     for(int i=0; i<list_controllers_response.controller.size(); i++)
    {
        std::string name_tmp=list_controllers_response.controller[i].name;
        std::string state_tmp=list_controllers_response.controller[i].state;
        std::vector<controller_manager_msgs::HardwareInterfaceResources> resrc_tmp=list_controllers_response.controller[i].claimed_resources;
        if(strcmp(name_tmp.c_str(), elfin_controller_name_.c_str())==0)
        {
            if(strcmp(state_tmp.c_str(), "running")!=0)
            {
                switch_controller_request.start_controllers.clear();
                switch_controller_request.start_controllers.push_back(elfin_controller_name_);
                switch_controller_request.stop_controllers.clear();
                switch_controller_request.strictness=switch_controller_request.STRICT;
                switch_controller_client_.call(switch_controller_request, switch_controller_response);
                if(!switch_controller_response.ok)
                {
                    resp.message="Failed to start the default controller";
                    resp.success=false;
                    return false;
                }
            }
        }
    }
    control_without_ros = false;
    return true;
}

void ElfinHWInterface::extractToolPose(const ros::Time& timestamp)
{
    
    tf2::Quaternion rotation;
    geometry_msgs::Quaternion q;
    q = tf::createQuaternionMsgFromRollPitchYaw(act_posPCS[3]/(180/M_PI),act_posPCS[4]/(180/M_PI),act_posPCS[5]/(180/M_PI));
    tcp_transform_.header.stamp = timestamp;
    tcp_transform_.transform.translation.x = act_posPCS[0]/1000.0;
    tcp_transform_.transform.translation.y = act_posPCS[1]/1000.0;
    tcp_transform_.transform.translation.z = act_posPCS[2]/1000.0;

    tcp_transform_.transform.rotation.x = q.x;
    tcp_transform_.transform.rotation.y = q.y;
    tcp_transform_.transform.rotation.z = q.z;
    tcp_transform_.transform.rotation.w = q.w;
}

void ElfinHWInterface::publishPose()
{
  if (tcp_pose_pub_)
  {
    if (tcp_pose_pub_->trylock())
    {
      tcp_pose_pub_->msg_.transforms.clear();
      tcp_pose_pub_->msg_.transforms.push_back(tcp_transform_);
      tcp_pose_pub_->unlockAndPublish();
    }
}
}

void ElfinHWInterface::publishOverride()
{
    if(override_pub_)
    {
        if(override_pub_->trylock())
        {
            override_pub_->msg_.data = cur_override;
            override_pub_->unlockAndPublish();
        }
    }
}

void ElfinHWInterface::publishBoxDO()
{
    std_msgs::Int32MultiArray do_data;
    do_data.data = box_do_state;
    box_do_pub_.publish(do_data);
}

void ElfinHWInterface::publishBoxDI()
{
    std_msgs::Int32MultiArray di_data;
    di_data.data = box_di_state;
    box_di_pub_.publish(di_data);
}

void ElfinHWInterface::publishBoxCO()
{
    std_msgs::Int32MultiArray co_data;
    co_data.data = box_co_state;
    box_co_pub_.publish(co_data);
}

    void ElfinHWInterface::publishBoxCI()
{
    std_msgs::Int32MultiArray ci_data;
    ci_data.data = box_ci_state;
    box_ci_pub_.publish(ci_data);
}

void ElfinHWInterface::publishEndDO()
{
    std_msgs::Int32MultiArray end_do_data;
    end_do_data.data = end_do_state;
    end_do_pub_.publish(end_do_data);
}

    void ElfinHWInterface::publishEndDI()
{
    std_msgs::Int32MultiArray end_di_data;
    end_di_data.data = end_di_state;
    end_di_pub_.publish(end_di_data);
}

void ElfinHWInterface::publishEnable()
{
    std_msgs::Bool enable_state_data;
    enable_state_data.data = enable_state;
    enable_state_pub_.publish(enable_state_data);
}

void ElfinHWInterface::publishCurFSM()
{
    std_msgs::Int32 curFSM_state_data;
    curFSM_state_data.data = curFSM_state;
    robot_cur_FSM_pub_.publish(curFSM_state_data);
}

void ElfinHWInterface::publishMoving()
{
    std_msgs::Bool moving_state_data;
    moving_state_data.data = moving_state;
    robot_moving_state_pub_.publish(moving_state_data);
}

void ElfinHWInterface::publishErrorCode()
{
                std_msgs::Int32 error_code_data;
    error_code_data.data = error_code;
    robot_error_code_pub_.publish(error_code_data);
}

void ElfinHWInterface::publishControlState()
{
    std_msgs::Bool control_state_data;
    control_state_data.data = control_without_ros;
    control_state_pub_.publish(control_state_data);
}

bool ElfinHWInterface::enableRobot(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp)
{
    if(!req.data)
    {
        resp.success=false;
        resp.message="require's data is false";
        return true;
    }else{
        string res;
        std_srvs::SetBool::Response resp_tmp;
        if(startElfinCtrlr(resp_tmp))
        {
            int nRet = HRCmdClient.enable_robot(res);
            if(nRet == 0)
            {
                resp.success = true;
                resp.message = res;
            }else{
                resp.success = false;
                resp.message = res;
            }
        }else{
            resp = resp_tmp;
        }
        return true;
    }
}

bool ElfinHWInterface::disableRobot(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp)
{
    if(!req.data)
    {
        resp.success=false;
        resp.message="require's data is false";
    return true;
    }else{
        string res;
        std_srvs::SetBool::Response resp_tmp;
        if(stopActCtrlrs(resp_tmp))
        {
            int nRet = HRCmdClient.disable_robot(res);
            if(nRet == 0)
            {
                resp.success = true;
                resp.message = res;
            }else{
                resp.success = false;
                resp.message = res;
            }
        }else{
            resp = resp_tmp;
        }
        return true;
    }
}

bool ElfinHWInterface::resetRobot(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp)
{
    if(!req.data)
    {
        resp.success=false;
        resp.message="require's data is false";
        return true;
    }else{
        string res;
        std_srvs::SetBool::Response resp_tmp;
        int nRet = HRCmdClient.reset_robot(res);
        if(nRet == 0)
        {
            resp.success = true;
            resp.message = res;
        }else{
            resp.success = false;
            resp.message = res;
        }
        return true;
    }
}

bool ElfinHWInterface::openFreeDriver(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp)
{
    if(!req.data)
    {
        resp.success=false;
        resp.message="require's data is false";
        return true;
    }else{
        string res;
        int nRet = HRCmdClient.open_freedriver(res);
        if(nRet == 0)
        {
            resp.success = true;
            resp.message = res;
            freedriverState = true;
        }else{
            resp.success = false;
            resp.message = res;
        }
        return true;
    }
}

bool ElfinHWInterface::closeFreeDriver(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp)
{
    if(!req.data)
    {
        resp.success=false;
        resp.message="require's data is false";
        return true;
    }else{
        string res;
        int nRet = HRCmdClient.close_freedriver(res);
        if(nRet == 0)
        {
        resp.success = true;
            resp.message = res;
            freedriverState = false;
        }else{
            resp.success = false;
            resp.message = res;
        }
        return true;
    }
}

bool ElfinHWInterface::setOverride(elfin_robot_msgs::SetFloat64::Request& req, elfin_robot_msgs::SetFloat64::Response& resp)
{
    string res;
    int nRet = HRCmdClient.set_override(req.data,res);
    if(nRet == 0)
    {
        resp.success = true;
        resp.message = res;
    }else{
        resp.success = false;
        resp.message = res;
    }
    return true;
}

bool ElfinHWInterface::setSecurityLevel(elfin_robot_msgs::SetInt16::Request& req, elfin_robot_msgs::SetInt16::Response& resp)
{
    string res;
    int nRet = HRCmdClient.set_security_level(6-req.data,res);
    if(nRet == 0)
    {
        resp.success = true;
        resp.message = res;
    }else{
        resp.success = false;
        resp.message = res;
    }
    return true;
}

bool ElfinHWInterface::setBoxDO(elfin_robot_msgs::ElfinDOCmd::Request& req, elfin_robot_msgs::ElfinDOCmd::Response& resp)
{
    string res;
    int nRet = HRCmdClient.set_robot_do(req.bit, req.val, res);
    if(nRet == 0)
    {
        resp.success = true;
        resp.message = res;
    }else{
        resp.success = false;
        resp.message = res;
    }
    return true;
}

bool ElfinHWInterface::setBoxCO(elfin_robot_msgs::ElfinDOCmd::Request& req, elfin_robot_msgs::ElfinDOCmd::Response& resp)
{
    string res;
    int nRet = HRCmdClient.set_robot_co(req.bit, req.val, res);
    if(nRet == 0)
    {
        resp.success = true;
        resp.message = res;
    }else{
        resp.success = false;
        resp.message = res;
    }
    return true;
}

bool ElfinHWInterface::setEndDO(elfin_robot_msgs::ElfinDOCmd::Request& req, elfin_robot_msgs::ElfinDOCmd::Response& resp)
{
    string res;
    int nRet = HRCmdClient.set_end_do(req.bit, req.val, res);
    if(nRet == 0)
    {
        resp.success = true;
        resp.message = res;
    }else{
        resp.success = false;
        resp.message = res;
    }
    return true;
    }

bool ElfinHWInterface::getRosControlState(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp)
{
        if(!req.data)
    {
        resp.success=false;
        resp.message="require's data is false";
        return true;
    }else{
        controller_manager_msgs::ListControllers::Request list_controllers_request;
        controller_manager_msgs::ListControllers::Response list_controllers_response;
        list_controllers_client_.call(list_controllers_request, list_controllers_response);
        // Start active controllers
        for(int i=0; i<list_controllers_response.controller.size(); i++)
        {
            std::string name_tmp=list_controllers_response.controller[i].name;
            std::string state_tmp=list_controllers_response.controller[i].state;
            std::vector<controller_manager_msgs::HardwareInterfaceResources> resrc_tmp=list_controllers_response.controller[i].claimed_resources;
            if(strcmp(name_tmp.c_str(), elfin_controller_name_.c_str())==0)
            {
                if(strcmp(state_tmp.c_str(), "running")!=0)
                {
                    resp.success = false;
                    resp.message = "elfin ros controller not in running state";
                }else{
                    resp.success = true;
                    resp.message = "elfin ros controller  in running state";
                }
                return  true;
            }
        }
    }
    return true;
}

bool ElfinHWInterface::openRosControl(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp)
{
    if(!req.data)
    {
        resp.success=false;
        resp.message="require's data is false";
        return true;
    }else{
        std_srvs::SetBool::Response resp_tmp;
        bool start_state = startElfinCtrlr(resp_tmp);
        if(start_state)
        {
            resp.success = true;
            resp.message = "Success to start elflin ros controller";
        }else{
            resp.success = false;
            resp.message = "Fail to start elflin ros controller";
        }
        return true;
    }
}

bool ElfinHWInterface::closeRosControl(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp)
{
    if(!req.data)
    {
        resp.success=false;
        resp.message="require's data is false";
        return true;
    }else{
        std_srvs::SetBool::Response resp_tmp;
        bool stop_state = stopActCtrlrs(resp_tmp);
        if(stop_state)
        {
            resp.success = true;
            resp.message = "Success to stop elflin ros controller";
        }else{
            resp.success = false;
            resp.message = "Fail to stop elflin ros controller";
        }
        return true;
    }
}

bool ElfinHWInterface::StopRobotMove(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp)
{
    if(!req.data)
    {
        resp.success=false;
        resp.message="require's data is false";
        return true;
    }else{
        std::string res;
        std_srvs::SetBool::Response resp_tmp;
        int nRet = HRCmdClient.StopMove();
        if(nRet == 0)
        {
            stop_pushServo = true;
            std_srvs::SetBool::Response stop_resp_tmp;
            if(stopActCtrlrs(stop_resp_tmp))
            {
                for(size_t i=0; i<module_infos_.size(); i++)
                {
                    module_infos_[i].axis1.position = HRClient.ActPosAcs[i]/(180/M_PI);
                    // module_infos_[i].axis2.position = HRClient.ActPosAcs[i*2+1]/(180/M_PI);
                    module_infos_[i].axis1.position_cmd= module_infos_[i].axis1.position;
                    // module_infos_[i].axis2.position_cmd= module_infos_[i].axis2.position;
                }
            }
            sleep(1.0);
            std_srvs::SetBool::Response start_resp_tmp;
            startElfinCtrlr(start_resp_tmp);
            resp.success = true;
            resp.message = "stop move";

        }else{
            resp.success = false;
            resp.message = "stop move fail";
        }
        return true;
    }
}

void ElfinHWInterface::update_robotState()
{
    robot_status_resource_.mode = HRStateClient_.autoMode? RobotMode::MANUAL : RobotMode::AUTO;

    // 急停状态
    robot_status_resource_.e_stopped = curFSM_state ==5 ? TriState::TRUE : TriState::FALSE;

    // 本体供电，电压绝对值小于1.0V认为已供电
    if(abs(HRStateClient_.power_on_voltage - 48.0) < 1.0)
    {
        robot_status_resource_.drives_powered = TriState::TRUE;
    }else{
        robot_status_resource_.drives_powered = TriState::FALSE;
    }
    // 运动状态  运动中/未运动
    robot_status_resource_.in_motion = moving_state ? TriState::TRUE : TriState::FALSE;

    // 错误状态，错误码大于0进入错误状态
    robot_status_resource_.in_error = HRStateClient_.robot_error_code>0 ? TriState::TRUE : TriState::FALSE;
    // 错误码
    robot_status_resource_.error_code = HRStateClient_.robot_error_code;

    // 错误状态，不允许运动
    if(robot_status_resource_.in_error == TriState::TRUE)
    {
        robot_status_resource_.motion_possible = TriState::FALSE;
    }
    // 就绪状态，可运动
    else if(HRStateClient_.robot_curFSM_state == 33)
    {
        robot_status_resource_.motion_possible = TriState::TRUE;
    }
    else
    {
        robot_status_resource_.motion_possible = TriState::FALSE;
    }
}

bool ElfinHWInterface::prepareSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
                                     const std::list<hardware_interface::ControllerInfo> &stop_list)
{
    std::list<hardware_interface::ControllerInfo>::const_iterator iter;

    if(!stop_list.empty())
    {
        for(iter=stop_list.begin(); iter!=stop_list.end(); iter++)
        {
            std::vector<hardware_interface::InterfaceResources> stop_resrcs=iter->claimed_resources;
            for(int i=0; i<stop_resrcs.size(); i++)
            {
                std::vector<int> module_no;
                module_no.clear();

                for(int j=0; j<module_infos_.size(); j++)
                {
                    if(stop_resrcs[i].resources.find(module_infos_[j].axis1.name)!=stop_resrcs[i].resources.end())
                    //    || stop_resrcs[i].resources.find(module_infos_[j].axis2.name)!=stop_resrcs[i].resources.end())
                    {
                        module_no.push_back(j);
                    }
                }
                std::vector<int> module_no_tmp=module_no;
            }
        }
    }

    if(start_list.empty())
        return true;
    
    for(iter=start_list.begin(); iter!=start_list.end(); iter++)
    {
        std::vector<hardware_interface::InterfaceResources> start_resrcs=iter->claimed_resources;
        for(int i=0; i<start_resrcs.size(); i++)
        {
            std::vector<int> module_no;
            module_no.clear();

            for(int j=0; j<module_infos_.size(); j++)
            {
                bool axis1_exist=(start_resrcs[i].resources.find(module_infos_[j].axis1.name)!=start_resrcs[i].resources.end());
                // bool axis2_exist=(start_resrcs[i].resources.find(module_infos_[j].axis2.name)!=start_resrcs[i].resources.end());
                std::cout<<"module axis1 name: "<<module_infos_[j].axis1.name<<","<<axis1_exist<<std::endl;

                if(axis1_exist)//|| axis2_exist
                {
                    module_no.push_back(j);
                    
                }
                // else{
                //         ROS_ERROR("%s  not includes %s", iter->name.c_str(), module_infos_[j].axis1.name.c_str());
                //         return false;
                    
                // }
            }

            if(strcmp(start_resrcs[i].hardware_interface.c_str(), "hardware_interface::PositionJointInterface")==0)
            {
                std::vector<int> module_no_tmp=module_no;
            }
            else
            {
                if(!module_no.empty())
                {
                  ROS_ERROR("Elfin doesn't support %s", start_resrcs[i].hardware_interface.c_str());
                  return false;
                }
            }
        }
    }
    return true;
}

void ElfinHWInterface::doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
                                const std::list<hardware_interface::ControllerInfo> &stop_list)
{
    for(size_t i=0; i<pre_switch_flags_.size(); i++)
    {
        boost::mutex::scoped_lock pre_switch_flags_lock(*pre_switch_mutex_ptrs_[i]);
        if(pre_switch_flags_[i])
        {
            module_infos_[i].axis1.velocity_cmd=0;
            module_infos_[i].axis1.vel_ff_cmd=0;
            module_infos_[i].axis1.effort_cmd=0;

            // module_infos_[i].axis2.velocity_cmd=0;
            // module_infos_[i].axis2.vel_ff_cmd=0;
            // module_infos_[i].axis2.effort_cmd=0;

            pre_switch_flags_[i]=false;
        }
        pre_switch_flags_lock.unlock();
    }
// 控制器标志
    for(auto& controller_it : stop_list)
    {
        for(auto& resource_it : controller_it.claimed_resources)
        {
            // if (checkControllerClaims(resource_it.resources))
            // {

            // }
            if(resource_it.hardware_interface == "hardware_interface::PositionJointInterface")
            {
                joint_position_cmd_interface_is_running = false;
            }
            // if(resource_it.hardware_interface == "hardware_interface::TrajectoryInterface<control_msgs::"
            //                                      "FollowJointTrajectoryGoal_<std::allocator<void> >, "
            //                                      "control_msgs::FollowJointTrajectoryFeedback_<std::allocator<void> > >")
            // {
            //     joint_trajectory_interface_is_running = false;
            // }
        }
    }

    for(auto& controller_it : start_list)
    {
        for(auto& resource_it : controller_it.claimed_resources)
        {
            if(resource_it.hardware_interface == "hardware_interface::PositionJointInterface")
            {
                joint_position_cmd_interface_is_running = true;
            }
            // if(resource_it.hardware_interface == "hardware_interface::TrajectoryInterface<control_msgs::"
            //                                      "FollowJointTrajectoryGoal_<std::allocator<void> >, "
            //                                      "control_msgs::FollowJointTrajectoryFeedback_<std::allocator<void> > >")
            // {
            //     joint_trajectory_interface_is_running = true;
            // }

        }
    }
}

void ElfinHWInterface::read_init()
{
    robot_status_resource_.mode = RobotMode::UNKNOWN;
    robot_status_resource_.e_stopped = TriState::UNKNOWN;
    robot_status_resource_.drives_powered = TriState::UNKNOWN;
    robot_status_resource_.motion_possible = TriState::UNKNOWN;
    robot_status_resource_.in_motion = TriState::UNKNOWN;
    robot_status_resource_.in_error = TriState::UNKNOWN;
    robot_status_resource_.error_code = 0;

    struct timespec read_update_tick;
    clock_gettime(CLOCK_REALTIME, &read_update_tick);
    read_update_time_.sec=read_update_tick.tv_sec;
    read_update_time_.nsec=read_update_tick.tv_nsec;

    for(size_t i=0; i<module_infos_.size(); i++)
    {
        module_infos_[i].axis1.position= HRClient.ActPosAcs[i]/(180/M_PI);
        // module_infos_[i].axis2.position= HRClient.ActPosAcs[i*2+1]/(180/M_PI);

            module_infos_[i].axis1.position_cmd= HRClient.ActPosAcs[i]/(180/M_PI);
        // module_infos_[i].axis2.position_cmd= HRClient.ActPosAcs[i*2+1]/(180/M_PI);

        module_infos_[i].axis1.last_position = module_infos_[i].axis1.position_cmd;
    // module_infos_[i].axis2.last_position = module_infos_[i].axis2.position_cmd;
    }
    // module_infos_[2].axis1.position *= -1;
    // module_infos_[1].axis1.position -= M_PI/2;
    // module_infos_[3].axis1.position -= M_PI/2;
}

void ElfinHWInterface::read_update(const ros::Time &time_now)
{
    read_update_dur_=time_now - read_update_time_;
    read_update_time_=time_now;

    for(size_t i=0; i<module_infos_.size(); i++)
    {
        module_infos_[i].axis1.position = HRClient.ActPosAcs[i]/(180/M_PI);
        module_infos_[i].axis1.velocity = HRClient.ActVelAcs[i]/(180/M_PI);
        // 电流-实际力矩
        module_infos_[i].axis1.effort = HRClient.ActMotorCur[i];

        // module_infos_[i].axis2.position = HRClient.ActPosAcs[i*2+1]/(180/M_PI);
        // module_infos_[i].axis2.velocity = HRClient.ActVelAcs[i*2+1]/(180/M_PI);
        // // 电流-实际力矩
        // module_infos_[i].axis2.effort = HRClient.ActMotorCur[i*2+1];
    }
    memcpy(act_posPCS, HRClient.ActPosPcs, sizeof(HRClient.ActPosPcs));    
    cur_override = HRClient.CmdOverride;

    enable_state = HRStateClient_.robot_enable_state;
    moving_state = HRStateClient_.robot_moving_state;
    curFSM_state = HRStateClient_.robot_curFSM_state;
    error_code = HRStateClient_.robot_error_code;
    box_do_state = HRStateClient_.robot_box_do_state;
    box_di_state = HRStateClient_.robot_box_di_state;
    box_co_state = HRStateClient_.robot_box_co_state;
    box_ci_state = HRStateClient_.robot_box_ci_state;
    end_do_state = HRStateClient_.robot_end_do_state;
    end_di_state = HRStateClient_.robot_end_di_state;

    if((!start_ServoJ_State && moving_state)||curFSM_state==31)
    {
        for(size_t i=0; i<module_infos_.size(); i++)
        {
            module_infos_[i].axis1.position_cmd= module_infos_[i].axis1.position;
            // module_infos_[i].axis2.position_cmd= module_infos_[i].axis2.position;
        }
    }
    extractToolPose(time_now);
    publishPose();
    publishOverride();

    publishEnable();
    publishCurFSM();
    publishMoving();
    publishErrorCode();
    publishBoxDO();
    publishBoxDI();
    publishBoxCO();
    publishBoxCI();
    publishEndDO();
    publishEndDI();
    publishControlState();
    update_robotState();

    // if(joint_trajectory_interface_is_running)
    // {
    //     control_msgs::FollowJointTrajectoryFeedback feedback = control_msgs::FollowJointTrajectoryFeedback();
    //     for (size_t i = 0; i < 3; i++)
    //     {
    //     feedback.desired.positions.push_back(module_infos_[i].axis1.position_cmd);
    //     feedback.desired.velocities.push_back(module_infos_[i].axis1.velocity_cmd);
    //     feedback.desired.positions.push_back(module_infos_[i].axis2.position_cmd);
    //     feedback.desired.velocities.push_back(module_infos_[i].axis2.velocity_cmd);

    //     feedback.actual.positions.push_back(module_infos_[i].axis1.position);
    //     feedback.actual.velocities.push_back(module_infos_[i].axis1.velocity);
    //     feedback.actual.positions.push_back(module_infos_[i].axis2.position);
    //     feedback.actual.velocities.push_back(module_infos_[i].axis2.velocity);

    //     feedback.error.positions.push_back(std::abs(module_infos_[i].axis1.position - 
    //                                                         module_infos_[i].axis1.position_cmd));
    //     feedback.error.velocities.push_back(std::abs(module_infos_[i].axis1.velocity - 
    //                                                     module_infos_[i].axis1.velocity_cmd));
    //     feedback.error.positions.push_back(std::abs(module_infos_[i].axis2.position - 
    //                                                         module_infos_[i].axis2.position_cmd));
    //     feedback.error.velocities.push_back(std::abs(module_infos_[i].axis2.velocity - 
    //                                                     module_infos_[i].axis2.velocity_cmd));
    //     }
    //     jnt_traj_interface_.setFeedback(feedback);
    // }
}

void ElfinHWInterface::write_update()
{
    if(joint_position_cmd_interface_is_running)
    {
        bool has_move = false;
    for(size_t i=0; i<module_infos_.size(); i++)
    {
        // module_infos_[2].axis1.position_cmd *= -1;
        // module_infos_[1].axis1.position_cmd += M_PI/2;
        // module_infos_[3].axis1.position_cmd += M_PI/2;
        if(std::abs(module_infos_[i].axis1.position_cmd - HRClient.ActPosAcs[i]/(180/M_PI))> 0.001745201 )
                        // std::abs(module_infos_[i].axis2.position_cmd - HRClient.ActPosAcs[i*2+1]/(180/M_PI))> 0.001745201)
            {
                if(module_infos_[i].axis1.position_cmd != module_infos_[i].axis1.last_position)//|| module_infos_[i].axis2.position_cmd!=module_infos_[i].axis2.last_position
                {
                    has_move = true;
                    break;
                }
            }
        }
        if(has_move)
        {
            if(enable_state && (!moving_state) && curFSM_state == 33 &&(!start_ServoJ_State))
            {
                int res = HRCmdClient.StartServo(servoTime,lookaheadTime);
                // sleep(0.02);
                if(res == 0)
                {
                    // ROS_INFO("Start ServoJ...");
                    start_ServoJ_State = true;
                    // sleep(0.02);
                    int push_res_start = HRCmdClient.pushServoJ(module_infos_[0].axis1.position_cmd*(180/M_PI),module_infos_[1].axis1.position_cmd*(180/M_PI),
                                            module_infos_[2].axis1.position_cmd*(180/M_PI),module_infos_[3].axis1.position_cmd*(180/M_PI),
                                            module_infos_[4].axis1.position_cmd*(180/M_PI),module_infos_[5].axis1.position_cmd*(180/M_PI));
                }
            }
            if(enable_state && start_ServoJ_State)
            {
                int push_res = HRCmdClient.pushServoJ(module_infos_[0].axis1.position_cmd*(180/M_PI),module_infos_[1].axis1.position_cmd*(180/M_PI),
                                            module_infos_[2].axis1.position_cmd*(180/M_PI),module_infos_[3].axis1.position_cmd*(180/M_PI),
                                            module_infos_[4].axis1.position_cmd*(180/M_PI),module_infos_[5].axis1.position_cmd*(180/M_PI));
                // ROS_INFO("Push ServoJ Joint point:  %f, %f, %f, %f, %f, %f",module_infos_[0].axis1.position_cmd,
                //             module_infos_[0].axis2.position_cmd,module_infos_[1].axis1.position_cmd,module_infos_[1].axis2.position_cmd,
                //             module_infos_[2].axis1.position_cmd,module_infos_[2].axis2.position_cmd);
                if (push_res != 0)
                {
                    start_ServoJ_State = false;
            }else{
                for(size_t i=0; i<module_infos_.size(); i++)
                    {
                        module_infos_[i].axis1.last_position = module_infos_[i].axis1.position_cmd;
                        // module_infos_[i].axis2.last_position = module_infos_[i].axis2.position_cmd;
                    }
                }
            }
            }else if(start_ServoJ_State && !moving_state){
                stop_send_time+=1;
                if(stop_send_time>=2)
                {
                    start_ServoJ_State = false;
                    stop_pushServo = false;
                    stop_send_time = 0;
                    // ROS_INFO("Close ServoJ...");
                }
        }
    }

    // if(joint_trajectory_interface_is_running)
    // {
        // 轨迹转发，不通过ROS插补
        // 使用movepath或waypoint
    // }
}

} // end namespace elfin_ros_control

typedef struct{
    controller_manager::ControllerManager *manager;
    elfin_ros_control::ElfinHWInterface *elfin_hw_interface;
}ArgsForThread;

static void timespecInc(struct timespec &tick, int nsec)
{
  int SEC_2_NSEC = 1e+9;
  tick.tv_nsec += nsec;
  while (tick.tv_nsec >= SEC_2_NSEC)
  {
    tick.tv_nsec -= SEC_2_NSEC;
    ++tick.tv_sec;
  }
}

void* update_loop(void* threadarg)
{
    ArgsForThread *arg=(ArgsForThread *)threadarg;
    controller_manager::ControllerManager *manager=arg->manager;
    elfin_ros_control::ElfinHWInterface *interface=arg->elfin_hw_interface;
    ros::Duration d(0.004);
    struct timespec tick;
    clock_gettime(CLOCK_REALTIME, &tick);
    //time for checking overrun
    struct timespec before;
    double overrun_time;
    while(ros::ok())
    {
        ros::Time this_moment(tick.tv_sec, tick.tv_nsec);
        interface->read_update(this_moment);
        manager->update(this_moment, d);
        interface->write_update();
        timespecInc(tick, d.nsec);
        // check overrun
        clock_gettime(CLOCK_REALTIME, &before);
        overrun_time = (before.tv_sec + double(before.tv_nsec)/1e+9) -  (tick.tv_sec + double(tick.tv_nsec)/1e+9);
        if(overrun_time > 0.0)
        {
            tick.tv_sec=before.tv_sec;
            tick.tv_nsec=before.tv_nsec;
        }
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &tick, NULL);
    }
    // sleep(0.005);
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"elfin_hardware_interface", ros::init_options::AnonymousName);
    ros::AsyncSpinner spinner(3);
    spinner.start();
    ros::NodeHandle root_n;
    ros::NodeHandle nh("~");
    
    elfin_ros_control::ElfinHWInterface elfin_hw;

    if(!elfin_hw.init(nh,root_n))
    {
        ROS_ERROR("Could not correctly initialize robot. Exiting elfin ros controller");
        exit(1);
    }
    elfin_hw.read_init();
    controller_manager::ControllerManager cm(&elfin_hw);
    pthread_t tid;
    ArgsForThread *thread_arg=new ArgsForThread();
    thread_arg->manager=&cm;
    thread_arg->elfin_hw_interface=&elfin_hw;
    pthread_create(&tid, NULL, update_loop, thread_arg);

    ros::Rate r(10);
    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }
}
