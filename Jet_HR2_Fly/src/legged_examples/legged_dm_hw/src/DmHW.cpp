#include "legged_dm_hw/DmHW.h"
#include "std_msgs/Float64MultiArray.h"
#include <ostream>
#include <vector>
#include <iostream>
#include <cmath>
#include <fstream>
#include <cmath>
#include "std_msgs/Float32.h"
#include <dmbot_serial/robot_connect.h>
//#include <dmbot_serial/robot3.h>

namespace legged
{
  
void writedata2file(float pos,float vel,float tau,std::string path)
{
  std::ofstream file(path, std::ios::app);
  if (!file.is_open()) {
        std::cerr << "无法打开文件用于写入\n";
        return;
    }
  file << pos << " "<<vel<<" "<<tau<<"\n";
  file.close();
}
bool DmHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
{

  root_nh.setParam("gsmp_controller_switch", "null");

  odom_sub_ = root_nh.subscribe("/imu/data", 1, &DmHW::OdomCallBack, this);
  // void OdomCallBack(const sensor_msgs::Imu::ConstPtr &odom)
   // {
  //    yesenceIMU_ = *odom;
  //  }

  if (!LeggedHW::init(root_nh, robot_hw_nh))
    return false;

  robot_hw_nh.getParam("power_limit", powerLimit_);
   
  setupJoints();
  setupImu();
  cmd_pos_pub_ = robot_hw_nh.advertise<std_msgs::Float64MultiArray>("cmd_pos", 10);
  cmd_vel_pub_ = robot_hw_nh.advertise<std_msgs::Float64MultiArray>("cmd_vel", 10);
  cmd_ff_pub_ = robot_hw_nh.advertise<std_msgs::Float64MultiArray>("cmd_ff", 10);

  read_pos_pub_ = robot_hw_nh.advertise<std_msgs::Float64MultiArray>("read_pos", 10);
  read_vel_pub_ = robot_hw_nh.advertise<std_msgs::Float64MultiArray>("read_vel", 10);
  read_ff_pub_ = robot_hw_nh.advertise<std_msgs::Float64MultiArray>("read_ff", 10);
  
  start = true;

  return true;
}


void DmHW::read(const ros::Time &time, const ros::Duration &period)
{
  double pos,vel,tau;
  for (int i=0; i<10; i++)
  {
    motorsInterface->get_motor_data(pos,vel,tau, i);
    jointData_[i].pos_ = pos * directionMotor_[i];
    jointData_[i].vel_ = vel * directionMotor_[i];
    jointData_[i].tau_ = tau * directionMotor_[i];
  }//电机到joint的映射
 
  imuData_.ori[0] = yesenceIMU_.orientation.x;       
  imuData_.ori[1] = yesenceIMU_.orientation.y; 
  imuData_.ori[2] = yesenceIMU_.orientation.z; 
  imuData_.ori[3] = yesenceIMU_.orientation.w; 
  imuData_.angular_vel[0] = yesenceIMU_.angular_velocity.x;  
  imuData_.angular_vel[1] = yesenceIMU_.angular_velocity.y;
  imuData_.angular_vel[2] = yesenceIMU_.angular_velocity.z;
  imuData_.linear_acc[0] = yesenceIMU_.linear_acceleration.x;   
  imuData_.linear_acc[1] = yesenceIMU_.linear_acceleration.y;
  imuData_.linear_acc[2] = yesenceIMU_.linear_acceleration.z;

}
void DmHW::write(const ros::Time& time, const ros::Duration& period)
{
  for (int i = 0; i < 10; ++i)//as the urdf rank
  {
    dmSendcmd_[i].kp_ = jointData_[i].kp_;
    dmSendcmd_[i].kd_ = jointData_[i].kd_;
    //从joint到电机的方向映射
    dmSendcmd_[i].pos_des_ = jointData_[i].pos_des_ * directionMotor_[i];
    dmSendcmd_[i].vel_des_ = jointData_[i].vel_des_ * directionMotor_[i];
    
    dmSendcmd_[i].ff_ = jointData_[i].ff_* directionMotor_[i];  
  }

  for (int i = 0; i < 10; ++i)
  {
    motorsInterface->fresh_cmd_motor_data(dmSendcmd_[i].pos_des_, 
                                              dmSendcmd_[i].vel_des_,
                                              dmSendcmd_[i].ff_, 
                                              dmSendcmd_[i].kp_, dmSendcmd_[i].kd_, i);
  }
  motorsInterface->send_motor_data();
}


bool DmHW::setupJoints()
{
  for (const auto& joint : urdfModel_->joints_)
  {
    int leg_index, joint_index;
    if (joint.first.find("leg_l") != std::string::npos)
    {
      leg_index = 0;
    }
    else if (joint.first.find("leg_r") != std::string::npos)
    {
      leg_index = 1;
    }
    else
      continue;
    if (joint.first.find("1_joint") != std::string::npos)
      joint_index = 0;
    else if (joint.first.find("2_joint") != std::string::npos)
      joint_index = 1;
    else if (joint.first.find("3_joint") != std::string::npos)
      joint_index = 2;
    else if (joint.first.find("4_joint") != std::string::npos)
      joint_index = 3;
    else if (joint.first.find("5_joint") != std::string::npos)
      joint_index = 4;
    else
      continue;

    int index = leg_index * 5 + joint_index;
    hardware_interface::JointStateHandle state_handle(joint.first, &jointData_[index].pos_, &jointData_[index].vel_,
                                                      &jointData_[index].tau_);
    jointStateInterface_.registerHandle(state_handle);
    hybridJointInterface_.registerHandle(HybridJointHandle(state_handle, &jointData_[index].pos_des_,
                                                           &jointData_[index].vel_des_, &jointData_[index].kp_,
                                                           &jointData_[index].kd_, &jointData_[index].ff_));
  }
  return true;
}

bool DmHW::setupImu()
{
  imuSensorInterface_.registerHandle(hardware_interface::ImuSensorHandle(
      "imu_link", "imu_link", imuData_.ori, imuData_.ori_cov, imuData_.angular_vel, imuData_.angular_vel_cov,
      imuData_.linear_acc, imuData_.linear_acc_cov));
  imuData_.ori_cov[0] = 0.0012;
  imuData_.ori_cov[4] = 0.0012;
  imuData_.ori_cov[8] = 0.0012;

  imuData_.angular_vel_cov[0] = 0.0004;
  imuData_.angular_vel_cov[4] = 0.0004;
  imuData_.angular_vel_cov[8] = 0.0004;

  return true;
}

}  // namespace legged
