#include "ros/ros.h"

#include <dmbot_serial/robot_connect.h>
#include <iostream>
#include <thread>
#include <condition_variable>

#include <dynamic_reconfigure/server.h>
#include <dmbot_serial/my_cfgConfig.h>   //my_cfg就是参数配置文件

// 定义一个用于表示斜坡发生器状态的结构体
typedef struct RampGenerator
{
    double currentValue; // 当前斜坡值
    double targetValue;  // 目标值
    double step;         // 每个控制周期应当改变的数值大小

}RampGenerator;

// 一个周期内对斜坡发生器状态的更新
void rampIterate(RampGenerator *ramp)
{
   // printf("Current Value Updated: %f\n", ramp->currentValue); // 添加此行代码以在每次迭代后打印当前值
  ramp->currentValue += ramp->step; // 增大当前值
  if (ramp->currentValue > ramp->targetValue&& ramp->step>0)
  { //递增方向，避免大过目标值
    ramp->currentValue = ramp->targetValue;
  }
  else if (ramp->currentValue < ramp->targetValue&& ramp->step<0)
  { //递减方向，避免小过目标值
    ramp->currentValue = ramp->targetValue;
  }  
}

// 初始化斜坡发生器
void rampInit(RampGenerator *ramp, double startValue, double targetValue, double time)
{
    ramp->currentValue = startValue;
    ramp->targetValue = targetValue;
    // 计算步进值，这里需要注意的是，确保斜坡时间和周期时间都不为零来避免除以零的错误
    ramp->step = (targetValue - startValue) / time;
   // std::cout<<"rampInit: startValue:"<<ramp->currentValue<<" targetValue:"<<ramp->targetValue<<" step:"<<ramp->step<<std::endl;
}

RampGenerator my_ramp[10];

double my_pos[10]={0.0};
double my_vel[10]={0.0};
double my_torque[10]={0.0};
double my_kp=0.0;
double my_kd=0.8;
void callback(dmbot_serial::my_cfgConfig &config, uint32_t level) {
    ROS_INFO("Reconfigure Request: ");
    //位置
    my_pos[0]=config.pos_set0;
    my_pos[1]=config.pos_set1;
    my_pos[2]=config.pos_set2;
    my_pos[3]=config.pos_set3;
    my_pos[4]=config.pos_set4;
    my_pos[5]=config.pos_set5;
    my_pos[6]=config.pos_set6;
    my_pos[7]=config.pos_set7;
    my_pos[8]=config.pos_set8;
    my_pos[9]=config.pos_set9;
    //速度
    my_vel[0]=config.vel_set0;
    my_vel[1]=config.vel_set1;
    my_vel[2]=config.vel_set2;
    my_vel[3]=config.vel_set3;
    my_vel[4]=config.vel_set4;
    my_vel[5]=config.vel_set5;
    my_vel[6]=config.vel_set6;
    my_vel[7]=config.vel_set7;
    my_vel[8]=config.vel_set8;
    my_vel[9]=config.vel_set9;
    //力矩
    my_torque[0]=config.torque_set0;
    my_torque[1]=config.torque_set1;
    my_torque[2]=config.torque_set2;
    my_torque[3]=config.torque_set3;
    my_torque[4]=config.torque_set4;
    my_torque[5]=config.torque_set5;
    my_torque[6]=config.torque_set6;
    my_torque[7]=config.torque_set7;
    my_torque[8]=config.torque_set8;
    my_torque[9]=config.torque_set9;
  

    my_kp=config.kp_set;
    my_kd=config.kd_set;
    // ROS_INFO("torque_set0: %8f,torque_set1: %8f,torque_set5: %8f,vel_set: %8f,kd_set: %8f",my_torque[0],my_torque[1],my_torque[5],my_vel[0],my_kd);
    // ROS_INFO("kp_set: %8f,kd_set: %8f",my_kp,my_kd);
}

double get_pos[10]={0.0};
double get_vel[10]={0.0};
double get_torque[10]={0.0};
int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_motor");
    ros::NodeHandle n;
    ros::Rate r(1000);
    dmbot_serial::robot rb;

    // ========================== singlethread send =====================
    dynamic_reconfigure::Server<dmbot_serial::my_cfgConfig> server;
    dynamic_reconfigure::Server<dmbot_serial::my_cfgConfig>::CallbackType ff;

    ff = boost::bind(&callback, _1, _2);
    server.setCallback(ff);
    
    my_pos[0]=0.0;//左腿
    my_pos[3]=0.0;
    my_pos[4]=0.0;

    my_pos[5]=0.0;//右腿
    my_pos[8]=0.0;
    my_pos[9]=0.0;
    rampInit(&my_ramp[0], 0.0,my_pos[0], 1000.0);
    rampInit(&my_ramp[3], 0.0,my_pos[3], 1000.0);
    rampInit(&my_ramp[4], 0.0,my_pos[4], 1000.0);

    rampInit(&my_ramp[5], 0.0,my_pos[5], 1000.0);
    rampInit(&my_ramp[8], 0.0,my_pos[8], 1000.0);
    rampInit(&my_ramp[9], 0.0,my_pos[9], 1000.0);

    int time=0;
    int flag=0;
    r.sleep();
    ros::spinOnce();
    while (ros::ok()) // 此用法为逐个电机发送控制指令
    {
      time++;
      if(time>2000&&flag==0)
      {
        time=0;
        flag=1;
        my_pos[0]=0.0;
        my_pos[3]=0.0;
        // my_pos[4]=0.0;

        // my_pos[5]=0.0;
        my_pos[8]=0.0;
        my_pos[9]=0.0;
        rampInit(&my_ramp[0], get_pos[0],my_pos[0], 1000.0);
        rampInit(&my_ramp[3], get_pos[3],my_pos[3], 1000.0);
        rampInit(&my_ramp[4], get_pos[4],my_pos[4], 1000.0);
        rampInit(&my_ramp[5], get_pos[5],my_pos[5], 1000.0);
        rampInit(&my_ramp[8], get_pos[8],my_pos[8], 1000.0);
        rampInit(&my_ramp[9], get_pos[9],my_pos[9], 1000.0);
      }
      else if(time>2000&&flag==1)
      {
        time=0;
        flag=0;
        my_pos[0]=0.0;
        my_pos[3]=0.0;
        my_pos[4]=0.0;

        my_pos[5]=0.0;
        my_pos[8]=0.0;
        my_pos[9]=0.0;
        rampInit(&my_ramp[0], get_pos[0],my_pos[0], 1000.0);
        rampInit(&my_ramp[3], get_pos[3],my_pos[3], 1000.0);
        rampInit(&my_ramp[4], get_pos[4],my_pos[4], 1000.0);
        rampInit(&my_ramp[5], get_pos[5],my_pos[5], 1000.0);
        rampInit(&my_ramp[8], get_pos[8],my_pos[8], 1000.0);
        rampInit(&my_ramp[9], get_pos[9],my_pos[9], 1000.0);
      }

      rampIterate(&my_ramp[0]);//更新斜坡状态
      rampIterate(&my_ramp[3]);//更新斜坡状态
      rampIterate(&my_ramp[4]);//更新斜坡状态
      rampIterate(&my_ramp[5]);//更新斜坡状态
      rampIterate(&my_ramp[8]);//更新斜坡状态
      rampIterate(&my_ramp[9]);//更新斜坡状态

      
      //rb.fresh_cmd_motor_data(my_ramp[0].currentValue, my_vel[0], my_torque[0], my_kp, my_kd, 0); //更新发给电机的参数、力矩等
      rb.fresh_cmd_motor_data(my_pos[0], my_vel[0], my_torque[0], my_kp, my_kd, 0);
      rb.fresh_cmd_motor_data(my_pos[1], my_vel[1], my_torque[1], my_kp, my_kd, 1);
      rb.fresh_cmd_motor_data(my_pos[2], my_vel[2], my_torque[2], my_kp, my_kd, 2);
      rb.fresh_cmd_motor_data(my_pos[3], my_vel[3], my_torque[3], my_kp, my_kd, 3);
      rb.fresh_cmd_motor_data(my_pos[4], my_vel[4], my_torque[4], my_kp, my_kd, 4);
      //rb.fresh_cmd_motor_data(my_ramp[3].currentValue, my_vel[3], my_torque[3], my_kp, my_kd, 3);
     //rb.fresh_cmd_motor_data(my_ramp[4].currentValue, my_vel[4], my_torque[4], my_kp, my_kd, 4);

     //rb.fresh_cmd_motor_data(my_ramp[5].currentValue, my_vel[5], my_torque[5], my_kp, my_kd, 5);
      rb.fresh_cmd_motor_data(my_pos[5], my_vel[5], my_torque[5], my_kp, my_kd, 5); 
      rb.fresh_cmd_motor_data(my_pos[6], my_vel[6], my_torque[6], my_kp, my_kd, 6); 
      rb.fresh_cmd_motor_data(my_pos[7], my_vel[7], my_torque[7], my_kp, my_kd, 7);
      rb.fresh_cmd_motor_data(my_pos[8], my_vel[8], my_torque[8], my_kp, my_kd, 8); 
      rb.fresh_cmd_motor_data(my_pos[9], my_vel[9], my_torque[9], my_kp, my_kd, 9);
      //rb.fresh_cmd_motor_data(my_ramp[8].currentValue, my_vel[8], my_torque[8], my_kp, my_kd, 8); 
      //rb.fresh_cmd_motor_data(my_ramp[9].currentValue, my_vel[9], my_torque[9], my_kp, my_kd, 9);

      rb.send_motor_data();

      rb.get_motor_data(get_pos[0], get_vel[0], get_torque[0], 0);
      rb.get_motor_data(get_pos[1], get_vel[1], get_torque[1], 1);
      rb.get_motor_data(get_pos[2], get_vel[2], get_torque[2], 2);
      rb.get_motor_data(get_pos[3], get_vel[3], get_torque[3], 3);
      rb.get_motor_data(get_pos[4], get_vel[4], get_torque[4], 4);

      rb.get_motor_data(get_pos[5], get_vel[5], get_torque[5], 5);
      rb.get_motor_data(get_pos[6], get_vel[6], get_torque[6], 6);
      rb.get_motor_data(get_pos[7], get_vel[7], get_torque[7], 7);
      rb.get_motor_data(get_pos[8], get_vel[8], get_torque[8], 8);
      rb.get_motor_data(get_pos[9], get_vel[9], get_torque[9], 9);
      std::cerr<<"**********************************************************************************"<<std::endl;
      // std::cerr<<"motor0: "<<"pos0: "<<my_pos[0]<<",vel0: "<<my_vel[0]<<",torque0: "<<my_torque[0]<<std::endl;
      std::cerr<<"gmotor1: "<<"gpos1: "<<get_pos[5]<<",gvel1: "<<get_vel[4]<<",gtorque1: "<<get_torque[5]<<std::endl;
      std::cerr<<"mmotor1: "<<"mpos1: "<<my_pos[5]<<",mvel1: "<<my_vel[4]<<",mtorque1: "<<my_torque[5]<<std::endl;
      // std::cerr<<"motor2: "<<"pos2: "<<my_pos[2]<<",vel2: "<<my_vel[2]<<",torque2: "<<my_torque[2]<<std::endl;
      // std::cerr<<"motor3: "<<"pos3: "<<my_pos[3]<<",vel3: "<<my_vel[3]<<",torque3: "<<my_torque[3]<<std::endl;
      // std::cerr<<"motor4: "<<"pos4: "<<my_pos[4]<<",vel4: "<<my_vel[4]<<",torque4: "<<my_torque[4]<<std::endl;
      std::cerr<<"**********************************************************************************"<<std::endl;
      r.sleep();
      ros::spinOnce();
    }

    return 0;
}




