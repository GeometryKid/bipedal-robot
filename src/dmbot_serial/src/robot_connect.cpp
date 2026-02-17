#include <dmbot_serial/robot_connect.h>


namespace dmbot_serial
{
robot::robot()
{

  n.param("port", motor_serial_port, std::string("/dev/ttyACM0")); 
  n.param("baud", motor_seial_baud, 921600);
  
  int i=0;
  for(auto& motor:motors)
  {
      motor.name = "Motor_" + std::to_string(i);   
      motor.pos = 0.0f;
      motor.vel = 0.0f;
      motor.tor = 0.0f;
      motor.tor_set = 0.0f;
      motor.pos_set = 0.0f;
      motor.vel_set = 0.0f;
      motor.kp = 0.0f;
      motor.kd = 0.0f;
      motor.index=i;
      i++;
  }
  motors[0].type = "4340";
  motors[1].type = "4340";
  motors[2].type = "4340";
  motors[3].type = "4340";                
  motors[4].type = "4340";

  motors[5].type = "4340";   
  motors[6].type = "4340";
  motors[7].type = "4340";
  motors[8].type = "4340";
  motors[9].type = "4340";                

  init_motor_serial();//初始化串口

  rec_thread = std::thread(&robot::get_motor_data_thread, this);
  
  joint_state_pub = n.advertise<sensor_msgs::JointState>("joint_states", 10);   
 // pub_thread = std::thread(&robot::publishJointStates, this);

  ros::Duration(2.0).sleep();

  ROS_INFO("robot init complete");

}
robot::~robot()
{   
   for(int i=0;i<10;i++)
  {
    fresh_cmd_motor_data(0.0, 0.0, 0.0, 0.0, 0.0, i); //更新发给电机的参数、力矩等
  }
  
  send_motor_data();

  stop_thread_ = true;

  if(rec_thread.joinable())
  {
    rec_thread.join(); 
  }
  if (serial_motor.isOpen())
  {
    serial_motor.close(); 
  }
  
 // if(pub_thread.joinable())
  //{
 //     pub_thread.join(); 
  //}

}

void robot::init_motor_serial()
{         
    try
    {
      serial_motor.setPort(motor_serial_port);
      serial_motor.setBaudrate(motor_seial_baud);
      serial_motor.setFlowcontrol(serial::flowcontrol_none);
      serial_motor.setParity(serial::parity_none); //default is parity_none
      serial_motor.setStopbits(serial::stopbits_one);
      serial_motor.setBytesize(serial::eightbits);
      serial::Timeout time_out = serial::Timeout::simpleTimeout(20);
      serial_motor.setTimeout(time_out);
      serial_motor.open();
    } 
    catch (serial::IOException &e)  // 抓取异常
    {
        ROS_ERROR_STREAM("In single initialization,Unable to open motor serial port ");
        exit(0);
    }
    if (serial_motor.isOpen())
    {
        ROS_INFO_STREAM("In single initialization,Motor Serial Port initialized");
    }
    else
    {
        ROS_ERROR_STREAM("In single initialization,Unable to open motor serial port ");
        exit(0);
    }
   
}

void robot::get_motor_data_thread()
{   
//ros::Rate ra(100000); 
  while (ros::ok()&&!stop_thread_)
  {    
    if (!serial_motor.isOpen())
    {
      ROS_WARN("In get_motor_data_thread,motor serial port unopen");
    }       

  short transition_16=0;  //中间变量
  uint8_t i=0,check=0, error=1,Receive_Data_Pr[1];  //临时变量，保存下位机数据
  static int count; //静态变量，用于计数
  serial_motor.read(Receive_Data_Pr,sizeof(Receive_Data_Pr)); //通过串口读取下位机发送过来的数据
  

  Receive_Data.rx[count] = Receive_Data_Pr[0];  //串口数据填入数组
  Receive_Data.Frame_Header = Receive_Data.rx[0]; 

  if(Receive_Data_Pr[0] == FRAME_HEADER || count>0) //确保数组第一个数据为FRAME_HEADER
      count++;
  else 
    count=0;
  if(count == RECEIVE_DATA_SIZE) ////验证数据包的长度
  {  
      count=0;  //为串口数据重新填入数组做准备
      check=Check_Sum(RECEIVE_DATA_SIZE-1,READ_DATA_CHECK);  //BCC check passes or two packets are interlaced //BCC校验通过或者两组数据包交错

      if(check == Receive_Data.rx[RECEIVE_DATA_SIZE-1])  
      {
        error=0;  //XOR bit check successful //异或位校验成功
      }
      if(error == 0)
      {

        for(int i=0;i<10;i++)
        {
          dm4340_fbdata(motors[i],&Receive_Data.rx[1+i*5]);
      
        }     
      }
  }      
  }
}


void robot::publishJointStates()
{
  ros::Rate rate(1000); 
  while (ros::ok())
  {
      sensor_msgs::JointState joint_state_msg;

      for(int i=0; i<10; i++)
      {    
          joint_state_msg.name.push_back(motors[i].name);                   
         
          joint_state_msg.position.push_back(motors[i].pos);
          joint_state_msg.velocity.push_back(motors[i].vel);
          joint_state_msg.effort.push_back(motors[i].tor); 
      }

    joint_state_pub.publish(joint_state_msg);

    rate.sleep();
  }
}

void robot::send_motor_data()
{   
  ros::Time last_time2 = ros::Time::now();
  for(auto& motor:motors)
  {  
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
    if(motor.type == "8006")
    {   
        pos_tmp = float_to_uint(motor.pos_set,  P_MIN4,  P_MAX4,  16);
        vel_tmp = float_to_uint(motor.vel_set,  V_MIN4,  V_MAX4,  12);
        kp_tmp  = float_to_uint(motor.kp,   KP_MIN4, KP_MAX4, 12);
        kd_tmp  = float_to_uint(motor.kd,   KD_MIN4, KD_MAX4, 12);
        tor_tmp = float_to_uint(motor.tor_set, T_MIN4,  T_MAX4,  12);
    }
    else if(motor.type == "6006")
    {   
        pos_tmp = float_to_uint(motor.pos_set,  P_MIN3,  P_MAX3,  16);
        vel_tmp = float_to_uint(motor.vel_set,  V_MIN3,  V_MAX3,  12);
        kp_tmp  = float_to_uint(motor.kp,   KP_MIN3, KP_MAX3, 12);
        kd_tmp  = float_to_uint(motor.kd,   KD_MIN3, KD_MAX3, 12);
        tor_tmp = float_to_uint(motor.tor_set, T_MIN3,  T_MAX3,  12);
    }
    else  if(motor.type == "4340")
    {
        pos_tmp = float_to_uint(motor.pos_set,  P_MIN2,  P_MAX2,  16);
        vel_tmp = float_to_uint(motor.vel_set,  V_MIN2,  V_MAX2,  12);
        kp_tmp  = float_to_uint(motor.kp,   KP_MIN2, KP_MAX2, 12);
        kd_tmp  = float_to_uint(motor.kd,   KD_MIN2, KD_MAX2, 12);
        tor_tmp = float_to_uint(motor.tor_set, T_MIN2,  T_MAX2,  12);
    }
    else  if(motor.type == "4310")
    {   
        pos_tmp = float_to_uint(motor.pos_set,  P_MIN1,  P_MAX1,  16);
        vel_tmp = float_to_uint(motor.vel_set,  V_MIN1,  V_MAX1,  12);
        kp_tmp  = float_to_uint(motor.kp,   KP_MIN1, KP_MAX1, 12);
        kd_tmp  = float_to_uint(motor.kd,   KD_MIN1, KD_MAX1, 12);
        tor_tmp = float_to_uint(motor.tor_set, T_MIN1,  T_MAX1,  12);
    }
    Send_Data.tx[0]=FRAME_HEADER; 
    Send_Data.tx[1]=motor.index;

    Send_Data.tx[2] = (pos_tmp >> 8);
    Send_Data.tx[3] = pos_tmp;
    Send_Data.tx[4] = (vel_tmp >> 4);
    Send_Data.tx[5] = ((vel_tmp&0x0F)<<4)|(kp_tmp>>8);
    Send_Data.tx[6] = kp_tmp;
    Send_Data.tx[7] = (kd_tmp >> 4);
    Send_Data.tx[8] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
    Send_Data.tx[9] = tor_tmp;

    Send_Data.tx[10]=Check_Sum(10,SEND_DATA_CHECK);   
     
    try
    { //通过串口向下位机发送数据 
      serial_motor.write(Send_Data.tx,sizeof(Send_Data.tx));
    //ROS_INFO("Current time Motor: %f", interval.toSec());
    }
    catch (serial::IOException& e)   
    {
      ROS_ERROR_STREAM("In send_motor_data,Unable to send data through motor serial port"); //If sending data fails, an error message is printed //如果发送数据失败，打印错误信息
    }

  }  
        
}


void robot::fresh_cmd_motor_data(double pos, double vel,double torque, double kp,double kd,int motor_idx)
{//更新发给电机的参数、力矩等
  motors[motor_idx].pos_set = pos;
  motors[motor_idx].vel_set = vel;
  motors[motor_idx].tor_set = torque;
  motors[motor_idx].kp = kp;
  motors[motor_idx].kd = kd;                
}

void robot::get_motor_data(double &pos,double &vel,double &torque, int motor_idx)
{//获取电机反馈的参数、力矩等
  pos = motors[motor_idx].pos;
  vel = motors[motor_idx].vel;
  torque =motors[motor_idx].tor;
}

/**************************************
功能: 串口通讯校验函数，数据包n有个字节，第n-1个字节为校验位，第n个字节位帧尾。第1个字节到第n-2个字节数据按位异或的结果与第n-1个字节对比，即为BCC校验
输入参数： Count_Number：数据包前几个字节加入校验   mode：对发送数据还是接收数据进行校验
***************************************/
unsigned char robot::Check_Sum(unsigned char Count_Number,unsigned char mode)
{
    unsigned char check_sum=0,k;

    if(mode==0) //Receive data mode //接收数据模式
    {
        for(k=0;k<Count_Number;k++)
        {
            check_sum=check_sum^Receive_Data.rx[k]; //By bit or by bit //按位异或
        }
    }

    if(mode==1) //Send data mode //发送数据模式
    {
        for(k=0;k<Count_Number;k++)
        {
            check_sum=check_sum^Send_Data.tx[k]; //By bit or by bit //按位异或
        }
    }
  return check_sum; //Returns the bitwise XOR result //返回按位异或结果
}

void robot::dm4310_fbdata(motor_data_t& moto,uint8_t *data)
{
  moto.p_int=(data[0]<<8)|data[1];
  moto.v_int=(data[2]<<4)|(data[3]>>4);
  moto.t_int=((data[3]&0x0F)<<8)|data[4];
  moto.pos = uint_to_float(moto.p_int, P_MIN1, P_MAX1, 16); // (-12.5,12.5)
  moto.vel = uint_to_float(moto.v_int, V_MIN1, V_MAX1, 12); // (-30.0,30.0)
  moto.tor = uint_to_float(moto.t_int, T_MIN1, T_MAX1, 12);  // (-10.0,10.0)
}

void robot::dm4340_fbdata(motor_data_t& moto,uint8_t *data)
{
  moto.p_int=(data[0]<<8)|data[1];
  moto.v_int=(data[2]<<4)|(data[3]>>4);
  moto.t_int=((data[3]&0x0F)<<8)|data[4];
  moto.pos = uint_to_float(moto.p_int, P_MIN2, P_MAX2, 16); // (-12.5,12.5)
  moto.vel = uint_to_float(moto.v_int, V_MIN2, V_MAX2, 12); // (-30.0,30.0)
  moto.tor = uint_to_float(moto.t_int, T_MIN2, T_MAX2, 12);  // (-10.0,10.0)
}

void robot::dm6006_fbdata(motor_data_t& moto,uint8_t *data)
{
  moto.p_int=(data[0]<<8)|data[1];
  moto.v_int=(data[2]<<4)|(data[3]>>4);
  moto.t_int=((data[3]&0x0F)<<8)|data[4];
  moto.pos = uint_to_float(moto.p_int, P_MIN3, P_MAX3, 16); // (-12.5,12.5)
  moto.vel = uint_to_float(moto.v_int, V_MIN3, V_MAX3, 12); // (-30.0,30.0)
  moto.tor = uint_to_float(moto.t_int, T_MIN3, T_MAX3, 12);  // (-10.0,10.0)
}

void robot::dm8006_fbdata(motor_data_t& moto,uint8_t *data)
{
  moto.p_int=(data[0]<<8)|data[1];
  moto.v_int=(data[2]<<4)|(data[3]>>4);
  moto.t_int=((data[3]&0x0F)<<8)|data[4];
  moto.pos = uint_to_float(moto.p_int, P_MIN4, P_MAX4, 16); // (-12.5,12.5)
  moto.vel = uint_to_float(moto.v_int, V_MIN4, V_MAX4, 12); // (-30.0,30.0)
  moto.tor = uint_to_float(moto.t_int, T_MIN4, T_MAX4, 12);  // (-10.0,10.0)
}

int16_t robot::float_to_uint(float x_float, float x_min, float x_max, int bits)
{
  /* Converts a float to an unsigned int, given range and number of bits */
  float span = x_max - x_min;
  float offset = x_min;
  return (int16_t)((x_float-offset)*((float)((1<<bits)-1))/span);
}

float robot::uint_to_float(int x_int, float x_min, float x_max, int bits)
{
  /* converts unsigned int to float, given range and number of bits */
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}


}


