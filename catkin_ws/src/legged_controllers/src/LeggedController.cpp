//
// Created by qiayuan on 2022/6/24.
//

/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "legged_controllers/LeggedController.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_sqp/SqpMpc.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>

#include <angles/angles.h>
#include <legged_estimation/FromTopiceEstimate.h>
#include <legged_estimation/LinearKalmanFilter.h>
#include <legged_wbc/WeightedWbc.h>
#include <pluginlib/class_list_macros.hpp>
#include "std_msgs/Float64MultiArray.h"
#include "legged_controllers/utilities.h"
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <chrono>



namespace legged
{
bool LeggedController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh)
{
  // Initialize OCS2
  std::string urdfFile;
  std::string taskFile;
  std::string referenceFile;
  controller_nh.getParam("/urdfFile", urdfFile);
  controller_nh.getParam("/taskFile", taskFile);
  controller_nh.getParam("/referenceFile", referenceFile);
  bool verbose = false;
  loadData::loadCppDataType(taskFile, "legged_robot_interface.verbose", verbose);

  setupLeggedInterface(taskFile, urdfFile, referenceFile, verbose);
  setupMpc();
  setupMrt();
  // Visualization
  ros::NodeHandle nh;
  CentroidalModelPinocchioMapping pinocchioMapping(leggedInterface_->getCentroidalModelInfo());
  eeKinematicsPtr_ = std::make_shared<PinocchioEndEffectorKinematics>(
      leggedInterface_->getPinocchioInterface(), pinocchioMapping, leggedInterface_->modelSettings().contactNames3DoF);
  robotVisualizer_ = std::make_shared<LeggedRobotVisualizer>(
      leggedInterface_->getPinocchioInterface(), leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_, nh);
  selfCollisionVisualization_.reset(new LeggedSelfCollisionVisualization(
      leggedInterface_->getPinocchioInterface(), leggedInterface_->getGeometryInterface(), pinocchioMapping, nh));

  //////////////////////////////////////////////////////////////////////////////////

  pos_pub_ = controller_nh.advertise<std_msgs::Float64MultiArray>("joint_position", 10);
  vel_pub_ = controller_nh.advertise<std_msgs::Float64MultiArray>("joint_velocity", 10);
  torque_pub_ = controller_nh.advertise<std_msgs::Float64MultiArray>("joint_torque", 10);

  body_pose_pub_ = controller_nh.advertise<geometry_msgs::PoseStamped>("pose_stamped", 10);
  imu_pub_ = controller_nh.advertise<std_msgs::Float64MultiArray>("imu", 10);

  cmd_pos_pub_ = controller_nh.advertise<std_msgs::Float64MultiArray>("cmd_joint_position", 10);
  cmd_vel_pub_ = controller_nh.advertise<std_msgs::Float64MultiArray>("cmd_joint_velocity", 10);
  cmd_tau_pub_ = controller_nh.advertise<std_msgs::Float64MultiArray>("cmd_joint_torque", 10);

  mpc_force_pub_ = controller_nh.advertise<std_msgs::Float64MultiArray>("mpc_force", 12);
  wbc_force_pub_ = controller_nh.advertise<std_msgs::Float64MultiArray>("wbc_force", 12);
  cmd_actau_pub_ = controller_nh.advertise<std_msgs::Float64MultiArray>("cmd_joint_actorque", 10);
  //////////////////////////////////////////////////////////////////////////////////


  rbdConversions_ = std::make_shared<CentroidalModelRbdConversions>(leggedInterface_->getPinocchioInterface(),
                                                                    leggedInterface_->getCentroidalModelInfo());

  // Hardware interface
  auto* hybridJointInterface = robot_hw->get<HybridJointInterface>();

  std::vector<std::string> joint_names{
    "leg_l1_joint", "leg_l2_joint", "leg_l3_joint", "leg_l4_joint", "leg_l5_joint",
    "leg_r1_joint", "leg_r2_joint", "leg_r3_joint", "leg_r4_joint", "leg_r5_joint"
  };
  for (const auto& joint_name : joint_names)
  {
    hybridJointHandles_.push_back(hybridJointInterface->getHandle(joint_name));
  }
  imuSensorHandle_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle("imu_link");

  // State estimation
  setupStateEstimate(taskFile, verbose);

  // Whole body control
  wbc_ = std::make_shared<WeightedWbc>(leggedInterface_->getPinocchioInterface(),
                                       leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_, controller_nh);
  wbc_->loadTasksSetting(taskFile, verbose);
  wbc_->setStanceMode(true);

  // Safety Checker
  safetyChecker_ = std::make_shared<SafetyChecker>(leggedInterface_->getCentroidalModelInfo());

  // Configuring the hardware interface
  eeKinematicsPtr_->setPinocchioInterface(leggedInterface_->getPinocchioInterface());

  // Reading relevant parameters
  RetrievingParameters();

  // loadEigenMatrix
  loadData::loadEigenMatrix(referenceFile, "defaultJointState", defalutJointPos_);

  // Configuring an inverse kinematics processing object
  inverseKinematics_.setParam(std::make_shared<PinocchioInterface>(leggedInterface_->getPinocchioInterface()),
                              std::make_shared<CentroidalModelInfo>(leggedInterface_->getCentroidalModelInfo()));

  info_ = leggedInterface_->getCentroidalModelInfo();

  return true;
}

void LeggedController::starting(const ros::Time& time)
{
  startingTime_.fromSec(time.toSec() - 0.0001);
  const ros::Time shifted_time = time - startingTime_;
  // Initial state
  currentObservation_.state.setZero(stateDim_);
  currentObservation_.input.setZero(inputDim_);
  currentObservation_.state.segment(6 + 6, jointDim_) = defalutJointPos_;
  currentObservation_.mode = ModeNumber::STANCE;

  TargetTrajectories target_trajectories({ currentObservation_.time }, { currentObservation_.state },
                                         { currentObservation_.input });

  mpcMrtInterface_->getReferenceManager().setTargetTrajectories(target_trajectories);

  mpcRunning_ = false;

  // Mode Subscribe
  ModeSubscribe();

  // Dynamic server
  serverPtr_ =
      std::make_unique<dynamic_reconfigure::Server<legged_controllers::TutorialsConfig>>(ros::NodeHandle("controller"));
  dynamic_reconfigure::Server<legged_controllers::TutorialsConfig>::CallbackType f;
  f = boost::bind(&LeggedController::dynamicParamCallback, this, _1, _2);
  serverPtr_->setCallback(f);
}

int number=20;
void LeggedController::update(const ros::Time& time, const ros::Duration& period)
{ 
  const ros::Time shifted_time = time - startingTime_;
  // State Estimate
  updateStateEstimation(shifted_time, period);
  std::vector<matrix_t> Jfoot = stateEstimate_->getJacobifoot();
  // Update the current state of the system
  // mpcMrtInterface_ = std::make_shared<MPC_MRT_Interface>(*mpc_);
  mpcMrtInterface_->setCurrentObservation(currentObservation_);

  // Evaluate the current policy
  vector_t optimizedState(stateDim_), optimizedInput(inputDim_);
  size_t plannedMode = 0;
  bool mpc_updated_ = false;

  if (firstStartMpc_)
  {//当mpc开启了会一直firstStartMpc_ = true
  //std::cout << "First Start MPC" << std::endl;
    // Load the latest MPC policy
    mpcMrtInterface_->updatePolicy();
    //获取mpc已经计算完的计算结果
    mpcMrtInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state, optimizedState,
                                     optimizedInput, plannedMode);
    currentObservation_.input = optimizedInput;
    mpc_updated_ = true;
  }

    const vector_t &mpc_p = optimizedState.segment(6 + 6, jointDim_);
    const vector_t &mpc_v = optimizedInput.segment(12, jointDim_);
    const vector_t &mpc_f = optimizedInput.segment(0, 12);

    const vector_t &mpc_planned_joint_force0 = optimizedInput.segment(0, 3);
    const vector_t &mpc_planned_joint_force1 = optimizedInput.segment(3, 3);
    const vector_t &mpc_planned_joint_force2 = optimizedInput.segment(6, 3);
    const vector_t &mpc_planned_joint_force3 = optimizedInput.segment(9, 3);

    vector_t tor0 = Jfoot[0].transpose() * mpc_planned_joint_force0;
    vector_t tor1 = Jfoot[1].transpose() * mpc_planned_joint_force1;
    vector_t tor2 = Jfoot[2].transpose() * mpc_planned_joint_force2;
    vector_t tor3 = Jfoot[3].transpose() * mpc_planned_joint_force3;

     if (mpc_planned_joint_force0[2] < 0)
      tor0.setZero();
    if (mpc_planned_joint_force1[2] < 0)
      tor1.setZero();
    if (mpc_planned_joint_force2[2] < 0)
      tor2.setZero();
    if (mpc_planned_joint_force3[2] < 0)
      tor3.setZero();

    vector_t torall = -(tor0 + tor1 + tor2 + tor3);

   // std::cout << "torall: " << torall << std::endl;
  
  //loadControllerFlag_  setWalkFlag_
  /**/
  if(setWalkFlag_)
  {//行走
   // wbc_->setStanceMode(false);
    optimizedState.setZero();
    optimizedState.segment(6, 6) = currentObservation_.state.segment<6>(6);
    optimizedState.segment(6 + 6, jointDim_) = defalutJointPos_;
  //  std::cerr<<"defalutJointPos_: "<<defalutJointPos_.transpose()<<std::endl;
    //std::cerr << "setWalkFlag_6778786"<< std::endl;
  }
  else
  { 
    optimizedState.setZero();
    optimizedInput.setZero();
    optimizedState.segment(6, 6) = currentObservation_.state.segment<6>(6);                                                  
    optimizedState.segment(6 + 6, jointDim_) = defalutJointPos_;

    plannedMode = 3;
  //  wbc_->setStanceMode(true);
  }
  //const vector_t& mpc_planned_body_pos = optimizedState.segment(6, 6);
 // const vector_t& mpc_planned_joint_pos = optimizedState.segment(6 + 6, jointDim_);
 // const vector_t& mpc_planned_joint_vel = optimizedInput.segment(12, jointDim_);

  vector_t mpc_planned_body_pos = optimizedState.segment(6, 6);
  vector_t mpc_planned_joint_pos = optimizedState.segment(6 + 6, jointDim_);
  vector_t mpc_planned_joint_vel = optimizedInput.segment(12, jointDim_);
  // std::cerr<<"足底力："<<optimizedInput.segment(0,3)<<std::endl;
  // WBC
  /*wbcTimer_.startTimer();
  vector_t x = wbc_->update(optimizedState, optimizedInput, measuredRbdState_, plannedMode, period.toSec());
  const vector_t& wbc_planned_torque = x.tail(jointDim_);
  const vector_t& wbc_planned_joint_acc = x.segment(6, jointDim_);
  const vector_t& wbc_planned_body_acc = x.head(6);
  const vector_t& wbc_planned_contact_force = x.segment(6 + jointDim_, wbc_->getContactForceSize());
  wbcTimer_.endTimer();

  posDes_ = centroidal_model::getJointAngles(optimizedState, leggedInterface_->getCentroidalModelInfo());
  velDes_ = centroidal_model::getJointVelocities(optimizedInput, leggedInterface_->getCentroidalModelInfo());

  scalar_t dt = period.toSec();
  posDes_ = posDes_ + 0.5 * wbc_planned_joint_acc * dt * dt;

  velDes_ = velDes_ + wbc_planned_joint_acc * dt;*/

  vector_t output_torque(jointDim_);
  //*********************** Set Joint Command: Normal Tracking *****************************//
  std_msgs::Float64MultiArray ctrl_pos_msg, ctrl_vel_msg, ctrl_tau_msg;
  std_msgs::Float64MultiArray ctrl_actau_msg;
  ctrl_pos_msg.data.resize(10);
  ctrl_vel_msg.data.resize(10);
  ctrl_tau_msg.data.resize(10);
  ctrl_actau_msg.data.resize(10);

  std_msgs::Float64MultiArray  mpc_force_msg,wbc_force_msg;
  mpc_force_msg.data.resize(12);
  wbc_force_msg.data.resize(12);

  for (size_t j = 0; j < jointDim_; ++j)
  {
    //"Limit protection
    const auto& model = leggedInterface_->getPinocchioInterface().getModel();
    double lower_bound = model.lowerPositionLimit(6 + j);
    double upper_bound = model.upperPositionLimit(6 + j);
    if (!emergencyStopFlag_ && loadControllerFlag_ &&
        (hybridJointHandles_[j].getPosition() > upper_bound + 0.02 ||
         hybridJointHandles_[j].getPosition() < lower_bound - 0.02))
    {
      number=j;
      emergencyStopFlag_ = true;
      std::cerr << "Reach Position Limit!!!!!!!!!!!!!!!!!!!!!! " << j << ":" << hybridJointHandles_[j].getPosition()
                << std::endl;
    }
      if(mpc_planned_joint_pos[j]>12.0f)
      {
        mpc_planned_joint_pos[j]=12.0f;
      }
      else if(mpc_planned_joint_pos[j]<-12.0f)
      {
        mpc_planned_joint_pos[j]=-12.0f;
      }

      if(mpc_planned_joint_vel[j]>9.5f)
      {
        mpc_planned_joint_vel[j]=9.5f;
      }
      else if(mpc_planned_joint_vel[j]<-9.5f)
      {
        mpc_planned_joint_vel[j]=-9.5f;  
      }

      if(torall[6 + j]>26.0f)
      {
        torall[6 + j]=26.0f;
      }
      else if(torall[6 + j]<-26.0f)
      {
        torall[6 + j]=-26.0f;
      }

    if (!loadControllerFlag_)
    {//还没有使能控制器  
      if (j == 4 || j == 9)
      {
        hybridJointHandles_[j].setCommand(mpc_planned_joint_pos[j], mpc_planned_joint_vel[j], kp_position, kd_position, 0);
      }
      else
      {
        hybridJointHandles_[j].setCommand(mpc_planned_joint_pos[j], mpc_planned_joint_vel[j], kp_position, kd_position,0);
      }
    }
   else
   {
      contact_flag_t cmdContactFlag = modeNumber2StanceLeg(
          mpcMrtInterface_->getReferenceManager().getModeSchedule().modeAtTime(currentObservation_.time));
    
      ctrl_pos_msg.data[j] = mpc_planned_joint_pos[j];
      ctrl_vel_msg.data[j] = mpc_planned_joint_vel[j];
     // ctrl_tau_msg.data[j] = wbc_planned_torque(j);
      ctrl_tau_msg.data[j] = torall[6 + j];
      //std::cerr<<j<<": "<<torall[6 + j]<<std::endl;
     // ctrl_tau_msg.data[j] = mpc_planned_joint_tau(j);

     if(j == 0)
     {    
       hybridJointHandles_[j].setCommand(mpc_planned_joint_pos[j],  mpc_planned_joint_vel[j],
                                        cmdContactFlag[int(j / 5)] ? kp_hip_pitch_stance : kp_hip_pitch_swing,
                                        cmdContactFlag[int(j / 5)] ? kd_hip_pitch_stance : kd_hip_pitch_swing,
                                        torall[6 + j]);  
       //hybridJointHandles_[j].setCommand(-0.4,  mpc_planned_joint_vel[j],
       // cmdContactFlag[int(j / 5)] ? kp_hip_pitch_stance : kp_hip_pitch_swing,
       // cmdContactFlag[int(j / 5)] ? kd_hip_pitch_stance : kd_hip_pitch_swing,
      //  torall[6 + j]); 
     }
     else if(j ==1)
     {  
       hybridJointHandles_[j].setCommand(mpc_planned_joint_pos[j], mpc_planned_joint_vel[j],
                                      cmdContactFlag[int(j / 5)] ? kp_hip_roll_stance : kp_hip_roll_swing, kd_hip_roll,
                                       torall[6 + j]); 
        //hybridJointHandles_[j].setCommand(0.0, mpc_planned_joint_vel[j],
       //   cmdContactFlag[int(j / 5)] ? kp_hip_roll_stance : kp_hip_roll_swing, kd_hip_roll,
       //   torall[6 + j]); 
     }
     else if(j == 2 )
     {
        hybridJointHandles_[j].setCommand(mpc_planned_joint_pos[j], mpc_planned_joint_vel[j],
                                        cmdContactFlag[int(j / 5)] ? kp_hip_yaw_stance : kp_hip_yaw_swing, kd_hip_yaw,
                                        torall[6 + j]); 
        // hybridJointHandles_[j].setCommand(0.0, mpc_planned_joint_vel[j],
        //  cmdContactFlag[int(j / 5)] ? kp_hip_yaw_stance : kp_hip_yaw_swing, kd_hip_yaw,
        //  torall[6 + j]);
     }
     else if(j == 3)
     {    
       hybridJointHandles_[j].setCommand(mpc_planned_joint_pos[j], mpc_planned_joint_vel[j],
                                       cmdContactFlag[int(j / 5)] ? kp_knee_stance : kp_knee_swing, kd_knee,
                                       torall[6 + j]); 
        //hybridJointHandles_[j].setCommand(0.8, mpc_planned_joint_vel[j],
        //  cmdContactFlag[int(j / 5)] ? kp_knee_stance : kp_knee_swing, kd_knee,
        //  torall[6 + j]);
     }
     else if(j == 4 )
     {   
        hybridJointHandles_[j].setCommand(mpc_planned_joint_pos[j], mpc_planned_joint_vel[j],
                                       cmdContactFlag[int(j / 5)] ? kp_feet_stance : kp_feet_swing, kd_feet,
                                       torall[6 + j]); 
         //hybridJointHandles_[j].setCommand(-0.4 ,mpc_planned_joint_vel[j],
         // cmdContactFlag[int(j / 5)] ? kp_feet_stance : kp_feet_swing, kd_feet,
        //  torall[6 + j]); 
     }


     if(j == 5)
     {    
       hybridJointHandles_[j].setCommand(mpc_planned_joint_pos[j],  mpc_planned_joint_vel[j],
                                        cmdContactFlag[int(j / 5)] ? kp_hip_pitch_stance : kp_hip_pitch_swing,
                                        cmdContactFlag[int(j / 5)] ? kd_hip_pitch_stance : kd_hip_pitch_swing,
                                        torall[6 + j]);  
         //hybridJointHandles_[j].setCommand(-0.4,  mpc_planned_joint_vel[j],
        //  cmdContactFlag[int(j / 5)] ? kp_hip_pitch_stance : kp_hip_pitch_swing,
        //  cmdContactFlag[int(j / 5)] ? kd_hip_pitch_stance : kd_hip_pitch_swing,
        //  torall[6 + j]);  
     }

     else if(j ==6)
     {  
       hybridJointHandles_[j].setCommand(mpc_planned_joint_pos[j], mpc_planned_joint_vel[j],
                                       cmdContactFlag[int(j / 5)] ? kp_hip_roll_stance : kp_hip_roll_swing, kd_hip_roll,
                                       torall[6 + j]); 
        //hybridJointHandles_[j].setCommand(0.0, mpc_planned_joint_vel[j],
        //  cmdContactFlag[int(j / 5)] ? kp_hip_roll_stance : kp_hip_roll_swing, kd_hip_roll,
        //  torall[6 + j]); 
     }
     else if(j == 7)
     {
        hybridJointHandles_[j].setCommand(mpc_planned_joint_pos[j], mpc_planned_joint_vel[j],
                                        cmdContactFlag[int(j / 5)] ? kp_hip_yaw_stance : kp_hip_yaw_swing, kd_hip_yaw,
                                        torall[6 + j]); 
          //hybridJointHandles_[j].setCommand(0.0, mpc_planned_joint_vel[j],
          //  cmdContactFlag[int(j / 5)] ? kp_hip_yaw_stance : kp_hip_yaw_swing, kd_hip_yaw,
          //  torall[6 + j]);
     }
     else if(j == 8)
     {    
       hybridJointHandles_[j].setCommand(mpc_planned_joint_pos[j], mpc_planned_joint_vel[j],
                                       cmdContactFlag[int(j / 5)] ? kp_knee_stance : kp_knee_swing, kd_knee,
                                       torall[6 + j]); 
        //hybridJointHandles_[j].setCommand(0.8, mpc_planned_joint_vel[j],
        //  cmdContactFlag[int(j / 5)] ? kp_knee_stance : kp_knee_swing, kd_knee,
        //  torall[6 + j]);
     }
     else if(j == 9)
     {   
        hybridJointHandles_[j].setCommand(mpc_planned_joint_pos[j], mpc_planned_joint_vel[j],
                                       cmdContactFlag[int(j / 5)] ? kp_feet_stance : kp_feet_swing, kd_feet,
                                       torall[6 + j]); 
        //hybridJointHandles_[j].setCommand(-0.4, mpc_planned_joint_vel[j],
        //  cmdContactFlag[int(j / 5)] ? kp_feet_stance : kp_feet_swing, kd_feet,
        //  torall[6 + j]);
     }
          
      if (emergencyStopFlag_)
      {
        std::cerr<<"emergencyStopFlag77: "<<number<<std::endl;
        hybridJointHandles_[j].setCommand(0, 0, 0, 1, 0);
      }
  }

      posDesOutput_(j) = hybridJointHandles_[j].getPositionDesired();
      velDesOutput_(j) = hybridJointHandles_[j].getVelocityDesired();

      output_torque(j) = hybridJointHandles_[j].getFeedforward() +
                       hybridJointHandles_[j].getKp() *
                           (hybridJointHandles_[j].getPositionDesired() - hybridJointHandles_[j].getPosition()) +
                       hybridJointHandles_[j].getKd() *
                           (hybridJointHandles_[j].getVelocityDesired() - hybridJointHandles_[j].getVelocity());
  }
  //*********************** Set Joint Command: Torque Tracking Test *****************************//

  CommandData command_data;
  vector_t planned_state = currentObservation_.state;
  vector_t planned_input = currentObservation_.input;
  planned_state.tail(jointDim_) = posDesOutput_;
  planned_input.tail(jointDim_) = velDesOutput_;
  command_data.mpcTargetTrajectories_.timeTrajectory.push_back(currentObservation_.time);
  command_data.mpcTargetTrajectories_.stateTrajectory.push_back(planned_state);
  command_data.mpcTargetTrajectories_.inputTrajectory.push_back(planned_input);
  //上面181行运行完mpc计算后将mpc_updated_置1
  command_data = mpc_updated_ ? mpcMrtInterface_->getCommand() : command_data;
  PrimalSolution primal_solution = mpc_updated_ ? mpcMrtInterface_->getPolicy() : PrimalSolution();

  // Visualization
  robotVisualizer_->update(currentObservation_, primal_solution, command_data,
                           leggedInterface_->getSwitchedModelReferenceManagerPtr()->getSwingTrajectoryPlanner());
  selfCollisionVisualization_->update(currentObservation_);

  // Publish the observation. Only needed for the command interface
  //在setupMPC里创建了这个topic，observationPublisher_ = nh.advertise<ocs2_msgs::mpc_observation>(robotName + "_mpc_observation", 1);
  observationPublisher_.publish(ros_msg_conversions::createObservationMsg(currentObservation_));
  cmd_pos_pub_.publish(ctrl_pos_msg);
  cmd_vel_pub_.publish(ctrl_vel_msg);
  cmd_tau_pub_.publish(ctrl_tau_msg);
  cmd_actau_pub_.publish(ctrl_actau_msg);

  for(size_t i = 0; i < 12; ++i)
  {
    mpc_force_msg.data[i]=optimizedInput[i];

  }
  //std::cout<<"wbc_force: "<<std::endl;
 // for(size_t i = 0; i < 12; ++i)
 // {
  //   wbc_force_msg.data[i]=wbc_planned_contact_force[i];
  //  std::cout<<wbc_planned_contact_force[i]<<" ";
 // }
 // std::cout<<std::endl;
  
  mpc_force_pub_.publish(mpc_force_msg);
//wbc_force_pub_.publish(wbc_force_msg);
}

void LeggedController::updateStateEstimation(const ros::Time& time, const ros::Duration& period)
{
  vector_t jointPos(hybridJointHandles_.size()), jointVel(hybridJointHandles_.size()),
      jointTor(hybridJointHandles_.size());
  Eigen::Quaternion<scalar_t> quat;
  contact_flag_t cmdContactFlag;
  vector3_t angularVel, linearAccel;
  matrix3_t orientationCovariance, angularVelCovariance, linearAccelCovariance;

  //////////////////////////////////////////////////////////////////////////////////////////////////////////

  std_msgs::Float64MultiArray torque_msg, pos_msg, vel_msg;
  torque_msg.data.resize(10);
  pos_msg.data.resize(10);
  vel_msg.data.resize(10);

  for (size_t i = 0; i < hybridJointHandles_.size(); ++i)
  {
    jointPos(i) = hybridJointHandles_[i].getPosition();
    jointVel(i) = hybridJointHandles_[i].getVelocity();
    jointTor(i) = hybridJointHandles_[i].getEffort();
  }

  for (size_t k = 0; k < 9; ++k)
  {
    pos_msg.data[k] = jointPos(k);
    vel_msg.data[k] = jointVel(k);    
    torque_msg.data[k] = jointTor(k);
  }

  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = "odom";
  pose_stamped.pose.position.x = currentObservation_.state(6);
  pose_stamped.pose.position.y = currentObservation_.state(7);
  pose_stamped.pose.position.z = currentObservation_.state(8);
  pose_stamped.pose.orientation.x = 0.0;
  pose_stamped.pose.orientation.y = 0.0;
  pose_stamped.pose.orientation.z = 0.0;
  pose_stamped.pose.orientation.w = 1.0;
  pose_stamped.header.stamp = ros::Time::now(); 

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  cmdContactFlag = modeNumber2StanceLeg(
      mpcMrtInterface_->getReferenceManager().getModeSchedule().modeAtTime(currentObservation_.time));
  if (!firstStartMpc_)
  {
    for (size_t i = 0; i < 4; ++i)
    {
      cmdContactFlag[i] = true;
    }
  }
  stateEstimate_->updateCmdContact(cmdContactFlag);
  stateEstimate_->setStartStopTime4Legs(
      leggedInterface_->getSwitchedModelReferenceManagerPtr()->getSwingTrajectoryPlanner()->threadSaftyGetStartStopTime(
          currentObservation_.time));


  std_msgs::Float64MultiArray array;

  for (size_t i = 0; i < 4; ++i)
  {
    quat.coeffs()(i) = imuSensorHandle_.getOrientation()[i];
    array.data.push_back(quat.coeffs()(i));
  }

  for (size_t i = 0; i < 3; ++i)
  {
    angularVel(i) = imuSensorHandle_.getAngularVelocity()[i];
    array.data.push_back(angularVel(i));
   
  }

  for (size_t i = 0; i < 3; ++i)
  {
    linearAccel(i) = imuSensorHandle_.getLinearAcceleration()[i];
    array.data.push_back(linearAccel(i));
  }
  //linearAccel(0) = 0.0;
  //linearAccel(1) = 0.0;
  //linearAccel(2) = 9.81;
  //array.data.push_back(linearAccel(0));
  //array.data.push_back(linearAccel(1));
  //array.data.push_back(linearAccel(2));

  for (size_t i = 0; i < 9; ++i)
  {
    orientationCovariance(i) = imuSensorHandle_.getOrientationCovariance()[i];
    angularVelCovariance(i) = imuSensorHandle_.getAngularVelocityCovariance()[i];
    linearAccelCovariance(i) = imuSensorHandle_.getLinearAccelerationCovariance()[i];
  }
  //在上面的构造函数中imu_pub_ = controller_nh.advertise<std_msgs::Float64MultiArray>("imu", 10);
  imu_pub_.publish(array);

  //////////////////////////////////////////////////////////////////////////////////////////////////
  static std::chrono::steady_clock::time_point last_publish_time = std::chrono::steady_clock::now();
  auto current_time = std::chrono::steady_clock::now();
  if (current_time - last_publish_time > std::chrono::milliseconds(100)) {
    pos_pub_.publish(pos_msg);
    vel_pub_.publish(vel_msg);
    torque_pub_.publish(torque_msg);
    body_pose_pub_.publish(pose_stamped);
    
    //ROS_INFO_STREAM("current Height: " << currentObservation_.state(8));

    last_publish_time = current_time;
  }
  //ROS_INFO_STREAM("current Height: " << currentObservation_.state(8));
  //////////////////////////////////////////////////////////////////////////////////////////////////

  stateEstimate_->updateJointStates(jointPos, jointVel);
  stateEstimate_->updateContact(cmdContactFlag);
  stateEstimate_->updateImu(quat, angularVel, linearAccel, orientationCovariance, angularVelCovariance,
                            linearAccelCovariance);
  measuredRbdState_ = stateEstimate_->update(time, period);

  currentObservation_.time = time.toSec();
  scalar_t yawLast = currentObservation_.state(9);
  currentObservation_.state.head(stateDim_) = rbdConversions_->computeCentroidalStateFromRbdModel(measuredRbdState_);
  currentObservation_.state(9) = yawLast + angles::shortest_angular_distance(yawLast, currentObservation_.state(9));
  currentObservation_.mode = stateEstimate_->getMode();

  const auto& reference_manager = leggedInterface_->getSwitchedModelReferenceManagerPtr();
  reference_manager->getSwingTrajectoryPlanner()->setBodyVelWorld(stateEstimate_->getBodyVelWorld());
  reference_manager->setEstContactFlag(cmdContactFlag);

  stateEstimate_->setCmdTorque(jointTor);
  stateEstimate_->estContactForce(period);

  auto remove_gravity = linearAccel;
  remove_gravity(2) -= 9.81;
}

LeggedController::~LeggedController()
{
  controllerRunning_ = false;
  mpcRunning_ = false;
  if (mpcThread_.joinable())
  {
    mpcThread_.join();
  }
  std::cerr << "########################################################################";
  std::cerr << "\n### MPC Benchmarking";
  std::cerr << "\n###   Maximum : " << mpcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cerr << "\n###   Average : " << mpcTimer_.getAverageInMilliseconds() << "[ms]." << std::endl;
  std::cerr << "########################################################################";
  std::cerr << "\n### WBC Benchmarking";
  std::cerr << "\n###   Maximum : " << wbcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cerr << "\n###   Average : " << wbcTimer_.getAverageInMilliseconds() << "[ms].";
}

void LeggedController::setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile,
                                            const std::string& referenceFile, bool verbose)
{
  leggedInterface_ = std::make_shared<LeggedInterface>(taskFile, urdfFile, referenceFile);
  leggedInterface_->setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);
}

void LeggedController::setupMpc()
{
  mpc_ = std::make_shared<SqpMpc>(leggedInterface_->mpcSettings(), leggedInterface_->sqpSettings(),
                                  leggedInterface_->getOptimalControlProblem(), leggedInterface_->getInitializer());

  const std::string robotName = "legged_robot";
  ros::NodeHandle nh;
  auto rosReferenceManagerPtr =
      std::make_shared<RosReferenceManager>(robotName, leggedInterface_->getReferenceManagerPtr());
  rosReferenceManagerPtr->subscribe(nh);
  mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
  observationPublisher_ = nh.advertise<ocs2_msgs::mpc_observation>(robotName + "_mpc_observation", 1);
}

void LeggedController::setupMrt()
{
  mpcMrtInterface_ = std::make_shared<MPC_MRT_Interface>(*mpc_);
  mpcMrtInterface_->initRollout(&leggedInterface_->getRollout());
  mpcTimer_.reset();
  controllerRunning_ = true;
  mpcThread_ = std::thread([&]() {
    while (controllerRunning_)
    {
      try
      {
        executeAndSleep(
            [&]() {
              if (mpcRunning_)//mpcRunning_和loadControllerFlag_通过loadControllerCallback回调函数置1
              {
                mpcTimer_.startTimer();
                //运行mpc计算 mpc_.run(currentObservation.time, currentObservation.state);
                mpcMrtInterface_->advanceMpc();
                mpcTimer_.endTimer();
                firstStartMpc_ = true;
              }
            },
            leggedInterface_->mpcSettings().mpcDesiredFrequency_);
      }
      catch (const std::exception& e)
      {
        controllerRunning_ = false;
        ROS_ERROR_STREAM("[Ocs2 MPC thread] Error : " << e.what());
        stopRequest(ros::Time());
      }
    }
  });
  setThreadPriority(leggedInterface_->sqpSettings().threadPriority, mpcThread_);
}

void LeggedController::setupStateEstimate(const std::string& taskFile, bool verbose)
{
  stateEstimate_ = std::make_shared<KalmanFilterEstimate>(
      leggedInterface_->getPinocchioInterface(), leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_);
  stateEstimate_->loadSettings(taskFile, verbose);
  dynamic_cast<KalmanFilterEstimate&>(*stateEstimate_).loadSettings(taskFile, verbose);
  currentObservation_.time = 0;
}

void LeggedController::dynamicParamCallback(legged_controllers::TutorialsConfig& config, uint32_t level)
{
  kp_position = config.kp_position;
  kd_position = config.kd_position;

  kp_hip_pitch_stance = config.kp_hip_pitch_stance;
  kp_hip_pitch_swing = config.kp_hip_pitch_swing;
  kd_hip_pitch_stance = config.kd_hip_pitch_stance;
  kd_hip_pitch_swing = config.kd_hip_pitch_swing;

  kp_hip_roll_stance = config.kp_hip_roll_stance;
  kp_hip_roll_swing = config.kp_hip_roll_swing;
  kd_hip_roll = config.kd_hip_roll;

  kp_hip_yaw_stance = config.kp_hip_yaw_stance;
  kp_hip_yaw_swing = config.kp_hip_yaw_swing;
  kd_hip_yaw = config.kd_hip_yaw;

  kp_knee_stance = config.kp_knee_stance;
  kp_knee_swing = config.kp_knee_swing;
  kd_knee = config.kd_knee;
 
  kp_feet_stance = config.kp_feet_stance;
  kp_feet_swing = config.kp_feet_swing;
  kd_feet = config.kd_feet;
}

void LeggedController::RetrievingParameters()
{
  stateDim_ = leggedInterface_->getCentroidalModelInfo().stateDim;
  inputDim_ = leggedInterface_->getCentroidalModelInfo().inputDim;
  jointDim_ = leggedInterface_->getCentroidalModelInfo().actuatedDofNum;
  footDim_ = leggedInterface_->getCentroidalModelInfo().numThreeDofContacts;
  gencoordDim_ = leggedInterface_->getCentroidalModelInfo().generalizedCoordinatesNum;
  dofPerLeg_ = jointDim_ / 2;

  defalutJointPos_.resize(jointDim_);
}

void LeggedController::resetMPC()
{
  TargetTrajectories target_trajectories({ currentObservation_.time }, { currentObservation_.state },
                                         { currentObservation_.input });
  mpcMrtInterface_->resetMpcNode(target_trajectories);
}
void LeggedController::ModeSubscribe()
{
  subSetWalk_ =
      ros::NodeHandle().subscribe<std_msgs::Float32>("/set_walk", 1, &LeggedController::setWalkCallback, this);
  subLoadcontroller_ = ros::NodeHandle().subscribe<std_msgs::Float32>("/load_controller", 1,
                                                                      &LeggedController::loadControllerCallback, this);
  subEmgstop_ = ros::NodeHandle().subscribe<std_msgs::Float32>("/emergency_stop", 1,
                                                               &LeggedController::EmergencyStopCallback, this);
  subResetTarget_ = ros::NodeHandle().subscribe<std_msgs::Float32>("/reset_estimation", 1,
                                                                  &LeggedController::ResetTargetCallback, this);                                                               
}

void LeggedController::EmergencyStopCallback(const std_msgs::Float32::ConstPtr& msg)
{
  emergencyStopFlag_ = true;
  ROS_INFO("unload the controller");
}

void LeggedController::setWalkCallback(const std_msgs::Float32::ConstPtr& msg)
{
  setWalkFlag_ = true;
  ROS_INFO("Set WALK Mode");
}

void LeggedController::loadControllerCallback(const std_msgs::Float32::ConstPtr& msg)
{
  loadControllerFlag_ = true;
  mpcRunning_ = true;
  ROS_INFO("Successfully load the controller");
}

void LeggedController::ResetTargetCallback(const std_msgs::Float32::ConstPtr& msg)
{
    // Initial state
  currentObservation_.state.setZero(stateDim_);
  currentObservation_.input.setZero(inputDim_);
  currentObservation_.state.segment(6 + 6, jointDim_) = defalutJointPos_;
  currentObservation_.mode = ModeNumber::STANCE;

  TargetTrajectories target_trajectories({ currentObservation_.time }, { currentObservation_.state },
                                         { currentObservation_.input });

  mpcMrtInterface_->getReferenceManager().setTargetTrajectories(target_trajectories);
  ROS_INFO("Reset the target");

} 

}  // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::LeggedController, controller_interface::ControllerBase)