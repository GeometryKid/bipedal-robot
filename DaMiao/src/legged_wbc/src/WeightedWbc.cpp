//
// Created by qiayuan on 22-12-23.
//

/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#include "legged_wbc/WeightedWbc.h"

#include <qpOASES.hpp>

namespace legged
{
vector_t WeightedWbc::update(const vector_t& stateDesired, const vector_t& inputDesired,
                             const vector_t& rbdStateMeasured, size_t mode, scalar_t period)
{
  WbcBase::update(stateDesired, inputDesired, rbdStateMeasured, mode, period);

  // Constraints
  Task constraints = formulateConstraints();
  size_t numConstraints = constraints.b_.size() + constraints.f_.size();
  //std::cout << "numConstraints: " << numConstraints << std::endl;//输出56
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A(numConstraints, getNumDecisionVars());
  vector_t lbA(numConstraints), ubA(numConstraints);  // clang-format off
  A << constraints.a_,
       constraints.d_;

  lbA << constraints.b_,
         -qpOASES::INFTY * vector_t::Ones(constraints.f_.size());
  ubA << constraints.b_,
         constraints.f_;  // clang-format on

  // Cost
  Task weighedTask = formulateWeightedTasks(stateDesired, inputDesired, period);
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H =
      weighedTask.a_.transpose() * weighedTask.a_;
  vector_t g = -weighedTask.a_.transpose() * weighedTask.b_;

  // Solve
  auto qpProblem = qpOASES::QProblem(getNumDecisionVars(), numConstraints);
  qpOASES::Options options;
  options.setToMPC();
  options.printLevel = qpOASES::PL_LOW;
  options.enableEqualities = qpOASES::BT_TRUE;
  qpProblem.setOptions(options);
  int nWsr = 20;

  qpProblem.init(H.data(), g.data(), A.data(), nullptr, nullptr, lbA.data(), ubA.data(), nWsr);
  vector_t qpSol(getNumDecisionVars());

  qpProblem.getPrimalSolution(qpSol.data());

  if (!qpProblem.isSolved())
  {
    std::cout << "ERROR: WeightWBC Not Solved!!!" << std::endl;
    if (last_qpSol.size() > 0)
      qpSol = last_qpSol;
  }

  last_qpSol = qpSol;
  return qpSol;
}

Task WeightedWbc::formulateConstraints()
{//浮动机动力方程约束、关节扭矩限制、摩擦锥约束
  return formulateFloatingBaseEomTask() + formulateTorqueLimitsTask() + formulateFrictionConeTask();
}

Task WeightedWbc::formulateWeightedTasks(const vector_t& stateDesired, const vector_t& inputDesired, scalar_t period)
{
  if (stance_mode_)
    return formulateStanceBaseAccelTask(stateDesired, inputDesired, period) * weightBaseAccel_;
  else
    return formulateSwingLegTask() * weightSwingLeg_ +//摆动腿位置任务
           formulateBaseAccelTask(stateDesired, inputDesired, period) * weightBaseAccel_ +//机体加速度任务，包括线加速度和角加速度
           formulateContactForceTask(inputDesired) * weightContactForce_;//接触力任务
}

Task WeightedWbc::formulateStanceBaseAccelTask(const vector_t& stateDesired, const vector_t& inputDesired,
                                               scalar_t period)
{
  matrix_t a(6, numDecisionVars_);
  a.setZero();
  a.block(0, 0, 6, 6) = matrix_t::Identity(6, 6);

  vector6_t b;
  b.setZero();//站立状态下，让浮动基线加速度和角速度的约束为0

  return { a, b, matrix_t(), vector_t() };
}

void WeightedWbc::loadTasksSetting(const std::string& taskFile, bool verbose)
{
  WbcBase::loadTasksSetting(taskFile, verbose);

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::string prefix = "weight.";
  if (verbose)
  {
    std::cerr << "\n #### WBC weight:";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, weightSwingLeg_, prefix + "swingLeg", verbose);
  loadData::loadPtreeValue(pt, weightBaseAccel_, prefix + "baseAccel", verbose);
  loadData::loadPtreeValue(pt, weightContactForce_, prefix + "contactForce", verbose);
}

}  // namespace legged
