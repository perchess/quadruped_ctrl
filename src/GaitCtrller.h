#ifndef GAIT_CTRLLER_H
#define GAIT_CTRLLER_H

#include <math.h>
#include <time.h>

#include <iostream>
#include <memory>
#include <string>
#include <ros/ros.h>

#include "Controllers/ContactEstimator.h"
#include "Controllers/ControlFSMData.h"
#include "Controllers/DesiredStateCommand.h"
#include "Controllers/OrientationEstimator.h"
#include "Controllers/PositionVelocityEstimator.h"
#include "Controllers/RobotLegState.h"
#include "Controllers/StateEstimatorContainer.h"
#include "Controllers/SafetyChecker.h"
#include "Dynamics/MiniCheetah.h"
#include "Dynamics/unitree_a1.h"
#include "MPC_Ctrl/ConvexMPCLocomotion.h"
#include "Utilities/IMUTypes.h"
#include "calculateTool.h"
#include <quadruped_msgs/generalConfig.h>
#include <quadruped_msgs/CMPC_Result.h>

#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include <unitree_legged_msgs/HighState.h>

struct JointEff {
  double eff[12];
};

/*!
 * @brief Структура для хранения параметров регуляторов
 * @param kpCartesian : пропорциональный коеффициент для перемещения в декардовом пространстве
 * @param kdCartesian : дифференциальный коеффициент для перемещения в декардовом пространстве
 * @param kpJoint : пропорциональный коеффициент для перемещения в конфигурационном пространстве
 * @param kdJoint : дифференциальный коеффициент для перемещения в конфигурационном пространстве
 */
struct PDcoeffs {
  PDcoeffs(float kpCartesian, float kdCartesian, float kpJoint, float kdJoint)
    : kpCartesian(kpCartesian)
    , kdCartesian(kdCartesian)
    , kpJoint(kpJoint)
    , kdJoint(kdJoint)
  {}
  float kpCartesian;
  float kdCartesian;
  float kpJoint;
  float kdJoint;
};

class GaitCtrller {
 public:
  GaitCtrller(double freq, std::vector<float> ctrlParam);
  ~GaitCtrller();
  void SetIMUData(double* imuData);
  void SetLegData(double* motorData);
//  void PreWork(double* imuData, double* motorData);
  void PreWork(VectorNavData& imuData, LegData& motorData);
  void SetGaitType(int gaitType);
  void SetRobotMode(int mode);
  void SetRobotVel(double& x, double&y, double& z);
  void SetLegParams(PDcoeffs coefs);
//  void TorqueCalculator(double* imuData, double* motorData, double* effort);
  void jump(bool trigger);
  void updateConfig(quadruped_msgs::generalConfig& cfg);
  Eigen::VectorXd TorqueCalculator(VectorNavData& imuData, LegData& motorData);
  void publushDebugToRos(VectorNavData& imu, LegData& legData);
  std::shared_ptr<ConvexMPCLocomotion> getConvexMpcPtr() {return convexMPC;}

 private:
  int _gaitType = 0;
  int _robotMode = 0;
  bool _safetyCheck = true;
  std::vector<double> _gamepadCommand;
  std::vector<float> ctrlParam;

  Quadruped<float> _quadruped;
  FloatingBaseModel<float> _model;
  std::shared_ptr<ConvexMPCLocomotion> convexMPC;
  std::unique_ptr<LegController<float>> _legController;
  std::unique_ptr<StateEstimatorContainer<float>> _stateEstimator;
  LegData _legdata;
  LegCommand legcommand;
  ControlFSMData<float> control_data;
  VectorNavData _vectorNavData;
  std::unique_ptr<CheaterState<double>> cheaterState;
  StateEstimate<float> _stateEstimate;
  std::unique_ptr<RobotControlParameters> controlParameters;
  std::unique_ptr<DesiredStateCommand<float>> _desiredStateCommand;
  std::unique_ptr<SafetyChecker<float>> safetyChecker;
  quadruped_msgs::generalConfig config_;

  ros::NodeHandle nh_;
  ros::Publisher low_state_pub_;
  ros::Publisher high_state_pub_;
  ros::Publisher cmpc_res_pub_;
};

//extern "C" {

//GaitCtrller* gCtrller = NULL;
//JointEff jointEff;

//// first step, init the controller
////void init_controller(double freq, double PIDParam[]) {
////  if (NULL != gCtrller) {
////    delete gCtrller;
////  }
////  gCtrller = new GaitCtrller(freq, PIDParam);
////}

//// the kalman filter need to work second
//void pre_work(double imuData[], double legData[]) {
//  gCtrller->PreWork(imuData, legData);
//}

//// gait type can be set in any time
//void set_gait_type(int gaitType) { gCtrller->SetGaitType(gaitType); }

//// set robot mode, 0: High performance model, 1: Low power mode
//void set_robot_mode(int mode) { gCtrller->SetRobotMode(mode); }

//// robot vel can be set in any time
//void set_robot_vel(double vel[]) { gCtrller->SetRobotVel(vel); }

//// after init controller and pre work, the mpc calculator can work
//JointEff* torque_calculator(double imuData[], double motorData[]) {
//  double eff[12] = {0.0};
//  gCtrller->TorqueCalculator(imuData, motorData, eff);
//  for (int i = 0; i < 12; i++) {
//    jointEff.eff[i] = eff[i];
//  }
//  return &jointEff;
//}
//}

#endif
