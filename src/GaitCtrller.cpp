#include "GaitCtrller.h"

GaitCtrller::GaitCtrller(double freq, std::vector<float> ctrlParam)
  : ctrlParam(ctrlParam)
  , _quadruped( buildUnitreeA1<float>() )
  , _model{ _quadruped.buildModel() }
  , convexMPC{ std::make_shared<ConvexMPCLocomotion>(1.0 / freq, 27 / (1000. / freq) ) }
  , _legController( std::make_unique<LegController<float>>(_quadruped) )
  , _stateEstimator( std::make_unique<StateEstimatorContainer<float>>(cheaterState.get(), &_vectorNavData, _legController->datas, &_stateEstimate,
                                                                      controlParameters.get()) )
  , _desiredStateCommand( std::make_unique<DesiredStateCommand<float>>(1.0 / freq) )
  , safetyChecker( std::make_unique<SafetyChecker<float>>() )
{
  _gamepadCommand.resize(4);

  // reset the state estimator
  _stateEstimator->removeAllEstimators();
  _stateEstimator->addEstimator<ContactEstimator<float>>();
  Vec4<float> contactDefault;
  contactDefault << 0.5, 0.5, 0.5, 0.5;
  _stateEstimator->setContactPhase(contactDefault);

  _stateEstimator->addEstimator<VectorNavOrientationEstimator<float>>();
  _stateEstimator->addEstimator<LinearKFPositionVelocityEstimator<float>>();

  std::cout << "finish init controller" << std::endl;
  low_state_pub_ = nh_.advertise<unitree_legged_msgs::LowState>("low_mpc", 1);
  high_state_pub_ = nh_.advertise<unitree_legged_msgs::HighState>("high_mpc", 1);
  cmpc_res_pub_ = nh_.advertise<quadruped_msgs::CMPC_Result>("cmpc_result", 1);
}

GaitCtrller::~GaitCtrller() {}

void GaitCtrller::SetIMUData(double* imuData) {
  _vectorNavData.accelerometer(0, 0) = imuData[0];
  _vectorNavData.accelerometer(1, 0) = imuData[1];
  _vectorNavData.accelerometer(2, 0) = imuData[2];
  _vectorNavData.quat(0, 0) = imuData[3];
  _vectorNavData.quat(1, 0) = imuData[4];
  _vectorNavData.quat(2, 0) = imuData[5];
  _vectorNavData.quat(3, 0) = imuData[6];
  _vectorNavData.gyro(0, 0) = imuData[7];
  _vectorNavData.gyro(1, 0) = imuData[8];
  _vectorNavData.gyro(2, 0) = imuData[9];
}

void GaitCtrller::SetLegData(double* motorData) {
  for (int i = 0; i < 4; i++) {
    _legdata.q_abad[i] = motorData[i * 3];
    _legdata.q_hip[i] = motorData[i * 3 + 1];
    _legdata.q_knee[i] = motorData[i * 3 + 2];
    _legdata.qd_abad[i] = motorData[12 + i * 3];
    _legdata.qd_hip[i] = motorData[12 + i * 3 + 1];
    _legdata.qd_knee[i] = motorData[12 + i * 3 + 2];
  }
}

//void GaitCtrller::PreWork(double* imuData, double* motorData) {
//  SetIMUData(imuData);
//  SetLegData(motorData);
//  _stateEstimator->run();
//  _legController->updateData(&_legdata);
//}

void GaitCtrller::PreWork(VectorNavData& imuData, LegData& motorData) {
  _vectorNavData = imuData;
  // Меняю направления сочленений для unitree a1
  //  for (int leg = 0; leg < 4; leg++)
  //  {
  //    motorData.q_hip[leg] = -motorData.q_hip[leg]; // hip
  //    motorData.q_knee[leg] = -motorData.q_knee[leg]; // knee
  //    motorData.qd_hip[leg] = -motorData.qd_hip[leg]; // hip
  //    motorData.qd_knee[leg] = -motorData.qd_knee[leg]; // knee
  //  }
  publushDebugToRos(_vectorNavData, _legdata);
  _legdata = motorData;
  _stateEstimator->run();
  _legController->updateData(&_legdata);
}

void GaitCtrller::SetGaitType(int gaitType) {
  _gaitType = gaitType;
  std::cout << "set gait type to: " << _gaitType << std::endl;
}

void GaitCtrller::SetRobotMode(int mode) {
  _robotMode = mode;
  std::cout << "set robot mode to: " << _robotMode << std::endl;
}

void GaitCtrller::SetRobotVel(double x, double y, double z) {
  if (abs(x) < 0.03) {
    _gamepadCommand[0] = 0.0;
  } else {
    _gamepadCommand[0] = x * 1.0;
  }

  if (abs(y) < 0.03) {
    _gamepadCommand[1] = 0.0;
  } else {
    _gamepadCommand[1] = y * 1.0;
  }

  if (abs(z) < 0.03) {
    _gamepadCommand[2] = 0.0;
  } else {
    _gamepadCommand[2] = z * 1.0;
  }
}

Eigen::VectorXd GaitCtrller::TorqueCalculator(VectorNavData& imuData, LegData& motorData)
{
  PreWork(imuData, motorData);

//    std::cout << "buffer size = " << convexMPC->getPointsBuffer()->size() << std::endl;

//  if (convexMPC->getPointsBuffer()->size() >= 5000)
//  {
    for (int leg = 0; leg < 1; leg++)
    {
      Pf_ = convexMPC->computePf(_quadruped,*_stateEstimator, leg);
      Pf_.z() = convexMPC->getPointsBuffer()->front().z();
//      if ( !canPlace(Pf_, *convexMPC->getPointsBuffer(), 0.01) )
//      {
//        //      std::cout << "STOP ROBOT" << std::endl;
//        SetRobotVel(_gamepadCommand[0]/2.0,
//            _gamepadCommand[1]/2.0,
//            _gamepadCommand[2]/2.0);
//      }
    }
//  }


  // Setup the leg controller for a new iteration
  _legController->zeroCommand();
  _legController->setEnabled(true);
  //  _legController->setMaxTorqueCheetah3(208.5);

  // Find the desired state trajectory
  _desiredStateCommand->convertToStateCommands(_gamepadCommand);

  //safety check
  if(!safetyChecker->checkSafeOrientation(*_stateEstimator)){
    _safetyCheck = false;
    ROS_ERROR_ONCE("Orientation safety check failed!\n");
    ROS_DEBUG("Orientation safety check failed!\n");

  }else if (!safetyChecker->checkPDesFoot(_quadruped, *_legController)) {
    _safetyCheck = false;
    std::cout << "broken: Foot Position Safety Check FAIL" << std::endl;

  }else if (!safetyChecker->checkForceFeedForward(*_legController)) {
    _safetyCheck = false;
    std::cout << "broken: Force FeedForward Safety Check FAIL" << std::endl;

  }else if (!safetyChecker->checkJointLimit(*_legController)) {
    //    _safetyCheck = false;
    ROS_WARN_STREAM_ONCE(ros::this_node::getName() + " : IGNORED JOINT LIMIT CHECK!!!");
    //    ROS_ERROR_STREAM_ONCE("broken: Joint Limit Safety Check FAIL");
    //    std::cout << "broken: Joint Limit Safety Check FAIL" << std::endl;
  }


  convexMPC->run(_quadruped, *_legController, *_stateEstimator,
                 *_desiredStateCommand, _gamepadCommand, _gaitType, _robotMode);
  // Если не получается поставить ногу, останавливаемся
  //  if ( !convexMPC->run(_quadruped, *_legController, *_stateEstimator,
  //                 *_desiredStateCommand, _gamepadCommand, _gaitType, _robotMode) )
  //  {
  //    std::fill(_gamepadCommand.begin(), _gamepadCommand.end(),0.0);
  //    _legController->zeroCommand();
  //    convexMPC->run(_quadruped, *_legController, *_stateEstimator,
  //                     *_desiredStateCommand, _gamepadCommand, _gaitType, _robotMode);
  //  }

  _legController->updateCommand(&legcommand);

  Eigen::VectorXd effort = VectorXd::Zero(12);

  if(_safetyCheck) {
    for (int i = 0; i < 4; i++) {
      effort(i * 3) = legcommand.tau_abad_ff[i];
      effort(i * 3 + 1) = legcommand.tau_hip_ff[i];
      effort(i * 3 + 2) = legcommand.tau_knee_ff[i];
    }
  } else {
    for (int i = 0; i < 4; i++) {
      effort(i * 3) = 0.0;
      effort(i * 3 + 1) = 0.0;
      effort(i * 3 + 2) = 0.0;
    }
  }

  return effort;
}

//void GaitCtrller::TorqueCalculator(double* imuData, double* motorData,
//                                   double* effort) {
//  PreWork(imuData, motorData);

//  // Setup the leg controller for a new iteration
//  _legController->zeroCommand();
//  _legController->setEnabled(true);
//  _legController->setMaxTorqueCheetah3(208.5);

//  // Find the desired state trajectory
//  _desiredStateCommand->convertToStateCommands(_gamepadCommand);

//  //safety check
//  if(!safetyChecker->checkSafeOrientation(*_stateEstimator)){
//    _safetyCheck = false;
//    std::cout << "broken: Orientation Safety Check FAIL" << std::endl;

//  }else if (!safetyChecker->checkPDesFoot(_quadruped, *_legController)) {
//    _safetyCheck = false;
//    std::cout << "broken: Foot Position Safety Check FAIL" << std::endl;

//  }else if (!safetyChecker->checkForceFeedForward(*_legController)) {
//    _safetyCheck = false;
//    std::cout << "broken: Force FeedForward Safety Check FAIL" << std::endl;

//  }else if (!safetyChecker->checkJointLimit(*_legController)) {
//    _safetyCheck = false;
//    std::cout << "broken: Joint Limit Safety Check FAIL" << std::endl;
//  }

//  convexMPC->run(_quadruped, *_legController, *_stateEstimator,
//                 *_desiredStateCommand, _gamepadCommand, _gaitType, _robotMode);

//  _legController->updateCommand(&legcommand);

//  if(_safetyCheck) {
//    for (int i = 0; i < 4; i++) {
//      effort[i * 3] = legcommand.tau_abad_ff[i];
//      effort[i * 3 + 1] = legcommand.tau_hip_ff[i];
//      effort[i * 3 + 2] = legcommand.tau_knee_ff[i];
//    }
//  } else {
//    for (int i = 0; i < 4; i++) {
//      effort[i * 3] = 0.0;
//      effort[i * 3 + 1] = 0.0;
//      effort[i * 3 + 2] = 0.0;
//    }
//  }

//  // return effort;
//}

void GaitCtrller::jump(bool trigger)
{
  if (trigger)
  {
    _desiredStateCommand->trigger_pressed = true;
  }
}

void GaitCtrller::SetLegParams(PDcoeffs coefs)
{
  ctrlParam[0] = coefs.kpCartesian;
  ctrlParam[1] = coefs.kdCartesian;
  ctrlParam[2] = coefs.kpJoint;
  ctrlParam[3] = coefs.kdJoint;
}

void GaitCtrller::updateConfig(quadruped_msgs::generalConfig& cfg)
{
  // Сброс параметров ног
  //    _legController->zeroCommand();

  config_ = cfg;
  // Коэффициенты для ног
  Mat3<float> kp;
  Mat3<float> kd;
  Mat3<float> kpJ;
  Mat3<float> kdJ;
  kp <<   config_.kp_cartesian_x, 0, 0,
      0, config_.kp_cartesian_y, 0,
      0, 0, config_.kp_cartesian_z;
  kd <<   config_.kd_cartesian_x, 0, 0,
      0, config_.kd_cartesian_y, 0,
      0, 0, config_.kd_cartesian_z;

  // Коэффициенты для сочленений
  for (size_t leg = 0; leg < 4; leg++)
  {
    kpJ <<
           config_.kp_joint_abad, 0, 0,
        0, config_.kp_joint_hip, 0,
        0, 0, config_.kp_joint_knee;
    kdJ <<
           config_.kd_joint_abad, 0, 0,
        0, config_.kd_joint_hip, 0,
        0, 0, config_.kd_joint_knee;
  }
  convexMPC->setPDcoefs(kp, kd, kpJ, kdJ);

  //  // Установить походку
  this->SetGaitType(cfg.gait);

  // Изменение горизонта планирования
  convexMPC->setHorizon(cfg.horizon_length);

  // Задать период шага
  convexMPC->setTrotDuration(cfg.trot_duration);

  // Регулирование высоты шага
  convexMPC->setStepHeight(cfg.step_height);

  //  convexMPC->setBodyHeight(float(cfg.body_height), "default");
  //  convexMPC->setBodyHeight(float(cfg.body_height_run), "run");
  //  convexMPC->setBodyHeight(float(cfg.body_height_jump), "jump");

  // "Плавная" походка
  this->SetRobotMode(cfg.walk_mode);
}

void GaitCtrller::publushDebugToRos(VectorNavData& imu, LegData& legData)
{
  unitree_legged_msgs::LowState msg;
  unitree_legged_msgs::HighState high_msg;
  quadruped_msgs::CMPC_Result cmpc_msg;

  // IMU
  msg.header.stamp = ros::Time::now();
  msg.imu.accelerometer.at(0) = imu.accelerometer(0);
  msg.imu.accelerometer.at(1) = imu.accelerometer(1);
  msg.imu.accelerometer.at(2) = imu.accelerometer(2);
  msg.imu.gyroscope.at(0) = imu.gyro(0);
  msg.imu.gyroscope.at(1) = imu.gyro(1);
  msg.imu.gyroscope.at(2) = imu.gyro(2);
  msg.imu.quaternion.at(0) = imu.quat(0);
  msg.imu.quaternion.at(1) = imu.quat(1);
  msg.imu.quaternion.at(2) = imu.quat(2);
  msg.imu.quaternion.at(3) = imu.quat(3);
  // Motors
  for (size_t leg = 0; leg < 4; leg++) {
    msg.motorState.at(leg * 3).q = legData.q_abad[leg];
    msg.motorState.at(leg * 3 + 1).q = legData.q_hip[leg];
    msg.motorState.at(leg * 3 + 2).q = legData.q_knee[leg];

    msg.motorState.at(leg * 3).dq = legData.qd_abad[leg];
    msg.motorState.at(leg * 3 + 1).dq = legData.qd_hip[leg];
    msg.motorState.at(leg * 3 + 2).dq = legData.qd_knee[leg];
  }
  // Estimators

  // Вывод pos векторы (size 3)
  high_msg.footPosition2Body.at(0).x =
      _stateEstimator->getEstimator(2)->_stateEstimatorData.result->
      position.transpose().operator()(0);
  high_msg.footPosition2Body.at(0).y =
      _stateEstimator->getEstimator(2)->_stateEstimatorData.result->
      position.transpose().operator()(1);
  high_msg.footPosition2Body.at(0).z =
      _stateEstimator->getEstimator(2)->_stateEstimatorData.result->
      position.transpose().operator()(2);

  // Вывод vWorld вектор (size 3)
  high_msg.footPosition2Body.at(1).x =
      _stateEstimator->getEstimator(2)->_stateEstimatorData.result->
      vWorld.transpose().operator()(0);
  high_msg.footPosition2Body.at(1).y =
      _stateEstimator->getEstimator(2)->_stateEstimatorData.result->
      vWorld.transpose().operator()(1);
  high_msg.footPosition2Body.at(1).z =
      _stateEstimator->getEstimator(2)->_stateEstimatorData.result->
      vWorld.transpose().operator()(2);

  // Вывод vBody вектор (size 3)
  high_msg.footPosition2Body.at(2).x =
      _stateEstimator->getEstimator(2)->_stateEstimatorData.result->
      vBody.transpose().operator()(0);
  high_msg.footPosition2Body.at(2).y =
      _stateEstimator->getEstimator(2)->_stateEstimatorData.result->
      vBody.transpose().operator()(1);
  high_msg.footPosition2Body.at(2).z =
      _stateEstimator->getEstimator(2)->_stateEstimatorData.result->
      vBody.transpose().operator()(2);

  // CCMPC структура (выход)
  for (size_t leg = 0; leg < 4; leg++)
  {
    for(size_t joint =0; joint < 3; joint++)
    {
      cmpc_msg.commands.at(leg).forceFeedForward.at(joint) =
          _legController->commands[leg].forceFeedForward(joint);
      cmpc_msg.commands.at(leg).tauFeedForward.at(joint) =
          _legController->commands[leg].tauFeedForward(joint);
      cmpc_msg.commands.at(leg).qDes.at(joint) =
          _legController->commands[leg].qDes(joint);
      cmpc_msg.commands.at(leg).qdDes.at(joint) =
          _legController->commands[leg].qdDes(joint);
      cmpc_msg.commands.at(leg).pDes.at(joint) =
          _legController->commands[leg].pDes(joint);
      cmpc_msg.commands.at(leg).vDes.at(joint) =
          _legController->commands[leg].vDes(joint);
      cmpc_msg.commands.at(leg).pAct.at(joint) =
          _legController->datas[leg].p(joint);
      cmpc_msg.commands.at(leg).vAct.at(joint) =
          _legController->datas[leg].v(joint);
      cmpc_msg.commands.at(leg).pError.at(joint) =
          (_legController->commands[leg].pDes(joint) - _legController->datas[leg].p(joint));
      cmpc_msg.commands.at(leg).vError.at(joint) =
          (_legController->commands[leg].vDes(joint) - _legController->datas[leg].v(joint));
      cmpc_msg.commands.at(leg).kpCartesian.at(joint) =
          _legController->commands[leg].kpCartesian(joint,joint);
      cmpc_msg.commands.at(leg).kdCartesian.at(joint) =
          _legController->commands[leg].kdCartesian(joint,joint);
      cmpc_msg.commands.at(leg).kpJoint.at(joint) =
          _legController->commands[leg].kpJoint(joint,joint);
      cmpc_msg.commands.at(leg).kdJoint.at(joint) =
          _legController->commands[leg].kdJoint(joint,joint);
      cmpc_msg.contactPhase.at(leg) = convexMPC->getCCMPCResult().contactPhase(leg);
    }
  }
  cmpc_res_pub_.publish(cmpc_msg);
  low_state_pub_.publish(msg);
  high_state_pub_.publish(high_msg);
}
