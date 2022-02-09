#include "ConvexMPCLocomotion.h"

#include <iostream>

#include "Utilities/Timer.h"
#include "Utilities/Utilities_print.h"
#include "convexMPC_interface.h"
// #include "../../../../common/FootstepPlanner/GraphSearch.h"

// #include "Gait.h"

//#define DRAW_DEBUG_SWINGS
//#define DRAW_DEBUG_PATH

////////////////////
// Controller
// 参考Dynamic Locomotion in the MIT Cheetah 3 Through Convex Model-Predictive
// Control 一个步态周期由horizonLength(10)个mpc周期组成 步态按1KHz处理
// mpc计数间隔为30左右 一毫秒计数一次来控制频率 即一个mpc周期为30ms 则步态周期为
// 10*30 =300ms
////////////////////

ConvexMPCLocomotion::ConvexMPCLocomotion(float _dt, int _iterations_between_mpc)
  : iterationsBetweenMPC(_iterations_between_mpc) //[китайский] Для регулирования частоты
  , horizonLength(10) // Значение по умолчанию
  , trot_duration_(horizonLength/2.0)
  , dt(_dt)
  , trotting(horizonLength, Vec4<int>(0, horizonLength/2.0, horizonLength/2.0, 0),
             Vec4<int>(trot_duration_, trot_duration_, trot_duration_, trot_duration_), "Trotting")
  , bounding(horizonLength, Vec4<int>(horizonLength/2.0, horizonLength/2.0, 0, 0),
             Vec4<int>(horizonLength/2.0 - 1.0, horizonLength/2.0 - 1.0, horizonLength/2.0 - 1.0, horizonLength/2.0 - 1.0), "Bounding")
  , pronking(horizonLength, Vec4<int>(0, 0, 0, 0),
             Vec4<int>(horizonLength/2.0 - 1.0, horizonLength/2.0 - 1.0, horizonLength/2.0 - 1.0, horizonLength/2.0 - 1.0), "Pronking")
  , jumping(horizonLength, Vec4<int>(0, 0, 0, 0), Vec4<int>(3, 3, 3, 3), "Jumping")
  , galloping(horizonLength, Vec4<int>(0, 4, 7, 11), Vec4<int>(7, 7, 7, 7), "Galloping")
  , standing( horizonLength, Vec4<int>(0, 0, 0, 0),  Vec4<int>(horizonLength, horizonLength, horizonLength, horizonLength), "Standing")
  , trotRunning(horizonLength, Vec4<int>(0, 7, 7, 0), Vec4<int>(6, 6, 6, 6), "Trot Running")
  , walking(horizonLength, Vec4<int>(0, horizonLength/2.0, horizonLength/4.0, 3.0*horizonLength/4.0),
            Vec4<int>(3.0*horizonLength/4.0,3.0*horizonLength/4.0,3.0*horizonLength/4.0,3.0*horizonLength/4.0), "Walking")
  , walking2(horizonLength, Vec4<int>(0, 5, 5, 0),
             Vec4<int>(5, 5, 5, 5), "Walking2")
  , pacing(horizonLength, Vec4<int>(7, 0, 7, 0), Vec4<int>(7, 7, 7, 7), "Pacing")
  , aio(horizonLength, Vec4<int>(0, horizonLength/2.0, horizonLength/2.0, 0), Vec4<int>(horizonLength, horizonLength, horizonLength, horizonLength), "aio")
  , points_(nullptr)
{
  dtMPC = dt * iterationsBetweenMPC;  // 0.03
  default_iterations_between_mpc = iterationsBetweenMPC;
  printf("[Convex MPC] dt: %.3f iterations: %d, dtMPC: %.3f\n", dt,
         iterationsBetweenMPC, dtMPC);  // 0.002, 15, 0.03
  // setup_problem(dtMPC, horizonLength, 0.4, 120);
  // setup_problem(dtMPC, horizonLength, 0.4, 650); // DH
  rpy_comp[0] = 0;
  rpy_comp[1] = 0;
  rpy_comp[2] = 0;
  rpy_int[0] = 0;
  rpy_int[1] = 0;
  rpy_int[2] = 0;

  for (int i = 0; i < 4; i++)
    firstSwing[i] = true;

  initSparseMPC();

  pBody_des.setZero();
  vBody_des.setZero();
  aBody_des.setZero();
  for (int i = 0; i < 4; i++)
    f_ff[i].setZero();

  /*
   * Параметры регуляторов по умолчанию.
   * Инициализируются в конструкторе, изменяются методом setPDcoefs
   * */
  Kp << 700, 0, 0, 0, 700, 0, 0, 0, 500;
  Kp_stance = 0.0 * Kp;

  Kd << 30, 0, 0, 0, 30, 0, 0, 0, 30;
  Kd_stance  << 2.5, 0, 0, 0, 2.5, 0, 0, 0, 2.5;
  step_height_ = 0.09;
  pf_add_x_ = 0;
  pf_add_y_ = 0;
}


void ConvexMPCLocomotion::initialize()
{
  for (int i = 0; i < 4; i++)
    firstSwing[i] = true;
  firstRun = true;
}

void ConvexMPCLocomotion::recompute_timing(int iterations_per_mpc)
{
  iterationsBetweenMPC = iterations_per_mpc;
  dtMPC = dt * iterations_per_mpc;
}

// Инициализация команды управления
void ConvexMPCLocomotion::_SetupCommand(StateEstimatorContainer<float>& _stateEstimator,
    std::vector<double> gamepadCommand)
{
  // Значение от mini cheetah. Для изменения - метод setBodyHeight
  _body_height = 0.29;

  float x_vel_cmd, y_vel_cmd, yaw_vel_cmd;
  float x_filter(0.01), y_filter(0.006), yaw_filter(0.03);

  x_vel_cmd = gamepadCommand[0];
  y_vel_cmd = gamepadCommand[1];
  yaw_vel_cmd = gamepadCommand[2];

  _x_vel_des = _x_vel_des * (1 - x_filter) + x_vel_cmd * x_filter;  //Фильтрация
  _y_vel_des = _y_vel_des * (1 - y_filter) + y_vel_cmd * y_filter;
  _yaw_turn_rate = _yaw_turn_rate * (1 - yaw_filter) + yaw_vel_cmd * yaw_filter;

  // Насыщение _x_vel_des
  if(_x_vel_des > 2.0)
    _x_vel_des = 2.0;
  else if(_x_vel_des < -1.0)
    _x_vel_des = -1.0;

  // Насыщение _y_vel_des
  if(_y_vel_des > 0.6)
    _y_vel_des = 0.6;
  else if(_y_vel_des < -0.6)
    _y_vel_des = -0.6;

  // Интегрирование скорости. Желаемый угол рысканья для оценки положения
  _yaw_des = _stateEstimator.getResult().rpy[2] +
      dt * _yaw_turn_rate;
  _roll_des = 0.;
  _pitch_des = 0.;

  // Если большая ошибка по углу рыскания, сбрасываем
  if((abs(_stateEstimator.getResult().rpy[2] - _yaw_des_true) > 5.0)){
    // _yaw_des_true = 3.14 * _stateEstimator.getResult().rpy[2] / abs(_stateEstimator.getResult().rpy[2]);
    _yaw_des_true = _stateEstimator.getResult().rpy[2];
  }
  _yaw_des_true =_yaw_des_true + dt * _yaw_turn_rate;
}

template <>
void ConvexMPCLocomotion::run(Quadruped<float>& _quadruped,
                              LegController<float>& _legController,
                              StateEstimatorContainer<float>& _stateEstimator,
                              DesiredStateCommand<float>& _desiredStateCommand,
                              std::vector<double> gamepadCommand,
                              int gaitType, int robotMode) {
  bool omniMode = false;
  // Command Setup
  _SetupCommand(_stateEstimator, gamepadCommand);

  gaitNumber = gaitType;

  if (gaitNumber >= 20) {
    gaitNumber -= 20;
    omniMode = true;
  }

  // Получаем результат наблюдателей
  auto& seResult = _stateEstimator.getResult();

  // Переход в режим standing
  if (((gaitNumber == 4) && current_gait != 4) || firstRun) {
    stand_traj[0] = seResult.position[0];
    stand_traj[1] = seResult.position[1];
    stand_traj[2] = 0.21;
    stand_traj[3] = 0;
    stand_traj[4] = 0;
    stand_traj[5] = seResult.rpy[2];
    world_position_desired[0] = stand_traj[0];
    world_position_desired[1] = stand_traj[1];
  }

  // pick gait
  Gait* gait = &trotting;
  if(robotMode == 0)
  {
    if (gaitNumber == 1)
      gait = &bounding;
    else if (gaitNumber == 2)
      gait = &pronking;
    // else if(gaitNumber == 3)
    //   gait = &random;
    else if (gaitNumber == 4)
      gait = &standing;
    else if (gaitNumber == 5)
      gait = &trotRunning;
    // else if(gaitNumber == 6)
    //   gait = &random2;
    else if (gaitNumber == 7)
      gait = &galloping;
    else if (gaitNumber == 8)
      gait = &pacing;
    else if (gaitNumber == 9)
      gait = &trotting;
    else if (gaitNumber == 10)
      gait = &walking;
    else if (gaitNumber == 11)
      gait = &walking2;
  }
  // "Плаваная ходьба" от китайцев
  else if(robotMode == 1)
  {
    int h = 10;
    double vBody = sqrt(_x_vel_des*_x_vel_des)+(_y_vel_des*_y_vel_des);
    gait = &aio;
    gaitNumber = 9;  // Trotting
    if(gait->getCurrentGaitPhase() == 0)
    {
      if(vBody < 0.002)
      {
        if(abs(_yaw_turn_rate) < 0.01)
        {
          gaitNumber = 4;  // Standing
          if(gait->getGaitHorizon() != h)
          {
            iterationCounter = 0;
          }
          gait->setGaitParam(h, Vec4<int>(0, 0, 0, 0), Vec4<int>(h, h, h, h), "Standing");
        }
        else
        {
          h = 10;
          if(gait->getGaitHorizon() != h)
          {
            iterationCounter = 0;
          }
          gait->setGaitParam(h, Vec4<int>(0, h / 2, h / 2, 0),
                             Vec4<int>(h / 2, h / 2, h / 2, h / 2), "trotting");
        }
      }
      else {
        if(vBody <= 0.2)
        {
          h = 16;
          if(gait->getGaitHorizon() != h)
          {
            iterationCounter = 0;
          }
          gait->setGaitParam(h,
                             Vec4<int>(0, 1 * h / 2, 1 * h / 4, 3 * h / 4),
                             Vec4<int>(3 * h / 4, 3 * h / 4, 3 * h / 4, 3 * h / 4), "Walking");
        }
        else if(vBody > 0.2 && vBody <= 0.4)
        {
          h = 16;
          if(gait->getGaitHorizon() != h)
          {
            iterationCounter = 0;
          }
          gait->setGaitParam(h,
                             Vec4<int>(0, 1 * h / 2, h*((5.0/4.0)*vBody), h*((5.0/4.0)*vBody+(1.0/2.0))),
                             Vec4<int>(h*((-5.0/4.0)*vBody+1.0), h*((-5.0/4.0)*vBody+1.0),
                                       h*((-5.0/4.0)*vBody+1.0), h*((-5.0/4.0)*vBody+1.0)), "Walking2trotting");
        } else if(vBody > 0.4 && vBody <= 1.4)
        {
          h = 14;
          if(gait->getGaitHorizon() != h) {
            iterationCounter = 0;
          }
          gait->setGaitParam(h, Vec4<int>(0, h / 2, h / 2, 0),
                             Vec4<int>(h / 2, h / 2, h / 2, h / 2), "trotting");
        }
        else
        {
          // h = 10;
          h = -20.0*vBody+42.0;
          if(h < 10) h = 10;
          if(gait->getGaitHorizon() != h)
          {
            iterationCounter = 0;
          }
          gait->setGaitParam(h, Vec4<int>(0, h / 2, h / 2, 0),
                             Vec4<int>(h / 2, h / 2, h / 2, h / 2), "trotting");

          std::cout << vBody << " " << h << " " << h / 2 << std::endl;
        }
      }
    }
    horizonLength = h;
  }
  else {
    std::cout << "err robot mode!!!" << std::endl;
  }

  current_gait = gaitNumber;
  // Установка числа итераций походки
  gait->setIterations(iterationsBetweenMPC, iterationCounter);

  // integrate position setpoint
  // Требуемая линейная скорость в системе координат тела
  Vec3<float> v_des_robot(_x_vel_des, _y_vel_des, 0);
  // Расчетная линейная скорость в мировой системе координат
  Vec3<float> v_des_world = omniMode
      ? v_des_robot
      : seResult.rBody.transpose() * v_des_robot;
  // Фактическая скорость робота в мировой системе координат
  Vec3<float> v_robot = seResult.vWorld;

  // Integral-esque pitche and roll compensation
  // Интегральная компенсация крена и тангажа
  // Для того, чтобы держать тело параллельно земле во время движений
  if (fabs(v_robot[0]) > .2)  // avoid dividing by zero
  {
    rpy_int[1] += dt * (_pitch_des - seResult.rpy[1]) / v_robot[0];
  }
  if (fabs(v_robot[1]) > 0.1) {
    rpy_int[0] += dt * (_roll_des - seResult.rpy[0]) / v_robot[1];
  }

  // Насыщение углов крена и тангажа
  rpy_int[0] = fminf(fmaxf(rpy_int[0], -.25), .25);  //-0.25~0.25
  rpy_int[1] = fminf(fmaxf(rpy_int[1], -.25), .25);  //-0.25~0.25
  rpy_comp[1] = v_robot[0] * rpy_int[1];  // compensation
  if (gaitNumber != 2)   // turn off for pronking
    rpy_comp[0] = v_robot[1] * rpy_int[0];

  // Определение положения стопы в мировой системе координат
  // Координаты тела + матрица поворота тела *
  // (координаты hip + декартовы координаты лапы)
  for (int leg = 0; leg < 4; leg++) {
    pFoot[leg] = seResult.position +
        seResult.rBody.transpose() *
        (_quadruped.getHipLocation(leg) + _legController.datas[leg].p);
  }

  // Для походок расчет желаемых координат интегрированием скорости
  if (gait != &standing)
  {
    world_position_desired +=
        dt * Vec3<float>(v_des_world[0], v_des_world[1], 0);
  }

  // some first time initialization
  if (firstRun)
  {
    world_position_desired[0] = seResult.position[0];
    world_position_desired[1] = seResult.position[1];
    world_position_desired[2] = seResult.rpy[2];

    for (int leg = 0; leg < 4; leg++)  // Swing траектория
    {
      footSwingTrajectories[leg].setHeight(0.09);
      footSwingTrajectories[leg].setInitialPosition(pFoot[leg]);  // set p0
      footSwingTrajectories[leg].setFinalPosition(pFoot[leg]);    // set pf
    }
    firstRun = false;
  }

  // foot placement
  for (int leg = 0; leg < 4; leg++) {
    swingTimes[leg] = gait->getCurrentSwingTime( dtMPC, leg);
  }

  float side_sign[4] = {-1, 1, -1, 1};
  float interleave_y[4] = {-0.08, 0.08, 0.02, -0.02};
  // float interleave_gain = -0.13;
  float interleave_gain = -0.2;
  // float v_abs = std::fabs(seResult.vBody[0]);
  float v_abs = std::fabs(v_des_robot[0]);
  for (int leg = 0; leg < 4; leg++) {
    if (firstSwing[leg])
    {
      swingTimeRemaining[leg] = swingTimes[leg];
    }
    else
    {
      swingTimeRemaining[leg] -= dt;
    }

    footSwingTrajectories[leg].setHeight(step_height_);
    Vec3<float> offset(0, side_sign[leg] * _quadruped._abadLinkLength, 0);

    // [китайский] Получить координаты hip в системе координат тела
    Vec3<float> pRobotFrame = (_quadruped.getHipLocation(leg) +
                               offset);
    pRobotFrame[1] += interleave_y[leg] * v_abs * interleave_gain;
    float stance_time =
        gait->getCurrentStanceTime(dtMPC, leg);

    // [китайский] При вращении пересчитываются координаты hip в системе координат тела
    Vec3<float> pYawCorrected =
        coordinateRotation(CoordinateAxis::Z,
                           -_yaw_turn_rate * stance_time / 2) * pRobotFrame;

    Vec3<float> des_vel;
    des_vel[0] = _x_vel_des;
    des_vel[1] = _y_vel_des;
    des_vel[2] = 0.0;

    // Расчет координат точки приземления лапы
    auto Pf = computePf(des_vel, stance_time, seResult, pYawCorrected, v_des_world, leg);
    if ( canPlace(Pf * 2.0, *points_, 0.01) )
    {
      // Точка приземления для ноги в мировой СК
      footSwingTrajectories[leg].setFinalPosition(Pf);
    }
    else
    {
      std::cout << "Cant place " << std::endl;
      if ( (std::abs(gamepadCommand.at(0)) < 0.1) ||
           (std::abs(gamepadCommand.at(1)) < 0.1) ||
           (std::abs(gamepadCommand.at(2)) < 0.1)  )
      {
        std::cout << "Stop " << std::endl;
        break;
      }
      std::cout << "Decreasing body speed " << std::endl;
      gamepadCommand.at(0) = gamepadCommand.at(0)/2.0;
      gamepadCommand.at(1) = gamepadCommand.at(1)/2.0;
      gamepadCommand.at(2) = gamepadCommand.at(2)/2.0;

      // Запускаем с обновленной скоростью
      this->run(_quadruped,
                _legController,
                _stateEstimator,
                _desiredStateCommand,
                gamepadCommand,
                gaitType, robotMode);
      return;
    }

#ifdef DEBUG_PF
    if (leg == 1) {
      std::cout << "pf0 = " << (seResult.rBody*(Pf- seResult.position)).transpose() << " " << std::endl;
      std::cout << pfx_rel << " " << pfy_rel << std::endl;
      std::cout << 0.5f * sqrt(seResult.position[2] / 9.81f) * (seResult.vWorld[1] * _yaw_turn_rate) << " "
                                                                                                     << (0.5f * sqrt(seResult.position[2] / 9.81f)) * (-seResult.vWorld[0] * _yaw_turn_rate) << std::endl;
      std::cout << (0.5f * seResult.position[2] / 9.81f) * (seResult.vWorld[0] * _yaw_turn_rate) << std::endl;
      std::cout << _yaw_turn_rate << std::endl;
    }
#endif
  }

  // calc gait
  iterationCounter++;

  Vec4<float> contactStates = gait->getContactState();
  Vec4<float> swingStates = gait->getSwingState();
  int* mpcTable = gait->getMpcTable();
  updateMPCIfNeeded(mpcTable, _stateEstimator, omniMode);

  //  StateEstimator* se = hw_i->state_estimator;
  Vec4<float> se_contactState(0, 0, 0, 0);

  bool use_wbc = false;

  for (int leg = 0; leg < 4; leg++) {
    float contactState = contactStates[leg];
    float swingState = swingStates[leg];
    /////////////////////////
    ///       SWING       ///
    /////////////////////////
    if (swingState > 0)  // foot is in swing
    {
      if (firstSwing[leg]) {
        firstSwing[leg] = false;
        footSwingTrajectories[leg].setInitialPosition(pFoot[leg]);
      }
      //      footSwingTrajectories[0].print();

      footSwingTrajectories[leg].computeSwingTrajectoryBezier(
            swingState, swingTimes[leg]);

      Vec3<float> pDesFootWorld = footSwingTrajectories[leg].getPosition();
      Vec3<float> vDesFootWorld = footSwingTrajectories[leg].getVelocity();

      Vec3<float> pDesLeg = seResult.rBody * (pDesFootWorld -
           seResult.position) - _quadruped.getHipLocation(leg);
      Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);

      // Update for WBC
      pFoot_des[leg] = pDesFootWorld;
      vFoot_des[leg] = vDesFootWorld;
      aFoot_des[leg] = footSwingTrajectories[leg].getAcceleration();

      if (!use_wbc)
      {
        // Update leg control command regardless of the usage of WBIC
        _legController.commands[leg].pDes = pDesLeg;
        _legController.commands[leg].vDes = vDesLeg;

        _legController.commands[leg].kpCartesian = Kp;
        _legController.commands[leg].kdCartesian = Kd;
      }
    }
    /////////////////////////
    ///       STANCE       ///
    /////////////////////////
    else  // foot is in stance
    {
      firstSwing[leg] = true;

      Vec3<float> pDesFootWorld = footSwingTrajectories[leg].getPosition();
      Vec3<float> vDesFootWorld = footSwingTrajectories[leg].getVelocity();
      Vec3<float> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) -
          _quadruped.getHipLocation(leg);
      Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);

      // Тут нет WBC )))
      if (!use_wbc)
      {
        _legController.commands[leg].pDes = pDesLeg;
        _legController.commands[leg].vDes = vDesLeg;
        _legController.commands[leg].forceFeedForward = f_ff[leg];

        _legController.commands[leg].kpCartesian = 0. * Kp_stance;
        _legController.commands[leg].kdCartesian = Kd_stance;
        _legController.commands[leg].kdJoint = KdJ;
        _legController.commands[leg].kpJoint = KpJ;

      }
      else
      {  // Stance leg damping
        _legController.commands[leg].pDes = pDesLeg;
        _legController.commands[leg].vDes = vDesLeg;
        _legController.commands[leg].kpCartesian = 0. * Kp_stance;
        _legController.commands[leg].kdCartesian = Kd_stance;
      }
      se_contactState[leg] = contactState;

      // Update for WBC
      // Fr_des[foot] = -f_ff[foot];
    } // end stance
  }
  // se->set_contact_state(se_contactState); todo removed
  _stateEstimator.setContactPhase(se_contactState);

  // Update For WBC
  pBody_des[0] = world_position_desired[0];
  pBody_des[1] = world_position_desired[1];
  pBody_des[2] = _body_height;

  vBody_des[0] = v_des_world[0];
  vBody_des[1] = v_des_world[1];
  vBody_des[2] = 0.;

  aBody_des.setZero();

  pBody_RPY_des[0] = 0.;
  pBody_RPY_des[1] = 0.;
  pBody_RPY_des[2] = _yaw_des;

  vBody_Ori_des[0] = 0.;
  vBody_Ori_des[1] = 0.;
  vBody_Ori_des[2] = _yaw_turn_rate;

  // contact_state = gait->getContactState();
  contact_state = gait->getContactState();
  // END of WBC Update
}

void ConvexMPCLocomotion::updateMPCIfNeeded(
    int* mpcTable, StateEstimatorContainer<float>& _stateEstimator,
    bool omniMode) {
  // iterationsBetweenMPC = 30;
  if ((iterationCounter % iterationsBetweenMPC) == 0) {
    auto seResult = _stateEstimator.getResult();
    float* p = seResult.position.data();

    Vec3<float> v_des_robot(_x_vel_des, _y_vel_des, 0);
    Vec3<float> v_des_world =
        omniMode ? v_des_robot : seResult.rBody.transpose() * v_des_robot;
    // float trajInitial[12] = {0,0,0, 0,0,.25, 0,0,0,0,0,0};

    // printf("Position error: %.3f, integral %.3f\n", pxy_err[0],
    // x_comp_integral);

    if (current_gait == 4) {
      float trajInitial[12] = {
        _roll_des,
        _pitch_des /*-hw_i->state_estimator->se_ground_pitch*/,
        (float)stand_traj[5] /*+(float)stateCommand->data.stateDes[11]*/,
        (float)stand_traj[0] /*+(float)fsm->main_control_settings.p_des[0]*/,
        (float)stand_traj[1] /*+(float)fsm->main_control_settings.p_des[1]*/,
        (float)_body_height /*fsm->main_control_settings.p_des[2]*/,
        0,
        0,
        0,
        0,
        0,
        0};

      for (int i = 0; i < horizonLength; i++)
        for (int j = 0; j < 12; j++) trajAll[12 * i + j] = trajInitial[j];
    }

    else {
      const float max_pos_error = .1;
      float xStart = world_position_desired[0];
      float yStart = world_position_desired[1];

      if (xStart - p[0] > max_pos_error) xStart = p[0] + 0.1;
      if (p[0] - xStart > max_pos_error) xStart = p[0] - 0.1;

      if (yStart - p[1] > max_pos_error) yStart = p[1] + 0.1;
      if (p[1] - yStart > max_pos_error) yStart = p[1] - 0.1;

      world_position_desired[0] = xStart;
      world_position_desired[1] = yStart;

      float trajInitial[12] = {(float)rpy_comp[0],  // 0
                               (float)rpy_comp[1],  // 1
                               _yaw_des_true,            // 2
                               // yawStart,    // 2
                               xStart,               // 3
                               yStart,               // 4
                               (float)_body_height,  // 5
                               0,                    // 6
                               0,                    // 7
                               _yaw_turn_rate,       // 8
                               v_des_world[0],       // 9
                               v_des_world[1],       // 10
                               0};                   // 11

      for (int i = 0; i < horizonLength; i++) {
        for (int j = 0; j < 12; j++) trajAll[12 * i + j] = trajInitial[j];

        if (i == 0)  // start at current position  TODO consider not doing this
        {
          // trajAll[2] = seResult.rpy[2];
          trajAll[2] = _yaw_des_true;
        } else {
          trajAll[12 * i + 3] =
              trajAll[12 * (i - 1) + 3] + dtMPC * v_des_world[0];
          trajAll[12 * i + 4] =
              trajAll[12 * (i - 1) + 4] + dtMPC * v_des_world[1];
          trajAll[12 * i + 2] =
              trajAll[12 * (i - 1) + 2] + dtMPC * _yaw_turn_rate;
        }
      }
    }

    Timer solveTimer;

    int cmpc_use_sparse = 0.0;

    if (cmpc_use_sparse > 0.5) {
      solveSparseMPC(mpcTable, _stateEstimator);
    } else {
      solveDenseMPC(mpcTable, _stateEstimator);
    }
    // printf("TOTAL SOLVE TIME: %.3f\n", solveTimer.getMs());
  }
}

void ConvexMPCLocomotion::solveDenseMPC(
    int* mpcTable, StateEstimatorContainer<float>& _stateEstimator) {
  auto seResult = _stateEstimator.getResult();

  // float Q[12] = {0.25, 0.25, 10, 2, 2, 20, 0, 0, 0.3, 0.2, 0.2, 0.2};

  float Q[12] = {2.5, 2.5, 10, 50, 50, 100, 0, 0, 0.5, 0.2, 0.2, 0.1};
  // float Q[12] = {0.25, 0.25, 10, 2, 2, 50, 0, 0, 0.3, 0.2, 0.2, 0.1}; // mini cheetah

  // float Q[12] = {0.25, 0.25, 10, 2, 2, 40, 0, 0, 0.3, 0.2, 0.2, 0.2};
  float yaw = seResult.rpy[2];
  float* weights = Q;
  float alpha = 4e-5;  // make setting eventually
  // float alpha = 4e-7; // make setting eventually: DH
  float* p = seResult.position.data();
  float* v = seResult.vWorld.data();
  float* w = seResult.omegaWorld.data();
  float* q = seResult.orientation.data();

  float r[12];
  for (int i = 0; i < 12; i++)
    r[i] = pFoot[i % 4][i / 4] - seResult.position[i / 4];

  // printf("current posistion: %3.f %.3f %.3f\n", p[0], p[1], p[2]);

  if (alpha > 1e-4) {
    std::cout << "Alpha was set too high (" << alpha << ") adjust to 1e-5\n";
    alpha = 1e-5;
  }

  Vec3<float> pxy_act(p[0], p[1], 0);
  Vec3<float> pxy_des(world_position_desired[0], world_position_desired[1], 0);
  // Vec3<float> pxy_err = pxy_act - pxy_des;
  float pz_err = p[2] - _body_height;
  Vec3<float> vxy(seResult.vWorld[0], seResult.vWorld[1], 0);

  Timer t1;
  dtMPC = dt * iterationsBetweenMPC;
  setup_problem(dtMPC, horizonLength, 0.4, 120);
  // setup_problem(dtMPC,horizonLength,0.4,650); //DH
  update_x_drag(x_comp_integral);

  float cmpc_x_drag = 3.0;

  if (vxy[0] > 0.3 || vxy[0] < -0.3) {
    // x_comp_integral += _parameters->cmpc_x_drag * pxy_err[0] * dtMPC /
    // vxy[0];
    x_comp_integral += cmpc_x_drag * pz_err * dtMPC / vxy[0];
  }

  // printf("pz err: %.3f, pz int: %.3f\n", pz_err, x_comp_integral);

  int jcqp_max_iter = 10000;
  double jcqp_rho = 0.0000001;
  double jcqp_sigma = 0.00000001;
  double jcqp_alpha = 1.5;
  double jcqp_terminate = 0.1;
  double use_jcqp = 0.0;

  update_solver_settings(jcqp_max_iter, jcqp_rho, jcqp_sigma, jcqp_alpha,
                         jcqp_terminate, use_jcqp);
  // t1.stopPrint("Setup MPC");

  Timer t2;
  // cout << "dtMPC: " << dtMPC << "\n";
  // std::cout << "pFoot = " << std::endl;
  // for(int i=0; i<4; i++) {
  //   for(int j=0; j<3; j++) {
  //     std::cout << pFoot[i][j] << " ";
  //   }
  // }
  // std::cout << std::endl;
  update_problem_data_floats(p, v, q, w, r, yaw, weights, trajAll, alpha,
                             mpcTable);

  // std::cout << "the value is " << mpcTable << std::endl;

  // t2.stopPrint("Run MPC");
  // printf("MPC Solve time %f ms\n", t2.getMs());

  for (int leg = 0; leg < 4; leg++) {
    Vec3<float> f;
    for (int axis = 0; axis < 3; axis++) f[axis] = get_solution(leg * 3 + axis);

    // if(myflags < 50){
    // printf("[%d] %7.3f %7.3f %7.3f\n", leg, f[0], f[1], f[2]);
    // }

    f_ff[leg] = -seResult.rBody * f;
    // std::cout << "Foot " << leg << " force: " << f.transpose() << "\n";
    // std::cout << "Foot " << leg << " force: " << f_ff[leg].transpose() <<
    // "\n"; Update for WBC
    Fr_des[leg] = f;
  }
  myflags = myflags + 1;
}

void ConvexMPCLocomotion::solveSparseMPC(
    int* mpcTable, StateEstimatorContainer<float>& _stateEstimator) {
  // X0, contact trajectory, state trajectory, feet, get result!
  (void)mpcTable;
  auto seResult = _stateEstimator.getResult();

  std::vector<ContactState> contactStates;
  for (int i = 0; i < horizonLength; i++) {
    contactStates.emplace_back(mpcTable[i * 4 + 0], mpcTable[i * 4 + 1],
        mpcTable[i * 4 + 2], mpcTable[i * 4 + 3]);
  }

  for (int i = 0; i < horizonLength; i++) {
    for (u32 j = 0; j < 12; j++) {
      _sparseTrajectory[i][j] = trajAll[i * 12 + j];
    }
  }

  Vec12<float> feet;
  for (u32 foot = 0; foot < 4; foot++) {
    for (u32 axis = 0; axis < 3; axis++) {
      feet[foot * 3 + axis] = pFoot[foot][axis] - seResult.position[axis];
    }
  }

  _sparseCMPC.setX0(seResult.position, seResult.vWorld, seResult.orientation,
                    seResult.omegaWorld);
  _sparseCMPC.setContactTrajectory(contactStates.data(), contactStates.size());
  _sparseCMPC.setStateTrajectory(_sparseTrajectory);
  _sparseCMPC.setFeet(feet);
  _sparseCMPC.run();

  Vec12<float> resultForce = _sparseCMPC.getResult();

  for (u32 foot = 0; foot < 4; foot++) {
    Vec3<float> force(resultForce[foot * 3], resultForce[foot * 3 + 1],
        resultForce[foot * 3 + 2]);
    printf("[%d] %7.3f %7.3f %7.3f\n", foot, force[0], force[1], force[2]);
    f_ff[foot] = -seResult.rBody * force;
    Fr_des[foot] = force;
  }
}

void ConvexMPCLocomotion::initSparseMPC() {
  Mat3<double> baseInertia;
  // Параметры инерции от mimi cheetah
  baseInertia << 0.07, 0, 0,
              0, 0.26, 0,
              0, 0, 0.242;
  double mass = 13.741;
  // Ограничение по максимальной силе (проекция на ось Z)
  double maxForce = 120;

  std::vector<double> dtTraj;
  for (int i = 0; i < horizonLength; i++) {
    dtTraj.push_back(dtMPC);
  }

  Vec12<double> weights;
  weights << 0.25, 0.25, 10, 2, 2, 20, 0, 0, 0.3, 0.2, 0.2, 0.2;
  // weights << 0,0,0,1,1,10,0,0,0,0.2,0.2,0;

  _sparseCMPC.setRobotParameters(baseInertia, mass, maxForce);
  _sparseCMPC.setFriction(1.0);
  _sparseCMPC.setWeights(weights, 4e-5);
  _sparseCMPC.setDtTrajectory(dtTraj);

  _sparseTrajectory.resize(horizonLength);
}

void ConvexMPCLocomotion::setBodyHeight(float h, std::string type)
{
  if (type.empty())
  {
    std::cout << " [ConvexMPCLocomotion]: Get empty body height type,"
                 " Should be : (default, run or jump) " << std::endl;
  }
  if (type == "default")
    _body_height = h;
  if (type == "run")
    _body_height_running = h;
  if (type == "jump")
    _body_height_jumping = h;
}


void ConvexMPCLocomotion::setTrotDuration(int d)
{
  trot_duration_=d;
  trotting.update(Vec4<int>(trot_duration_, trot_duration_, trot_duration_, trot_duration_), horizonLength);
}


Vec3<float> ConvexMPCLocomotion::computePf(Vec3<float>& des_vel, float& stance_time,
                                           const StateEstimate<float>& seResult,
                                           Vec3<float>& pYawCorrected,
                                           Vec3<float>& v_des_world, int leg)
{
  // [китайский] Координаты бедра в мировой системе координат оцениваются по равномерному
  // движению тела на оставшемся промежутке времени
  Vec3<float> Pf = seResult.position + seResult.rBody.transpose() *
      (pYawCorrected + des_vel * swingTimeRemaining[leg]);
  float p_rel_max = 0.3f;

  // Using the estimated velocity is correct
  // Vec3<float> des_vel_world = seResult.rBody.transpose() * des_vel;
  float pfx_rel = seResult.vWorld[0] * (.5 + 0.0) * stance_time +
      .03f*(seResult.vWorld[0]-v_des_world[0]) +
      (0.5f*seResult.position[2]/9.81f) * (seResult.vWorld[1]*_yaw_turn_rate);

  float pfy_rel = seResult.vWorld[1] * .5 * stance_time * dtMPC +
      .03f*(seResult.vWorld[1]-v_des_world[1]) +
      (0.5f*seResult.position[2]/9.81f) * (-seResult.vWorld[0]*_yaw_turn_rate);

  pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
  pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);
  Pf[0] += pfx_rel;
  Pf[1] += pfy_rel;
  Pf[2] = 0.0;
  return Pf;
}


double calcDistance(Vec3<float> const& p1, Vec3<double> const& p2)
{
  return sqrt(pow(p1.x() - p2.x(), 2) +
              pow(p1.y() - p2.y(), 2) +
              pow(p1.z() - p2.z(), 2)  );
}

const Eigen::Vector3d findClosestPoint(const Vec3<float>& point,
                                       boost::circular_buffer<Eigen::Vector3d>& buffer)
{
  double dist = 9999;
  double cur_dist = 0;
  Eigen::Vector3d ans = buffer.front();
  for (auto it:buffer)
  {
    cur_dist = calcDistance(point, it);
    if (cur_dist < dist)
    {
      dist = cur_dist;
      ans = it;
    }
  }
  return ans;
}

bool canPlace(const Vec3<float>& point, boost::circular_buffer<Eigen::Vector3d>& buffer, double thresh)
{
  auto closest = findClosestPoint(point, buffer);
//  std::cout << "distance = " << calcDistance(point, closest) << std::endl;
  if (calcDistance(point, closest) >= thresh)
    return true;
  return false;
}
