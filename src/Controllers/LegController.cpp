/*! @file LegController.cpp
 *  @brief Common Leg Control Interface
 *
 *  Implements low-level leg control for Mini Cheetah and Cheetah 3 Robots
 *  Abstracts away the difference between the SPIne and the TI Boards
 *  All quantities are in the "leg frame" which has the same orientation as the
 * body frame, but is shifted so that 0,0,0 is at the ab/ad pivot (the "hip
 * frame").
 */

#include "Controllers/LegController.h"

#include <math.h>
#include <sys/time.h>

#include <fstream>
#include <vector>
/*!
 * Zero the leg command so the leg will not output torque
 */
template <typename T>
void LegControllerCommand<T>::zero() {
  tauFeedForward = Vec3<T>::Zero();
  forceFeedForward = Vec3<T>::Zero();
  qDes = Vec3<T>::Zero();
  qdDes = Vec3<T>::Zero();
  pDes = Vec3<T>::Zero();
  vDes = Vec3<T>::Zero();
  kpCartesian = Mat3<T>::Zero();
  kdCartesian = Mat3<T>::Zero();
  kpJoint = Mat3<T>::Zero();
  kdJoint = Mat3<T>::Zero();
}

/*!
 * Zero the leg data
 */
template <typename T>
void LegControllerData<T>::zero() {
  q = Vec3<T>::Zero();
  qd = Vec3<T>::Zero();
  p = Vec3<T>::Zero();
  v = Vec3<T>::Zero();
  J = Mat3<T>::Zero();
  tauEstimate = Vec3<T>::Zero();
}

/*!
 * Zero all leg commands.  This should be run *before* any control code, so if
 * the control code is confused and doesn't change the leg command, the legs
 * won't remember the last command.
 */
template <typename T>
void LegController<T>::zeroCommand() {
  for (auto& cmd : commands) {
    cmd.zero();
  }
  _legsEnabled = false;
}

/*!
 * Set the leg to edamp.  This overwrites all command data and generates an
 * emergency damp command using the given gain. For the mini-cheetah, the edamp
 * gain is Nm/(rad/s), and for the Cheetah 3 it is N/m. You still must call
 * updateCommand for this command to end up in the low-level command data!
 */
template <typename T>
void LegController<T>::edampCommand(RobotType robot, T gain) {
  zeroCommand();
  if (robot == RobotType::CHEETAH_3) {
    for (int leg = 0; leg < 4; leg++) {
      for (int axis = 0; axis < 3; axis++) {
        commands[leg].kdCartesian(axis, axis) = gain;
      }
    }
  } else {  // mini-cheetah
    for (int leg = 0; leg < 4; leg++) {
      for (int axis = 0; axis < 3; axis++) {
        commands[leg].kdJoint(axis, axis) = gain;
      }
    }
  }
}

/*!
 * Update the "leg data" from a SPIne board message
 */
template <typename T>
void LegController<T>::updateData(LegData* legData) {
  for (size_t leg = 0; leg < 4; leg++) {
    // q:
    datas[leg].q(0) = legData->q_abad[leg];
    datas[leg].q(1) = legData->q_hip[leg];
    datas[leg].q(2) = legData->q_knee[leg];

    // qd:
    datas[leg].qd(0) = legData->qd_abad[leg];
    datas[leg].qd(1) = legData->qd_hip[leg];
    datas[leg].qd(2) = legData->qd_knee[leg];

    // J and position
    computeLegJacobianAndPosition<T>(_quadruped, datas[leg].q, &(datas[leg].J),
                                     &(datas[leg].p), leg);

    // v linear
    datas[leg].v = datas[leg].J * datas[leg].qd;
  }
}

/*!
 * Update the "leg command" for the SPIne board message
 * Заполняет принятую структуру данными
 */
template <typename T>
void LegController<T>::updateCommand(LegCommand* legCommand)
{
  for (int leg = 0; leg < 4; leg++)
  {
    /// [tauFF] крутящий момент для лапы (прямая связь)
    Vec3<T> legTorque = commands[leg].tauFeedForward;

    /// [forceFF] значение силы на конце лапы (прямая связь)
    Vec3<T> footForce = commands[leg].forceFeedForward;

    /// [cartesian PD] коэффициенты регулятора (ПД) в декартовом пространстве
    footForce +=
        commands[leg].kpCartesian * (commands[leg].pDes - datas[leg].p);
    footForce +=
        commands[leg].kdCartesian * (commands[leg].vDes - datas[leg].v);

    /// [Torque] приведение силы на конце лапы к крутящему момменту в двигателях
    legTorque += datas[leg].J.transpose() * footForce;

    // Китайское решение ОЗК с моим добавлением qd
    computeLegIK(leg);

//      legCommand->tau_abad_ff[leg] = legTorque(0);
//      legCommand->tau_hip_ff[leg] = legTorque(1);
//      legCommand->tau_knee_ff[leg] = legTorque(2);

    legCommand->tau_abad_ff[leg] =
        commands[leg].kpJoint(0,0) * (commands[leg].qDes(0) - datas[leg].q(0)) -
        commands[leg].kdJoint(0,0) * (commands[leg].qdDes(0) - datas[leg].qd(0))
        + legTorque(0);
//    std::cout << "kpJoint : " << commands[leg].kpJoint(0,0) << " " << commands[leg].kpJoint(1,1) << " " << commands[leg].kpJoint(2,2) << std::endl;
    legCommand->tau_hip_ff[leg] =
        commands[leg].kpJoint(1,1) * (commands[leg].qDes(1) - datas[leg].q(1)) -
        commands[leg].kdJoint(1,1) * (commands[leg].qdDes(1) - datas[leg].qd(1))
        + legTorque(1);
    legCommand->tau_knee_ff[leg] =
        commands[leg].kpJoint(2,2) * (commands[leg].qDes(2) - datas[leg].q(2)) -
        commands[leg].kdJoint(2,2) * (commands[leg].qdDes(2) - datas[leg].qd(2))
        + legTorque(2);

      legCommand->flags[leg] = _legsEnabled ? 1 : 0;
  }
}

template struct LegControllerCommand<double>;
template struct LegControllerCommand<float>;

template struct LegControllerData<double>;
template struct LegControllerData<float>;

template class LegController<double>;
template class LegController<float>;

/*!
 * Compute the position of the foot and its Jacobian.  This is done in the local
 * leg coordinate system. If J/p are NULL, the calculation will be skipped.
 */
template <typename T>
void computeLegJacobianAndPosition(Quadruped<T>& quad, Vec3<T>& q, Mat3<T>* J,
                                   Vec3<T>* p, int leg) {
  T l1 = quad._abadLinkLength;
  T l2 = quad._hipLinkLength;
  T l3 = quad._kneeLinkLength;
  T l4 = quad._kneeLinkY_offset;
  T sideSign = quad.getSideSign(leg);

  T s1 = std::sin(q(0));
  T s2 = std::sin(q(1));
  T s3 = std::sin(q(2));

  T c1 = std::cos(q(0));
  T c2 = std::cos(q(1));
  T c3 = std::cos(q(2));

  T c23 = c2 * c3 - s2 * s3;
  T s23 = s2 * c3 + c2 * s3;

  if (J) {
    J->operator()(0, 0) = 0;
    J->operator()(0, 1) = l3 * c23 + l2 * c2;
    J->operator()(0, 2) = l3 * c23;
    J->operator()(1, 0) =
        l3 * c1 * c23 + l2 * c1 * c2 - (l1 + l4) * sideSign * s1;
    J->operator()(1, 1) = -l3 * s1 * s23 - l2 * s1 * s2;
    J->operator()(1, 2) = -l3 * s1 * s23;
    J->operator()(2, 0) =
        l3 * s1 * c23 + l2 * c2 * s1 + (l1 + l4) * sideSign * c1;
    J->operator()(2, 1) = l3 * c1 * s23 + l2 * c1 * s2;
    J->operator()(2, 2) = l3 * c1 * s23;
  }

  if (p) {
    p->operator()(0) = l3 * s23 + l2 * s2;
    p->operator()(1) =
        (l1 + l4) * sideSign * c1 + l3 * (s1 * c23) + l2 * c2 * s1;
    p->operator()(2) =
        (l1 + l4) * sideSign * s1 - l3 * (c1 * c23) - l2 * c1 * c2;
  }
}

template void computeLegJacobianAndPosition<double>(Quadruped<double>& quad,
                                                    Vec3<double>& q,
                                                    Mat3<double>* J,
                                                    Vec3<double>* p, int leg);
template void computeLegJacobianAndPosition<float>(Quadruped<float>& quad,
                                                   Vec3<float>& q,
                                                   Mat3<float>* J,
                                                   Vec3<float>* p, int leg);



