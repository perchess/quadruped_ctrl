#pragma once

#include "FloatingBaseModel.h"
#include "Quadruped.h"

/*!
 * Generate a Quadruped model of Unitree A1
 */
template <typename T>
Quadruped<T> buildUnitreeA1() {
  Quadruped<T> unitree_a1;
  unitree_a1._robotType = RobotType::MINI_CHEETAH;

  unitree_a1._bodyMass = 6.0;
  unitree_a1._bodyLength = 0.1805 * 2;
  unitree_a1._bodyWidth = 0.047 * 2;
  unitree_a1._bodyHeight = 0.114;

  unitree_a1._abadGearRatio = 6;
  unitree_a1._hipGearRatio = 6;
  unitree_a1._kneeGearRatio = 9.33;

  unitree_a1._abadLinkLength = 0.0838;//+
  unitree_a1._hipLinkLength = 0.2;//+
  //unitree_a1._kneeLinkLength = 0.175;
  //unitree_a1._maxLegLength = 0.384;
  unitree_a1._kneeLinkY_offset = 0.0028;//+
  //unitree_a1._kneeLinkLength = 0.20;
  unitree_a1._kneeLinkLength = 0.2;//+
  unitree_a1._maxLegLength = 0.409;//+


  unitree_a1._motorTauMax = 3.f;
  unitree_a1._batteryV = 24;
  unitree_a1._motorKT = .05;  // this is flux linkage * pole pairs
  unitree_a1._motorR = 0.173;
  unitree_a1._jointDamping = .01;
  unitree_a1._jointDryFriction = .2;
  //unitree_a1._jointDamping = .0;
  //unitree_a1._jointDryFriction = .0;


  // rotor inertia if the rotor is oriented so it spins around the z-axis
  Mat3<T> rotorRotationalInertiaZ;
  rotorRotationalInertiaZ << 33, 0, 0, 0, 33, 0, 0, 0, 63;
  rotorRotationalInertiaZ = 1e-6 * rotorRotationalInertiaZ;

  Mat3<T> RY = coordinateRotation<T>(CoordinateAxis::Y, M_PI / 2);
  Mat3<T> RX = coordinateRotation<T>(CoordinateAxis::X, M_PI / 2);
  Mat3<T> rotorRotationalInertiaX =
      RY * rotorRotationalInertiaZ * RY.transpose();
  Mat3<T> rotorRotationalInertiaY =
      RX * rotorRotationalInertiaZ * RX.transpose();

  // spatial inertias
  Mat3<T> abadRotationalInertia;
  abadRotationalInertia << 381, 58, 0.45, 58, 560, 0.95, 0.45, 0.95, 444;
  abadRotationalInertia = abadRotationalInertia * 1e-6;
  Vec3<T> abadCOM(0, 0.036, 0);  // LEFT
  SpatialInertia<T> abadInertia(0.54, abadCOM, abadRotationalInertia);

  Mat3<T> hipRotationalInertia;
  hipRotationalInertia << 1983, 245, 13, 245, 2103, 1.5, 13, 1.5, 408;
  hipRotationalInertia = hipRotationalInertia * 1e-6;
  Vec3<T> hipCOM(0, 0.016, -0.02);
  SpatialInertia<T> hipInertia(0.634, hipCOM, hipRotationalInertia);

  Mat3<T> kneeRotationalInertia, kneeRotationalInertiaRotated;
  kneeRotationalInertiaRotated << 6, 0, 0, 0, 248, 0, 0, 0, 245;
  kneeRotationalInertiaRotated = kneeRotationalInertiaRotated * 1e-6;
  kneeRotationalInertia = RY * kneeRotationalInertiaRotated * RY.transpose();
  Vec3<T> kneeCOM(0, 0, -0.061);
  SpatialInertia<T> kneeInertia(0.064, kneeCOM, kneeRotationalInertia);

  Vec3<T> rotorCOM(0, 0, 0);
  SpatialInertia<T> rotorInertiaX(0.055, rotorCOM, rotorRotationalInertiaX);
  SpatialInertia<T> rotorInertiaY(0.055, rotorCOM, rotorRotationalInertiaY);

  Mat3<T> bodyRotationalInertia;
  bodyRotationalInertia << 11253, 0, 0, 0, 36203, 0, 0, 0, 42673;
  bodyRotationalInertia = bodyRotationalInertia * 1e-6;
  Vec3<T> bodyCOM(0, 0, 0);
  SpatialInertia<T> bodyInertia(unitree_a1._bodyMass, bodyCOM,
                                bodyRotationalInertia);

  unitree_a1._abadInertia = abadInertia;
  unitree_a1._hipInertia = hipInertia;
  unitree_a1._kneeInertia = kneeInertia;
  unitree_a1._abadRotorInertia = rotorInertiaX;
  unitree_a1._hipRotorInertia = rotorInertiaY;
  unitree_a1._kneeRotorInertia = rotorInertiaY;
  unitree_a1._bodyInertia = bodyInertia;

  // locations
  unitree_a1._abadRotorLocation = Vec3<T>(0.125, 0.049, 0);
  unitree_a1._abadLocation =
      Vec3<T>(unitree_a1._bodyLength, unitree_a1._bodyWidth, 0) * 0.5;
  unitree_a1._hipLocation = Vec3<T>(0, unitree_a1._abadLinkLength, 0);
  unitree_a1._hipRotorLocation = Vec3<T>(0, 0.04, 0);
  unitree_a1._kneeLocation = Vec3<T>(0, 0, -unitree_a1._hipLinkLength);
  unitree_a1._kneeRotorLocation = Vec3<T>(0, 0, 0);

  return unitree_a1;
}
