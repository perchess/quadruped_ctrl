#pragma once

#include "FloatingBaseModel.h"
#include "Quadruped.h"

/*!
 * Generate a Quadruped model of Unitree A1
 */
template <typename T>
Quadruped<T> buildUnitreeA1() {
  Quadruped<T> unitree;

  unitree._bodyMass = 6.0;
  unitree._bodyLength = 0.267;
  unitree._bodyWidth = 0.194;
  unitree._bodyHeight = 0.114;

  unitree._abadGearRatio = 6;
  unitree._hipGearRatio = 6;
  unitree._kneeGearRatio = 9.33;

  unitree._abadLinkLength = 0.04;
  unitree._hipLinkLength = 0.2;
  unitree._kneeLinkLength = 0.2;
  //unitree._kneeLinkLength = 0.175;
  //unitree._maxLegLength = 0.384;
  unitree._kneeLinkY_offset = 0.047;
  //unitree._kneeLinkLength = 0.20;

  unitree._maxLegLength = 0.409;


  unitree._motorTauMax = 3.f;
  unitree._batteryV = 24;
  unitree._motorKT = .05;  // this is flux linkage * pole pairs
  unitree._motorR = 0.173;
  unitree._jointDamping = .01;
  unitree._jointDryFriction = .2;
  //unitree._jointDamping = .0;
  //unitree._jointDryFriction = .0;


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
  abadRotationalInertia << 469, 58, 0.45, 58, 807, 0.95, 0.45, 0.95, 553;
  abadRotationalInertia = abadRotationalInertia * 1e-6;
  Vec3<T> abadCOM(-0.003311, 0.000635, 0.000031);  // LEFT
  SpatialInertia<T> abadInertia(0.696, abadCOM, abadRotationalInertia);

  Mat3<T> hipRotationalInertia;
  hipRotationalInertia << 5529, 245, 13, 245, 5139, 1.5, 13, 1.5, 1367;
  hipRotationalInertia = hipRotationalInertia * 1e-6;
  Vec3<T> hipCOM(-0.003237, -0.022327, -0.027326);
  SpatialInertia<T> hipInertia(1.013, hipCOM, hipRotationalInertia);

  Mat3<T> kneeRotationalInertia, kneeRotationalInertiaRotated;
  kneeRotationalInertiaRotated << 2997, 0, 0, 0, 3014, 0, 0, 0, 32;
  kneeRotationalInertiaRotated = kneeRotationalInertiaRotated * 1e-6;
  kneeRotationalInertia = RY * kneeRotationalInertiaRotated * RY.transpose();
  Vec3<T> kneeCOM(0.006435, 0, -0.107388);
  SpatialInertia<T> kneeInertia(0.166, kneeCOM, kneeRotationalInertia);

  Vec3<T> rotorCOM(0, 0, 0);
  SpatialInertia<T> rotorInertiaX(0.055, rotorCOM, rotorRotationalInertiaX);
  SpatialInertia<T> rotorInertiaY(0.055, rotorCOM, rotorRotationalInertiaY);

  Mat3<T> bodyRotationalInertia;
  bodyRotationalInertia << 15853, 0, 0, 0, 37799, 0, 0, 0, 45654;
  bodyRotationalInertia = bodyRotationalInertia * 1e-6;
  Vec3<T> bodyCOM(0, 0.0041, -0.0005);
  SpatialInertia<T> bodyInertia(unitree._bodyMass, bodyCOM,
                                bodyRotationalInertia);

  unitree._abadInertia = abadInertia;
  unitree._hipInertia = hipInertia;
  unitree._kneeInertia = kneeInertia;
  unitree._abadRotorInertia = rotorInertiaX;
  unitree._hipRotorInertia = rotorInertiaY;
  unitree._kneeRotorInertia = rotorInertiaY;
  unitree._bodyInertia = bodyInertia;

  // locations
  unitree._abadRotorLocation = Vec3<T>(0.125, 0.049, 0);
  unitree._abadLocation =
      Vec3<T>(unitree._bodyLength, unitree._bodyWidth, 0) * 0.5;
  unitree._hipLocation = Vec3<T>(0, unitree._abadLinkLength, 0);
  unitree._hipRotorLocation = Vec3<T>(0, 0.04, 0);
  unitree._kneeLocation = Vec3<T>(0, 0, -unitree._hipLinkLength);
  unitree._kneeRotorLocation = Vec3<T>(0, 0, 0);

  return unitree;
}
