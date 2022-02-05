/*!
 * @file FootSwingTrajectory.h
 * @brief Utility to generate foot swing trajectories.
 *
 * Currently uses Bezier curves like Cheetah 3 does
 */

#ifndef CHEETAH_SOFTWARE_FOOTSWINGTRAJECTORY_H
#define CHEETAH_SOFTWARE_FOOTSWINGTRAJECTORY_H

#include "Utilities/cppTypes.h"
#include "iostream"

/*!
 * A foot swing trajectory for a single foot
 */
template<typename T>
class FootSwingTrajectory {
public:

  /*!
   * Construct a new foot swing trajectory with everything set to zero
   */
  FootSwingTrajectory() {
    _p0.setZero();
    _pf.setZero();
    _p.setZero();
    _v.setZero();
    _a.setZero();
    _height = 0;
  }

  /*!
   * Set the starting location of the foot
   * @param p0 : the initial foot position
   */
  void setInitialPosition(Vec3<T> p0) {
    _p0 = p0;
    _p = p0;    // solve init stand pDesFootWorld=0 problem
    _step_len = _pf - _p0;
  }

  /*!
   * Set the desired final position of the foot
   * @param pf : the final foot posiiton
   */
  void setFinalPosition(Vec3<T> pf) {
    _pf = pf;
  }

  /*!
   * Set the maximum height of the swing
   * @param h : the maximum height of the swing, achieved halfway through the swing
   */
  void setHeight(T h) {
    _height = h;
  }

  void computeSwingTrajectoryBezier(T phase, T swingTime);

  /*!
   * Get the foot position at the current point along the swing
   * @return : the foot position
   */
  Vec3<T> getPosition() {
    return _p;
  }

  /*!
   * Get the desired final  foot position
   */
  Vec3<T> getFinalPosition() {
    return _pf;
  }


  /*!
   * Get the initial  foot position
   */
  Vec3<T> getStartPosition() {
    return _p0;
  }

  /*!
   * Get the step length as a vector (x y z)
   */
  Vec3<T> getStepLength() {
    return _step_len;
  }

  /*!
   * Get the foot velocity at the current point along the swing
   * @return : the foot velocity
   */
  Vec3<T> getVelocity() {
    return _v;
  }

  /*!
   * Get the foot acceleration at the current point along the swing
   * @return : the foot acceleration
   */
  Vec3<T> getAcceleration() {
    return _a;
  }
  // Вывод дебаг информации полей
  void print()
  {
    std::cout << "p0: " << _p0 << std::endl;
    std::cout << "pf: " << _pf << std::endl;
    std::cout << "height: " << _height << std::endl;
  }


private:
  Vec3<T> _p0, _pf, _p, _v, _a, _step_len;
  T _height;
};


#endif //CHEETAH_SOFTWARE_FOOTSWINGTRAJECTORY_H
