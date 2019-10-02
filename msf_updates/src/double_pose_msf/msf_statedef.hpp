#ifndef MSF_STATEDEF_HPP_
#define MSF_STATEDEF_HPP_

#include <Eigen/Dense>
#include <msf_core/msf_fwds.h>
#include <boost/fusion/container.hpp>

namespace msf_updates {

/*
 * This file contains the state definition of the EKF as defined for a given set
 * of sensors / states to estimate
 */

enum StateDefinition {  // Must not manually set the enum values!
  p,
  v,
  q,
  b_w,
  b_a,
  L,
  q_wv,
  p_wv,
  q1_wv,
  p1_wv,
  q_ic,
  p_ic,
  q1_ic,
  p1_ic
};

namespace {

/***
 * Setup core state, then auxiliary state.
 */
typedef boost::fusion::vector<
    // States varying during propagation - must not change the ordering here for
    // now, CalcQ has the ordering hardcoded
    msf_core::StateVar_T<Eigen::Matrix<double, 3, 1>, p,
        msf_core::CoreStateWithPropagation>,  ///< Translation from the world frame to the IMU frame expressed in the world frame.
    msf_core::StateVar_T<Eigen::Matrix<double, 3, 1>, v,
        msf_core::CoreStateWithPropagation>,  ///< Velocity of the IMU frame expressed in the world frame.
    msf_core::StateVar_T<Eigen::Quaternion<double>, q,
        msf_core::CoreStateWithPropagation>,  ///< Rotation from the world frame to the IMU frame expressed in the world frame.
    msf_core::StateVar_T<Eigen::Matrix<double, 3, 1>, b_w,
        msf_core::CoreStateWithoutPropagation>,  ///< Gyro biases.
    msf_core::StateVar_T<Eigen::Matrix<double, 3, 1>, b_a,
        msf_core::CoreStateWithoutPropagation>,  ///< Acceleration biases.

    // States not varying during propagation.
    msf_core::StateVar_T<Eigen::Matrix<double, 1, 1>, L, msf_core::Auxiliary>,  ///< Visual scale.
    msf_core::StateVar_T<Eigen::Quaternion<double>, q_wv,
        msf_core::AuxiliaryNonTemporalDrifting>,  ///< Rotation from the world frame to the frame in which the pose is measured expressed in the world frame.
    msf_core::StateVar_T<Eigen::Matrix<double, 3, 1>, p_wv>,  ///< Translation from the world frame to the frame in which the pose is measured expressed in the world frame.
    msf_core::StateVar_T<Eigen::Quaternion<double>, q1_wv,
        msf_core::AuxiliaryNonTemporalDrifting>,  ///< Rotation from the world frame to the frame in which the second pose is measured expressed in the world frame.
    msf_core::StateVar_T<Eigen::Matrix<double, 3, 1>, p1_wv>,  ///< Translation from the world frame to the frame in which the second pose is measured expressed in the world frame.
    msf_core::StateVar_T<Eigen::Quaternion<double>, q_ic>,  ///< Rotation from the IMU frame to the camera frame expressed in the IMU frame.
    msf_core::StateVar_T<Eigen::Matrix<double, 3, 1>, p_ic>,  ///< Translation from the IMU frame to the camera frame expressed in the IMU frame.
    msf_core::StateVar_T<Eigen::Quaternion<double>, q1_ic>,  ///< Rotation from the IMU frame to the camera frame expressed in the IMU frame.
    msf_core::StateVar_T<Eigen::Matrix<double, 3, 1>, p1_ic>  ///< Translation from the IMU frame to the camera frame expressed in the IMU frame.
> fullState_T;
}

typedef msf_core::GenericState_T<fullState_T, StateDefinition> EKFState;  ///< The state we want to use in this EKF.
typedef shared_ptr<EKFState> EKFStatePtr;
typedef shared_ptr<const EKFState> EKFStateConstPtr;

}

#include <msf_updates/static_ordering_assertions.h> //DO NOT REMOVE THIS
#endif  // MSF_STATEDEF_HPP_
