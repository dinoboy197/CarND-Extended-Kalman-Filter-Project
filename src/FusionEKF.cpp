#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  // initialize Kalman filter
  ekf_ = KalmanFilter();

  // state covariance matrix P
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1000, 0,
             0, 0, 0, 1000;


  // measurement matrix
  ekf_.H_ = MatrixXd(2, 4);
  ekf_.H_ << 1, 0, 0, 0,
             0, 1, 0, 0;

  // set up noise and state transition matrices
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.F_ = MatrixXd(4, 4);
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    // Initialize the state ekf_.x_ with the first measurement.
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */

      // measurement covariance matrix for radar
      ekf_.R_ = R_radar_;
    } else {
      // measurement covariance matrix for laser
      ekf_.R_ = R_laser_;
    }

    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  // Update the state transition matrix F according to the new elapsed time.
  float timeDelta = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0; //dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;

  ekf_.F_ << 1, 0, timeDelta, 0,
            0, 1, 0, timeDelta,
            0, 0, 1, 0,
            0, 0, 0, 1;

  // Update the process noise covariance matrix.
  float timeDelta2 = pow(timeDelta, 2);
  float timeDelta3 = pow(timeDelta, 3);
  float timeDelta4 = pow(timeDelta, 4);

  // Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
  float noise_ax = 9;
  float noise_ay = 9;

  ekf_.Q_ << timeDelta4*noise_ax/4, 0, timeDelta3*noise_ax/2, 0,
             0, timeDelta4*noise_ay/4, 0, timeDelta3*noise_ay/2,
             timeDelta3*noise_ax/2, 0, timeDelta2*noise_ax, 0,
             0, timeDelta3*noise_ay/2, 0, timeDelta2*noise_ay;

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates

    // measurement covariance matrix
    ekf_.R_ = R_radar_;

    // ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates

    // measurement covariance matrix
    ekf_.R_ = R_laser_;

    // update location
    ekf_.Update(measurement_pack.raw_measurements_);
  }
}
