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
  H_radar_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ <<
  0.0225, 0,
  0,      0.0225;

  //measurement covariance matrix - radar
  R_radar_ <<
  0.09, 0,      0,
  0,    0.0009, 0,
  0,    0,      0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  H_laser_ <<
  1, 0, 0, 0,
  0, 1, 0, 0;

  H_radar_ <<
  1, 1, 0, 0,
  1, 1, 0, 0,
  1, 1, 1, 1; 


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
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;
    //state covariance matrix P
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ <<
    1, 0, 0,    0,
    0, 1, 0,    0,
    0, 0, 1000, 0,
    0, 0, 0,    1000;
    //the initial transition matrix F_
    ekf_.F_ = MatrixXd(4, 4);
    ekf_.F_ <<
    1, 0, 1, 0,
    0, 1, 0, 1,
    0, 0, 1, 0,
    0, 0, 0, 1;
    ekf_.Q_ = MatrixXd(4, 4);
    ekf_.Q_ <<
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      double r = measurement_pack.raw_measurements_[0];
      double phi = measurement_pack.raw_measurements_[1];
      double rdot = measurement_pack.raw_measurements_[2];
      ekf_.x_(0) = r * cos(phi);
      ekf_.x_(1) = r * sin(phi);
      ekf_.x_(2) = rdot * cos(phi);
      ekf_.x_(3) = rdot * sin(phi);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      ekf_.x_(0) = measurement_pack.raw_measurements_[0];
      ekf_.x_(1) = measurement_pack.raw_measurements_[1];
      ekf_.x_(2) = 0.0;
      ekf_.x_(3) = 0.0;
    }

    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
	//compute the time elapsed between the current and previous measurements
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;
  
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  //Modify the F matrix so that the time is integrated
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  //Set the noise vals
  float sig_ax2 = 9.0;
  float sig_ay2 = 9.0;
  //set the process covariance matrix Q
  MatrixXd Qv(2, 2);
  Qv(0, 0) = sig_ax2; Qv(0, 1) = 0.0;
  Qv(1, 0) = 0.0;     Qv(1, 1) = sig_ay2;
  MatrixXd G(4, 2);
  G(0, 0) = dt_2/2.0; G(0, 1) = 0.0;
  G(1, 0) = 0.0;      G(1, 1) = dt_2/2.0;
  G(2, 0) = dt;       G(2, 1) = 0.0;
  G(3, 0) = 0.0;      G(3, 1) = dt;
  ekf_.Q_ = G * Qv * G.transpose();

  // Predict
  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    Tools tools;
    H_radar_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = H_radar_;
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
