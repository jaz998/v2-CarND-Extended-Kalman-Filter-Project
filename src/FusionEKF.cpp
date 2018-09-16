#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <math.h>

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

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  noise_ax = 9.0;
  noise_ay = 9.0;



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
    cout << "Initializing EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

	// Initialize positions and velocities 
	float px = 0.0;
	float py = 0.0;
	float vx = 0.0;
	float vy = 0.0;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
		float rho = measurement_pack.raw_measurements_[0];
		float phi = measurement_pack.raw_measurements_[1];
		px = rho * cos(phi);
		py = rho * sin(phi);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
		px = measurement_pack.raw_measurements_[0];
		py = measurement_pack.raw_measurements_[1];
    }

	ekf_.x_ << px, py, vx, vy;
	previous_timestamp_ = measurement_pack.timestamp_;

	// Estimate the initial state covariance matrix
	// with a covariance of 1 for the x and y position values.
	ekf_.P_ = MatrixXd(4, 4);
	ekf_.P_ << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1000, 0,
		0, 0, 0, 1000;

	// Initialize state transition matrix
	ekf_.F_ = MatrixXd(4, 4);
	ekf_.F_ = MatrixXd::Identity(4, 4);

	// Initialize measurement matrix for laser measurements
	H_laser_ << 1, 0, 0, 0,
		0, 1, 0, 0;

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

  // Calculate the time difference between two measurements in seconds
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  // Update the state transition function with a constant velocity model
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  //update the proces covriance matrix Q
  cout << "update the proces covriance matrix Q" << endl;
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << pow(dt, 4) / 4 * noise_ax, 0, pow(dt, 3) / 2 * noise_ax, 0,
	  0, pow(dt, 4) / 4 * noise_ay, 0, pow(dt, 3) / 2 * noise_ay,
	  pow(dt, 3) / 2 * noise_ax, 0, pow(dt, 2)*noise_ax, 0,
	  0, pow(dt, 3) / 2 * noise_ay, 0, pow(dt, 2)*noise_ay;


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
	  ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
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
