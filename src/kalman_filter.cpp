#include "kalman_filter.h"
#include <math.h>
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_*Ft + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */

	std::cout << "Calling EKF update function" << std::endl;
	// Measurement Update
	VectorXd y = z - H_ * x_;
	MatrixXd S = H_ * P_*H_.transpose() + R_;
	MatrixXd K = P_ * H_.transpose()*S.inverse();

	x_ = x_ + (K * y); // Updating the state variables
	MatrixXd I = MatrixXd::Identity(4, 4);
	P_ = (I - K * H_)*P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
	float px = x_[0];
	float py = x_[1];
	float vx = x_[2];
	float vy = x_[3];

	// If rho == 0, skip the update step to avoid dividing by zero.
	if (px == 0. && py == 0.)
		return;



	float rho = sqrt(pow(px, 2) + pow(py, 2));
	float phi = atan2(py, px);
	float rho_dot = ((px*vx) + (py*vy)) / sqrt(pow(px, 2) + pow(py, 2));
	cout << "rho, phi, rho_dot " << rho << "," << phi << "," << rho_dot << endl;
	VectorXd h = VectorXd(3);
	h << rho, phi, rho_dot;
	VectorXd y = z - h;

	// Normalize the angle 
	if (y[1] > M_PI)
		y[1] -= 2.f*M_PI;
	if (y[1] < -M_PI)
		y[1] += 2.f*M_PI;


	MatrixXd S = H_ * P_*H_.transpose() + R_;
	cout << "S " << endl;
	cout << S << endl;

	MatrixXd K = P_ * H_.transpose()*S.inverse();
	cout << "K " << endl;
	cout << K << endl;

	x_ = x_ + K * y;
	cout << "x_ " << x_;
	MatrixXd I = MatrixXd::Identity(4, 4);
	P_ = (I - K * H_)*P_;
	cout << "P_ " << P_ << endl;
}
