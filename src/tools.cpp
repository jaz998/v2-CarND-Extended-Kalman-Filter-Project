#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
	const vector<VectorXd> &ground_truth) {
	/**
	TODO:
	* Calculate the RMSE here.
	*/
	VectorXd RMSE(4);
	RMSE << 0, 0, 0, 0;

	//Perform validation of the input data
	if (estimations.size() == 0 || ground_truth.size() == 0) {
		cout << "Input validation failed: either estimations or ground truth has zero size" << endl;
		return RMSE;
	}
	if (estimations.size() != ground_truth.size()) {
		cout << "Input validation failed: estimations and groud truths have different sizes" << endl;
		return RMSE;
	}

	for (unsigned int i = 0; i < estimations.size(); ++i) {
		VectorXd diff = estimations[i] - ground_truth[i];
		//cout << "Estimation size " << estimations.size() << endl;
		//cout << "Diff " << diff << endl;
		//cout << "line 37 " << endl;
		diff = pow(diff.array(), 2);
		RMSE = diff + RMSE;
		//cout << "Showing temp RMSE " << RMSE << endl;
		//getchar();
	}
	RMSE = RMSE / estimations.size();
	//cout << "Estimation size " << estimations.size() << endl;
	RMSE = RMSE.array().sqrt();
	//cout << "Printing RMSE " << endl;
	//cout << RMSE << endl;
	return RMSE;

}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
	MatrixXd Hj(3, 4);

	if (x_state.size() != 4) {
		cout << "ERROR - CalculateJacobian () - The state vector must have size 4." << endl;
		return Hj;
	}
	double px = x_state[0];
	double py = x_state[1];
	double vx = x_state[2];
	double vy = x_state[3];

	//pre-calculate a set of terms to avoid repeated calculation
	double squared_sum = pow(px, 2) + pow(py, 2);
	double squared_root = sqrt(pow(px, 2) + pow(py, 2));

	//check division by zero
	if (fabs(squared_sum) < 0.0001) {
		cout << "ERROR - CalculateJacobian () - Division by Zero" << endl;
		return Hj;
	}

	//compute the Jacobian matrix
	Hj << (px / squared_root), (py / squared_root), 0, 0,
		-(py / squared_sum), (px / squared_sum), 0, 0,
		py*(vx*py - vy * px) / pow(squared_sum, 3.0 / 2.0), px*(px*vy - py * vx) / pow(squared_sum, 3.0 / 2.0), px / squared_root, py / squared_root;

	return Hj;
}
