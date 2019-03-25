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
  //initialize vector
	VectorXd rmse(4);
	rmse << 0, 0, 0, 0;

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	if (estimations.size() == 0 || estimations.size() != ground_truth.size()) {
		cout << "Error with estimated states size for RMSE calculation" << endl;
		return rmse;
	}

	//accumulate squared residuals
	VectorXd diff_square(4);
	diff_square << 0, 0, 0, 0;
	for (int i = 0; i < estimations.size(); ++i) {
		VectorXd diff = estimations[i] - ground_truth[i];
		VectorXd diff_square_i = diff.array()*diff.array();
		diff_square += diff_square_i;
	}

	//calculate the mean
	VectorXd diff_s_norm = diff_square / estimations.size();

	//calculate the squared root
	rmse = diff_s_norm.array().sqrt();

	//return the result
	return rmse;
}