#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
* Initializes Unscented Kalman filter
* This is scaffolding, do not modify
*/
UKF::UKF() {
	// if this is false, laser measurements will be ignored (except during init)
	use_laser_ = true;

	// if this is false, radar measurements will be ignored (except during init)
	use_radar_ = true;

	// initially set to false, set to true in first call of ProcessMeasurement
	is_initialized_ = false;

	//states and augmented states dimensions 
	n_x_ = 5;
	
	// Augmented state dimension
	n_aug_ = 7;
	
	// Augmented sigma point number
	n_sig_ = 2 * n_aug_ + 1;

	// initial state vector
	x_ = VectorXd(n_x_);

	// initial covariance matrix
	P_ = MatrixXd(n_x_, n_x_);

	// predicted sigma point matrix
	Xsig_pred_ = MatrixXd(n_x_, n_sig_);

	// Process noise standard deviation longitudinal acceleration in m/s^2
	std_a_ = 0.6;

	// Process noise standard deviation yaw acceleration in rad/s^2
	std_yawdd_ = 0.8;

	//DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
	// Laser measurement noise standard deviation position1 in m
	std_laspx_ = 0.15;

	// Laser measurement noise standard deviation position2 in m
	std_laspy_ = 0.15;

	// Radar measurement noise standard deviation radius in m
	std_radr_ = 0.5;

	// Radar measurement noise standard deviation angle in rad
	std_radphi_ = 0.03;

	// Radar measurement noise standard deviation radius change in m/s
	std_radrd_ = 0.6;
	//DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
    
	/**
	TODO:

	Complete the initialization. See ukf.h for other member properties.

	Hint: one or more values initialized above might be wildly off...
	*/
	//Sigma point spreading parameter
	lambda_ = 3 - n_aug_;

	//inistialize weights vector
	weights_ = VectorXd(n_sig_);
	weights_(0) = lambda_ / (lambda_ + n_aug_);
	for (int i = 1; i < (2*n_aug_+1) ; i++){
		weights_(i) = 0.5 / (lambda_ + n_aug_);
	}
}

UKF::~UKF() {}

/**
* @param {MeasurementPackage} measurement_pack The latest measurement data of
* either radar or laser.
*/
void UKF::ProcessMeasurement(MeasurementPackage measurement_pack) {
	/**
	TODO:

	Complete this function! Make sure you switch between lidar and radar
	measurements.
	*/
	// Initialize
	if (!is_initialized_) {
		/**
		* Initialize the state ekf_.x_ with the first measurement.
		* Create the covariance matrix.
		*/
		if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
			/**
			Convert radar from polar to cartesian coordinates and initialize state.
			*/
			// polar coordinates angle
			float r = measurement_pack.raw_measurements_[0];
			float phi = measurement_pack.raw_measurements_[1];
			float r_dot = measurement_pack.raw_measurements_[2];
			
			// check that polar angle is between +-pi
			while (phi > PI) {phi -= 2 * PI;}
			while (phi  < -PI) {phi += 2 * PI;}

			// calculate initial states from radar measurments
			x_ <<  r* cos(phi), r * sin(phi), r_dot, 0, 0;

			
		}
		else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
			/**
			Initialize state.
			*/	
			float p_x = measurement_pack.raw_measurements_[0];
			float p_y = measurement_pack.raw_measurements_[1];
			// calculate initial states from radar measurments
			x_ << p_x, p_y, 0, 0, 0;
		}
		
		// initialize state covariance matrix
		P_ << MatrixXd::Identity(n_x_, n_x_);

		//update timestamp
		previous_time_ = measurement_pack.timestamp_;
		// done initializing, no need to predict or update
		is_initialized_ = true;
		return;
	}

	double dt = (measurement_pack.timestamp_ - previous_time_) / 1000000.0; //dt - expressed in seconds
	previous_time_ = measurement_pack.timestamp_;

	//apply prediction algorithim
	Prediction(dt);

	//apply measurment update algorithim
	if ((measurement_pack.sensor_type_ == MeasurementPackage::RADAR) && (use_radar_)) {
		UpdateRadar(measurement_pack);
	}
	else if ((measurement_pack.sensor_type_ == MeasurementPackage::LASER) && (use_laser_)) {
		UpdateLidar(measurement_pack);
	}
}

/**
* Predicts sigma points, the state, and the state covariance matrix.
* @param {double} delta_t the change in time (in seconds) between the last
* measurement and this one.
*/
void UKF::Prediction(double delta_t) {
	/**
	TODO:

	Complete this function! Estimate the object's location. Modify the state
	vector, x_. Predict sigma points, the state, and the state covariance matrix.
	*/
	///* create augmented sigma points
	// Augmented mean state
	VectorXd x_aug = VectorXd::Zero(n_aug_);
	x_aug.head(n_x_) = x_;

	// Augmented state covariance
	MatrixXd P_aug = MatrixXd::Zero(n_aug_, n_aug_);
	// update augmented covariance matrix
	P_aug.topLeftCorner(n_x_, n_x_) = P_;
	P_aug(n_x_, n_x_) = std_a_ * std_a_;
	P_aug(n_x_ + 1, n_x_ + 1) = std_yawdd_ * std_yawdd_;
	
	// update square root matrix
	MatrixXd A = P_aug.llt().matrixL();
	
	// Compute sigma point matrix
    MatrixXd Xsig_aug = MatrixXd(n_aug_, n_sig_);
    // create augmented sigma points
    float c_ = sqrt(lambda_ + n_aug_);
    Xsig_aug.col(0)  = x_aug;
    for (int i = 0; i< n_aug_; ++i) {
		Xsig_aug.col(i+1)       = x_aug + c_ * A.col(i);
		Xsig_aug.col(i+1+n_aug_) = x_aug - c_ * A.col(i);
    }
  
	///* predict the sigma points
	for (int i = 0; i < n_sig_; ++i) {
		double p_x = Xsig_aug(0,i);
		double p_y = Xsig_aug(1,i);
		double v = Xsig_aug(2,i);
		double yaw = Xsig_aug(3,i);
		double yawd = Xsig_aug(4,i);
		double nu_a = Xsig_aug(5,i);
		double nu_yawdd = Xsig_aug(6,i);

		// avoid division by zero
		double px_p, py_p;
		if (fabs(yawd) > 0.001) {
			px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
			py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
		} else {
			px_p = p_x + v*delta_t*cos(yaw);
			py_p = p_y + v*delta_t*sin(yaw);
		}
		
		double v_p = v;
		double yaw_p = yaw + yawd*delta_t;
		double yawd_p = yawd;

		// add noise
		px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
		py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
		v_p = v_p + nu_a*delta_t;
		yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
		yawd_p = yawd_p + nu_yawdd*delta_t;

		// write predicted sigma point into right column
		Xsig_pred_(0,i) = px_p;
		Xsig_pred_(1,i) = py_p;
		Xsig_pred_(2,i) = v_p;
		Xsig_pred_(3,i) = yaw_p;
		Xsig_pred_(4,i) = yawd_p;
	}

	///* Predict mean and covariance
    x_.fill(0.0);
    for (int i = 0; i < n_sig_; ++i){
        x_ += weights_(i) * Xsig_pred_.col(i);
    }
    // predict state covariance matrix
    P_.fill(0.0);
    for (int i = 0; i < n_sig_; ++i) {  // iterate over sigma points
		// state difference
		VectorXd x_diff = Xsig_pred_.col(i) - x_;
		// angle normalization
		while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
		while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

		P_ += weights_(i) * x_diff * x_diff.transpose() ;
    }
}

/**
* Updates the state and the state covariance matrix using a laser measurement.
* @param {MeasurementPackage} measurement_pack
*/
void UKF::UpdateLidar(MeasurementPackage measurement_pack) {
	/**
	TODO:

	Complete this function! Use lidar data to update the belief about the object's
	position. Modify the state vector, x_, and covariance, P_.

	You'll also need to calculate the lidar NIS.
	*/
	//number of measurment variables
	int n_z = 2;

	//create matrix for sigma points in measurement space
	MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

	//mean predicted measurement
	VectorXd z_pred = VectorXd::Zero(n_z);

	//innovation covariance matrix S
	MatrixXd S = MatrixXd::Zero(n_z, n_z);

	//measurment covariance matrix
	VectorXd R_vec(n_z);
	R_vec << pow(std_laspx_, 2), pow(std_laspy_, 2);
	MatrixXd R = R_vec.asDiagonal();

	//create matrix for cross correlation Tc
	MatrixXd Tc = MatrixXd::Zero(n_x_, n_z);

	//transform sigma points into measurement space
	for (int i = 0; i<Zsig.cols(); i++) {
		float x = Xsig_pred_(0, i); //x position
		float y = Xsig_pred_(1, i); //y position

		Zsig(0, i) = x; //predicted x position measurment 
		Zsig(1, i) = y; //predicted x position measurment 
	}

	//calculate mean predicted measurement
	for (int i = 0; i < Zsig.cols(); i++) {  //iterate over sigma points
		z_pred += weights_(i) * Zsig.col(i);
	}

	//calculate innovation covariance matrix S
	for (int i = 0; i < Zsig.cols(); i++) {  //iterate over sigma points
		VectorXd meas_diff = Zsig.col(i) - z_pred;
		S = S + weights_(i) * meas_diff*meas_diff.transpose();
	}
	S += R;

	//create matrix for cross correlation Tc
	for (int i = 0; i < Zsig.cols(); i++){
		Tc += weights_(i)*(Xsig_pred_.col(i) - x_)*(Zsig.col(i) - z_pred).transpose();
	}

	//calculate Kalman gain K;
	MatrixXd K = Tc * S.inverse();

	//update state mean and covariance matrix
	x_ += K * (measurement_pack.raw_measurements_ - z_pred);
	P_ -= K * S * K.transpose();

	//calculate NIS
	NIS_laser = (measurement_pack.raw_measurements_ - z_pred).transpose() * S.inverse() * (measurement_pack.raw_measurements_ - z_pred);
}

/**
* Updates the state and the state covariance matrix using a radar measurement.
* @param {MeasurementPackage} measurement_pack
*/
void UKF::UpdateRadar(MeasurementPackage measurement_pack) {
	/**
	TODO:

	Complete this function! Use radar data to update the belief about the object's
	position. Modify the state vector, x_, and covariance, P_.

	You'll also need to calculate the radar NIS.
	*/

	//number of measurment variables
	int n_z = 3;

	//create matrix for sigma points in measurement space
	MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

	//mean predicted measurement
	VectorXd z_pred = VectorXd::Zero(n_z);

	//innovation covariance matrix S
	MatrixXd S = MatrixXd::Zero(n_z, n_z);

	//measurment covariance matrix
	VectorXd R_vec(n_z);
	R_vec << pow(std_radr_, 2), pow(std_radphi_, 2), pow(std_radrd_, 2);
	MatrixXd R = R_vec.asDiagonal();

	//create matrix for cross correlation Tc
	MatrixXd Tc = MatrixXd::Zero(n_x_, n_z);


	//transform sigma points into measurement space
	for (int i = 0; i<Zsig.cols(); i++) {
		float x = Xsig_pred_(0, i); //x position
		float y = Xsig_pred_(1, i); //y position
		float v = Xsig_pred_(2, i); //velocity
		float yaw = Xsig_pred_(3, i); //heading angle

		Zsig(0, i) = sqrt(x*x + y * y); //predicted distance measurment 

		Zsig(1, i) = atan2(y, x); //predicted polar angle
								  //normalize angle
		while (Zsig(1, i)> M_PI) Zsig(1, i) -= 2.*PI;
		while (Zsig(1, i)<-M_PI) Zsig(1, i) += 2.*PI;

		Zsig(2, i) = (x*cos(yaw)*v + y * sin(yaw)*v) / (Zsig(0, i)); //predicted velocity parallel to position vector
	}

	//calculate mean predicted measurement
	for (int i = 0; i < Zsig.cols(); i++) {  //iterate over sigma points
		z_pred += weights_(i) * Zsig.col(i);
	}

	//calculate innovation covariance matrix S
	for (int i = 0; i < Zsig.cols(); i++) {  //iterate over sigma points
		VectorXd meas_diff = Zsig.col(i) - z_pred;
		S = S + weights_(i) * meas_diff*meas_diff.transpose();
	}
	S += R;

	//create matrix for cross correlation Tc
	for (int i = 0; i < Zsig.cols(); i++){
		Tc += weights_(i)*(Xsig_pred_.col(i) - x_)*(Zsig.col(i) - z_pred).transpose();
	}

	//calculate Kalman gain K;
	MatrixXd K = Tc * S.inverse();

	//update state mean and covariance matrix
	x_ += K * (measurement_pack.raw_measurements_ - z_pred);
	P_ -= K * S * K.transpose();

	//calculate NIS
	NIS_radar = (measurement_pack.raw_measurements_ - z_pred).transpose() * S.inverse() * (measurement_pack.raw_measurements_ - z_pred);
}
