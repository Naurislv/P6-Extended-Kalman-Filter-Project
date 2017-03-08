/*
ifndef checks if unique value (in this case KalmanFilter.h) is defined
then if it's not defined, it defines it and continues to the rest of the page.

When the code is included again, the first ifndef fails, resulting in blank file
Thant prevents double declaration of any identifiers such as types, enums and
static variables.

At the ned of header -> #endif
*/

#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "../Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class KalmanFilter {
public:

	///* state vector
	VectorXd x_;

	///* state covariance matrix
	MatrixXd P_;

	///* state transistion matrix
	MatrixXd F_;

	///* process covariance matrix
	MatrixXd Q_;

	///* measurement matrix
	MatrixXd H_;

	///* measurement covariance matrix
	MatrixXd R_;

	/**
	 * Constructor
	 */
	KalmanFilter();

	/**
	 * Destructor
	 */
	virtual ~KalmanFilter();

	/**
	 * Predict Predicts the state and the state covariance
	 * using the process model
	 */
	void Predict();

	/**
	 * Updates the state and
	 * @param z The measurement at k+1
	 */
	void Update(const VectorXd &z);

};

#endif /* KALMAN_FILTER_H_ */
