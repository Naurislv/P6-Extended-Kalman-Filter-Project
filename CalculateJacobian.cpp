#include <iostream>
#include "Eigen/Dense"
#include <vector>
#include <cmath>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

MatrixXd CalculateJacobian(const VectorXd& x_state);

int main() {

	/*
	 * Compute the Jacobian Matrix
	 */

	//predicted state  example
	//px = 1, py = 2, vx = 0.2, vy = 0.4
	VectorXd x_predicted(4);
	x_predicted << 1, 2, 0.2, 0.4;

	MatrixXd Hj = CalculateJacobian(x_predicted);

	cout << "Hj:" << endl << Hj << endl;

	return 0;
}

MatrixXd CalculateJacobian(const VectorXd& x_state) {

	MatrixXd Hj(3, 4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	//check division by zero
    float sum = pow(px, 2) + pow(py, 2);
    float sum2 = sqrt(sum);
    float sum32 = sqrt(pow(sum, 3));

    if (fabs(sum) < 0.0001) {
        std::cout << "CaculateJacobian () - Error - Division by Zero" << std::endl;
        return Hj;
    } else { //compute the Jacobian matrix
        Hj << px / sum2, py / sum2, 0, 0,
             -py / sum, px / sum, 0, 0,
              py * (vx * py - vy * px) / sum32,
              px * (vy * px - vx * py) / sum32,
              px / sum2, py / sum2;
    }

    return Hj;
}
