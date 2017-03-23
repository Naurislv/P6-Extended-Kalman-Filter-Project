#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

    int est_size = estimations.size();
    vector<VectorXd> residuals;
    VectorXd rmse(4);
    rmse << 0,0,0,0;
    VectorXd mean(4);

    //  * the estimation vector size should not be zero
    //  * the estimation vector size should equal ground truth vector size
    if (est_size == 0) {
        std::cout << "Estimantions vector size is 0" << std::endl;
        return rmse;
    } else if (est_size != ground_truth.size()) {
        std::cout << "Estimantions vector size is not equal to Ground Truth vector size" << std::endl;
        return rmse;
    }

    //accumulate squared residuals
    for(int i=0; i < est_size; ++i){
        VectorXd residual = estimations[i] - ground_truth[i];
        residual = residual.array() * residual.array();
        rmse = rmse.array() + residual.array();
    }
    //calculate the mean
    rmse = rmse / est_size;
    //calculate the squared root
    rmse = rmse.array().sqrt();


    //return the result
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
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
