#include "FusionEKF.h"
#include "tools.h"
#include "../Eigen/Dense"
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
    // MEASUREMENT NOISE --> measurement covariance matrix - laser
    R_laser_ << 0.0225, 0,
                0, 0.0225;
    H_laser_ = MatrixXd(2, 4);
    H_laser_ << 1, 0, 0, 0,
			    0, 1, 0, 0;

    R_radar_ = MatrixXd(3, 3);
    // MEASUREMENT NOISE --> measurement covariance matrix - radar
    R_radar_ << 0.09, 0, 0,
                0, 0.0009, 0,
                0, 0, 0.09;

    /**
    TODO:
        * Finish initializing the FusionEKF.
        * Set the process and measurement noises
    */

    // ACCELERATION NOISE --> set the acceleration noise components
	noise_ax = 9;
	noise_ay = 9;
    set_x = false;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


    /*****************************************************************************
     *    Initialization
     ****************************************************************************/
    if (!is_initialized_) {
        /**
        TODO:
            * Initialize the state ekf_.x_ with the first measurement.
            * Create the covariance matrix.
            * Remember: you'll need to convert radar from polar to cartesian coordinates.
        */
        // first measurement
        Tools tools;
        cout << "Extended Kalman Filter: " << endl;
        ekf_.x_ = VectorXd(4);
        //the initial transition matrix F_
        ekf_.F_ = MatrixXd(4, 4);
        ekf_.F_ << 1, 0, 1, 0,
                   0, 1, 0, 1,
                   0, 0, 1, 0,
                   0, 0, 0, 1;

        //state covariance matrix P
        ekf_.P_ = MatrixXd(4, 4);
        ekf_.P_ << 1, 0, 0, 0,
                   0, 1, 0, 0,
                   0, 0, 1000, 0,
                   0, 0, 0, 1000;

        //the initial noise matrix Q
        ekf_.Q_ = MatrixXd(4, 4);

        std::cout << "Sensor Type " << measurement_pack.sensor_type_ << '\n';
        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
            /**
            Convert radar from polar to cartesian coordinates and initialize state.
            */
            std::cout << "\nInitializing x with RADAR measurements:\n"
                      << "  ro : object range (radial distance from origin)\n"
                      << "  phi: bearing (angle between ro and x)\n"
                      << "  rho dot: radial velocity (change of rho)\n";

            float ro = measurement_pack.raw_measurements_[0];
            float phi = measurement_pack.raw_measurements_[1];

            float px = ro * cos(phi);
            float py = ro * sin(phi);

            std::cout << "\nInitialized px: " << px << '\n';
            std::cout << "Initialized py: " << py << '\n';
            std::cout << "Initialized vx: " << 0 << '\n';
            std::cout << "Initialized vy: " << 0 << '\n';

            ekf_.x_ << px, py, 0, 0;
            previous_timestamp_ = measurement_pack.timestamp_;
        }
        else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
            /**
            Initialize state.
            */
            std::cout << "\nInitializing x with LASER measurements.\n";
            ekf_.x_ << measurement_pack.raw_measurements_[0],
                       measurement_pack.raw_measurements_[1],
                       0, 0;
            previous_timestamp_ = measurement_pack.timestamp_;
        }

        // done initializing, no need to predict or update
        is_initialized_ = true;
        return;
    }

    /*****************************************************************************
     *    Prediction
     ****************************************************************************/

    /**
     TODO:
         * Update the state transition matrix F according to the new elapsed time.
            - Time is measured in seconds.
         * Update the process noise covariance matrix.
         * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
     */
    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
    previous_timestamp_ = measurement_pack.timestamp_;

    ekf_.F_(0, 2) = dt;
	ekf_.F_(1, 3) = dt;

    float dt2 = dt * dt;
	float dt3 = dt2 * dt;
	float dt4 = dt3 * dt;

    ekf_.Q_ << dt4 / 4 * noise_ax, 0, dt3 / 2 * noise_ax, 0,
		       0, dt4 / 4 * noise_ay, 0, dt3 / 2 * noise_ay,
	           dt3 / 2 * noise_ax, 0, dt2 * noise_ax, 0,
		       0, dt3 / 2 * noise_ay, 0, dt2 * noise_ay;

    ekf_.Predict();

    /*****************************************************************************
     *    Update
     ****************************************************************************/

    /**
     TODO:
         * Use the sensor type to perform the update step.
         * Update the state and covariance matrices.
     */

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        // Radar updates

        ekf_.R_ = R_radar_;
        if (measurement_pack.raw_measurements_(0) == 0 and
            measurement_pack.raw_measurements_(1) == 0) {
            set_x = true;
        } else {
            if (set_x == true) {
                ekf_.x_[0] = measurement_pack.raw_measurements_(0);
                ekf_.x_[1] = measurement_pack.raw_measurements_(1);
                set_x = false;
            }
            ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
            ekf_.UpdateEKF(measurement_pack.raw_measurements_);
        }

    } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
        // Laser updates
        ekf_.R_ = R_laser_;
        ekf_.H_ = H_laser_;
        ekf_.Update(measurement_pack.raw_measurements_);
    }

    // print the output
    // cout << "x_ = " << ekf_.x_ << endl;
    // cout << "P_ = " << ekf_.P_ << endl;
}
