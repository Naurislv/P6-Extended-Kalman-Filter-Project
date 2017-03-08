/*
ifndef checks if unique value (in this case measurement_package.h) is defined
then if it's not defined, it defines it and continues to the rest of the page.

When the code is included again, the first ifndef fails, resulting in blank file
Thant prevents double declaration of any identifiers such as types, enums and
static variables.

At the ned of header -> #endif
*/

#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include "../Eigen/Dense"

class MeasurementPackage {
public:
	long timestamp_;

	// enuumerator
	enum SensorType {
		LASER, RADAR
	} sensor_type_;

	Eigen::VectorXd raw_measurements_;

};

#endif /* MEASUREMENT_PACKAGE_H_ */
