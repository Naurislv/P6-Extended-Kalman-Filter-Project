/*
ifndef checks if unique value (in this case tracking.h) is defined
then if it's not defined, it defines it and continues to the rest of the page.

When the code is included again, the first ifndef fails, resulting in blank file
Thant prevents double declaration of any identifiers such as types, enums and
static variables.

At the ned of header -> #endif
*/

#ifndef TRACKING_H_
#define TRACKING_H_

#include "measurement_package.h"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"

class Tracking {
public:
	Tracking();
	virtual ~Tracking();
	void ProcessMeasurement(const MeasurementPackage &measurement_pack);
	KalmanFilter kf_;

private:
	bool is_initialized_;
	long previous_timestamp_;

	//acceleration noise components
	float noise_ax;
	float noise_ay;

};

#endif /* TRACKING_H_ */
