#ifndef UM7_CALIBRATION_H
#define UM7_CALIBRATION_H

#include "sensor_msgs/Imu.h"

namespace um7
{

// Corrects for various intrinsic errors
class Calibration
{
public:

	Calibration(double scale);

	void Correct(sensor_msgs::Imu& msg);

private:

	double _xlScale;

};

}

#endif