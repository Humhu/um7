#include "um7/calibration.h"

namespace um7
{

Calibration::Calibration(double xlScale)
: _xlScale(xlScale) 
{}

void Calibration::Correct(sensor_msgs::Imu& msg)
{
	msg.linear_acceleration.x *= _xlScale;
	msg.linear_acceleration.y *= _xlScale;
	msg.linear_acceleration.z *= _xlScale;	
}

}