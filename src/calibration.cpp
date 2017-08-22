#include "um7/calibration.h"
#include "um7/firmware_registers.h"
#include <ros/ros.h>

namespace um7
{

double compute_xl_mag(const sensor_msgs::Imu& msg)
{
	double ss = msg.linear_acceleration.x * msg.linear_acceleration.x
	          + msg.linear_acceleration.y * msg.linear_acceleration.y
			  + msg.linear_acceleration.z * msg.linear_acceleration.z;
	return std::sqrt(ss);
}

Calibration::Calibration(double xlScale, unsigned int minNumSamples)
: _xlScale(xlScale), _minNumSamples(minNumSamples)
{}

void Calibration::Correct(sensor_msgs::Imu& msg)
{
	msg.linear_acceleration.x *= _xlScale;
	msg.linear_acceleration.y *= _xlScale;
	msg.linear_acceleration.z *= _xlScale;	
}

void Calibration::BufferStationarySample(const sensor_msgs::Imu& msg)
{
	_stationarySamples.push_back(msg);
}

void Calibration::UpdateCalibration()
{
	if(_stationarySamples.size() < _minNumSamples)
	{
		ROS_WARN("Have %lu samples < min %d, skipping calibration update",
		         _stationarySamples.size(), _minNumSamples);
		return;
	}

	double acc;
	for(int i = 0; i < _stationarySamples.size(); ++i)
	{
		acc += compute_xl_mag(_stationarySamples[i]);
	}
	_xlScale = GRAVITY * _stationarySamples.size() / acc;
	ROS_INFO("Updated xl scale to %f", _xlScale);
	_stationarySamples.clear();
}

}