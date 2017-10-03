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

bool Calibration::HasEnoughSamples() const
{
	return _stationarySamples.size() >= _minNumSamples;
}

void Calibration::UpdateCalibration()
{
	if( _stationarySamples.size() == 0)
	{
		ROS_WARN_STREAM( "Cannot calibrate with 0 samples" );
		return;
	}

	double acc = 0;
	for(int i = 0; i < _stationarySamples.size(); ++i)
	{
		acc += compute_xl_mag(_stationarySamples[i]);
	}
	_xlScale *= GRAVITY * _stationarySamples.size() / acc;
	ROS_INFO("Updated xl scale to %f", _xlScale);
	_stationarySamples.clear();
}

}
