#include "um7/stationary.h"
#include "um7/firmware_registers.h"

namespace um7
{

StationaryDetector::StationaryDetector(double xlMagTol, double gyroMagTol,
                                       unsigned int minSamples)
: _minNumSamples(minSamples), _xlMagTol(xlMagTol), _gyroMagTol(gyroMagTol) 
{}

bool StationaryDetector::Test(const sensor_msgs::Imu& msg)
{
	double xlSs = msg.linear_acceleration.x * msg.linear_acceleration.x
	            + msg.linear_acceleration.y * msg.linear_acceleration.y
				+ msg.linear_acceleration.z * msg.linear_acceleration.z;
	double gyroSs = msg.angular_velocity.x * msg.angular_velocity.x
	              + msg.angular_velocity.y * msg.angular_velocity.y
				  + msg.angular_velocity.z * msg.angular_velocity.z;
	
	double xlMag = std::sqrt(xlSs);
	double gyroMag = std::sqrt(gyroSs);

	double xlErr = std::abs(xlMag - GRAVITY); // XL should be gravity
	double gyroErr = gyroMag; // Gyro should be 0

	if(xlErr < _xlMagTol && gyroErr < _gyroMagTol)
	{
		_bufferedPositives++;
	}
	
	return _bufferedPositives >= _minNumSamples;
}

void StationaryDetector::Reset()
{
	_bufferedPositives = 0;
}

}