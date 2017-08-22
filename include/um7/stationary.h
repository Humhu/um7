#ifndef UM7_STATIONARY_DET_
#define UM7_STATIONARY_DET_

#include "sensor_msgs/Imu.h"

namespace um7
{

// Uses buffer of sample tests to determine whether the IMU is stationary
class StationaryDetector
{
public:

	StationaryDetector(double xlMagTol, double gyroMagTol, unsigned int minSamples);

	// Adds a sample to the test buffer, returns if stationary detection fires
	bool Test(const sensor_msgs::Imu& msg);

	// Resets test buffer
	void Reset();

private:
	
	unsigned int _minNumSamples;
	double _xlMagTol;
	double _gyroMagTol;

	unsigned int _bufferedPositives;
};

}

#endif