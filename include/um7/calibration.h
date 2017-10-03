#ifndef UM7_CALIBRATION_H
#define UM7_CALIBRATION_H

#include "sensor_msgs/Imu.h"

namespace um7
{

// Corrects for various intrinsic errors
class Calibration
{
public:

	Calibration(double scale, unsigned int minNumSamples);

	void Correct(sensor_msgs::Imu& msg);

	// Stores data from stationary condition
	void BufferStationarySample(const sensor_msgs::Imu& msg);

	// Returns whether there are enough buffered samples to perform calibration
	bool HasEnoughSamples() const;

	// Recomputes intrinsics using stationary data and clears buffer
	// Returns success
	bool UpdateCalibration();

private:

	double _xlScale;
	unsigned int _minNumSamples;
	std::vector<sensor_msgs::Imu> _stationarySamples;

};

}

#endif