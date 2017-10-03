#include "um7/calibration.h"
#include "um7/firmware_registers.h"
#include <ros/ros.h>

#define MIN_SCALE (0.8)
#define MAX_SCALE (1.2)

namespace um7
{
double compute_xl_mag( const sensor_msgs::Imu& msg )
{
	double ss = msg.linear_acceleration.x * msg.linear_acceleration.x
	            + msg.linear_acceleration.y * msg.linear_acceleration.y
	            + msg.linear_acceleration.z * msg.linear_acceleration.z;
	return std::sqrt( ss );
}

Calibration::Calibration( double xlScale, unsigned int minNumSamples )
	: _xlScale( xlScale ), _minNumSamples( minNumSamples ),
	_minScale( MIN_SCALE ), _maxScale( MAX_SCALE )
{}

void Calibration::SetLimits( double min, double max )
{
	_minScale = min;
	_maxScale = max;
}

void Calibration::Correct( sensor_msgs::Imu& msg )
{
	msg.linear_acceleration.x *= _xlScale;
	msg.linear_acceleration.y *= _xlScale;
	msg.linear_acceleration.z *= _xlScale;
}

void Calibration::BufferStationarySample( const sensor_msgs::Imu& msg )
{
	_stationarySamples.push_back( msg );
}

bool Calibration::HasEnoughSamples() const
{
	return _stationarySamples.size() >= _minNumSamples;
}

bool Calibration::UpdateCalibration()
{
	if( _stationarySamples.size() == 0 )
	{
		ROS_WARN_STREAM( "Cannot calibrate with 0 samples" );
		return false;
	}

	double acc = 0;
	for( int i = 0; i < _stationarySamples.size(); ++i )
	{
		acc += compute_xl_mag( _stationarySamples[i] );
	}
	double scale = _xlScale * GRAVITY * _stationarySamples.size() / acc;

	if( !std::isfinite( scale ) )
	{
		ROS_WARN_STREAM( "Computed non-finite scale!" );
		return false;
	}
	if( scale > _maxScale || scale < _minScale )
	{
		ROS_WARN_STREAM( "Computed scale " << scale << " exceeds limits (" <<
		                 _minScale << ", " << _maxScale << ")" );
		return false;
	}

	_xlScale = scale;
	ROS_INFO( "Updated xl scale to %f", _xlScale );
	_stationarySamples.clear();
	return true;
}
}
