/*
 * @(#) Util.cpp   1.0  Sep 16, 2021
 *
 * Sindiso Mkhatshwa (mkhsin035@myuct.ac.za)
 *
 * Nitschke Laboratory, UCT
 *
 * @(#) $Id$
 */
#include "Util.h"
#include <string>
#include <random>

// random generator functions, header declaration in common.h (general scope)
std::random_device rnd;
std::minstd_rand randint;
std::mt19937 engine(rnd());
std::uniform_real_distribution<double> disRandom(0.0, 1.0);
std::normal_distribution<> disNormal(0,1);
std::uniform_int_distribution<int> randomSensor(
		2, // Resource type-1 sensor element
		6  // Resource type-5 sensor element
		); // uniform, unbiased


std::string getpidAsReadableString()
{
    return boost::lexical_cast<std::string>((long)::getpid());
}

std::string getCurrentTimeAsReadableString()
{
	// --- get current time information

	time_t now = time(NULL);
	char timestamp[19] = "";
	//strftime (timestamp, 19,"%Y%m%d-%Hh%Mm%Ss", localtime(&now));
	strftime (timestamp, 19,"%Y%m%d-%Hh%Mm%Ss", localtime(&now));
	std::string s = timestamp;

	// --- get milliseconds resolution (note: could be used to replace code block above - left for tutorial)

	struct timeval now2;
    int mtime;
    gettimeofday(&now2, NULL);

	mtime = now2.tv_usec;
	if ( mtime < 100000 )
	{
		s+="0";
		if ( mtime < 10000 )
			s+="0";
		if ( mtime < 1000 )
			s+="0";
		if ( mtime < 100 )
			s+="0";
		if ( mtime < 10 )
			s+="0";
	}
	s += convertToString(mtime) + "us"; // microseconds

	return s;

}

std::string convertToString( int __value )
{
	std::string s;
	std::stringstream sOutTmp;
	sOutTmp << __value;
	s = sOutTmp.str();

	return s;
}
bool isOverlap(double minX, double maxX, double minY, double maxY, double minZ, double maxZ, double oMinX, double oMaxX, double oMinY, double oMaxY, double oMinZ, double oMaxZ){
	bool isOverlapping = false;

	bool inRangeX = false;
	if ((oMinX <= minX && oMaxX >= maxX) || (oMinX >= minX && oMinX <= maxX)
			|| (oMaxX >= minX && oMaxX <= maxX)) {
		inRangeX = true;
	}

	bool inRangeY = false;
	if ((oMinY <= minY && oMaxY >= maxY) || (oMinY >= minY && oMinY <= maxY)
			|| (oMaxY >= minY && oMaxY <= maxY)) {
		inRangeY = true;
	}

	bool inRangeZ = false;
	if ((oMinZ <= minZ && oMaxZ >= maxZ) || (oMinZ >= minZ && oMinZ <= maxZ)
			|| (oMaxZ >= minZ && oMaxZ <= maxZ)) {
		inRangeZ = true;
	}

	if (inRangeX && inRangeY /*|| inRangeZ*/){
		isOverlapping = true;
	}
	return isOverlapping;
}
double distance (osg::Vec3d a, osg::Vec3d b){
	double distance = sqrt(pow(a.x() - b.x(), 2)
						+ pow(a.y() - b.y(), 2)
						+ pow(a.z() - b.z(), 2));
	return distance;
}

double normaliseValue(double newValue, double _minValue, double _maxValue){
	double value = newValue;
	// bouncing upper/lower bounds
	if ( value < _minValue )
	{
		double range = _maxValue - _minValue;
		double overflow = - ( (double)value - _minValue );
		overflow = overflow - 2*range * (int)( overflow / (2*range) );
		if ( overflow < range )
			value = _minValue + overflow;
		else // overflow btw range and range*2
			value = _minValue + range - (overflow-range);
	}
	else if ( value > _maxValue )
	{
		double range = _maxValue - _minValue;
		double overflow = (double)value - _maxValue;
		overflow = overflow - 2*range * (int)( overflow / (2*range) );
		if ( overflow < range )
			value = _maxValue - overflow;
		else // overflow btw range and range*2
			value = _maxValue - range + (overflow-range);
	}
	return value;
}
