/*
 * @(#) EDQDController.h   1.0   Sep 07, 2021
 *
 * Sindiso Mkhatshwa (mkhsin035@myuct.ac.za)
 *
 * Nitschke Laboratory, UCT
 *
 * @(#) $Id$
 */

#ifndef EDQD_UTIL_H_
#define EDQD_UTIL_H_
/************************************************************************
 * INCLUDES
 ***********************************************************************/

#include <cstdlib> // RAND_MAX
#include <random>
#include <chrono>
#include <sys/time.h>
#include <boost/multi_array.hpp>
#include <boost/lexical_cast.hpp>
#include "Robogen.h"
#include "Logger.h"

/**
 * \brief GLOBAL VARIABLES
 */
/**
 * \brief display through "-v[ersion]" command line option -- check/set value in config.h
 */
extern std::string gCurrentBuildInfo;
/**
 * \brief version
 */
extern long int gVersion;

/**
 * \brief a string containing a readable form of the time when the current program was started (useful for naming log files).
 */
extern std::string gStartTime;
/**
 * \brief a string containing a readable form of the time when the current program was stopped.
 */
extern std::string gStopTime;
/**
 * \brief Start time - raw format
 */
extern time_t gStartTimeRawFormat;
/**
 * \brief Stop time - raw format
 */
extern time_t gStopTimeRawFormat;
/**
 * \brief Compile time
 */
extern std::string gCompileTime;
/**
 * \brief Compile date
 */
extern std::string gCompileDate;

// files
/**
 * \brief user comment that will be written in the log file (e.g. description of experimental setup)
 */
extern std::string gLogCommentText;
/**
 * \brief Verbose cmd argument
 */
extern bool gVerbose_commandlineargument;
/**
 *\brief Log directory cmd argument
 */
extern bool gLogDirectoryname_commandlineargument;
/**
 * \brief Batch mode flag
 */
extern bool gBatchMode_commandlineargument;
/**
 * \brief LOg directory name
 */
extern std::string gLogDirectoryname;
/**
 * \brief Log file name
 */
extern std::string gLogFilename;
/**
 * \brief Log full filename - directory
 */
extern std::string gLogFullFilename;
/**
 * \brief Log file
 */
extern std::ofstream gLogFile;
/**
 * \brief Logger
 */
extern Logger *gLogger;

/**
 *\brief Random seed. Default value (="-1") means time based.
 *
 * "Note that boost/c++11 random is slower (by one order of magnitude), but more precise,
 * than old classic rand()"
 *
 */
extern int gRandomSeed;

extern std::random_device rnd;
/**
 * \brief Returns an int with value drawn uniformly in [0,max)
 */
extern std::minstd_rand randint;
/**
 * \brief Engine
 */
extern std::mt19937 engine;
/**
 * \brief Uniform real distribution
 */
extern std::uniform_real_distribution<double> disRandom;
/**
 * \brief Normal distribution
 */
extern std::normal_distribution<> disNormal;
/**
 * \brief Iniform, unbiased
 */
extern std::uniform_int_distribution<int> randomSensor;
/**
 * Uniform in [0,1), return double
 */
#define random() disRandom(engine)
/**
 * \brief Normal distribution mean=0 and stddev=1 (use: mean+rand*stddev)
 */
#define randgaussian() disNormal(engine)

/**
 * \brief Return current time in a string, with readable format - e.g. 20100526-10h12m08s
 *
 * check: http://www.cplusplus.com/reference/clibrary/ctime/strftime/
 */
std::string getCurrentTimeAsReadableString();

/** \brief Return PID as a string. Useful to desambiguate timestamps */
std::string getpidAsReadableString();

/**
 * \brief Convert integer to string. check: http://notfaq.wordpress.com/2006/08/30/c-convert-int-to-string/
 * 	@param int value
 * 	@return std::string string representation
 */
std::string convertToString( int __value );

/**
 * \brief Calculates and returns euclidean distance: two 2D points
 * @param double x1
 * @param double y1
 * @param double x2
 * @param double y2
 * @return euclidean distance
 */
inline double getEuclideanDistance(double x1, double y1, double x2, double y2)
{
	return sqrt( ( x1 - x2 ) * ( x1 - x2 ) + ( y1 - y2 ) * ( y1 - y2 ) );
}
/**
 * \brief Checks if there is an overlap between two regions
 * @param double minX
 * @param double maxX
 * @param double minY
 * @param double maxY
 * @param double minZ
 * @param double maxZ
 * @param double other region minX
 * @param double other region maxX
 * @param double other region minY
 * @param double other region maxY
 * @param double other region minZ
 * @param double other region maxZ
 * @return true if there is, otherwise false
 */
bool isOverlap(double minX, double maxX, double minY, double maxY, double minZ, double maxZ, double oMinX, double oMaxX, double oMinY, double oMaxY, double oMinZ, double oMaxZ);
/**
 * \brief Calculates and returns euclidean distance: two 3D points
 * @param osg::Vec3d a
 * @param osg::Vec3d b
 * @return euclidean distance
 */
double distance (osg::Vec3d a, osg::Vec3d b);
/**
 * \brief Normalize value within bounds
 * @param double value to normalize
 * @param double minimum
 * @param double maximum
 * @double normalized value
 */
double normaliseValue(double newValue, double _minValue, double _maxValue);

#endif /* EDQD_UTIL_H_ */
