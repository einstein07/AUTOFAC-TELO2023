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
#include "Logger.h"

/************************************************************************
 * GLOBAL VARIABLES
 ***********************************************************************/
extern std::string gCurrentBuildInfo;  // display through "-v[ersion]" command line option -- check/set value in config.h
extern long int gVersion;


extern std::string gStartTime; // a string containing a readable form of the time when the current program was started (useful for naming log files).
extern std::string gStopTime; // a string containing a readable form of the time when the current program was stopped.
extern time_t gStartTimeRawFormat;
extern time_t gStopTimeRawFormat;

extern std::string gCompileTime;
extern std::string gCompileDate;

// files

extern std::string gLogCommentText; // user comment that will be written in the log file (e.g. description of experimental setup)
extern bool gVerbose_commandlineargument;
extern bool gLogDirectoryname_commandlineargument;
extern bool gBatchMode_commandlineargument;
extern std::string gLogDirectoryname;
extern std::string gLogFilename;
extern std::string gLogFullFilename;

extern std::ofstream gLogFile;
extern Logger *gLogger;

/**
 * According to previous work:
 * "Note that boost/c++11 random is slower (by one order of magnitude), but more precise,
 * than old classic rand()"
 * The same is used here in order to ensure that the algorithm is not any less efficient
 */

extern std::random_device rnd;
extern std::minstd_rand randint; // randint() returns an int with value drawn uniformly in [0,max)
extern std::mt19937 engine;
extern std::uniform_real_distribution<double> disRandom;
extern std::normal_distribution<> disNormal;

#define random() disRandom(engine) // uniform in [0,1), return double
#define randgaussian() disNormal(engine) // normal distribution mean=0 and stddev=1 (use: mean+rand*stddev)

// return current time in a string, with readable format - e.g. 20100526-10h12m08s
// check: http://www.cplusplus.com/reference/clibrary/ctime/strftime/
std::string getCurrentTimeAsReadableString();

// return PID as a string. Useful to desambiguate timestamps
std::string getpidAsReadableString();

// check: http://notfaq.wordpress.com/2006/08/30/c-convert-int-to-string/
std::string convertToString( int __value );

// return euclidean distance btw two 2D points
inline double getEuclideanDistance(double x1, double y1, double x2, double y2)
{
	return sqrt( ( x1 - x2 ) * ( x1 - x2 ) + ( y1 - y2 ) * ( y1 - y2 ) );
}

#endif /* EDQD_UTIL_H_ */
