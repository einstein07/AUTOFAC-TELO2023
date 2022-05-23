/*
 * @(#) Log.cpp   1.0   May 18, 2022
 *
 * Sindiso Mkhatshwa (mkhsin035@myuct.ac.za)
 *
 * Nitschke Laboratory, UCT
 *
 * @(#) $Id$
 */

#include "Logger.h"

#include "Util.h"


Logger::Logger(std::ofstream __logFile)
{
    logFile = &__logFile;
	buffer = "";
}

Logger::Logger()
{
    logFile = &gLogFile;
}

Logger::~Logger()
{
}

void Logger::write(std::string str)
{
    buffer += str;
}

void Logger::flush()
{
    if ( ! buffer.empty() )
    {
        (*logFile) << buffer;
        buffer.clear();
    }
}

Logger* Logger::make_DefaultLogger()
{
    return new Logger();
}

void Logger::setLoggerFile ( std::ofstream &__logFile )
{
    logFile = &__logFile;
}
