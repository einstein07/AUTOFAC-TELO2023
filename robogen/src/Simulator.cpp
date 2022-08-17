/*
 * @(#) Simulator.cpp   1.0   Nov 27, 2014
 *
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2013 Joshua Auerbach
 *
 * Laboratory of Intelligent Systems, EPFL
 *
 * This file is part of the ROBOGEN Framework.
 *
 * The ROBOGEN Framework is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License (GPL)
 * as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @(#) $Id$
 */

#include "Simulator.h"
#include "utils/RobogenCollision.h"
#include "Models.h"
#include "Robot.h"
#include "viewer/WebGLLogger.h"
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <time.h> //for benchmarking purposes
#include "EDQD/Util.h"
#include "EDQD/EDQDRobot.h"
#include "EDQD/Parameters.h"
#include "EDQD/PickUpHeuristic.h"
#include "EDQD/DropOffHeuristic.h"
#include "EDQD/MutatePerturbSensorState.h"
#include "EDQD/CollisionAvoidanceHeuristic.h"
//#define DEBUG_MASSES
//#define DEBUG_SIM_LOOP
//#define DEBUG_ROBOT_LOOP
//#define EVOLVE_SENSORS
// ODE World
extern dWorldID odeWorld;

// Container for collisions
extern dJointGroupID odeContactGroup;
std::string gCurrentBuildInfo = "Robot logging testing";
long int gVersion  = 20220518;
std::string gLogCommentText  = "(under development)"; //
std::string gStartTime = getCurrentTimeAsReadableString();
time_t gStartTimeRawFormat = time(0);
std::string gStopTime;
time_t gStopTimeRawFormat = time(0);

std::string gCompileTime = __TIME__;
std::string gCompileDate = __DATE__;

//filenames

std::ofstream gLogFile;
Logger *gLogger = NULL;

bool gVerbose_commandlineargument =             false; // set to true if given as command line argument (priority over properties file)
bool gLogDirectoryname_commandlineargument =    false; // set to true only if given as command line argument (priority over properties file)
bool gBatchMode_commandlineargument = false; // set to true only if given as command line argument (priority over properties file)
std::string gLogDirectoryname =                 "logs";
std::string gLogFilename =						/**"datalog.txt";*/ "datalog_" + gStartTime + "_" + getpidAsReadableString() + ".txt";
std::string gLogFullFilename =                  ""; // cf. the initLog method

int gRandomSeed = -1; // (default value should be "-1" => time-based random seed)

// Number of robots that are active - used to set each robot's unique ID
int gNbOfRobots = 0;
namespace robogen{

unsigned int iterations = 0;


void initLogging()
{
	// test log directory.

    /*

    // notes, 2014-09-02: unfortunatly, boost::filesystem is not a header-only boost library...
    // http://www.boost.org/doc/libs/1_53_0/more/getting_started/windows.html#header-only-libraries

    boost::filesystem::path dir (gLogDirectoryname);
    try
    {
		if (boost::filesystem::exists(dir))
	    {
			if (boost::filesystem::is_regular_file(dir))
			{
	        	std::cout << "[ERROR] directory for logging \"" << dir << "\" already exists, but is a regular file!\n";
				exit (-1);
			}
			else
				if (!boost::filesystem::is_directory(dir))
				{
					// directory does not exist. Create it.
                    std::cout << "[INFO] directory for logging \"" << dir << "\" did not exist. Creating new directory.\n";
					boost::filesystem::create_directories(dir);
				}
	    }
	    else
		{
			// directory does not exist. Create it.
            std::cout << "[INFO] directory for logging \"" << dir << "\" did not exist. Creating new directory.\n";
			boost::filesystem::create_directories(dir);
		}
	}
	catch (const boost::filesystem::filesystem_error& ex)
	{
		std::cout << ex.what() << std::endl;
		exit (-1);
	}

    */


    // init log file

    gLogFullFilename = gLogDirectoryname + "/" + gLogFilename;

	gLogFile.open(gLogFullFilename.c_str());//, std::ofstream::out | std::ofstream::app);

	if(!gLogFile) {
		std::cout << "[CRITICAL] Cannot open log file " << gLogFullFilename << "." << std::endl << std::endl;
		exit(-1);
	}

	gLogFile << "# =-=-=-=-=-=-=-=-=-=-=" << std::endl;
	gLogFile << "# LOG DATA " << std::endl;
	gLogFile << "# =-=-=-=-=-=-=-=-=-=-=" << std::endl;
	gLogFile << "#" << std::endl;
	gLogFile << "# =-= Official version tag    : " << gVersion << std::endl;
	gLogFile << "# =-= Current build name      : " << gCurrentBuildInfo << std::endl;
	gLogFile << "# =-= Compilation version tag : " << gCompileDate << " - " << gCompileTime << std::endl;
	gLogFile << "#" << std::endl;
	gLogFile << "# Loaded time stamp           : " << gStartTime << std::endl;
    gLogFile << "# process ID                  : " << getpidAsReadableString() << std::endl;
	gLogFile << "#" << std::endl;

	//gLogFile << "# log comment      : " << gLogCommentText << std::endl;

    gLogger = Logger::make_DefaultLogger(); // it is recommended (though not forced) to use gLogger instead of gLogFile.

    // ==== create specific "lite" logger file

	std::string litelogFullFilename = gLogDirectoryname + "/lite_" + gLogFilename;
	gLitelogFile.open(litelogFullFilename.c_str());

	if(!gLitelogFile) {
		std::cout << "[CRITICAL] Cannot open \"lite\" log file " << litelogFullFilename << "." << std::endl << std::endl;
		exit(-1);
	}

	gLiteLogger = new Logger();
	gLiteLogger->setLoggerFile(gLitelogFile);
	gLiteLogger->write("# lite logger\n");
	gLiteLogger->write("# generation,iteration,populationSize,minFitness,maxFitness,avgFitness");
	for (int i = 0; i < EDQD::Parameters::nbOfPhysicalObjectGroups; i++ ) {
		gLiteLogger->write(",t" + std::to_string(i));
	}
	gLiteLogger->flush();


	// ==== create specific "eog" logger file

	std::string eogLogFullFilename = gLogDirectoryname + "/eog_"
			+ gStartTime + "_" + getpidAsReadableString() + ".csv";
	EDQD::Parameters::gEOGLogFile.open(eogLogFullFilename.c_str());

	if(!EDQD::Parameters::gEOGLogFile) {
		std::cout << "[CRITICAL] Cannot open \"eog\" log file " << eogLogFullFilename << "." << std::endl << std::endl;
		exit(-1);
	}

	EDQD::Parameters::gEOGLogger = new Logger();
	EDQD::Parameters::gEOGLogger->setLoggerFile(EDQD::Parameters::gEOGLogFile);
	EDQD::Parameters::gEOGLogger->write("generation,iteration,robot,birthday,ancestor_id,ancestor_birthday,cell_id,age,sigma,maps,genomes,fitness,cells_in_map,dist_max,dist_pot_max,dist_total,com_rx,com_tx");
	for (int i = 1; i <= EDQD::Parameters::nbOfPhysicalObjectGroups; i++) {
		EDQD::Parameters::gEOGLogger->write(std::string(",obj_group" + std::to_string(i)));
	}
	EDQD::Parameters::gEOGLogger->write(std::string("\n"));
	EDQD::Parameters::gEOGLogger->flush();


	// ==== create specific "maps" logger file

	std::string mapsLogFullFilename = gLogDirectoryname + "/maps_"
			+ gStartTime + "_" + getpidAsReadableString() + ".csv";
	EDQD::Parameters::gMapsLogFile.open(mapsLogFullFilename.c_str());

	if(!EDQD::Parameters::gMapsLogFile) {
		std::cout << "[CRITICAL] Cannot open \"maps\" log file " << mapsLogFullFilename << "." << std::endl << std::endl;
		exit(-1);
	}

	EDQD::Parameters::gMapsLogger = new Logger();
	EDQD::Parameters::gMapsLogger->setLoggerFile(EDQD::Parameters::gMapsLogFile);
	EDQD::Parameters::gMapsLogger->write("generation,map_index");

	std::vector<std::string> _behav_dim_names = {"token_ratio","explore_dist"};
	for (int i = 0; i < EDQD::Parameters::nbOfDimensions; i++) {
		EDQD::Parameters::gMapsLogger->write(std::string("," + _behav_dim_names[i]));
	}
	EDQD::Parameters::gMapsLogger->write(",min_fitness,max_fitness,mean_fitness,median_fitness,sd,se,var,n,ancestors");
	EDQD::Parameters::gMapsLogger->write(std::string("\n"));
	EDQD::Parameters::gMapsLogger->flush();

}


void stopLogging(){
	gLogFile.close();

	gLitelogFile.close();
	EDQD::Parameters::gEOGLogFile.close();
	EDQD::Parameters::gMapsLogFile.close();
}


/**
 * Thread function assigned to a robot
 * @param indiQueue queue of Individuals to be evaluated
 * @param queueMutex mutex for access to queue
 * @param socket socket to simulator
 * @param confFile simulator configuration file to be used for evaluations
 */
void actionThread(
                actionThreadParams& atp,
                boost::shared_ptr<Robot>& robot,
                boost::shared_ptr<NeuralNetwork>& neuralNetwork,
                std::vector<boost::shared_ptr<Model> >& bodyParts,
                std::vector<boost::shared_ptr<Sensor> >& sensors,
                std::vector<boost::shared_ptr<Motor> >& motors,
                boost::shared_ptr<RobogenConfig> configuration,
                boost::shared_ptr<Environment>& env,
                boost::mutex& queueMutex
                ) {
    if (configuration->isCapAlleration()) {
        dBodyID rootBody = robot->getCoreComponent()->getRoot()->getBody();
        const dReal *angVel, *linVel;
        angVel = dBodyGetAngularVel(rootBody);
        linVel = dBodyGetLinearVel(rootBody);
        if(atp.t > 0) {
            // TODO make this use the step size and update default
            // limits to account for this
            double angAccel = dCalcPointsDistance3(
                            angVel, atp.previousAngVel);
            double linAccel = dCalcPointsDistance3(
                            linVel, atp.previousLinVel);
            if(angAccel > configuration->getMaxAngularAcceleration() ||
               linAccel > configuration->getMaxLinearAcceleration()) {
                printf("EVALUATION CANCELED: max accel");
                printf(" exceeded at time %f.", atp.t);
                printf(" Angular accel: %f, Linear accel: %f.\n",
                                angAccel, linAccel);
                printf("Will give %f fitness.\n", MIN_FITNESS);
                *atp.constraintViolated = true;
                //break;
            }
        }
        // save current velocities as previous
        for(unsigned int j=0; j<3; j++) {
                atp.previousAngVel[j] = angVel[j];
                atp.previousLinVel[j] = linVel[j];
        }
    }
    //std::cout<<"ACCELERATION CHECK PERFORMED."<<std::endl;
    float networkInput[MAX_INPUT_NEURONS];
    float networkOutputs[MAX_OUTPUT_NEURONS];
    // Elapsed time since last call
    env->setTimeElapsed(atp.step);
    //std::cout<<"ENV: SET ELAPSED TIME."<<std::endl;
    // Update Sensors
    for (unsigned int i = 0; i < bodyParts.size();
                    ++i) {
        if (boost::dynamic_pointer_cast<
                PerceptiveComponent>(bodyParts[i])) {
                    boost::dynamic_pointer_cast<PerceptiveComponent>(
                            bodyParts[i])->updateSensors(env);
        }
    }
    //std::cout<<"SENSORS UPDATED."<<std::endl;
    if(((atp.count - 1) % configuration->getActuationPeriod()) == 0) {
        //std::cout<<"INSIDE MODULUS IF STATEMENT. ABOUT TO FEED ANN. . ."<<std::endl;
        // Feed neural network
        for (unsigned int i = 0; i < sensors.size(); ++i) {
            networkInput[i] = sensors[i]->read();
            //std::cout<<"SENSORS HAVE BEEN READ."<<std::endl;
            // Add sensor noise: Gaussian with std dev of
            // sensorNoiseLevel * actualValue
            if (configuration->getSensorNoiseLevel() > 0.0) {
                    networkInput[i] += (atp.normalDistribution(atp.rng) *
                                    configuration->getSensorNoiseLevel() *
                                    networkInput[i]);
            }
            //std::cout<<"SENSOR NOISE HAS BEEN ADDED."<<std::endl;
        }

		//if (log) {
		//        log->logSensors(networkInput, sensors.size());
		//}
		::feed(neuralNetwork.get(), &networkInput[0]);
		//std::cout<<"ANN FED."<<std::endl;
		// Step the neural network
		::step(neuralNetwork.get(), atp.t);
			//std::cout<<"ANN STEPPED."<<std::endl;
			// Fetch the neural network outputs
		::fetch(neuralNetwork.get(), &networkOutputs[0]);
			//std::cout<<"ANN OUTPUT FETCHED."<<std::endl;
			// Send control to motors
			for (unsigned int i = 0; i < motors.size(); ++i) {
				// Add motor noise:
				// uniform in range +/- motorNoiseLevel * actualValue
				if(configuration->getMotorNoiseLevel() > 0.0) {
						networkOutputs[i] += (
											((atp.uniformDistribution(atp.rng) *
											2.0 *
											configuration->getMotorNoiseLevel())
											- configuration->getMotorNoiseLevel())
											* networkOutputs[i]);
				}
				//std::cout<<"MOTOR NOISE ADDED."<<std::endl;
				if (boost::dynamic_pointer_cast<
								RotationMotor>(motors[i])) {
						boost::dynamic_pointer_cast<RotationMotor>(motors[i]
						   )->setDesiredVelocity(networkOutputs[i], atp.step *
												configuration->getActuationPeriod());
						std::cout<<"DESIRED MOTOR VELOCITY SET: " << networkOutputs[i] << std::endl;
				} else if (boost::dynamic_pointer_cast<
								ServoMotor>(motors[i])) {
						boost::dynamic_pointer_cast<ServoMotor>(motors[i]
						   )->setDesiredPosition(networkOutputs[i], atp.step *
										configuration->getActuationPeriod());
						//motor->setPosition(networkOutputs[i], step *
						//		configuration->getActuationPeriod());
						std::cout<<"DESIRED SERVO MOTOR POSITION SET: " << networkOutputs[i] << std::endl;
				}
			}
//        if(log) {
//                log->logMotors(networkOutputs, motors.size());
//        }

    }
    //std::cout<<"OUTSIDE THE MODULUS IF STATEMENT - ABOUT TO STEP MOTORS. . ."<<std::endl;
    for (unsigned int i = 0; i < motors.size(); ++i) {
        motors[i]->step( atp.step ) ; //* configuration->getActuationPeriod() );
        //std::cout<<"MOTORS STEPPED."<<std::endl;
        // TODO find a cleaner way to do this
        // for now will reuse accel cap infrastructure
        if (motors[i]->isBurntOut()) {
                std::cout << "Motor burnt out, will terminate now "
                                << std::endl;
                *atp.motorBurntOut = true;
                //constraintViolated = true;
        }
        //std::cout<<"BURN OUT CHECKED."<<std::endl;
    }
//    if(log) {
//            log->logPosition(
//                    scenario->getRobot(
//                                    robotIndex)->getCoreComponent()->getRootPosition());
//    }
}

void stepEmbodied(
					boost::shared_ptr<Robot>& robot,
					boost::shared_ptr<CollisionAvoidanceHeuristic>& heuristicC,
					boost::shared_ptr<PickUpHeuristic>& heuristicP,
					boost::shared_ptr<DropOffHeuristic>& heuristicD,
					boost::shared_ptr<Scenario>& scene,
					boost::shared_ptr<RobogenConfig>& configuration,
					actionThreadParams& atp,
					boost::mutex& queueMutex
				){





	float networkOutputs[MAX_OUTPUT_NEURONS];

	/****************************************
	 */
	if (boost::dynamic_pointer_cast<EDQDRobot>(robot)){
		if (robot -> isAlive()){
			//std::cout << "Robot " << robot ->getId() <<" stepping drop-off" << std::endl;
			heuristicD -> step(queueMutex);
			//std::cout << "Stepping drop-off Done" << std::endl;
			osg::Vec2d signal;
			/*if (robot -> getId() == 10)
				signal = osg::Vec2d(0.5, 0.5);
			else*/
				signal = osg::Vec2d(-1000, -1000);
			if (heuristicC -> isStaticObject() && heuristicC -> getCounter() > 0){
				signal =  heuristicC -> driveToTargetPosition( heuristicC -> getSignal());
				heuristicC -> decrement();
				//std::cout << "Robot " << robot -> getId() << "Avoiding static object - current counter value: " << heuristicC -> getCounter() << std::endl;
			}
			if (signal.x() == -1000 || signal.y() == -1000) {
				signal = heuristicC->step(queueMutex);
				/**if (signal.x() != -1000){
					std::cout << "Robot " << robot -> getId() << " collision active" <<std::endl;
				}*/
			}

			if (signal.x() == -1000 || signal.y() == -1000) {
				/**if (robot -> getId() == 1){
					signal = heuristicC ->driveToTargetPosition( osg::Vec2d(0.5, 5));
				}
				else{

					signal = heuristicC ->driveToTargetPosition( osg::Vec2d(0, -5));

				}*/
				signal = heuristicP->step(queueMutex);
				/**if (signal.x() != -1000){
					std::cout << "Robot " << robot -> getId() << " pick-up active" <<std::endl;
				}*/
			}
			if (signal.x() == -1000 || signal.y() == -1000){
				signal.set(
						boost::dynamic_pointer_cast<EDQDRobot>(robot)-> motorOutputs.x(),
						boost::dynamic_pointer_cast<EDQDRobot>(robot)-> motorOutputs.y()
						);
				/**if (signal.x() != -1000){
					std::cout << "Robot " << robot -> getId() << " ann active" <<std::endl;
				}*/
			}
			//if(((atp.count - 1) % configuration->getActuationPeriod()) == 0) {

				if (boost::dynamic_pointer_cast<RotationMotor>(robot->getMotors()[0])) {

					boost::dynamic_pointer_cast<RotationMotor>(robot->getMotors()[0])
							->setDesiredVelocity(
													(signal.x()),
													atp.step * configuration->getActuationPeriod()
												);
					boost::dynamic_pointer_cast<RotationMotor>(robot->getMotors()[1])
							->setDesiredVelocity(
													(signal.y()),
													atp.step * configuration->getActuationPeriod()
												);
				}
			//}
		}
		else{
			if (boost::dynamic_pointer_cast<RotationMotor>(robot->getMotors()[0])) {

				boost::dynamic_pointer_cast<RotationMotor>(robot->getMotors()[0])
						->setDesiredVelocity(
												0.5,
												atp.step * configuration->getActuationPeriod()
											);
				boost::dynamic_pointer_cast<RotationMotor>(robot->getMotors()[1])
						->setDesiredVelocity(
												0.5,
												atp.step * configuration->getActuationPeriod()
											);
			}
		}
		for (unsigned int i = 0; i < robot->getMotors().size(); ++i) {
			robot->getMotors()[i]->step(atp.step) ;
		}
		boost::dynamic_pointer_cast<EDQDRobot>(robot)->step(queueMutex);
	}
	else{
		std::cout << "Stepping embodied with a robot that is not EDQD" <<std::endl;
	}
}
void stepGatheringZone(
					std::vector<boost::shared_ptr<Robot>>& robots,
					boost::shared_ptr<Environment>& env,
					boost::mutex& queueMutex
				){
	env -> stepGatheringZone(robots, queueMutex);
}
void monitorPopulation( bool localVerbose, std::vector<boost::shared_ptr<Robot> > robots ){
    // * monitoring: count number of active agents.
    int generation = iterations / EDQD::Parameters::evaluationTime;

    int activeCount = 0;
    double sumOfFitnesses = 0;
    double minFitness = DBL_MAX;
    double maxFitness = -DBL_MAX;

    int sumOfToken = 0;
    std::map<int,int> tokens;
//    double tokenRatio = 0.0;

    std::vector<EDQDMap> maps(gNbOfRobots);

    for ( int i = 1; i <= EDQD::Parameters::nbOfPhysicalObjectGroups; i++ ){
		tokens[i] = 0;
	}


    for ( int i = 0 ; i != gNbOfRobots ; i++ ) {
        //EDQDRobot *ctl = static_cast<EDQDRobot*>(robots[i]);
        boost::shared_ptr<EDQDRobot> ctl = boost::static_pointer_cast<EDQDRobot>(robots[i]);
        if ( ctl->isAlive() == true ) {
            activeCount++;
            sumOfFitnesses += ctl->getFitness() ;
            if ( ctl->getFitness() < minFitness )
                minFitness = ctl->getFitness();
            if ( ctl->getFitness() > maxFitness )
                maxFitness = ctl->getFitness();
        }

        for ( auto &oc : ctl->getResourceCounters() ) {
        	tokens[oc.first] += oc.second;
        	sumOfToken += oc.second;
        }

		ctl->updateCellId();

        ctl->writeEOGEntry(EDQD::Parameters::gEOGLogger);

        maps.push_back( *(ctl->getMap()) );

//        std::string __map =
//        		EDQD::mapToString(
//        				ctl->getMap()->getMap(),
//						std::string(
//								std::to_string(generation) + "," +
//								std::to_string(ctl->getWorldModel()->getId()) + ","
//								));
//		EDQDSharedData::gMapsLogManager->write(__map);
    }
    EDQD::Parameters::gEOGLogger->flush();
//    EDQDSharedData::gMapsLogManager->flush();


    if (maps.size() > 0) {
		std::vector<double> fit;
		std::map<std::pair<int,int>,int> ancestors;
		double f = 0.0;
		int sum = 0;
		int min_fit = 0;
		int max_fit = 0;
		double mean_fit = 0;
		int median_fit = 0;
		double se = 0.0;
		double sd = 0.0;
		double var = 0.0;
		int N = 0;

		behav_index_t _index;

		size_t behav_dim = EDQD::Parameters::nbOfDimensions;
	//	size_t behav_interval = EDQDSharedData::gEDQDNbOfIntervals;
		size_t num_elements = maps.begin()->getMap().num_elements();
	//	EDQDMap* last;

		for(size_t c = 0; c < num_elements; c++ ) {
			// initialise
			sum = 0;
			fit.clear();
			ancestors.clear();
			f = 0.0;
			min_fit = 0;
			max_fit = 0;
			mean_fit = 0;
			median_fit = 0;
			se = 0.0;
			sd = 0.0;
			var = 0.0;
			N = 0;


			// iterate through maps
			for (std::vector<EDQDMap>::iterator m = maps.begin(); m != maps.end(); m++ ) {
				f = m->getMap().data()[c].fitness_;
				if (f >= 0) {
					N++;
					fit.push_back(f);
					if (f < min_fit) min_fit = f;
					if (f > max_fit) max_fit = f;
	//				last = &(*m);
					//will add new entry to map if ancestor is not yet in
					// => size of ancestors is used as approximation for number of unique genomes
					ancestors[m->getMap().data()[c].id_];
				}
			}

			// calculate values
			if (N > 0) {
				size_t n = fit.size() / 2;
				nth_element(fit.begin(), fit.begin()+n, fit.end());
				if (fit.size() % 2 == 0) {
					nth_element(fit.begin(), fit.begin()+n-1, fit.end());
					median_fit = (fit[n-1] + fit[n]) / 2;
				} else {
					median_fit = fit[n];
				}
				mean_fit = sum / (double)N;
				for( int i = 0; i < N; i++ ) {
				  var += (fit[i] - mean_fit) * (fit[i] - mean_fit);
				}
				var /= N;
				sd = sqrt(var);
				se = sd / sqrt(N);
			}

			// get index values
			for (size_t dim = 0; dim < behav_dim; dim++ ) {
			  _index[dim] = (c / maps.begin()->getMap().strides()[dim] % maps.begin()->getMap().shape()[dim] +  maps.begin()->getMap().index_bases()[dim]);
			}

			// output
			std::string ofs = std::to_string(generation) + ","
					+ std::to_string(c) + ",";
			for (size_t dim = 0; dim < behav_dim; ++dim) {
				ofs += std::to_string(_index[dim] / (float)maps.begin()->getMap().shape()[dim]) + ",";
			}
			ofs += std::to_string(min_fit) + "," + std::to_string(max_fit) + ","
					+ std::to_string(mean_fit) + "," + std::to_string(median_fit) + ","
					+ std::to_string(sd) + "," + std::to_string(se) + ","
					+ std::to_string(var) + "," + std::to_string(N);
			ofs += "," + std::to_string(ancestors.size());
			ofs += "\n";

			EDQD::Parameters::gMapsLogger->write(std::string(ofs));
			ofs.clear();
			ofs = "";
		}
		EDQD::Parameters::gMapsLogger->flush();


    }

    if ( /**gVerbose && */localVerbose ) {
        std::cout << "[ gen:" << (iterations/EDQD::Parameters::evaluationTime)
					<< "\tit:" << iterations
					<< "\tpop:" << activeCount
					<< "\tminFitness:" << minFitness
					<< "\tmaxFitness:" << maxFitness
					<< "\tavgFitness:" << sumOfFitnesses/activeCount;

        for (auto &t : tokens ) {
        	std::cout << "\tt" << t.first << ": " << t.second << "(" << (double)t.second / sumOfToken << ")";
        }

        std::cout << " ]\n";
    }

    // display lightweight logs for easy-parsing
    std::string sLitelog =
        "log:"
        + std::to_string(iterations/EDQD::Parameters::evaluationTime)
        + ","
        + std::to_string(iterations)
        + ","
        + std::to_string(activeCount)
        + ","
        + std::to_string(minFitness)
        + ","
        + std::to_string(maxFitness)
        + ","
        + std::to_string(sumOfFitnesses/activeCount);
    for (auto &t : tokens ) {
		sLitelog += "," + t.second;
	}
    gLiteLogger->write(sLitelog);
    gLiteLogger->flush();  // flush internal buffer to file
    gLitelogFile << std::endl; // flush file output (+ "\n")

    // Logging, population-level: alive
    std::string sLog = std::string("") + std::to_string(iterations) + ",pop,alive," + std::to_string(activeCount) + "\n";
    gLogger->write(sLog);
    gLogger->flush();
}

void exchangeMapsWithinPopulation( std::vector<boost::shared_ptr<Robot> > robots ){
	for (unsigned int i = 0; i < robots.size(); ++i){
		for (unsigned int j = 0; j < robots.size(); ++j){
			if (i != j){
				boost::dynamic_pointer_cast<EDQDRobot>(robots[j])->storeMap(
									boost::dynamic_pointer_cast<EDQDRobot>(robots[i]) -> getMap(),
									boost::dynamic_pointer_cast<EDQDRobot>(robots[i]) -> getId()
									);
			}
		}
	}
}

unsigned int runSimulations(boost::shared_ptr<Scenario> scenario,
		boost::shared_ptr<RobogenConfig> configuration,
		const robogenMessage::Robot &robotMessage,
		IViewer *viewer, boost::random::mt19937 &rng) {
	boost::shared_ptr<FileViewerLog> log;
	return runSimulations(scenario, configuration,
			robotMessage, viewer, rng, false, log);
}
unsigned int runSimulations(boost::shared_ptr<Scenario> scenario,
		boost::shared_ptr<RobogenConfig> configuration,
		const robogenMessage::Robot &robotMessage, IViewer *viewer,
		boost::random::mt19937 &rng,
		bool onlyOnce, boost::shared_ptr<FileViewerLog> log) {

	bool constraintViolated = false;
	boost::random::normal_distribution<float> normalDistribution;
	boost::random::uniform_01<float> uniformDistribution;
	//SM added
	bool motorBurntOut = false;

	if (configuration->getMode() == "embodied"){
		std::cout<<"*****EMBODIED*****" <<std::endl;
		// ======================================================================================================
		// Configure Simulator
		// ======================================================================================================
		// * Initialize log file(s)

		initLogging();


		// =======================================
		// Simulator initialization
		// =======================================
		dInitODE();

		// Create ODE world
		odeWorld = dWorldCreate();

		// Set gravity from config
		osg::Vec3 gravity = configuration->getGravity();
		dWorldSetGravity(odeWorld, gravity.x(), gravity.y(), gravity.z());

		dWorldSetERP(odeWorld, 0.1);
		dWorldSetCFM(odeWorld, 10e-6);
		dWorldSetAutoDisableFlag(odeWorld, 1);

		// Create collision world
		//std::cout << "*****EMBODIED***** - Create ode space . . ." << std::endl;
		dSpaceID odeSpace = /**dSimpleSpaceCreate(0);*/dHashSpaceCreate(0);
		//Set space sublevel
		dSpaceSetSublevel (odeSpace, 2);
		//std::cout << "*****EMBODIED***** - Done. Space ID: " << odeSpace << std::endl;
		// Create contact group
		odeContactGroup = dJointGroupCreate(0);
		/**
		 *  wrap all this in block so things get cleaned up before
		 *  shutting down ode
		 */
		{


			// =======================================
			// Create and initialize multiple robots
			// =======================================
			//std::cout<<"EMBODIED - Create and initialize multiple robots" <<std::endl;
			std::vector<boost::shared_ptr<Robot> > robots;
			//create a collision space for each robot
			/**
			 * Reverting back to factory settings - i.e. have all robots added to main odeSpace
			 * TODO: Come back and think about necessary changes for collisions efficiency sake
			 */
			std::vector<dSpaceID> perRobotSpaces;
			for (unsigned int i = 0; i < configuration->getSwarmSize(); ++i) {
				robots.push_back(
							boost::shared_ptr<Robot>(new EDQDRobot));
				//std::cout<<"*****EMBODIED***** - create collision space for robot" << (i+1) << ". . ." <<std::endl;
				perRobotSpaces.push_back(dHashSpaceCreate(odeSpace));
				dSpaceSetSublevel (perRobotSpaces[i], 1);
				//std::cout << "*****EMBODIED***** - Done. Space ID: " << perRobotSpaces[i] << std::endl;
				//std::cout<<"*****EMBODIED***** - initialising. . ." <<std::endl;
				if (!(boost::dynamic_pointer_cast<EDQDRobot>(robots[i])->initialise(odeWorld, odeSpace, perRobotSpaces[i], gNbOfRobots, configuration, robotMessage, scenario))) {
					std::cout << "Problems decoding robot "<<(i+1)<<". Quit."
							<< std::endl;
					return SIMULATION_FAILURE;
				}

				//Add robot space to top level space
				//dSpaceAdd (odeSpace, (dGeomID)perRobotSpaces[i]);
				//std::cout<<"Robot id: " << robots[i] ->getId() <<std::endl;
				//nbAlive_++;
			}
					//SM tampered/added
			/**std::cout << "Evaluating robot team " << robots[0]->getId()
			<< ", trial: " << scenario->getCurTrial()
			<< std::endl;*/

			// =======================================
			// Register sensors
			// =======================================
			std::vector<std::vector<boost::shared_ptr<Sensor> > > robots_sensors;
			std::vector<std::vector<boost::shared_ptr<TouchSensor> > > robots_touchSensors;
			for (unsigned int i = 0; i < robots.size(); ++i){
				robots_sensors.push_back(robots[i]->getSensors());
				std::vector<boost::shared_ptr<TouchSensor> > robot_touchSensors;
				for (unsigned int j = 0; j < robots_sensors[i].size(); ++j) {
					if (boost::dynamic_pointer_cast<TouchSensor>(
							robots_sensors[i][j])) {
								robot_touchSensors.push_back(
										boost::dynamic_pointer_cast<TouchSensor>(
												robots_sensors[i][j]));
					}
				}
				//TODO: What if the robot does not have touch sensors? - is this even possible? - need to verify and handle accordingly
				robots_touchSensors.push_back(robot_touchSensors);
			}

			// =======================================
			// Register motors
			// =======================================
			std::vector<std::vector<boost::shared_ptr<Motor> > > robots_motors;
			for (unsigned int i = 0; i < robots.size(); ++i){
				robots_motors.push_back(robots[i]->getMotors());
			}

			// =======================================
			// Set cap for checking motor burn out
			// =======================================
			for (unsigned int i = 0; i < robots.size(); ++i){
			   for(unsigned int j=0; j < robots_motors[i].size(); ++j) {
				   robots_motors[i][j]->setMaxDirectionShiftsPerSecond(
						   configuration->getMaxDirectionShiftsPerSecond());
				}
			}

			// =======================================
			// Register brain and body parts
			// =======================================
			std::vector<boost::shared_ptr<NeuralNetwork> > robots_neuralNetwork;
			std::vector<std::vector<boost::shared_ptr<Model> > > robots_bodyParts;
			for (unsigned int i = 0; i < robots.size(); ++i){
				robots_neuralNetwork.push_back(robots[i]->getBrain());
				robots_bodyParts.push_back(robots[i]->getBodyParts());
			}


			// =======================================
			// Initialize scenario
			// =======================================

			dSpaceID envObjectsSpace = dHashSpaceCreate(odeSpace);
			dSpaceSetSublevel (envObjectsSpace, 1);
			dSpaceID areaSpace = dHashSpaceCreate(odeSpace);
			dSpaceSetSublevel (areaSpace, 1);
			if (!scenario->init(odeWorld, /**odeSpace*/envObjectsSpace, areaSpace, robots)) {
				std::cout << "Cannot initialize scenario with swarm. Quit."
						<< std::endl;
				return SIMULATION_FAILURE;
			}

			if((configuration->getObstacleOverlapPolicy() ==
					RobogenConfig::CONSTRAINT_VIOLATION) &&
						scenario->wereObstaclesRemoved()) {
				std::cout	<< "Using 'contraintViolation' obstacle overlap policy,"
							<< " and obstacles were removed, so will return min fitness."
							<< std::endl;
				constraintViolated = true;
			}

			// =======================================
			// Setup environment
			// =======================================
			boost::shared_ptr<Environment> env = scenario->getEnvironment();
			//========================================
			//Set up sensor mutator
			//========================================
			boost::shared_ptr<MutatePerturbSensorState> sensorStateMutator = boost::shared_ptr<MutatePerturbSensorState>(new MutatePerturbSensorState(EDQD::Parameters::sigmaRef));
			// =======================================
			// SET UP HEURISTICS
			// =======================================
			std::vector<boost::shared_ptr<PickUpHeuristic>> swarmHeuristicP_;
			std::vector<boost::shared_ptr<CollisionAvoidanceHeuristic>> swarmHeuristicC_;
			std::vector<boost::shared_ptr<DropOffHeuristic>> swarmHeuristicD_;

			for (unsigned int i = 0; i < robots.size(); ++i){
				swarmHeuristicP_.push_back(boost::shared_ptr<PickUpHeuristic>(new PickUpHeuristic(robots[i], scenario)));
				swarmHeuristicC_.push_back(boost::shared_ptr<CollisionAvoidanceHeuristic>(new CollisionAvoidanceHeuristic(robots[i], scenario)));
				swarmHeuristicD_.push_back(boost::shared_ptr<DropOffHeuristic>(new DropOffHeuristic(robots[i], scenario)));
			}

			if (!scenario->setupSimulation()) {
				std::cout	<< "Cannot setup scenario. Quit."
							<< std::endl;
				return SIMULATION_FAILURE;
			}

			// =======================================
			// Configure scene
			// =======================================
			bool visualize = (viewer != NULL);
			if(visualize && !viewer->configureScene(robots_bodyParts, scenario)) {
				std::cout 	<< "Cannot configure scene. Quit."
							<< std::endl;
				return SIMULATION_FAILURE;
			}

			std::cout << /**"Space ID: " << perRobotSpaces[0] <<*/"Total number of geoms per robot space: " << dSpaceGetNumGeoms(perRobotSpaces[0]) << std::endl;
			std::cout << /**"Space ID: " << envObjectsSpace <<*/"Total number of geoms in environment space: " << dSpaceGetNumGeoms(envObjectsSpace) << std::endl;
			std::cout << /**"Space ID: " << odeSpace <<*/"Total number of geoms in odeSpace: " << dSpaceGetNumGeoms(odeSpace) << std::endl;
			//setup vectors for keeping velocities
			int len = configuration->getSwarmSize();
			dReal previousLinVel[len][3];
			dReal previousAngVel[len][3];

			// =======================================
			// Main Loop
			// =======================================

			int count = 0;
			double t = 0;

			boost::shared_ptr<CollisionData> collisionData( new CollisionData(scenario) );
			double step = configuration->getTimeStepLength();
			osg::Vec3d gZone = env->getGatheringZone()->getPosition();
			//std::cout << "Gathering zone position (" << gZone.x() << ", " << gZone.y() << ")" << std::endl;
			//std::cout<<"EMBODIED - Main Loop . . ." <<std::endl;
			while ( (!constraintViolated) && (iterations < EDQD::Parameters::maxIterations) && (!(visualize && viewer->done())) && (env -> getGatheringZone() -> getNumberOfContainedResources() != env -> getResources().size()) ){
				if(visualize) {
					if(!viewer->frame(t, count)) {
						continue;
					}
				}


				if (scenario->shouldStopSimulationNow()) {
					std::cout << "Scenario has stopped the simulation!"
							<< std::endl;
					break;
				}

				count++;

				/*if ((count++) % 500 == 0) {
					std::cout << "." << std::flush;
				}*/
#ifdef DEBUG_SIM_LOOP
				std::cout << "MAIN LOOP - Calling SpaceCollide . . ." << std::endl;
#endif
				// Collision detection
				dSpaceCollide(odeSpace, collisionData.get(), odeCollisionCallback);
#ifdef DEBUG_SIM_LOOP
				std::cout << "MAIN LOOP - Done Calling SpaceCollide. Step world." << std::endl;
#endif
				//dWorldStep(odeWorld, step);
				dWorldQuickStep(odeWorld, step);
#ifdef DEBUG_SIM_LOOP
				std::cout << "MAIN LOOP - Done stepping world. Empty joint group." << std::endl;
#endif
				dJointGroupEmpty(odeContactGroup);

				if (configuration->isDisallowObstacleCollisions() && collisionData->hasObstacleCollisions()) {
					constraintViolated = true;
					break;
				}
#ifdef DEBUG_SIM_LOOP
				std::cout << "Done emptying joint group." << std::endl;
#endif
				// Update each robot's sensors in the threads but, update environment first - elapsed time since last call
				env->setTimeElapsed(step);

				if (robogen::iterations > 0 && robogen::iterations % EDQD::Parameters::evaluationTime == 0){
					std::cout << "******************************************************\n"
							"Lifetime ended: replace genome (if possible)\n"
							     "******************************************************" << std::endl;
					exchangeMapsWithinPopulation(robots);
#ifdef EVOLVE_SENSORS
					float dice = float(randint()%100) / 100.0;
					std::cout << "Dice value: "<< dice << std::endl;
					if ( dice <= EDQD::Parameters::pMutateSensorState )
					{
						sensorStateMutator -> mutateSensorGroup(robots);
					}
#endif
				}
				// 1. Prepare thread structure
				//if (robogen::iterations > 0 && robogen::iterations % EDQD::Parameters::evaluationTime == 0)
#ifdef DEBUG_SIM_LOOP
				std::cout << "MAIN LOOP - Preparing threads . . ." << std::endl;
#endif
				boost::thread_group actions;
				boost::mutex queueMutex;
				// 2. Launch threads
				actions.add_thread(
									new boost::thread(
													 stepGatheringZone,
													 boost::ref(robots),
													 boost::ref(env),
													 boost::ref(queueMutex)
													)
									);
				for (unsigned int i = 0; i < robots.size(); i++) {

					actionThreadParams p;
					p.step = step;
					p.t = t;
					p.count = count;
					if (boost::dynamic_pointer_cast<EDQDRobot>(robots[i])){
							actions.add_thread(
											new boost::thread(
													stepEmbodied,
													boost::ref(robots[i]),
													boost::ref(swarmHeuristicC_	[i]),
													boost::ref(swarmHeuristicP_[i]),
													boost::ref(swarmHeuristicD_[i]),
													boost::ref(scenario),
													boost::ref(configuration),
													boost::ref(p),
													boost::ref(queueMutex)));
					}
				}

				//env -> stepGatheringZone(robots);
				// 3. Join threads. Individuals are now evaluated. - put here for debugging purposes
				// Update Sensors
#ifdef DEBUG_SIM_LOOP
				std::cout << "Updating sensors" << std::endl;
#endif
				for (unsigned int i = 0; i < robots.size(); i++) {
					if (boost::dynamic_pointer_cast<EDQDRobot>(robots[i])){
						if (boost::dynamic_pointer_cast<EDQDRobot>(robots[i])->isAlive()){
							for (unsigned int j = 0; j < robots_bodyParts[i].size(); ++j) {
								if (boost::dynamic_pointer_cast<PerceptiveComponent>(robots_bodyParts[i][j])) {
									boost::dynamic_pointer_cast<PerceptiveComponent>(robots_bodyParts[i][j])->
											updateSensors(env);
								}
							}
						}
					}
				}
#ifdef DEBUG_SIM_LOOP
				std::cout << "Done with sensors" << std::endl;
#endif
				// 3. Join threads. Individuals are now evaluated.
				actions.join_all();
				if( iterations % EDQD::Parameters::evaluationTime == EDQD::Parameters::evaluationTime-1 ){
				        monitorPopulation(true, robots);
				}
#ifdef DEBUG_SIM_LOOP
				std::cout << "Robot action loop complete" << std::endl;
#endif
				if (robogen::iterations > 0 && robogen::iterations % EDQD::Parameters::evaluationTime == 0)
				std::cout << "**********************************************\n"
						"Done. Replace life iteration: " << iterations << "\n"
							 "***********************************************" <<std::endl;
				t += step;
				iterations++;
				/**if ( (iterations == EDQD::Parameters::maxIterations)){
					actions.join_all();
				}*/


			}

			if (!scenario->endSimulation()) {
				std::cout << "Cannot complete scenario. Quit."
						<< std::endl;
				return SIMULATION_FAILURE;
			}


		}
		//std::cout << "OUTSIDE MAIN LOOP - finalizing simulation . . ." << std::endl;
		// =======================================
		// Simulator finalization
		// =======================================
		// scenario has a shared ptr to the robot, so need to prune it
		scenario->pruneswarm();
		// Destroy the joint group
		dJointGroupDestroy(odeContactGroup);
		// Destroy ODE space
		dSpaceDestroy(odeSpace);
		// Destroy ODE world
		dWorldDestroy(odeWorld);
		// Destroy the ODE engine
		dCloseODE();
		/**if(constraintViolated || onlyOnce) {
			break;
		}*/

		// * clean up and quit

	    gStopTime = getCurrentTimeAsReadableString();
	    gStopTimeRawFormat = time(0);

	    time_t elapsed = int(gStopTimeRawFormat - gStartTimeRawFormat);
	    int seconds = (int)elapsed;
	    int minutes = seconds/60;
	    seconds = seconds - ( minutes * 60 );
	    int hours = minutes/60;
	    minutes = minutes - (hours * 60 );
	    int days = hours/24;
	    hours = hours - (days*24);

	    gLogFile << "# Started: " << gStartTime << std::endl;
	    gLogFile << "# Stopped: " << gStopTime << std::endl;
	    gLogFile << "# Elapsed: ";
	    if ( days > 0 )
	        gLogFile << days << " day(s), ";
	    if ( hours > 0 )
	        gLogFile << hours << " hour(s), ";
	    if ( minutes > 0 )
	        gLogFile << minutes << " minutes(s), ";
	    gLogFile <<  seconds << " second";
	    if ( seconds > 1 )
	        gLogFile << "s";
	    gLogFile << "." << std::endl;


		stopLogging();

	    std::cout << std::endl;
	    std::cout << "[timestamp::start] " << gStartTime << std::endl;
	    std::cout << "[timestamp::stop ] " << gStopTime << std::endl;
	    std::cout << "[Chronometer] ";
	    if ( days > 0 )
	        std::cout << days << " day(s), ";
	    if ( hours > 0 )
	        std::cout << hours << " hour(s), ";
	    if ( minutes > 0 )
	        std::cout << minutes << " minutes(s), ";
	    std::cout << seconds << " second";
	    if ( seconds > 1 )
	        std::cout << "s";
	    std::cout << "." << std::endl << std::endl;
	}
	else{

		while (scenario->remainingTrials() && (!constraintViolated)) {

			// =======================================
			// Simulator initialization
			// =======================================
			dInitODE();
			// Create ODE world
			odeWorld = dWorldCreate();
			// Set gravity from config
			osg::Vec3 gravity = configuration->getGravity();
			dWorldSetGravity(odeWorld, gravity.x(), gravity.y(), gravity.z());
			dWorldSetERP(odeWorld, 0.1);
			dWorldSetCFM(odeWorld, 10e-6);
			dWorldSetAutoDisableFlag(odeWorld, 1);

			// Create collision world
			dSpaceID odeSpace = dSimpleSpaceCreate(0);

			// Create contact group
			odeContactGroup = dJointGroupCreate(0);
			// wrap all this in block so things get cleaned up before shutting down
			// ode
			{
				int nbAlive_ = 0;
				// =======================================
				// Generate Robot
				// =======================================
				/**
				 * *****SM Added*****
				 * Replaced the commented block of code with code that creates and initializes multiple robots
				 */

				/**boost::shared_ptr<Robot> robot(new Robot);
				if (!robot->init(odeWorld, odeSpace, robotMessage)) {
					std::cout << "Problems decoding the robot. Quit."
							<< std::endl;
					return SIMULATION_FAILURE;
				}*/

				std::vector<boost::shared_ptr<Robot> > robots;
				//create a collision space for each robot
				std::vector<dSpaceID> robots_odeSpaces;
				for (unsigned int i = 0; i < configuration->getSwarmSize(); ++i) {
					robots.push_back(
								boost::shared_ptr<Robot>(new Robot));
					robots_odeSpaces.push_back(dSimpleSpaceCreate(0));
					if (!robots[i]->init(odeWorld, odeSpace, robots_odeSpaces[i], nbAlive_, robotMessage)) {
						std::cout << "Problems decoding robot "<<(i+1)<<". Quit."
								<< std::endl;
						return SIMULATION_FAILURE;
					}

					//Add robot space to top level space
					dSpaceAdd (odeSpace, (dGeomID)robots_odeSpaces[i]);
				}

		#ifdef DEBUG_MASSES
					float totalMass = 0;
					for (unsigned c = 0; c < robots.size(); ++c){ //SM added, changed it to work with more than one robot
				for (unsigned int i = 0; i < robots[c]->getBodyParts().size(); ++i) {
					float partMass = 0;
					for (unsigned int j = 0;
							j < robots[c]->getBodyParts()[i]->getBodies().size(); ++j) {

						dMass mass;
						dBodyGetMass(robots[c]->getBodyParts()[i]->getBodies()[j], &mass);
						partMass += mass.mass;

					}
					std::cout << robots[c]->getBodyParts()[i]->getId() <<  " has mass: "
							<< partMass * 1000. << "g" << std::endl;
					totalMass += partMass;
				}

				std::cout << "total mass is " << totalMass * 1000. << "g" << std::endl;
					}
		#endif



		//		if (log) {
		//			if (!log->init(robot, configuration)) {
		//				std::cout << "Problem initializing log!" << std::endl;
		//				return SIMULATION_FAILURE;
		//			}
		//		}
				//SM tampered/added
				//Only log first robot for now. TODO: log the behavior of all robots in the swarm
				if (log) {
					if (!log->init(robots[0], configuration)) {
						std::cout << "Problem initializing log!" << std::endl;
						return SIMULATION_FAILURE;
					}
				}



		//		std::cout << "Evaluating individual " << robot->getId()
		//				<< ", trial: " << scenario->getCurTrial()
		//				<< std::endl;
						//SM tampered/added
				std::cout << "Evaluating robot team " << robots[0]->getId()
				<< ", trial: " << scenario->getCurTrial()
				<< std::endl;

				// =======================================
				// Register sensors
				// =======================================
				/**
				 * *****SM Added*****
				 * Replaced commented out block of code with code that registers sensors for all robots in the swarm.
				 */

				/**std::vector<boost::shared_ptr<Sensor> > sensors =
						robot->getSensors();
				std::vector<boost::shared_ptr<TouchSensor> > touchSensors;
				for (unsigned int i = 0; i < sensors.size(); ++i) {
					if (boost::dynamic_pointer_cast<TouchSensor>(
							sensors[i])) {
						touchSensors.push_back(
								boost::dynamic_pointer_cast<TouchSensor>(
										sensors[i]));
					}
				}*/

				std::vector<std::vector<boost::shared_ptr<Sensor> > > robots_sensors;
				std::vector<std::vector<boost::shared_ptr<TouchSensor> > > robots_touchSensors;
				for (unsigned int i = 0; i < robots.size(); ++i){
					robots_sensors.push_back(robots[i]->getSensors());
					std::vector<boost::shared_ptr<TouchSensor> > robot_touchSensors;
					for (unsigned int j = 0; j < robots_sensors[i].size(); ++j) {
						if (boost::dynamic_pointer_cast<TouchSensor>(
								robots_sensors[i][j])) {
									robot_touchSensors.push_back(
											boost::dynamic_pointer_cast<TouchSensor>(
													robots_sensors[i][j]));
						}
					}
					//TODO: What if the robot does not have touch sensors? - is this even possible? - need to verify and handle accordingly
					robots_touchSensors.push_back(robot_touchSensors);
				}
				// =======================================
				// Register motors
				// =======================================

				/**
				 * *****SM Added*****
				 * Replaced commented out code block below with code that registers sensors for all robots in the swarm
				 */

				/**std::vector<boost::shared_ptr<Motor> > motors =
						robot->getMotors();*/

				std::vector<std::vector<boost::shared_ptr<Motor> > > robots_motors;
				for (unsigned int i = 0; i < robots.size(); ++i){
					robots_motors.push_back(robots[i]->getMotors());
				}
		//                std::cout<<"MOTORS DONE!"<<std::endl;
				// =======================================
				// Set cap for checking motor burn out
				// =======================================

				/**
				 * *****SM Added*****
				 * Set cap for all robots in the swarm
				 */

				/**for(unsigned int i=0; i< motors.size(); i++) {
					motors[i]->setMaxDirectionShiftsPerSecond(
								configuration->getMaxDirectionShiftsPerSecond());

				}*/

				for (unsigned int i = 0; i < robots.size(); ++i){
				   for(unsigned int j=0; j < robots_motors[i].size(); ++j) {
					   robots_motors[i][j]->setMaxDirectionShiftsPerSecond(
							   configuration->getMaxDirectionShiftsPerSecond());
					}
				}
		//                std::cout<<"CAP ADDED TO CHECK MOTOR BURNOUT!"<<std::endl;
				// =======================================
				// Register brain and body parts
				// =======================================

				/**
				 * *****SM Added*****
				 * Replaced commented out code block with code that registers brain and body parts for all
				 * robots in the swarm
				 */

				/**boost::shared_ptr<NeuralNetwork> neuralNetwork =
						robot->getBrain();
				std::vector<boost::shared_ptr<Model> > bodyParts =
						robot->getBodyParts();*/

				std::vector<boost::shared_ptr<NeuralNetwork> > robots_neuralNetwork;
				std::vector<std::vector<boost::shared_ptr<Model> > > robots_bodyParts;
				for (unsigned int i = 0; i < robots.size(); ++i){
					robots_neuralNetwork.push_back(robots[i]->getBrain());
					robots_bodyParts.push_back(robots[i]->getBodyParts());
				}
		//                std::cout<<"NETWORK & BODYPARTS SET!"<<std::endl;
				// =======================================
				// Initialize scenario
				// =======================================

				/**
				 * *****SM Added*****
				 * Replaced commented out code block with code that initializes secenario with swarm
				 */

				/**if (!scenario->init(odeWorld, odeSpace, robot)) {
					std::cout << "Cannot initialize scenario. Quit."
							<< std::endl;
					return SIMULATION_FAILURE;
				}*/

				std::vector<dSpaceID> perResourceSpaces;
				if (!scenario->init(odeWorld, odeSpace, odeSpace, robots)) {
					std::cout << "Cannot initialize scenario with swarm. Quit."
							<< std::endl;
					return SIMULATION_FAILURE;
				}
		//                std::cout<<"SCENARIO INITIALIZED!"<<std::endl;
				if((configuration->getObstacleOverlapPolicy() ==
						RobogenConfig::CONSTRAINT_VIOLATION) &&
							scenario->wereObstaclesRemoved()) {
					std::cout	<< "Using 'contraintViolation' obstacle overlap policy,"
								<< " and obstacles were removed, so will return min fitness."
								<< std::endl;
					constraintViolated = true;
					break;
				}

				// =======================================
				// Setup environment
				// =======================================
				boost::shared_ptr<Environment> env = scenario->getEnvironment();

				if (!scenario->setupSimulation()) {
					std::cout	<< "Cannot setup scenario. Quit."
								<< std::endl;
					return SIMULATION_FAILURE;
				}
		//                std::cout<<"ENV DONE!"<<std::endl;
				// =======================================
				// Configure scene
				// =======================================

				/**
				 * SM Added: Commented code block below and replaced it with a code block that visualizes more than one robot
				 * (a robo swarm) at a given go
				 */

				/**bool visualize = (viewer != NULL);
				if(visualize && !viewer->configureScene(bodyParts, scenario)) {
					std::cout << "Cannot configure scene. Quit."
							<< std::endl;
					return SIMULATION_FAILURE;
				}*/

				bool visualize = (viewer != NULL);
				if(visualize && !viewer->configureScene(robots_bodyParts, scenario)) {
					std::cout 	<< "Cannot configure scene. Quit."
								<< std::endl;
					return SIMULATION_FAILURE;
				}
				//std::cout<<"SCENE CONFIGURED!"<<std::endl;

				// =======================================
				// Init the webGLLogger
				// =======================================
				boost::shared_ptr<WebGLLogger> webGLlogger;
				if (log && log->isWriteWebGL()) {
					webGLlogger.reset(new WebGLLogger(log->getWebGLFileName(),
														scenario));
				}
				//std::cout<<"WEB LOGGER INIT!"<<std::endl;



				//setup vectors for keeping velocities
				int len = configuration->getSwarmSize();
				dReal previousLinVel[len][3];
				dReal previousAngVel[len][3];
						//SM added
						//std::vector<std::vector<dReal> > previousLinVel(robots.size());
						//std::vector<std::vector<dReal> > previousAngVel(robots.size());
				// =======================================
				// Main Loop
				// =======================================

				int count = 0;
				double t = 0;

				boost::shared_ptr<CollisionData> collisionData(
						new CollisionData(scenario) );
				//std::cout<<"COLLISION DATA CREATED!"<<std::endl;
				double step = configuration->getTimeStepLength();

				// CH - imposes a complexity cost, directly proportional to robot complexity, on a given robot's simulation run
				// set current simulation time to be (totalSimRuntime - (1-robotComplexity)*totalSimRuntime)
				if (robots[0]->complexityCost_==1){
					double propTime = (double) 1-static_cast<double>(robots[0]->complexity_);
					t = configuration->getSimulationTime()-(propTime*configuration->getSimulationTime());
				}

							//std::cout<<"MAIN LOOP!"<<std::endl;
				while ((t < configuration->getSimulationTime())
					   && (!(visualize && viewer->done()))) {

					if(visualize) {
						if(!viewer->frame(t, count)) {
							continue;
						}
					}


					if (scenario->shouldStopSimulationNow()) {
						std::cout << "Scenario has stopped the simulation!"
								<< std::endl;
						break;
					}

					if ((count++) % 500 == 0) {
						std::cout << "." << std::flush;
					}

					// Collision detection
					dSpaceCollide(odeSpace, collisionData.get(), odeCollisionCallback);

					// Step the world by one timestep
					dWorldStep(odeWorld, step);

					// Empty contact groups used for collisions handling
					dJointGroupEmpty(odeContactGroup);

					if (configuration->isDisallowObstacleCollisions() &&
							collisionData->hasObstacleCollisions()) {
						constraintViolated = true;
						break;
					}
							///***THREAD THIS BLOCK OF CODE***///
					/**if (configuration->isCapAlleration()) {
						dBodyID rootBody =
								robots[0]->getCoreComponent()->getRoot()->getBody();
						const dReal *angVel, *linVel;

						angVel = dBodyGetAngularVel(rootBody);
						linVel = dBodyGetLinearVel(rootBody);

						if(t > 0) {
							// TODO make this use the step size and update default
							// limits to account for this
							double angAccel = dCalcPointsDistance3(
									angVel, previousAngVel);
							double linAccel = dCalcPointsDistance3(
									linVel, previousLinVel);

							if(angAccel > configuration->getMaxAngularAcceleration() ||
							   linAccel > configuration->getMaxLinearAcceleration()) {

								printf("EVALUATION CANCELED: max accel");
								printf(" exceeded at time %f.", t);
								printf(" Angular accel: %f, Linear accel: %f.\n",
										angAccel, linAccel);
								printf("Will give %f fitness.\n", MIN_FITNESS);
								constraintViolated = true;
								break;
							}

						}

						// save current velocities as previous
						for(unsigned int j=0; j<3; j++) {
							previousAngVel[j] = angVel[j];
							previousLinVel[j] = linVel[j];
						}
					}


					float networkInput[MAX_INPUT_NEURONS];
					float networkOutputs[MAX_OUTPUT_NEURONS];

					// Elapsed time since last call
					env->setTimeElapsed(step);

					// Update Sensors
		//			for (unsigned int i = 0; i < bodyParts.size();
		//					++i) {
		//				if (boost::dynamic_pointer_cast<
		//						PerceptiveComponent>(bodyParts[i])) {
		//					boost::dynamic_pointer_cast<
		//							PerceptiveComponent>(bodyParts[i])->updateSensors(
		//							env);
		//				}
		//			}

								for (unsigned int i = 0; i < robots_bodyParts[0].size();
							++i) {
						if (boost::dynamic_pointer_cast<
								PerceptiveComponent>(robots_bodyParts[0][i])) {
							boost::dynamic_pointer_cast<
									PerceptiveComponent>(robots_bodyParts[0][i])->updateSensors(
									env);
						}
					}

					if(((count - 1) % configuration->getActuationPeriod()) == 0) {
						// Feed neural network
		//				for (unsigned int i = 0; i < sensors.size(); ++i) {
		//					networkInput[i] = sensors[i]->read();
		//
		//					// Add sensor noise: Gaussian with std dev of
		//					// sensorNoiseLevel * actualValue
		//					if (configuration->getSensorNoiseLevel() > 0.0) {
		//						networkInput[i] += (normalDistribution(rng) *
		//								configuration->getSensorNoiseLevel() *
		//								networkInput[i]);
		//					}
		//				}
										for (unsigned int i = 0; i < robots_sensors[0].size(); ++i) {
							networkInput[i] = robots_sensors[0][i]->read();

							// Add sensor noise: Gaussian with std dev of
							// sensorNoiseLevel * actualValue
							if (configuration->getSensorNoiseLevel() > 0.0) {
								networkInput[i] += (normalDistribution(rng) *
										configuration->getSensorNoiseLevel() *
										networkInput[i]);
							}
						}
		//				if (log) {
		//					log->logSensors(networkInput, sensors.size());
		//				}

										if (log) {
							log->logSensors(networkInput, robots_sensors[0].size());
						}


						//::feed(neuralNetwork.get(), &networkInput[0]);
										::feed(robots_neuralNetwork[0].get(), &networkInput[0]);
						// Step the neural network
						//::step(neuralNetwork.get(), t);
										::step(robots_neuralNetwork[0].get(), t);

						// Fetch the neural network ouputs
						//::fetch(neuralNetwork.get(), &networkOutputs[0]);
										::fetch(robots_neuralNetwork[0].get(), &networkOutputs[0]);

						// Send control to motors
		//				for (unsigned int i = 0; i < motors.size(); ++i) {
		//
		//					// Add motor noise:
		//					// uniform in range +/- motorNoiseLevel * actualValue
		//					if(configuration->getMotorNoiseLevel() > 0.0) {
		//						networkOutputs[i] += (
		//									((uniformDistribution(rng) *
		//									2.0 *
		//									configuration->getMotorNoiseLevel())
		//									- configuration->getMotorNoiseLevel())
		//									* networkOutputs[i]);
		//					}
		//
		//
		//					if (boost::dynamic_pointer_cast<
		//							RotationMotor>(motors[i])) {
		//						boost::dynamic_pointer_cast<RotationMotor>(motors[i]
		//						   )->setDesiredVelocity(networkOutputs[i], step *
		//								   	   	  configuration->getActuationPeriod());
		//					} else if (boost::dynamic_pointer_cast<
		//							ServoMotor>(motors[i])) {
		//						boost::dynamic_pointer_cast<ServoMotor>(motors[i]
		//						   )->setDesiredPosition(networkOutputs[i], step *
		//								configuration->getActuationPeriod());
		//						//motor->setPosition(networkOutputs[i], step *
		//						//		configuration->getActuationPeriod());
		//					}
		//
		//				}
										for (unsigned int i = 0; i < robots_motors[0].size(); ++i) {

							// Add motor noise:
							// uniform in range +/- motorNoiseLevel * actualValue
							if(configuration->getMotorNoiseLevel() > 0.0) {
								networkOutputs[i] += (
											((uniformDistribution(rng) *
											2.0 *
											configuration->getMotorNoiseLevel())
											- configuration->getMotorNoiseLevel())
											* networkOutputs[i]);
							}


							if (boost::dynamic_pointer_cast<
									RotationMotor>(robots_motors[0][i])) {
								boost::dynamic_pointer_cast<RotationMotor>(robots_motors[0][i]
								   )->setDesiredVelocity(networkOutputs[i], step *
												  configuration->getActuationPeriod());
							} else if (boost::dynamic_pointer_cast<
									ServoMotor>(robots_motors[0][i])) {
								boost::dynamic_pointer_cast<ServoMotor>(robots_motors[0][i]
								   )->setDesiredPosition(networkOutputs[i], step *
										configuration->getActuationPeriod());
								//motor->setPosition(networkOutputs[i], step *
								//		configuration->getActuationPeriod());
							}

						}

		//				if(log) {
		//					log->logMotors(networkOutputs, motors.size());
		//				}
										if(log) {
							log->logMotors(networkOutputs, robots_motors[0].size());
						}
					}


					bool motorBurntOut = false;
		//			for (unsigned int i = 0; i < motors.size(); ++i) {
		//				motors[i]->step( step ) ; //* configuration->getActuationPeriod() );
		//
		//				// TODO find a cleaner way to do this
		//				// for now will reuse accel cap infrastructure
		//				if (motors[i]->isBurntOut()) {
		//					std::cout << "Motor burnt out, will terminate now "
		//							<< std::endl;
		//					motorBurntOut = true;
		//					//constraintViolated = true;
		//				}
		//
		//			}
								for (unsigned int i = 0; i < robots_motors[0].size(); ++i) {
						robots_motors[0][i]->step( step ) ; //* configuration->getActuationPeriod() );

						// TODO find a cleaner way to do this
						// for now will reuse accel cap infrastructure
						if (robots_motors[0][i]->isBurntOut()) {
							std::cout << "Motor burnt out, will terminate now "
									<< std::endl;
							motorBurntOut = true;
							//constraintViolated = true;
						}

					}

					if(constraintViolated || motorBurntOut) {
						break;
					}

					if (!scenario->afterSimulationStep()) {
						std::cout
								<< "Cannot execute scenario after simulation step. Quit."
								<< std::endl;
						return SIMULATION_FAILURE;
					}

		//			if(log) {
		//				log->logPosition(
		//					scenario->getRobot(
		//							)->getCoreComponent()->getRootPosition());
		//			}
								if(log) {
						log->logPosition(
							scenario->getRobot(0
									)->getCoreComponent()->getRootPosition());
					}
						}*/
					// ==========================================================================================
					// *****END OF THREADED BLOCK*****
					// ==========================================================================================

					// 1. Prepare thread structure
					boost::thread_group actions;
					boost::mutex queueMutex;
					// 2. Launch threads
					for (unsigned int i = 0; i < robots.size(); i++) {
						actionThreadParams p;
						p.previousAngVel = previousAngVel[i];
						p.previousLinVel = previousLinVel[i];
						p.step = step;
						p.t = t;
						p.count = count;
						p.constraintViolated = &constraintViolated;
						p.motorBurntOut = &motorBurntOut;
						p.rng = rng;
						p.normalDistribution = normalDistribution;
						p.uniformDistribution = uniformDistribution;

						actions.add_thread(
							new boost::thread(
								actionThread,
								boost::ref(p),
								boost::ref(robots[i]),
								boost::ref(robots_neuralNetwork[i]),
								boost::ref(robots_bodyParts[i]),
								boost::ref(robots_sensors[i]),
								boost::ref(robots_motors[i]),
								configuration,
								boost::ref(env),
								boost::ref(queueMutex)
							)
						);
					}
					// 3. Join threads. Individuals are now evaluated.
					actions.join_all();

					if(constraintViolated || motorBurntOut) {
						break;
					}
					if (!scenario->afterSimulationStep()) {
						std::cout
							<< "Cannot execute scenario after simulation step. Quit."
							<< std::endl;
						return SIMULATION_FAILURE;
					}
					if(webGLlogger) {
							webGLlogger->log(t);
					}
					t += step;

				}

				if (!scenario->endSimulation()) {
					std::cout << "Cannot complete scenario. Quit."
							<< std::endl;
					return SIMULATION_FAILURE;
				}

				// ---------------------------------------
				// Simulator finalization
				// ---------------------------------------
		//		std::cout<<"SIM FINALIZATION!"<<std::endl;
				// Destroy the WebGlLogger, since contains pointer to scenario
				if(webGLlogger) {
					webGLlogger.reset();
				}
			} // end code block protecting objects for ode code clean up


			// scenario has a shared ptr to the robot, so need to prune it
	//		scenario->prune();
			scenario->pruneswarm();
	//		std::cout<<"PRUNED!"<<std::endl;
			// Destroy the joint group
			dJointGroupDestroy(odeContactGroup);
	//		std::cout<<"JOINT GROUP DESTROYED!"<<std::endl;
			// Destroy ODE space
			dSpaceDestroy(odeSpace);
	//		std::cout<<"ODE SPACE DESTROYED!"<<std::endl;
			// Destroy ODE world
			dWorldDestroy(odeWorld);
	//		std::cout<<"ODE WORLD DESTROYED!"<<std::endl;
			// Destroy the ODE engine
			dCloseODE();
	//                std::cout<<"ODE CLOSED!"<<std::endl;
			if(constraintViolated || onlyOnce) {
				break;
			}
					std::cout<<""<<std::endl;
		}
	}
	if(constraintViolated)
		return CONSTRAINT_VIOLATED;
        
	return SIMULATION_SUCCESS;
}

}
