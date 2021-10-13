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
#include "EDQD/EDQDRobot.h"

//#define DEBUG_MASSES

// ODE World
extern dWorldID odeWorld;

// Container for collisions
extern dJointGroupID odeContactGroup;

namespace robogen{

unsigned int iterations = 0;
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
					boost::shared_ptr<Environment>& env,
					/**boost::shared_ptr<RobogenConfig>& configuration,*/
					/**boost::random::mt19937 &rng,*/
					/**int count,*/
					/**double step,*/
					double elapsedEvaluationTime,
					boost::mutex& queueMutex
				){
	//std::cout << "************************STEP STARTS NOW - THREAD********************************" << std::endl;
	boost::dynamic_pointer_cast<EDQDRobot>(robot)->step(
														env,
														/**configuration,*/
														/**rng,*/
														/**count,*/
														/**step,*/
														elapsedEvaluationTime
														);
	//std::cout << "************************STEP DONE - THREAD********************************" << std::endl;

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
		dSpaceID odeSpace = /**dSimpleSpaceCreate(0)*/dHashSpaceCreate(0);
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

			// Number of robots that are stil actve
			int nbAlive_ = 0;
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
				std::cout<<"*****EMBODIED***** - create collision space for robot" << (i+1) << ". . ." <<std::endl;
				perRobotSpaces.push_back(dHashSpaceCreate(odeSpace));
				dSpaceSetSublevel (perRobotSpaces[i], 1);
				std::cout << "*****EMBODIED***** - Done. Space ID: " << perRobotSpaces[i] << std::endl;
				//std::cout<<"*****EMBODIED***** - initialising. . ." <<std::endl;
				if (!(boost::dynamic_pointer_cast<EDQDRobot>(robots[i])->initialise(odeWorld, odeSpace, perRobotSpaces[i], nbAlive_, configuration, robotMessage, scenario))) {
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
			std::cout << "Evaluating robot team " << robots[0]->getId()
			<< ", trial: " << scenario->getCurTrial()
			<< std::endl;

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
			if (!scenario->init(odeWorld, /**odeSpace*/envObjectsSpace, robots)) {
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

			std::cout << "Space ID: " << perRobotSpaces[0] <<". Total number of geoms in 1st robot space: " << dSpaceGetNumGeoms(perRobotSpaces[0]) << std::endl;
			std::cout << "Space ID: " << envObjectsSpace <<". Total number of geoms in environment space: " << dSpaceGetNumGeoms(envObjectsSpace) << std::endl;
			std::cout << "Space ID: " << odeSpace <<". Total number of geoms in odeSpace: " << dSpaceGetNumGeoms(odeSpace) << std::endl;
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
			std::cout << "Gathering zone position (" << gZone.x() << ", " << gZone.y() << ")" << std::endl;
			//std::cout<<"EMBODIED - Main Loop . . ." <<std::endl;
			while ( (!constraintViolated) && (iterations < EDQD::Parameters::maxIterations) && (!(visualize && viewer->done())) ){
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
				//std::cout << "MAIN LOOP - Calling SpaceCollide . . ." << std::endl;
				// Collision detection
				dSpaceCollide(odeSpace, collisionData.get(), odeCollisionCallback);
				//std::cout << "Done" << std::endl;
				// Step the world by one timestep
				//std::cout << "MAIN LOOP - Calling WorldStep . . ." << std::endl;
				dWorldStep(odeWorld, step);
				//std::cout << "Done." << std::endl;
				// Empty contact groups used for collisions handling
				//std::cout << "MAIN LOOP - Calling SpaceCollide . . ." << std::endl;
				dJointGroupEmpty(odeContactGroup);
				//std::cout << "Done." << std::endl;
				//std::cout << "MAIN LOOP - Checking if constraints were violated . . ." << std::endl;
				if (configuration->isDisallowObstacleCollisions() && collisionData->hasObstacleCollisions()) {
					constraintViolated = true;
					break;
				}
				//std::cout << "Done." << std::endl;
				// Update each robot's sensors in the threads but, update environment first - elapsed time since last call
				env->setTimeElapsed(step);

				// 1. Prepare thread structure
				//std::cout << "MAIN LOOP - Preparing threads . . ." << std::endl;
				boost::thread_group actions;
				boost::mutex queueMutex;
				// 2. Launch threads
				for (unsigned int i = 0; i < robots.size(); i++) {

					actions.add_thread(
									new boost::thread(
											stepEmbodied,
											boost::ref(robots[i]),
											boost::ref(env),
											t,
											boost::ref(queueMutex)));
				}
				// Update Sensors
				for (unsigned int i = 0; i < robots.size(); i++) {
					for (unsigned int j = 0; j < robots_bodyParts[i].size(); ++j) {
						if (boost::dynamic_pointer_cast<PerceptiveComponent>(robots_bodyParts[i][j])) {
							//std::cout << "Updating sensor for robot "<< (i+1) << std::endl;
							boost::dynamic_pointer_cast<PerceptiveComponent>(robots_bodyParts[i][j])->
									updateSensors(env);
						}
					}
				}
				//std::cout << "Done with sensors" << std::endl;
				// 3. Join threads. Individuals are now evaluated.
				actions.join_all();
				//std::cout << "Done. Iteration: " << iterations << std::endl;
				t += step;
				iterations++;

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

				if (!scenario->init(odeWorld, odeSpace, robots)) {
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
