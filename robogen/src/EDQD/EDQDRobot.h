/*
 * @(#) EDQDController.h   1.0   Sep 01, 2021
 *
 * Sindiso Mkhatshwa (mkhsin035@myuct.ac.za)
 *
 * Nitschke Laboratory, UCT
 *
 * @(#) $Id$
 */

#ifndef EDQDROBOT_H_
#define EDQDROBOT_H_

#include <EDQD/contrib/neuralnetworks/NeuralNetwork.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/lock_guard.hpp>
#include "evolution/representation/RobotRepresentation.h"
#include "model/components/perceptive/ColorSensorModel.h"
#include "model/sensors/SensorMorphology.h"
#include "config/EvolverConfiguration.h"
#include "evolution/engine/Mutator.h"
#include "Heuristic.h"
#include "Simulator.h"
#include <map>
#include "Robot.h"
#include "EDQDMap.h"
#include "Util.h"

//#include "PickUpHeuristic.h"
//#include "CollisionAvoidanceHeuristic.h"
namespace robogen {
class EDQDMap;
//================================================================================================
/**
 * \brief Class for describing an EDQD-robot
 * Extends Robot
 */
//================================================================================================

class EDQDRobot : public Robot{
	protected:
		//========================================================================================
		// Methods
		//========================================================================================
		/**
		 * \brief Load a new genome - i.e. replace currently active genome with a new one
		 */
		void loadNewGenome();
		/**
		 * \brief Checks if a new genome was loaded
		 * @return bool flagging new genome status
		 */
		bool getNewGenomeStatus() { return isNewGenome_; }
		/**
		 * \brief Updates new genome flag
		 * @param bool new status
		 */
		void setNewGenomeStatus( bool status ) { isNewGenome_ = status; }
		/**
		 * \brief Clears genomesList, sigmaList, fitnessesList and birthdayList
		 */
		void clearReservoir();
		/**
		 * \brief Mutate genome using Gaussian distribution\n
		 * Mutates within bounds.
		 */
		void mutateGaussian( float sigma );
		/**
		 *  \brief Mutate genome using Uniform distribution\n
		 *  Mutates within bounds.
		 */
		void mutateUniform();
		/**
		 * \brief Mutate sigma value
		 */
		void mutateSigmaValue();
		//*****************************************************************************************
		// * Morphology adaptation
		//****************************************************************************************/
		/**
		 * \brief Mutate robot sensors
		 */
		void mutateSensors();
		/**
		 * \brief Map genotype to phenotype
		 */
		void mapGenotypeToPhenotype();
		/**
		 * \brief Perform selection\n
		 * Called only if at least 1 genome was stored.
		 */
		void performSelection();
		/**
		 * \brief Perform variation
		 */
		void performVariation();
		/**
		 * \brief Select random genome form behavior-related merged map
		 */
		void selectRandomGenomeFromMergedMap();
		/**
		 * \brief Select random genome from morphology-related merged map
		 */
		void selectRandomGenomeFromMorphMergedMap();
		/**
		 * \brief Select random genome from received genome list
		 */
		void selectRandomGenome();
		/**
		 * \brief Select best genome from genome list
		 */
		void selectBestGenome();
		/**
		 * \brief Select first genome from genome list
		 */
		void selectFirstGenome();
		/**
		 * \brief Select fitness proportionate
		 */
		void selectFitProp();
		/**
		 * \brief Log current agent state
		 */
		void logCurrentState();
		/**
		 * \brief Reset fitness\n
		 * Note: resetFitness is first called by the Controller's constructor.
		 */
		void resetFitness();
		/**
		 * \brief Compute required number of weights for given ANN
		 */
	    unsigned int computeRequiredNumberOfWeights();

		//========================================================================================
		// Members
		//========================================================================================
	    /**
	     * \brief Status check of this robot - i.e. has it been initialized?
	     */
	    bool isNull;
	    /**
	     * \brief Normal distribution
	     */
		boost::random::normal_distribution<float> normalDistribution;
		/**
		 * \brief Uniform distribution
		 */
		boost::random::uniform_01<float> uniformDistribution;
		/**
		 * \brief Robogen config
		 */
		boost::shared_ptr<RobogenConfig> configuration;
		/**
		 * \brief Scenario
		 */
		boost::shared_ptr<Scenario> scenario_;
		/**
		 * \brief Evaluation when this controller was initialised
		 */
		int birthdate_;
		/** \brief Flags whether this agent can receive broadcasted genomes/maps from nearby agents */
		bool isListening_;
		/** \brief Amount of time to be spent in a not-listening-state*/
		int notListeningDelay_;
		/** \brief Duration of listening state */
		int listeningDelay_;
		/** \brief Number of genomes transmitted */
		int nbGenomeTransmission_;
		/** \brief Flags whether a new genome was recently loaded*/
		bool isNewGenome_;
		/** \brief Behavioral competence */
		double fitness_;
		/** \brief Step size */
		double step_;
		/** \brief The number of iterations that have passed while this agent is attached to a specific resource */
		int timeResourceBound_;
		/** \brief Behavior map */
		EDQDMap* map_;
		/** \brief Merged map */
		EDQDMap* mergedMap_;
		/** \brief Morphology map */
		EDQDMap* morphMap_;
		/** \brief Morphology merged map */
		EDQDMap* morphMergedMap_;
		/** \brief Incoming genomes reservoir */
		std::map< std::pair<int,int>, std::vector<double> > genomesList_;
		/** \brief List of sigma values */
		std::map< std::pair<int,int>, float > sigmaList_;
		/** \brief List of fitness values */
		std::map< std::pair<int,int>, float > fitnessValuesList_;
		/** \brief Behavior map list */
		std::map< int, EDQDMap* > mapList_;
		/** \brief Morphology map list */
		std::map< int, EDQDMap* > morphMapList_;

		/**
		 * \brief Current genome
		 */
		static const int maxGenomeSize = (MAX_INPUT_NEURONS + MAX_OUTPUT_NEURONS + MAX_HIDDEN_NEURONS)
							             * (MAX_OUTPUT_NEURONS + MAX_HIDDEN_NEURONS);
		/**
		 * \brief Vector of current genome
		 */
		std::vector<double> currentGenome_;
		/**
		 * \brief Current sigma
		 */
		float currentSigma_;
		/**
		 * \brief This robot's ancestor
		 */
		std::pair<int,int> ancestor_;

		/**
		 * \brief Morphology birthdate
		 */
		int morphBirthdate_;
		/**
		 * \brief This morphology's ancestor
		 */
		std::pair<int,int> morphAncestor_;

		/**
		 * \brief ANN minimum value
		 */
		double minValue_;
		/**
		 * \brief ANN max value
		 */
		double maxValue_;
		/**
		 * \brief Number of input neurons
		 */
		unsigned int nbInputs_;
		/**
		 * \brief Number of output neurons
		 */
		unsigned int nbOutputs_;
		/**
		 * \brief Number of hidden layers
		 */
		unsigned int nbHiddenLayers_;
		/**
		 * \brief Number of neurons per hidden layer
		 */
		std::vector<unsigned int>* nbNeuronsPerHiddenLayer_;
		/**
		 * \brief ANN parameters
		 */
		std::vector<double> parameters_;
		/**
		 * \brief Neural Network type
		 */
		std::string nnType_;
		/**
		 *
		 */
		std::vector<int> nbHiddenNeuronsPerLayer_;
		/**
		 * \brief Number of bias neurons per layer
		 */
		std::vector<int> nbBiasNeuronsPerLayer_;
		/**
		 * \brief Neural Network
		 */
		Neural::NeuralNetwork* nn;
		/**
		 * \brief Creates ANN
		 */
		void createNN();
		/**
		 * \brief Gets inputs
		 */
		std::vector<double> getInputs();
		/**
		 * \brief Gets sensors inputs
		 */
		std::vector<double> getSensorInputs();
		/**
		 * \brief Sets IO size
		 */
		void setIOcontrollerSize();

		/**
		 * \brief Initializes controller
		 *
		 * Called by resetRobot
		 */
		void initController();

		/**
		 * \brief Map morphology representation to phenotype
		 * c
		 *
		 * Called by resetRobot
		 */
		void mapMorphPhenotype();
		/**
		 * \brief Step controller
		 */
		void stepController();


		/**
		 * \brief Log robot initial x position
		 */
		double Xinit_;
		/**
		 * \brief Log robot initial y position
		 */
		double Yinit_;
		/**
		 * \brief Log robot last x position
		 */
		double Xlast_;
		/**
		 * \brief Log robot last y position
		 */
		double Ylast_;
		/**
		 * \brief Log sum of distance traveled
		 */
		double dSumTravelled_;
		/**
		 * \brief Log maximum distance traveled
		 */
		double dMaxTravelled_;
		/**
		 * \brief Log potential distance that could have been traveled by robot
		 */
		double dPotMaxTravelled_;
		/**
		 * \brief Number of genomes/maps transmitted
		 */
		int nbComTx_;
		/**
		 * \brief Number of genomes/maps received
		 */
		int nbComRx_;
		/**
		 * \brief Current cell id
		 */
		int currentCellId_;

		/**
		 * \brief Map resource size (in terms of the number of robots required to push it)
		 * to number collected
		 */
		std::map<int,int> resourceCounters_;

		/**************************************************************************************************
		 * Morphology properties
		 ************************************************************************************************/
		/**
		 * \brief Flags whether sensor type is active or not
		 */
		std::map<int,bool> isSensorTypeActive_;
		/**
		 * \brief Maps number of sensors active per sensor type
		 */
		std::map<int,int> numOfActiveSensorsPerType_;
		/**
		 * \brief Keeps total number of sensors (inactive + active)
		 */
		int possibleTotalNumberOfSensors_;
		/**
		 * \brief Maps max range possible per sensors type
		 */
		std::map<int,double> perSensorTypeMaxRange_;
		/**
		 * \brief Maps current range of each sensor type
		 */
		std::map<int,double> perSensorTypeRange_;
		/**
		 * \brief Average range of all active sensors
		 */
		double averageActiveSensorRange_;
		/**
		 * \brief Max active sensor range average
		 */
		double maxActiveSensorRangeAverage_;
		/**
		 * \brief Sensor minimum value
		 */
		double sensorMinValue_;
		/**
		 * Sensor maximum value
		 */
		double sensorMaxValue_;

		/**
		 * \brief Target area location
		 */
		osg::Vec2d targetArea_;
		/**
		 * \brief Position at which resource was picked up
		 */
		osg::Vec3d pickUpPosition;

		/**
		 * \brief Resource logger
		 */
		Logger *resourceLogger;
		/**
		 * \brief Resource log ofstream
		 */
		std::ofstream resourceLogFile;
		/**
		 * \brief Name of resource log file
		 */
		std::string resourceLogFilename;
		/**
		 * \brief Sensor logger
		 */
		Logger *sensorLogger;
		/**
		 * \brief Sensor log ofstream
		 */
		std::ofstream sensorLogFile;
		/**
		 * Sensor log file name
		 */
		std::string sensorLogFilename;

	public:
		/**
		 * \brief Motor outputs
		 */
		osg::Vec2d motorOutputs;
		//========================================================================================
		// Methods
		//========================================================================================

		/**
		 * \brief Default constructor
		 */
		EDQDRobot();

		/**
		 * \brief Destructor
		 */
	    virtual ~EDQDRobot();

	    /**
	     * \brief Initializes a new EDQDRobot
	     */
	    bool initialise(
							dWorldID odeWorld,
							dSpaceID odeSpace, dSpaceID robotSpace, int& nbALive,
							boost::shared_ptr<RobogenConfig> configuration,
							const robogenMessage::Robot& robotSpec, boost::shared_ptr<Scenario> scenario
						);

	    /**
		 * \brief Manage storage of a map received from a neighbour
		 *
		 * Note that in case of multiple encounters with the same robot (same id, same "birthdate"),
		 * genome is stored only once, and last known fitness value is stored
		 * (i.e. updated at each encounter).
		 * Fitness is optional (default: 0)
		 */
	    bool storeMap(EDQDMap* map, int senderId);

	    /**
		 * \brief Manage storage of behavior and morphology maps received from a neighbour
		 *
		 * Note that in case of multiple encounters with the same robot (same id, same "birthdate"),
		 * genome is stored only once, and last known fitness value is stored
		 * (i.e. updated at each encounter).
		 * Fitness is optional (default: 0)
		 */
	    bool storeMaps(EDQDMap* map, EDQDMap* morphMap, int senderId);

	    /* \brief Manage storage of a genome received from a neighbour
		 *
		 * Note that in case of multiple encounters with the same robot (same id, same "birthdate"), genome is stored only once, and last known fitness value is stored (i.e. updated at each encounter).
		 */
	    bool storeGenome(std::vector<double> genome, std::pair<int,int> senderId, float sigma, float fitness, EDQDMap* map);
	    /**
	     * \brief Broadcast map
	     */
	    void broadcastMap();
	    /**
	     * \brief Update fitness
	     */
	    void updateFitness(int pushingRobots, double value);
	    /**
	     * \brief Update fitness
	     */
	    void updateFitness(osg::Vec3d dropOffPosition, int pushingRobots, double value);

	    /**
		 * \brief Perform control decision and evolution
		 */
	    void step(boost::mutex& queueMutex);
	    /**
	     * \brief STep evolution
	     */
	    void stepEvolution(boost::mutex& queueMutex);
	    /**
	     * \brief Step controller
	     */
	    void stepController(
	    					boost::shared_ptr<Environment> env,
							double elapsedEvaluationTime
							);
	    /**
	     * \brief Set ANN weights
	     */
	    void setWeights();
	    /**
	     * \brief Sets pick-up position
	     */
	    void setPickUpPosition(osg::Vec3d position){pickUpPosition = position;}
	    /**
	     * \brief Returns listening status
	     */
	    bool isListening() { return isListening_; }
	    /**
	     * \brief Increments number of genomes/maps transmitted
	     */
	    void incNbComTx(){nbComTx_++;}
	    /**
	     * \brief Increments resource counters
	     */
	    void incResourceCounter(const int group) {
	    	resourceCounters_[group] +=1;
		}
	    /**
	     * \brief Decrements resource counters
	     */
	    void decResourceCounter(const int group) {resourceCounters_[group]--;}
	    /**
	     * \brief Resets maximum possible distance that could have been traveled
	     */
	    void resetPotMaxTravelled();
	    /**
	     * \brief Updates cell id
	     */
	    int updateCellId();
	    /**
	     * \brief Returns birthdate
	     */
	    int getBirthdate() { return birthdate_; }

	    /**
	     * \brief Returns fitness of the robot
		 * @return double
		 */
	    double getFitness();
	    /**
	     * \brief Returns map container of resource counters
	     */
	    const std::map<int,int>& getResourceCounters() const {
			return resourceCounters_;
		}
	    /********************************************************************************
	     * MORPHO-EVO
	     *******************************************************************************/
	    /**
	     * \brief Updates sensor info
	     */
	    void updateSensorInfo();
	    /**
	     * \brief Returns active sensors
	     */
	    const std::map<int,int>& getActiveSensors() const {
	    	return numOfActiveSensorsPerType_;
		}
	    /**
	     * \brief Sets active sensors per type
	     */
	    void setActiveSensors(std::map<int,int> active_sensors){
	    	for (int i = SensorElement::RESOURCET1; i <= SensorElement::RESOURCET5 ; i++){
	    		numOfActiveSensorsPerType_[i] = active_sensors[i];
			}
		}
	    /**
	     * \brief Returns sensor status
	     */
	    void isSensorTypeActive(std::map<int,bool> isSensorTypeActive){
			for (int i = SensorElement::RESOURCET1; i <= SensorElement::RESOURCET5 ; i++){
				isSensorTypeActive_[i] = isSensorTypeActive[i];
			}
		}
	    /**
	     * \brief Returns average range of active sensors
	     */
	    double getAverageActiveSensorRange();
	    /**
	     * \brief Returns maximum range possible of active sensors
	     */
	    double getMaxActiveSensorRangeAverage();
	    /**
	     * \brief Sets per sensor type range
	     */
	    void setPerSensorTypeRange(std::map<int,double> perSensorTypeRange){
			for (int i = SensorElement::RESOURCET1; i <= SensorElement::RESOURCET5 ; i++){
				perSensorTypeRange_[i] = perSensorTypeRange[i];
			}
		}
	    /**
	     * \brief Sets per sensor type max range
	     */
	    void setPerSensorTypeMaxRange(std::map<int,double> perSensorTypeMaxRange){
	    	for (int i = SensorElement::RESOURCET1; i <= SensorElement::RESOURCET5 ; i++){
	    		perSensorTypeMaxRange_[i] = perSensorTypeMaxRange[i];
			}
	    }
	    /**
	     * \brief Gets total possible number of sensors
	     */
	    int getPossibleNumberOfSensors(){return possibleTotalNumberOfSensors_;}
	    /**
	     * \brief Sets the total possible number of sensors
	     */
	    void setPossibleNumberOfSensors(int possible_number_of_sensors){possibleTotalNumberOfSensors_ = possible_number_of_sensors;}
	    /**
	     * \brief Resets sensor info
	     */
	    void resetSensorInfo();
	    /**
	     * \brief Resets resource counters
	     */
		void resetResourceCounter() {
			/*
			 * 5 types of resources in the environment right now (sizes 1, 2, 3, 4, 5)
			 */
			for (int i = 1; i <= 5 ; i++){
				resourceCounters_[i] = 0;
			}
		}
		/**
		 * \brief Gets maximum distance traveled
		 */
		double getMaxTravelled() const {
			return dMaxTravelled_;
		}
		/**
		 * \brief Returns total distance that could have been traveled
		 */
		double getPotMaxTravelled() const {
			return dPotMaxTravelled_;
		}
		/**
		 * \brief Returns sum traveled
		 */
		double getSumTravelled() const {
			return dSumTravelled_;
		}
		/**
		 * Returns behavior map
		 */
		EDQDMap*& getMap() {
			return (map_);
		}
		/**
		 * \brief Returns morphology map
		 */
		EDQDMap*& getMorphMap() {
			return (morphMap_);
		}
		/**
		 * Returns current genome
		 */
		const std::vector<double>& getCurrentGenome() const {
			return currentGenome_;
		}
		/**
		 * \brief Returns current sigma
		 */
		float getCurrentSigma() const {
			return currentSigma_;
		}
		/**
		 * \brief Returns time robot was bound to resource
		 */
		int getTimeResourceBound(){return timeResourceBound_;}
		/**
		 * \brief Increment time robot is bound to resource
		 */
		void incTimeResourceBound(){timeResourceBound_++;}
		/**
		 * \brief Reset time robot was bound to resource
		 */
		void resetTimeResourceBound(){timeResourceBound_ = 0;}
		/**
		 * \brief Reset this robot
		 */
		void reset();
		/**
		 * \brief Log end of generation info
		 */
		void writeEOGEntry(Logger* lm);
};

}//namespace robogen
#endif /* EDQDROBOT_H_ */
