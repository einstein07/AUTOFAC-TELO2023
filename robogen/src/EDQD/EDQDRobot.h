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
// EDQD Robot Class
//================================================================================================

class EDQDRobot : public Robot{
	protected:
		//========================================================================================
		// Methods
		//========================================================================================

		void loadNewGenome();
		bool getNewGenomeStatus() { return isNewGenome_; }
		void setNewGenomeStatus( bool status ) { isNewGenome_ = status; }
		bool storeGenome(std::vector<double> genome, std::pair<int,int> senderId, float sigma, float fitness, EDQDMap* map);
		bool storeMap(EDQDMap* map, int senderId);
		/**
		 * clear genomesList, sigmaList, fitnessesList and birthdayList
		 */
		void clearReservoir();

		void mutateGaussian( float sigma );
		void mutateUniform();
		void mutateSigmaValue();

		void mapGenotypeToPhenotype();
		void performSelection();
		void performVariation();
		void selectRandomGenomeFromMergedMap();
		void logCurrentState();
		void resetFitness();

	    unsigned int computeRequiredNumberOfWeights();

		//========================================================================================
		// Members
		//========================================================================================
	    //TODO: Find clever way for this
	    bool isNull;
		boost::random::normal_distribution<float> normalDistribution;
		boost::random::uniform_01<float> uniformDistribution;
		boost::shared_ptr<RobogenConfig> configuration;
		boost::shared_ptr<Scenario> scenario_;
		boost::shared_ptr<Mutator> mutator_;
		int iteration_;
		/**
		 * Evaluation when this controller was initialised
		 */
		int birthdate_;
		bool isListening_;
		int notListeningDelay_;
		int listeningDelay_;
		int nbGenomeTransmission_;
		bool isNewGenome_;
		double fitness_;
		double step_;
		int count_;
		int timeResourceBound_;
		// map
		EDQDMap* map_;
		EDQDMap* mergedMap_;

		// incoming genomes reservoir
		std::map< std::pair<int,int>, std::vector<double> > genomesList_;
		std::map< std::pair<int,int>, float > sigmaList_;
		std::map< std::pair<int,int>, float > fitnessValuesList_;
		std::map< int, EDQDMap* > mapList_;

		/**
		 * Current genome
		 */
		static const int maxGenomeSize = (MAX_INPUT_NEURONS + MAX_OUTPUT_NEURONS + MAX_HIDDEN_NEURONS)
							             * (MAX_OUTPUT_NEURONS + MAX_HIDDEN_NEURONS);

		std::vector<double> currentGenome_;
		float currentSigma_;

		std::pair<int,int> ancestor_;

		/**
		 * ANN
		 */
		/**************************************************/
		// ANN
		double minValue_;
		double maxValue_;
		unsigned int nbInputs_;
		unsigned int nbOutputs_;
		unsigned int nbHiddenLayers_;




		std::vector<unsigned int>* nbNeuronsPerHiddenLayer_;

		std::vector<double> parameters_;
		std::string nnType_;
		std::vector<int> nbHiddenNeuronsPerLayer_;
		std::vector<int> nbBiasNeuronsPerLayer_;
		Neural::NeuralNetwork* nn;

		void createNN();

		std::vector<double> getInputs();
		std::vector<double> getSensorInputs();
		void setIOcontrollerSize();

		void initController(); // called by resetRobot
		void stepController();
		/***************************************************/
		//double minValue_;
		//double maxValue_;
		//float weights_[maxGenomeSize];
		//unsigned int nbInputs_;
		//unsigned int nbOutputs_;
		//unsigned int nbHidden_;

		//float params_[MAX_PARAMS * (MAX_OUTPUT_NEURONS + MAX_HIDDEN_NEURONS)];
		//unsigned int types_[(MAX_OUTPUT_NEURONS + MAX_HIDDEN_NEURONS)];



		//std::vector<unsigned int>* nbNeuronsPerHiddenLayer_;

		// logging purpose
		double Xinit_;
		double Yinit_;
		double Xlast_;
		double Ylast_;
		double dSumTravelled_;
		double dMaxTravelled_;
		double dPotMaxTravelled_;
		int nbComTx_;
		int nbComRx_;

		int currentCellId_;

		/**
		 * Map resource size (in terms of the number of robots required to push it)
		 * to number collected
		 */
		std::map<int,int> resourceCounters_;

		/**
		 * Behavioral heuristics
		 */
		osg::Vec2d targetArea_;

		/**
		 *
		 * LOGGING Purposes
		 */
		Logger *logger;
		std::ofstream robotLogFile;
		std::string logFilename;

	public:
		double motorOutputs[2];
		//========================================================================================
		// Methods
		//========================================================================================
		EDQDRobot();

	    virtual ~EDQDRobot();

	    bool initialise(
							dWorldID odeWorld,
							dSpaceID odeSpace, dSpaceID robotSpace, int& nbALive,
							boost::shared_ptr<RobogenConfig> configuration,
							const robogenMessage::Robot& robotSpec, boost::shared_ptr<Scenario> scenario
						);


	    void broadcastMap();
	    void updateFitness();
	    void step(
	    			/**boost::shared_ptr<Environment> env,*/
					/**boost::shared_ptr<RobogenConfig> configuration,*/
					/**boost::random::mt19937 &rng,*/
					/**int count,*/
					/**double step,*/
					double elapsedEvaluationTime
	    		);
	    void stepEvolution();
	    void stepController(
	    					boost::shared_ptr<Environment> env,
							/**boost::shared_ptr<RobogenConfig> configuration,*/
							/**boost::random::mt19937 &rng,*/
							/**int count,*/
							/**double step,*/
							double elapsedEvaluationTime
							);
	    void setWeights();


	    bool isListening() { return isListening_; }

	    void incResourceCounter(const int group) {
	    	/*if (resourceCounters_[group] == -1 ){
	    		resourceCounters_[group] = 1;
	    	}else{*/
			resourceCounters_[group]++;
	    	//}
		}

	    void resetPotMaxTravelled();

	    int updateCellId();

	    int getBirthdate() { return birthdate_; }

	    double getFitness();

	    const std::map<int,int>& getResourceCounters() const {
			return resourceCounters_;
		}

		void resetResourceCounter() {
			/**
			 * Only 3 types of resources in the environment right now (sizes 1, 2 & 3)
			 * Simple environment - i.e. 1 & 2 pushing robots per resource only
			 */
			for (int i = 1; i <= 5 ; i++){
				resourceCounters_[i] = 0; /**randint()%100;//*///0;
			}
		}

		double getMaxTravelled() const {
			return dMaxTravelled_;
		}

		double getPotMaxTravelled() const {
			return dPotMaxTravelled_;
		}

		double getSumTravelled() const {
			return dSumTravelled_;
		}

		EDQDMap*& getMap() {
			return (map_);
		}

		const std::vector<double>& getCurrentGenome() const {
			return currentGenome_;
		}

		float getCurrentSigma() const {
			return currentSigma_;
		}

		int getTimeResourceBound(){return timeResourceBound_;}
		void incTimeResourceBound(){timeResourceBound_++;}
		void resetTimeResourceBound(){timeResourceBound_ = 0;}

		void reset();

		void writeEOGEntry(Logger* lm);
};

}//namespace robogen
#endif /* EDQDROBOT_H_ */
