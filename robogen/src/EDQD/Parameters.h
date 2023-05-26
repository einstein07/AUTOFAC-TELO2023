/*
 * @(#) Parameters.h   1.0   Sep 05, 2021
 *
 * Sindiso Mkhatshwa (mkhsin035@myuct.ac.za)
 *
 * Nitschke Laboratory, UCT
 *
 * @(#) $Id$
 */

#ifndef EDQD_PARAMETERS_H_
#define EDQD_PARAMETERS_H_
#include "Logger.h"
#include <fstream>

extern std::ofstream gLitelogFile;
extern Logger *gLiteLogger;
namespace EDQD{
	//=============================================================================================
	// Embodied Evolution Parameters
	//=============================================================================================
	/**
	 * \brief Class for describing experiment parameters
	 */
	class Parameters{
	public:
		//=========================================================================================
		// Members
		//=========================================================================================

		/**
		 * \brief Used with gDynamicSigma defined to true
		 */
		static double sigmaMin;

		/**
		 * \brief Maximal value of sigma
		 */
		static double sigmaMax;

		/**
		 * \brief Number of hidden layers
		 */
		static int nbHiddenLayers; // default: 1
		/**
		 * \brief Number of neurons per hidden layer
		 */
		static int nbNeuronsPerHiddenLayer; // default: 5
		/**
		 * \brief Bounds of brain weights
		 *
		 * Default this to 800 (i.e. weights are in [-400,+400] as done in previous work)
		 */
		static int weightRange;
		/**
		 * \brief Number of resource types in the enviornment
		 */
		static int nbOfPhysicalObjectGroups;


		/**
		 * \brief Step used in the decrease or increase of the value of sigma
		 */
		static double updateSigmaStep;

		/**
		 * \brief Step used in the decrease or increase of the value of sigma
		 */
		static double sigmaRef;

		/**
		 * \brief Probability of transmitting the current genome mutated with sigma ref
		 */
		static double pMutation;

		/**
		 * \brief Probability of mutating sensor state
		 */
		static double pMutateSensorState;

		/**
		 * \brief Switch sensor on/off
		 */
		static double sensorState;

		/**
		 * \brief Theoretical duration of a generation (ie. maximum time a controller
		 * will be evaluated on a robot)
		 */
		static unsigned int evaluationTime;

		/**
		 * \brief Maximum number of iterations
		 */
		static unsigned int maxIterations;

		/**
		 * \brief Sync flag
		 *
		 * If set to false, a robot will restart its controller as soon as it has no more energy.
		 * If set to true, the robot without energy will wait and reload its controller at the
		 * same time as every other robots.
		 */
		static bool synchronization;

		/**
		 * \brief Defines the number of dimensions of a map
		 *
		 * NB: needs to be set at compile time
		 */
		static const int nbOfDimensions = 2;

		/**
		 * \brief Defines the number of intervals per dimension of a map
		 *
		 * NB: needs to be set at compile time
		 */
		static const int nbOfIntervals = 10;//2;///*15*/5;

		/**
		 * \brief Used to decide whether to discard or keep an elite in the map
		 */
		static double fitEpsilon;

		/**
		 * \brief Controller type (0: MLP, 1: Perceptron, 2: Elman)
		 */
		static int controllerType;

		/**
		 * \brief Max. nb of genome transmission per individual per lifetime.
		 */
		static int maxNbGenomeTransmission;

		/**
		 * \brief Enable/disable limiting transmission
		 */
		static bool limitGenomeTransmission;

		/**
		 * \brief Not listening state delay
		 *
		 * -1: infinite ; 0: no delay ; >0: delay
		 */
		static int notListeningStateDelay;

		/**
		 * \brief Listening state delay
		 *
		 * -1: infinite ; 0: no delay ; >0: delay (ignored if gNotListeningStateDelay=-1)
		 */
		static int listeningStateDelay;
		/**
		 * \brief Individual mutation rate
		 */
		static double individualMutationRate;

		/**
		 *\brief Mutation operator
		 *
		 * 0: uniform, 1: gaussian with evolved sigma, 2: gaussian with fixed sigma
		 */
		static int mutationOperator;

		/**
		 * \brief Value of sigma
		 */
		static double sigma;

		/**
		 * \brief Keep maps for only one generation
		 */
		static bool onlyKeepMapsForGeneration;
		/**
		 * \brief Flags map selection usage
		 *
		 * If true EDQD is used instead of mEDEA
		 */
		static bool EDQDMapSelection;
		/**
		 * \brief Flags maintenance of two behavioral maps
		 *
		 * If true behavior and morphology diversity maps are maintained
		 */
		static bool EDQDMultiBCMap;
		/**
		 * \brief Selection method for medea
		 */
		static int selectionMethod;
		/**
		 * \brief Flags evolution of sensors
		 */
		static bool evolveSensors;
		/**
		 * \brief Maximum time a robot can stay attached to a resource before eventually
		 * getting forced to drop it off for other robots to pick it up.
		 */
		static int maxTimeResourceBound;

		/**
		 * \brief Base directory for logs
		 */
		static std::string logDirectoryname;

		/**
		 * \brief End of generation log file
		 */
		static std::ofstream gEOGLogFile;
		/**
		 * \brief End of generation logger
		 */
		static Logger* gEOGLogger;

		/**
		 * \brief Behavior map log file
		 */
		static std::ofstream gMapsLogFile;
		/**
		 * \brief Behavior map logger
		 */
		static Logger* gMapsLogger;
		/**
		 * \brief Morphology map log file
		 */
		static std::ofstream gMorphMapsLogFile;
		/**
		 * \brief Morphology map logger
		 */
		static Logger* gMorphMapsLogger;

		/**
		 * \brief Resources log file
		 */
		static std::ofstream gResourcesLogFile;
		/**
		 * \brief Resources logger
		 */
		static Logger* gResourcesLogger;
		/**
		 * \brief Morphology log file ofstream
		 */
		static std::ofstream gMorphLogFile;
		/**
		 * \brief Morphology logger
		 */
		static Logger* gMorphLogger;

		// =========================================================================================
		// Constructors
		// =========================================================================================
		/** \brief Default constructor */
		Parameters();

		// =========================================================================================
		// Destructor
		// =========================================================================================
		/**
		 * \brief Destructor
		 */
		~Parameters();

		//==========================================================================================
		// Methods
		//==========================================================================================

		/** \brief Load the parameters from a file

		 	returns 0 on success
		 */
		int Load(const char* filename);
		/**
		 * \brief Load the parameters from an already opened file for reading
		 */
		int Load(std::ifstream& a_DataFile);
		/**
		 * \brief Save to a file
		 */

		void Save(const char* filename);
		/**
		 * \brief Saves the parameters to an already opened file for writing
		 */
		void Save(FILE* a_fstream);
		/**
		 * \brief Resets the parameters to built-in defaults
		 */
		void Reset();
	};
}



#endif /* EDQD_PARAMETERS_H_ */
